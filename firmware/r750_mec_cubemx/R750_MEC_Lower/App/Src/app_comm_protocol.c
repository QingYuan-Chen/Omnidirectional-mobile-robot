#include "app_comm_protocol.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * 逐字节解析器把不可信 UART4 文本转换为有界、可验序的内部命令。
 * 所有解析都在固定 64 字节行缓冲和最多四个 token 内完成；不调用 strtol，也不依赖
 * 零终止输入，从而能逐字节检查字符范围与整数溢出。协议层与执行状态机各自再次验序，
 * 前者保护通信队列，后者保护执行器，形成两道防重放边界。
 */

_Static_assert(ROBOT_CONFIG_UART_LINE_BUFFER_SIZE >= 16U,
               "UART command line buffer is too small");

typedef struct {
  const uint8_t *data;
  uint16_t length;
} AppCommToken;

static uint32_t AppCommProtocol_AddSaturated(uint32_t value, uint32_t increment)
{
  /* 诊断计数只允许单调增加；到达上限后保持 UINT32_MAX，避免长期运行回绕。 */
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static bool AppCommProtocol_TokenEquals(
  const AppCommToken *token,
  const char *literal)
{
  const size_t literal_length = strlen(literal);
  return token != NULL && literal_length == token->length &&
         memcmp(token->data, literal, literal_length) == 0;
}

static uint16_t AppCommProtocol_Tokenize(
  const uint8_t *line,
  uint16_t length,
  AppCommToken tokens[4])
{
  /*
   * 前置字符检查已拒绝制表符和其他控制字符，这里只把普通空格视为分隔符。连续空格
   * 被折叠，但命令名和数字 token 必须完整匹配；超过三个有效字段时返回哨兵计数 4，
   * 由上层统一按语法错误拒绝。
   */
  uint16_t position = 0U;
  uint16_t count = 0U;
  while (position < length) {
    while (position < length && line[position] == (uint8_t)' ') {
      position++;
    }
    if (position >= length) {
      break;
    }
    if (count >= 4U) {
      return 4U;
    }

    const uint16_t start = position;
    while (position < length && line[position] != (uint8_t)' ') {
      position++;
    }
    tokens[count].data = &line[start];
    tokens[count].length = (uint16_t)(position - start);
    count++;
  }
  return count;
}

static bool AppCommProtocol_ParseU32(const AppCommToken *token, uint32_t *value)
{
  /* 逐位做乘十前边界检查，禁止符号、空串及超过 uint32_t 的十进制文本。 */
  if (token == NULL || value == NULL || token->length == 0U) {
    return false;
  }

  uint32_t parsed = 0U;
  for (uint16_t i = 0U; i < token->length; ++i) {
    const uint8_t character = token->data[i];
    if (character < (uint8_t)'0' || character > (uint8_t)'9') {
      return false;
    }
    const uint32_t digit = (uint32_t)(character - (uint8_t)'0');
    if (parsed > ((UINT32_MAX - digit) / 10U)) {
      return false;
    }
    parsed = (parsed * 10U) + digit;
  }
  *value = parsed;
  return true;
}

static bool AppCommProtocol_ParseI32(const AppCommToken *token, int32_t *value)
{
  /*
   * 先在无符号幅值域解析，以便合法表示 INT32_MIN 的绝对值 2147483648；正数上限仍限制
   * 为 INT32_MAX。协议不接受显式加号，减少同一个数值的多种文本表示。
   */
  if (token == NULL || value == NULL || token->length == 0U) {
    return false;
  }

  uint16_t position = 0U;
  bool negative = false;
  if (token->data[position] == (uint8_t)'-') {
    negative = true;
    position++;
    if (position >= token->length) {
      return false;
    }
  }

  const uint32_t limit = negative ? ((uint32_t)INT32_MAX + 1U) : (uint32_t)INT32_MAX;
  uint32_t magnitude = 0U;
  for (; position < token->length; ++position) {
    const uint8_t character = token->data[position];
    if (character < (uint8_t)'0' || character > (uint8_t)'9') {
      return false;
    }
    const uint32_t digit = (uint32_t)(character - (uint8_t)'0');
    if (magnitude > ((limit - digit) / 10U)) {
      return false;
    }
    magnitude = (magnitude * 10U) + digit;
  }

  if (negative) {
    *value = magnitude == ((uint32_t)INT32_MAX + 1U)
               ? INT32_MIN
               : -(int32_t)magnitude;
  } else {
    *value = (int32_t)magnitude;
  }
  return true;
}

static bool AppCommProtocol_SequenceIsNewer(uint32_t sequence, uint32_t previous)
{
  /*
   * 在模 2^32 序号空间中，前向距离 1～INT32_MAX 视为更新；0 是重复，另一半空间视为旧值。
   * 该规则允许自然回绕，同时要求发送端不能一次跳过超过半个序号空间。
   */
  const uint32_t difference = sequence - previous;
  return difference != 0U && difference <= (uint32_t)INT32_MAX;
}

static AppCommFeedResult AppCommProtocol_RejectSyntax(AppCommProtocol *protocol)
{
  protocol->stats.syntax_error_count = AppCommProtocol_AddSaturated(
    protocol->stats.syntax_error_count, 1U);
  return APP_COMM_FEED_REJECTED;
}

static AppCommFeedResult AppCommProtocol_ParseLine(
  AppCommProtocol *protocol,
  AppCommCommand *command)
{
  /*
   * 一条完整行的处理顺序为：兼容 CRLF、检查 ASCII 可打印范围、分词、匹配命令形状、
   * 解析序号、执行防重放预检、最后检查 PWM 数值范围。任一步失败只增加对应诊断计数，
   * 不部分更新已提交序号。
   */
  uint16_t length = protocol->line_length;
  if (length > 0U && protocol->line[length - 1U] == (uint8_t)'\r') {
    length--;
  }
  for (uint16_t i = 0U; i < length; ++i) {
    if (protocol->line[i] < 0x20U || protocol->line[i] > 0x7EU) {
      return AppCommProtocol_RejectSyntax(protocol);
    }
  }

  AppCommToken tokens[4] = {0};
  const uint16_t token_count = AppCommProtocol_Tokenize(
    protocol->line, length, tokens);
  if (token_count == 0U || token_count >= 4U) {
    return AppCommProtocol_RejectSyntax(protocol);
  }

  command->type = APP_COMM_COMMAND_NONE;
  command->sequence = 0U;
  command->pwm = 0;
  bool needs_sequence = false;
  if (AppCommProtocol_TokenEquals(&tokens[0], "ARM") && token_count == 2U) {
    command->type = APP_COMM_COMMAND_ARM;
    needs_sequence = true;
  } else if (AppCommProtocol_TokenEquals(&tokens[0], "PWM") && token_count == 3U) {
    command->type = APP_COMM_COMMAND_PWM;
    needs_sequence = true;
  } else if (AppCommProtocol_TokenEquals(&tokens[0], "STOP") && token_count == 2U) {
    command->type = APP_COMM_COMMAND_STOP;
    needs_sequence = true;
  } else if (AppCommProtocol_TokenEquals(&tokens[0], "ESTOP") && token_count == 1U) {
    command->type = APP_COMM_COMMAND_ESTOP;
  } else if (AppCommProtocol_TokenEquals(&tokens[0], "STATUS") && token_count == 1U) {
    command->type = APP_COMM_COMMAND_STATUS;
  } else if (AppCommProtocol_TokenEquals(&tokens[0], "CAPTURE") &&
             token_count == 2U) {
    if (AppCommProtocol_TokenEquals(&tokens[1], "START")) {
      command->type = APP_COMM_COMMAND_CAPTURE_START;
    } else if (AppCommProtocol_TokenEquals(&tokens[1], "STOP")) {
      command->type = APP_COMM_COMMAND_CAPTURE_STOP;
    } else if (AppCommProtocol_TokenEquals(&tokens[1], "EXPORT")) {
      command->type = APP_COMM_COMMAND_CAPTURE_EXPORT;
    } else if (AppCommProtocol_TokenEquals(&tokens[1], "STATUS")) {
      command->type = APP_COMM_COMMAND_CAPTURE_STATUS;
    } else {
      return AppCommProtocol_RejectSyntax(protocol);
    }
  } else if (AppCommProtocol_TokenEquals(&tokens[0], "CAPTURE") &&
             token_count == 3U &&
             AppCommProtocol_TokenEquals(&tokens[1], "SPEED")) {
    if (AppCommProtocol_TokenEquals(&tokens[2], "START")) {
      command->type = APP_COMM_COMMAND_SPEED_CAPTURE_START;
    } else if (AppCommProtocol_TokenEquals(&tokens[2], "STOP")) {
      command->type = APP_COMM_COMMAND_SPEED_CAPTURE_STOP;
    } else if (AppCommProtocol_TokenEquals(&tokens[2], "EXPORT")) {
      command->type = APP_COMM_COMMAND_SPEED_CAPTURE_EXPORT;
    } else if (AppCommProtocol_TokenEquals(&tokens[2], "STATUS")) {
      command->type = APP_COMM_COMMAND_SPEED_CAPTURE_STATUS;
    } else {
      return AppCommProtocol_RejectSyntax(protocol);
    }
  } else if (AppCommProtocol_TokenEquals(&tokens[0], "CAPTURE") &&
             token_count == 3U &&
             AppCommProtocol_TokenEquals(&tokens[1], "IMU")) {
    if (AppCommProtocol_TokenEquals(&tokens[2], "START")) {
      command->type = APP_COMM_COMMAND_IMU_CAPTURE_START;
    } else if (AppCommProtocol_TokenEquals(&tokens[2], "STOP")) {
      command->type = APP_COMM_COMMAND_IMU_CAPTURE_STOP;
    } else if (AppCommProtocol_TokenEquals(&tokens[2], "EXPORT")) {
      command->type = APP_COMM_COMMAND_IMU_CAPTURE_EXPORT;
    } else if (AppCommProtocol_TokenEquals(&tokens[2], "STATUS")) {
      command->type = APP_COMM_COMMAND_IMU_CAPTURE_STATUS;
    } else {
      return AppCommProtocol_RejectSyntax(protocol);
    }
  } else {
    return AppCommProtocol_RejectSyntax(protocol);
  }

  if (needs_sequence) {
    if (!AppCommProtocol_ParseU32(&tokens[1], &command->sequence)) {
      protocol->stats.numeric_error_count = AppCommProtocol_AddSaturated(
        protocol->stats.numeric_error_count, 1U);
      return APP_COMM_FEED_REJECTED;
    }
    if (protocol->has_sequence &&
        !AppCommProtocol_SequenceIsNewer(
          command->sequence, protocol->last_sequence)) {
      protocol->stats.old_sequence_count = AppCommProtocol_AddSaturated(
        protocol->stats.old_sequence_count, 1U);
      return APP_COMM_FEED_REJECTED;
    }
  }

  if (command->type == APP_COMM_COMMAND_PWM) {
    int32_t pwm = 0;
    if (!AppCommProtocol_ParseI32(&tokens[2], &pwm)) {
      protocol->stats.numeric_error_count = AppCommProtocol_AddSaturated(
        protocol->stats.numeric_error_count, 1U);
      return APP_COMM_FEED_REJECTED;
    }
    if (pwm < -ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT ||
        pwm > ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT) {
      protocol->stats.range_error_count = AppCommProtocol_AddSaturated(
        protocol->stats.range_error_count, 1U);
      return APP_COMM_FEED_REJECTED;
    }
    command->pwm = (int16_t)pwm;
  }

  protocol->stats.accepted_command_count = AppCommProtocol_AddSaturated(
    protocol->stats.accepted_command_count, 1U);
  return APP_COMM_FEED_COMMAND;
}

void AppCommProtocol_Init(AppCommProtocol *protocol)
{
  if (protocol != NULL) {
    memset(protocol, 0, sizeof(*protocol));
  }
}

AppCommFeedResult AppCommProtocol_FeedByte(
  AppCommProtocol *protocol,
  uint8_t byte,
  AppCommCommand *command)
{
  if (protocol == NULL || command == NULL) {
    return APP_COMM_FEED_REJECTED;
  }
  command->type = APP_COMM_COMMAND_NONE;
  command->sequence = 0U;
  command->pwm = 0;

  if (byte == (uint8_t)'\n') {
    if (protocol->discarding_line) {
      protocol->discarding_line = false;
      protocol->line_length = 0U;
      return APP_COMM_FEED_NONE;
    }
    const AppCommFeedResult result = AppCommProtocol_ParseLine(protocol, command);
    protocol->line_length = 0U;
    return result;
  }
  if (protocol->discarding_line) {
    /*
     * 一旦当前行判坏，后续内容全部丢弃到换行。立即重新开始组帧会把超长命令的尾部
     * 误解释成一条新命令，尤其可能绕过原命令名检查。
     */
    return APP_COMM_FEED_NONE;
  }
  if (byte != (uint8_t)'\r' && (byte < 0x20U || byte > 0x7EU)) {
    protocol->stats.syntax_error_count = AppCommProtocol_AddSaturated(
      protocol->stats.syntax_error_count, 1U);
    protocol->line_length = 0U;
    protocol->discarding_line = true;
    return APP_COMM_FEED_REJECTED;
  }
  if (protocol->line_length >= ROBOT_CONFIG_UART_LINE_BUFFER_SIZE) {
    protocol->stats.line_overflow_count = AppCommProtocol_AddSaturated(
      protocol->stats.line_overflow_count, 1U);
    protocol->line_length = 0U;
    protocol->discarding_line = true;
    return APP_COMM_FEED_REJECTED;
  }
  protocol->line[protocol->line_length] = byte;
  protocol->line_length++;
  return APP_COMM_FEED_NONE;
}

bool AppCommProtocol_CommitSequence(
  AppCommProtocol *protocol,
  const AppCommCommand *command)
{
  if (protocol == NULL || command == NULL ||
      (command->type != APP_COMM_COMMAND_ARM &&
       command->type != APP_COMM_COMMAND_PWM &&
       command->type != APP_COMM_COMMAND_STOP)) {
    return false;
  }
  if (protocol->has_sequence &&
      !AppCommProtocol_SequenceIsNewer(
        command->sequence, protocol->last_sequence)) {
    return false;
  }
  /*
   * 序号提交是解析与排队之间的事务边界：队列拥有命令副本后才推进 last_sequence。
   * 若队列已满，调用者不提交，发送端即可稍后原序号重发而不会被误判为重放。
   */
  protocol->last_sequence = command->sequence;
  protocol->has_sequence = true;
  return true;
}

bool AppCommProtocol_GetStats(
  const AppCommProtocol *protocol,
  AppCommProtocolStats *stats)
{
  if (protocol == NULL || stats == NULL) {
    return false;
  }
  *stats = protocol->stats;
  return true;
}

uint32_t AppCommProtocol_GetRejectedCount(const AppCommProtocolStats *stats)
{
  if (stats == NULL) {
    return 0U;
  }
  uint32_t total = 0U;
  total = AppCommProtocol_AddSaturated(total, stats->syntax_error_count);
  total = AppCommProtocol_AddSaturated(total, stats->numeric_error_count);
  total = AppCommProtocol_AddSaturated(total, stats->range_error_count);
  total = AppCommProtocol_AddSaturated(total, stats->old_sequence_count);
  total = AppCommProtocol_AddSaturated(total, stats->line_overflow_count);
  return total;
}
