#ifndef APP_COMM_PROTOCOL_H
#define APP_COMM_PROTOCOL_H

#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/* 临时试验协议只负责解析与验序，普通运动命令成功入队后才提交序号。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 解析后可交给任务编排层处理的命令类型。 */
typedef enum {
  APP_COMM_COMMAND_NONE = 0,
  APP_COMM_COMMAND_ARM,
  APP_COMM_COMMAND_PWM,
  APP_COMM_COMMAND_STOP,
  APP_COMM_COMMAND_ESTOP,
  APP_COMM_COMMAND_STATUS
} AppCommCommandType;

/* 逐字节输入的结果：等待更多字节、形成命令或拒绝当前行。 */
typedef enum {
  APP_COMM_FEED_NONE = 0,
  APP_COMM_FEED_COMMAND,
  APP_COMM_FEED_REJECTED
} AppCommFeedResult;

/* 规范化后的命令，不携带原始文本指针。 */
typedef struct {
  AppCommCommandType type;
  uint32_t sequence;
  int16_t pwm;
} AppCommCommand;

/* 各类拒绝原因独立计数，便于遥测定位链路问题。 */
typedef struct {
  uint32_t accepted_command_count;
  uint32_t syntax_error_count;
  uint32_t numeric_error_count;
  uint32_t range_error_count;
  uint32_t old_sequence_count;
  uint32_t line_overflow_count;
} AppCommProtocolStats;

/* 解析器保存当前行、已提交序号和饱和统计量。 */
typedef struct {
  uint8_t line[ROBOT_CONFIG_UART_LINE_BUFFER_SIZE];
  uint16_t line_length;
  uint32_t last_sequence;
  AppCommProtocolStats stats;
  bool discarding_line;
  bool has_sequence;
} AppCommProtocol;

/* 清空行状态、序号状态和统计量。 */
void AppCommProtocol_Init(AppCommProtocol *protocol);
/* 每次输入一个字节；换行时严格解析完整命令。 */
AppCommFeedResult AppCommProtocol_FeedByte(
  AppCommProtocol *protocol,
  uint8_t byte,
  AppCommCommand *command);
/* 仅在普通运动命令成功入队后提交序号。 */
bool AppCommProtocol_CommitSequence(
  AppCommProtocol *protocol,
  const AppCommCommand *command);
/* 复制当前协议统计快照。 */
bool AppCommProtocol_GetStats(
  const AppCommProtocol *protocol,
  AppCommProtocolStats *stats);
/* 饱和求和全部拒绝计数，供紧凑遥测使用。 */
uint32_t AppCommProtocol_GetRejectedCount(const AppCommProtocolStats *stats);

#ifdef __cplusplus
}
#endif

#endif
