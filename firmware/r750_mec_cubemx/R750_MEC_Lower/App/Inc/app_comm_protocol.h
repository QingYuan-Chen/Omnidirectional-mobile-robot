#ifndef APP_COMM_PROTOCOL_H
#define APP_COMM_PROTOCOL_H

#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * UART4 临时试验命令的无动态内存解析器。
 *
 * 模块以字节流为输入，按换行符组帧，再把来自信任边界之外的文本转换成定长内部命令。
 * 解析过程会检查可打印字符、字段个数、整数溢出、PWM 范围和 32 位回绕序号。解析成功
 * 只表示语法与序号候选有效；ARM、PWM、STOP 必须同时通过实时运动许可并成功入队后
 * 再提交序号，从而允许许可关闭或队满时使用原序号重试。ESTOP 与 STATUS 不参与
 * 有序运动命令序列。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 解析后交给通信任务分派的命令类型；NONE 仅表示没有形成有效命令。 */
typedef enum {
  APP_COMM_COMMAND_NONE = 0,
  APP_COMM_COMMAND_ARM,
  APP_COMM_COMMAND_PWM,
  APP_COMM_COMMAND_STOP,
  APP_COMM_COMMAND_ESTOP,
  APP_COMM_COMMAND_STATUS
} AppCommCommandType;

/*
 * 单字节处理结果。
 * NONE：仍在收集当前行，或正在丢弃一条已判坏的行；
 * COMMAND：换行已结束一条完整且合法的命令；
 * REJECTED：本字节或本行触发拒绝，调用者可继续输入后续字节。
 */
typedef enum {
  APP_COMM_FEED_NONE = 0,
  APP_COMM_FEED_COMMAND,
  APP_COMM_FEED_REJECTED
} AppCommFeedResult;

/* 规范化命令。结构体只保存值，不引用解析器行缓冲区，可安全复制到消息队列。 */
typedef struct {
  AppCommCommandType type;
  uint32_t sequence;
  int16_t pwm;
} AppCommCommand;

/*
 * 协议累计统计。所有计数均采用 uint32_t 饱和递增，不会回绕成较小值。
 * 语法、数字、范围、旧序号和行溢出分开记录，便于区分链路噪声、上位机格式错误与
 * 命令重放；accepted 仅表示解析接受，不代表命令已经执行。
 */
typedef struct {
  uint32_t accepted_command_count;
  uint32_t syntax_error_count;
  uint32_t numeric_error_count;
  uint32_t range_error_count;
  uint32_t old_sequence_count;
  uint32_t line_overflow_count;
} AppCommProtocolStats;

/*
 * 解析器私有状态，由一个通信任务独占读写。
 * line 保存尚未遇到换行的字节；discarding_line 为真时持续忽略输入直到下一换行，
 * 防止超长行或非法字符后的残片被误当成新命令；last_sequence 只保存已经提交的序号。
 */
typedef struct {
  uint8_t line[ROBOT_CONFIG_UART_LINE_BUFFER_SIZE];
  uint16_t line_length;
  uint32_t last_sequence;
  AppCommProtocolStats stats;
  bool discarding_line;
  bool has_sequence;
} AppCommProtocol;

/*
 * 初始化解析器并清空全部历史。
 * 参数为空时不执行操作；重新调用会同时丢弃未完成行、已提交序号和累计统计。
 */
void AppCommProtocol_Init(AppCommProtocol *protocol);
/*
 * 输入一个接收字节。
 * protocol 和 command 必须有效；command 每次调用都会先复位为 NONE。普通字节通常返回
 * NONE，遇到换行后才解析完整行并返回 COMMAND 或 REJECTED。函数不阻塞、不分配内存，
 * 适合由通信任务连续清空 UART 接收环形缓冲区。
 */
AppCommFeedResult AppCommProtocol_FeedByte(
  AppCommProtocol *protocol,
  uint8_t byte,
  AppCommCommand *command);
/*
 * 提交一条已经通过实时运动许可且成功进入电机命令队列的 ARM、PWM 或 STOP 序号。
 * 返回 true 表示序号已成为新的防重放基线；参数无效、命令类型不适用或序号不再更新时
 * 返回 false。不得在许可检查或队列入队前调用，否则前置拒绝会造成合法重试被当作
 * 旧命令拒绝。
 */
bool AppCommProtocol_CommitSequence(
  AppCommProtocol *protocol,
  const AppCommCommand *command);
/* 复制当前协议统计到调用者缓冲区；任一指针为空时返回 false。 */
bool AppCommProtocol_GetStats(
  const AppCommProtocol *protocol,
  AppCommProtocolStats *stats);
/*
 * 把五类拒绝计数做饱和求和，生成遥测使用的总拒绝数。
 * 参数为空时返回 0；总数只用于诊断，不能反推出某一具体错误类型。
 */
uint32_t AppCommProtocol_GetRejectedCount(const AppCommProtocolStats *stats);

#ifdef __cplusplus
}
#endif

#endif
