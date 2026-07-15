#ifndef APP_COMM_PROTOCOL_H
#define APP_COMM_PROTOCOL_H

#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/* 临时试验协议只负责解析与验序，普通运动命令成功入队后才提交序号。 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_COMM_COMMAND_NONE = 0,
  APP_COMM_COMMAND_ARM,
  APP_COMM_COMMAND_PWM,
  APP_COMM_COMMAND_STOP,
  APP_COMM_COMMAND_ESTOP,
  APP_COMM_COMMAND_STATUS
} AppCommCommandType;

typedef enum {
  APP_COMM_FEED_NONE = 0,
  APP_COMM_FEED_COMMAND,
  APP_COMM_FEED_REJECTED
} AppCommFeedResult;

typedef struct {
  AppCommCommandType type;
  uint32_t sequence;
  int16_t pwm;
} AppCommCommand;

typedef struct {
  uint32_t accepted_command_count;
  uint32_t syntax_error_count;
  uint32_t numeric_error_count;
  uint32_t range_error_count;
  uint32_t old_sequence_count;
  uint32_t line_overflow_count;
} AppCommProtocolStats;

typedef struct {
  uint8_t line[ROBOT_CONFIG_UART_LINE_BUFFER_SIZE];
  uint16_t line_length;
  uint32_t last_sequence;
  AppCommProtocolStats stats;
  bool discarding_line;
  bool has_sequence;
} AppCommProtocol;

void AppCommProtocol_Init(AppCommProtocol *protocol);
AppCommFeedResult AppCommProtocol_FeedByte(
  AppCommProtocol *protocol,
  uint8_t byte,
  AppCommCommand *command);
bool AppCommProtocol_CommitSequence(
  AppCommProtocol *protocol,
  const AppCommCommand *command);
bool AppCommProtocol_GetStats(
  const AppCommProtocol *protocol,
  AppCommProtocolStats *stats);
uint32_t AppCommProtocol_GetRejectedCount(const AppCommProtocolStats *stats);

#ifdef __cplusplus
}
#endif

#endif
