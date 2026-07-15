#include "app_comm_protocol.h"

#include "app_motor_open_loop.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

static AppCommFeedResult FeedWithoutCommit(
  AppCommProtocol *protocol,
  const char *text,
  AppCommCommand *command)
{
  AppCommFeedResult result = APP_COMM_FEED_NONE;
  for (size_t i = 0U; text[i] != '\0'; ++i) {
    const AppCommFeedResult current = AppCommProtocol_FeedByte(
      protocol, (uint8_t)text[i], command);
    if (current != APP_COMM_FEED_NONE) {
      result = current;
    }
  }
  return result;
}

static bool IsSequencedCommand(AppCommCommandType type)
{
  return type == APP_COMM_COMMAND_ARM ||
         type == APP_COMM_COMMAND_PWM ||
         type == APP_COMM_COMMAND_STOP;
}

static AppCommFeedResult Feed(
  AppCommProtocol *protocol,
  const char *text,
  AppCommCommand *command)
{
  const AppCommFeedResult result = FeedWithoutCommit(protocol, text, command);
  if (result == APP_COMM_FEED_COMMAND && IsSequencedCommand(command->type)) {
    assert(AppCommProtocol_CommitSequence(protocol, command));
  }
  return result;
}

static AppMotorOpenLoopRequest ToMotorRequest(const AppCommCommand *command)
{
  AppMotorRequestType type = APP_MOTOR_REQUEST_NONE;
  if (command->type == APP_COMM_COMMAND_ARM) {
    type = APP_MOTOR_REQUEST_ARM;
  } else if (command->type == APP_COMM_COMMAND_PWM) {
    type = APP_MOTOR_REQUEST_SET_PWM;
  } else if (command->type == APP_COMM_COMMAND_STOP) {
    type = APP_MOTOR_REQUEST_STOP;
  } else if (command->type == APP_COMM_COMMAND_ESTOP) {
    type = APP_MOTOR_REQUEST_ESTOP;
  }
  const AppMotorOpenLoopRequest request = {
    .type = type,
    .sequence = command->sequence,
    .pwm = command->pwm,
  };
  return request;
}

static void TestValidCommands(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppCommProtocolStats stats;
  AppCommProtocol_Init(&protocol);

  assert(Feed(&protocol, "ARM 1\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_ARM);
  assert(command.sequence == 1U);
  assert(Feed(&protocol, "PWM 2 -840\r\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_PWM);
  assert(command.sequence == 2U);
  assert(command.pwm == -840);
  assert(Feed(&protocol, "STOP 3\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_STOP);
  assert(Feed(&protocol, "STATUS\r\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_STATUS);
  assert(Feed(&protocol, "ESTOP\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_ESTOP);
  assert(AppCommProtocol_GetStats(&protocol, &stats));
  assert(stats.accepted_command_count == 5U);
  assert(AppCommProtocol_GetRejectedCount(&stats) == 0U);
}

static void TestSequenceAndWrap(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppCommProtocolStats stats;
  AppCommProtocol_Init(&protocol);

  assert(Feed(&protocol, "ARM 4294967294\n", &command) == APP_COMM_FEED_COMMAND);
  assert(Feed(&protocol, "PWM 0 1\n", &command) == APP_COMM_FEED_COMMAND);
  assert(Feed(&protocol, "STOP 0\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "STOP 4294967295\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "STOP 2147483648\n", &command) == APP_COMM_FEED_REJECTED);
  assert(AppCommProtocol_GetStats(&protocol, &stats));
  assert(stats.old_sequence_count == 3U);
  assert(protocol.last_sequence == 0U);
}

static void TestNumericRangeAndSyntax(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppCommProtocolStats stats;
  AppCommProtocol_Init(&protocol);

  assert(Feed(&protocol, "ARM 4294967296\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "PWM 1 2147483648\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "PWM 1 -2147483649\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "PWM 1 841\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "PWM 1 -841\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "arm 1\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "STATUS extra\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "ST\rATUS\n", &command) == APP_COMM_FEED_REJECTED);
  assert(AppCommProtocol_GetStats(&protocol, &stats));
  assert(stats.numeric_error_count == 3U);
  assert(stats.range_error_count == 2U);
  assert(stats.syntax_error_count == 4U);
  assert(stats.accepted_command_count == 0U);
}

static void TestOverflowAndRecovery(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppCommProtocolStats stats;
  AppCommProtocol_Init(&protocol);

  AppCommFeedResult result = APP_COMM_FEED_NONE;
  for (uint32_t i = 0U; i < ROBOT_CONFIG_UART_LINE_BUFFER_SIZE; ++i) {
    const AppCommFeedResult current = AppCommProtocol_FeedByte(
      &protocol, (uint8_t)'A', &command);
    assert(current == APP_COMM_FEED_NONE);
  }
  result = AppCommProtocol_FeedByte(&protocol, (uint8_t)'A', &command);
  assert(result == APP_COMM_FEED_REJECTED);
  assert(AppCommProtocol_FeedByte(&protocol, (uint8_t)'\n', &command) == APP_COMM_FEED_NONE);
  assert(Feed(&protocol, "STATUS\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_STATUS);
  assert(AppCommProtocol_GetStats(&protocol, &stats));
  assert(stats.line_overflow_count == 1U);
}

static void TestWhitespaceControlBytesAndPwmBoundaries(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppCommProtocolStats stats;
  AppCommProtocol_Init(&protocol);

  assert(Feed(&protocol, "  ARM   1  \n", &command) == APP_COMM_FEED_COMMAND);
  assert(Feed(&protocol, "PWM 2 840\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.pwm == 840);
  assert(Feed(&protocol, "PWM 3 -840\n", &command) == APP_COMM_FEED_COMMAND);
  assert(command.pwm == -840);
  assert(Feed(&protocol, "PWM 4 +1\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(
           &protocol, "PWM 4 2147483647\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(
           &protocol, "PWM 4 -2147483648\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "STATUS\t\n", &command) == APP_COMM_FEED_REJECTED);
  assert(AppCommProtocol_GetStats(&protocol, &stats));
  assert(stats.accepted_command_count == 3U);
  assert(stats.numeric_error_count == 1U);
  assert(stats.range_error_count == 2U);
  assert(stats.syntax_error_count == 1U);
}

static void TestCommitOnlyAfterDispatchAndCrossLayerSequence(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppCommProtocol_Init(&protocol);
  AppMotorOpenLoop_Init(&controller, 0U);

  assert(FeedWithoutCommit(&protocol, "ARM 0\n", &command) == APP_COMM_FEED_COMMAND);
  assert(!protocol.has_sequence);
  assert(AppCommProtocol_CommitSequence(&protocol, &command));
  AppMotorOpenLoopRequest request = ToMotorRequest(&command);
  assert(AppMotorOpenLoop_Step(&controller, 0U, false, false, &request, &output));

  assert(Feed(&protocol, "PWM 1 10\n", &command) == APP_COMM_FEED_COMMAND);
  request = ToMotorRequest(&command);
  assert(AppMotorOpenLoop_Step(&controller, 1U, false, false, &request, &output));

  assert(Feed(
           &protocol, "STOP 2147483648\n", &command) == APP_COMM_FEED_COMMAND);
  request = ToMotorRequest(&command);
  assert(AppMotorOpenLoop_Step(&controller, 2U, false, false, &request, &output));
  assert(controller.snapshot.last_sequence == 2147483648U);

  assert(Feed(
           &protocol, "PWM 2147483649 10\n", &command) == APP_COMM_FEED_COMMAND);
  request = ToMotorRequest(&command);
  assert(AppMotorOpenLoop_Step(&controller, 3U, false, false, &request, &output));
  assert(controller.snapshot.last_sequence == 2147483649U);
  assert(controller.snapshot.target_pwm == 10);

  AppCommProtocol_Init(&protocol);
  assert(Feed(&protocol, "ARM 1\n", &command) == APP_COMM_FEED_COMMAND);
  assert(FeedWithoutCommit(
           &protocol, "STOP 2147483648\n", &command) == APP_COMM_FEED_COMMAND);
  assert(protocol.last_sequence == 1U);
  assert(Feed(&protocol, "PWM 2 10\n", &command) == APP_COMM_FEED_COMMAND);
  assert(protocol.last_sequence == 2U);
}

static void TestRejectedLineDoesNotRefreshMotorTimeout(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppCommProtocol_Init(&protocol);
  AppMotorOpenLoop_Init(&controller, 0U);

  assert(Feed(&protocol, "ARM 1\n", &command) == APP_COMM_FEED_COMMAND);
  AppMotorOpenLoopRequest request = ToMotorRequest(&command);
  assert(AppMotorOpenLoop_Step(&controller, 0U, false, false, &request, &output));
  assert(Feed(&protocol, "PWM 2 100\n", &command) == APP_COMM_FEED_COMMAND);
  request = ToMotorRequest(&command);
  assert(AppMotorOpenLoop_Step(&controller, 100U, false, false, &request, &output));

  assert(Feed(&protocol, "BAD\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "PWM 1 100\n", &command) == APP_COMM_FEED_REJECTED);
  assert(Feed(&protocol, "PWM 3 841\n", &command) == APP_COMM_FEED_REJECTED);
  assert(FeedWithoutCommit(
           &protocol, "PWM 3 100\n", &command) == APP_COMM_FEED_COMMAND);
  assert(protocol.last_sequence == 2U);
  assert(controller.last_valid_command_ms == 100U);
  const AppMotorOpenLoopRequest no_request = {
    .type = APP_MOTOR_REQUEST_NONE,
    .sequence = 0U,
    .pwm = 0,
  };
  assert(AppMotorOpenLoop_Step(&controller, 600U, false, false, &no_request, &output));
  assert(controller.snapshot.command_timeout_count == 1U);
  assert(controller.snapshot.target_pwm == 0);
}

static void TestArgumentsAndSaturation(void)
{
  AppCommProtocol protocol;
  AppCommCommand command;
  AppCommProtocolStats stats;
  AppCommProtocol_Init(NULL);
  AppCommProtocol_Init(&protocol);
  assert(AppCommProtocol_FeedByte(NULL, (uint8_t)'A', &command) == APP_COMM_FEED_REJECTED);
  assert(AppCommProtocol_FeedByte(&protocol, (uint8_t)'A', NULL) == APP_COMM_FEED_REJECTED);
  assert(!AppCommProtocol_CommitSequence(NULL, &command));
  assert(!AppCommProtocol_CommitSequence(&protocol, NULL));
  command.type = APP_COMM_COMMAND_STATUS;
  assert(!AppCommProtocol_CommitSequence(&protocol, &command));
  assert(!AppCommProtocol_GetStats(NULL, &stats));
  assert(!AppCommProtocol_GetStats(&protocol, NULL));
  assert(AppCommProtocol_GetRejectedCount(NULL) == 0U);

  protocol.stats.syntax_error_count = UINT32_MAX;
  assert(Feed(&protocol, "BAD\n", &command) == APP_COMM_FEED_REJECTED);
  assert(protocol.stats.syntax_error_count == UINT32_MAX);
  memset(&stats, 0, sizeof(stats));
  stats.syntax_error_count = UINT32_MAX;
  stats.numeric_error_count = UINT32_MAX;
  assert(AppCommProtocol_GetRejectedCount(&stats) == UINT32_MAX);
}

int main(void)
{
  TestValidCommands();
  TestSequenceAndWrap();
  TestNumericRangeAndSyntax();
  TestOverflowAndRecovery();
  TestWhitespaceControlBytesAndPwmBoundaries();
  TestCommitOnlyAfterDispatchAndCrossLayerSequence();
  TestRejectedLineDoesNotRefreshMotorTimeout();
  TestArgumentsAndSaturation();
  return 0;
}
