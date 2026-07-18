#include "app_motion_gate.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>

static AppCommFeedResult FeedLine(
  AppCommProtocol *protocol,
  const char *line,
  AppCommCommand *command)
{
  AppCommFeedResult result = APP_COMM_FEED_NONE;
  for (size_t index = 0U; line[index] != '\0'; ++index) {
    const AppCommFeedResult current = AppCommProtocol_FeedByte(
      protocol, (uint8_t)line[index], command);
    if (current != APP_COMM_FEED_NONE) {
      result = current;
    }
  }
  return result;
}

static void TestAvailabilityPredicate(void)
{
  assert(AppMotionGate_ComputeAvailable(true, false, false, true));
  assert(!AppMotionGate_ComputeAvailable(false, false, false, true));
  assert(!AppMotionGate_ComputeAvailable(true, true, false, true));
  assert(!AppMotionGate_ComputeAvailable(true, false, true, true));
  assert(!AppMotionGate_ComputeAvailable(true, false, false, false));
}

static void TestGenerationInvalidatesQueuedRequest(void)
{
  AppMotionGate gate;
  AppMotionGate_Init(&gate);
  assert(!gate.available);
  assert(gate.generation == 0U);
  assert(AppMotionGate_Update(&gate, true));

  const AppCommCommand command = {
    .type = APP_COMM_COMMAND_PWM,
    .sequence = 7U,
    .pwm = 100,
  };
  AppMotionGatedRequest request;
  assert(AppMotionGate_PrepareCommand(&gate, &command, &request) == APP_MOTION_PREPARE_READY);
  assert(AppMotionGate_IsRequestCurrent(&gate, &request));

  assert(AppMotionGate_Update(&gate, false));
  assert(gate.generation == 1U);
  assert(!AppMotionGate_IsRequestCurrent(&gate, &request));
  assert(AppMotionGate_Update(&gate, false));
  assert(gate.generation == 1U);
  assert(AppMotionGate_Update(&gate, true));
  assert(!AppMotionGate_IsRequestCurrent(&gate, &request));
}

static void TestCommandAdmission(void)
{
  AppMotionGate gate;
  AppMotionGate_Init(&gate);
  AppMotionGatedRequest request;
  AppCommCommand command = {
    .type = APP_COMM_COMMAND_ARM,
    .sequence = 1U,
    .pwm = 0,
  };

  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_GATE_REJECTED);
  command.type = APP_COMM_COMMAND_PWM;
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_GATE_REJECTED);
  command.type = APP_COMM_COMMAND_STOP;
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_GATE_REJECTED);

  command.type = APP_COMM_COMMAND_STATUS;
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_NOT_MOTION_COMMAND);
  command.type = APP_COMM_COMMAND_ESTOP;
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_NOT_MOTION_COMMAND);

  assert(AppMotionGate_Update(&gate, true));
  command.type = APP_COMM_COMMAND_STOP;
  command.sequence = 9U;
  assert(AppMotionGate_PrepareCommand(&gate, &command, &request) == APP_MOTION_PREPARE_READY);
  assert(request.request.type == APP_MOTOR_REQUEST_STOP);
  assert(request.request.sequence == 9U);
}

static void TestGateRejectDoesNotCommitSequence(void)
{
  AppCommProtocol protocol;
  AppCommProtocol_Init(&protocol);
  AppMotionGate gate;
  AppMotionGate_Init(&gate);
  AppCommCommand command;
  AppMotionGatedRequest request;

  assert(
    FeedLine(&protocol, "ARM 1\n", &command) ==
    APP_COMM_FEED_COMMAND);
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_GATE_REJECTED);
  assert(!protocol.has_sequence);

  /*
   * 前置门拒绝时没有提交序号，所以恢复许可后，上位机不需要猜测新序号，可以原样重试。
   * 真正准备入队成功后才提交，随后再次使用同一序号会被协议层作为旧命令拒绝。
   */
  assert(AppMotionGate_Update(&gate, true));
  assert(
    FeedLine(&protocol, "ARM 1\n", &command) ==
    APP_COMM_FEED_COMMAND);
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_READY);
  assert(AppCommProtocol_CommitSequence(&protocol, &command));
  assert(
    FeedLine(&protocol, "ARM 1\n", &command) ==
    APP_COMM_FEED_REJECTED);

  /*
   * STATUS 与 ESTOP 不属于普通运动序列，无论许可状态如何都由通信任务独立分流。
   */
  assert(AppMotionGate_Update(&gate, false));
  assert(
    FeedLine(&protocol, "STATUS\n", &command) ==
    APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_STATUS);
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_NOT_MOTION_COMMAND);
  assert(
    FeedLine(&protocol, "ESTOP\n", &command) ==
    APP_COMM_FEED_COMMAND);
  assert(command.type == APP_COMM_COMMAND_ESTOP);
  assert(
    AppMotionGate_PrepareCommand(&gate, &command, &request) ==
    APP_MOTION_PREPARE_NOT_MOTION_COMMAND);
}

static void TestGenerationOverflowFailsClosed(void)
{
  AppMotionGate gate = {
    .generation = UINT32_MAX,
    .available = true,
  };
  assert(!AppMotionGate_Update(&gate, false));
  assert(!gate.available);
  assert(gate.generation == UINT32_MAX);
  assert(!AppMotionGate_Update(NULL, false));
  assert(!AppMotionGate_IsRequestCurrent(NULL, NULL));
}

int main(void)
{
  TestAvailabilityPredicate();
  TestGenerationInvalidatesQueuedRequest();
  TestCommandAdmission();
  TestGateRejectDoesNotCommitSequence();
  TestGenerationOverflowFailsClosed();
  return 0;
}
