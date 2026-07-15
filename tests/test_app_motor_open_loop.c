#include "app_motor_open_loop.h"

#include "robot_config.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>

static const AppMotorOpenLoopRequest no_request = {
  .type = APP_MOTOR_REQUEST_NONE,
  .sequence = 0U,
  .pwm = 0,
};

static AppMotorOpenLoopRequest Request(
  AppMotorRequestType type,
  uint32_t sequence,
  int32_t pwm)
{
  const AppMotorOpenLoopRequest request = {
    .type = type,
    .sequence = sequence,
    .pwm = pwm,
  };
  return request;
}

static void Step(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  bool motion_inhibited,
  bool hard_fault_latched,
  const AppMotorOpenLoopRequest *request,
  AppMotorOpenLoopOutput *output)
{
  assert(AppMotorOpenLoop_Step(
    controller,
    now_ms,
    motion_inhibited,
    hard_fault_latched,
    request,
    output));
}

static void Arm(AppMotorOpenLoop *controller, uint32_t now_ms)
{
  AppMotorOpenLoopOutput output;
  const AppMotorOpenLoopRequest request = Request(APP_MOTOR_REQUEST_ARM, 0U, 0);
  Step(controller, now_ms, false, false, &request, &output);
  assert(output.mode == APP_MOTOR_OUTPUT_BRAKE);
  assert(controller->snapshot.state == APP_MOTOR_OPEN_LOOP_ARMED);
}

static void TestBootDisarmedAndArguments(void)
{
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoopSnapshot snapshot;
  AppMotorOpenLoop_Init(&controller, 100U);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED);
  Step(&controller, 100U, false, false, &no_request, &output);
  assert(output.mode == APP_MOTOR_OUTPUT_BRAKE);
  assert(output.pwm == 0);
  assert(!output.emergency_coast_request);
  assert(AppMotorOpenLoop_GetSnapshot(&controller, &snapshot));
  assert(snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED);

  AppMotorOpenLoop_Init(NULL, 0U);
  assert(!AppMotorOpenLoop_Step(NULL, 0U, false, false, &no_request, &output));
  assert(!AppMotorOpenLoop_Step(&controller, 0U, false, false, NULL, &output));
  assert(!AppMotorOpenLoop_Step(&controller, 0U, false, false, &no_request, NULL));
  assert(!AppMotorOpenLoop_GetSnapshot(NULL, &snapshot));
  assert(!AppMotorOpenLoop_GetSnapshot(&controller, NULL));
}

static void TestLimitRampAndSequence(void)
{
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoop_Init(&controller, 0U);
  Arm(&controller, 0U);

  AppMotorOpenLoopRequest request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 1000);
  Step(&controller, 10U, false, false, &request, &output);
  assert(controller.snapshot.target_pwm == ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT);
  assert(controller.snapshot.applied_pwm == 10);
  assert(controller.snapshot.clamped_command_count == 1U);
  assert(output.mode == APP_MOTOR_OUTPUT_DRIVE);
  assert(output.pwm == 10);

  Step(&controller, 20U, false, false, &no_request, &output);
  assert(controller.snapshot.applied_pwm == 20);

  request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 100);
  Step(&controller, 21U, false, false, &request, &output);
  assert(controller.snapshot.rejected_command_count == 1U);
  assert(controller.snapshot.target_pwm == ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT);

  controller.snapshot.last_sequence = UINT32_MAX - 1U;
  controller.snapshot.has_sequence = true;
  request = Request(APP_MOTOR_REQUEST_SET_PWM, 0U, 100);
  Step(&controller, 22U, false, false, &request, &output);
  assert(controller.snapshot.last_sequence == 0U);
  assert(controller.snapshot.target_pwm == 100);

  request = Request((AppMotorRequestType)99, 0U, 0);
  Step(&controller, 23U, false, false, &request, &output);
  assert(controller.snapshot.rejected_command_count == 2U);
}

static void TestReverseCrossesZeroAndBrakes(void)
{
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoop_Init(&controller, 0U);
  Arm(&controller, 0U);

  AppMotorOpenLoopRequest request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 10);
  Step(&controller, 10U, false, false, &request, &output);
  assert(controller.snapshot.applied_pwm == 10);

  request = Request(APP_MOTOR_REQUEST_SET_PWM, 2U, -10);
  Step(&controller, 20U, false, false, &request, &output);
  assert(controller.snapshot.applied_pwm == 0);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE);
  assert(output.mode == APP_MOTOR_OUTPUT_BRAKE);

  for (uint32_t i = 1U; i < ROBOT_CONFIG_MA_REVERSE_BRAKE_TICKS; ++i) {
    Step(&controller, 20U + i, false, false, &no_request, &output);
    assert(output.mode == APP_MOTOR_OUTPUT_BRAKE);
    assert(controller.snapshot.applied_pwm == 0);
  }
  Step(
    &controller,
    20U + ROBOT_CONFIG_MA_REVERSE_BRAKE_TICKS,
    false,
    false,
    &no_request,
    &output);
  assert(controller.snapshot.applied_pwm == -1);
  assert(output.mode == APP_MOTOR_OUTPUT_DRIVE);
  assert(output.pwm == -1);
}

static void TestStopAndTimeout(void)
{
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoop_Init(&controller, 0U);
  Arm(&controller, 0U);

  AppMotorOpenLoopRequest request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 20);
  Step(&controller, 20U, false, false, &request, &output);
  request = Request(APP_MOTOR_REQUEST_STOP, 0U, 0);
  Step(&controller, 30U, false, false, &request, &output);
  assert(controller.snapshot.applied_pwm == 10);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_STOPPING);
  assert(output.mode == APP_MOTOR_OUTPUT_DRIVE);
  Step(&controller, 40U, false, false, &no_request, &output);
  assert(controller.snapshot.applied_pwm == 0);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_ARMED);
  assert(output.mode == APP_MOTOR_OUTPUT_BRAKE);
  Step(&controller, 1000U, false, false, &no_request, &output);
  assert(controller.snapshot.command_timeout_count == 1U);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED);

  AppMotorOpenLoop_Init(&controller, 0U);
  Arm(&controller, 0U);
  request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 100);
  Step(&controller, 100U, false, false, &request, &output);
  Step(&controller, 599U, false, false, &no_request, &output);
  assert(controller.snapshot.command_timeout_count == 0U);
  Step(&controller, 600U, false, false, &no_request, &output);
  assert(controller.snapshot.command_timeout_count == 1U);
  assert(controller.snapshot.target_pwm == 0);
  assert(controller.snapshot.applied_pwm == 99);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_STOPPING);
  assert(controller.timeout_active);
  assert(controller.disarm_at_zero);
  request = Request(APP_MOTOR_REQUEST_STOP, 0U, 0);
  Step(&controller, 601U, false, false, &request, &output);
  assert(controller.timeout_active);
  assert(controller.disarm_at_zero);
  request = Request(APP_MOTOR_REQUEST_SET_PWM, 2U, 100);
  Step(&controller, 602U, false, false, &request, &output);
  assert(controller.snapshot.rejected_command_count == 1U);
  assert(controller.snapshot.target_pwm == 0);
  assert(controller.snapshot.applied_pwm == 97);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_STOPPING);
  Step(&controller, 699U, false, false, &no_request, &output);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED);
  assert(output.mode == APP_MOTOR_OUTPUT_BRAKE);
  Step(&controller, 700U, false, false, &request, &output);
  assert(controller.snapshot.rejected_command_count == 2U);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED);
  Arm(&controller, 701U);
  request = Request(APP_MOTOR_REQUEST_SET_PWM, 3U, 10);
  Step(&controller, 702U, false, false, &request, &output);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_RUNNING);
  assert(controller.snapshot.applied_pwm == 1);
}

static void TestInhibitAndLatchedFaults(void)
{
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoop_Init(&controller, 0U);
  Arm(&controller, 0U);
  AppMotorOpenLoopRequest request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 10);
  Step(&controller, 10U, false, false, &request, &output);

  Step(&controller, 11U, true, false, &no_request, &output);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_INHIBITED);
  assert(controller.snapshot.applied_pwm == 0);
  assert(controller.snapshot.inhibit_transition_count == 1U);
  assert(output.mode == APP_MOTOR_OUTPUT_COAST);
  assert(!output.emergency_coast_request);
  Step(&controller, 12U, false, false, &no_request, &output);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED);
  assert(output.mode == APP_MOTOR_OUTPUT_BRAKE);

  request = Request(APP_MOTOR_REQUEST_ESTOP, 0U, 0);
  Step(&controller, 13U, false, false, &request, &output);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED);
  assert(controller.snapshot.estop_latch_count == 1U);
  assert(output.mode == APP_MOTOR_OUTPUT_NONE);
  assert(output.emergency_coast_request);
  request = Request(APP_MOTOR_REQUEST_ARM, 0U, 0);
  Step(&controller, 14U, false, false, &request, &output);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED);
  assert(!output.emergency_coast_request);
  assert(output.mode == APP_MOTOR_OUTPUT_NONE);

  AppMotorOpenLoop_Init(&controller, 0U);
  Step(&controller, 1U, false, true, &no_request, &output);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED);
  assert(output.emergency_coast_request);
}

static void TestCountersSaturate(void)
{
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoop_Init(&controller, 0U);
  controller.snapshot.rejected_command_count = UINT32_MAX;
  controller.snapshot.clamped_command_count = UINT32_MAX;
  controller.snapshot.inhibit_transition_count = UINT32_MAX;
  controller.snapshot.estop_latch_count = UINT32_MAX;

  AppMotorOpenLoopRequest request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 1000);
  Step(&controller, 1U, false, false, &request, &output);
  assert(controller.snapshot.rejected_command_count == UINT32_MAX);

  AppMotorOpenLoop_Init(&controller, 0U);
  controller.snapshot.clamped_command_count = UINT32_MAX;
  Arm(&controller, 0U);
  request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 1000);
  Step(&controller, 1U, false, false, &request, &output);
  assert(controller.snapshot.clamped_command_count == UINT32_MAX);
  controller.snapshot.inhibit_transition_count = UINT32_MAX;
  controller.snapshot.estop_latch_count = UINT32_MAX;
  Step(&controller, 2U, true, false, &no_request, &output);
  assert(controller.snapshot.inhibit_transition_count == UINT32_MAX);
  Step(&controller, 3U, false, true, &no_request, &output);
  assert(controller.snapshot.estop_latch_count == UINT32_MAX);
  assert(controller.snapshot.clamped_command_count == UINT32_MAX);
}

static void TestTickWrap(void)
{
  AppMotorOpenLoop controller;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoop_Init(&controller, UINT32_MAX - 5U);
  Arm(&controller, UINT32_MAX - 5U);
  AppMotorOpenLoopRequest request = Request(APP_MOTOR_REQUEST_SET_PWM, 1U, 10);
  Step(&controller, UINT32_MAX - 1U, false, false, &request, &output);
  assert(controller.snapshot.applied_pwm == 4);
  Step(&controller, 2U, false, false, &no_request, &output);
  assert(controller.snapshot.applied_pwm == 8);

  Step(
    &controller,
    (UINT32_MAX - 1U) + ROBOT_CONFIG_CMD_TIMEOUT_MS,
    false,
    false,
    &no_request,
    &output);
  assert(controller.snapshot.command_timeout_count == 1U);
  assert(controller.snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED);
}

int main(void)
{
  TestBootDisarmedAndArguments();
  TestLimitRampAndSequence();
  TestReverseCrossesZeroAndBrakes();
  TestStopAndTimeout();
  TestInhibitAndLatchedFaults();
  TestCountersSaturate();
  TestTickWrap();
  return 0;
}
