#include "app_motor_open_loop.h"

#include "robot_config.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

_Static_assert(ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT > 0,
               "MA open-loop limit must be positive");
_Static_assert(ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT <= ROBOT_CONFIG_PWM_LIMIT,
               "MA open-loop limit exceeds BSP PWM limit");
_Static_assert(ROBOT_CONFIG_MA_PWM_RAMP_COUNTS_PER_MS > 0U,
               "MA open-loop ramp must be non-zero");
_Static_assert(ROBOT_CONFIG_MA_REVERSE_BRAKE_TICKS > 0U,
               "MA reverse brake duration must be non-zero");

static uint32_t AppMotorOpenLoop_AddSaturated(uint32_t value, uint32_t increment)
{
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static bool AppMotorOpenLoop_SequenceIsNewer(uint32_t sequence, uint32_t previous)
{
  /* 与协议层使用相同的半区间规则，保证序号回绕时两层判断一致。 */
  const uint32_t difference = sequence - previous;
  return difference != 0U && difference <= (uint32_t)INT32_MAX;
}

static int16_t AppMotorOpenLoop_ClampPwm(AppMotorOpenLoop *controller, int32_t pwm)
{
  int32_t limited = pwm;
  if (limited > ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT) {
    limited = ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT;
  } else if (limited < -ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT) {
    limited = -ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT;
  }
  if (limited != pwm) {
    controller->snapshot.clamped_command_count = AppMotorOpenLoop_AddSaturated(
      controller->snapshot.clamped_command_count, 1U);
  }
  return (int16_t)limited;
}

static uint32_t AppMotorOpenLoop_RampCapacity(uint32_t elapsed_ms)
{
  if (elapsed_ms > (UINT32_MAX / ROBOT_CONFIG_MA_PWM_RAMP_COUNTS_PER_MS)) {
    return UINT32_MAX;
  }
  return elapsed_ms * ROBOT_CONFIG_MA_PWM_RAMP_COUNTS_PER_MS;
}

static int16_t AppMotorOpenLoop_MoveToward(int16_t current, int16_t target, uint32_t capacity)
{
  const int32_t difference = (int32_t)target - (int32_t)current;
  if (difference > 0) {
    const uint32_t increment = (uint32_t)difference < capacity
                                 ? (uint32_t)difference
                                 : capacity;
    return (int16_t)((int32_t)current + (int32_t)increment);
  }
  if (difference < 0) {
    const uint32_t magnitude = (uint32_t)(-difference);
    const uint32_t decrement = magnitude < capacity ? magnitude : capacity;
    return (int16_t)((int32_t)current - (int32_t)decrement);
  }
  return current;
}

static void AppMotorOpenLoop_RejectRequest(AppMotorOpenLoop *controller)
{
  controller->snapshot.rejected_command_count = AppMotorOpenLoop_AddSaturated(
    controller->snapshot.rejected_command_count, 1U);
}

static void AppMotorOpenLoop_LatchEstop(
  AppMotorOpenLoop *controller,
  AppMotorOpenLoopOutput *output)
{
  if (controller->snapshot.state == APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED) {
    return;
  }

  /* 急停是复位恢复型状态，不允许后续普通命令在本次运行中解除。 */
  controller->snapshot.state = APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED;
  controller->snapshot.target_pwm = 0;
  controller->snapshot.applied_pwm = 0;
  controller->snapshot.estop_latch_count = AppMotorOpenLoop_AddSaturated(
    controller->snapshot.estop_latch_count, 1U);
  controller->timeout_active = false;
  controller->disarm_at_zero = true;
  controller->reverse_brake_ticks_remaining = 0U;
  output->mode = APP_MOTOR_OUTPUT_NONE;
  output->pwm = 0;
  output->emergency_coast_request = true;
}

static bool AppMotorOpenLoop_IsCommandState(AppMotorOpenLoopState state)
{
  return state == APP_MOTOR_OPEN_LOOP_ARMED ||
         state == APP_MOTOR_OPEN_LOOP_RUNNING ||
         state == APP_MOTOR_OPEN_LOOP_STOPPING ||
         state == APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE;
}

static bool AppMotorOpenLoop_RequiresCommandRefresh(AppMotorOpenLoopState state)
{
  return state == APP_MOTOR_OPEN_LOOP_ARMED ||
         state == APP_MOTOR_OPEN_LOOP_RUNNING ||
         state == APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE;
}

void AppMotorOpenLoop_Init(AppMotorOpenLoop *controller, uint32_t now_ms)
{
  if (controller == NULL) {
    return;
  }

  memset(controller, 0, sizeof(*controller));
  controller->snapshot.state = APP_MOTOR_OPEN_LOOP_DISARMED;
  controller->last_step_ms = now_ms;
  controller->last_valid_command_ms = now_ms;
}

bool AppMotorOpenLoop_Step(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  bool motion_inhibited,
  bool hard_fault_latched,
  const AppMotorOpenLoopRequest *request,
  AppMotorOpenLoopOutput *output)
{
  if (controller == NULL || request == NULL || output == NULL) {
    return false;
  }

  output->mode = APP_MOTOR_OUTPUT_BRAKE;
  output->pwm = 0;
  output->emergency_coast_request = false;
  const uint32_t elapsed_ms = now_ms - controller->last_step_ms;
  controller->last_step_ms = now_ms;

  if (hard_fault_latched || request->type == APP_MOTOR_REQUEST_ESTOP) {
    AppMotorOpenLoop_LatchEstop(controller, output);
    return true;
  }
  if (controller->snapshot.state == APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED) {
    output->mode = APP_MOTOR_OUTPUT_NONE;
    return true;
  }

  if (motion_inhibited) {
    if (controller->snapshot.state != APP_MOTOR_OPEN_LOOP_INHIBITED) {
      controller->snapshot.inhibit_transition_count = AppMotorOpenLoop_AddSaturated(
        controller->snapshot.inhibit_transition_count, 1U);
    }
    /* 可恢复禁止也清除旧序号，使恢复后必须重新 ARM，不能沿用旧运行上下文。 */
    controller->snapshot.state = APP_MOTOR_OPEN_LOOP_INHIBITED;
    controller->snapshot.target_pwm = 0;
    controller->snapshot.applied_pwm = 0;
    controller->snapshot.has_sequence = false;
    controller->timeout_active = false;
    controller->disarm_at_zero = true;
    controller->reverse_brake_ticks_remaining = 0U;
    output->mode = APP_MOTOR_OUTPUT_COAST;
    return true;
  }
  if (controller->snapshot.state == APP_MOTOR_OPEN_LOOP_INHIBITED) {
    controller->snapshot.state = APP_MOTOR_OPEN_LOOP_DISARMED;
    controller->disarm_at_zero = false;
  }

  AppMotorRequestType request_type = request->type;
  if (request_type == APP_MOTOR_REQUEST_ARM ||
      request_type == APP_MOTOR_REQUEST_SET_PWM ||
      request_type == APP_MOTOR_REQUEST_STOP) {
    /* 合法新序号先被状态机消费，即使当前状态拒绝其语义也不能再次重放。 */
    if (controller->snapshot.has_sequence &&
        !AppMotorOpenLoop_SequenceIsNewer(
          request->sequence, controller->snapshot.last_sequence)) {
      AppMotorOpenLoop_RejectRequest(controller);
      request_type = APP_MOTOR_REQUEST_NONE;
    } else {
      controller->snapshot.last_sequence = request->sequence;
      controller->snapshot.has_sequence = true;
    }
  }

  bool command_refreshed = false;
  switch (request_type) {
    case APP_MOTOR_REQUEST_NONE:
      break;

    case APP_MOTOR_REQUEST_ARM:
      if (controller->snapshot.state != APP_MOTOR_OPEN_LOOP_DISARMED) {
        AppMotorOpenLoop_RejectRequest(controller);
        break;
      }
      controller->snapshot.state = APP_MOTOR_OPEN_LOOP_ARMED;
      controller->snapshot.target_pwm = 0;
      controller->snapshot.applied_pwm = 0;
      controller->last_valid_command_ms = now_ms;
      controller->timeout_active = false;
      controller->disarm_at_zero = false;
      command_refreshed = true;
      break;

    case APP_MOTOR_REQUEST_SET_PWM:
      if (!AppMotorOpenLoop_IsCommandState(controller->snapshot.state) ||
          controller->timeout_active || controller->disarm_at_zero) {
        AppMotorOpenLoop_RejectRequest(controller);
        break;
      }
      controller->snapshot.target_pwm = AppMotorOpenLoop_ClampPwm(controller, request->pwm);
      if (controller->snapshot.target_pwm == 0) {
        controller->snapshot.state = APP_MOTOR_OPEN_LOOP_STOPPING;
      } else if (controller->snapshot.state != APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE) {
        controller->snapshot.state = APP_MOTOR_OPEN_LOOP_RUNNING;
      }
      controller->last_valid_command_ms = now_ms;
      controller->timeout_active = false;
      controller->disarm_at_zero = false;
      command_refreshed = true;
      break;

    case APP_MOTOR_REQUEST_STOP:
      if (controller->snapshot.state != APP_MOTOR_OPEN_LOOP_DISARMED) {
        controller->snapshot.target_pwm = 0;
        controller->snapshot.state = APP_MOTOR_OPEN_LOOP_STOPPING;
        if (!controller->timeout_active && !controller->disarm_at_zero) {
          controller->last_valid_command_ms = now_ms;
          controller->disarm_at_zero = false;
          command_refreshed = true;
        }
      }
      break;

    case APP_MOTOR_REQUEST_ESTOP:
      break;

    default:
      AppMotorOpenLoop_RejectRequest(controller);
      break;
  }

  if (!command_refreshed && !controller->timeout_active &&
      AppMotorOpenLoop_RequiresCommandRefresh(controller->snapshot.state) &&
      (now_ms - controller->last_valid_command_ms) >= ROBOT_CONFIG_CMD_TIMEOUT_MS) {
    /* 超时停车不可被新 PWM 或 STOP 取消，停稳并撤销使能后才能重新 ARM。 */
    controller->snapshot.command_timeout_count = AppMotorOpenLoop_AddSaturated(
      controller->snapshot.command_timeout_count, 1U);
    controller->snapshot.target_pwm = 0;
    controller->snapshot.state = APP_MOTOR_OPEN_LOOP_STOPPING;
    controller->timeout_active = true;
    controller->disarm_at_zero = true;
  }

  const uint32_t ramp_capacity = AppMotorOpenLoop_RampCapacity(elapsed_ms);
  const bool reversing =
    (controller->snapshot.applied_pwm > 0 && controller->snapshot.target_pwm < 0) ||
    (controller->snapshot.applied_pwm < 0 && controller->snapshot.target_pwm > 0);

  if (controller->snapshot.state == APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE) {
    if (controller->reverse_brake_ticks_remaining > 0U) {
      controller->reverse_brake_ticks_remaining--;
      output->mode = APP_MOTOR_OUTPUT_BRAKE;
      return true;
    }
    controller->snapshot.state = APP_MOTOR_OPEN_LOOP_RUNNING;
  } else if (reversing) {
    /* 反向目标先按斜坡降到零，再保持规定的制动周期，禁止跨零直接换向。 */
    controller->snapshot.applied_pwm = AppMotorOpenLoop_MoveToward(
      controller->snapshot.applied_pwm, 0, ramp_capacity);
    if (controller->snapshot.applied_pwm == 0) {
      controller->snapshot.state = APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE;
      controller->reverse_brake_ticks_remaining =
        ROBOT_CONFIG_MA_REVERSE_BRAKE_TICKS - 1U;
      output->mode = APP_MOTOR_OUTPUT_BRAKE;
    } else {
      output->mode = APP_MOTOR_OUTPUT_DRIVE;
      output->pwm = controller->snapshot.applied_pwm;
    }
    return true;
  }

  controller->snapshot.applied_pwm = AppMotorOpenLoop_MoveToward(
    controller->snapshot.applied_pwm,
    controller->snapshot.target_pwm,
    ramp_capacity);
  if (controller->snapshot.applied_pwm != 0) {
    output->mode = APP_MOTOR_OUTPUT_DRIVE;
    output->pwm = controller->snapshot.applied_pwm;
    return true;
  }

  output->mode = APP_MOTOR_OUTPUT_BRAKE;
  if (controller->snapshot.target_pwm == 0 &&
      controller->snapshot.state == APP_MOTOR_OPEN_LOOP_STOPPING) {
    controller->snapshot.state = controller->disarm_at_zero
                                   ? APP_MOTOR_OPEN_LOOP_DISARMED
                                   : APP_MOTOR_OPEN_LOOP_ARMED;
    controller->timeout_active = false;
    controller->disarm_at_zero = false;
    if (controller->snapshot.state == APP_MOTOR_OPEN_LOOP_DISARMED) {
      controller->snapshot.has_sequence = false;
      controller->reverse_brake_ticks_remaining = 0U;
    }
  }
  return true;
}

bool AppMotorOpenLoop_GetSnapshot(
  const AppMotorOpenLoop *controller,
  AppMotorOpenLoopSnapshot *snapshot)
{
  if (controller == NULL || snapshot == NULL) {
    return false;
  }
  *snapshot = controller->snapshot;
  return true;
}
