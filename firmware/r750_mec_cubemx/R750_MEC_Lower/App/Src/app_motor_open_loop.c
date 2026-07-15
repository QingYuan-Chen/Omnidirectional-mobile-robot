#include "app_motor_open_loop.h"

#include "robot_config.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * 与硬件无关的 MA 开环状态机。
 *
 * 该模块只根据输入状态计算下一状态和建议输出，不包含 HAL/RTOS 调用，因此可在主机端
 * 穷举命令时序。安全优先级固定为“硬故障或急停→可恢复运动禁止→普通命令→超时→
 * 斜坡与换向”。所有输出仍需任务级最终安全门复核后才能写入电机 BSP。
 */

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
  /* 诊断计数饱和后保持最大值，防止长期运行回绕掩盖历史异常。 */
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static bool AppMotorOpenLoop_SequenceIsNewer(uint32_t sequence, uint32_t previous)
{
  /*
   * 与协议层使用相同的模 2^32 半区间规则。即使命令绕过或破坏通信层，执行状态机仍会
   * 独立拒绝重复/旧序号；两层规则必须保持一致，否则回绕附近可能出现解析接受但执行拒绝。
   */
  const uint32_t difference = sequence - previous;
  return difference != 0U && difference <= (uint32_t)INT32_MAX;
}

static int16_t AppMotorOpenLoop_ClampPwm(AppMotorOpenLoop *controller, int32_t pwm)
{
  /* 状态机再次限幅，不信任协议层已经检查过范围；被限幅命令仍可执行但会留下诊断计数。 */
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
  /* 按实际毫秒间隔计算本周期最大变化量，使偶发调度延迟不会改变长期斜坡速率。 */
  if (elapsed_ms > (UINT32_MAX / ROBOT_CONFIG_MA_PWM_RAMP_COUNTS_PER_MS)) {
    return UINT32_MAX;
  }
  return elapsed_ms * ROBOT_CONFIG_MA_PWM_RAMP_COUNTS_PER_MS;
}

static int16_t AppMotorOpenLoop_MoveToward(int16_t current, int16_t target, uint32_t capacity)
{
  /* 单调靠近目标且不越过目标，32 位中间值避免 int16_t 相减溢出。 */
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

  /*
   * 急停锁存只在首次进入时计数并发出一次 emergency_coast_request。后续 hard_fault 输入
   * 仍会进入本函数，但不再发出紧急请求，并保留 Step 入口设置的零 PWM/BRAKE 默认输出；
   * PWM 基础设施已关闭，普通制动写入不会恢复驱动。任何 ARM/STOP/PWM 都不能解除锁存。
   */
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
  /* 默认输出为零主动制动，确保任何未覆盖的普通状态都不会意外保留上一周期 PWM。 */
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
    /*
     * 可恢复禁止立即空转并清除目标、实际输出、序号和超时上下文。安全门恢复时只退回
     * DISARMED，操作者必须发送全新序号的 ARM，旧 PWM 不会自动恢复。
     */
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
    /*
     * 先消费合法新序号，再判断命令在当前状态下是否允许。这样发送端不能反复重放一条
     * “序号有效但状态不允许”的命令，等待状态变化后让它意外生效。
     */
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
      /* ARM 只允许从 DISARMED 进入零输出 ARMED，不隐含任何 PWM 目标。 */
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
      /* 超时停车或计划在零点撤销使能时拒绝新 PWM，避免停车过程被中途抢占。 */
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
      /* 主动 STOP 斜坡到零后仍保持 ARMED；只有超时路径设置 disarm_at_zero。 */
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
    /*
     * 仅 ARMED、RUNNING、REVERSING_BRAKE 需要命令保活。超时后进入不可取消的降零过程，
     * 到零自动 DISARMED 并清除序号基线，必须重新 ARM 才能恢复。
     */
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
    /*
     * 检测到目标与实际符号相反时，当前周期只向零斜坡，不直接向反向目标移动。到零后
     * 进入 REVERSING_BRAKE，并按完整控制节拍保持主动制动，再恢复 RUNNING 加速到反向。
     */
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
  /* 零点收尾统一决定回到 ARMED 还是 DISARMED，避免 STOP、超时和换向各自重复清理状态。 */
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
