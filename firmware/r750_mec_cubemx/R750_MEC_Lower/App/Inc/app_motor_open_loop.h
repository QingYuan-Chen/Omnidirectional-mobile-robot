#ifndef APP_MOTOR_OPEN_LOOP_H
#define APP_MOTOR_OPEN_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/* 单电机试验状态机统一实现显式使能、斜坡、反转过零、超时停车和急停锁存。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 状态显式区分正常运行、可恢复禁止和必须复位的急停锁存。 */
typedef enum {
  APP_MOTOR_OPEN_LOOP_DISARMED = 0,
  APP_MOTOR_OPEN_LOOP_ARMED,
  APP_MOTOR_OPEN_LOOP_RUNNING,
  APP_MOTOR_OPEN_LOOP_STOPPING,
  APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE,
  APP_MOTOR_OPEN_LOOP_INHIBITED,
  APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED
} AppMotorOpenLoopState;

/* 控制任务每个节拍最多消费一个请求。 */
typedef enum {
  APP_MOTOR_REQUEST_NONE = 0,
  APP_MOTOR_REQUEST_ARM,
  APP_MOTOR_REQUEST_SET_PWM,
  APP_MOTOR_REQUEST_STOP,
  APP_MOTOR_REQUEST_ESTOP
} AppMotorRequestType;

/* 纯状态机输出的硬件动作，由任务编排层最终提交到 BSP。 */
typedef enum {
  APP_MOTOR_OUTPUT_NONE = 0,
  APP_MOTOR_OUTPUT_BRAKE,
  APP_MOTOR_OUTPUT_COAST,
  APP_MOTOR_OUTPUT_DRIVE
} AppMotorOutputMode;

/* 来自有序命令队列的规范化运动请求。 */
typedef struct {
  AppMotorRequestType type;
  uint32_t sequence;
  int32_t pwm;
} AppMotorOpenLoopRequest;

/* 可发布到遥测的状态机只读快照。 */
typedef struct {
  AppMotorOpenLoopState state;
  int16_t target_pwm;
  int16_t applied_pwm;
  uint32_t last_sequence;
  uint32_t command_timeout_count;
  uint32_t rejected_command_count;
  uint32_t clamped_command_count;
  uint32_t inhibit_transition_count;
  uint32_t estop_latch_count;
  bool has_sequence;
} AppMotorOpenLoopSnapshot;

/* 状态机内部时间锚点和过渡控制变量。 */
typedef struct {
  AppMotorOpenLoopSnapshot snapshot;
  uint32_t last_step_ms;
  uint32_t last_valid_command_ms;
  uint32_t reverse_brake_ticks_remaining;
  bool timeout_active;
  bool disarm_at_zero;
} AppMotorOpenLoop;

/* 当前周期应执行的电机模式和 PWM，急停请求具有最高优先级。 */
typedef struct {
  AppMotorOutputMode mode;
  int16_t pwm;
  bool emergency_coast_request;
} AppMotorOpenLoopOutput;

/* 初始化为未使能状态并建立命令超时时间基线。 */
void AppMotorOpenLoop_Init(AppMotorOpenLoop *controller, uint32_t now_ms);
/* 推进一步状态机，融合命令、安全门、斜坡、换向和超时规则。 */
bool AppMotorOpenLoop_Step(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  bool motion_inhibited,
  bool hard_fault_latched,
  const AppMotorOpenLoopRequest *request,
  AppMotorOpenLoopOutput *output);
/* 复制当前状态机快照。 */
bool AppMotorOpenLoop_GetSnapshot(
  const AppMotorOpenLoop *controller,
  AppMotorOpenLoopSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
