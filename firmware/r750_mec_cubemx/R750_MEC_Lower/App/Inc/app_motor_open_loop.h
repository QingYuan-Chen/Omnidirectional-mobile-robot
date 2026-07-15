#ifndef APP_MOTOR_OPEN_LOOP_H
#define APP_MOTOR_OPEN_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/* 单电机试验状态机统一实现显式使能、斜坡、反转过零、超时停车和急停锁存。 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_MOTOR_OPEN_LOOP_DISARMED = 0,
  APP_MOTOR_OPEN_LOOP_ARMED,
  APP_MOTOR_OPEN_LOOP_RUNNING,
  APP_MOTOR_OPEN_LOOP_STOPPING,
  APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE,
  APP_MOTOR_OPEN_LOOP_INHIBITED,
  APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED
} AppMotorOpenLoopState;

typedef enum {
  APP_MOTOR_REQUEST_NONE = 0,
  APP_MOTOR_REQUEST_ARM,
  APP_MOTOR_REQUEST_SET_PWM,
  APP_MOTOR_REQUEST_STOP,
  APP_MOTOR_REQUEST_ESTOP
} AppMotorRequestType;

typedef enum {
  APP_MOTOR_OUTPUT_NONE = 0,
  APP_MOTOR_OUTPUT_BRAKE,
  APP_MOTOR_OUTPUT_COAST,
  APP_MOTOR_OUTPUT_DRIVE
} AppMotorOutputMode;

typedef struct {
  AppMotorRequestType type;
  uint32_t sequence;
  int32_t pwm;
} AppMotorOpenLoopRequest;

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

typedef struct {
  AppMotorOpenLoopSnapshot snapshot;
  uint32_t last_step_ms;
  uint32_t last_valid_command_ms;
  uint32_t reverse_brake_ticks_remaining;
  bool timeout_active;
  bool disarm_at_zero;
} AppMotorOpenLoop;

typedef struct {
  AppMotorOutputMode mode;
  int16_t pwm;
  bool emergency_coast_request;
} AppMotorOpenLoopOutput;

void AppMotorOpenLoop_Init(AppMotorOpenLoop *controller, uint32_t now_ms);
bool AppMotorOpenLoop_Step(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  bool motion_inhibited,
  bool hard_fault_latched,
  const AppMotorOpenLoopRequest *request,
  AppMotorOpenLoopOutput *output);
bool AppMotorOpenLoop_GetSnapshot(
  const AppMotorOpenLoop *controller,
  AppMotorOpenLoopSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
