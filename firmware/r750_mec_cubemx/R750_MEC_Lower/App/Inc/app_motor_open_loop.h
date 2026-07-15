#ifndef APP_MOTOR_OPEN_LOOP_H
#define APP_MOTOR_OPEN_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/*
 * MA 单电机开环试验的纯状态机。
 *
 * 本模块不直接访问 PWM 硬件，而是把有序运动请求、安全门状态和毫秒时基转换成明确的
 * 制动、空转或驱动输出。状态机强制执行“先 ARM、再给 PWM”、斜坡限速、换向先过零并
 * 制动、命令超时停车、可恢复禁止后重新 ARM，以及急停只可复位恢复。控制任务仍需在
 * 最终安全仲裁区内把输出提交给 BSP，不能把本模块当作安全层的替代品。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 状态机状态。
 * DISARMED 不接受 PWM；ARMED 已使能但目标为零；RUNNING 正在跟踪非零目标；
 * STOPPING 正在斜坡降零；REVERSING_BRAKE 位于换向过零后的强制定时制动；
 * INHIBITED 由可恢复安全门触发；ESTOP_LATCHED 为本次上电不可解除的硬锁存。
 */
typedef enum {
  APP_MOTOR_OPEN_LOOP_DISARMED = 0,
  APP_MOTOR_OPEN_LOOP_ARMED,
  APP_MOTOR_OPEN_LOOP_RUNNING,
  APP_MOTOR_OPEN_LOOP_STOPPING,
  APP_MOTOR_OPEN_LOOP_REVERSING_BRAKE,
  APP_MOTOR_OPEN_LOOP_INHIBITED,
  APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED
} AppMotorOpenLoopState;

/* 控制任务每个 TIM7 节拍最多传入一个请求，NONE 表示本周期仅推进状态机。 */
typedef enum {
  APP_MOTOR_REQUEST_NONE = 0,
  APP_MOTOR_REQUEST_ARM,
  APP_MOTOR_REQUEST_SET_PWM,
  APP_MOTOR_REQUEST_STOP,
  APP_MOTOR_REQUEST_ESTOP
} AppMotorRequestType;

/*
 * 状态机建议的硬件模式。
 * NONE 只在紧急空转已经单独请求或锁存后使用；BRAKE/COAST/DRIVE 分别映射到 BSP 的
 * 主动制动、自由空转和有符号 PWM。任务层必须对未试验的 MB～MD 再次强制空转。
 */
typedef enum {
  APP_MOTOR_OUTPUT_NONE = 0,
  APP_MOTOR_OUTPUT_BRAKE,
  APP_MOTOR_OUTPUT_COAST,
  APP_MOTOR_OUTPUT_DRIVE
} AppMotorOutputMode;

/*
 * 从通信任务队列复制来的规范化请求。
 * sequence 仅对 ARM、SET_PWM、STOP 有意义；pwm 仅对 SET_PWM 有意义。结构体不保存
 * 外部指针，可由 CMSIS-RTOS 消息队列按值传递。
 */
typedef struct {
  AppMotorRequestType type;
  uint32_t sequence;
  int32_t pwm;
} AppMotorOpenLoopRequest;

/*
 * 可发布的状态机快照。
 * target_pwm 是命令目标，applied_pwm 是斜坡与换向约束后的实际输出；last_sequence 只在
 * has_sequence 为真时有效。各类计数采用饱和累计，用于区分超时、状态拒绝、限幅、
 * 安全禁止和急停锁存。
 */
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

/*
 * 状态机私有上下文，由控制任务独占。
 * last_step_ms 决定本周期允许变化的 PWM 量；last_valid_command_ms 驱动命令看门狗；
 * reverse_brake_ticks_remaining 按控制节拍计数；timeout_active 和 disarm_at_zero 防止
 * 超时停车过程中被新 PWM 抢占。
 */
typedef struct {
  AppMotorOpenLoopSnapshot snapshot;
  uint32_t last_step_ms;
  uint32_t last_valid_command_ms;
  uint32_t reverse_brake_ticks_remaining;
  bool timeout_active;
  bool disarm_at_zero;
} AppMotorOpenLoop;

/*
 * 单周期输出。
 * emergency_coast_request 为真时优先级高于 mode，任务层应立即调用复位恢复型紧急空转；
 * DRIVE 时 pwm 才有效，其他模式下应忽略 pwm。
 */
typedef struct {
  AppMotorOutputMode mode;
  int16_t pwm;
  bool emergency_coast_request;
} AppMotorOpenLoopOutput;

/*
 * 清零上下文并进入 DISARMED，使用 now_ms 建立斜坡和命令超时基线。
 * 参数为空时不执行操作；初始化不会产生任何硬件输出。
 */
void AppMotorOpenLoop_Init(AppMotorOpenLoop *controller, uint32_t now_ms);
/*
 * 推进一步状态机并生成本周期输出。
 * hard_fault_latched 或 ESTOP 请求拥有最高优先级；motion_inhibited 次之并进入可恢复禁止；
 * 之后才处理有序普通命令和超时。now_ms 允许 uint32_t 自然回绕。成功返回 true；任一
 * 必需指针为空返回 false。函数不阻塞、不分配内存，也不操作电机寄存器。
 */
bool AppMotorOpenLoop_Step(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  bool motion_inhibited,
  bool hard_fault_latched,
  const AppMotorOpenLoopRequest *request,
  AppMotorOpenLoopOutput *output);
/* 复制只读快照；成功返回 true，任一指针为空返回 false。 */
bool AppMotorOpenLoop_GetSnapshot(
  const AppMotorOpenLoop *controller,
  AppMotorOpenLoopSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
