#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "app_comm_protocol.h"
#include "app_control_timing.h"
#include "app_imu.h"
#include "app_motion_gate.h"
#include "app_motor_open_loop.h"
#include "bsp_types.h"
#include "bsp_uart.h"

/*
 * FreeRTOS/CMSIS-RTOS 任务编排与跨模块安全仲裁层。
 *
 * 本模块创建控制、安全、通信、IMU、监控五个长期任务，规定优先级、心跳、命令队列和
 * 共享快照所有权。控制任务独占电机状态机和确定性 TIM7 节拍；通信任务独占协议解析器
 * 与电池采样；IMU 任务独占估计器；安全任务是运动禁止和故障锁存的常规策略写入者，
 * 通信急停与任一任务的统一故障收敛路径也可紧急置位；监控任务只采集栈余量。跨任务
 * 字段组通过短临界区复制，任何执行器提交都必须重新读取最终安全门。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 固定任务编号，同时作为任务句柄、任务属性和 stack_free_bytes 数组索引。 */
typedef enum {
  APP_TASK_CONTROL = 0,
  APP_TASK_SAFETY,
  APP_TASK_COMM,
  APP_TASK_IMU,
  APP_TASK_MONITOR,
  APP_TASK_COUNT
} AppTaskId;

/*
 * 通信任务拥有的累计诊断快照。
 * protocol 和 debug_uart 分别来自解析器与当前配置的调试 UART；queue_drop 表示运动
 * 命令未进入控制队列；telemetry_enqueue_drop 表示完整遥测帧未进入发送队列；
 * format_error 表示本地缓冲容量不足或参数异常；motion_gate_reject 表示普通命令在
 * 运行许可关闭时被前置拒绝，
 * estop_command_count 记录绕过普通队列的急停命令。
 */
typedef struct {
  AppCommProtocolStats protocol;
  BspUartStats debug_uart;
  uint32_t command_queue_drop_count;
  uint32_t telemetry_enqueued_count;
  uint32_t telemetry_enqueue_drop_count;
  uint32_t telemetry_format_error_count;
  uint32_t adc_error_count;
  uint32_t motion_gate_reject_count;
  uint32_t estop_command_count;
  uint32_t capture_command_reject_count;
  uint32_t capture_event_drop_count;
  uint32_t capture_export_count;
  uint32_t capture_export_error_count;
} AppCommRuntimeSnapshot;

/*
 * 整机运行快照，是应用层向启动检查、遥测和诊断提供的唯一聚合视图。
 *
 * 常规数据字段组只有一个主要写入者：控制任务写编码器、时序和电机状态；通信任务写
 * 通信与电池；IMU 任务写 imu；监控任务写栈余量。安全状态允许通信 ESTOP 和统一失败
 * 路径紧急置位；motion_gate 是明确的多写入者仲裁字段，只能在调度器挂起或最终失败
 * 临界区内更新。读取者通过 AppTasks_GetSnapshot 获得独立副本，禁止持有内部地址。
 */
typedef struct {
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
  int16_t encoder_delta[BSP_MOTOR_COUNT];
  int64_t encoder_total[BSP_MOTOR_COUNT];
  AppControlTimingSnapshot control_timing;
  AppMotorOpenLoopSnapshot motor_open_loop;
  AppCommRuntimeSnapshot communication;
  AppImuOutput imu;
  uint16_t battery_millivolts;
  uint32_t health_miss_count;
  uint32_t invalidated_motor_command_count;
  uint32_t stack_free_bytes[APP_TASK_COUNT];
  AppMotionGate motion_gate;
  bool critical_tasks_alive;
  bool runtime_ready;
  bool motion_inhibited;
  bool fault_latched;
} AppRuntimeSnapshot;

/*
 * 创建事件标志、运动命令队列和五个任务。
 * 初始快照强制禁止运动；只有全部对象和任务创建成功后才设置统一启动位。任何中途失败
 * 都终止已创建任务并删除对象，返回 BSP_ERROR；重复调用返回 BSP_BUSY。
 */
BspStatus AppTasks_Create(void);
/*
 * 在短临界区内复制完整运行快照。
 * 成功返回 BSP_OK，参数为空返回 BSP_INVALID_ARG。返回后调用者拥有独立副本，可在临界区
 * 外格式化或分析，不得把其中状态反向写回作为控制输入。
 */
BspStatus AppTasks_GetSnapshot(AppRuntimeSnapshot *snapshot);
/*
 * 尝试锁存本次上电的运行就绪状态。
 * 只有最近安全检查窗确认关键任务心跳完整、实时 IMU 运动判据通过且没有硬故障时返回
 * BSP_OK；条件尚未满足返回 BSP_BUSY，已锁存故障或内部许可代际异常返回 BSP_ERROR。
 * runtime_ready 一旦成功置位，本次上电不再清零；动态运动许可仍由安全与 IMU 门持续控制。
 */
BspStatus AppTasks_TrySetRuntimeReady(void);

#ifdef __cplusplus
}
#endif

#endif
