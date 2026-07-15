#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "app_comm_protocol.h"
#include "app_control_timing.h"
#include "app_imu.h"
#include "app_motor_open_loop.h"
#include "bsp_types.h"
#include "bsp_uart.h"

/* 任务编排层集中管理控制、安全、通信、IMU和监控任务及其共享运行快照。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 固定任务编号同时作为任务句柄、栈余量数组的索引。 */
typedef enum {
  APP_TASK_CONTROL = 0,
  APP_TASK_SAFETY,
  APP_TASK_COMM,
  APP_TASK_IMU,
  APP_TASK_MONITOR,
  APP_TASK_COUNT
} AppTaskId;

/* 通信任务累计的协议、串口、遥测和 ADC 诊断。 */
typedef struct {
  AppCommProtocolStats protocol;
  BspUartStats uart_ttl;
  uint32_t command_queue_drop_count;
  uint32_t telemetry_enqueued_count;
  uint32_t telemetry_enqueue_drop_count;
  uint32_t telemetry_format_error_count;
  uint32_t adc_error_count;
  uint32_t estop_command_count;
} AppCommRuntimeSnapshot;

/* 跨任务发布的整机运行快照；写入时使用短临界区保证字段组一致。 */
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
  uint32_t stack_free_bytes[APP_TASK_COUNT];
  bool critical_tasks_alive;
  bool motion_inhibited;
  bool fault_latched;
} AppRuntimeSnapshot;

/* 创建全部同步对象和任务，全部成功后再统一发布启动事件。 */
BspStatus AppTasks_Create(void);
/* 在临界区内复制运行快照，调用者获得独立副本。 */
BspStatus AppTasks_GetSnapshot(AppRuntimeSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
