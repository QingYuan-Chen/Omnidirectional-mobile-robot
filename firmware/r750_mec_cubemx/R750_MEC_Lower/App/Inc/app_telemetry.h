#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include "app_control_timing.h"
#include "app_imu.h"
#include "app_motor_open_loop.h"
#include "bsp_types.h"
#include "bsp_uart.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 面向 UART4 临时试验链路的长度有界文本遥测格式化器。
 *
 * 模块只把调用者提供的一致快照编码到调用者缓冲区，不读取全局状态、不发送串口、
 * 不分配动态内存，也不引入 printf 或浮点文本格式化。帧以换行结束，字段组和顺序固定，
 * 正式层级由树莓派直接解析，电脑调参端经树莓派转发；临时板测直连不改变系统层级。
 * 本协议当前用于板级调试，不等同于后续 ROS2 正式协议。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define APP_TELEMETRY_SCHEMA_VERSION (1U)

typedef enum {
  APP_TELEMETRY_FRAME_STAT = 0,
  APP_TELEMETRY_FRAME_IMUQ,
  APP_TELEMETRY_FRAME_RES,
  APP_TELEMETRY_FRAME_EVENT
} AppTelemetryFrameType;

typedef enum {
  APP_TELEMETRY_EVENT_BOOT = (1UL << 0U),
  APP_TELEMETRY_EVENT_READY_CHANGED = (1UL << 1U),
  APP_TELEMETRY_EVENT_INHIBIT_CHANGED = (1UL << 2U),
  APP_TELEMETRY_EVENT_FAULT_CHANGED = (1UL << 3U),
  APP_TELEMETRY_EVENT_MOTOR_STATE_CHANGED = (1UL << 4U),
  APP_TELEMETRY_EVENT_IMU_HEALTH_CHANGED = (1UL << 5U),
  APP_TELEMETRY_EVENT_UART_ERROR_CHANGED = (1UL << 6U),
  APP_TELEMETRY_EVENT_DROP_CHANGED = (1UL << 7U),
  APP_TELEMETRY_EVENT_ADC_ERROR_CHANGED = (1UL << 8U),
  APP_TELEMETRY_EVENT_INVALIDATED_COMMAND_CHANGED = (1UL << 9U),
  APP_TELEMETRY_EVENT_ESTOP_CHANGED = (1UL << 10U)
} AppTelemetryEventFlags;

/*
 * 单帧遥测所需的只读字段集合。
 * 通信任务应先复制 AppRuntimeSnapshot，再在临界区外构造本结构，确保编码期间不会引用
 * 正在变化的共享数据。cycle 类时序量保留 CPU 周期单位；电池使用毫伏；IMU 年龄使用
 * 毫秒；布尔量在文本帧中编码为 0 或 1。
 */
typedef struct {
  uint32_t now_ms;
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
  int16_t encoder_delta[BSP_MOTOR_COUNT];
  int64_t encoder_total[BSP_MOTOR_COUNT];
  AppControlTimingSnapshot control_timing;
  AppMotorOpenLoopSnapshot motor;
  uint16_t battery_millivolts;
  uint32_t imu_sample_age_ms;
  AppImuHealth imu_health;
  uint32_t uart_error_count;
  uint32_t uart_rx_overflow_count;
  uint32_t uart_tx_fault_count;
  uint32_t command_reject_count;
  uint32_t command_queue_drop_count;
  uint32_t motion_gate_reject_count;
  uint32_t invalidated_motor_command_count;
  uint32_t adc_error_count;
  uint32_t telemetry_enqueued_count;
  uint32_t telemetry_enqueue_drop_count;
  uint32_t telemetry_format_error_count;
  uint32_t telemetry_frame_failure_count[4];
  uint32_t capture_event_drop_count;
  uint32_t capture_export_error_count;
  uint32_t health_miss_count;
  uint32_t stack_free_bytes[5];
  uint32_t minimum_free_heap_bytes;
  BspUartStats uart;
  AppImuOutput imu;
  bool critical_tasks_alive;
  bool runtime_ready;
  bool motion_inhibited;
  bool fault_latched;
} AppTelemetryInput;

typedef struct {
  uint32_t last_stat_ms;
  uint32_t last_imuq_ms;
  uint32_t last_res_ms;
  uint32_t last_event_ms;
  uint32_t event_sequence;
  uint32_t pending_event_flags;
  uint32_t previous_uart_error_count;
  uint32_t previous_telemetry_drop_count;
  uint32_t previous_telemetry_format_error_count;
  uint32_t previous_adc_error_count;
  uint32_t previous_invalidated_motor_command_count;
  AppMotorOpenLoopState previous_motor_state;
  AppImuHealth previous_imu_health;
  bool previous_runtime_ready;
  bool previous_motion_inhibited;
  bool previous_fault_latched;
  bool previous_estop;
  bool event_baseline_valid;
} AppTelemetrySchedule;

/*
 * 把 input 编码成一帧以 '\n' 结尾的文本。
 * capacity 包含可写数据空间；成功时 length 返回实际帧长，若仍有剩余空间会额外写入
 * 一个零终止符，但零终止符不计入 length。参数无效或任一字段导致容量不足时返回 false，
 * length 保持为 0，调用者不得发送缓冲区中的部分帧。
 */
bool AppTelemetry_Format(
  const AppTelemetryInput *input,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);

/*
 * 初始化分类型遥测调度器。初始化后 STAT/IMUQ/RES 均立即到期，EVENT 先挂起一次 BOOT
 * 事件。时间比较全部使用无符号减法，允许 HAL 毫秒计数自然回绕。
 */
void AppTelemetrySchedule_Init(
  AppTelemetrySchedule *schedule,
  uint32_t now_ms);
/* 比较安全状态与累计错误计数，把变化合并到待发 EVENT；待发位不会因后续变化而丢失。 */
void AppTelemetrySchedule_UpdateEvents(
  AppTelemetrySchedule *schedule,
  const AppTelemetryInput *input);
/*
 * 每次通信循环最多选择一帧。优先级为 EVENT、强制 STAT、周期 STAT、IMUQ、RES；
 * force_stat 只影响本次选择，不绕过 EVENT 的防突发节流。
 */
bool AppTelemetrySchedule_Select(
  const AppTelemetrySchedule *schedule,
  uint32_t now_ms,
  bool force_stat,
  AppTelemetryFrameType *frame_type);
/*
 * 记录一次格式化/入队尝试。周期帧无论成功与否都推进节拍以避免故障洪泛；EVENT 只有
 * 成功入队才清除待发位并递增序号，失败后按最小间隔重试。
 */
void AppTelemetrySchedule_MarkAttempt(
  AppTelemetrySchedule *schedule,
  AppTelemetryFrameType frame_type,
  uint32_t now_ms,
  bool enqueued);
/* 按 schema 1 编码 STAT/IMUQ/RES/EVENT；EVENT 使用调度器当前序号和待发位。 */
bool AppTelemetry_FormatTyped(
  AppTelemetryFrameType frame_type,
  const AppTelemetrySchedule *schedule,
  const AppTelemetryInput *input,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);

#ifdef __cplusplus
}
#endif

#endif
