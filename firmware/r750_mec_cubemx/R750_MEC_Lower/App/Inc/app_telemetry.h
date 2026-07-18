#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include "app_control_timing.h"
#include "app_imu.h"
#include "app_motor_open_loop.h"
#include "bsp_types.h"

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
  bool critical_tasks_alive;
  bool runtime_ready;
  bool motion_inhibited;
  bool fault_latched;
} AppTelemetryInput;

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

#ifdef __cplusplus
}
#endif

#endif
