#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include "app_control_timing.h"
#include "app_imu.h"
#include "app_motor_open_loop.h"
#include "bsp_types.h"

#include <stdbool.h>
#include <stdint.h>

/* 遥测采用定长缓冲区和整数格式化，禁止动态分配及浮点文本格式化。 */

#ifdef __cplusplus
extern "C" {
#endif

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
  uint32_t adc_error_count;
  bool critical_tasks_alive;
  bool motion_inhibited;
  bool fault_latched;
} AppTelemetryInput;

bool AppTelemetry_Format(
  const AppTelemetryInput *input,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);

#ifdef __cplusplus
}
#endif

#endif
