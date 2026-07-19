#ifndef APP_WHEEL_SPEED_ESTIMATOR_H
#define APP_WHEEL_SPEED_ESTIMATOR_H

#include "app_encoder_period_accumulator.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * G3 固定窗口 M 法、边沿周期 T 法和带滞回硬切换候选。
 *
 * 本模块只消费 1 kHz 编码器增量与边沿聚合快照，不访问 HAL/RTOS。候选阈值由配置传入，
 * 便于主机重放后再冻结；当前实现不是 G5 已验收速度环。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define APP_WHEEL_SPEED_M_WINDOW_MAX_TICKS (50U)

typedef enum {
  APP_WHEEL_SPEED_SOURCE_NONE = 0,
  APP_WHEEL_SPEED_SOURCE_M,
  APP_WHEEL_SPEED_SOURCE_T
} AppWheelSpeedSource;

typedef struct {
  uint32_t encoder_counts_per_revolution;
  uint32_t control_rate_hz;
  uint32_t timestamp_clock_hz;
  uint32_t period_events_per_revolution;
  uint32_t t_stale_timeout_cycles;
  uint16_t switch_to_t_max_activity_counts;
  uint16_t switch_to_m_min_activity_counts;
  uint8_t m_window_ticks;
} AppWheelSpeedEstimatorConfig;

typedef struct {
  int16_t encoder_delta;
  AppEncoderPeriodSnapshot period;
} AppWheelSpeedEstimatorInput;

typedef struct {
  float speed_rpm;
  float m_speed_rpm;
  float t_speed_rpm;
  uint32_t m_activity_counts;
  AppWheelSpeedSource source;
  bool valid;
  bool m_valid;
  bool t_valid;
  bool t_updated;
  bool t_stale;
} AppWheelSpeedEstimatorOutput;

typedef struct {
  AppWheelSpeedEstimatorConfig config;
  int16_t m_delta_ring[APP_WHEEL_SPEED_M_WINDOW_MAX_TICKS];
  int32_t m_delta_sum;
  uint32_t m_activity_sum;
  float last_t_speed_rpm;
  uint8_t m_ring_index;
  uint8_t m_fill_count;
  AppWheelSpeedSource source;
  bool t_valid;
} AppWheelSpeedEstimator;

bool AppWheelSpeedEstimator_Init(
  AppWheelSpeedEstimator *estimator,
  const AppWheelSpeedEstimatorConfig *config);

bool AppWheelSpeedEstimator_Update(
  AppWheelSpeedEstimator *estimator,
  const AppWheelSpeedEstimatorInput *input,
  AppWheelSpeedEstimatorOutput *output);

#ifdef __cplusplus
}
#endif

#endif
