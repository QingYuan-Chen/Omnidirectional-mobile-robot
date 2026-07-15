#ifndef APP_IMU_H
#define APP_IMU_H

#include "bsp_imu.h"
#include "bsp_types.h"

/* 原始物理量、独立低通输出和姿态估计结果分别保存，低通结果不回灌 ESKF。 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_IMU_FLAG_SENSOR_PRESENT = (1UL << 0U),
  APP_IMU_FLAG_CALIBRATED = (1UL << 1U),
  APP_IMU_FLAG_DATA_VALID = (1UL << 2U),
  APP_IMU_FLAG_FILTER_INITIALIZED = (1UL << 3U),
  APP_IMU_FLAG_FILTER_CONVERGED = (1UL << 4U),
  APP_IMU_FLAG_ACCEL_UPDATE_USED = (1UL << 5U),
  APP_IMU_FLAG_VIBRATION_HIGH = (1UL << 6U),
  APP_IMU_FLAG_SENSOR_FAULT = (1UL << 7U),
  APP_IMU_FLAG_DATA_STALE = (1UL << 8U),
  APP_IMU_FLAG_ABSOLUTE_YAW_VALID = (1UL << 9U),
  APP_IMU_FLAG_TIMESTAMP_VALID = (1UL << 10U),
  APP_IMU_FLAG_TILT_VALID = (1UL << 11U),
  APP_IMU_FLAG_SAMPLE_SPIKE = (1UL << 12U),
  APP_IMU_FLAG_SENSOR_DEGRADED = (1UL << 13U),
  APP_IMU_FLAG_RECOVERING = (1UL << 14U),
  APP_IMU_FLAG_ESTIMATOR_FAULT = (1UL << 15U)
} AppImuFlags;

typedef enum {
  APP_IMU_HEALTH_UNINITIALIZED = 0,
  APP_IMU_HEALTH_HEALTHY,
  APP_IMU_HEALTH_TRANSIENT_DEGRADED,
  APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT,
  APP_IMU_HEALTH_RECOVERING,
  APP_IMU_HEALTH_ESTIMATOR_FAULT
} AppImuHealth;

typedef struct {
  BspImuSample raw_sample;
  float acceleration_mps2[3];
  float angular_rate_rad_s[3];
  float filtered_acceleration_mps2[3];
  float filtered_angular_rate_rad_s[3];
  float quaternion[4];
  float euler_rad[3];
  float gyro_bias_rad_s[3];
  float temperature_celsius;
  AppImuHealth health;
  uint32_t flags;
  uint32_t sequence;
  uint32_t sensor_timestamp;
  uint32_t host_tick_ms;
  uint32_t last_good_tick_ms;
  uint32_t sample_age_ms;
  uint32_t read_error_count;
  uint32_t consecutive_error_count;
  uint32_t backoff_count;
  uint32_t retry_delay_ms;
  uint32_t next_retry_tick_ms;
  uint32_t duplicate_count;
  uint32_t dropped_sample_count;
  uint32_t accel_update_accept_count;
  uint32_t accel_update_reject_count;
  uint32_t spike_reject_count;
  uint32_t consecutive_spike_count;
  uint32_t stable_sample_count;
  uint32_t estimator_fault_count;
} AppImuOutput;

BspStatus AppImu_Calibrate(void);
BspStatus AppImu_Process(uint32_t now_ms, AppImuOutput *output);

#ifdef __cplusplus
}
#endif

#endif
