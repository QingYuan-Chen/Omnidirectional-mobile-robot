#ifndef APP_IMU_H
#define APP_IMU_H

#include "bsp_imu.h"
#include "bsp_types.h"

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
  APP_IMU_FLAG_SAMPLE_SPIKE = (1UL << 12U)
} AppImuFlags;

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
  uint32_t flags;
  uint32_t sequence;
  uint32_t sensor_timestamp;
  uint32_t host_tick_ms;
  uint32_t last_good_tick_ms;
  uint32_t sample_age_ms;
  uint32_t read_error_count;
  uint32_t consecutive_error_count;
  uint32_t duplicate_count;
  uint32_t dropped_sample_count;
  uint32_t accel_update_accept_count;
  uint32_t accel_update_reject_count;
  uint32_t spike_reject_count;
  uint32_t consecutive_spike_count;
} AppImuOutput;

BspStatus AppImu_Calibrate(void);
BspStatus AppImu_Process(uint32_t now_ms, AppImuOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_H */
