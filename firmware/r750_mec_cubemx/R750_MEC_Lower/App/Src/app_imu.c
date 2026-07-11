#include "app_imu.h"

#include "cmsis_os.h"
#include "imu_eskf.h"
#include "main.h"
#include "robot_config.h"

#include <math.h>
#include <string.h>

#define APP_IMU_PI                         (3.14159265358979323846f)
#define APP_IMU_TIMESTAMP_MASK             (0x00FFFFFFUL)
#define APP_IMU_MAX_CALIBRATION_READ_ERRORS (20U)
#define APP_IMU_CALIBRATION_DISCARD_SAMPLES  (3U)
#define APP_IMU_MAX_TIMESTAMP_STEP         (11U)
#define APP_IMU_MAX_FILTER_DELTA_TIME_S    (0.05f)
#define APP_IMU_CALIBRATION_ACCEL_NORM_TOLERANCE (0.15f * ROBOT_CONFIG_STANDARD_GRAVITY_MPS2)
#define APP_IMU_CALIBRATION_ACCEL_STD_LIMIT      (0.02f * ROBOT_CONFIG_STANDARD_GRAVITY_MPS2)
#define APP_IMU_CALIBRATION_GYRO_NORM_LIMIT      (3.0f * APP_IMU_PI / 180.0f)
#define APP_IMU_CALIBRATION_GYRO_STD_LIMIT       (0.2f * APP_IMU_PI / 180.0f)

static ImuEskf imu_filter;
static AppImuOutput imu_output;
static bool imu_calibrated;
static uint32_t previous_sensor_timestamp;
static float previous_acceleration_mps2[3];
static float previous_angular_rate_rad_s[3];

static uint32_t AppImu_SaturatingIncrement(uint32_t value)
{
  return value < UINT32_MAX ? value + 1U : UINT32_MAX;
}

static float AppImu_VectorNorm(const float vector[3])
{
  return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

static uint32_t AppImu_SaturatingAdd(uint32_t value, uint32_t increment)
{
  return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

static bool AppImu_IsSpike(const float acceleration_mps2[3], const float angular_rate_rad_s[3])
{
  float acceleration_delta[3];
  float angular_rate_delta[3];
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    acceleration_delta[axis] = acceleration_mps2[axis] - previous_acceleration_mps2[axis];
    angular_rate_delta[axis] = angular_rate_rad_s[axis] - previous_angular_rate_rad_s[axis];
  }

  return AppImu_VectorNorm(acceleration_delta) > ROBOT_CONFIG_IMU_ACCEL_SPIKE_LIMIT_MPS2 ||
         AppImu_VectorNorm(angular_rate_delta) > ROBOT_CONFIG_IMU_GYRO_SPIKE_LIMIT_RAD_S;
}

static void AppImu_UpdateFilteredOutput(const float acceleration_mps2[3],
                                        const float angular_rate_rad_s[3],
                                        float delta_time_s)
{
  const float time_constant_s = 1.0f / (2.0f * APP_IMU_PI * ROBOT_CONFIG_IMU_OUTPUT_FILTER_CUTOFF_HZ);
  const float alpha = delta_time_s / (time_constant_s + delta_time_s);
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    imu_output.filtered_acceleration_mps2[axis] +=
      alpha * (acceleration_mps2[axis] - imu_output.filtered_acceleration_mps2[axis]);
    imu_output.filtered_angular_rate_rad_s[axis] +=
      alpha * (angular_rate_rad_s[axis] - imu_output.filtered_angular_rate_rad_s[axis]);
  }
}

static void AppImu_ConvertToBodySi(const BspImuSample *sample,
                                   float acceleration_mps2[3],
                                   float angular_rate_rad_s[3])
{
  const float acceleration_scale =
    ROBOT_CONFIG_STANDARD_GRAVITY_MPS2 / ROBOT_CONFIG_IMU_ACCEL_LSB_PER_G;
  const float angular_rate_scale = APP_IMU_PI / (180.0f * ROBOT_CONFIG_IMU_GYRO_LSB_PER_DPS);

  acceleration_mps2[0] = (float)sample->acceleration[1] * acceleration_scale;
  acceleration_mps2[1] = -(float)sample->acceleration[0] * acceleration_scale;
  acceleration_mps2[2] = (float)sample->acceleration[2] * acceleration_scale;
  angular_rate_rad_s[0] = (float)sample->angular_rate[1] * angular_rate_scale;
  angular_rate_rad_s[1] = -(float)sample->angular_rate[0] * angular_rate_scale;
  angular_rate_rad_s[2] = (float)sample->angular_rate[2] * angular_rate_scale;
}

static void AppImu_UpdateMeanAndVariance(float sample, uint32_t count, float *mean, float *m2)
{
  const float delta = sample - *mean;
  *mean += delta / (float)count;
  const float delta_after_mean = sample - *mean;
  *m2 += delta * delta_after_mean;
}

static void AppImu_CopyFilterState(void)
{
  memcpy(imu_output.quaternion, imu_filter.quaternion, sizeof(imu_output.quaternion));
  memcpy(imu_output.gyro_bias_rad_s, imu_filter.gyro_bias_rad_s, sizeof(imu_output.gyro_bias_rad_s));
  ImuEskf_GetEulerRad(&imu_filter, imu_output.euler_rad);
  if (ImuEskf_IsConverged(&imu_filter)) {
    imu_output.flags |= APP_IMU_FLAG_FILTER_CONVERGED;
  } else {
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_FILTER_CONVERGED;
  }
  imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_ABSOLUTE_YAW_VALID;
}

static void AppImu_UpdateStaleState(uint32_t now_ms)
{
  imu_output.sample_age_ms = now_ms - imu_output.last_good_tick_ms;
  if (!imu_calibrated || (now_ms - imu_output.last_good_tick_ms) > ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS) {
    imu_output.flags |= APP_IMU_FLAG_DATA_STALE;
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_DATA_VALID;
  }
}

BspStatus AppImu_Calibrate(void)
{
  memset(&imu_filter, 0, sizeof(imu_filter));
  memset(&imu_output, 0, sizeof(imu_output));
  imu_calibrated = false;
  previous_sensor_timestamp = 0U;
  memset(previous_acceleration_mps2, 0, sizeof(previous_acceleration_mps2));
  memset(previous_angular_rate_rad_s, 0, sizeof(previous_angular_rate_rad_s));
  imu_output.flags = APP_IMU_FLAG_SENSOR_PRESENT;

  float acceleration_mean[3] = {0.0f};
  float acceleration_m2[3] = {0.0f};
  float angular_rate_mean[3] = {0.0f};
  float angular_rate_m2[3] = {0.0f};
  BspImuSample latest_sample = {0};
  uint32_t sample_count = 0U;
  uint32_t read_error_count = 0U;
  uint32_t discard_count = APP_IMU_CALIBRATION_DISCARD_SAMPLES;
  const uint32_t started_at = HAL_GetTick();

  while (sample_count < ROBOT_CONFIG_IMU_CALIBRATION_SAMPLES &&
         (HAL_GetTick() - started_at) < ROBOT_CONFIG_IMU_CALIBRATION_TIMEOUT_MS) {
    BspImuSample sample;
    const BspStatus status = BspImu_ReadSample(&sample);
    if (status == BSP_OK) {
      if (discard_count > 0U) {
        discard_count--;
        osDelay(1U);
        continue;
      }
      if (sample_count > 0U && sample.sensor_timestamp == latest_sample.sensor_timestamp) {
        osDelay(1U);
        continue;
      }

      float acceleration_mps2[3];
      float angular_rate_rad_s[3];
      AppImu_ConvertToBodySi(&sample, acceleration_mps2, angular_rate_rad_s);
      if (fabsf(AppImu_VectorNorm(acceleration_mps2) - ROBOT_CONFIG_STANDARD_GRAVITY_MPS2) >
            APP_IMU_CALIBRATION_ACCEL_NORM_TOLERANCE ||
          AppImu_VectorNorm(angular_rate_rad_s) > APP_IMU_CALIBRATION_GYRO_NORM_LIMIT) {
        sample_count = 0U;
        memset(acceleration_mean, 0, sizeof(acceleration_mean));
        memset(acceleration_m2, 0, sizeof(acceleration_m2));
        memset(angular_rate_mean, 0, sizeof(angular_rate_mean));
        memset(angular_rate_m2, 0, sizeof(angular_rate_m2));
        memset(&latest_sample, 0, sizeof(latest_sample));
        osDelay(1U);
        continue;
      }
      sample_count++;
      for (uint32_t axis = 0U; axis < 3U; ++axis) {
        AppImu_UpdateMeanAndVariance(
          acceleration_mps2[axis], sample_count, &acceleration_mean[axis], &acceleration_m2[axis]);
        AppImu_UpdateMeanAndVariance(
          angular_rate_rad_s[axis], sample_count, &angular_rate_mean[axis], &angular_rate_m2[axis]);
      }
      latest_sample = sample;
    } else if (status != BSP_BUSY) {
      read_error_count++;
      if (read_error_count >= APP_IMU_MAX_CALIBRATION_READ_ERRORS) {
        imu_output.flags |= APP_IMU_FLAG_SENSOR_FAULT;
        return status;
      }
    }
    osDelay(1U);
  }

  if (sample_count < ROBOT_CONFIG_IMU_CALIBRATION_SAMPLES) {
    return BSP_TIMEOUT;
  }

  float acceleration_std[3];
  float angular_rate_std[3];
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    acceleration_std[axis] = sqrtf(acceleration_m2[axis] / (float)(sample_count - 1U));
    angular_rate_std[axis] = sqrtf(angular_rate_m2[axis] / (float)(sample_count - 1U));
    if (acceleration_std[axis] > APP_IMU_CALIBRATION_ACCEL_STD_LIMIT ||
        angular_rate_std[axis] > APP_IMU_CALIBRATION_GYRO_STD_LIMIT) {
      return BSP_ERROR;
    }
  }

  if (fabsf(AppImu_VectorNorm(acceleration_mean) - ROBOT_CONFIG_STANDARD_GRAVITY_MPS2) >
        APP_IMU_CALIBRATION_ACCEL_NORM_TOLERANCE ||
      AppImu_VectorNorm(angular_rate_mean) > APP_IMU_CALIBRATION_GYRO_NORM_LIMIT) {
    return BSP_ERROR;
  }
  if (!ImuEskf_Init(&imu_filter, acceleration_mean, angular_rate_mean)) {
    return BSP_ERROR;
  }

  imu_calibrated = true;
  previous_sensor_timestamp = latest_sample.sensor_timestamp;
  imu_output.raw_sample = latest_sample;
  memcpy(imu_output.acceleration_mps2, acceleration_mean, sizeof(acceleration_mean));
  memcpy(imu_output.filtered_acceleration_mps2, acceleration_mean, sizeof(acceleration_mean));
  memcpy(previous_acceleration_mps2, acceleration_mean, sizeof(acceleration_mean));
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    imu_output.angular_rate_rad_s[axis] = 0.0f;
    imu_output.filtered_angular_rate_rad_s[axis] = 0.0f;
    previous_angular_rate_rad_s[axis] = angular_rate_mean[axis];
  }
  imu_output.temperature_celsius = (float)latest_sample.temperature / 256.0f;
  imu_output.sensor_timestamp = latest_sample.sensor_timestamp;
  imu_output.host_tick_ms = latest_sample.host_tick_ms;
  imu_output.last_good_tick_ms = latest_sample.host_tick_ms;
  imu_output.sample_age_ms = 0U;
  imu_output.read_error_count = read_error_count;
  imu_output.flags |= APP_IMU_FLAG_CALIBRATED | APP_IMU_FLAG_DATA_VALID | APP_IMU_FLAG_FILTER_INITIALIZED |
                      APP_IMU_FLAG_TIMESTAMP_VALID | APP_IMU_FLAG_TILT_VALID;
  AppImu_CopyFilterState();
  return BSP_OK;
}

BspStatus AppImu_Process(uint32_t now_ms, AppImuOutput *output)
{
  if (output == NULL) {
    return BSP_INVALID_ARG;
  }
  if (!imu_calibrated) {
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_ERROR;
  }

  BspImuSample sample;
  const BspStatus status = BspImu_ReadSample(&sample);
  if (status == BSP_BUSY) {
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_BUSY;
  }
  if (status != BSP_OK) {
    imu_output.read_error_count = AppImu_SaturatingIncrement(imu_output.read_error_count);
    imu_output.consecutive_error_count = AppImu_SaturatingIncrement(imu_output.consecutive_error_count);
    if (imu_output.consecutive_error_count >= ROBOT_CONFIG_IMU_ERROR_BACKOFF_THRESHOLD) {
      imu_output.flags |= APP_IMU_FLAG_SENSOR_FAULT;
      imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_DATA_VALID;
    }
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return status;
  }

  const uint32_t timestamp_step = (sample.sensor_timestamp - previous_sensor_timestamp) & APP_IMU_TIMESTAMP_MASK;
  if (timestamp_step == 0U) {
    imu_output.duplicate_count = AppImu_SaturatingIncrement(imu_output.duplicate_count);
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_BUSY;
  }

  if (timestamp_step > APP_IMU_MAX_TIMESTAMP_STEP) {
    previous_sensor_timestamp = sample.sensor_timestamp;
    imu_output.dropped_sample_count = AppImu_SaturatingAdd(imu_output.dropped_sample_count, timestamp_step - 1U);
    imu_output.flags |= APP_IMU_FLAG_DATA_STALE;
    imu_output.flags &=
      ~((uint32_t)APP_IMU_FLAG_DATA_VALID | (uint32_t)APP_IMU_FLAG_TIMESTAMP_VALID |
        (uint32_t)APP_IMU_FLAG_FILTER_CONVERGED);
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_ERROR;
  }

  if (timestamp_step > 1U) {
    const uint32_t dropped = timestamp_step - 1U;
    imu_output.dropped_sample_count = AppImu_SaturatingAdd(imu_output.dropped_sample_count, dropped);
  }

  float acceleration_mps2[3];
  float angular_rate_rad_s[3];
  AppImu_ConvertToBodySi(&sample, acceleration_mps2, angular_rate_rad_s);
  previous_sensor_timestamp = sample.sensor_timestamp;
  if (AppImu_IsSpike(acceleration_mps2, angular_rate_rad_s)) {
    imu_output.spike_reject_count = AppImu_SaturatingIncrement(imu_output.spike_reject_count);
    imu_output.consecutive_spike_count = AppImu_SaturatingIncrement(imu_output.consecutive_spike_count);
    memcpy(previous_acceleration_mps2, acceleration_mps2, sizeof(previous_acceleration_mps2));
    memcpy(previous_angular_rate_rad_s, angular_rate_rad_s, sizeof(previous_angular_rate_rad_s));
    imu_output.flags |= APP_IMU_FLAG_SAMPLE_SPIKE;
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_DATA_VALID;
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_ERROR;
  }
  float delta_time_s = (float)timestamp_step / ROBOT_CONFIG_IMU_ODR_HZ;
  if (delta_time_s > APP_IMU_MAX_FILTER_DELTA_TIME_S) {
    delta_time_s = APP_IMU_MAX_FILTER_DELTA_TIME_S;
  }

  bool accel_update_used = false;
  bool vibration_high = false;
  if (!ImuEskf_Update(
        &imu_filter, angular_rate_rad_s, acceleration_mps2, delta_time_s, &accel_update_used, &vibration_high)) {
    imu_output.flags |= APP_IMU_FLAG_SENSOR_FAULT;
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_DATA_VALID;
    *output = imu_output;
    return BSP_ERROR;
  }

  imu_output.raw_sample = sample;
  memcpy(imu_output.acceleration_mps2, acceleration_mps2, sizeof(acceleration_mps2));
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    imu_output.angular_rate_rad_s[axis] = angular_rate_rad_s[axis] - imu_filter.gyro_bias_rad_s[axis];
  }
  AppImu_UpdateFilteredOutput(
    acceleration_mps2, imu_output.angular_rate_rad_s, delta_time_s);
  memcpy(previous_acceleration_mps2, acceleration_mps2, sizeof(previous_acceleration_mps2));
  memcpy(previous_angular_rate_rad_s, angular_rate_rad_s, sizeof(previous_angular_rate_rad_s));
  imu_output.temperature_celsius = (float)sample.temperature / 256.0f;
  imu_output.sensor_timestamp = sample.sensor_timestamp;
  imu_output.host_tick_ms = sample.host_tick_ms;
  imu_output.last_good_tick_ms = now_ms;
  imu_output.sample_age_ms = 0U;
  imu_output.sequence = AppImu_SaturatingIncrement(imu_output.sequence);
  imu_output.consecutive_error_count = 0U;
  imu_output.consecutive_spike_count = 0U;
  imu_output.flags |= APP_IMU_FLAG_SENSOR_PRESENT | APP_IMU_FLAG_CALIBRATED | APP_IMU_FLAG_DATA_VALID |
                      APP_IMU_FLAG_FILTER_INITIALIZED | APP_IMU_FLAG_TIMESTAMP_VALID | APP_IMU_FLAG_TILT_VALID;
  imu_output.flags &=
    ~((uint32_t)APP_IMU_FLAG_SENSOR_FAULT | (uint32_t)APP_IMU_FLAG_DATA_STALE |
      (uint32_t)APP_IMU_FLAG_ACCEL_UPDATE_USED | (uint32_t)APP_IMU_FLAG_VIBRATION_HIGH);
  imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_SAMPLE_SPIKE;
  if (accel_update_used) {
    imu_output.flags |= APP_IMU_FLAG_ACCEL_UPDATE_USED;
    imu_output.accel_update_accept_count = AppImu_SaturatingIncrement(imu_output.accel_update_accept_count);
  } else {
    imu_output.accel_update_reject_count = AppImu_SaturatingIncrement(imu_output.accel_update_reject_count);
  }
  if (vibration_high) {
    imu_output.flags |= APP_IMU_FLAG_VIBRATION_HIGH;
  }
  AppImu_CopyFilterState();
  *output = imu_output;
  return BSP_OK;
}
