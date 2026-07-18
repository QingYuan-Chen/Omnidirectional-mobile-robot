#include "app_imu.h"
#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_TIMESTAMP_MASK (0x00FFFFFFUL)

static uint32_t fake_tick_ms;
static uint32_t fake_sensor_timestamp;
static uint32_t fake_read_count;
static BspStatus fake_read_status;
static bool fake_use_explicit_timestamp;
static uint32_t fake_explicit_timestamp;

static void Test_Fail(const char *expression, int line)
{
  (void)fprintf(stderr, "断言失败，第 %d 行：%s\n", line, expression);
  exit(EXIT_FAILURE);
}

#define TEST_ASSERT(expression) \
  do { \
    if (!(expression)) { \
      Test_Fail(#expression, __LINE__); \
    } \
  } while (false)

uint32_t HAL_GetTick(void)
{
  return fake_tick_ms;
}

void osDelay(uint32_t millisec)
{
  fake_tick_ms += millisec;
}

BspStatus BspImu_ReadSample(BspImuSample *sample)
{
  fake_read_count++;
  if (fake_read_status != BSP_OK) {
    return fake_read_status;
  }
  if (sample == NULL) {
    return BSP_INVALID_ARG;
  }

  memset(sample, 0, sizeof(*sample));
  sample->acceleration[2] = (int16_t)8192;
  sample->temperature = (int16_t)(25 * 256);
  if (fake_use_explicit_timestamp) {
    fake_sensor_timestamp = fake_explicit_timestamp & TEST_TIMESTAMP_MASK;
    fake_use_explicit_timestamp = false;
  } else {
    fake_sensor_timestamp = (fake_sensor_timestamp + 1U) & TEST_TIMESTAMP_MASK;
  }
  sample->sensor_timestamp = fake_sensor_timestamp;
  sample->host_tick_ms = fake_tick_ms;
  sample->status = 0x03U;
  return BSP_OK;
}

static void Test_ResetAndCalibrate(uint32_t initial_tick_ms)
{
  fake_tick_ms = initial_tick_ms;
  fake_sensor_timestamp = 0U;
  fake_read_count = 0U;
  fake_read_status = BSP_OK;
  fake_use_explicit_timestamp = false;
  fake_explicit_timestamp = 0U;
  TEST_ASSERT(AppImu_Calibrate() == BSP_OK);
}

static AppImuOutput Test_AcceptAt(uint32_t now_ms)
{
  AppImuOutput output;
  fake_tick_ms = now_ms;
  fake_read_status = BSP_OK;
  TEST_ASSERT(AppImu_Process(now_ms, &output) == BSP_OK);
  return output;
}

static AppImuOutput Test_BusyAt(uint32_t now_ms)
{
  AppImuOutput output;
  fake_tick_ms = now_ms;
  fake_read_status = BSP_BUSY;
  TEST_ASSERT(AppImu_Process(now_ms, &output) == BSP_BUSY);
  return output;
}

static void Test_StaleBoundaryAndStableRecovery(void)
{
  Test_ResetAndCalibrate(1000U);
  AppImuOutput output = Test_AcceptAt(fake_tick_ms);
  const uint32_t last_good_tick_ms = output.last_good_tick_ms;

  output = Test_BusyAt(last_good_tick_ms + 19U);
  TEST_ASSERT(output.sample_age_ms == 19U);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_HEALTHY);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_VALID) != 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_STALE) == 0U);

  output = Test_BusyAt(last_good_tick_ms + ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS);
  TEST_ASSERT(output.sample_age_ms == ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_TRANSIENT_DEGRADED);
  TEST_ASSERT(output.stable_sample_count == 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_VALID) == 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_STALE) != 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_SENSOR_DEGRADED) != 0U);

  for (uint32_t sample_index = 1U; sample_index < ROBOT_CONFIG_IMU_RECOVERY_STABLE_SAMPLES;
       ++sample_index) {
    output = Test_AcceptAt(last_good_tick_ms + ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS + sample_index);
    TEST_ASSERT(output.health == APP_IMU_HEALTH_RECOVERING);
    TEST_ASSERT(output.stable_sample_count == sample_index);
    TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_VALID) == 0U);
  }

  output = Test_AcceptAt(
    last_good_tick_ms + ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS + ROBOT_CONFIG_IMU_RECOVERY_STABLE_SAMPLES);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_HEALTHY);
  TEST_ASSERT(output.stable_sample_count == ROBOT_CONFIG_IMU_RECOVERY_STABLE_SAMPLES);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_VALID) != 0U);
  TEST_ASSERT((output.flags & (APP_IMU_FLAG_DATA_STALE | APP_IMU_FLAG_SENSOR_DEGRADED |
                               APP_IMU_FLAG_RECOVERING)) == 0U);
}

static void Test_DuplicateTimestampResetsRecovery(void)
{
  Test_ResetAndCalibrate(2000U);
  AppImuOutput output = Test_AcceptAt(fake_tick_ms);
  const uint32_t last_good_tick_ms = output.last_good_tick_ms;
  output = Test_BusyAt(last_good_tick_ms + ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS);

  fake_use_explicit_timestamp = true;
  fake_explicit_timestamp = output.sensor_timestamp;
  fake_read_status = BSP_OK;
  fake_tick_ms++;
  TEST_ASSERT(AppImu_Process(fake_tick_ms, &output) == BSP_BUSY);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_TRANSIENT_DEGRADED);
  TEST_ASSERT(output.stable_sample_count == 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_RECOVERING) == 0U);

  output = Test_AcceptAt(last_good_tick_ms + ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS + 1U);
  TEST_ASSERT(output.stable_sample_count == 1U);

  output = Test_BusyAt(fake_tick_ms + 1U);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_RECOVERING);
  TEST_ASSERT(output.stable_sample_count == 1U);

  fake_use_explicit_timestamp = true;
  fake_explicit_timestamp = output.sensor_timestamp;
  fake_read_status = BSP_OK;
  fake_tick_ms++;
  TEST_ASSERT(AppImu_Process(fake_tick_ms, &output) == BSP_BUSY);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_RECOVERING);
  TEST_ASSERT(output.stable_sample_count == 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_VALID) == 0U);

  output = Test_AcceptAt(fake_tick_ms + 1U);
  TEST_ASSERT(output.stable_sample_count == 1U);
}

static void Test_BackoffDoesNotBlockSnapshotRefresh(void)
{
  Test_ResetAndCalibrate(3000U);
  AppImuOutput output = Test_AcceptAt(fake_tick_ms);
  const uint32_t last_good_tick_ms = output.last_good_tick_ms;

  fake_tick_ms = last_good_tick_ms + 1U;
  const uint32_t error_tick_ms = fake_tick_ms;
  fake_read_status = BSP_ERROR;
  const uint32_t reads_before_error = fake_read_count;
  TEST_ASSERT(AppImu_Process(fake_tick_ms, &output) == BSP_ERROR);
  TEST_ASSERT(fake_read_count == reads_before_error + 1U);
  TEST_ASSERT(output.retry_delay_ms == 20U);

  fake_read_status = BSP_OK;
  const uint32_t reads_before_backoff_poll = fake_read_count;
  fake_tick_ms = error_tick_ms + 19U;
  TEST_ASSERT(AppImu_Process(fake_tick_ms, &output) == BSP_BUSY);
  TEST_ASSERT(fake_read_count == reads_before_backoff_poll);
  TEST_ASSERT(output.sample_age_ms == 20U);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_TRANSIENT_DEGRADED);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_STALE) != 0U);

  fake_tick_ms = error_tick_ms + 20U;
  TEST_ASSERT(AppImu_Process(fake_tick_ms, &output) == BSP_OK);
  TEST_ASSERT(fake_read_count == reads_before_backoff_poll + 1U);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_RECOVERING);
}

static void Test_StaleDoesNotDowngradePersistentFault(void)
{
  Test_ResetAndCalibrate(4000U);
  AppImuOutput output = Test_AcceptAt(fake_tick_ms);
  uint32_t attempt_tick_ms = output.last_good_tick_ms + 1U;
  const uint32_t delays_ms[] = {20U, 50U, 100U};

  fake_read_status = BSP_ERROR;
  for (uint32_t index = 0U; index < 3U; ++index) {
    TEST_ASSERT(AppImu_Process(attempt_tick_ms, &output) == BSP_ERROR);
    if (index + 1U < 3U) {
      attempt_tick_ms += delays_ms[index];
    }
  }
  TEST_ASSERT(output.health == APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT);

  TEST_ASSERT(AppImu_Process(output.last_good_tick_ms + 100U, &output) == BSP_BUSY);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_DATA_STALE) != 0U);

  fake_use_explicit_timestamp = true;
  fake_explicit_timestamp = output.sensor_timestamp;
  fake_read_status = BSP_OK;
  const uint32_t recovery_tick_ms = output.next_retry_tick_ms;
  fake_tick_ms = recovery_tick_ms;
  TEST_ASSERT(AppImu_Process(fake_tick_ms, &output) == BSP_BUSY);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT);
  TEST_ASSERT(output.stable_sample_count == 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_RECOVERING) == 0U);

  output = Test_AcceptAt(recovery_tick_ms + 1U);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_RECOVERING);
  TEST_ASSERT(output.stable_sample_count == 1U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_SENSOR_FAULT) != 0U);

  output = Test_BusyAt(output.last_good_tick_ms + ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS);
  TEST_ASSERT(output.health == APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT);
  TEST_ASSERT(output.stable_sample_count == 0U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_SENSOR_FAULT) != 0U);
}

static void Test_MotionUsabilityUsesCurrentTime(void)
{
  Test_ResetAndCalibrate(UINT32_MAX - 510U);
  AppImuOutput output = Test_AcceptAt(fake_tick_ms);

  for (uint32_t index = 1U; index < 99U; ++index) {
    output = Test_AcceptAt(fake_tick_ms + 1U);
  }
  TEST_ASSERT((output.flags & APP_IMU_FLAG_FILTER_CONVERGED) == 0U);
  TEST_ASSERT(!AppImu_IsMotionUsable(&output, output.last_good_tick_ms));

  output = Test_AcceptAt(fake_tick_ms + 1U);
  TEST_ASSERT((output.flags & APP_IMU_FLAG_FILTER_CONVERGED) != 0U);
  TEST_ASSERT(AppImu_IsMotionUsable(&output, output.last_good_tick_ms + 19U));
  output.flags &= ~(uint32_t)APP_IMU_FLAG_ACCEL_UPDATE_USED;
  output.flags |= APP_IMU_FLAG_VIBRATION_HIGH;
  TEST_ASSERT(AppImu_IsMotionUsable(&output, output.last_good_tick_ms + 19U));
  TEST_ASSERT(!AppImu_IsMotionUsable(
    &output, output.last_good_tick_ms + ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS));
  TEST_ASSERT(!AppImu_IsMotionUsable(NULL, output.last_good_tick_ms));
}

int main(void)
{
  Test_StaleBoundaryAndStableRecovery();
  Test_DuplicateTimestampResetsRecovery();
  Test_BackoffDoesNotBlockSnapshotRefresh();
  Test_StaleDoesNotDowngradePersistentFault();
  Test_MotionUsabilityUsesCurrentTime();
  (void)puts("app_imu 测试通过");
  return EXIT_SUCCESS;
}
