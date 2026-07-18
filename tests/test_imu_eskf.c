#include "imu_eskf.h"
#include "robot_config.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

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

static void Test_InitializationAndConvergenceBoundary(void)
{
  ImuEskf filter;
  const float gravity[3] = {0.0f, 0.0f, ROBOT_CONFIG_STANDARD_GRAVITY_MPS2};
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  bool accel_update_used = false;
  bool vibration_high = false;

  TEST_ASSERT(ImuEskf_Init(&filter, gravity, zero_gyro));
  TEST_ASSERT(ImuEskf_IsStateFinite(&filter));
  for (uint32_t update_index = 0U; update_index < 99U; ++update_index) {
    TEST_ASSERT(ImuEskf_Update(
      &filter, zero_gyro, gravity, 1.0f / ROBOT_CONFIG_IMU_ODR_HZ, &accel_update_used, &vibration_high));
    TEST_ASSERT(accel_update_used);
    TEST_ASSERT(!vibration_high);
  }
  TEST_ASSERT(!ImuEskf_IsConverged(&filter));

  TEST_ASSERT(ImuEskf_Update(
    &filter, zero_gyro, gravity, 1.0f / ROBOT_CONFIG_IMU_ODR_HZ, &accel_update_used, &vibration_high));
  TEST_ASSERT(ImuEskf_IsConverged(&filter));
}

static void Test_AccelerationAndVibrationGates(void)
{
  ImuEskf filter;
  const float gravity[3] = {0.0f, 0.0f, ROBOT_CONFIG_STANDARD_GRAVITY_MPS2};
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float tilted_gravity[3] = {0.0f, -6.934f, 6.934f};
  const float high_vibration[3] = {0.0f, 0.0f, 10.77f};
  bool accel_update_used = true;
  bool vibration_high = true;

  TEST_ASSERT(ImuEskf_Init(&filter, gravity, zero_gyro));
  TEST_ASSERT(ImuEskf_Update(
    &filter, zero_gyro, tilted_gravity, 1.0f / ROBOT_CONFIG_IMU_ODR_HZ, &accel_update_used, &vibration_high));
  TEST_ASSERT(!accel_update_used);
  TEST_ASSERT(!vibration_high);

  TEST_ASSERT(ImuEskf_Update(
    &filter, zero_gyro, high_vibration, 1.0f / ROBOT_CONFIG_IMU_ODR_HZ, &accel_update_used, &vibration_high));
  TEST_ASSERT(accel_update_used);
  TEST_ASSERT(vibration_high);
}

static void Test_InvalidTimeAndNonFiniteState(void)
{
  ImuEskf filter;
  const float gravity[3] = {0.0f, 0.0f, ROBOT_CONFIG_STANDARD_GRAVITY_MPS2};
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  const float invalid_gyro[3] = {NAN, 0.0f, 0.0f};
  bool accel_update_used = false;
  bool vibration_high = false;

  TEST_ASSERT(ImuEskf_Init(&filter, gravity, zero_gyro));
  TEST_ASSERT(!ImuEskf_Update(
    &filter, zero_gyro, gravity, 0.0f, &accel_update_used, &vibration_high));
  TEST_ASSERT(ImuEskf_IsStateFinite(&filter));
  TEST_ASSERT(!ImuEskf_Update(
    &filter, invalid_gyro, gravity, 1.0f / ROBOT_CONFIG_IMU_ODR_HZ, &accel_update_used, &vibration_high));
  TEST_ASSERT(!ImuEskf_IsStateFinite(&filter));
}

int main(void)
{
  Test_InitializationAndConvergenceBoundary();
  Test_AccelerationAndVibrationGates();
  Test_InvalidTimeAndNonFiniteState();
  (void)puts("imu_eskf 测试通过");
  return EXIT_SUCCESS;
}
