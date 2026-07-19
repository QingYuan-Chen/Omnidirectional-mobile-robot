#include "app_wheel_speed_estimator.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static AppWheelSpeedEstimatorConfig MakeConfig(void)
{
  const AppWheelSpeedEstimatorConfig config = {
    .encoder_counts_per_revolution = 122880U,
    .control_rate_hz = 1000U,
    .timestamp_clock_hz = 168000000U,
    .period_events_per_revolution = 30720U,
    .t_stale_timeout_cycles = 16800000U,
    .switch_to_t_max_activity_counts = 20U,
    .switch_to_m_min_activity_counts = 40U,
    .m_window_ticks = 10U,
  };
  return config;
}

static AppEncoderPeriodSnapshot MakePeriod(
  uint32_t sum,
  uint16_t count,
  uint32_t age,
  int8_t direction,
  uint8_t extra_flags)
{
  AppEncoderPeriodSnapshot period;
  memset(&period, 0, sizeof(period));
  period.period_sum_cycles = sum;
  period.period_count = count;
  period.last_edge_age_cycles = age;
  period.direction = direction;
  period.flags = APP_ENCODER_PERIOD_FLAG_HAS_EDGE | extra_flags;
  return period;
}

static void PushTicks(
  AppWheelSpeedEstimator *estimator,
  int16_t delta,
  uint32_t count,
  const AppEncoderPeriodSnapshot *period,
  AppWheelSpeedEstimatorOutput *output)
{
  for (uint32_t i = 0U; i < count; ++i) {
    AppWheelSpeedEstimatorInput input;
    memset(&input, 0, sizeof(input));
    input.encoder_delta = delta;
    if (period != NULL) {
      input.period = *period;
    }
    assert(AppWheelSpeedEstimator_Update(estimator, &input, output));
  }
}

static void TestConfigurationAndMWindow(void)
{
  AppWheelSpeedEstimator estimator;
  AppWheelSpeedEstimatorConfig config = MakeConfig();
  AppWheelSpeedEstimatorOutput output;

  assert(!AppWheelSpeedEstimator_Init(NULL, &config));
  assert(!AppWheelSpeedEstimator_Init(&estimator, NULL));
  config.m_window_ticks = 0U;
  assert(!AppWheelSpeedEstimator_Init(&estimator, &config));
  config = MakeConfig();
  config.switch_to_t_max_activity_counts = 40U;
  assert(!AppWheelSpeedEstimator_Init(&estimator, &config));

  config = MakeConfig();
  assert(AppWheelSpeedEstimator_Init(&estimator, &config));
  PushTicks(&estimator, 2, 9U, NULL, &output);
  assert(!output.m_valid);
  assert(!output.valid);
  PushTicks(&estimator, 2, 1U, NULL, &output);
  assert(output.m_valid);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_M);
  assert(output.valid);
  assert(fabsf(output.m_speed_rpm - 0.9765625f) < 0.00001f);
}

static void TestTUpdateHoldAndStaleFallback(void)
{
  AppWheelSpeedEstimator estimator;
  const AppWheelSpeedEstimatorConfig config = MakeConfig();
  AppWheelSpeedEstimatorOutput output;
  assert(AppWheelSpeedEstimator_Init(&estimator, &config));

  AppEncoderPeriodSnapshot period =
    MakePeriod(1312500U, 4U, 100U, 1, APP_ENCODER_PERIOD_FLAG_HAS_PERIOD);
  PushTicks(&estimator, 1, 10U, &period, &output);
  assert(output.m_valid);
  assert(output.t_valid);
  assert(output.t_updated);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_T);
  assert(fabsf(output.speed_rpm - 1.0f) < 0.00001f);

  period = MakePeriod(0U, 0U, 1000U, 1, 0U);
  PushTicks(&estimator, 1, 1U, &period, &output);
  assert(output.t_valid);
  assert(!output.t_updated);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_T);

  period.last_edge_age_cycles = config.t_stale_timeout_cycles + 1U;
  PushTicks(&estimator, 1, 1U, &period, &output);
  assert(output.t_stale);
  assert(!output.t_valid);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_M);
  assert(output.valid);
}

static void TestHysteresisAndDirectionReset(void)
{
  AppWheelSpeedEstimator estimator;
  const AppWheelSpeedEstimatorConfig config = MakeConfig();
  AppWheelSpeedEstimatorOutput output;
  AppEncoderPeriodSnapshot period =
    MakePeriod(1312500U, 4U, 100U, 1, APP_ENCODER_PERIOD_FLAG_HAS_PERIOD);
  assert(AppWheelSpeedEstimator_Init(&estimator, &config));

  PushTicks(&estimator, 5, 10U, &period, &output);
  assert(output.m_activity_counts == 50U);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_M);

  period = MakePeriod(0U, 0U, 100U, 1, 0U);
  PushTicks(&estimator, 2, 10U, &period, &output);
  assert(output.m_activity_counts == 20U);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_T);

  PushTicks(&estimator, 3, 10U, &period, &output);
  assert(output.m_activity_counts == 30U);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_T);

  PushTicks(&estimator, 4, 10U, &period, &output);
  assert(output.m_activity_counts == 40U);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_M);

  period = MakePeriod(
    0U, 0U, 0U, -1, APP_ENCODER_PERIOD_FLAG_DIRECTION_RESET);
  PushTicks(&estimator, 4, 1U, &period, &output);
  assert(!output.t_valid);
  assert(output.source == APP_WHEEL_SPEED_SOURCE_M);
}

int main(void)
{
  TestConfigurationAndMWindow();
  TestTUpdateHoldAndStaleFallback();
  TestHysteresisAndDirectionReset();
  return 0;
}
