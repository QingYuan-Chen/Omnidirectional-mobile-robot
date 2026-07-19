#include "app_speed_capture.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

static void TestLifecycleAndCapacity(void)
{
  AppSpeedCapture capture;
  AppSpeedCaptureStatus status;
  AppSpeedCaptureSample sample = {
    .tick_sequence = 1U,
    .irq_timestamp_cycles = 2U,
    .period_sum_cycles = 3U,
    .last_edge_age_cycles = 4U,
    .event_sequence = 5U,
    .encoder_delta_ma = -6,
    .applied_pwm = -7,
    .period_count = 8U,
    .direction = -1,
    .period_flags = 9U,
  };

  AppSpeedCapture_Init(&capture);
  assert(AppSpeedCapture_GetStatus(&capture, &status));
  assert(status.state == APP_SPEED_CAPTURE_IDLE);
  assert(AppSpeedCapture_Start(&capture));
  assert(AppSpeedCapture_Record(&capture, &sample));
  assert(AppSpeedCapture_Stop(&capture));
  memset(&sample, 0, sizeof(sample));
  assert(AppSpeedCapture_GetSample(&capture, 0U, &sample));
  assert(sample.encoder_delta_ma == -6);
  assert(sample.direction == -1);

  assert(AppSpeedCapture_Start(&capture));
  capture.sample_count = ROBOT_CONFIG_SPEED_CAPTURE_CAPACITY;
  assert(AppSpeedCapture_Record(&capture, &sample));
  assert(AppSpeedCapture_GetStatus(&capture, &status));
  assert(status.state == APP_SPEED_CAPTURE_COMPLETE);
  assert(status.dropped_sample_count == 1U);
}

static void TestStatsAndFormatting(void)
{
  AppSpeedCapture capture;
  AppSpeedCaptureStatus status;
  AppSpeedCaptureSample sample = {
    .tick_sequence = 10U,
    .irq_timestamp_cycles = 20U,
    .period_sum_cycles = 30U,
    .last_edge_age_cycles = 40U,
    .event_sequence = 50U,
    .encoder_delta_ma = -60,
    .applied_pwm = -70,
    .period_count = 8U,
    .direction = -1,
    .period_flags = 3U,
  };
  const AppEncoderPeriodStats stats = {
    .invalid_direction_count = 1U,
    .zero_period_count = 2U,
    .aggregate_drop_count = 3U,
    .direction_reset_count = 4U,
  };
  uint8_t buffer[256];
  uint16_t length = 0U;

  AppSpeedCapture_Init(&capture);
  assert(AppSpeedCapture_SetPeriodStats(&capture, &stats));
  assert(AppSpeedCapture_GetStatus(&capture, &status));
  assert(AppSpeedCapture_FormatEvent(
    APP_SPEED_CAPTURE_EVENT_STATUS,
    &status,
    buffer,
    (uint16_t)sizeof(buffer),
    &length));
  assert(strcmp(
    (const char *)buffer,
    "SCAP,1,STATUS,0,0,2200,0,1,2,3,4\n") == 0);

  assert(AppSpeedCapture_FormatSample(
    9U, &sample, buffer, (uint16_t)sizeof(buffer), &length));
  assert(strcmp(
    (const char *)buffer,
    "SC,1,9,10,20,-60,-70,30,8,40,50,-1,3\n") == 0);
  assert(!AppSpeedCapture_FormatSample(
    0U, &sample, buffer, 8U, &length));
}

int main(void)
{
  TestLifecycleAndCapacity();
  TestStatsAndFormatting();
  return 0;
}
