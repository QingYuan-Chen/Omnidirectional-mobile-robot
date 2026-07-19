#include "app_motor_capture.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static AppMotorCaptureInput MakeInput(uint32_t index)
{
  const AppMotorCaptureInput input = {
    .tick_sequence = index + 10U,
    .irq_timestamp_cycles = index * 168000U,
    .wake_latency_cycles = index + 20U,
    .previous_wcet_cycles = index + 40U,
    .encoder_raw_ma = (uint16_t)index,
    .encoder_delta_ma = (int16_t)(index % 100U),
    .target_pwm = -400,
    .applied_pwm = (int16_t)(index % 401U),
    .battery_millivolts = 11600U,
    .motor_state = 2U,
    .safety_flags = APP_MOTOR_CAPTURE_FLAG_RUNTIME_READY |
                    APP_MOTOR_CAPTURE_FLAG_MOTION_AVAILABLE,
  };
  return input;
}

static void TestManualCapture(void)
{
  static AppMotorCapture capture;
  AppMotorCapture_Init(&capture);

  AppMotorCaptureStatus status;
  assert(AppMotorCapture_GetStatus(&capture, &status));
  assert(status.state == APP_MOTOR_CAPTURE_IDLE);
  assert(status.sample_count == 0U);
  assert(status.capacity == ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY);

  assert(AppMotorCapture_Start(&capture));
  const AppMotorCaptureInput first = MakeInput(0U);
  const AppMotorCaptureInput second = MakeInput(1U);
  assert(AppMotorCapture_Record(&capture, &first));
  assert(AppMotorCapture_Record(&capture, &second));
  assert(AppMotorCapture_Stop(&capture));
  assert(!AppMotorCapture_Stop(&capture));

  assert(AppMotorCapture_GetStatus(&capture, &status));
  assert(status.state == APP_MOTOR_CAPTURE_COMPLETE);
  assert(status.sample_count == 2U);
  assert(status.dropped_sample_count == 0U);

  AppMotorCaptureSample sample;
  assert(AppMotorCapture_GetSample(&capture, 0U, &sample));
  assert(memcmp(&sample, &first, sizeof(sample)) == 0);
  assert(AppMotorCapture_GetSample(&capture, 1U, &sample));
  assert(memcmp(&sample, &second, sizeof(sample)) == 0);
  assert(!AppMotorCapture_GetSample(&capture, 2U, &sample));

  assert(AppMotorCapture_Start(&capture));
  assert(AppMotorCapture_GetStatus(&capture, &status));
  assert(status.state == APP_MOTOR_CAPTURE_RECORDING);
  assert(status.sample_count == 0U);
  assert(!AppMotorCapture_GetSample(&capture, 0U, &sample));
}

static void TestFullBufferStopsWithoutOverwrite(void)
{
  static AppMotorCapture capture;
  AppMotorCapture_Init(&capture);
  assert(AppMotorCapture_Start(&capture));

  for (uint32_t i = 0U; i < ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY; ++i) {
    const AppMotorCaptureInput input = MakeInput(i);
    assert(AppMotorCapture_Record(&capture, &input));
  }
  const AppMotorCaptureInput overflow = MakeInput(UINT32_MAX);
  assert(AppMotorCapture_Record(&capture, &overflow));

  AppMotorCaptureStatus status;
  assert(AppMotorCapture_GetStatus(&capture, &status));
  assert(status.state == APP_MOTOR_CAPTURE_COMPLETE);
  assert(status.sample_count == ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY);
  assert(status.dropped_sample_count == 1U);

  AppMotorCaptureSample first;
  AppMotorCaptureSample last;
  assert(AppMotorCapture_GetSample(&capture, 0U, &first));
  assert(AppMotorCapture_GetSample(
    &capture, ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY - 1U, &last));
  assert(first.tick_sequence == 10U);
  assert(last.tick_sequence ==
         ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY - 1U + 10U);
}

static void TestFormatting(void)
{
  const AppMotorCaptureStatus status = {
    .state = APP_MOTOR_CAPTURE_COMPLETE,
    .sample_count = 2U,
    .capacity = ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY,
    .dropped_sample_count = 1U,
  };
  uint8_t buffer[160];
  uint16_t length = 0U;
  assert(AppMotorCapture_FormatEvent(
    APP_MOTOR_CAPTURE_EVENT_BEGIN,
    &status,
    buffer,
    (uint16_t)sizeof(buffer),
    &length));
  assert(strcmp(
    (const char *)buffer,
    "MCAP,1,BEGIN,2,2,2200,1\n") == 0);
  assert(length == strlen((const char *)buffer));

  const AppMotorCaptureSample sample = MakeInput(7U);
  assert(AppMotorCapture_FormatSample(
    7U, &sample, buffer, (uint16_t)sizeof(buffer), &length));
  assert(strcmp(
    (const char *)buffer,
    "MC,1,7,17,1176000,27,47,7,7,-400,7,11600,2,9\n") == 0);
  assert(length == strlen((const char *)buffer));

  uint8_t short_buffer[8];
  length = 99U;
  assert(!AppMotorCapture_FormatSample(
    7U,
    &sample,
    short_buffer,
    (uint16_t)sizeof(short_buffer),
    &length));
  assert(length == 0U);
}

int main(void)
{
  TestManualCapture();
  TestFullBufferStopsWithoutOverwrite();
  TestFormatting();
  return 0;
}
