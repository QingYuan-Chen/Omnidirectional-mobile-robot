#include "app_imu_capture.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static AppImuCaptureSample MakeSample(uint32_t sequence)
{
  AppImuCaptureSample sample = {
    .sequence = sequence,
    .sensor_timestamp = 0x00FFFFFEU,
    .host_tick_ms = 1234U,
    .flags = 7U,
    .dropped_sample_count = 8U,
    .acceleration = {INT16_MIN, 0, INT16_MAX},
    .angular_rate = {-3, 4, -5},
    .temperature = -6,
    .sensor_status = 3U,
    .health = (uint8_t)APP_IMU_HEALTH_HEALTHY,
  };
  return sample;
}

static void TestLifecycleSequenceAndCapacity(void)
{
  AppImuCapture capture;
  AppImuCaptureStatus status;
  AppImuCaptureSample sample = MakeSample(UINT32_MAX);
  AppImuCapture_Init(&capture);
  assert(AppImuCapture_GetStatus(&capture, &status));
  assert(status.state == APP_IMU_CAPTURE_IDLE);
  assert(AppImuCapture_Start(&capture));
  assert(AppImuCapture_Record(&capture, &sample));
  assert(AppImuCapture_Record(&capture, &sample));
  sample.sequence = 1U;
  assert(AppImuCapture_Record(&capture, &sample));
  assert(AppImuCapture_Stop(&capture));
  assert(AppImuCapture_GetStatus(&capture, &status));
  assert(status.sample_count == 2U);
  assert(status.duplicate_sequence_count == 1U);
  assert(status.source_gap_count == 1U);

  memset(&sample, 0, sizeof(sample));
  assert(AppImuCapture_GetSample(&capture, 1U, &sample));
  assert(sample.sequence == 1U);

  assert(AppImuCapture_Start(&capture));
  capture.sample_count = ROBOT_CONFIG_IMU_CAPTURE_CAPACITY;
  sample.sequence = 7U;
  assert(AppImuCapture_Record(&capture, &sample));
  assert(AppImuCapture_GetStatus(&capture, &status));
  assert(status.state == APP_IMU_CAPTURE_COMPLETE);
  assert(status.dropped_sample_count == 1U);
}

static void TestFormattingAndBoundaries(void)
{
  AppImuCaptureStatus status = {
    .state = APP_IMU_CAPTURE_COMPLETE,
    .sample_count = 1700U,
    .capacity = 1700U,
    .dropped_sample_count = 1U,
    .duplicate_sequence_count = 2U,
    .source_gap_count = 3U,
  };
  AppImuCaptureSample sample = MakeSample(9U);
  uint8_t buffer[256];
  uint16_t length = 0U;

  assert(AppImuCapture_FormatEvent(
    APP_IMU_CAPTURE_EVENT_END,
    &status,
    buffer,
    (uint16_t)sizeof(buffer),
    &length));
  assert(strcmp(
    (const char *)buffer,
    "ICAP,1,END,2,1700,1700,1,2,3\n") == 0);
  assert(AppImuCapture_FormatSample(
    8U, &sample, buffer, (uint16_t)sizeof(buffer), &length));
  assert(strcmp(
    (const char *)buffer,
    "IC,1,8,9,16777214,1234,7,8,-32768,0,32767,-3,4,-5,-6,3,1\n") == 0);
  assert(!AppImuCapture_FormatSample(
    8U, &sample, buffer, 8U, &length));
  assert(length == 0U);
}

static void TestStartBaselineContract(void)
{
  AppImuCapture capture;
  AppImuCaptureSample sample = MakeSample(42U);
  AppImuCaptureStatus status;
  AppImuCapture_Init(&capture);
  assert(AppImuCapture_Start(&capture));

  /*
   * 任务编排层把 START 瞬间最后已完整发布并记录的 sequence 设为基线；相等序号不应
   * 形成样本行。发布快照与 Record 必须保持原子顺序，下一条 sequence 才是首样本。
   */
  capture.last_sequence = 42U;
  capture.has_last_sequence = true;
  assert(AppImuCapture_Record(&capture, &sample));
  sample.sequence = 43U;
  assert(AppImuCapture_Record(&capture, &sample));
  assert(AppImuCapture_GetStatus(&capture, &status));
  assert(status.sample_count == 1U);
  assert(status.duplicate_sequence_count == 1U);
  assert(capture.samples[0].sequence == 43U);
}

static void TestArguments(void)
{
  AppImuCapture capture;
  AppImuCaptureSample sample = MakeSample(1U);
  AppImuCaptureStatus status;
  AppImuCapture_Init(&capture);
  assert(!AppImuCapture_Record(NULL, &sample));
  assert(!AppImuCapture_Record(&capture, NULL));
  assert(!AppImuCapture_GetStatus(NULL, &status));
  assert(!AppImuCapture_GetSample(&capture, 0U, &sample));
}

int main(void)
{
  TestLifecycleSequenceAndCapacity();
  TestFormattingAndBoundaries();
  TestStartBaselineContract();
  TestArguments();
  return 0;
}
