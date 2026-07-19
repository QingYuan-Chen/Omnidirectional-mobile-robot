#include "app_imu_capture.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(AppImuCaptureSample) == 36U,
               "IMU capture sample size must remain 36 bytes");
_Static_assert(ROBOT_CONFIG_IMU_CAPTURE_CAPACITY > 0U,
               "IMU capture capacity must be non-zero");

typedef struct {
  uint8_t *buffer;
  uint16_t capacity;
  uint16_t length;
  bool valid;
} AppImuCaptureWriter;

static uint32_t AppImuCapture_AddSaturated(
  uint32_t value,
  uint32_t increment)
{
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static void AppImuCapture_AppendByte(
  AppImuCaptureWriter *writer,
  uint8_t value)
{
  if (writer == NULL || !writer->valid ||
      writer->length >= writer->capacity) {
    if (writer != NULL) {
      writer->valid = false;
    }
    return;
  }
  writer->buffer[writer->length] = value;
  writer->length++;
}

static void AppImuCapture_AppendLiteral(
  AppImuCaptureWriter *writer,
  const char *literal)
{
  if (writer == NULL || !writer->valid || literal == NULL) {
    if (writer != NULL) {
      writer->valid = false;
    }
    return;
  }
  const size_t literal_length = strlen(literal);
  if (literal_length > (size_t)(writer->capacity - writer->length)) {
    writer->valid = false;
    return;
  }
  memcpy(&writer->buffer[writer->length], literal, literal_length);
  writer->length = (uint16_t)(writer->length + (uint16_t)literal_length);
}

static void AppImuCapture_AppendU32(
  AppImuCaptureWriter *writer,
  uint32_t value)
{
  uint8_t digits[10];
  uint16_t count = 0U;
  do {
    digits[count] = (uint8_t)('0' + (value % 10U));
    count++;
    value /= 10U;
  } while (value != 0U);
  while (count > 0U) {
    count--;
    AppImuCapture_AppendByte(writer, digits[count]);
  }
}

static void AppImuCapture_AppendI32(
  AppImuCaptureWriter *writer,
  int32_t value)
{
  uint32_t magnitude;
  if (value < 0) {
    AppImuCapture_AppendByte(writer, (uint8_t)'-');
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  } else {
    magnitude = (uint32_t)value;
  }
  AppImuCapture_AppendU32(writer, magnitude);
}

static const char *AppImuCapture_EventName(AppImuCaptureEvent event)
{
  switch (event) {
    case APP_IMU_CAPTURE_EVENT_STATUS:
      return "STATUS";
    case APP_IMU_CAPTURE_EVENT_STARTED:
      return "STARTED";
    case APP_IMU_CAPTURE_EVENT_STOPPED:
      return "STOPPED";
    case APP_IMU_CAPTURE_EVENT_BEGIN:
      return "BEGIN";
    case APP_IMU_CAPTURE_EVENT_END:
      return "END";
    case APP_IMU_CAPTURE_EVENT_REJECTED:
      return "REJECTED";
    default:
      return NULL;
  }
}

static bool AppImuCapture_Finish(
  AppImuCaptureWriter *writer,
  uint16_t *length)
{
  if (writer == NULL || length == NULL || !writer->valid) {
    return false;
  }
  AppImuCapture_AppendByte(writer, (uint8_t)'\n');
  if (!writer->valid) {
    return false;
  }
  *length = writer->length;
  if (writer->length < writer->capacity) {
    writer->buffer[writer->length] = 0U;
  }
  return true;
}

void AppImuCapture_Init(AppImuCapture *capture)
{
  if (capture == NULL) {
    return;
  }
  capture->sample_count = 0U;
  capture->dropped_sample_count = 0U;
  capture->duplicate_sequence_count = 0U;
  capture->source_gap_count = 0U;
  capture->last_sequence = 0U;
  capture->state = APP_IMU_CAPTURE_IDLE;
  capture->has_last_sequence = false;
}

bool AppImuCapture_Start(AppImuCapture *capture)
{
  if (capture == NULL || capture->state == APP_IMU_CAPTURE_RECORDING) {
    return false;
  }
  AppImuCapture_Init(capture);
  capture->state = APP_IMU_CAPTURE_RECORDING;
  return true;
}

bool AppImuCapture_Stop(AppImuCapture *capture)
{
  if (capture == NULL || capture->state != APP_IMU_CAPTURE_RECORDING) {
    return false;
  }
  capture->state = APP_IMU_CAPTURE_COMPLETE;
  return true;
}

bool AppImuCapture_Record(
  AppImuCapture *capture,
  const AppImuCaptureInput *input)
{
  if (capture == NULL || input == NULL) {
    return false;
  }
  if (capture->state != APP_IMU_CAPTURE_RECORDING) {
    return true;
  }
  if (capture->has_last_sequence) {
    const uint32_t sequence_delta = input->sequence - capture->last_sequence;
    if (sequence_delta == 0U) {
      capture->duplicate_sequence_count = AppImuCapture_AddSaturated(
        capture->duplicate_sequence_count, 1U);
      return true;
    }
    if (sequence_delta > 1U) {
      capture->source_gap_count = AppImuCapture_AddSaturated(
        capture->source_gap_count, sequence_delta - 1U);
    }
  }
  if (capture->sample_count >= ROBOT_CONFIG_IMU_CAPTURE_CAPACITY) {
    capture->dropped_sample_count = AppImuCapture_AddSaturated(
      capture->dropped_sample_count, 1U);
    capture->state = APP_IMU_CAPTURE_COMPLETE;
    return true;
  }
  capture->samples[capture->sample_count] = *input;
  capture->sample_count++;
  capture->last_sequence = input->sequence;
  capture->has_last_sequence = true;
  return true;
}

bool AppImuCapture_GetStatus(
  const AppImuCapture *capture,
  AppImuCaptureStatus *status)
{
  if (capture == NULL || status == NULL) {
    return false;
  }
  status->state = capture->state;
  status->sample_count = capture->sample_count;
  status->capacity = ROBOT_CONFIG_IMU_CAPTURE_CAPACITY;
  status->dropped_sample_count = capture->dropped_sample_count;
  status->duplicate_sequence_count = capture->duplicate_sequence_count;
  status->source_gap_count = capture->source_gap_count;
  return true;
}

bool AppImuCapture_GetSample(
  const AppImuCapture *capture,
  uint32_t index,
  AppImuCaptureSample *sample)
{
  if (capture == NULL || sample == NULL ||
      capture->state == APP_IMU_CAPTURE_RECORDING ||
      index >= capture->sample_count) {
    return false;
  }
  *sample = capture->samples[index];
  return true;
}

bool AppImuCapture_FormatEvent(
  AppImuCaptureEvent event,
  const AppImuCaptureStatus *status,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length)
{
  if (status == NULL || buffer == NULL || capacity == 0U || length == NULL) {
    return false;
  }
  *length = 0U;
  const char *const name = AppImuCapture_EventName(event);
  if (name == NULL) {
    return false;
  }
  AppImuCaptureWriter writer = {
    .buffer = buffer,
    .capacity = capacity,
    .length = 0U,
    .valid = true,
  };
  AppImuCapture_AppendLiteral(&writer, "ICAP,");
  AppImuCapture_AppendU32(&writer, APP_IMU_CAPTURE_SCHEMA_VERSION);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendLiteral(&writer, name);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, (uint32_t)status->state);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, status->sample_count);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, status->capacity);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, status->dropped_sample_count);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, status->duplicate_sequence_count);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, status->source_gap_count);
  return AppImuCapture_Finish(&writer, length);
}

bool AppImuCapture_FormatSample(
  uint32_t index,
  const AppImuCaptureSample *sample,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length)
{
  if (sample == NULL || buffer == NULL || capacity == 0U || length == NULL) {
    return false;
  }
  *length = 0U;
  AppImuCaptureWriter writer = {
    .buffer = buffer,
    .capacity = capacity,
    .length = 0U,
    .valid = true,
  };
  AppImuCapture_AppendLiteral(&writer, "IC,");
  AppImuCapture_AppendU32(&writer, APP_IMU_CAPTURE_SCHEMA_VERSION);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, index);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, sample->sequence);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, sample->sensor_timestamp);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, sample->host_tick_ms);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, sample->flags);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, sample->dropped_sample_count);
  for (uint32_t i = 0U; i < 3U; ++i) {
    AppImuCapture_AppendByte(&writer, (uint8_t)',');
    AppImuCapture_AppendI32(&writer, sample->acceleration[i]);
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    AppImuCapture_AppendByte(&writer, (uint8_t)',');
    AppImuCapture_AppendI32(&writer, sample->angular_rate[i]);
  }
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendI32(&writer, sample->temperature);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, sample->sensor_status);
  AppImuCapture_AppendByte(&writer, (uint8_t)',');
  AppImuCapture_AppendU32(&writer, sample->health);
  return AppImuCapture_Finish(&writer, length);
}
