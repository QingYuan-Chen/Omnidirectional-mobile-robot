#include "app_speed_capture.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(AppSpeedCaptureSample) == 28U,
               "speed capture sample size must remain 28 bytes");
_Static_assert(ROBOT_CONFIG_SPEED_CAPTURE_CAPACITY > 0U,
               "speed capture capacity must be non-zero");

typedef struct {
  uint8_t *buffer;
  uint16_t capacity;
  uint16_t length;
  bool valid;
} AppSpeedCaptureWriter;

static uint32_t AppSpeedCapture_AddSaturated(
  uint32_t value,
  uint32_t increment)
{
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static void AppSpeedCapture_AppendByte(
  AppSpeedCaptureWriter *writer,
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

static void AppSpeedCapture_AppendLiteral(
  AppSpeedCaptureWriter *writer,
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

static void AppSpeedCapture_AppendU32(
  AppSpeedCaptureWriter *writer,
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
    AppSpeedCapture_AppendByte(writer, digits[count]);
  }
}

static void AppSpeedCapture_AppendI32(
  AppSpeedCaptureWriter *writer,
  int32_t value)
{
  uint32_t magnitude;
  if (value < 0) {
    AppSpeedCapture_AppendByte(writer, (uint8_t)'-');
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  } else {
    magnitude = (uint32_t)value;
  }
  AppSpeedCapture_AppendU32(writer, magnitude);
}

static const char *AppSpeedCapture_EventName(AppSpeedCaptureEvent event)
{
  switch (event) {
    case APP_SPEED_CAPTURE_EVENT_STATUS:
      return "STATUS";
    case APP_SPEED_CAPTURE_EVENT_STARTED:
      return "STARTED";
    case APP_SPEED_CAPTURE_EVENT_STOPPED:
      return "STOPPED";
    case APP_SPEED_CAPTURE_EVENT_BEGIN:
      return "BEGIN";
    case APP_SPEED_CAPTURE_EVENT_END:
      return "END";
    case APP_SPEED_CAPTURE_EVENT_REJECTED:
      return "REJECTED";
    default:
      return NULL;
  }
}

static bool AppSpeedCapture_FinishWriter(
  AppSpeedCaptureWriter *writer,
  uint16_t *length)
{
  if (writer == NULL || length == NULL || !writer->valid) {
    return false;
  }
  AppSpeedCapture_AppendByte(writer, (uint8_t)'\n');
  if (!writer->valid) {
    return false;
  }
  *length = writer->length;
  if (writer->length < writer->capacity) {
    writer->buffer[writer->length] = 0U;
  }
  return true;
}

void AppSpeedCapture_Init(AppSpeedCapture *capture)
{
  if (capture != NULL) {
    capture->sample_count = 0U;
    capture->dropped_sample_count = 0U;
    capture->state = APP_SPEED_CAPTURE_IDLE;
    memset(&capture->period_stats, 0, sizeof(capture->period_stats));
  }
}

bool AppSpeedCapture_Start(AppSpeedCapture *capture)
{
  if (capture == NULL || capture->state == APP_SPEED_CAPTURE_RECORDING) {
    return false;
  }
  capture->sample_count = 0U;
  capture->dropped_sample_count = 0U;
  capture->state = APP_SPEED_CAPTURE_RECORDING;
  memset(&capture->period_stats, 0, sizeof(capture->period_stats));
  return true;
}

bool AppSpeedCapture_Stop(AppSpeedCapture *capture)
{
  if (capture == NULL || capture->state != APP_SPEED_CAPTURE_RECORDING) {
    return false;
  }
  capture->state = APP_SPEED_CAPTURE_COMPLETE;
  return true;
}

bool AppSpeedCapture_Record(
  AppSpeedCapture *capture,
  const AppSpeedCaptureInput *input)
{
  if (capture == NULL || input == NULL) {
    return false;
  }
  if (capture->state != APP_SPEED_CAPTURE_RECORDING) {
    return true;
  }
  if (capture->sample_count >= ROBOT_CONFIG_SPEED_CAPTURE_CAPACITY) {
    capture->dropped_sample_count = AppSpeedCapture_AddSaturated(
      capture->dropped_sample_count, 1U);
    capture->state = APP_SPEED_CAPTURE_COMPLETE;
    return true;
  }
  capture->samples[capture->sample_count] = *input;
  capture->sample_count++;
  return true;
}

bool AppSpeedCapture_SetPeriodStats(
  AppSpeedCapture *capture,
  const AppEncoderPeriodStats *stats)
{
  if (capture == NULL || stats == NULL) {
    return false;
  }
  capture->period_stats = *stats;
  return true;
}

bool AppSpeedCapture_GetStatus(
  const AppSpeedCapture *capture,
  AppSpeedCaptureStatus *status)
{
  if (capture == NULL || status == NULL) {
    return false;
  }
  status->state = capture->state;
  status->sample_count = capture->sample_count;
  status->capacity = ROBOT_CONFIG_SPEED_CAPTURE_CAPACITY;
  status->dropped_sample_count = capture->dropped_sample_count;
  status->period_stats = capture->period_stats;
  return true;
}

bool AppSpeedCapture_GetSample(
  const AppSpeedCapture *capture,
  uint32_t index,
  AppSpeedCaptureSample *sample)
{
  if (capture == NULL || sample == NULL ||
      capture->state == APP_SPEED_CAPTURE_RECORDING ||
      index >= capture->sample_count) {
    return false;
  }
  *sample = capture->samples[index];
  return true;
}

bool AppSpeedCapture_FormatEvent(
  AppSpeedCaptureEvent event,
  const AppSpeedCaptureStatus *status,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length)
{
  if (status == NULL || buffer == NULL || length == NULL || capacity == 0U) {
    return false;
  }
  *length = 0U;
  const char *const event_name = AppSpeedCapture_EventName(event);
  if (event_name == NULL) {
    return false;
  }
  AppSpeedCaptureWriter writer = {
    .buffer = buffer,
    .capacity = capacity,
    .length = 0U,
    .valid = true,
  };
  AppSpeedCapture_AppendLiteral(&writer, "SCAP,");
  AppSpeedCapture_AppendU32(&writer, APP_SPEED_CAPTURE_SCHEMA_VERSION);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendLiteral(&writer, event_name);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, (uint32_t)status->state);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, status->sample_count);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, status->capacity);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, status->dropped_sample_count);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(
    &writer, status->period_stats.invalid_direction_count);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, status->period_stats.zero_period_count);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, status->period_stats.aggregate_drop_count);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, status->period_stats.direction_reset_count);
  return AppSpeedCapture_FinishWriter(&writer, length);
}

bool AppSpeedCapture_FormatSample(
  uint32_t index,
  const AppSpeedCaptureSample *sample,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length)
{
  if (sample == NULL || buffer == NULL || length == NULL || capacity == 0U) {
    return false;
  }
  *length = 0U;
  AppSpeedCaptureWriter writer = {
    .buffer = buffer,
    .capacity = capacity,
    .length = 0U,
    .valid = true,
  };
  AppSpeedCapture_AppendLiteral(&writer, "SC,");
  AppSpeedCapture_AppendU32(&writer, APP_SPEED_CAPTURE_SCHEMA_VERSION);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, index);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, sample->tick_sequence);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, sample->irq_timestamp_cycles);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendI32(&writer, sample->encoder_delta_ma);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendI32(&writer, sample->applied_pwm);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, sample->period_sum_cycles);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, sample->period_count);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, sample->last_edge_age_cycles);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, sample->event_sequence);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendI32(&writer, sample->direction);
  AppSpeedCapture_AppendByte(&writer, (uint8_t)',');
  AppSpeedCapture_AppendU32(&writer, sample->period_flags);
  return AppSpeedCapture_FinishWriter(&writer, length);
}
