#include "app_motor_capture.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

_Static_assert(sizeof(AppMotorCaptureSample) == 28U,
               "motor capture sample size must remain 28 bytes");
_Static_assert(ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY > 0U,
               "motor capture capacity must be non-zero");

typedef struct {
  uint8_t *buffer;
  uint16_t capacity;
  uint16_t length;
  bool valid;
} AppMotorCaptureWriter;

static uint32_t AppMotorCapture_AddSaturated(uint32_t value, uint32_t increment)
{
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static void AppMotorCapture_AppendByte(
  AppMotorCaptureWriter *writer,
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

static void AppMotorCapture_AppendLiteral(
  AppMotorCaptureWriter *writer,
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

static void AppMotorCapture_AppendU32(
  AppMotorCaptureWriter *writer,
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
    AppMotorCapture_AppendByte(writer, digits[count]);
  }
}

static void AppMotorCapture_AppendI32(
  AppMotorCaptureWriter *writer,
  int32_t value)
{
  uint32_t magnitude;
  if (value < 0) {
    AppMotorCapture_AppendByte(writer, (uint8_t)'-');
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  } else {
    magnitude = (uint32_t)value;
  }
  AppMotorCapture_AppendU32(writer, magnitude);
}

static const char *AppMotorCapture_EventName(AppMotorCaptureEvent event)
{
  switch (event) {
    case APP_MOTOR_CAPTURE_EVENT_STATUS:
      return "STATUS";
    case APP_MOTOR_CAPTURE_EVENT_STARTED:
      return "STARTED";
    case APP_MOTOR_CAPTURE_EVENT_STOPPED:
      return "STOPPED";
    case APP_MOTOR_CAPTURE_EVENT_BEGIN:
      return "BEGIN";
    case APP_MOTOR_CAPTURE_EVENT_END:
      return "END";
    case APP_MOTOR_CAPTURE_EVENT_REJECTED:
      return "REJECTED";
    default:
      return NULL;
  }
}

static bool AppMotorCapture_FinishWriter(
  AppMotorCaptureWriter *writer,
  uint16_t *length)
{
  if (writer == NULL || length == NULL || !writer->valid) {
    return false;
  }
  AppMotorCapture_AppendByte(writer, (uint8_t)'\n');
  if (!writer->valid) {
    return false;
  }
  *length = writer->length;
  if (writer->length < writer->capacity) {
    writer->buffer[writer->length] = 0U;
  }
  return true;
}

void AppMotorCapture_Init(AppMotorCapture *capture)
{
  if (capture != NULL) {
    capture->sample_count = 0U;
    capture->dropped_sample_count = 0U;
    capture->state = APP_MOTOR_CAPTURE_IDLE;
  }
}

bool AppMotorCapture_Start(AppMotorCapture *capture)
{
  if (capture == NULL || capture->state == APP_MOTOR_CAPTURE_RECORDING) {
    return false;
  }
  capture->sample_count = 0U;
  capture->dropped_sample_count = 0U;
  capture->state = APP_MOTOR_CAPTURE_RECORDING;
  return true;
}

bool AppMotorCapture_Stop(AppMotorCapture *capture)
{
  if (capture == NULL || capture->state != APP_MOTOR_CAPTURE_RECORDING) {
    return false;
  }
  capture->state = APP_MOTOR_CAPTURE_COMPLETE;
  return true;
}

bool AppMotorCapture_Record(
  AppMotorCapture *capture,
  const AppMotorCaptureInput *input)
{
  if (capture == NULL || input == NULL) {
    return false;
  }
  if (capture->state != APP_MOTOR_CAPTURE_RECORDING) {
    return true;
  }
  if (capture->sample_count >= ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY) {
    capture->dropped_sample_count = AppMotorCapture_AddSaturated(
      capture->dropped_sample_count, 1U);
    capture->state = APP_MOTOR_CAPTURE_COMPLETE;
    return true;
  }

  capture->samples[capture->sample_count] = *input;
  capture->sample_count++;
  return true;
}

bool AppMotorCapture_GetStatus(
  const AppMotorCapture *capture,
  AppMotorCaptureStatus *status)
{
  if (capture == NULL || status == NULL) {
    return false;
  }
  status->state = capture->state;
  status->sample_count = capture->sample_count;
  status->capacity = ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY;
  status->dropped_sample_count = capture->dropped_sample_count;
  return true;
}

bool AppMotorCapture_GetSample(
  const AppMotorCapture *capture,
  uint32_t index,
  AppMotorCaptureSample *sample)
{
  if (capture == NULL || sample == NULL ||
      capture->state == APP_MOTOR_CAPTURE_RECORDING ||
      index >= capture->sample_count) {
    return false;
  }
  *sample = capture->samples[index];
  return true;
}

bool AppMotorCapture_FormatEvent(
  AppMotorCaptureEvent event,
  const AppMotorCaptureStatus *status,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length)
{
  if (status == NULL || buffer == NULL || length == NULL || capacity == 0U) {
    return false;
  }
  *length = 0U;
  const char *const event_name = AppMotorCapture_EventName(event);
  if (event_name == NULL) {
    return false;
  }

  AppMotorCaptureWriter writer = {
    .buffer = buffer,
    .capacity = capacity,
    .length = 0U,
    .valid = true,
  };
  AppMotorCapture_AppendLiteral(&writer, "MCAP,");
  AppMotorCapture_AppendU32(&writer, APP_MOTOR_CAPTURE_SCHEMA_VERSION);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendLiteral(&writer, event_name);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, (uint32_t)status->state);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, status->sample_count);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, status->capacity);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, status->dropped_sample_count);
  return AppMotorCapture_FinishWriter(&writer, length);
}

bool AppMotorCapture_FormatSample(
  uint32_t index,
  const AppMotorCaptureSample *sample,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length)
{
  if (sample == NULL || buffer == NULL || length == NULL || capacity == 0U) {
    return false;
  }
  *length = 0U;
  AppMotorCaptureWriter writer = {
    .buffer = buffer,
    .capacity = capacity,
    .length = 0U,
    .valid = true,
  };

  AppMotorCapture_AppendLiteral(&writer, "MC,");
  AppMotorCapture_AppendU32(&writer, APP_MOTOR_CAPTURE_SCHEMA_VERSION);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, index);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->tick_sequence);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->irq_timestamp_cycles);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->wake_latency_cycles);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->previous_wcet_cycles);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->encoder_raw_ma);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendI32(&writer, sample->encoder_delta_ma);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendI32(&writer, sample->target_pwm);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendI32(&writer, sample->applied_pwm);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->battery_millivolts);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->motor_state);
  AppMotorCapture_AppendByte(&writer, (uint8_t)',');
  AppMotorCapture_AppendU32(&writer, sample->safety_flags);
  return AppMotorCapture_FinishWriter(&writer, length);
}
