#include "app_telemetry.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/* 遥测格式化器使用固定标签和整数文本，保证内存占用及最坏帧长可计算。 */

typedef struct {
  uint8_t *buffer;
  uint16_t capacity;
  uint16_t length;
  bool valid;
} AppTelemetryWriter;

static void AppTelemetry_AppendByte(AppTelemetryWriter *writer, uint8_t value)
{
  if (!writer->valid || writer->length >= writer->capacity) {
    writer->valid = false;
    return;
  }
  writer->buffer[writer->length] = value;
  writer->length++;
}

static void AppTelemetry_AppendLiteral(
  AppTelemetryWriter *writer,
  const char *literal)
{
  if (!writer->valid || literal == NULL) {
    writer->valid = false;
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

static void AppTelemetry_AppendU64(AppTelemetryWriter *writer, uint64_t value)
{
  /* 手工整数转换使帧长和执行路径可界定，不引入格式化库及浮点支持。 */
  uint8_t digits[20];
  uint16_t count = 0U;
  do {
    digits[count] = (uint8_t)('0' + (uint8_t)(value % 10U));
    count++;
    value /= 10U;
  } while (value != 0U);

  while (count > 0U) {
    count--;
    AppTelemetry_AppendByte(writer, digits[count]);
  }
}

static void AppTelemetry_AppendU32(AppTelemetryWriter *writer, uint32_t value)
{
  AppTelemetry_AppendU64(writer, value);
}

static void AppTelemetry_AppendI64(AppTelemetryWriter *writer, int64_t value)
{
  uint64_t magnitude;
  if (value < 0) {
    AppTelemetry_AppendByte(writer, (uint8_t)'-');
    magnitude = (uint64_t)(-(value + 1)) + 1U;
  } else {
    magnitude = (uint64_t)value;
  }
  AppTelemetry_AppendU64(writer, magnitude);
}

static void AppTelemetry_AppendI32(AppTelemetryWriter *writer, int32_t value)
{
  AppTelemetry_AppendI64(writer, value);
}

static void AppTelemetry_AppendEncoderU16(
  AppTelemetryWriter *writer,
  const uint16_t values[BSP_MOTOR_COUNT])
{
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    AppTelemetry_AppendByte(writer, (uint8_t)',');
    AppTelemetry_AppendU32(writer, values[i]);
  }
}

static void AppTelemetry_AppendEncoderI16(
  AppTelemetryWriter *writer,
  const int16_t values[BSP_MOTOR_COUNT])
{
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    AppTelemetry_AppendByte(writer, (uint8_t)',');
    AppTelemetry_AppendI32(writer, values[i]);
  }
}

static void AppTelemetry_AppendEncoderI64(
  AppTelemetryWriter *writer,
  const int64_t values[BSP_MOTOR_COUNT])
{
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    AppTelemetry_AppendByte(writer, (uint8_t)',');
    AppTelemetry_AppendI64(writer, values[i]);
  }
}

bool AppTelemetry_Format(
  const AppTelemetryInput *input,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length)
{
  if (input == NULL || buffer == NULL || capacity == 0U || length == NULL) {
    return false;
  }
  *length = 0U;
  AppTelemetryWriter writer = {
    .buffer = buffer,
    .capacity = capacity,
    .length = 0U,
    .valid = true,
  };

  /* 标签顺序固定，树莓派端可按字段组解析并检查帧预算。 */
  AppTelemetry_AppendLiteral(&writer, "T,");
  AppTelemetry_AppendU32(&writer, input->now_ms);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->control_timing.tick_sequence);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->control_timing.irq_timestamp_cycles);
  AppTelemetry_AppendLiteral(&writer, ",R");
  AppTelemetry_AppendEncoderU16(&writer, input->encoder_raw);
  AppTelemetry_AppendLiteral(&writer, ",D");
  AppTelemetry_AppendEncoderI16(&writer, input->encoder_delta);
  AppTelemetry_AppendLiteral(&writer, ",E");
  AppTelemetry_AppendEncoderI64(&writer, input->encoder_total);
  AppTelemetry_AppendLiteral(&writer, ",P,");
  AppTelemetry_AppendI32(&writer, input->motor.target_pwm);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendI32(&writer, input->motor.applied_pwm);
  AppTelemetry_AppendLiteral(&writer, ",B,");
  AppTelemetry_AppendU32(&writer, input->battery_millivolts);
  AppTelemetry_AppendLiteral(&writer, ",S,");
  AppTelemetry_AppendU32(&writer, input->critical_tasks_alive ? 1U : 0U);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->motion_inhibited ? 1U : 0U);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->fault_latched ? 1U : 0U);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, (uint32_t)input->motor.state);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(
    &writer,
    (input->fault_latched ||
     input->motor.state == APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED)
      ? 1U
      : 0U);
  AppTelemetry_AppendLiteral(&writer, ",I,");
  AppTelemetry_AppendU32(&writer, input->imu_sample_age_ms);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, (uint32_t)input->imu_health);
  AppTelemetry_AppendLiteral(&writer, ",J,");
  AppTelemetry_AppendU32(&writer, input->control_timing.irq_jitter_cycles);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->control_timing.irq_jitter_max_cycles);
  AppTelemetry_AppendLiteral(&writer, ",L,");
  AppTelemetry_AppendU32(&writer, input->control_timing.wake_latency_cycles);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->control_timing.wcet_max_cycles);
  AppTelemetry_AppendLiteral(&writer, ",M,");
  AppTelemetry_AppendU32(&writer, input->control_timing.missed_period_count);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->control_timing.deadline_miss_count);
  AppTelemetry_AppendLiteral(&writer, ",C,");
  AppTelemetry_AppendU32(&writer, input->uart_error_count);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->uart_rx_overflow_count);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->uart_tx_fault_count);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->command_reject_count);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->command_queue_drop_count);
  AppTelemetry_AppendByte(&writer, (uint8_t)',');
  AppTelemetry_AppendU32(&writer, input->adc_error_count);
  AppTelemetry_AppendByte(&writer, (uint8_t)'\n');

  if (!writer.valid) {
    return false;
  }
  *length = writer.length;
  if (writer.length < writer.capacity) {
    writer.buffer[writer.length] = 0U;
  }
  return true;
}
