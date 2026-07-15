#include "app_telemetry.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * 遥测格式化器把一致运行快照编码为固定标签、逗号分隔、换行结束的整数文本帧。
 * 所有追加函数共享 writer.valid 失败锁存：一旦容量不足，后续追加都变成空操作，最终
 * Format 返回 false 且 length 保持零，避免调用者误发截断帧。实现不使用 printf，因而
 * 不引入不可控格式化开销、浮点支持或动态内存。
 */

typedef struct {
  /* capacity 是缓冲区总可写字节数，length 始终指向下一写入位置，valid 为整帧事务状态。 */
  uint8_t *buffer;
  uint16_t capacity;
  uint16_t length;
  bool valid;
} AppTelemetryWriter;

static void AppTelemetry_AppendByte(AppTelemetryWriter *writer, uint8_t value)
{
  /* 最小写入原语负责唯一的逐字节边界检查，失败后锁存 valid=false。 */
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
  /* 字面量先整体检查剩余容量，再一次 memcpy，保证不会只写入标签前缀。 */
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
  /*
   * uint64_t 最多 20 个十进制数字，先逆序放入固定局部数组，再反向追加。do-while 保证
   * 数值 0 也输出一个字符。每个最终字节仍经过统一容量检查。
   */
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
  /* -(INT64_MIN) 会溢出，使用 -(value+1)+1 在无符号域安全取得其幅值。 */
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

  /*
   * 帧布局固定如下：
   * T=主机时刻/控制节拍/IRQ 时间，R=四路原始计数，D=四路周期增量，E=四路累计计数，
   * P=目标/实际 PWM，B=电池毫伏，S=系统安全与电机状态，I=IMU 年龄/健康，
   * J=中断抖动，L=唤醒延迟/最大执行时间，M=漏周期/超期，C=通信与 ADC 错误。
   * 标签和字段顺序变化属于线协议变更，必须同步树莓派直接解析端及其转发的电脑调参端。
   */
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
    /* 失败时不暴露部分长度；调用者据此丢弃整帧并累计 format_error。 */
    return false;
  }
  *length = writer.length;
  if (writer.length < writer.capacity) {
    /* 零终止符只方便本地调试查看，不属于 UART 帧，发送长度仍以换行结尾。 */
    writer.buffer[writer.length] = 0U;
  }
  return true;
}
