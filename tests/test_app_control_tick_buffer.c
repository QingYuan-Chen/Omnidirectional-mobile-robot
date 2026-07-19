#include "app_control_tick_buffer.h"

#include <assert.h>
#include <limits.h>

static void FillRaw(uint16_t raw[BSP_MOTOR_COUNT], uint16_t value)
{
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    raw[i] = value;
  }
}

static AppEncoderPeriodSnapshot MakePeriod(uint32_t event_sequence)
{
  const AppEncoderPeriodSnapshot period = {
    .period_sum_cycles = event_sequence * 10U,
    .last_edge_age_cycles = event_sequence,
    .event_sequence = event_sequence,
    .period_count = (uint16_t)event_sequence,
    .direction = 1,
    .flags = APP_ENCODER_PERIOD_FLAG_HAS_EDGE,
  };
  return period;
}

static void TestNotificationSnapshotRace(void)
{
  AppControlTickBuffer buffer;
  AppControlTickSample sample;
  uint16_t raw[BSP_MOTOR_COUNT];
  AppControlTickBuffer_Init(&buffer);

  FillRaw(raw, 11U);
  AppEncoderPeriodSnapshot period = MakePeriod(1U);
  AppControlTickBuffer_PublishFromIsr(
    &buffer, 100U, 0U, 0U, &period, raw);

  /* 模拟任务已取走第 1 个通知，但复制快照前第 2 次 ISR 到达。 */
  FillRaw(raw, 22U);
  period = MakePeriod(2U);
  AppControlTickBuffer_PublishFromIsr(
    &buffer, 200U, 100U, 0U, &period, raw);

  assert(AppControlTickBuffer_Consume(&buffer, 1U, &sample));
  assert(sample.tick_sequence == 1U);
  assert(sample.irq_timestamp_cycles == 100U);
  assert(sample.encoder_raw[BSP_MOTOR_MA] == 11U);
  assert(sample.encoder_period_ma.event_sequence == 1U);

  assert(AppControlTickBuffer_Consume(&buffer, 1U, &sample));
  assert(sample.tick_sequence == 2U);
  assert(sample.irq_timestamp_cycles == 200U);
  assert(sample.encoder_raw[BSP_MOTOR_MA] == 22U);
  assert(sample.encoder_period_ma.event_sequence == 2U);
}

static void TestCoalescingOverwriteAndWrap(void)
{
  AppControlTickBuffer buffer;
  AppControlTickSample sample;
  uint16_t raw[BSP_MOTOR_COUNT];
  AppControlTickBuffer_Init(&buffer);

  for (uint16_t value = 1U; value <= 3U; ++value) {
    FillRaw(raw, value);
    const AppEncoderPeriodSnapshot period = MakePeriod(value);
    AppControlTickBuffer_PublishFromIsr(
      &buffer, value, 1U, 0U, &period, raw);
  }
  assert(AppControlTickBuffer_Consume(&buffer, 3U, &sample));
  assert(sample.tick_sequence == 3U);
  assert(sample.encoder_raw[BSP_MOTOR_MA] == 3U);

  AppControlTickBuffer_Init(&buffer);
  FillRaw(raw, 1U);
  AppEncoderPeriodSnapshot period = MakePeriod(1U);
  AppControlTickBuffer_PublishFromIsr(
    &buffer, 1U, 0U, 0U, &period, raw);
  for (uint32_t i = 0U; i < ROBOT_CONFIG_CONTROL_TICK_RING_SIZE; ++i) {
    FillRaw(raw, (uint16_t)(i + 2U));
    period = MakePeriod(i + 2U);
    AppControlTickBuffer_PublishFromIsr(
      &buffer, i + 2U, 1U, 0U, &period, raw);
  }
  assert(!AppControlTickBuffer_Consume(&buffer, 1U, &sample));

  AppControlTickBuffer_Init(&buffer);
  buffer.produced_sequence = UINT32_MAX - 1U;
  buffer.consumed_sequence = UINT32_MAX - 1U;
  FillRaw(raw, 7U);
  period = MakePeriod(7U);
  AppControlTickBuffer_PublishFromIsr(
    &buffer, 7U, 1U, 0U, &period, raw);
  assert(AppControlTickBuffer_Consume(&buffer, 1U, &sample));
  assert(sample.tick_sequence == UINT32_MAX);
  FillRaw(raw, 8U);
  period = MakePeriod(8U);
  AppControlTickBuffer_PublishFromIsr(
    &buffer, 8U, 1U, 0U, &period, raw);
  assert(AppControlTickBuffer_Consume(&buffer, 1U, &sample));
  assert(sample.tick_sequence == 0U);
}

int main(void)
{
  TestNotificationSnapshotRace();
  TestCoalescingOverwriteAndWrap();
  return 0;
}
