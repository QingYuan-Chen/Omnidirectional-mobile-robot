#include "app_encoder_period_accumulator.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static uint32_t AppEncoderPeriod_AddSaturated(
  uint32_t value,
  uint32_t increment)
{
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

void AppEncoderPeriodAccumulator_Init(AppEncoderPeriodAccumulator *accumulator)
{
  if (accumulator != NULL) {
    memset(accumulator, 0, sizeof(*accumulator));
  }
}

bool AppEncoderPeriodAccumulator_OnEdge(
  AppEncoderPeriodAccumulator *accumulator,
  uint32_t timestamp_cycles,
  int8_t direction)
{
  if (accumulator == NULL || (direction != -1 && direction != 1)) {
    if (accumulator != NULL) {
      accumulator->stats.invalid_direction_count =
        AppEncoderPeriod_AddSaturated(
          accumulator->stats.invalid_direction_count, 1U);
    }
    return false;
  }

  accumulator->event_sequence++;
  if (!accumulator->has_edge) {
    accumulator->last_edge_timestamp_cycles = timestamp_cycles;
    accumulator->direction = direction;
    accumulator->has_edge = true;
    accumulator->pending_flags |= APP_ENCODER_PERIOD_FLAG_HAS_EDGE;
    return true;
  }

  if (direction != accumulator->direction) {
    accumulator->period_sum_cycles = 0U;
    accumulator->period_count = 0U;
    accumulator->last_edge_timestamp_cycles = timestamp_cycles;
    accumulator->direction = direction;
    accumulator->pending_flags |=
      APP_ENCODER_PERIOD_FLAG_HAS_EDGE |
      APP_ENCODER_PERIOD_FLAG_DIRECTION_RESET;
    accumulator->stats.direction_reset_count = AppEncoderPeriod_AddSaturated(
      accumulator->stats.direction_reset_count, 1U);
    return true;
  }

  const uint32_t period_cycles =
    timestamp_cycles - accumulator->last_edge_timestamp_cycles;
  accumulator->last_edge_timestamp_cycles = timestamp_cycles;
  accumulator->pending_flags |= APP_ENCODER_PERIOD_FLAG_HAS_EDGE;
  if (period_cycles == 0U) {
    accumulator->pending_flags |= APP_ENCODER_PERIOD_FLAG_ZERO_PERIOD;
    accumulator->stats.zero_period_count = AppEncoderPeriod_AddSaturated(
      accumulator->stats.zero_period_count, 1U);
    return false;
  }

  if (accumulator->period_count == UINT16_MAX ||
      period_cycles > (UINT32_MAX - accumulator->period_sum_cycles)) {
    AppEncoderPeriodAccumulator_MarkAggregateDropped(accumulator);
    return false;
  }

  accumulator->period_sum_cycles += period_cycles;
  accumulator->period_count++;
  accumulator->pending_flags |= APP_ENCODER_PERIOD_FLAG_HAS_PERIOD;
  return true;
}

void AppEncoderPeriodAccumulator_MarkAggregateDropped(
  AppEncoderPeriodAccumulator *accumulator)
{
  if (accumulator == NULL) {
    return;
  }
  accumulator->period_sum_cycles = 0U;
  accumulator->period_count = 0U;
  accumulator->has_edge = false;
  accumulator->direction = 0;
  accumulator->pending_flags &=
    (uint8_t)~(
      APP_ENCODER_PERIOD_FLAG_HAS_EDGE |
      APP_ENCODER_PERIOD_FLAG_HAS_PERIOD);
  accumulator->pending_flags |= APP_ENCODER_PERIOD_FLAG_AGGREGATE_DROPPED;
  accumulator->stats.aggregate_drop_count = AppEncoderPeriod_AddSaturated(
    accumulator->stats.aggregate_drop_count, 1U);
}

bool AppEncoderPeriodAccumulator_Snapshot(
  AppEncoderPeriodAccumulator *accumulator,
  uint32_t now_cycles,
  AppEncoderPeriodSnapshot *snapshot)
{
  if (accumulator == NULL || snapshot == NULL) {
    return false;
  }

  snapshot->period_sum_cycles = accumulator->period_sum_cycles;
  snapshot->last_edge_age_cycles = accumulator->has_edge
                                     ? now_cycles -
                                         accumulator->last_edge_timestamp_cycles
                                     : 0U;
  snapshot->event_sequence = accumulator->event_sequence;
  snapshot->period_count = accumulator->period_count;
  snapshot->direction = accumulator->direction;
  snapshot->flags = accumulator->pending_flags;

  accumulator->period_sum_cycles = 0U;
  accumulator->period_count = 0U;
  accumulator->pending_flags = accumulator->has_edge
                                 ? APP_ENCODER_PERIOD_FLAG_HAS_EDGE
                                 : 0U;
  return true;
}

bool AppEncoderPeriodAccumulator_GetStats(
  const AppEncoderPeriodAccumulator *accumulator,
  AppEncoderPeriodStats *stats)
{
  if (accumulator == NULL || stats == NULL) {
    return false;
  }
  *stats = accumulator->stats;
  return true;
}
