#include "app_control_timing.h"

#include <limits.h>
#include <string.h>

#define APP_CONTROL_TIMING_MICROSECONDS_PER_SECOND (1000000U)

static uint32_t AppControlTiming_AddSaturated(uint32_t value, uint32_t increment)
{
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static uint32_t AppControlTiming_AbsoluteDifference(uint32_t first, uint32_t second)
{
  return first >= second ? first - second : second - first;
}

static void AppControlTiming_RecordWakeHistogram(AppControlTiming *timing, uint32_t wake_latency_cycles)
{
  uint32_t bucket = wake_latency_cycles / timing->cycles_per_us;
  if ((wake_latency_cycles % timing->cycles_per_us) != 0U) {
    bucket++;
  }
  if (bucket >= APP_CONTROL_TIMING_HISTOGRAM_BUCKETS) {
    bucket = APP_CONTROL_TIMING_HISTOGRAM_BUCKETS - 1U;
  }
  timing->wake_histogram[bucket] = AppControlTiming_AddSaturated(timing->wake_histogram[bucket], 1U);
}

static uint32_t AppControlTiming_CalculateP99Us(const AppControlTiming *timing)
{
  uint64_t sample_total = 0U;
  for (uint32_t i = 0U; i < APP_CONTROL_TIMING_HISTOGRAM_BUCKETS; ++i) {
    sample_total += (uint64_t)timing->wake_histogram[i];
  }
  if (sample_total == 0U) {
    return 0U;
  }

  const uint64_t threshold = ((sample_total * 99U) + 99U) / 100U;
  uint64_t cumulative = 0U;
  for (uint32_t i = 0U; i < APP_CONTROL_TIMING_HISTOGRAM_BUCKETS; ++i) {
    cumulative += (uint64_t)timing->wake_histogram[i];
    if (cumulative >= threshold) {
      return i;
    }
  }

  return APP_CONTROL_TIMING_HISTOGRAM_MAX_US;
}

bool AppControlTiming_Init(AppControlTiming *timing, uint32_t cpu_hz, uint32_t control_rate_hz)
{
  if (timing == NULL || cpu_hz == 0U || control_rate_hz == 0U ||
      (cpu_hz % control_rate_hz) != 0U ||
      (cpu_hz % APP_CONTROL_TIMING_MICROSECONDS_PER_SECOND) != 0U) {
    return false;
  }

  memset(timing, 0, sizeof(*timing));
  timing->expected_period_cycles = cpu_hz / control_rate_hz;
  timing->cycles_per_us = cpu_hz / APP_CONTROL_TIMING_MICROSECONDS_PER_SECOND;
  return timing->expected_period_cycles != 0U && timing->cycles_per_us != 0U;
}

uint32_t AppControlTiming_CountMissedTimerPeriods(
  uint64_t elapsed_cycles_since_start,
  uint32_t expected_period_cycles,
  uint64_t serviced_irq_count)
{
  if (expected_period_cycles == 0U) {
    return 0U;
  }
  const uint64_t expected_update_count =
    elapsed_cycles_since_start / (uint64_t)expected_period_cycles;
  if (expected_update_count <= serviced_irq_count) {
    return 0U;
  }

  const uint64_t missed_periods = expected_update_count - serviced_irq_count;
  return missed_periods > UINT32_MAX ? UINT32_MAX : (uint32_t)missed_periods;
}

void AppControlTiming_RecordWake(
  AppControlTiming *timing,
  uint32_t tick_sequence,
  uint32_t irq_cycles,
  uint32_t irq_period_cycles,
  uint32_t timer_irq_missed_period_count,
  uint32_t wake_cycles,
  uint32_t notification_count)
{
  if (timing == NULL || timing->expected_period_cycles == 0U || timing->cycles_per_us == 0U) {
    return;
  }

  AppControlTimingSnapshot *const snapshot = &timing->snapshot;
  snapshot->tick_sequence = tick_sequence;
  snapshot->irq_timestamp_cycles = irq_cycles;
  snapshot->irq_period_cycles = irq_period_cycles;
  snapshot->wake_latency_cycles = wake_cycles - irq_cycles;
  if (snapshot->wake_latency_cycles > snapshot->wake_latency_max_cycles) {
    snapshot->wake_latency_max_cycles = snapshot->wake_latency_cycles;
  }

  if (irq_period_cycles != 0U) {
    snapshot->irq_jitter_cycles =
      AppControlTiming_AbsoluteDifference(irq_period_cycles, timing->expected_period_cycles);
    if (snapshot->irq_jitter_cycles > snapshot->irq_jitter_max_cycles) {
      snapshot->irq_jitter_max_cycles = snapshot->irq_jitter_cycles;
    }
  } else {
    snapshot->irq_jitter_cycles = 0U;
  }

  if (timing->has_previous_tick) {
    const uint32_t sequence_delta = tick_sequence - timing->previous_tick_sequence;
    snapshot->actual_dt_cycles = irq_cycles - timing->previous_irq_cycles;
    if (sequence_delta > 1U) {
      snapshot->task_iteration_missed_period_count = AppControlTiming_AddSaturated(
        snapshot->task_iteration_missed_period_count, sequence_delta - 1U);
    }
  } else {
    snapshot->actual_dt_cycles = 0U;
    if (notification_count > 1U) {
      snapshot->task_iteration_missed_period_count = notification_count - 1U;
    }
    timing->has_previous_tick = true;
  }

  snapshot->timer_irq_missed_period_count = timer_irq_missed_period_count;
  snapshot->missed_period_count = AppControlTiming_AddSaturated(
    snapshot->timer_irq_missed_period_count,
    snapshot->task_iteration_missed_period_count);

  if (notification_count > 1U) {
    snapshot->notification_coalesced_count =
      AppControlTiming_AddSaturated(snapshot->notification_coalesced_count, notification_count - 1U);
  }

  AppControlTiming_RecordWakeHistogram(timing, snapshot->wake_latency_cycles);
  snapshot->sample_count = AppControlTiming_AddSaturated(snapshot->sample_count, 1U);
  timing->previous_irq_cycles = irq_cycles;
  timing->previous_tick_sequence = tick_sequence;
  timing->active_irq_cycles = irq_cycles;
  timing->active_wake_cycles = wake_cycles;
  timing->cycle_active = true;
}

void AppControlTiming_RecordComplete(AppControlTiming *timing, uint32_t complete_cycles)
{
  if (timing == NULL || !timing->cycle_active) {
    return;
  }

  AppControlTimingSnapshot *const snapshot = &timing->snapshot;
  snapshot->wcet_cycles = complete_cycles - timing->active_wake_cycles;
  if (snapshot->wcet_cycles > snapshot->wcet_max_cycles) {
    snapshot->wcet_max_cycles = snapshot->wcet_cycles;
  }
  if ((complete_cycles - timing->active_irq_cycles) >= timing->expected_period_cycles) {
    snapshot->deadline_miss_count = AppControlTiming_AddSaturated(snapshot->deadline_miss_count, 1U);
  }
  timing->cycle_active = false;
}

bool AppControlTiming_GetSnapshot(const AppControlTiming *timing, AppControlTimingSnapshot *snapshot)
{
  if (timing == NULL || snapshot == NULL || timing->expected_period_cycles == 0U) {
    return false;
  }

  *snapshot = timing->snapshot;
  snapshot->wake_latency_p99_us = AppControlTiming_CalculateP99Us(timing);
  return true;
}
