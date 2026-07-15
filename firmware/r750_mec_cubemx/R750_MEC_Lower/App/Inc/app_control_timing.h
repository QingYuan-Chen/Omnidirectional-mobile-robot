#ifndef APP_CONTROL_TIMING_H
#define APP_CONTROL_TIMING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CONTROL_TIMING_HISTOGRAM_MAX_US (100U)
#define APP_CONTROL_TIMING_HISTOGRAM_BUCKETS (APP_CONTROL_TIMING_HISTOGRAM_MAX_US + 1U)

typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t actual_dt_cycles;
  uint32_t irq_period_cycles;
  uint32_t irq_jitter_cycles;
  uint32_t irq_jitter_max_cycles;
  uint32_t wake_latency_cycles;
  uint32_t wake_latency_max_cycles;
  uint32_t wake_latency_p99_us;
  uint32_t wcet_cycles;
  uint32_t wcet_max_cycles;
  uint32_t notification_coalesced_count;
  uint32_t timer_irq_missed_period_count;
  uint32_t task_iteration_missed_period_count;
  uint32_t missed_period_count;
  uint32_t deadline_miss_count;
  uint32_t sample_count;
} AppControlTimingSnapshot;

typedef struct {
  AppControlTimingSnapshot snapshot;
  uint32_t expected_period_cycles;
  uint32_t cycles_per_us;
  uint32_t previous_irq_cycles;
  uint32_t previous_tick_sequence;
  uint32_t active_irq_cycles;
  uint32_t active_wake_cycles;
  uint32_t wake_histogram[APP_CONTROL_TIMING_HISTOGRAM_BUCKETS];
  bool has_previous_tick;
  bool cycle_active;
} AppControlTiming;

bool AppControlTiming_Init(AppControlTiming *timing, uint32_t cpu_hz, uint32_t control_rate_hz);
uint32_t AppControlTiming_CountMissedTimerPeriods(
  uint64_t elapsed_cycles_since_start,
  uint32_t expected_period_cycles,
  uint64_t serviced_irq_count);
void AppControlTiming_RecordWake(
  AppControlTiming *timing,
  uint32_t tick_sequence,
  uint32_t irq_cycles,
  uint32_t irq_period_cycles,
  uint32_t timer_irq_missed_period_count,
  uint32_t wake_cycles,
  uint32_t notification_count);
void AppControlTiming_RecordComplete(AppControlTiming *timing, uint32_t complete_cycles);
bool AppControlTiming_GetSnapshot(const AppControlTiming *timing, AppControlTimingSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
