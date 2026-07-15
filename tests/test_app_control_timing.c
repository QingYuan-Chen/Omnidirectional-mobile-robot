#include "app_control_timing.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>

static void TestInitialization(void)
{
  AppControlTiming timing;
  assert(!AppControlTiming_Init(NULL, 168000000U, 1000U));
  assert(!AppControlTiming_Init(&timing, 0U, 1000U));
  assert(!AppControlTiming_Init(&timing, 168000000U, 0U));
  assert(!AppControlTiming_Init(&timing, 168000001U, 1000U));
  assert(AppControlTiming_Init(&timing, 168000000U, 1000U));
  assert(timing.expected_period_cycles == 168000U);
  assert(timing.cycles_per_us == 168U);
}

static void TestWrapAndCounters(void)
{
  AppControlTiming timing;
  AppControlTimingSnapshot snapshot;
  assert(AppControlTiming_Init(&timing, 168000000U, 1000U));

  AppControlTiming_RecordWake(&timing, UINT32_MAX, UINT32_MAX - 99U, 168000U, 0U, 50U, 1U);
  AppControlTiming_RecordComplete(&timing, 100U);
  assert(AppControlTiming_GetSnapshot(&timing, &snapshot));
  assert(snapshot.wake_latency_cycles == 150U);
  assert(snapshot.wcet_cycles == 50U);
  assert(snapshot.deadline_miss_count == 0U);

  AppControlTiming_RecordWake(&timing, 0U, 167900U, 168000U, 0U, 168068U, 2U);
  AppControlTiming_RecordComplete(&timing, 168200U);
  assert(AppControlTiming_GetSnapshot(&timing, &snapshot));
  assert(snapshot.actual_dt_cycles == 168000U);
  assert(snapshot.notification_coalesced_count == 1U);
  assert(snapshot.missed_period_count == 0U);

  AppControlTiming_RecordWake(&timing, 2U, 503900U, 168000U, 0U, 504068U, 2U);
  AppControlTiming_RecordComplete(&timing, 504200U);
  assert(AppControlTiming_GetSnapshot(&timing, &snapshot));
  assert(snapshot.missed_period_count == 1U);
  assert(snapshot.task_iteration_missed_period_count == 1U);
  assert(snapshot.timer_irq_missed_period_count == 0U);
  assert(snapshot.notification_coalesced_count == 2U);

  AppControlTiming_RecordWake(&timing, 3U, 671900U, 336000U, 1U, 672068U, 1U);
  AppControlTiming_RecordComplete(&timing, 839900U);
  assert(AppControlTiming_GetSnapshot(&timing, &snapshot));
  assert(snapshot.timer_irq_missed_period_count == 1U);
  assert(snapshot.missed_period_count == 2U);
  assert(snapshot.deadline_miss_count == 1U);
}

static void TestJitterAndP99(void)
{
  AppControlTiming timing;
  AppControlTimingSnapshot snapshot;
  assert(AppControlTiming_Init(&timing, 168000000U, 1000U));

  uint32_t irq_cycles = 1000U;
  for (uint32_t i = 0U; i < 99U; ++i) {
    AppControlTiming_RecordWake(&timing, i + 1U, irq_cycles, 168010U, 0U, irq_cycles + 1680U, 1U);
    AppControlTiming_RecordComplete(&timing, irq_cycles + 2000U);
    irq_cycles += 168000U;
  }
  AppControlTiming_RecordWake(&timing, 100U, irq_cycles, 168010U, 0U, irq_cycles + 16800U, 1U);
  AppControlTiming_RecordComplete(&timing, irq_cycles + 17000U);

  assert(AppControlTiming_GetSnapshot(&timing, &snapshot));
  assert(snapshot.irq_jitter_cycles == 10U);
  assert(snapshot.irq_jitter_max_cycles == 10U);
  assert(snapshot.wake_latency_p99_us == 10U);

  irq_cycles += 168000U;
  for (uint32_t i = 0U; i < 2U; ++i) {
    AppControlTiming_RecordWake(&timing, 101U + i, irq_cycles, 168000U, 0U, irq_cycles + 16800U, 1U);
    AppControlTiming_RecordComplete(&timing, irq_cycles + 17000U);
    irq_cycles += 168000U;
  }
  assert(AppControlTiming_GetSnapshot(&timing, &snapshot));
  assert(snapshot.wake_latency_p99_us == APP_CONTROL_TIMING_HISTOGRAM_MAX_US);
}

static void TestP99RoundsUpAtBoundary(void)
{
  AppControlTiming timing;
  AppControlTimingSnapshot snapshot;
  assert(AppControlTiming_Init(&timing, 168000000U, 1000U));

  AppControlTiming_RecordWake(&timing, 1U, 1000U, 0U, 0U, 9401U, 1U);
  AppControlTiming_RecordComplete(&timing, 9500U);
  assert(AppControlTiming_GetSnapshot(&timing, &snapshot));
  assert(snapshot.wake_latency_p99_us == 51U);
}

static void TestMissedTimerPeriodCounting(void)
{
  assert(AppControlTiming_CountMissedTimerPeriods(0U, 168000U, 0U) == 0U);
  assert(AppControlTiming_CountMissedTimerPeriods(335999U, 168000U, 1U) == 0U);
  assert(AppControlTiming_CountMissedTimerPeriods(336000U, 168000U, 1U) == 1U);
  assert(AppControlTiming_CountMissedTimerPeriods(504123U, 168000U, 1U) == 2U);
  assert(AppControlTiming_CountMissedTimerPeriods(UINT64_MAX, 0U, 0U) == 0U);

  /* 前次 ISR 晚到 0.9T，随后 UIF 折叠；相邻入口仅 1.1T，累计相位仍检测到缺失。 */
  const uint64_t first_irq_elapsed = 319200U;
  assert(AppControlTiming_CountMissedTimerPeriods(first_irq_elapsed, 168000U, 1U) == 0U);
  const uint64_t second_irq_elapsed = first_irq_elapsed + 184800U;
  assert(AppControlTiming_CountMissedTimerPeriods(second_irq_elapsed, 168000U, 2U) == 1U);
}

int main(void)
{
  TestInitialization();
  TestWrapAndCounters();
  TestJitterAndP99();
  TestP99RoundsUpAtBoundary();
  TestMissedTimerPeriodCounting();
  return 0;
}
