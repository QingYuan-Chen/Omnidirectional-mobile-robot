#include "app_control_timing.h"

#include <limits.h>
#include <string.h>

/*
 * 控制时序统计保持为纯计算模块，不依赖 HAL 或 RTOS。
 * 输入时间戳都来自同一个 DWT 32 位计数域，短间隔使用无符号减法自然处理回绕；累计值
 * 采用饱和运算。这样主机测试可以构造通知合并、漏中断、计数回绕和截止期边界，而无需
 * 模拟 STM32 外设。
 */

#define APP_CONTROL_TIMING_MICROSECONDS_PER_SECOND (1000000U)

static uint32_t AppControlTiming_AddSaturated(uint32_t value, uint32_t increment)
{
  /* 所有长期诊断计数保持单调，饱和后不因回绕掩盖历史故障。 */
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
  /*
   * 先向上取整到微秒桶，避免把非零亚微秒延迟记录为 0；超过 100 us 的全部样本压入
   * 最后一桶。固定 101 个桶使 RAM 和每次更新时间有明确上界，不保存原始样本序列。
   */
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
  /* ceil(0.99*N) 选择至少覆盖 99% 样本的第一个桶；最后一桶代表“100 us 或更高”。 */
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
  /*
   * 不能只看相邻 IRQ 周期：定时器更新标志可能在长时间屏蔽中断期间多次置位但只服务
   * 一次，之后相邻间隔又恢复正常。累计相位给出启动以来应有更新总数，可保留这段历史。
   */
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
    /*
     * sequence_delta>1 表示中断快照已产生但控制任务跳过了中间迭代，属于调度/负载问题；
     * timer_irq_missed_period_count 则由时基累计相位检测，属于中断服务问题。两者分别累计。
     */
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
  /*
   * wcet_cycles 当前字段名沿用“周期执行时间”语义，记录最近一次任务从唤醒到完成的耗时；
   * wcet_max_cycles 才是观测到的最坏值。截止期从中断入口计时，因此还包含唤醒延迟。
   */
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
