#ifndef APP_CONTROL_TIMING_H
#define APP_CONTROL_TIMING_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 控制链路时序诊断的纯计算模块。
 *
 * 本模块把 TIM7 中断时间戳、任务唤醒时间和控制周期完成时间换算成中断抖动、唤醒延迟、
 * 单周期执行时间、漏周期及截止期违约。它不依赖 HAL 或 RTOS，可直接在主机测试中覆盖
 * 计数回绕、通知合并和饱和边界。所有 cycle 字段都以 CPU 周期为单位，只有 P99 字段
 * 通过固定直方图换算为微秒。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CONTROL_TIMING_HISTOGRAM_MAX_US (100U)
#define APP_CONTROL_TIMING_HISTOGRAM_BUCKETS (APP_CONTROL_TIMING_HISTOGRAM_MAX_US + 1U)

/*
 * 可发布的时序快照。
 * “当前值”反映最近一次已登记周期，“最大值”和累计计数自初始化起单调不减直至饱和。
 * timer_irq_missed_period_count 表示硬件更新没有得到中断服务；
 * task_iteration_missed_period_count 表示中断已服务但任务没有逐周期运行；二者求和得到
 * missed_period_count，不能把两类问题混为同一个调度故障。
 */
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

/*
 * 内部统计状态，由控制任务独占读写。
 * previous_* 用于相邻节拍差分，active_* 标记正在测量的周期，wake_histogram 使用
 * 0～100 微秒固定桶保存近似分布，最后一个桶同时代表所有超过上限的样本。
 */
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

/*
 * 清零状态并建立频率换算基准。
 * cpu_hz 必须同时整除 control_rate_hz 和 1 MHz，否则无法得到整数周期与整数
 * cycles_per_us，函数返回 false。成功后可调用 RecordWake/RecordComplete。
 */
bool AppControlTiming_Init(AppControlTiming *timing, uint32_t cpu_hz, uint32_t control_rate_hz);
/*
 * 根据启动以来累计 CPU 周期计算理论更新次数，再减去实际服务中断数。
 * 这种累计相位算法能发现多个更新被折叠成一次中断、但相邻入口间隔后来又恢复正常的
 * 情况。expected_period_cycles 为零时返回 0，超过 uint32_t 的结果饱和。
 */
uint32_t AppControlTiming_CountMissedTimerPeriods(
  uint64_t elapsed_cycles_since_start,
  uint32_t expected_period_cycles,
  uint64_t serviced_irq_count);
/*
 * 在控制任务被唤醒并取得快照后调用，开始一次周期测量。
 * notification_count 用于累计通知合并，tick_sequence 用于识别任务漏迭代；函数同时
 * 更新抖动、唤醒延迟和直方图。无效或未初始化状态会被忽略，不修改统计。
 */
void AppControlTiming_RecordWake(
  AppControlTiming *timing,
  uint32_t tick_sequence,
  uint32_t irq_cycles,
  uint32_t irq_period_cycles,
  uint32_t timer_irq_missed_period_count,
  uint32_t wake_cycles,
  uint32_t notification_count);
/*
 * 在本周期所有控制工作完成后调用。
 * 执行时间从任务唤醒算到 complete_cycles；截止期从对应中断入口算到完成时刻，因此包含
 * 调度延迟。没有活动周期时调用不会重复累计。
 */
void AppControlTiming_RecordComplete(AppControlTiming *timing, uint32_t complete_cycles);
/*
 * 复制当前快照，并在副本中根据直方图计算近似 P99 唤醒延迟。
 * 函数不改变内部统计；参数无效或未初始化时返回 false。
 */
bool AppControlTiming_GetSnapshot(const AppControlTiming *timing, AppControlTimingSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
