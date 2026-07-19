#ifndef APP_ENCODER_PERIOD_ACCUMULATOR_H
#define APP_ENCODER_PERIOD_ACCUMULATOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * G3 T 法使用的纯 C 边沿周期聚合器。
 *
 * 中断侧只调用 OnEdge，1 kHz 控制侧在短临界区内调用 Snapshot。模块不访问 HAL/RTOS，
 * 不逐边沿保存数组；相邻时间戳使用 uint32_t 模减法，允许 DWT->CYCCNT 自然回绕。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define APP_ENCODER_PERIOD_FLAG_HAS_EDGE         (1U << 0U)
#define APP_ENCODER_PERIOD_FLAG_HAS_PERIOD       (1U << 1U)
#define APP_ENCODER_PERIOD_FLAG_DIRECTION_RESET  (1U << 2U)
#define APP_ENCODER_PERIOD_FLAG_AGGREGATE_DROPPED (1U << 3U)
#define APP_ENCODER_PERIOD_FLAG_ZERO_PERIOD       (1U << 4U)

typedef struct {
  uint32_t period_sum_cycles;
  uint32_t last_edge_age_cycles;
  uint32_t event_sequence;
  uint16_t period_count;
  int8_t direction;
  uint8_t flags;
} AppEncoderPeriodSnapshot;

typedef struct {
  uint32_t invalid_direction_count;
  uint32_t zero_period_count;
  uint32_t aggregate_drop_count;
  uint32_t direction_reset_count;
} AppEncoderPeriodStats;

typedef struct {
  uint32_t last_edge_timestamp_cycles;
  uint32_t period_sum_cycles;
  uint32_t event_sequence;
  uint16_t period_count;
  int8_t direction;
  uint8_t pending_flags;
  bool has_edge;
  AppEncoderPeriodStats stats;
} AppEncoderPeriodAccumulator;

void AppEncoderPeriodAccumulator_Init(AppEncoderPeriodAccumulator *accumulator);

/*
 * 记录一个同定义边沿。direction 只接受 -1 或 +1。
 * 首个边沿仅建立基线；换向会丢弃尚未快照的旧方向周期，禁止旧周期泄漏到新方向。
 */
bool AppEncoderPeriodAccumulator_OnEdge(
  AppEncoderPeriodAccumulator *accumulator,
  uint32_t timestamp_cycles,
  int8_t direction);

/*
 * 复制当前聚合并清空“本控制周期”的 period_sum/period_count/事件标志。
 * 最后边沿、方向、事件序号和累计统计保留，供后续年龄与陈旧判断使用。
 */
bool AppEncoderPeriodAccumulator_Snapshot(
  AppEncoderPeriodAccumulator *accumulator,
  uint32_t now_cycles,
  AppEncoderPeriodSnapshot *snapshot);

bool AppEncoderPeriodAccumulator_GetStats(
  const AppEncoderPeriodAccumulator *accumulator,
  AppEncoderPeriodStats *stats);

#ifdef __cplusplus
}
#endif

#endif
