#include "app_encoder_period_accumulator.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>

static void TestWrapSnapshotAndDirectionReset(void)
{
  AppEncoderPeriodAccumulator accumulator;
  AppEncoderPeriodSnapshot snapshot;
  AppEncoderPeriodStats stats;

  AppEncoderPeriodAccumulator_Init(NULL);
  AppEncoderPeriodAccumulator_Init(&accumulator);
  assert(!AppEncoderPeriodAccumulator_OnEdge(NULL, 0U, 1));
  assert(!AppEncoderPeriodAccumulator_OnEdge(&accumulator, 0U, 0));
  assert(AppEncoderPeriodAccumulator_OnEdge(
    &accumulator, UINT32_MAX - 9U, 1));
  assert(AppEncoderPeriodAccumulator_OnEdge(&accumulator, 10U, 1));
  assert(AppEncoderPeriodAccumulator_Snapshot(&accumulator, 15U, &snapshot));
  assert(snapshot.period_sum_cycles == 20U);
  assert(snapshot.period_count == 1U);
  assert(snapshot.last_edge_age_cycles == 5U);
  assert(snapshot.event_sequence == 2U);
  assert(snapshot.direction == 1);
  assert((snapshot.flags & APP_ENCODER_PERIOD_FLAG_HAS_EDGE) != 0U);
  assert((snapshot.flags & APP_ENCODER_PERIOD_FLAG_HAS_PERIOD) != 0U);

  assert(AppEncoderPeriodAccumulator_Snapshot(&accumulator, 25U, &snapshot));
  assert(snapshot.period_sum_cycles == 0U);
  assert(snapshot.period_count == 0U);
  assert(snapshot.last_edge_age_cycles == 15U);
  assert((snapshot.flags & APP_ENCODER_PERIOD_FLAG_HAS_PERIOD) == 0U);

  assert(AppEncoderPeriodAccumulator_OnEdge(&accumulator, 30U, -1));
  assert(AppEncoderPeriodAccumulator_Snapshot(&accumulator, 31U, &snapshot));
  assert(snapshot.period_count == 0U);
  assert(snapshot.direction == -1);
  assert((snapshot.flags & APP_ENCODER_PERIOD_FLAG_DIRECTION_RESET) != 0U);
  assert(AppEncoderPeriodAccumulator_OnEdge(&accumulator, 50U, -1));
  assert(!AppEncoderPeriodAccumulator_OnEdge(&accumulator, 50U, -1));
  assert(AppEncoderPeriodAccumulator_Snapshot(&accumulator, 55U, &snapshot));
  assert(snapshot.period_sum_cycles == 20U);
  assert(snapshot.period_count == 1U);
  assert((snapshot.flags & APP_ENCODER_PERIOD_FLAG_ZERO_PERIOD) != 0U);

  assert(AppEncoderPeriodAccumulator_GetStats(&accumulator, &stats));
  assert(stats.invalid_direction_count == 1U);
  assert(stats.zero_period_count == 1U);
  assert(stats.direction_reset_count == 1U);
  assert(stats.aggregate_drop_count == 0U);
}

static void TestAggregateOverflowIsRejected(void)
{
  AppEncoderPeriodAccumulator accumulator;
  AppEncoderPeriodSnapshot snapshot;
  AppEncoderPeriodStats stats;

  AppEncoderPeriodAccumulator_Init(&accumulator);
  assert(AppEncoderPeriodAccumulator_OnEdge(&accumulator, 100U, 1));
  accumulator.period_sum_cycles = UINT32_MAX - 5U;
  accumulator.period_count = 1U;
  assert(!AppEncoderPeriodAccumulator_OnEdge(&accumulator, 110U, 1));
  assert(AppEncoderPeriodAccumulator_Snapshot(&accumulator, 111U, &snapshot));
  assert(snapshot.period_sum_cycles == 0U);
  assert(snapshot.period_count == 0U);
  assert((snapshot.flags & APP_ENCODER_PERIOD_FLAG_AGGREGATE_DROPPED) != 0U);
  assert(AppEncoderPeriodAccumulator_GetStats(&accumulator, &stats));
  assert(stats.aggregate_drop_count == 1U);
}

int main(void)
{
  TestWrapSnapshotAndDirectionReset();
  TestAggregateOverflowIsRejected();
  return 0;
}
