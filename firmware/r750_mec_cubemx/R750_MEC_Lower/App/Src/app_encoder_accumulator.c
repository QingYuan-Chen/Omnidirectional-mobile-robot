#include "app_encoder_accumulator.h"

#include <limits.h>
#include <string.h>

static int64_t AppEncoderAccumulator_AddSaturated(int64_t total, int16_t delta)
{
  const int64_t delta_wide = (int64_t)delta;
  if (delta_wide > 0 && total > (INT64_MAX - delta_wide)) {
    return INT64_MAX;
  }
  if (delta_wide < 0 && total < (INT64_MIN - delta_wide)) {
    return INT64_MIN;
  }
  return total + delta_wide;
}

bool AppEncoderAccumulator_Init(AppEncoderAccumulator *state, const uint16_t raw_count[BSP_MOTOR_COUNT])
{
  if (state == NULL || raw_count == NULL) {
    return false;
  }

  memcpy(state->previous_raw, raw_count, sizeof(state->previous_raw));
  memset(state->total_count, 0, sizeof(state->total_count));
  return true;
}

bool AppEncoderAccumulator_Update(
  AppEncoderAccumulator *state,
  const uint16_t raw_count[BSP_MOTOR_COUNT],
  int16_t delta_count[BSP_MOTOR_COUNT],
  int64_t total_count[BSP_MOTOR_COUNT])
{
  if (state == NULL || raw_count == NULL || delta_count == NULL || total_count == NULL) {
    return false;
  }

  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    /* 先在 16 位无符号域相减，再解释为有符号增量，可自然处理计数器回绕。 */
    const uint16_t difference = (uint16_t)(raw_count[i] - state->previous_raw[i]);
    const int32_t signed_difference = difference <= (uint16_t)INT16_MAX
                                        ? (int32_t)difference
                                        : (int32_t)difference - 65536;
    const int16_t delta = (int16_t)signed_difference;
    state->previous_raw[i] = raw_count[i];
    state->total_count[i] = AppEncoderAccumulator_AddSaturated(state->total_count[i], delta);
    delta_count[i] = delta;
    total_count[i] = state->total_count[i];
  }

  return true;
}
