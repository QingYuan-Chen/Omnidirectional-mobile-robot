#include "app_encoder_accumulator.h"

#include <limits.h>
#include <string.h>

/*
 * 编码器累计模块在 16 位模空间计算相邻快照增量，并扩展为 64 位长期位置计数。
 * 模差分可同时处理向上和向下跨零回绕，但前提是单个控制间隔内的真实计数变化绝对值
 * 不超过 32767；漏周期过多时该前提可能失效，因此时序统计必须同步上报。
 */

static int64_t AppEncoderAccumulator_AddSaturated(int64_t total, int16_t delta)
{
  /* 极长期运行到达 int64_t 边界后保持饱和，避免位置突然翻转符号。 */
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
    /*
     * 先按模 2^16 求差，再把 0～65535 映射到最接近零的有符号代表值。这样例如
     * 65530→4 得到 +10，4→65530 得到 -10，而不需要在边界处写特殊分支。
     */
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
