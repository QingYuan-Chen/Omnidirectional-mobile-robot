#include "app_encoder_accumulator.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>

static void FillRaw(uint16_t raw[BSP_MOTOR_COUNT], uint16_t value)
{
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    raw[i] = value;
  }
}

static void TestInitializationAndWrap(void)
{
  AppEncoderAccumulator state;
  uint16_t raw[BSP_MOTOR_COUNT];
  int16_t delta[BSP_MOTOR_COUNT];
  int64_t total[BSP_MOTOR_COUNT];

  FillRaw(raw, UINT16_MAX);
  assert(!AppEncoderAccumulator_Init(NULL, raw));
  assert(!AppEncoderAccumulator_Init(&state, NULL));
  assert(AppEncoderAccumulator_Init(&state, raw));

  FillRaw(raw, 0U);
  assert(AppEncoderAccumulator_Update(&state, raw, delta, total));
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    assert(delta[i] == 1);
    assert(total[i] == 1);
  }

  FillRaw(raw, UINT16_MAX);
  assert(AppEncoderAccumulator_Update(&state, raw, delta, total));
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    assert(delta[i] == -1);
    assert(total[i] == 0);
  }
}

static void TestDirectionAndSaturation(void)
{
  AppEncoderAccumulator state;
  uint16_t raw[BSP_MOTOR_COUNT] = {0U, 100U, 200U, 300U};
  int16_t delta[BSP_MOTOR_COUNT];
  int64_t total[BSP_MOTOR_COUNT];
  assert(AppEncoderAccumulator_Init(&state, raw));

  const uint16_t next[BSP_MOTOR_COUNT] = {10U, 90U, 32768U, 299U};
  assert(AppEncoderAccumulator_Update(&state, next, delta, total));
  assert(delta[BSP_MOTOR_MA] == 10);
  assert(delta[BSP_MOTOR_MB] == -10);
  assert(delta[BSP_MOTOR_MC] == 32568);
  assert(delta[BSP_MOTOR_MD] == -1);

  state.total_count[BSP_MOTOR_MA] = INT64_MAX;
  state.total_count[BSP_MOTOR_MB] = INT64_MIN;
  const uint16_t saturated_next[BSP_MOTOR_COUNT] = {11U, 89U, 32768U, 299U};
  assert(AppEncoderAccumulator_Update(&state, saturated_next, delta, total));
  assert(total[BSP_MOTOR_MA] == INT64_MAX);
  assert(total[BSP_MOTOR_MB] == INT64_MIN);
}

int main(void)
{
  TestInitializationAndWrap();
  TestDirectionAndSaturation();
  return 0;
}
