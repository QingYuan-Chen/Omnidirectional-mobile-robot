#ifndef APP_ENCODER_ACCUMULATOR_H
#define APP_ENCODER_ACCUMULATOR_H

#include "bsp_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t previous_raw[BSP_MOTOR_COUNT];
  int64_t total_count[BSP_MOTOR_COUNT];
} AppEncoderAccumulator;

bool AppEncoderAccumulator_Init(AppEncoderAccumulator *state, const uint16_t raw_count[BSP_MOTOR_COUNT]);
bool AppEncoderAccumulator_Update(
  AppEncoderAccumulator *state,
  const uint16_t raw_count[BSP_MOTOR_COUNT],
  int16_t delta_count[BSP_MOTOR_COUNT],
  int64_t total_count[BSP_MOTOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
