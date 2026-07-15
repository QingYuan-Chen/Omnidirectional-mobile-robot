#ifndef APP_ENCODER_ACCUMULATOR_H
#define APP_ENCODER_ACCUMULATOR_H

#include "bsp_types.h"

#include <stdbool.h>
#include <stdint.h>

/* 将四路 16 位硬件计数器转换为单周期有符号增量和 64 位累计计数。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 保存上次原始计数和防溢出的长期累计值。 */
typedef struct {
  uint16_t previous_raw[BSP_MOTOR_COUNT];
  int64_t total_count[BSP_MOTOR_COUNT];
} AppEncoderAccumulator;

/* 以当前硬件计数为基线，初始化时不产生虚假增量。 */
bool AppEncoderAccumulator_Init(AppEncoderAccumulator *state, const uint16_t raw_count[BSP_MOTOR_COUNT]);
/* 更新四路增量和累计值，按模 2^16 处理计数器回绕。 */
bool AppEncoderAccumulator_Update(
  AppEncoderAccumulator *state,
  const uint16_t raw_count[BSP_MOTOR_COUNT],
  int16_t delta_count[BSP_MOTOR_COUNT],
  int64_t total_count[BSP_MOTOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
