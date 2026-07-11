#include "bsp_encoder.h"

#include "tim.h"

static TIM_HandleTypeDef *const encoder_timers[BSP_MOTOR_COUNT] = {
  [BSP_MOTOR_MA] = &htim2,
  [BSP_MOTOR_MB] = &htim3,
  [BSP_MOTOR_MC] = &htim5,
  [BSP_MOTOR_MD] = &htim4,
};

static uint16_t encoder_last_count[BSP_MOTOR_COUNT];

static bool BspEncoder_IsValid(BspMotorId motor)
{
  return (uint32_t)motor < (uint32_t)BSP_MOTOR_COUNT;
}

BspStatus BspEncoder_Init(void)
{
  for (uint32_t i = 0; i < BSP_MOTOR_COUNT; ++i) {
    if (HAL_TIM_Encoder_Start(encoder_timers[i], TIM_CHANNEL_ALL) != HAL_OK) {
      return BSP_ERROR;
    }
  }

  BspEncoder_ResetAll();
  return BSP_OK;
}

uint16_t BspEncoder_ReadRaw(BspMotorId motor)
{
  if (!BspEncoder_IsValid(motor)) {
    return 0U;
  }

  return (uint16_t)__HAL_TIM_GET_COUNTER(encoder_timers[motor]);
}

int16_t BspEncoder_ReadDelta(BspMotorId motor)
{
  if (!BspEncoder_IsValid(motor)) {
    return 0;
  }

  const uint16_t current = BspEncoder_ReadRaw(motor);
  const int16_t delta = (int16_t)(current - encoder_last_count[motor]);
  encoder_last_count[motor] = current;

  return delta;
}

void BspEncoder_Reset(BspMotorId motor)
{
  if (!BspEncoder_IsValid(motor)) {
    return;
  }

  __HAL_TIM_SET_COUNTER(encoder_timers[motor], 0U);
  encoder_last_count[motor] = 0U;
}

void BspEncoder_ResetAll(void)
{
  for (uint32_t i = 0; i < BSP_MOTOR_COUNT; ++i) {
    BspEncoder_Reset((BspMotorId)i);
  }
}
