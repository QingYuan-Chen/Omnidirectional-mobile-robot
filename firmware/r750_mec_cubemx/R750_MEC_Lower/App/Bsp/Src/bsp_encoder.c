#include "bsp_encoder.h"

#include "tim.h"

/*
 * 四路编码器逻辑映射固定为 MA→TIM2、MB→TIM3、MC→TIM5、MD→TIM4。
 * 数组使用指定初始化器绑定枚举值，避免依赖声明顺序猜测；若硬件接线或 CubeMX 定时器
 * 分配改变，必须同时复核此表、计数方向和上层麦克纳姆轮序号。
 */

static TIM_HandleTypeDef *const encoder_timers[BSP_MOTOR_COUNT] = {
  [BSP_MOTOR_MA] = &htim2,
  [BSP_MOTOR_MB] = &htim3,
  [BSP_MOTOR_MC] = &htim5,
  [BSP_MOTOR_MD] = &htim4,
};

static uint16_t encoder_last_count[BSP_MOTOR_COUNT];
/* encoder_last_count 只服务兼容 ReadDelta，不参与 TIM7 快照链的 AppEncoderAccumulator。 */

static bool BspEncoder_IsValid(BspMotorId motor)
{
  return (uint32_t)motor < (uint32_t)BSP_MOTOR_COUNT;
}

BspStatus BspEncoder_Init(void)
{
  /* 四个定时器均以编码器模式启动全部通道；全部启动后才统一清零基线。 */
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

  /*
   * C 的无符号模减法自然处理 0/65535 回绕，再转换为 int16_t 取得最短方向差。该接口没有
   * 时间戳且基线独立，仅适合低速人工板测；确定性控制必须使用同步 raw 快照链。
   */
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
