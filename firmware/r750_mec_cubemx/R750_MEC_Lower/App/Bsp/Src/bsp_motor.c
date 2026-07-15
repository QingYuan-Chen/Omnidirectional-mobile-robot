#include "bsp_motor.h"

#include "robot_config.h"
#include "tim.h"

/* 电机驱动使用双输入互补占空比，普通制动、普通空转和紧急空转语义严格区分。 */

typedef struct {
  TIM_HandleTypeDef *timer;
  uint32_t forward_channel;
  uint32_t reverse_channel;
} BspMotorPwm;

#define BSP_MOTOR_GPIOE_PINS \
  ((uint32_t)(PWM_MA_IN1_Pin | PWM_MA_IN2_Pin | PWM_MB_IN1_Pin | PWM_MB_IN2_Pin | PWM_MC_IN1_Pin | PWM_MC_IN2_Pin))
#define BSP_MOTOR_GPIOB_PINS ((uint32_t)(PWM_MD_IN1_Pin | PWM_MD_IN2_Pin))
#define BSP_MOTOR_GPIOE_MODE_MASK                                                                                  \
  (GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE11_Msk |                   \
   GPIO_MODER_MODE13_Msk | GPIO_MODER_MODE14_Msk)
#define BSP_MOTOR_GPIOE_OUTPUT_MODE                                                                                \
  (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE9_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE13_0 |     \
   GPIO_MODER_MODE14_0)
#define BSP_MOTOR_GPIOB_MODE_MASK   (GPIO_MODER_MODE14_Msk | GPIO_MODER_MODE15_Msk)
#define BSP_MOTOR_GPIOB_OUTPUT_MODE (GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0)

static const BspMotorPwm motor_pwm[BSP_MOTOR_COUNT] = {
  [BSP_MOTOR_MA] = {&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2},
  [BSP_MOTOR_MB] = {&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4},
  [BSP_MOTOR_MC] = {&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2},
  [BSP_MOTOR_MD] = {&htim12, TIM_CHANNEL_1, TIM_CHANNEL_2},
};

static bool BspMotor_IsValid(BspMotorId motor)
{
  return (uint32_t)motor < (uint32_t)BSP_MOTOR_COUNT;
}

static uint32_t BspMotor_GetFullScale(const BspMotorPwm *pwm)
{
  return __HAL_TIM_GET_AUTORELOAD(pwm->timer) + 1U;
}

static uint32_t BspMotor_ClampMagnitude(const BspMotorPwm *pwm, int16_t command)
{
  int32_t magnitude = command;

  if (magnitude < 0) {
    magnitude = -magnitude;
  }

  if (magnitude > ROBOT_CONFIG_PWM_LIMIT) {
    magnitude = ROBOT_CONFIG_PWM_LIMIT;
  }

  const uint32_t full_scale = BspMotor_GetFullScale(pwm);
  if ((uint32_t)magnitude > full_scale) {
    magnitude = (int32_t)full_scale;
  }

  return (uint32_t)magnitude;
}

BspStatus BspMotor_Init(void)
{
  /* 在复用功能和 PWM 通道启用前后都保持零输出，缩短潜在毛刺窗口。 */
  BspMotor_EmergencyCoastAll();

  HAL_TIM_MspPostInit(&htim1);
  HAL_TIM_MspPostInit(&htim9);
  HAL_TIM_MspPostInit(&htim12);

  for (uint32_t i = 0; i < BSP_MOTOR_COUNT; ++i) {
    const BspMotorPwm *pwm = &motor_pwm[i];

    __HAL_TIM_SET_COMPARE(pwm->timer, pwm->forward_channel, 0U);
    __HAL_TIM_SET_COMPARE(pwm->timer, pwm->reverse_channel, 0U);

    if (HAL_TIM_PWM_Start(pwm->timer, pwm->forward_channel) != HAL_OK) {
      BspMotor_EmergencyCoastAll();
      return BSP_ERROR;
    }
    if (HAL_TIM_PWM_Start(pwm->timer, pwm->reverse_channel) != HAL_OK) {
      BspMotor_EmergencyCoastAll();
      return BSP_ERROR;
    }
  }

  BspMotor_CoastAll();
  return BSP_OK;
}

BspStatus BspMotor_SetPwm(BspMotorId motor, int16_t command)
{
  if (!BspMotor_IsValid(motor)) {
    return BSP_INVALID_ARG;
  }

  const BspMotorPwm *pwm = &motor_pwm[motor];
  const uint32_t full_scale = BspMotor_GetFullScale(pwm);
  const uint32_t magnitude = BspMotor_ClampMagnitude(pwm, command);

  if (command > 0) {
    /* 一侧保持满量程，另一侧用差值形成有效占空比，符号决定方向。 */
    __HAL_TIM_SET_COMPARE(pwm->timer, pwm->forward_channel, full_scale);
    __HAL_TIM_SET_COMPARE(pwm->timer, pwm->reverse_channel, full_scale - magnitude);
  } else if (command < 0) {
    __HAL_TIM_SET_COMPARE(pwm->timer, pwm->forward_channel, full_scale - magnitude);
    __HAL_TIM_SET_COMPARE(pwm->timer, pwm->reverse_channel, full_scale);
  } else {
    BspMotor_Brake(motor);
  }

  return BSP_OK;
}

void BspMotor_Brake(BspMotorId motor)
{
  if (!BspMotor_IsValid(motor)) {
    return;
  }

  const BspMotorPwm *pwm = &motor_pwm[motor];
  const uint32_t full_scale = BspMotor_GetFullScale(pwm);
  /* 两侧同时为高电平，对应驱动芯片的主动制动状态。 */
  __HAL_TIM_SET_COMPARE(pwm->timer, pwm->forward_channel, full_scale);
  __HAL_TIM_SET_COMPARE(pwm->timer, pwm->reverse_channel, full_scale);
}

void BspMotor_BrakeAll(void)
{
  for (uint32_t i = 0; i < BSP_MOTOR_COUNT; ++i) {
    BspMotor_Brake((BspMotorId)i);
  }
}

void BspMotor_Coast(BspMotorId motor)
{
  if (!BspMotor_IsValid(motor)) {
    return;
  }

  const BspMotorPwm *pwm = &motor_pwm[motor];
  /* 两侧同时为低电平，仅撤销当前驱动，不停止 PWM 定时器。 */
  __HAL_TIM_SET_COMPARE(pwm->timer, pwm->forward_channel, 0U);
  __HAL_TIM_SET_COMPARE(pwm->timer, pwm->reverse_channel, 0U);
}

void BspMotor_CoastAll(void)
{
  for (uint32_t i = 0; i < BSP_MOTOR_COUNT; ++i) {
    BspMotor_Coast((BspMotorId)i);
  }
}

void BspMotor_EmergencyCoastAll(void)
{
  /* 直接操作 GPIO 和定时器寄存器，使该路径在常规 HAL 初始化前后都可用。 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
  (void)RCC->AHB1ENR;
  __DSB();

  GPIOE->BSRR = BSP_MOTOR_GPIOE_PINS << 16U;
  GPIOB->BSRR = BSP_MOTOR_GPIOB_PINS << 16U;
  GPIOE->OTYPER &= ~BSP_MOTOR_GPIOE_PINS;
  GPIOB->OTYPER &= ~BSP_MOTOR_GPIOB_PINS;
  GPIOE->PUPDR &= ~BSP_MOTOR_GPIOE_MODE_MASK;
  GPIOB->PUPDR &= ~BSP_MOTOR_GPIOB_MODE_MASK;
  GPIOE->OSPEEDR &= ~BSP_MOTOR_GPIOE_MODE_MASK;
  GPIOB->OSPEEDR &= ~BSP_MOTOR_GPIOB_MODE_MASK;
  GPIOE->MODER = (GPIOE->MODER & ~BSP_MOTOR_GPIOE_MODE_MASK) | BSP_MOTOR_GPIOE_OUTPUT_MODE;
  GPIOB->MODER = (GPIOB->MODER & ~BSP_MOTOR_GPIOB_MODE_MASK) | BSP_MOTOR_GPIOB_OUTPUT_MODE;

  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  TIM9->CCR1 = 0U;
  TIM9->CCR2 = 0U;
  TIM12->CCR1 = 0U;
  TIM12->CCR2 = 0U;

  /* 清比较值、关闭通道和主输出，最后停止所有电机 PWM 定时器。 */
  TIM1->CCER = 0U;
  TIM9->CCER = 0U;
  TIM12->CCER = 0U;
  TIM1->BDTR &= ~TIM_BDTR_MOE;

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM9->CR1 &= ~TIM_CR1_CEN;
  TIM12->CR1 &= ~TIM_CR1_CEN;
}
