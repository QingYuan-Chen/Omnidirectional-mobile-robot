#include "bsp_motor.h"

#include "robot_config.h"
#include "tim.h"

/*
 * 电机驱动把四个逻辑电机映射到三组定时器的八个 PWM 通道。
 * 每个 H 桥输入由独立 PWM 控制，驱动采用“一侧保持高、另一侧从高向低拉开差值”的
 * 双输入方式形成有效电压。普通制动/空转只改比较值，PWM 基础设施继续运行；紧急空转
 * 直接接管 GPIO 和定时器寄存器并关闭输出，作为初始化失败、急停和任务崩溃的最终路径。
 */

typedef struct {
  /* forward/reverse 只表达当前接线下的软件正负方向，不代表车辆坐标系已完成板上确认。 */
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
  /* 指定初始化器把每个逻辑轮与实际定时器/通道显式绑定，顺序必须与原理图和接线一致。 */
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
  /* PWM 计数范围为 0..ARR，因此持续高电平比较值使用 ARR+1。 */
  return __HAL_TIM_GET_AUTORELOAD(pwm->timer) + 1U;
}

static uint32_t BspMotor_ClampMagnitude(const BspMotorPwm *pwm, int16_t command)
{
  /* 同时受软件统一上限和具体定时器满量程限制，避免未来定时器 ARR 不一致导致越界。 */
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
  /*
   * 先把引脚作为低电平 GPIO 并关闭所有 PWM，再由 HAL_MspPostInit 恢复复用功能。每个通道
   * 启动前比较值均为零；任一路启动失败都会重新走寄存器级紧急空转，避免部分电机已启用。
   */
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
    /*
     * 正向：forward 恒高，reverse 的低电平比例等于 magnitude/full_scale；反向对调两侧。
     * 这种编码与驱动板输入真值表绑定，若更换驱动器必须重新核对 Brake/Coast 语义。
     */
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
  /* 两侧同时保持高电平，按当前驱动芯片真值表短接电机端形成主动制动。 */
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
  /* 两侧同时保持低电平，H 桥进入高阻空转；定时器与通道仍运行，下一次 SetPwm 可恢复。 */
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
  /*
   * 先开 GPIOB/E 时钟并读回加 DSB，保证后续寄存器写已到达外设。BSRR 原子写低八个输入，
   * 再清输出类型、上下拉和速度并强制普通输出模式，切断定时器复用对引脚的控制。
   */
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

  /*
   * 第二层再清全部 CCR、CCER 和高级定时器主输出 MOE，最后停 TIM1/TIM9/TIM12 计数器。
   * GPIO 低电平与 PWM 关闭形成双重保护。该操作破坏普通 PWM 运行上下文，不能用 SetPwm
   * 直接恢复；BSP 技术上可重新执行完整 Init，但应用安全策略只允许系统复位恢复。
   */
  TIM1->CCER = 0U;
  TIM9->CCER = 0U;
  TIM12->CCER = 0U;
  TIM1->BDTR &= ~TIM_BDTR_MOE;

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM9->CR1 &= ~TIM_CR1_CEN;
  TIM12->CR1 &= ~TIM_CR1_CEN;
}
