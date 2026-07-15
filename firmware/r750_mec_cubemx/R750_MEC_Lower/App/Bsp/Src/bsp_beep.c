#include "bsp_beep.h"

#include "main.h"

/* 蜂鸣器为简单 GPIO 执行器，初始化默认关闭，避免上电误响。 */

BspStatus BspBeep_Init(void)
{
  BspBeep_Set(false);
  return BSP_OK;
}

void BspBeep_Set(bool enabled)
{
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BspBeep_Toggle(void)
{
  HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
}
