#include "bsp_beep.h"

#include "main.h"

/*
 * 蜂鸣器是高电平有效的简单 GPIO 执行器，不保存软件镜像状态。
 * 初始化第一动作即写低，避免 CubeMX GPIO 初值或复位过渡造成持续误响；Set/Toggle 都是
 * 非阻塞寄存器操作，提示时长由调用任务控制。
 */

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
