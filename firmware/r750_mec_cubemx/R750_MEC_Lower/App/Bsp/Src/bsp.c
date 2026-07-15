#include "bsp.h"

#include "bsp_adc.h"
#include "bsp_beep.h"
#include "bsp_encoder.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

/*
 * 板级初始化总控把各外设的独立初始化组合成可诊断、失败封闭的启动事务。
 * 阶段与状态在调用组件前后持续更新，启动故障循环可读取它们定位失败模块。电机紧急
 * 空转既在流程入口执行，也在每个组件失败出口执行，确保任何部分初始化都不会留下输出。
 */

static BspInitStage bsp_init_stage = BSP_INIT_STAGE_NONE;
static BspStatus bsp_init_status = BSP_ERROR;

static BspStatus Bsp_InitComponent(BspInitStage stage, BspStatus (*init_function)(void))
{
  /*
   * 先写 stage，再调用函数，函数内部即使阻塞或失败也能看到准确阶段；状态直接保存组件
   * 原始 BSP 返回值，避免统一压成 BSP_ERROR 丢失 TIMEOUT/BUSY 等诊断信息。
   */
  bsp_init_stage = stage;
  bsp_init_status = init_function();
  if (bsp_init_status != BSP_OK) {
    BspMotor_EmergencyCoastAll();
  }
  return bsp_init_status;
}

BspStatus Bsp_Init(void)
{
  /*
   * 紧急空转使用寄存器级实现，可在 CubeMX 的 PWM GPIO 复用尚未建立时调用。普通外设
   * 顺序先传感/通信、后电机：只有全部上游依赖成功后，最后一步才重新启动 PWM 基础设施。
   */
  BspMotor_EmergencyCoastAll();
  bsp_init_stage = BSP_INIT_STAGE_NONE;
  bsp_init_status = BSP_ERROR;

  if (Bsp_InitComponent(BSP_INIT_STAGE_BEEP, BspBeep_Init) != BSP_OK) {
    return bsp_init_status;
  }

  if (Bsp_InitComponent(BSP_INIT_STAGE_ENCODER, BspEncoder_Init) != BSP_OK) {
    return bsp_init_status;
  }

  if (Bsp_InitComponent(BSP_INIT_STAGE_ADC, BspAdc_Init) != BSP_OK) {
    return bsp_init_status;
  }

  if (Bsp_InitComponent(BSP_INIT_STAGE_IMU, BspImu_Init) != BSP_OK) {
    return bsp_init_status;
  }

  if (Bsp_InitComponent(BSP_INIT_STAGE_UART, BspUart_Init) != BSP_OK) {
    return bsp_init_status;
  }

  if (Bsp_InitComponent(BSP_INIT_STAGE_MOTOR, BspMotor_Init) != BSP_OK) {
    return bsp_init_status;
  }

  bsp_init_stage = BSP_INIT_STAGE_READY;
  bsp_init_status = BSP_OK;
  return BSP_OK;
}

BspInitStage Bsp_GetInitStage(void)
{
  return bsp_init_stage;
}

BspStatus Bsp_GetInitStatus(void)
{
  return bsp_init_status;
}
