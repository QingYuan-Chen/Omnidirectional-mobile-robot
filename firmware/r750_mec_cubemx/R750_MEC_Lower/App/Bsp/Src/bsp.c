#include "bsp.h"

#include "bsp_adc.h"
#include "bsp_beep.h"
#include "bsp_encoder.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

static BspInitStage bsp_init_stage = BSP_INIT_STAGE_NONE;
static BspStatus bsp_init_status = BSP_ERROR;

static BspStatus Bsp_InitComponent(BspInitStage stage, BspStatus (*init_function)(void))
{
  bsp_init_stage = stage;
  bsp_init_status = init_function();
  if (bsp_init_status != BSP_OK) {
    BspMotor_EmergencyCoastAll();
  }
  return bsp_init_status;
}

BspStatus Bsp_Init(void)
{
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
