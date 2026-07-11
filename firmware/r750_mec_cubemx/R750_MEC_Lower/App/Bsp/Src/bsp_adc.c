#include "bsp_adc.h"

#include "adc.h"
#include "robot_config.h"

BspStatus BspAdc_Init(void)
{
  return BSP_OK;
}

BspStatus BspAdc_ReadBatteryRaw(uint16_t *raw)
{
  if (raw == NULL) {
    return BSP_INVALID_ARG;
  }

  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    return BSP_ERROR;
  }

  const HAL_StatusTypeDef poll_status = HAL_ADC_PollForConversion(&hadc2, 10U);
  if (poll_status != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc2);
    return poll_status == HAL_TIMEOUT ? BSP_TIMEOUT : BSP_ERROR;
  }

  *raw = (uint16_t)HAL_ADC_GetValue(&hadc2);

  if (HAL_ADC_Stop(&hadc2) != HAL_OK) {
    return BSP_ERROR;
  }

  return BSP_OK;
}

BspStatus BspAdc_ReadBatteryMillivolts(uint16_t *millivolts)
{
  if (millivolts == NULL) {
    return BSP_INVALID_ARG;
  }

  uint16_t raw;
  const BspStatus status = BspAdc_ReadBatteryRaw(&raw);
  if (status != BSP_OK) {
    return status;
  }

  const uint32_t scaled_mv =
    ((uint32_t)raw * ROBOT_CONFIG_ADC_VREF_MV * ROBOT_CONFIG_BATTERY_DIVIDER_NUM) /
    (ROBOT_CONFIG_ADC_MAX_RAW * ROBOT_CONFIG_BATTERY_DIVIDER_DEN);
  *millivolts = (uint16_t)scaled_mv;

  return BSP_OK;
}
