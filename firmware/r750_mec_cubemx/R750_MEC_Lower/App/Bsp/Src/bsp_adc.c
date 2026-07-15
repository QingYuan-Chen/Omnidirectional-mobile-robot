#include "bsp_adc.h"

#include "adc.h"
#include "robot_config.h"

/*
 * 电池 ADC 使用“单次启动→有限时轮询→取值→停止”的完整事务。
 * 当前只由通信任务按遥测周期调用，因此不增加互斥锁；若未来安全任务也需要电压，应该
 * 共享一次采样快照，而不是并发操作 hadc2。ADC 参数和通道选择仍由 CubeMX 生成代码负责。
 */

BspStatus BspAdc_Init(void)
{
  /* 保留统一生命周期入口，便于未来加入校准或配置一致性检查。 */
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

  /*
   * 轮询最多占用 1 ms，防止通信任务因 ADC 硬件异常无限阻塞。轮询失败后无论原因都调用
   * Stop 清理外设状态；HAL_TIMEOUT 单独映射，其他 HAL 错误统一为 BSP_ERROR。
   */
  const HAL_StatusTypeDef poll_status = HAL_ADC_PollForConversion(
    &hadc2, ROBOT_CONFIG_ADC_POLL_TIMEOUT_MS);
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

  /*
   * 换算式为 raw/4095×Vref×分压比，乘法在除法前完成以保留整数精度。当前参数最大中间值
   * 小于 uint32_t 上限；输出 uint16_t 足以覆盖 3S 电池电压。最终比例仍需万用表标定。
   */
  const uint32_t scaled_mv =
    ((uint32_t)raw * ROBOT_CONFIG_ADC_VREF_MV * ROBOT_CONFIG_BATTERY_DIVIDER_NUM) /
    (ROBOT_CONFIG_ADC_MAX_RAW * ROBOT_CONFIG_BATTERY_DIVIDER_DEN);
  *millivolts = (uint16_t)scaled_mv;

  return BSP_OK;
}
