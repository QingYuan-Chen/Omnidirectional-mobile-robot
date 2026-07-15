#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_types.h"

/* 电池 ADC 接口集中管理单次采样和分压换算，调用者不得直接操作 hadc2。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 当前 ADC 已由 CubeMX 初始化，此函数保留统一 BSP 生命周期接口。 */
BspStatus BspAdc_Init(void);
/* 启动一次转换并返回 12 位原始值。 */
BspStatus BspAdc_ReadBatteryRaw(uint16_t *raw);
/* 按编译期分压参数把原始值换算为毫伏。 */
BspStatus BspAdc_ReadBatteryMillivolts(uint16_t *millivolts);

#ifdef __cplusplus
}
#endif

#endif
