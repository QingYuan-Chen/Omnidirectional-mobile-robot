#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

BspStatus BspAdc_Init(void);
BspStatus BspAdc_ReadBatteryRaw(uint16_t *raw);
BspStatus BspAdc_ReadBatteryMillivolts(uint16_t *millivolts);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
