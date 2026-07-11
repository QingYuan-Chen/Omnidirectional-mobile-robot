#ifndef BSP_H
#define BSP_H

#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BSP_INIT_STAGE_NONE = 0,
  BSP_INIT_STAGE_BEEP,
  BSP_INIT_STAGE_ENCODER,
  BSP_INIT_STAGE_ADC,
  BSP_INIT_STAGE_IMU,
  BSP_INIT_STAGE_UART,
  BSP_INIT_STAGE_MOTOR,
  BSP_INIT_STAGE_READY
} BspInitStage;

BspStatus Bsp_Init(void);
BspInitStage Bsp_GetInitStage(void);
BspStatus Bsp_GetInitStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_H */
