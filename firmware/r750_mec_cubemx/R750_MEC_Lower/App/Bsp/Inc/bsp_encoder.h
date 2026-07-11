#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

BspStatus BspEncoder_Init(void);
uint16_t BspEncoder_ReadRaw(BspMotorId motor);
int16_t BspEncoder_ReadDelta(BspMotorId motor);
void BspEncoder_Reset(BspMotorId motor);
void BspEncoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
