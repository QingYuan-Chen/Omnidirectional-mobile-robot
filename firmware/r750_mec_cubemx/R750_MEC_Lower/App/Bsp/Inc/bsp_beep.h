#ifndef BSP_BEEP_H
#define BSP_BEEP_H

#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

BspStatus BspBeep_Init(void);
void BspBeep_Set(bool enabled);
void BspBeep_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BEEP_H */
