#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "bsp_types.h"

/* 四路编码器接口封装定时器映射；确定性控制链优先使用同步原始快照。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 启动全部编码器定时器并清零计数。 */
BspStatus BspEncoder_Init(void);
/* 读取指定电机的 16 位硬件计数。 */
uint16_t BspEncoder_ReadRaw(BspMotorId motor);
/* 兼容性增量接口，自带独立历史状态，不得与控制快照链混用。 */
int16_t BspEncoder_ReadDelta(BspMotorId motor);
/* 清零一路硬件计数及兼容性增量基线。 */
void BspEncoder_Reset(BspMotorId motor);
/* 清零全部编码器。 */
void BspEncoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif
