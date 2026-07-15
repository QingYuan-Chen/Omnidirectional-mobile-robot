#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "bsp_types.h"

/*
 * 四路正交编码器定时器封装。
 *
 * 本模块固定维护 BspMotorId 到 TIM2/TIM3/TIM5/TIM4 的映射，并只暴露 16 位硬件计数。
 * 确定性控制链应在同一次 TIM7 中断中读取四路 raw 快照，再交给
 * AppEncoderAccumulator 统一差分；ReadDelta 只为简单板测兼容，拥有另一套历史基线，
 * 不得与控制链交叉调用。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 启动四个编码器定时器的全部输入通道，随后清零硬件计数和兼容差分基线。
 * 任一路 HAL 启动失败立即返回 BSP_ERROR；该接口不负责回滚此前已启动的编码器。
 */
BspStatus BspEncoder_Init(void);
/*
 * 读取指定逻辑电机的 16 位计数器快照。
 * motor 无效时返回 0；由于 0 也是合法计数，上层不应把该返回值用作错误判据。
 */
uint16_t BspEncoder_ReadRaw(BspMotorId motor);
/*
 * 基于模块内部历史值返回一次有符号增量。
 * 只适用于相邻读取真实变化不超过半量程的低速板测；调用会推进该路内部基线。控制任务
 * 已使用独立累计器，因此不得同时调用本接口。motor 无效时返回 0。
 */
int16_t BspEncoder_ReadDelta(BspMotorId motor);
/* 清零一路硬件计数器和 ReadDelta 基线；motor 无效时不执行操作。 */
void BspEncoder_Reset(BspMotorId motor);
/* 依次清零四路编码器；运行中调用会破坏里程连续性，只允许初始化或明确板测使用。 */
void BspEncoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif
