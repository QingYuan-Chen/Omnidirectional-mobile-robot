#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_types.h"

/*
 * 电池电压 ADC 板级接口。
 *
 * ADC2 的启动、有限时轮询、取值和停止由本模块完整持有，调用者不得绕过接口直接操作
 * hadc2。当前通信任务是唯一周期采样者，因此接口不提供并发仲裁；未来增加欠压保护任务
 * 时必须先建立统一采样所有权，不能让两个任务交叉启动同一 ADC。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 接入统一 BSP 生命周期。ADC 外设参数由 CubeMX 在调用本函数前完成，当前无需额外配置，
 * 因而直接返回 BSP_OK。
 */
BspStatus BspAdc_Init(void);
/*
 * 完成一次 ADC2 单次转换并返回 12 位原始计数。
 * raw 为空返回 BSP_INVALID_ARG；启动或停止失败返回 BSP_ERROR；轮询超时返回 BSP_TIMEOUT。
 * 任一轮询失败路径都会尝试停止 ADC，避免外设残留在活动状态。
 */
BspStatus BspAdc_ReadBatteryRaw(uint16_t *raw);
/*
 * 读取原始值并按编译期参考电压及分压比换算为电池端毫伏。
 * 计算使用 32 位整数，中间量按当前参数不会溢出；结果尚未经过板上万用表标定，当前仅
 * 供遥测观察。读取失败时原样返回底层状态。
 */
BspStatus BspAdc_ReadBatteryMillivolts(uint16_t *millivolts);

#ifdef __cplusplus
}
#endif

#endif
