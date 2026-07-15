#ifndef BSP_BEEP_H
#define BSP_BEEP_H

#include "bsp_types.h"

/* 蜂鸣器接口封装有效电平，应用层只表达开、关和翻转。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化后确保蜂鸣器关闭。 */
BspStatus BspBeep_Init(void);
/* 设置蜂鸣器逻辑状态。 */
void BspBeep_Set(bool enabled);
/* 翻转蜂鸣器当前输出。 */
void BspBeep_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif
