#ifndef BSP_BEEP_H
#define BSP_BEEP_H

#include "bsp_types.h"

/*
 * 有源蜂鸣器 GPIO 封装。
 *
 * 应用层只使用逻辑开、关和翻转，不依赖端口、引脚或有效电平。当前硬件为高电平开启；
 * 初始化必须先关闭蜂鸣器，避免上电及故障启动过程中产生无意义持续鸣叫。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 把蜂鸣器置为关闭并返回 BSP_OK；GPIO 模式已由 CubeMX 配置。 */
BspStatus BspBeep_Init(void);
/* enabled 为 true 时输出有效电平，为 false 时关闭；函数不延时。 */
void BspBeep_Set(bool enabled);
/* 翻转当前 GPIO 输出，适合简单提示，不维护独立软件状态，也不承诺并发事务原子性。 */
void BspBeep_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif
