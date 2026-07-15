#ifndef BSP_H
#define BSP_H

#include "bsp_types.h"

/*
 * 板级支持包统一启动入口。
 *
 * 模块把分散的蜂鸣器、编码器、ADC、IMU、UART 和电机驱动初始化串成单一、可诊断的
 * 安全启动流程。调用任何普通初始化前先执行寄存器级电机紧急空转；随后每进入一个阶段
 * 都先记录阶段号。任一组件失败都会再次紧急空转并立即返回，不继续初始化后续执行器。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 最近一次进入的初始化阶段。
 * 失败时保持在失败组件对应阶段；全部成功后为 READY；NONE 表示尚未开始或刚重置状态。
 */
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

/*
 * 按固定顺序初始化全部板级组件。
 * 成功返回 BSP_OK 并把阶段置为 READY；失败返回具体组件状态，并保证电机 PWM 基础设施
 * 已关闭。该函数设计为启动阶段调用，不支持运行中重复初始化。
 */
BspStatus Bsp_Init(void);
/* 返回最近一次进入或完成的初始化阶段，用于启动故障定位。 */
BspInitStage Bsp_GetInitStage(void);
/* 返回最近一次组件初始化状态；READY 时应为 BSP_OK。 */
BspStatus Bsp_GetInitStatus(void);

#ifdef __cplusplus
}
#endif

#endif
