#ifndef BSP_H
#define BSP_H

#include "bsp_types.h"

/* BSP 总入口按安全顺序初始化外设，并保留失败阶段供启动诊断。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 最近一次正在执行或已经完成的初始化阶段。 */
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

/* 初始化前先紧急空转全部电机，任一阶段失败立即保持安全输出。 */
BspStatus Bsp_Init(void);
/* 返回最近一次初始化阶段。 */
BspInitStage Bsp_GetInitStage(void);
/* 返回最近一次初始化状态。 */
BspStatus Bsp_GetInitStatus(void);

#ifdef __cplusplus
}
#endif

#endif
