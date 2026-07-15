#ifndef APP_CONTROL_TIMEBASE_H
#define APP_CONTROL_TIMEBASE_H

#include "bsp_types.h"

#include <stdint.h>

/* TIM7 中断只采集时间戳和编码器快照，控制计算始终留在任务上下文。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 控制任务一次唤醒所需的时基和四路编码器快照。 */
typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t irq_period_cycles;
  uint32_t timer_irq_missed_period_count;
  uint32_t task_wake_cycles;
  uint32_t notification_count;
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
} AppControlTick;

/* 校验时钟配置、登记控制任务并启动 TIM7 中断。 */
BspStatus AppControlTimebase_Start(void);
/* 停止 TIM7 并撤销控制任务所有权。 */
BspStatus AppControlTimebase_Stop(void);
/* 仅允许已登记控制任务阻塞等待下一个或合并后的节拍。 */
BspStatus AppControlTimebase_Wait(AppControlTick *tick);
/* 返回 DWT 周期计数，用于控制任务执行时间测量。 */
uint32_t AppControlTimebase_GetCycleCount(void);
/* 在 TIM7 向量入口尽早记录时间戳，减少 HAL 分派开销对测量的污染。 */
void AppControlTimebase_OnTimerIrqEntryFromIsr(void);
/* 在 HAL 更新回调中采集快照并通知控制任务。 */
void AppControlTimebase_OnTimerElapsedFromIsr(void);

#ifdef __cplusplus
}
#endif

#endif
