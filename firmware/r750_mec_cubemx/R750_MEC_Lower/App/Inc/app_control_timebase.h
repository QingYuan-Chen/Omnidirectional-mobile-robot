#ifndef APP_CONTROL_TIMEBASE_H
#define APP_CONTROL_TIMEBASE_H

#include "bsp_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t irq_period_cycles;
  uint32_t timer_irq_missed_period_count;
  uint32_t task_wake_cycles;
  uint32_t notification_count;
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
} AppControlTick;

BspStatus AppControlTimebase_Start(void);
BspStatus AppControlTimebase_Stop(void);
BspStatus AppControlTimebase_Wait(AppControlTick *tick);
uint32_t AppControlTimebase_GetCycleCount(void);
void AppControlTimebase_OnTimerIrqEntryFromIsr(void);
void AppControlTimebase_OnTimerElapsedFromIsr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_TIMEBASE_H */
