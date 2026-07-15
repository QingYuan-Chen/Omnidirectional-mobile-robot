#ifndef APP_CONTROL_TICK_BUFFER_H
#define APP_CONTROL_TICK_BUFFER_H

#include "bsp_types.h"
#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t irq_period_cycles;
  uint32_t timer_irq_missed_period_count;
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
} AppControlTickSample;

typedef struct {
  volatile AppControlTickSample slots[ROBOT_CONFIG_CONTROL_TICK_RING_SIZE];
  volatile uint32_t produced_sequence;
  uint32_t consumed_sequence;
} AppControlTickBuffer;

void AppControlTickBuffer_Init(AppControlTickBuffer *buffer);
void AppControlTickBuffer_PublishFromIsr(
  AppControlTickBuffer *buffer,
  uint32_t irq_timestamp_cycles,
  uint32_t irq_period_cycles,
  uint32_t timer_irq_missed_period_count,
  const uint16_t encoder_raw[BSP_MOTOR_COUNT]);
bool AppControlTickBuffer_Consume(
  AppControlTickBuffer *buffer,
  uint32_t notification_count,
  AppControlTickSample *sample);

#ifdef __cplusplus
}
#endif

#endif
