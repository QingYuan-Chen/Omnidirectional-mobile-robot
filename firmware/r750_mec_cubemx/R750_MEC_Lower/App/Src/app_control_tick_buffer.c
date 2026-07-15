#include "app_control_tick_buffer.h"

#include <stddef.h>

/* 带序号环形缓冲区保证中断快照与计数型通知一一对应。 */

_Static_assert(ROBOT_CONFIG_CONTROL_TICK_RING_SIZE >= 2U,
               "control tick ring requires at least two slots");
_Static_assert((ROBOT_CONFIG_CONTROL_TICK_RING_SIZE &
                (ROBOT_CONFIG_CONTROL_TICK_RING_SIZE - 1U)) == 0U,
               "control tick ring size must be a power of two");

static uint32_t AppControlTickBuffer_Index(uint32_t sequence)
{
  return sequence & (ROBOT_CONFIG_CONTROL_TICK_RING_SIZE - 1U);
}

void AppControlTickBuffer_Init(AppControlTickBuffer *buffer)
{
  if (buffer == NULL) {
    return;
  }

  buffer->produced_sequence = 0U;
  buffer->consumed_sequence = 0U;
  for (uint32_t slot_index = 0U;
       slot_index < ROBOT_CONFIG_CONTROL_TICK_RING_SIZE;
       ++slot_index) {
    volatile AppControlTickSample *const slot = &buffer->slots[slot_index];
    slot->tick_sequence = 0U;
    slot->irq_timestamp_cycles = 0U;
    slot->irq_period_cycles = 0U;
    slot->timer_irq_missed_period_count = 0U;
    for (uint32_t motor_index = 0U; motor_index < (uint32_t)BSP_MOTOR_COUNT; ++motor_index) {
      slot->encoder_raw[motor_index] = 0U;
    }
  }
}

void AppControlTickBuffer_PublishFromIsr(
  AppControlTickBuffer *buffer,
  uint32_t irq_timestamp_cycles,
  uint32_t irq_period_cycles,
  uint32_t timer_irq_missed_period_count,
  const uint16_t encoder_raw[BSP_MOTOR_COUNT])
{
  if (buffer == NULL || encoder_raw == NULL) {
    return;
  }

  const uint32_t sequence = buffer->produced_sequence + 1U;
  volatile AppControlTickSample *const slot = &buffer->slots[AppControlTickBuffer_Index(sequence)];
  slot->irq_timestamp_cycles = irq_timestamp_cycles;
  slot->irq_period_cycles = irq_period_cycles;
  slot->timer_irq_missed_period_count = timer_irq_missed_period_count;
  for (uint32_t motor_index = 0U; motor_index < (uint32_t)BSP_MOTOR_COUNT; ++motor_index) {
    slot->encoder_raw[motor_index] = encoder_raw[motor_index];
  }

  /* 序号最后写入，作为该槽内容已经完整提交的标志。 */
  slot->tick_sequence = sequence;
  buffer->produced_sequence = sequence;
}

bool AppControlTickBuffer_Consume(
  AppControlTickBuffer *buffer,
  uint32_t notification_count,
  AppControlTickSample *sample)
{
  if (buffer == NULL || sample == NULL || notification_count == 0U) {
    return false;
  }

  const uint32_t target_sequence = buffer->consumed_sequence + notification_count;
  volatile const AppControlTickSample *const slot =
    &buffer->slots[AppControlTickBuffer_Index(target_sequence)];
  /* 序号不匹配表示目标槽尚未提交或已被后续中断覆盖，不能返回错配数据。 */
  if (slot->tick_sequence != target_sequence) {
    return false;
  }

  sample->tick_sequence = slot->tick_sequence;
  sample->irq_timestamp_cycles = slot->irq_timestamp_cycles;
  sample->irq_period_cycles = slot->irq_period_cycles;
  sample->timer_irq_missed_period_count = slot->timer_irq_missed_period_count;
  for (uint32_t motor_index = 0U; motor_index < (uint32_t)BSP_MOTOR_COUNT; ++motor_index) {
    sample->encoder_raw[motor_index] = slot->encoder_raw[motor_index];
  }
  buffer->consumed_sequence = target_sequence;
  return true;
}
