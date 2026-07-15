#ifndef APP_CONTROL_TICK_BUFFER_H
#define APP_CONTROL_TICK_BUFFER_H

#include "bsp_types.h"
#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/* 该环形缓冲区把计数型任务通知与对应的中断快照绑定，避免只读“最新值”错配。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 单次 TIM7 更新事件在中断上下文采集的固定长度快照。 */
typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t irq_period_cycles;
  uint32_t timer_irq_missed_period_count;
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
} AppControlTickSample;

/* 生产序号由中断推进，消费序号只由控制任务推进。 */
typedef struct {
  volatile AppControlTickSample slots[ROBOT_CONFIG_CONTROL_TICK_RING_SIZE];
  volatile uint32_t produced_sequence;
  uint32_t consumed_sequence;
} AppControlTickBuffer;

/* 清零全部槽位和生产、消费序号。 */
void AppControlTickBuffer_Init(AppControlTickBuffer *buffer);
/* 中断侧发布快照；序号最后写入，表示该槽内容完整。 */
void AppControlTickBuffer_PublishFromIsr(
  AppControlTickBuffer *buffer,
  uint32_t irq_timestamp_cycles,
  uint32_t irq_period_cycles,
  uint32_t timer_irq_missed_period_count,
  const uint16_t encoder_raw[BSP_MOTOR_COUNT]);
/* 任务侧按通知累计数消费目标序号，槽位已被覆盖时返回失败。 */
bool AppControlTickBuffer_Consume(
  AppControlTickBuffer *buffer,
  uint32_t notification_count,
  AppControlTickSample *sample);

#ifdef __cplusplus
}
#endif

#endif
