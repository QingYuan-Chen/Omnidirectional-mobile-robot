#ifndef APP_CONTROL_TICK_BUFFER_H
#define APP_CONTROL_TICK_BUFFER_H

#include "app_encoder_period_accumulator.h"
#include "bsp_types.h"
#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * TIM7 中断到控制任务之间的单生产者、单消费者快照缓冲区。
 *
 * FreeRTOS 计数型任务通知只能保存“累计到达了多少个节拍”，不能携带每个节拍对应的
 * 中断时间戳和编码器值。本模块用同一递增序号把通知数与环形槽位绑定，使控制任务在
 * 被延迟后仍能定位它应消费的目标快照。若目标槽已被后续中断覆盖，模块明确返回失败，
 * 不会把最新编码器值冒充成旧节拍数据。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 单次 TIM7 更新事件的不可变快照。
 * 时间字段单位为 CPU 周期；encoder_raw 按 BspMotorId 顺序保存四路 16 位硬件计数。
 * tick_sequence 同时承担数据版本标志，必须在其余字段写完后最后发布。
 */
typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t irq_period_cycles;
  uint32_t timer_irq_missed_period_count;
  AppEncoderPeriodSnapshot encoder_period_ma;
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
} AppControlTickSample;

/*
 * 缓冲区状态。produced_sequence 只由 TIM7 中断更新，consumed_sequence 只由控制任务更新；
 * slots 数量必须是 2 的幂，索引可用位与计算。当前实现依靠单核 MCU、volatile 访问及
 * 调用处临界区维持发布顺序，不支持多个生产者或多个消费者。
 */
typedef struct {
  volatile AppControlTickSample slots[ROBOT_CONFIG_CONTROL_TICK_RING_SIZE];
  volatile uint32_t produced_sequence;
  uint32_t consumed_sequence;
} AppControlTickBuffer;

/* 清零所有槽位和序号；参数为空时不执行操作，只应在时基尚未启动时调用。 */
void AppControlTickBuffer_Init(AppControlTickBuffer *buffer);
/*
 * 从 TIM7 中断发布一个新快照。
 * encoder_raw 必须指向四路同步读取结果；函数固定执行，不阻塞。模块先写时间和编码器
 * 字段，再写槽位序号和全局生产序号，使任务侧不会观察到半写入快照。
 */
void AppControlTickBuffer_PublishFromIsr(
  AppControlTickBuffer *buffer,
  uint32_t irq_timestamp_cycles,
  uint32_t irq_period_cycles,
  uint32_t timer_irq_missed_period_count,
  const AppEncoderPeriodSnapshot *encoder_period_ma,
  const uint16_t encoder_raw[BSP_MOTOR_COUNT]);
/*
 * 控制任务按本次取得的 notification_count 消费目标序号。
 * 成功时复制完整快照并推进 consumed_sequence；通知数为零、参数无效、目标尚未提交或
 * 已被覆盖时返回 false，且不推进消费位置。调用者应把失败视为确定性控制链故障。
 */
bool AppControlTickBuffer_Consume(
  AppControlTickBuffer *buffer,
  uint32_t notification_count,
  AppControlTickSample *sample);

#ifdef __cplusplus
}
#endif

#endif
