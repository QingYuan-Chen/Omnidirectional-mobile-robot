#ifndef APP_SPEED_CAPTURE_H
#define APP_SPEED_CAPTURE_H

#include "app_encoder_period_accumulator.h"
#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * G3 MA 轮速诊断固定内存记录器。
 *
 * 每个 TIM7 tick 保存同一时刻的 M 法输入和真实边沿周期聚合输入。该记录器与 G2 电机
 * 记录器在任务编排层互斥复用 CCMRAM；模块本身不访问 HAL/RTOS、不分配动态内存。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SPEED_CAPTURE_SCHEMA_VERSION (1U)

typedef enum {
  APP_SPEED_CAPTURE_IDLE = 0,
  APP_SPEED_CAPTURE_RECORDING,
  APP_SPEED_CAPTURE_COMPLETE
} AppSpeedCaptureState;

typedef enum {
  APP_SPEED_CAPTURE_EVENT_STATUS = 0,
  APP_SPEED_CAPTURE_EVENT_STARTED,
  APP_SPEED_CAPTURE_EVENT_STOPPED,
  APP_SPEED_CAPTURE_EVENT_BEGIN,
  APP_SPEED_CAPTURE_EVENT_END,
  APP_SPEED_CAPTURE_EVENT_REJECTED
} AppSpeedCaptureEvent;

/*
 * 单样本固定 28 B。irq_timestamp_cycles 与 tick_sequence 来自 TIM7；period 聚合也在该
 * TIM7 ISR 中完成，因此不会把控制 tick 后才到达的边沿错误归入本样本。
 */
typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t period_sum_cycles;
  uint32_t last_edge_age_cycles;
  uint32_t event_sequence;
  int16_t encoder_delta_ma;
  int16_t applied_pwm;
  uint16_t period_count;
  int8_t direction;
  uint8_t period_flags;
} AppSpeedCaptureSample;

typedef AppSpeedCaptureSample AppSpeedCaptureInput;

typedef struct {
  AppSpeedCaptureState state;
  uint32_t sample_count;
  uint32_t capacity;
  uint32_t dropped_sample_count;
  AppEncoderPeriodStats period_stats;
} AppSpeedCaptureStatus;

typedef struct {
  AppSpeedCaptureSample samples[ROBOT_CONFIG_SPEED_CAPTURE_CAPACITY];
  uint32_t sample_count;
  uint32_t dropped_sample_count;
  AppSpeedCaptureState state;
  AppEncoderPeriodStats period_stats;
} AppSpeedCapture;

void AppSpeedCapture_Init(AppSpeedCapture *capture);
bool AppSpeedCapture_Start(AppSpeedCapture *capture);
bool AppSpeedCapture_Stop(AppSpeedCapture *capture);
bool AppSpeedCapture_Record(
  AppSpeedCapture *capture,
  const AppSpeedCaptureInput *input);
bool AppSpeedCapture_SetPeriodStats(
  AppSpeedCapture *capture,
  const AppEncoderPeriodStats *stats);
bool AppSpeedCapture_GetStatus(
  const AppSpeedCapture *capture,
  AppSpeedCaptureStatus *status);
bool AppSpeedCapture_GetSample(
  const AppSpeedCapture *capture,
  uint32_t index,
  AppSpeedCaptureSample *sample);

/*
 * 事件：SCAP,版本,事件,状态,样本数,容量,丢样,非法方向,零周期,聚合丢弃,换向清空
 * 样本：SC,版本,索引,tick,irq,delta,applied,period_sum,period_count,last_age,
 *       event_sequence,direction,flags
 */
bool AppSpeedCapture_FormatEvent(
  AppSpeedCaptureEvent event,
  const AppSpeedCaptureStatus *status,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);
bool AppSpeedCapture_FormatSample(
  uint32_t index,
  const AppSpeedCaptureSample *sample,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);

#ifdef __cplusplus
}
#endif

#endif
