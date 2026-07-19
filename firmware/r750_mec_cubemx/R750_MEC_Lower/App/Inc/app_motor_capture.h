#ifndef APP_MOTOR_CAPTURE_H
#define APP_MOTOR_CAPTURE_H

#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * G2 电机辨识使用的固定内存高速记录器。
 *
 * 控制任务以候选 1 kHz 节拍写入单个 MA 样本，通信任务只在记录停止后逐条复制并异步
 * 导出。模块不访问 HAL/RTOS、不分配动态内存，也不覆盖已经写入的样本。跨任务调用者
 * 必须在任务编排层提供短临界区，保证开始、停止、状态和样本复制不会与单个写入交叉。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define APP_MOTOR_CAPTURE_SCHEMA_VERSION (1U)

#define APP_MOTOR_CAPTURE_FLAG_RUNTIME_READY       (1U << 0U)
#define APP_MOTOR_CAPTURE_FLAG_MOTION_INHIBITED    (1U << 1U)
#define APP_MOTOR_CAPTURE_FLAG_FAULT_LATCHED       (1U << 2U)
#define APP_MOTOR_CAPTURE_FLAG_MOTION_AVAILABLE    (1U << 3U)
#define APP_MOTOR_CAPTURE_FLAG_CRITICAL_TASKS_ALIVE (1U << 4U)

typedef enum {
  APP_MOTOR_CAPTURE_IDLE = 0,
  APP_MOTOR_CAPTURE_RECORDING,
  APP_MOTOR_CAPTURE_COMPLETE
} AppMotorCaptureState;

typedef enum {
  APP_MOTOR_CAPTURE_EVENT_STATUS = 0,
  APP_MOTOR_CAPTURE_EVENT_STARTED,
  APP_MOTOR_CAPTURE_EVENT_STOPPED,
  APP_MOTOR_CAPTURE_EVENT_BEGIN,
  APP_MOTOR_CAPTURE_EVENT_END,
  APP_MOTOR_CAPTURE_EVENT_REJECTED
} AppMotorCaptureEvent;

/*
 * 单个 1 kHz 样本保持 28 B：保留硬件节拍时间戳、当前唤醒延迟、上一完整周期 WCET、
 * MA 编码器原始值/增量、目标与实际 PWM、电池最近一次低频采样以及当周期电机/安全
 * 状态。previous_wcet_cycles 比本行 tick 早一个周期，离线工具必须按该语义对齐。
 */
typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t wake_latency_cycles;
  uint32_t previous_wcet_cycles;
  uint16_t encoder_raw_ma;
  int16_t encoder_delta_ma;
  int16_t target_pwm;
  int16_t applied_pwm;
  uint16_t battery_millivolts;
  uint8_t motor_state;
  uint8_t safety_flags;
} AppMotorCaptureSample;

typedef AppMotorCaptureSample AppMotorCaptureInput;

typedef struct {
  AppMotorCaptureState state;
  uint32_t sample_count;
  uint32_t capacity;
  uint32_t dropped_sample_count;
} AppMotorCaptureStatus;

typedef struct {
  AppMotorCaptureSample samples[ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY];
  uint32_t sample_count;
  uint32_t dropped_sample_count;
  AppMotorCaptureState state;
} AppMotorCapture;

void AppMotorCapture_Init(AppMotorCapture *capture);
bool AppMotorCapture_Start(AppMotorCapture *capture);
bool AppMotorCapture_Stop(AppMotorCapture *capture);
bool AppMotorCapture_Record(
  AppMotorCapture *capture,
  const AppMotorCaptureInput *input);
bool AppMotorCapture_GetStatus(
  const AppMotorCapture *capture,
  AppMotorCaptureStatus *status);
bool AppMotorCapture_GetSample(
  const AppMotorCapture *capture,
  uint32_t index,
  AppMotorCaptureSample *sample);

/*
 * 生成可由 Windows 工具严格解析的换行文本。
 * 事件行：MCAP,版本,事件,状态,样本数,容量,丢弃数
 * 样本行：MC,版本,索引,tick,irq,wake,previous_wcet,raw,delta,target,applied,
 * battery,state,flags
 */
bool AppMotorCapture_FormatEvent(
  AppMotorCaptureEvent event,
  const AppMotorCaptureStatus *status,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);
bool AppMotorCapture_FormatSample(
  uint32_t index,
  const AppMotorCaptureSample *sample,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);

#ifdef __cplusplus
}
#endif

#endif
