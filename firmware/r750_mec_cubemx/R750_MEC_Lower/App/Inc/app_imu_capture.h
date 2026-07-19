#ifndef APP_IMU_CAPTURE_H
#define APP_IMU_CAPTURE_H

#include "app_imu.h"
#include "robot_config.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 阶段 4 使用的固定内存 IMU 新样本记录器。
 *
 * IMU 任务只把 AppImu_Process 真正接受且 sequence 前进的原始样本写入本模块；通信
 * 任务只在记录停止后逐条导出。记录器不访问 HAL/RTOS、不分配内存、满后停止且不覆盖
 * 旧样本，并在任务编排层与 MOTOR/G3_SPEED 记录器互斥复用 CCMRAM。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define APP_IMU_CAPTURE_SCHEMA_VERSION (1U)

typedef enum {
  APP_IMU_CAPTURE_IDLE = 0,
  APP_IMU_CAPTURE_RECORDING,
  APP_IMU_CAPTURE_COMPLETE
} AppImuCaptureState;

typedef enum {
  APP_IMU_CAPTURE_EVENT_STATUS = 0,
  APP_IMU_CAPTURE_EVENT_STARTED,
  APP_IMU_CAPTURE_EVENT_STOPPED,
  APP_IMU_CAPTURE_EVENT_BEGIN,
  APP_IMU_CAPTURE_EVENT_END,
  APP_IMU_CAPTURE_EVENT_REJECTED
} AppImuCaptureEvent;

/*
 * 单样本固定 36 B，覆盖芯片原始六轴、温度、STATUS0、传感器/主机时刻、应用标志、
 * 累计源丢样和健康级。
 * 不保存浮点换算或滤波值，避免在高速记录路径复制非权威派生量。
 */
typedef struct {
  uint32_t sequence;
  uint32_t sensor_timestamp;
  uint32_t host_tick_ms;
  uint32_t flags;
  uint32_t dropped_sample_count;
  int16_t acceleration[3];
  int16_t angular_rate[3];
  int16_t temperature;
  uint8_t sensor_status;
  uint8_t health;
} AppImuCaptureSample;

typedef AppImuCaptureSample AppImuCaptureInput;

typedef struct {
  AppImuCaptureState state;
  uint32_t sample_count;
  uint32_t capacity;
  uint32_t dropped_sample_count;
  uint32_t duplicate_sequence_count;
  uint32_t source_gap_count;
} AppImuCaptureStatus;

typedef struct {
  AppImuCaptureSample samples[ROBOT_CONFIG_IMU_CAPTURE_CAPACITY];
  uint32_t sample_count;
  uint32_t dropped_sample_count;
  uint32_t duplicate_sequence_count;
  uint32_t source_gap_count;
  uint32_t last_sequence;
  AppImuCaptureState state;
  bool has_last_sequence;
} AppImuCapture;

void AppImuCapture_Init(AppImuCapture *capture);
bool AppImuCapture_Start(AppImuCapture *capture);
bool AppImuCapture_Stop(AppImuCapture *capture);
bool AppImuCapture_Record(
  AppImuCapture *capture,
  const AppImuCaptureInput *input);
bool AppImuCapture_GetStatus(
  const AppImuCapture *capture,
  AppImuCaptureStatus *status);
bool AppImuCapture_GetSample(
  const AppImuCapture *capture,
  uint32_t index,
  AppImuCaptureSample *sample);

/*
 * 事件：ICAP,版本,事件,状态,样本数,容量,缓冲丢弃,重复序号,源序号缺口
 * 样本：IC,版本,索引,sequence,sensor_timestamp,host_tick,flags,source_dropped,
 *       ax,ay,az,gx,gy,gz,temperature,status,health
 */
bool AppImuCapture_FormatEvent(
  AppImuCaptureEvent event,
  const AppImuCaptureStatus *status,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);
bool AppImuCapture_FormatSample(
  uint32_t index,
  const AppImuCaptureSample *sample,
  uint8_t *buffer,
  uint16_t capacity,
  uint16_t *length);

#ifdef __cplusplus
}
#endif

#endif
