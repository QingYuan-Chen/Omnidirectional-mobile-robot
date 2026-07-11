#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "app_imu.h"
#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_TASK_CONTROL = 0,
  APP_TASK_SAFETY,
  APP_TASK_COMM,
  APP_TASK_IMU,
  APP_TASK_MONITOR,
  APP_TASK_COUNT
} AppTaskId;

typedef struct {
  int16_t encoder_delta[BSP_MOTOR_COUNT];
  AppImuOutput imu;
  uint16_t battery_millivolts;
  uint32_t health_miss_count;
  uint32_t stack_free_bytes[APP_TASK_COUNT];
  bool critical_tasks_alive;
  bool motion_inhibited;
  bool fault_latched;
} AppRuntimeSnapshot;

BspStatus AppTasks_Create(void);
BspStatus AppTasks_GetSnapshot(AppRuntimeSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
