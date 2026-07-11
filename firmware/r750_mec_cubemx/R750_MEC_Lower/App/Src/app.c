#include "app.h"

#include "app_tasks.h"
#include "app_imu.h"
#include "bsp.h"
#include "bsp_beep.h"
#include "bsp_motor.h"
#include "cmsis_os.h"
#include "main.h"
#include "robot_config.h"

static void App_FatalLoop(void)
{
  BspMotor_EmergencyCoastAll();
  for (;;) {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    osDelay(100U);
  }
}

static bool App_WaitForRuntimeReady(void)
{
  const uint32_t required_imu_flags =
    (uint32_t)APP_IMU_FLAG_CALIBRATED | (uint32_t)APP_IMU_FLAG_DATA_VALID |
    (uint32_t)APP_IMU_FLAG_FILTER_INITIALIZED | (uint32_t)APP_IMU_FLAG_FILTER_CONVERGED |
    (uint32_t)APP_IMU_FLAG_TIMESTAMP_VALID | (uint32_t)APP_IMU_FLAG_TILT_VALID;
  const uint32_t rejected_imu_flags = (uint32_t)APP_IMU_FLAG_SENSOR_FAULT | (uint32_t)APP_IMU_FLAG_DATA_STALE;
  const uint32_t started_at = HAL_GetTick();

  do {
    AppRuntimeSnapshot snapshot;
    if (AppTasks_GetSnapshot(&snapshot) == BSP_OK) {
      if (snapshot.fault_latched) {
        return false;
      }
      if ((snapshot.imu.flags & required_imu_flags) == required_imu_flags &&
          (snapshot.imu.flags & rejected_imu_flags) == 0U) {
        return true;
      }
    }
    osDelay(10U);
  } while ((HAL_GetTick() - started_at) < ROBOT_CONFIG_RUNTIME_READY_TIMEOUT_MS);

  return false;
}

void App_DefaultTask(void *argument)
{
  (void)argument;
  const BspStatus init_status = Bsp_Init();

  if (init_status != BSP_OK) {
    App_FatalLoop();
  }

  if (AppImu_Calibrate() != BSP_OK) {
    App_FatalLoop();
  }

  if (AppTasks_Create() != BSP_OK) {
    App_FatalLoop();
  }

  if (!App_WaitForRuntimeReady()) {
    App_FatalLoop();
  }

  BspBeep_Set(true);
  osDelay(500U);
  BspBeep_Set(false);

  osThreadExit();
}
