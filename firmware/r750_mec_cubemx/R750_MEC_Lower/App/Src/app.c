#include "app.h"

#include "app_tasks.h"
#include "app_imu.h"
#include "bsp.h"
#include "bsp_beep.h"
#include "bsp_motor.h"
#include "cmsis_os.h"
#include "main.h"
#include "robot_config.h"

/*
 * 应用启动流程采用失败即安全停机策略。
 * 默认任务只负责把 CubeMX 已建立的 RTOS 环境过渡到项目自有任务体系，不长期占用资源。
 * 启动顺序中的每一步都依赖前一步完整成功，任何异常都不会继续创建或放行运行任务，
 * 从源头避免“部分外设可用、执行器却已经接受命令”的不确定状态。
 */

static void App_FatalLoop(void)
{
  /*
   * 紧急空转在进入循环前执行一次即可关闭 PWM 基础设施；LED2 以 5 Hz 翻转、约 2.5 Hz
   * 完整闪烁提示启动失败。这里不尝试自动重启外设，因为失败阶段可能涉及总线或执行器
   * 配置不完整，未经人工检查的自动恢复会扩大风险。
   */
  BspMotor_EmergencyCoastAll();
  for (;;) {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    osDelay(100U);
  }
}

static bool App_WaitForRuntimeReady(void)
{
  /*
   * 默认任务不复制 IMU 标志组合，而是要求任务层用统一实时判据原子锁存 runtime_ready，
   * 再等待安全任务解除动态 motion_inhibited。这样启动检查、通信准入、控制最终门和
   * 安全策略共享同一真源；六轴 IMU 的绝对航向不可观仍由统一判据内部处理。
   */
  const uint32_t started_at = HAL_GetTick();

  do {
    const BspStatus ready_status =
      AppTasks_TrySetRuntimeReady();
    AppRuntimeSnapshot snapshot;
    if (AppTasks_GetSnapshot(&snapshot) == BSP_OK) {
      const uint32_t now_ms = HAL_GetTick();
      if (ready_status == BSP_ERROR ||
          snapshot.fault_latched) {
        return false;
      }
      if (ready_status == BSP_OK && snapshot.runtime_ready &&
          !snapshot.motion_inhibited &&
          AppImu_IsMotionUsable(&snapshot.imu, now_ms)) {
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
  /*
   * 板级初始化先建立安全硬件状态，静止标定随后建立 IMU 初始条件，最后才创建运行任务。
   * 若调换顺序，IMU 任务可能与标定过程争用传感器，或控制任务在电机 BSP 完成前运行。
   */
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

  /*
   * 500 ms 鸣笛只确认软件运行门槛已经满足，不代表板上 M1.3 或电机闭环验收完成。
   * 提示结束后主动退出，让 CMSIS-RTOS 回收默认任务资源。
   */
  BspBeep_Set(true);
  osDelay(500U);
  BspBeep_Set(false);

  osThreadExit();
}
