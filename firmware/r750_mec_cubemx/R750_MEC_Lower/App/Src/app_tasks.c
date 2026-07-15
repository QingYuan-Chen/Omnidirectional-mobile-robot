#include "app_tasks.h"

#include "FreeRTOS.h"
#include "app_control_timebase.h"
#include "app_control_timing.h"
#include "app_encoder_accumulator.h"
#include "app_imu.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "main.h"
#include "robot_config.h"
#include "task.h"

#include <string.h>

#define APP_EVENT_START             (1UL << 0U)
#define APP_EVENT_CONTROL_HEARTBEAT (1UL << 1U)
#define APP_EVENT_COMM_HEARTBEAT    (1UL << 2U)
#define APP_EVENT_IMU_HEARTBEAT     (1UL << 3U)
#define APP_EVENT_CRITICAL_HEARTBEATS \
  (APP_EVENT_CONTROL_HEARTBEAT | APP_EVENT_COMM_HEARTBEAT | APP_EVENT_IMU_HEARTBEAT)
#define APP_IMU_THREAD_FLAG_DATA_READY (1UL << 0U)

#define APP_CONTROL_STACK_BYTES (1536U)
#define APP_SAFETY_STACK_BYTES  (1024U)
#define APP_COMM_STACK_BYTES    (1536U)
#define APP_IMU_STACK_BYTES     (1536U)
#define APP_MONITOR_STACK_BYTES (1024U)

_Static_assert(
  ((ROBOT_CONFIG_CONTROL_RATE_HZ * ROBOT_CONFIG_CONTROL_HEARTBEAT_PERIOD_MS) % 1000U) == 0U,
  "control heartbeat period must contain an integer number of control ticks");
_Static_assert(ROBOT_CONFIG_CONTROL_HEARTBEAT_DIVIDER > 0U,
               "control heartbeat divider must be non-zero");

static osEventFlagsId_t runtime_events;
static osThreadId_t task_handles[APP_TASK_COUNT];
static AppRuntimeSnapshot runtime_snapshot;

static void AppTasks_Control(void *argument);
static void AppTasks_Safety(void *argument);
static void AppTasks_Comm(void *argument);
static void AppTasks_Imu(void *argument);
static void AppTasks_Monitor(void *argument);

static const osThreadAttr_t task_attributes[APP_TASK_COUNT] = {
  [APP_TASK_CONTROL] = {.name = "controlTask", .stack_size = APP_CONTROL_STACK_BYTES, .priority = osPriorityHigh},
  [APP_TASK_SAFETY] = {.name = "safetyTask", .stack_size = APP_SAFETY_STACK_BYTES, .priority = osPriorityHigh},
  [APP_TASK_COMM] = {.name = "commTask", .stack_size = APP_COMM_STACK_BYTES, .priority = osPriorityAboveNormal},
  [APP_TASK_IMU] = {.name = "imuTask", .stack_size = APP_IMU_STACK_BYTES, .priority = osPriorityAboveNormal},
  [APP_TASK_MONITOR] = {.name = "monitorTask", .stack_size = APP_MONITOR_STACK_BYTES, .priority = osPriorityLow},
};

static osThreadFunc_t const task_functions[APP_TASK_COUNT] = {
  [APP_TASK_CONTROL] = AppTasks_Control,
  [APP_TASK_SAFETY] = AppTasks_Safety,
  [APP_TASK_COMM] = AppTasks_Comm,
  [APP_TASK_IMU] = AppTasks_Imu,
  [APP_TASK_MONITOR] = AppTasks_Monitor,
};

static bool AppTasks_WaitForStart(void)
{
  const uint32_t flags = osEventFlagsWait(runtime_events, APP_EVENT_START, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
  return (flags & osFlagsError) == 0U;
}

static void AppTasks_StopCreatedThreads(void)
{
  for (uint32_t i = 0U; i < APP_TASK_COUNT; ++i) {
    if (task_handles[i] != NULL) {
      (void)osThreadTerminate(task_handles[i]);
      task_handles[i] = NULL;
    }
  }

  if (runtime_events != NULL) {
    (void)osEventFlagsDelete(runtime_events);
    runtime_events = NULL;
  }
}

static _Noreturn void AppTasks_FailCurrentThread(void)
{
  (void)AppControlTimebase_Stop();
  BspMotor_EmergencyCoastAll();
  osThreadExit();
  for (;;) {
  }
}

BspStatus AppTasks_Create(void)
{
  if (runtime_events != NULL) {
    return BSP_BUSY;
  }

  memset(task_handles, 0, sizeof(task_handles));
  memset(&runtime_snapshot, 0, sizeof(runtime_snapshot));

  runtime_events = osEventFlagsNew(NULL);
  if (runtime_events == NULL) {
    return BSP_ERROR;
  }

  for (uint32_t i = 0U; i < APP_TASK_COUNT; ++i) {
    task_handles[i] = osThreadNew(task_functions[i], NULL, &task_attributes[i]);
    if (task_handles[i] == NULL) {
      AppTasks_StopCreatedThreads();
      return BSP_ERROR;
    }
  }

  if ((osEventFlagsSet(runtime_events, APP_EVENT_START) & osFlagsError) != 0U) {
    AppTasks_StopCreatedThreads();
    return BSP_ERROR;
  }

  return BSP_OK;
}

BspStatus AppTasks_GetSnapshot(AppRuntimeSnapshot *snapshot)
{
  if (snapshot == NULL) {
    return BSP_INVALID_ARG;
  }

  taskENTER_CRITICAL();
  *snapshot = runtime_snapshot;
  taskEXIT_CRITICAL();
  return BSP_OK;
}

static void AppTasks_Control(void *argument)
{
  (void)argument;
  if (!AppTasks_WaitForStart()) {
    AppTasks_FailCurrentThread();
  }

  BspMotor_BrakeAll();
  uint16_t initial_encoder_raw[BSP_MOTOR_COUNT];
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    initial_encoder_raw[i] = BspEncoder_ReadRaw((BspMotorId)i);
  }

  AppEncoderAccumulator encoder_accumulator;
  AppControlTiming control_timing;
  if (!AppEncoderAccumulator_Init(&encoder_accumulator, initial_encoder_raw) ||
      !AppControlTiming_Init(&control_timing, SystemCoreClock, ROBOT_CONFIG_CONTROL_RATE_HZ) ||
      AppControlTimebase_Start() != BSP_OK) {
    AppTasks_FailCurrentThread();
  }

  uint32_t heartbeat_divider = 0U;
  bool heartbeat_pending = false;

  for (;;) {
    AppControlTick tick;
    int16_t encoder_delta[BSP_MOTOR_COUNT];
    int64_t encoder_total[BSP_MOTOR_COUNT];
    if (AppControlTimebase_Wait(&tick) != BSP_OK) {
      AppTasks_FailCurrentThread();
    }

    /* 发布的是上一个已经完整结束的控制周期；本次发布开销计入当前周期 WCET。 */
    if (heartbeat_pending) {
      AppControlTimingSnapshot timing_snapshot;
      if (!AppControlTiming_GetSnapshot(&control_timing, &timing_snapshot)) {
        AppTasks_FailCurrentThread();
      }
      taskENTER_CRITICAL();
      runtime_snapshot.control_timing = timing_snapshot;
      taskEXIT_CRITICAL();
      (void)osEventFlagsSet(runtime_events, APP_EVENT_CONTROL_HEARTBEAT);
      heartbeat_pending = false;
    }

    if (!AppEncoderAccumulator_Update(
          &encoder_accumulator, tick.encoder_raw, encoder_delta, encoder_total)) {
      AppTasks_FailCurrentThread();
    }

    AppControlTiming_RecordWake(
      &control_timing,
      tick.tick_sequence,
      tick.irq_timestamp_cycles,
      tick.irq_period_cycles,
      tick.timer_irq_missed_period_count,
      tick.task_wake_cycles,
      tick.notification_count);

    taskENTER_CRITICAL();
    memcpy(runtime_snapshot.encoder_raw, tick.encoder_raw, sizeof(tick.encoder_raw));
    memcpy(runtime_snapshot.encoder_delta, encoder_delta, sizeof(encoder_delta));
    memcpy(runtime_snapshot.encoder_total, encoder_total, sizeof(encoder_total));
    const bool fault_latched = runtime_snapshot.fault_latched;
    taskEXIT_CRITICAL();

    if (!fault_latched) {
      BspMotor_BrakeAll();
    }

    heartbeat_divider++;
    if (heartbeat_divider >= ROBOT_CONFIG_CONTROL_HEARTBEAT_DIVIDER) {
      heartbeat_pending = true;
      heartbeat_divider = 0U;
    }

    AppControlTiming_RecordComplete(&control_timing, AppControlTimebase_GetCycleCount());
  }
}

static void AppTasks_Safety(void *argument)
{
  (void)argument;
  if (!AppTasks_WaitForStart()) {
    AppTasks_FailCurrentThread();
  }

  TickType_t last_wake = xTaskGetTickCount();
  vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONFIG_SAFETY_PERIOD_MS));

  for (;;) {
    const uint32_t flags = osEventFlagsGet(runtime_events);
    const bool tasks_healthy = (flags & osFlagsError) == 0U &&
                               (flags & APP_EVENT_CRITICAL_HEARTBEATS) == APP_EVENT_CRITICAL_HEARTBEATS;
    (void)osEventFlagsClear(runtime_events, APP_EVENT_CRITICAL_HEARTBEATS);

    bool stop_motion = false;
    taskENTER_CRITICAL();
    const bool imu_healthy = runtime_snapshot.imu.health == APP_IMU_HEALTH_HEALTHY;
    runtime_snapshot.critical_tasks_alive = tasks_healthy;
    if (tasks_healthy) {
      runtime_snapshot.health_miss_count = 0U;
    } else if (runtime_snapshot.health_miss_count < UINT32_MAX) {
      runtime_snapshot.health_miss_count++;
    }

    const bool inhibit_motion = runtime_snapshot.fault_latched || !tasks_healthy || !imu_healthy;
    if (inhibit_motion && !runtime_snapshot.motion_inhibited) {
      stop_motion = true;
    }
    runtime_snapshot.motion_inhibited = inhibit_motion;

    if (!runtime_snapshot.fault_latched &&
        runtime_snapshot.health_miss_count >= ROBOT_CONFIG_HEALTH_MISS_LIMIT) {
      runtime_snapshot.fault_latched = true;
      stop_motion = true;
    }
    taskEXIT_CRITICAL();

    if (stop_motion) {
      BspMotor_EmergencyCoastAll();
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONFIG_SAFETY_PERIOD_MS));
  }
}

static void AppTasks_Comm(void *argument)
{
  (void)argument;
  if (!AppTasks_WaitForStart()) {
    AppTasks_FailCurrentThread();
  }

  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    BspUart_Service();
    (void)osEventFlagsSet(runtime_events, APP_EVENT_COMM_HEARTBEAT);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONFIG_COMM_SERVICE_PERIOD_MS));
  }
}

static void AppTasks_Imu(void *argument)
{
  (void)argument;
  if (!AppTasks_WaitForStart()) {
    AppTasks_FailCurrentThread();
  }

  for (;;) {
    (void)osThreadFlagsWait(
      APP_IMU_THREAD_FLAG_DATA_READY, osFlagsWaitAny, pdMS_TO_TICKS(ROBOT_CONFIG_IMU_PERIOD_MS * 2U));
    AppImuOutput output;
    const BspStatus status = AppImu_Process(HAL_GetTick(), &output);

    taskENTER_CRITICAL();
    runtime_snapshot.imu = output;
    taskEXIT_CRITICAL();

    (void)osEventFlagsSet(runtime_events, APP_EVENT_IMU_HEARTBEAT);
    (void)status;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  if (gpio_pin == IMU_INT_Pin && task_handles[APP_TASK_IMU] != NULL) {
    (void)osThreadFlagsSet(task_handles[APP_TASK_IMU], APP_IMU_THREAD_FLAG_DATA_READY);
  }
}

static void AppTasks_Monitor(void *argument)
{
  (void)argument;
  if (!AppTasks_WaitForStart()) {
    AppTasks_FailCurrentThread();
  }

  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    uint16_t battery_millivolts = 0U;
    const BspStatus battery_status = BspAdc_ReadBatteryMillivolts(&battery_millivolts);
    uint32_t stack_free_bytes[APP_TASK_COUNT];
    for (uint32_t i = 0U; i < APP_TASK_COUNT; ++i) {
      stack_free_bytes[i] = osThreadGetStackSpace(task_handles[i]);
    }

    taskENTER_CRITICAL();
    if (battery_status == BSP_OK) {
      runtime_snapshot.battery_millivolts = battery_millivolts;
    }
    memcpy(runtime_snapshot.stack_free_bytes, stack_free_bytes, sizeof(stack_free_bytes));
    taskEXIT_CRITICAL();

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONFIG_MONITOR_PERIOD_MS));
  }
}
