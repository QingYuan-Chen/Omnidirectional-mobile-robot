#include "app_tasks.h"

#include "FreeRTOS.h"
#include "app_comm_protocol.h"
#include "app_control_timebase.h"
#include "app_control_timing.h"
#include "app_debug_uart_config.h"
#include "app_encoder_accumulator.h"
#include "app_imu.h"
#include "app_motion_gate.h"
#include "app_motor_capture.h"
#include "app_motor_open_loop.h"
#include "app_safety_policy.h"
#include "app_speed_capture.h"
#include "app_telemetry.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "main.h"
#include "robot_config.h"
#include "task.h"

#include <string.h>

/*
 * 任务编排层是各纯算法模块、BSP 与 RTOS 之间的唯一集成边界。
 *
 * 五个长期任务各自拥有私有状态，只通过定长消息队列、事件标志和 runtime_snapshot 交换
 * 信息。控制任务由 TIM7 确定性唤醒；安全任务按窗口清点心跳并拥有常规执行器覆盖权；
 * 通信和 IMU 任务即使遇到链路/传感器错误也继续服务与上报心跳，让数据故障和任务失联
 * 能被分开诊断。通信 ESTOP 和任一任务失败也可直接紧急锁存；任何内部契约破坏都收敛
 * 到全局故障锁存和电机紧急空转。
 */

#define APP_EVENT_START             (1UL << 0U)
#define APP_EVENT_CONTROL_HEARTBEAT (1UL << 1U)
#define APP_EVENT_COMM_HEARTBEAT    (1UL << 2U)
#define APP_EVENT_IMU_HEARTBEAT     (1UL << 3U)
#define APP_EVENT_CRITICAL_HEARTBEATS \
  (APP_EVENT_CONTROL_HEARTBEAT | APP_EVENT_COMM_HEARTBEAT | APP_EVENT_IMU_HEARTBEAT)
#define APP_IMU_THREAD_FLAG_DATA_READY (1UL << 0U)
#define APP_CAPTURE_EVENT_QUEUE_DEPTH (4U)

#define APP_CONTROL_STACK_BYTES (1536U)
#define APP_SAFETY_STACK_BYTES  (1024U)
#define APP_COMM_STACK_BYTES    (2048U)
#define APP_IMU_STACK_BYTES     (1536U)
#define APP_MONITOR_STACK_BYTES (1024U)

_Static_assert(
  ((ROBOT_CONFIG_CONTROL_RATE_HZ * ROBOT_CONFIG_CONTROL_HEARTBEAT_PERIOD_MS) % 1000U) == 0U,
  "control heartbeat period must contain an integer number of control ticks");
_Static_assert(ROBOT_CONFIG_CONTROL_HEARTBEAT_DIVIDER > 0U,
               "control heartbeat divider must be non-zero");
_Static_assert(ROBOT_CONFIG_MOTOR_COMMAND_QUEUE_DEPTH > 0U,
               "motor command queue depth must be non-zero");
_Static_assert(ROBOT_CONFIG_TELEMETRY_PERIOD_MS > 0U,
               "telemetry period must be non-zero");
_Static_assert(ROBOT_CONFIG_IMUQ_TELEMETRY_PERIOD_MS > 0U,
               "IMUQ telemetry period must be non-zero");
_Static_assert(ROBOT_CONFIG_RES_TELEMETRY_PERIOD_MS > 0U,
               "RES telemetry period must be non-zero");
_Static_assert(ROBOT_CONFIG_EVENT_TELEMETRY_MIN_PERIOD_MS > 0U,
               "EVENT telemetry minimum period must be non-zero");
_Static_assert(ROBOT_CONFIG_UART_TX_CAPTURE_RESERVED_SLOTS > 0U,
               "capture export must reserve at least one UART TX slot");
_Static_assert(
  ROBOT_CONFIG_UART_TX_CAPTURE_RESERVED_SLOTS <
    ROBOT_CONFIG_UART_TX_QUEUE_DEPTH,
  "capture export reservation must leave at least one export slot");
_Static_assert(
  ROBOT_CONFIG_DEBUG_UART_PORT == BSP_UART_ROS ||
  ROBOT_CONFIG_DEBUG_UART_PORT == BSP_UART_TTL,
  "debug UART port must name a supported BSP UART");

static osEventFlagsId_t runtime_events;
static osMessageQueueId_t motor_command_queue;
static osThreadId_t task_handles[APP_TASK_COUNT];

typedef enum {
  APP_CAPTURE_MODE_MOTOR = 0,
  APP_CAPTURE_MODE_SPEED
} AppCaptureMode;

typedef union {
  AppMotorCapture motor;
  AppSpeedCapture speed;
} AppCaptureStorage;

/*
 * CCMRAM 不能被 DMA 访问，但记录与导出都由 CPU 逐样本复制，适合保存大容量高速缓冲。
 * 独立 NOLOAD 段不会占用主 SRAM 或 Flash；AppTasks_Create 会显式初始化全部控制字段。
 * G2 电机与 G3_SPEED 记录器通过 union 互斥复用同一段，禁止同时保留两份 61.6 KB 数组。
 */
static AppCaptureStorage capture_storage
  __attribute__((section(".motor_capture"), aligned(4)));
static volatile AppCaptureMode capture_mode;

#define APP_MOTOR_CAPTURE_STORAGE (&capture_storage.motor)
#define APP_SPEED_CAPTURE_STORAGE (&capture_storage.speed)

_Static_assert(sizeof(AppCaptureStorage) <= (64U * 1024U),
               "capture union must fit in 64 KiB CCMRAM");
/*
 * runtime_snapshot 的常规数据按字段组划分单一写入者，安全状态和 motion_gate 是经过
 * 明确定义的多写入者例外。短临界区只用于内存复制或最终失败标志更新，禁止在
 * taskENTER_CRITICAL 内调用可能阻塞的 HAL/RTOS 接口；其他安全写入与 motion_gate 更新
 * 使用 vTaskSuspendAll 形成全序，允许 TIM7 中断继续到达但阻止相关任务互相切换。
 */
static AppRuntimeSnapshot runtime_snapshot;

static void AppTasks_Control(void *argument);
static void AppTasks_Safety(void *argument);
static void AppTasks_Comm(void *argument);
static void AppTasks_Imu(void *argument);
static void AppTasks_Monitor(void *argument);
static BspStatus AppTasks_ApplyMotorOutput(const AppMotorOpenLoopOutput *output);
static BspStatus AppTasks_CommitControlMotorOutput(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  const AppMotionGatedRequest *request,
  bool has_queued_request);
static void AppTasks_LatchCommEstop(void);
static bool AppTasks_QueueMotorCommand(
  AppCommProtocol *protocol,
  const AppCommCommand *command,
  AppCommRuntimeSnapshot *communication);
static uint32_t AppTasks_AddSaturated(uint32_t value, uint32_t increment);
static uint32_t AppTasks_SumUartErrors(const BspUartStats *stats);
static uint32_t AppTasks_SumUartTxFaults(const BspUartStats *stats);
static bool AppTasks_RefreshMotionGate(
  uint32_t now_ms,
  bool *motion_available);
static void AppTasks_FillTelemetryInput(
  const AppRuntimeSnapshot *snapshot,
  uint32_t now_ms,
  AppTelemetryInput *input);
static bool AppTasks_IsMotorStoppedForCapture(
  const AppRuntimeSnapshot *snapshot);

typedef enum {
  APP_CAPTURE_EXPORT_IDLE = 0,
  APP_CAPTURE_EXPORT_BEGIN,
  APP_CAPTURE_EXPORT_SAMPLES,
  APP_CAPTURE_EXPORT_END
} AppCaptureExportState;

typedef struct {
  AppCaptureMode mode;
  union {
    AppMotorCaptureEvent motor;
    AppSpeedCaptureEvent speed;
  } event;
  union {
    AppMotorCaptureStatus motor;
    AppSpeedCaptureStatus speed;
  } status;
} AppCapturePendingEvent;

typedef struct {
  AppCapturePendingEvent entries[APP_CAPTURE_EVENT_QUEUE_DEPTH];
  uint32_t head;
  uint32_t count;
} AppCaptureEventQueue;

static bool AppTasks_MotorCaptureEventPush(
  AppCaptureEventQueue *queue,
  AppMotorCaptureEvent event,
  const AppMotorCaptureStatus *status)
{
  if (queue == NULL || status == NULL ||
      queue->count >= APP_CAPTURE_EVENT_QUEUE_DEPTH) {
    return false;
  }
  const uint32_t index =
    (queue->head + queue->count) % APP_CAPTURE_EVENT_QUEUE_DEPTH;
  queue->entries[index].mode = APP_CAPTURE_MODE_MOTOR;
  queue->entries[index].event.motor = event;
  queue->entries[index].status.motor = *status;
  queue->count++;
  return true;
}

static bool AppTasks_SpeedCaptureEventPush(
  AppCaptureEventQueue *queue,
  AppSpeedCaptureEvent event,
  const AppSpeedCaptureStatus *status)
{
  if (queue == NULL || status == NULL ||
      queue->count >= APP_CAPTURE_EVENT_QUEUE_DEPTH) {
    return false;
  }
  const uint32_t index =
    (queue->head + queue->count) % APP_CAPTURE_EVENT_QUEUE_DEPTH;
  queue->entries[index].mode = APP_CAPTURE_MODE_SPEED;
  queue->entries[index].event.speed = event;
  queue->entries[index].status.speed = *status;
  queue->count++;
  return true;
}

static bool AppTasks_CaptureEventPeek(
  const AppCaptureEventQueue *queue,
  AppCapturePendingEvent *event)
{
  if (queue == NULL || event == NULL || queue->count == 0U) {
    return false;
  }
  *event = queue->entries[queue->head];
  return true;
}

static bool AppTasks_CaptureEventPop(AppCaptureEventQueue *queue)
{
  if (queue == NULL || queue->count == 0U) {
    return false;
  }
  queue->head = (queue->head + 1U) % APP_CAPTURE_EVENT_QUEUE_DEPTH;
  queue->count--;
  return true;
}

static const osThreadAttr_t task_attributes[APP_TASK_COUNT] = {
  /*
   * 控制与安全同为高优先级，靠执行器仲裁区保证最终安全门原子性；通信与 IMU 次一级，
   * 监控最低。栈大小以字节给 CMSIS-RTOS，运行后由 Monitor 持续记录剩余空间供板测复核。
   */
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
  /*
   * START 使用 osFlagsNoClear，成为一次设置、所有任务都可观察的启动闩锁。任何任务等待
   * 失败都视为 RTOS 基础设施故障，不能单独带病继续。
   */
  const uint32_t flags = osEventFlagsWait(runtime_events, APP_EVENT_START, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
  return (flags & osFlagsError) == 0U;
}

static void AppTasks_StopCreatedThreads(void)
{
  /* 创建中途失败的逆向清理路径；只处理已经非空的句柄，最终恢复可再次 Create 的状态。 */
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

  if (motor_command_queue != NULL) {
    (void)osMessageQueueDelete(motor_command_queue);
    motor_command_queue = NULL;
  }
}

static _Noreturn void AppTasks_FailCurrentThread(void)
{
  /*
   * 运行期不可恢复故障的统一收敛点。先在临界区发布禁止与锁存，再停止控制时基并关闭
   * 电机 PWM。当前任务退出后不尝试局部重建，因为其他任务可能已经依赖其数据所有权。
   */
  taskENTER_CRITICAL();
  runtime_snapshot.motion_inhibited = true;
  runtime_snapshot.fault_latched = true;
  (void)AppMotionGate_Update(&runtime_snapshot.motion_gate, false);
  taskEXIT_CRITICAL();
  (void)AppControlTimebase_Stop();
  BspMotor_EmergencyCoastAll();
  osThreadExit();
  for (;;) {
  }
}

static uint32_t AppTasks_AddSaturated(uint32_t value, uint32_t increment)
{
  if (increment > (UINT32_MAX - value)) {
    return UINT32_MAX;
  }
  return value + increment;
}

static uint32_t AppTasks_SumUartErrors(const BspUartStats *stats)
{
  uint32_t total = 0U;
  total = AppTasks_AddSaturated(total, stats->parity_error_count);
  total = AppTasks_AddSaturated(total, stats->noise_error_count);
  total = AppTasks_AddSaturated(total, stats->framing_error_count);
  total = AppTasks_AddSaturated(total, stats->overrun_error_count);
  total = AppTasks_AddSaturated(total, stats->rx_restart_failure_count);
  return total;
}

static uint32_t AppTasks_SumUartTxFaults(const BspUartStats *stats)
{
  uint32_t total = 0U;
  total = AppTasks_AddSaturated(total, stats->tx_queue_full_count);
  total = AppTasks_AddSaturated(total, stats->tx_start_failure_count);
  total = AppTasks_AddSaturated(total, stats->tx_completion_recovery_count);
  return total;
}

static bool AppTasks_RefreshMotionGate(
  uint32_t now_ms,
  bool *motion_available)
{
  if (motion_available == NULL) {
    return false;
  }

  /*
   * 调用者必须已经挂起调度器，使快照条件、许可下降沿和代际更新对控制、安全、通信和
   * IMU 四个任务形成同一全序。实时年龄直接使用 now_ms，不能只信任快照中的旧 age。
   */
  const bool imu_motion_usable =
    AppImu_IsMotionUsable(&runtime_snapshot.imu, now_ms);
  const bool available = AppMotionGate_ComputeAvailable(
    runtime_snapshot.runtime_ready,
    runtime_snapshot.motion_inhibited,
    runtime_snapshot.fault_latched,
    imu_motion_usable);
  const bool updated = AppMotionGate_Update(
    &runtime_snapshot.motion_gate, available);
  *motion_available = runtime_snapshot.motion_gate.available;
  return updated;
}

static void AppTasks_LatchCommEstop(void)
{
  /*
   * ESTOP 绕过普通命令队列和控制节拍，直接锁存共享安全状态并关闭 PWM。挂起调度器期间
   * TIM7 中断仍可运行，但控制任务不能在“写锁存—紧急空转”之间提交新的普通 PWM。
   */
  vTaskSuspendAll();
  runtime_snapshot.motion_inhibited = true;
  runtime_snapshot.fault_latched = true;
  (void)AppMotionGate_Update(&runtime_snapshot.motion_gate, false);
  BspMotor_EmergencyCoastAll();
  (void)xTaskResumeAll();
}

static bool AppTasks_QueueMotorCommand(
  AppCommProtocol *protocol,
  const AppCommCommand *command,
  AppCommRuntimeSnapshot *communication)
{
  if (protocol == NULL || command == NULL || communication == NULL) {
    return false;
  }

  AppMotionGatedRequest request;
  bool motion_available = false;
  bool gate_updated;
  bool sequence_committed = false;
  osStatus_t queue_status = osError;
  AppMotionPrepareResult prepare_result;

  /*
   * 普通命令的实时许可检查、带代际请求入队和协议序号提交在同一调度器挂起区完成。
   * 零等待队列操作不会阻塞；TIM7 ISR 仍可到达，但其他任务不能在检查与提交之间改变
   * motion gate。STATUS/ESTOP 在调用本函数前已经分流。
   */
  vTaskSuspendAll();
  gate_updated = AppTasks_RefreshMotionGate(HAL_GetTick(), &motion_available);
  prepare_result = gate_updated
                     ? AppMotionGate_PrepareCommand(
                         &runtime_snapshot.motion_gate, command, &request)
                     : APP_MOTION_PREPARE_INVALID;
  if (prepare_result == APP_MOTION_PREPARE_READY) {
    queue_status = osMessageQueuePut(
      motor_command_queue, &request, 0U, 0U);
    if (queue_status == osOK) {
      sequence_committed = AppCommProtocol_CommitSequence(
        protocol, command);
    }
  }
  (void)xTaskResumeAll();

  if (!gate_updated || prepare_result == APP_MOTION_PREPARE_INVALID ||
      prepare_result == APP_MOTION_PREPARE_NOT_MOTION_COMMAND ||
      (queue_status == osOK && !sequence_committed)) {
    AppTasks_FailCurrentThread();
  }
  if (prepare_result == APP_MOTION_PREPARE_GATE_REJECTED) {
    communication->motion_gate_reject_count = AppTasks_AddSaturated(
      communication->motion_gate_reject_count, 1U);
    return false;
  }
  if (queue_status != osOK) {
    communication->command_queue_drop_count = AppTasks_AddSaturated(
      communication->command_queue_drop_count, 1U);
    return false;
  }
  return true;
}

static void AppTasks_FillTelemetryInput(
  const AppRuntimeSnapshot *snapshot,
  uint32_t now_ms,
  AppTelemetryInput *input)
{
  /*
   * 只从已经一致复制的运行快照投影遥测字段，格式化器不接触共享全局。UART 错误和发送
   * 故障在这里做饱和汇总，同时保留环形缓冲溢出与命令队列丢弃等独立字段。
   */
  memset(input, 0, sizeof(*input));
  input->now_ms = now_ms;
  memcpy(input->encoder_raw, snapshot->encoder_raw, sizeof(input->encoder_raw));
  memcpy(input->encoder_delta, snapshot->encoder_delta, sizeof(input->encoder_delta));
  memcpy(input->encoder_total, snapshot->encoder_total, sizeof(input->encoder_total));
  input->control_timing = snapshot->control_timing;
  input->motor = snapshot->motor_open_loop;
  input->battery_millivolts = snapshot->battery_millivolts;
  input->imu_sample_age_ms = snapshot->imu.sample_age_ms;
  input->imu_health = snapshot->imu.health;
  input->imu = snapshot->imu;
  input->uart = snapshot->communication.debug_uart;
  input->uart_error_count = AppTasks_SumUartErrors(
    &snapshot->communication.debug_uart);
  input->uart_rx_overflow_count =
    snapshot->communication.debug_uart.rx_buffer_overflow_count;
  input->uart_tx_fault_count = AppTasks_SumUartTxFaults(
    &snapshot->communication.debug_uart);
  input->command_reject_count = AppCommProtocol_GetRejectedCount(
    &snapshot->communication.protocol);
  input->command_queue_drop_count =
    snapshot->communication.command_queue_drop_count;
  input->motion_gate_reject_count =
    snapshot->communication.motion_gate_reject_count;
  input->invalidated_motor_command_count =
    snapshot->invalidated_motor_command_count;
  input->adc_error_count = snapshot->communication.adc_error_count;
  input->telemetry_enqueued_count =
    snapshot->communication.telemetry_enqueued_count;
  input->telemetry_enqueue_drop_count =
    snapshot->communication.telemetry_enqueue_drop_count;
  input->telemetry_format_error_count =
    snapshot->communication.telemetry_format_error_count;
  memcpy(
    input->telemetry_frame_failure_count,
    snapshot->communication.telemetry_frame_failure_count,
    sizeof(input->telemetry_frame_failure_count));
  input->capture_event_drop_count =
    snapshot->communication.capture_event_drop_count;
  input->capture_export_error_count =
    snapshot->communication.capture_export_error_count;
  input->health_miss_count = snapshot->health_miss_count;
  memcpy(
    input->stack_free_bytes,
    snapshot->stack_free_bytes,
    sizeof(input->stack_free_bytes));
  input->minimum_free_heap_bytes = snapshot->minimum_free_heap_bytes;
  input->critical_tasks_alive = snapshot->critical_tasks_alive;
  input->runtime_ready = snapshot->runtime_ready;
  input->motion_inhibited = snapshot->motion_inhibited;
  input->fault_latched = snapshot->fault_latched;
}

static bool AppTasks_IsMotorStoppedForCapture(
  const AppRuntimeSnapshot *snapshot)
{
  return snapshot != NULL &&
         snapshot->motor_open_loop.state == APP_MOTOR_OPEN_LOOP_DISARMED &&
         snapshot->motor_open_loop.target_pwm == 0 &&
         snapshot->motor_open_loop.applied_pwm == 0 &&
         !snapshot->fault_latched;
}

static BspStatus AppTasks_CommitControlMotorOutput(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  const AppMotionGatedRequest *request,
  bool has_queued_request)
{
  if (controller == NULL || request == NULL) {
    return BSP_INVALID_ARG;
  }

  AppMotorOpenLoopRequest effective_request = request->request;
  AppMotorOpenLoopOutput output;
  AppMotorOpenLoopSnapshot snapshot;
  BspStatus status = BSP_OK;

  /*
   * 任务级执行器仲裁区：不中断 TIM7 ISR，只阻止控制任务与安全任务在“读取最终安全门
   * →推进状态机→提交 PWM→发布电机快照”之间互相切换。这样安全任务不可能在控制任务
   * 读取旧的 motion_inhibited 后、真正写 PWM 前插入。区域内不得加入阻塞调用或格式化。
   */
  vTaskSuspendAll();
  bool motion_available = false;
  const bool gate_updated = AppTasks_RefreshMotionGate(
    now_ms, &motion_available);
  const bool fault_latched = runtime_snapshot.fault_latched;
  if (!gate_updated) {
    status = BSP_ERROR;
  } else {
    if (has_queued_request &&
        !AppMotionGate_IsRequestCurrent(
          &runtime_snapshot.motion_gate, request)) {
      effective_request.type = APP_MOTOR_REQUEST_NONE;
      effective_request.sequence = 0U;
      effective_request.pwm = 0;
      runtime_snapshot.invalidated_motor_command_count =
        AppTasks_AddSaturated(
          runtime_snapshot.invalidated_motor_command_count, 1U);
    }
  }
  if (status == BSP_OK &&
      (!AppMotorOpenLoop_Step(
        controller,
        now_ms,
        !motion_available,
        fault_latched,
        &effective_request,
        &output) ||
       !AppMotorOpenLoop_GetSnapshot(controller, &snapshot))) {
    status = BSP_ERROR;
  } else if (status == BSP_OK) {
    status = AppTasks_ApplyMotorOutput(&output);
    if (status == BSP_OK) {
      runtime_snapshot.motor_open_loop = snapshot;
    }
  }
  (void)xTaskResumeAll();
  return status;
}

static BspStatus AppTasks_ApplyMotorOutput(const AppMotorOpenLoopOutput *output)
{
  if (output == NULL) {
    return BSP_INVALID_ARG;
  }
  if (output->emergency_coast_request) {
    BspMotor_EmergencyCoastAll();
    return BSP_OK;
  }
  if (output->mode == APP_MOTOR_OUTPUT_NONE) {
    return BSP_OK;
  }

  /*
   * G1 单电机阶段只有 MA 获得驱动许可；MB、MC、MD 每个非紧急输出周期都重新空转，
   * 防止先前板测残留比较值或未来代码误写让未受控电机动作。紧急路径已一次性关闭全部 PWM。
   */
  BspMotor_Coast(BSP_MOTOR_MB);
  BspMotor_Coast(BSP_MOTOR_MC);
  BspMotor_Coast(BSP_MOTOR_MD);
  switch (output->mode) {
    case APP_MOTOR_OUTPUT_BRAKE:
      BspMotor_Brake(BSP_MOTOR_MA);
      return BSP_OK;
    case APP_MOTOR_OUTPUT_COAST:
      BspMotor_Coast(BSP_MOTOR_MA);
      return BSP_OK;
    case APP_MOTOR_OUTPUT_DRIVE:
      return BspMotor_SetPwm(BSP_MOTOR_MA, output->pwm);
    case APP_MOTOR_OUTPUT_NONE:
    default:
      return BSP_ERROR;
  }
}

BspStatus AppTasks_Create(void)
{
  /*
   * 创建顺序为共享事件→定长命令队列→全部任务→START 闩锁。任务虽然可能创建后立即获得
   * 调度，但首条语句会等待 START，因此不会访问尚未创建完的句柄数组或同步对象。
   */
  if (runtime_events != NULL || motor_command_queue != NULL) {
    return BSP_BUSY;
  }

  memset(task_handles, 0, sizeof(task_handles));
  memset(&runtime_snapshot, 0, sizeof(runtime_snapshot));
  AppMotorCapture_Init(APP_MOTOR_CAPTURE_STORAGE);
  capture_mode = APP_CAPTURE_MODE_MOTOR;
  runtime_snapshot.motion_inhibited = true;
  AppMotionGate_Init(&runtime_snapshot.motion_gate);

  runtime_events = osEventFlagsNew(NULL);
  if (runtime_events == NULL) {
    return BSP_ERROR;
  }

  motor_command_queue = osMessageQueueNew(
    ROBOT_CONFIG_MOTOR_COMMAND_QUEUE_DEPTH,
    sizeof(AppMotionGatedRequest),
    NULL);
  if (motor_command_queue == NULL) {
    AppTasks_StopCreatedThreads();
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

BspStatus AppTasks_TrySetRuntimeReady(void)
{
  if (runtime_events == NULL) {
    return BSP_ERROR;
  }

  BspStatus status = BSP_BUSY;
  bool motion_available = false;
  vTaskSuspendAll();
  const uint32_t now_ms = HAL_GetTick();
  if (runtime_snapshot.fault_latched) {
    status = BSP_ERROR;
  } else if (runtime_snapshot.runtime_ready) {
    status = BSP_OK;
  } else if (runtime_snapshot.critical_tasks_alive &&
             AppImu_IsMotionUsable(
               &runtime_snapshot.imu, now_ms)) {
    /*
     * runtime_ready 是本次上电单向锁，只证明启动门曾完整通过；动态 IMU 或心跳故障仍由
     * motion_inhibited 和 motion gate 在后续每个任务周期持续禁止运动。
     */
    runtime_snapshot.runtime_ready = true;
    status = BSP_OK;
  }

  if (!AppTasks_RefreshMotionGate(now_ms, &motion_available)) {
    runtime_snapshot.motion_inhibited = true;
    runtime_snapshot.fault_latched = true;
    (void)AppMotionGate_Update(&runtime_snapshot.motion_gate, false);
    status = BSP_ERROR;
  }
  (void)xTaskResumeAll();
  return status;
}

static void AppTasks_Control(void *argument)
{
  (void)argument;
  if (!AppTasks_WaitForStart()) {
    AppTasks_FailCurrentThread();
  }

  const uint32_t motor_started_at_ms = HAL_GetTick();
  /*
   * 启动时先用 NONE 请求提交一次安全默认输出，再读取编码器基线和启动时序。这样首个
   * TIM7 周期不会把上电计数当作增量，也不会在状态机尚未初始化时沿用硬件残留输出。
   */
  AppMotorOpenLoop motor_open_loop;
  AppMotorOpenLoop_Init(&motor_open_loop, motor_started_at_ms);
  const AppMotionGatedRequest no_motor_request = {
    .request = {
      .type = APP_MOTOR_REQUEST_NONE,
      .sequence = 0U,
      .pwm = 0,
    },
    .gate_generation = 0U,
  };
  if (AppTasks_CommitControlMotorOutput(
        &motor_open_loop,
        motor_started_at_ms,
        &no_motor_request,
        false) != BSP_OK) {
    AppTasks_FailCurrentThread();
  }

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

    /*
     * timing_snapshot 只能在 RecordComplete 后代表完整周期，因此延迟到下一次唤醒发布。
     * 发布与心跳开销发生在新周期内，自然计入新周期执行时间，不会人为漏掉诊断成本。
     */
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
    taskEXIT_CRITICAL();

    /*
     * 消息队列只做一次零等待 Get，保证每个确定性节拍最多推进一条命令。发送端洪泛只会
     * 填满有限队列并留下 drop 计数，不会把控制周期变成无界 while 循环。
     */
    AppMotionGatedRequest motor_request = no_motor_request;
    const osStatus_t command_status = osMessageQueueGet(
      motor_command_queue, &motor_request, NULL, 0U);
    if (command_status != osOK && command_status != osErrorResource) {
      AppTasks_FailCurrentThread();
    }
    if (AppTasks_CommitControlMotorOutput(
          &motor_open_loop,
          HAL_GetTick(),
          &motor_request,
          command_status == osOK) != BSP_OK) {
      AppTasks_FailCurrentThread();
    }

    /*
     * 高速记录发生在电机输出已经提交之后、RecordComplete 之前，因此样本中的目标/实际
     * PWM 与本周期硬件动作一致，记录开销也会计入控制任务 WCET。通信任务只在短临界区
     * 开始、停止或复制已完成样本，不能与这里的单生产者写入交叉。
     */
    AppMotorCaptureInput capture_input = {
      .tick_sequence = tick.tick_sequence,
      .irq_timestamp_cycles = tick.irq_timestamp_cycles,
      .wake_latency_cycles =
        control_timing.snapshot.wake_latency_cycles,
      .previous_wcet_cycles =
        control_timing.snapshot.wcet_cycles,
      .encoder_raw_ma = tick.encoder_raw[BSP_MOTOR_MA],
      .encoder_delta_ma = encoder_delta[BSP_MOTOR_MA],
    };
    taskENTER_CRITICAL();
    capture_input.target_pwm = runtime_snapshot.motor_open_loop.target_pwm;
    capture_input.applied_pwm = runtime_snapshot.motor_open_loop.applied_pwm;
    capture_input.battery_millivolts = runtime_snapshot.battery_millivolts;
    capture_input.motor_state =
      (uint8_t)runtime_snapshot.motor_open_loop.state;
    if (runtime_snapshot.runtime_ready) {
      capture_input.safety_flags |= APP_MOTOR_CAPTURE_FLAG_RUNTIME_READY;
    }
    if (runtime_snapshot.motion_inhibited) {
      capture_input.safety_flags |= APP_MOTOR_CAPTURE_FLAG_MOTION_INHIBITED;
    }
    if (runtime_snapshot.fault_latched) {
      capture_input.safety_flags |= APP_MOTOR_CAPTURE_FLAG_FAULT_LATCHED;
    }
    if (runtime_snapshot.motion_gate.available) {
      capture_input.safety_flags |= APP_MOTOR_CAPTURE_FLAG_MOTION_AVAILABLE;
    }
    if (runtime_snapshot.critical_tasks_alive) {
      capture_input.safety_flags |=
        APP_MOTOR_CAPTURE_FLAG_CRITICAL_TASKS_ALIVE;
    }
    taskEXIT_CRITICAL();
    if (capture_mode == APP_CAPTURE_MODE_MOTOR) {
      if (!AppMotorCapture_Record(
            APP_MOTOR_CAPTURE_STORAGE, &capture_input)) {
        AppTasks_FailCurrentThread();
      }
    } else {
      const bool speed_was_recording =
        capture_storage.speed.state == APP_SPEED_CAPTURE_RECORDING;
      AppSpeedCaptureInput speed_input = {
        .tick_sequence = tick.tick_sequence,
        .irq_timestamp_cycles = tick.irq_timestamp_cycles,
        .period_sum_cycles = tick.encoder_period_ma.period_sum_cycles,
        .last_edge_age_cycles =
          tick.encoder_period_ma.last_edge_age_cycles,
        .event_sequence = tick.encoder_period_ma.event_sequence,
        .encoder_delta_ma = encoder_delta[BSP_MOTOR_MA],
        .applied_pwm = capture_input.applied_pwm,
        .period_count = tick.encoder_period_ma.period_count,
        .direction = tick.encoder_period_ma.direction,
        .period_flags = tick.encoder_period_ma.flags,
      };
      if (!AppSpeedCapture_Record(
            APP_SPEED_CAPTURE_STORAGE, &speed_input)) {
        AppTasks_FailCurrentThread();
      }
      AppSpeedCaptureStatus speed_status;
      if (!AppSpeedCapture_GetStatus(
            APP_SPEED_CAPTURE_STORAGE, &speed_status)) {
        AppTasks_FailCurrentThread();
      }
      if (speed_was_recording &&
          speed_status.state == APP_SPEED_CAPTURE_COMPLETE) {
        AppEncoderPeriodStats period_stats;
        if (AppControlTimebase_StopMaPeriodCapture() != BSP_OK ||
            !AppControlTimebase_GetMaPeriodStats(&period_stats) ||
            !AppSpeedCapture_SetPeriodStats(
              APP_SPEED_CAPTURE_STORAGE, &period_stats)) {
          AppTasks_FailCurrentThread();
        }
      }
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
  AppSafetyPolicy safety_policy;
  AppSafetyPolicy_Init(&safety_policy);
  vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONFIG_SAFETY_PERIOD_MS));
  /* 首次先等待完整检查窗，避免任务刚启动时因各任务尚未来得及置心跳而产生虚假缺失。 */

  for (;;) {
    /*
     * osEventFlagsClear 的返回值就是清除前快照，一次原子操作完成“取本窗心跳并开下一窗”。
     * 若先 Get 再 Clear，两者之间到达的新心跳会被清除并错误计入下一窗缺失。
     */
    const uint32_t flags = osEventFlagsClear(
      runtime_events, APP_EVENT_CRITICAL_HEARTBEATS);
    const bool tasks_healthy = (flags & osFlagsError) == 0U &&
                               (flags & APP_EVENT_CRITICAL_HEARTBEATS) == APP_EVENT_CRITICAL_HEARTBEATS;

    AppSafetyPolicyOutput safety_output;
    bool policy_updated;
    bool gate_updated = false;
    vTaskSuspendAll();
    /*
     * 在同一调度仲裁区读取 IMU/外部锁存、更新策略、发布安全状态并执行空转。控制任务无法
     * 在状态发布和硬件覆盖之间插入，但中断仍保持运行，时基可记录安全处理带来的任务延迟。
     */
    const uint32_t now_ms = HAL_GetTick();
    const bool imu_motion_usable =
      AppImu_IsMotionUsable(&runtime_snapshot.imu, now_ms);
    policy_updated = AppSafetyPolicy_Update(
      &safety_policy,
      runtime_snapshot.runtime_ready,
      tasks_healthy,
      imu_motion_usable,
      runtime_snapshot.fault_latched,
      ROBOT_CONFIG_HEALTH_MISS_LIMIT,
      &safety_output);
    if (policy_updated) {
      runtime_snapshot.critical_tasks_alive = tasks_healthy;
      runtime_snapshot.health_miss_count = safety_policy.critical_heartbeat_miss_count;
      runtime_snapshot.motion_inhibited = safety_policy.motion_inhibited;
      runtime_snapshot.fault_latched = safety_policy.fault_latched;
      bool motion_available = false;
      gate_updated = AppTasks_RefreshMotionGate(
        now_ms, &motion_available);

      if (gate_updated &&
          safety_output.emergency_coast_request) {
        BspMotor_EmergencyCoastAll();
      } else if (gate_updated &&
                 safety_output.normal_coast_request) {
        BspMotor_CoastAll();
      }
    }
    (void)xTaskResumeAll();
    if (!policy_updated || !gate_updated) {
      AppTasks_FailCurrentThread();
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

  AppCommProtocol protocol;
  AppCommProtocol_Init(&protocol);
  const BspUartPort debug_uart_port = ROBOT_CONFIG_DEBUG_UART_PORT;
  AppCommRuntimeSnapshot communication;
  memset(&communication, 0, sizeof(communication));
  AppTelemetrySchedule telemetry_schedule;
  AppTelemetrySchedule_Init(&telemetry_schedule, HAL_GetTick());
  bool force_telemetry = false;
  static AppRuntimeSnapshot telemetry_snapshot;
  static AppTelemetryInput telemetry_input;
  static uint8_t telemetry_frame[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
  AppCaptureEventQueue capture_events;
  memset(&capture_events, 0, sizeof(capture_events));
  AppCaptureExportState capture_export_state = APP_CAPTURE_EXPORT_IDLE;
  AppCaptureMode capture_export_mode = APP_CAPTURE_MODE_MOTOR;
  uint32_t capture_export_index = 0U;

  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    /* 每 2 ms 先服务 UART 恢复与发送，再一次性清空当前已接收字节，解析器本身不阻塞。 */
    BspUart_Service();

    uint8_t byte;
    while (BspUart_ReadByte(debug_uart_port, &byte)) {
      AppCommCommand command;
      const AppCommFeedResult result = AppCommProtocol_FeedByte(
        &protocol, byte, &command);
      if (result != APP_COMM_FEED_COMMAND) {
        continue;
      }
      if (command.type == APP_COMM_COMMAND_ESTOP) {
        /*
         * ESTOP 不带序号且具有最高优先级，绕过可能已满的普通队列立即锁存；STATUS 只要求
         * 尽快发一帧快照。其余运动命令必须经过队列与两阶段序号提交。
         */
        communication.estop_command_count = AppTasks_AddSaturated(
          communication.estop_command_count, 1U);
        AppTasks_LatchCommEstop();
        force_telemetry = true;
      } else if (command.type == APP_COMM_COMMAND_STATUS) {
        force_telemetry = true;
      } else if (command.type >= APP_COMM_COMMAND_CAPTURE_START &&
                 command.type <= APP_COMM_COMMAND_CAPTURE_STATUS) {
        AppRuntimeSnapshot command_snapshot;
        AppMotorCaptureStatus capture_status;
        bool command_accepted = false;
        taskENTER_CRITICAL();
        command_snapshot = runtime_snapshot;
        if (command.type == APP_COMM_COMMAND_CAPTURE_START) {
          if (capture_export_state == APP_CAPTURE_EXPORT_IDLE &&
              AppTasks_IsMotorStoppedForCapture(&command_snapshot) &&
              (capture_mode != APP_CAPTURE_MODE_SPEED ||
               capture_storage.speed.state != APP_SPEED_CAPTURE_RECORDING)) {
            if (capture_mode != APP_CAPTURE_MODE_MOTOR) {
              AppMotorCapture_Init(APP_MOTOR_CAPTURE_STORAGE);
              capture_mode = APP_CAPTURE_MODE_MOTOR;
            }
            command_accepted =
              AppMotorCapture_Start(APP_MOTOR_CAPTURE_STORAGE);
          }
        } else if (command.type == APP_COMM_COMMAND_CAPTURE_STOP) {
          command_accepted =
            capture_export_state == APP_CAPTURE_EXPORT_IDLE &&
            capture_mode == APP_CAPTURE_MODE_MOTOR &&
            AppMotorCapture_Stop(APP_MOTOR_CAPTURE_STORAGE);
        } else if (command.type == APP_COMM_COMMAND_CAPTURE_EXPORT) {
          command_accepted =
            capture_export_state == APP_CAPTURE_EXPORT_IDLE &&
            capture_mode == APP_CAPTURE_MODE_MOTOR &&
            AppTasks_IsMotorStoppedForCapture(&command_snapshot) &&
            AppMotorCapture_GetStatus(
              APP_MOTOR_CAPTURE_STORAGE, &capture_status) &&
            capture_status.state == APP_MOTOR_CAPTURE_COMPLETE &&
            capture_status.sample_count > 0U;
          if (command_accepted) {
            capture_export_state = APP_CAPTURE_EXPORT_BEGIN;
            capture_export_mode = APP_CAPTURE_MODE_MOTOR;
            capture_export_index = 0U;
          }
        } else {
          command_accepted = capture_mode == APP_CAPTURE_MODE_MOTOR;
        }
        if (capture_mode == APP_CAPTURE_MODE_MOTOR) {
          if (!AppMotorCapture_GetStatus(
                APP_MOTOR_CAPTURE_STORAGE, &capture_status)) {
            taskEXIT_CRITICAL();
            AppTasks_FailCurrentThread();
          }
        } else {
          memset(&capture_status, 0, sizeof(capture_status));
          capture_status.state = APP_MOTOR_CAPTURE_IDLE;
          capture_status.capacity = ROBOT_CONFIG_MOTOR_CAPTURE_CAPACITY;
        }
        taskEXIT_CRITICAL();

        if (command.type != APP_COMM_COMMAND_CAPTURE_EXPORT ||
            !command_accepted) {
          AppMotorCaptureEvent event = APP_MOTOR_CAPTURE_EVENT_STATUS;
          if (!command_accepted) {
            event = APP_MOTOR_CAPTURE_EVENT_REJECTED;
            communication.capture_command_reject_count =
              AppTasks_AddSaturated(
                communication.capture_command_reject_count, 1U);
          } else if (command.type == APP_COMM_COMMAND_CAPTURE_START) {
            event = APP_MOTOR_CAPTURE_EVENT_STARTED;
          } else if (command.type == APP_COMM_COMMAND_CAPTURE_STOP) {
            event = APP_MOTOR_CAPTURE_EVENT_STOPPED;
          }
          if (!AppTasks_MotorCaptureEventPush(
                &capture_events, event, &capture_status)) {
            communication.capture_event_drop_count = AppTasks_AddSaturated(
              communication.capture_event_drop_count, 1U);
          }
        }
      } else if (command.type >= APP_COMM_COMMAND_SPEED_CAPTURE_START &&
                 command.type <= APP_COMM_COMMAND_SPEED_CAPTURE_STATUS) {
        AppRuntimeSnapshot command_snapshot;
        AppSpeedCaptureStatus capture_status;
        bool command_accepted = false;
        taskENTER_CRITICAL();
        command_snapshot = runtime_snapshot;
        taskEXIT_CRITICAL();

        if (command.type == APP_COMM_COMMAND_SPEED_CAPTURE_START) {
          taskENTER_CRITICAL();
          const bool can_start =
            capture_export_state == APP_CAPTURE_EXPORT_IDLE &&
            AppTasks_IsMotorStoppedForCapture(&command_snapshot) &&
            ((capture_mode == APP_CAPTURE_MODE_MOTOR &&
              capture_storage.motor.state != APP_MOTOR_CAPTURE_RECORDING) ||
             (capture_mode == APP_CAPTURE_MODE_SPEED &&
              capture_storage.speed.state != APP_SPEED_CAPTURE_RECORDING));
          if (can_start &&
              AppControlTimebase_StartMaPeriodCapture() == BSP_OK) {
            AppSpeedCapture_Init(APP_SPEED_CAPTURE_STORAGE);
            command_accepted =
              AppSpeedCapture_Start(APP_SPEED_CAPTURE_STORAGE);
            if (command_accepted) {
              capture_mode = APP_CAPTURE_MODE_SPEED;
            }
            if (!command_accepted) {
              (void)AppControlTimebase_StopMaPeriodCapture();
            }
          }
          taskEXIT_CRITICAL();
        } else if (command.type == APP_COMM_COMMAND_SPEED_CAPTURE_STOP) {
          taskENTER_CRITICAL();
          const bool can_stop =
            capture_export_state == APP_CAPTURE_EXPORT_IDLE &&
            capture_mode == APP_CAPTURE_MODE_SPEED &&
            capture_storage.speed.state == APP_SPEED_CAPTURE_RECORDING;
          if (can_stop &&
              AppControlTimebase_StopMaPeriodCapture() == BSP_OK) {
            AppEncoderPeriodStats period_stats;
            if (!AppControlTimebase_GetMaPeriodStats(&period_stats)) {
              AppTasks_FailCurrentThread();
            }
            command_accepted =
              AppSpeedCapture_Stop(APP_SPEED_CAPTURE_STORAGE) &&
              AppSpeedCapture_SetPeriodStats(
                APP_SPEED_CAPTURE_STORAGE, &period_stats);
          }
          taskEXIT_CRITICAL();
        } else {
          AppEncoderPeriodStats period_stats;
          if (!AppControlTimebase_GetMaPeriodStats(&period_stats)) {
            AppTasks_FailCurrentThread();
          }
          taskENTER_CRITICAL();
          if (capture_mode == APP_CAPTURE_MODE_SPEED) {
            (void)AppSpeedCapture_SetPeriodStats(
              APP_SPEED_CAPTURE_STORAGE, &period_stats);
          }
          if (command.type == APP_COMM_COMMAND_SPEED_CAPTURE_EXPORT) {
            command_accepted =
              capture_export_state == APP_CAPTURE_EXPORT_IDLE &&
              capture_mode == APP_CAPTURE_MODE_SPEED &&
              AppTasks_IsMotorStoppedForCapture(&command_snapshot) &&
              AppSpeedCapture_GetStatus(
                APP_SPEED_CAPTURE_STORAGE, &capture_status) &&
              capture_status.state == APP_SPEED_CAPTURE_COMPLETE &&
              capture_status.sample_count > 0U;
            if (command_accepted) {
              capture_export_state = APP_CAPTURE_EXPORT_BEGIN;
              capture_export_mode = APP_CAPTURE_MODE_SPEED;
              capture_export_index = 0U;
            }
          } else {
            command_accepted = capture_mode == APP_CAPTURE_MODE_SPEED;
          }
          taskEXIT_CRITICAL();
        }

        taskENTER_CRITICAL();
        if (capture_mode == APP_CAPTURE_MODE_SPEED) {
          if (!AppSpeedCapture_GetStatus(
                APP_SPEED_CAPTURE_STORAGE, &capture_status)) {
            taskEXIT_CRITICAL();
            AppTasks_FailCurrentThread();
          }
        } else {
          memset(&capture_status, 0, sizeof(capture_status));
          capture_status.state = APP_SPEED_CAPTURE_IDLE;
          capture_status.capacity = ROBOT_CONFIG_SPEED_CAPTURE_CAPACITY;
        }
        taskEXIT_CRITICAL();

        if (command.type != APP_COMM_COMMAND_SPEED_CAPTURE_EXPORT ||
            !command_accepted) {
          AppSpeedCaptureEvent event = APP_SPEED_CAPTURE_EVENT_STATUS;
          if (!command_accepted) {
            event = APP_SPEED_CAPTURE_EVENT_REJECTED;
            communication.capture_command_reject_count =
              AppTasks_AddSaturated(
                communication.capture_command_reject_count, 1U);
          } else if (
            command.type == APP_COMM_COMMAND_SPEED_CAPTURE_START) {
            event = APP_SPEED_CAPTURE_EVENT_STARTED;
          } else if (
            command.type == APP_COMM_COMMAND_SPEED_CAPTURE_STOP) {
            event = APP_SPEED_CAPTURE_EVENT_STOPPED;
          }
          if (!AppTasks_SpeedCaptureEventPush(
                &capture_events, event, &capture_status)) {
            communication.capture_event_drop_count = AppTasks_AddSaturated(
              communication.capture_event_drop_count, 1U);
          }
        }
      } else {
        /*
         * EXPORT 开始时已经确认 MA 完全停机；导出结束前继续拒绝普通运动命令，避免异步
         * 批量发送期间电机重新动作。协议序号不提交，发送端可在 END 后使用原序号重试；
         * ESTOP 仍在更早分支始终可用。
         */
        if (capture_export_state != APP_CAPTURE_EXPORT_IDLE) {
          communication.capture_command_reject_count =
            AppTasks_AddSaturated(
              communication.capture_command_reject_count, 1U);
          AppMotorCaptureStatus capture_status;
          taskENTER_CRITICAL();
          const bool status_valid =
            capture_mode == APP_CAPTURE_MODE_MOTOR &&
            AppMotorCapture_GetStatus(
              APP_MOTOR_CAPTURE_STORAGE, &capture_status);
          taskEXIT_CRITICAL();
          if (status_valid) {
            if (!AppTasks_MotorCaptureEventPush(
                  &capture_events,
                  APP_MOTOR_CAPTURE_EVENT_REJECTED,
                  &capture_status)) {
              communication.capture_event_drop_count = AppTasks_AddSaturated(
                communication.capture_event_drop_count, 1U);
            }
          } else {
            AppSpeedCaptureStatus speed_status;
            taskENTER_CRITICAL();
            const bool speed_status_valid =
              capture_mode == APP_CAPTURE_MODE_SPEED &&
              AppSpeedCapture_GetStatus(
                APP_SPEED_CAPTURE_STORAGE, &speed_status);
            taskEXIT_CRITICAL();
            if (!speed_status_valid) {
              AppTasks_FailCurrentThread();
            }
            if (!AppTasks_SpeedCaptureEventPush(
                  &capture_events,
                  APP_SPEED_CAPTURE_EVENT_REJECTED,
                  &speed_status)) {
              communication.capture_event_drop_count = AppTasks_AddSaturated(
                communication.capture_event_drop_count, 1U);
            }
          }
        } else {
          (void)AppTasks_QueueMotorCommand(
            &protocol, &command, &communication);
        }
      }
    }

    const uint32_t now_ms = HAL_GetTick();
    const bool periodic_stat_due =
      (now_ms - telemetry_schedule.last_stat_ms) >=
        ROBOT_CONFIG_TELEMETRY_PERIOD_MS;
    AppTelemetryFrameType telemetry_frame_type;
    bool telemetry_due = AppTelemetrySchedule_Select(
      &telemetry_schedule,
      now_ms,
      force_telemetry,
      &telemetry_frame_type);
    if (telemetry_due &&
        telemetry_frame_type == APP_TELEMETRY_FRAME_STAT &&
        periodic_stat_due) {
      /*
       * ADC 是轮询式独占资源，只绑定周期遥测节拍以限制执行时间与采样负载。STATUS 强制
       * 遥测不会额外触发 ADC，避免命令洪泛把通信任务变成连续阻塞采样。
       */
      uint16_t battery_millivolts = 0U;
      const BspStatus battery_status = BspAdc_ReadBatteryMillivolts(
        &battery_millivolts);
      if (battery_status != BSP_OK) {
        communication.adc_error_count = AppTasks_AddSaturated(
          communication.adc_error_count, 1U);
      }
      if (battery_status == BSP_OK) {
        taskENTER_CRITICAL();
        runtime_snapshot.battery_millivolts = battery_millivolts;
        taskEXIT_CRITICAL();
      }
    }

    if (telemetry_due) {
      /*
       * 先在短临界区复制完整运行快照，再在临界区外构造和格式化帧。发送接口复制帧内容，
       * 因此静态 telemetry_frame 在成功入队后可由下一周期安全复用。更新 EVENT 后重新
       * 选择一次，保证本循环新发现的状态跃迁优先于普通周期帧；每个循环仍最多入队一帧。
       */
      (void)AppCommProtocol_GetStats(&protocol, &communication.protocol);
      (void)BspUart_GetStats(debug_uart_port, &communication.debug_uart);

      taskENTER_CRITICAL();
      runtime_snapshot.communication = communication;
      telemetry_snapshot = runtime_snapshot;
      taskEXIT_CRITICAL();

      AppTasks_FillTelemetryInput(
        &telemetry_snapshot, now_ms, &telemetry_input);
      AppTelemetrySchedule_UpdateEvents(
        &telemetry_schedule, &telemetry_input);
      telemetry_due = AppTelemetrySchedule_Select(
        &telemetry_schedule,
        now_ms,
        force_telemetry,
        &telemetry_frame_type);
      if (!telemetry_due) {
        AppTasks_FailCurrentThread();
      }
      uint16_t telemetry_length = 0U;
      bool telemetry_enqueued = false;
      if (!AppTelemetry_FormatTyped(
            telemetry_frame_type,
            &telemetry_schedule,
            &telemetry_input,
            telemetry_frame,
            (uint16_t)sizeof(telemetry_frame),
            &telemetry_length)) {
        communication.telemetry_format_error_count = AppTasks_AddSaturated(
          communication.telemetry_format_error_count, 1U);
        communication.telemetry_frame_failure_count[telemetry_frame_type] =
          AppTasks_AddSaturated(
            communication.telemetry_frame_failure_count[telemetry_frame_type],
            1U);
      } else if (BspUart_WriteAsync(
                   debug_uart_port, telemetry_frame, telemetry_length) != BSP_OK) {
        communication.telemetry_enqueue_drop_count = AppTasks_AddSaturated(
          communication.telemetry_enqueue_drop_count, 1U);
        communication.telemetry_frame_failure_count[telemetry_frame_type] =
          AppTasks_AddSaturated(
            communication.telemetry_frame_failure_count[telemetry_frame_type],
            1U);
      } else {
        communication.telemetry_enqueued_count = AppTasks_AddSaturated(
          communication.telemetry_enqueued_count, 1U);
        telemetry_enqueued = true;
      }
      AppTelemetrySchedule_MarkAttempt(
        &telemetry_schedule,
        telemetry_frame_type,
        now_ms,
        telemetry_enqueued);
      if (telemetry_frame_type == APP_TELEMETRY_FRAME_STAT) {
        force_telemetry = false;
      }
    }

    /*
     * 每个通信周期最多追加一帧高速采集数据。高速导出必须为下一次分类型诊断遥测
     * 预留一个槽位，否则导出可在两个通信周期之间填满队列，使随后到期的遥测调用
     * WriteAsync 并增加 queue_full。队列达到导出上限时保持当前索引稍后重试；事件优先
     * 于批量样本，BEGIN/END 和样本都使用同一异步发送链，期间仍解析 ESTOP 与普通命令。
     */
    BspUartStats capture_uart_stats;
    if (BspUart_GetStats(debug_uart_port, &capture_uart_stats) != BSP_OK) {
      AppTasks_FailCurrentThread();
    }
    if (capture_uart_stats.tx_queued_frame_count <
        (ROBOT_CONFIG_UART_TX_QUEUE_DEPTH -
         ROBOT_CONFIG_UART_TX_CAPTURE_RESERVED_SLOTS)) {
      uint16_t capture_frame_length = 0U;
      bool capture_frame_ready = false;
      bool capture_frame_is_event = false;
      AppCapturePendingEvent pending_event;

      if (AppTasks_CaptureEventPeek(&capture_events, &pending_event)) {
        if (pending_event.mode == APP_CAPTURE_MODE_MOTOR) {
          capture_frame_ready = AppMotorCapture_FormatEvent(
            pending_event.event.motor,
            &pending_event.status.motor,
            telemetry_frame,
            (uint16_t)sizeof(telemetry_frame),
            &capture_frame_length);
        } else {
          capture_frame_ready = AppSpeedCapture_FormatEvent(
            pending_event.event.speed,
            &pending_event.status.speed,
            telemetry_frame,
            (uint16_t)sizeof(telemetry_frame),
            &capture_frame_length);
        }
        capture_frame_is_event = true;
      } else if (capture_export_state != APP_CAPTURE_EXPORT_IDLE) {
        if (capture_export_mode == APP_CAPTURE_MODE_MOTOR) {
          AppMotorCaptureStatus capture_status;
          taskENTER_CRITICAL();
          const bool status_valid = AppMotorCapture_GetStatus(
            APP_MOTOR_CAPTURE_STORAGE, &capture_status);
          taskEXIT_CRITICAL();
          if (!status_valid ||
              capture_status.state != APP_MOTOR_CAPTURE_COMPLETE) {
            AppTasks_FailCurrentThread();
          }

          if (capture_export_state == APP_CAPTURE_EXPORT_BEGIN) {
            capture_frame_ready = AppMotorCapture_FormatEvent(
              APP_MOTOR_CAPTURE_EVENT_BEGIN,
              &capture_status,
              telemetry_frame,
              (uint16_t)sizeof(telemetry_frame),
              &capture_frame_length);
          } else if (
            capture_export_state == APP_CAPTURE_EXPORT_SAMPLES &&
            capture_export_index < capture_status.sample_count) {
            AppMotorCaptureSample sample;
            taskENTER_CRITICAL();
            const bool sample_valid = AppMotorCapture_GetSample(
              APP_MOTOR_CAPTURE_STORAGE, capture_export_index, &sample);
            taskEXIT_CRITICAL();
            if (!sample_valid) {
              AppTasks_FailCurrentThread();
            }
            capture_frame_ready = AppMotorCapture_FormatSample(
              capture_export_index,
              &sample,
              telemetry_frame,
              (uint16_t)sizeof(telemetry_frame),
              &capture_frame_length);
          } else {
            capture_export_state = APP_CAPTURE_EXPORT_END;
            capture_frame_ready = AppMotorCapture_FormatEvent(
              APP_MOTOR_CAPTURE_EVENT_END,
              &capture_status,
              telemetry_frame,
              (uint16_t)sizeof(telemetry_frame),
              &capture_frame_length);
          }
        } else {
          AppSpeedCaptureStatus capture_status;
          taskENTER_CRITICAL();
          const bool status_valid = AppSpeedCapture_GetStatus(
            APP_SPEED_CAPTURE_STORAGE, &capture_status);
          taskEXIT_CRITICAL();
          if (!status_valid ||
              capture_status.state != APP_SPEED_CAPTURE_COMPLETE) {
            AppTasks_FailCurrentThread();
          }

          if (capture_export_state == APP_CAPTURE_EXPORT_BEGIN) {
            capture_frame_ready = AppSpeedCapture_FormatEvent(
              APP_SPEED_CAPTURE_EVENT_BEGIN,
              &capture_status,
              telemetry_frame,
              (uint16_t)sizeof(telemetry_frame),
              &capture_frame_length);
          } else if (
            capture_export_state == APP_CAPTURE_EXPORT_SAMPLES &&
            capture_export_index < capture_status.sample_count) {
            AppSpeedCaptureSample sample;
            taskENTER_CRITICAL();
            const bool sample_valid = AppSpeedCapture_GetSample(
              APP_SPEED_CAPTURE_STORAGE, capture_export_index, &sample);
            taskEXIT_CRITICAL();
            if (!sample_valid) {
              AppTasks_FailCurrentThread();
            }
            capture_frame_ready = AppSpeedCapture_FormatSample(
              capture_export_index,
              &sample,
              telemetry_frame,
              (uint16_t)sizeof(telemetry_frame),
              &capture_frame_length);
          } else {
            capture_export_state = APP_CAPTURE_EXPORT_END;
            capture_frame_ready = AppSpeedCapture_FormatEvent(
              APP_SPEED_CAPTURE_EVENT_END,
              &capture_status,
              telemetry_frame,
              (uint16_t)sizeof(telemetry_frame),
              &capture_frame_length);
          }
        }
      }

      if (!capture_frame_ready &&
          (capture_frame_is_event ||
           capture_export_state != APP_CAPTURE_EXPORT_IDLE)) {
        AppTasks_FailCurrentThread();
      }
      if (capture_frame_ready) {
        const BspStatus capture_send_status = BspUart_WriteAsync(
          debug_uart_port, telemetry_frame, capture_frame_length);
        if (capture_send_status == BSP_OK) {
          if (capture_frame_is_event) {
            if (!AppTasks_CaptureEventPop(&capture_events)) {
              AppTasks_FailCurrentThread();
            }
          } else if (capture_export_state == APP_CAPTURE_EXPORT_BEGIN) {
            capture_export_state = APP_CAPTURE_EXPORT_SAMPLES;
          } else if (capture_export_state == APP_CAPTURE_EXPORT_SAMPLES) {
            capture_export_index++;
          } else {
            capture_export_state = APP_CAPTURE_EXPORT_IDLE;
            communication.capture_export_count = AppTasks_AddSaturated(
              communication.capture_export_count, 1U);
          }
        } else if (capture_send_status != BSP_BUSY) {
          communication.capture_export_error_count = AppTasks_AddSaturated(
            communication.capture_export_error_count, 1U);
          capture_export_state = APP_CAPTURE_EXPORT_IDLE;
        }
      }
    }

    (void)AppCommProtocol_GetStats(&protocol, &communication.protocol);
    (void)BspUart_GetStats(debug_uart_port, &communication.debug_uart);
    taskENTER_CRITICAL();
    runtime_snapshot.communication = communication;
    taskEXIT_CRITICAL();
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
    /*
     * 数据就绪外部中断通常触发立即处理；两倍任务周期的超时保证即使 EXTI 丢失或处于
     * 非阻塞退避，任务仍周期醒来刷新 sample_age_ms、健康状态与任务心跳。
     */
    (void)osThreadFlagsWait(
      APP_IMU_THREAD_FLAG_DATA_READY, osFlagsWaitAny, pdMS_TO_TICKS(ROBOT_CONFIG_IMU_PERIOD_MS * 2U));
    AppImuOutput output;
    const uint32_t now_ms = HAL_GetTick();
    const BspStatus status = AppImu_Process(now_ms, &output);

    bool motion_available = false;
    vTaskSuspendAll();
    runtime_snapshot.imu = output;
    const bool gate_updated = AppTasks_RefreshMotionGate(
      now_ms, &motion_available);
    (void)xTaskResumeAll();
    if (!gate_updated) {
      AppTasks_FailCurrentThread();
    }

    /*
     * Process 返回值描述本次数据机会，不能代表任务存活。无论 BUSY、退避还是数据故障，
     * 只要循环仍执行就置 IMU_HEARTBEAT；安全任务再独立读取 output.health 决定是否允许运动。
     */
    (void)osEventFlagsSet(runtime_events, APP_EVENT_IMU_HEARTBEAT);
    (void)status;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  /* EXTI 只置线程标志，不在中断中访问 I2C 或运行 ESKF；任务尚未创建时安全忽略早期中断。 */
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
    /*
     * 栈余量是 CMSIS-RTOS 报告的历史最小剩余字节；heap_4 同时提供本次上电最小剩余堆。
     * 两者只建立 RES 可观测性，仍需阶段 5 板测才可验收；LED1 不参与安全判定。
     */
    uint32_t stack_free_bytes[APP_TASK_COUNT];
    for (uint32_t i = 0U; i < APP_TASK_COUNT; ++i) {
      stack_free_bytes[i] = osThreadGetStackSpace(task_handles[i]);
    }

    taskENTER_CRITICAL();
    memcpy(runtime_snapshot.stack_free_bytes, stack_free_bytes, sizeof(stack_free_bytes));
    runtime_snapshot.minimum_free_heap_bytes =
      (uint32_t)xPortGetMinimumEverFreeHeapSize();
    taskEXIT_CRITICAL();

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONFIG_MONITOR_PERIOD_MS));
  }
}
