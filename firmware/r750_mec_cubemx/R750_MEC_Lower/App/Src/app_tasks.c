#include "app_tasks.h"

#include "FreeRTOS.h"
#include "app_comm_protocol.h"
#include "app_control_timebase.h"
#include "app_control_timing.h"
#include "app_encoder_accumulator.h"
#include "app_imu.h"
#include "app_motor_open_loop.h"
#include "app_safety_policy.h"
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

static osEventFlagsId_t runtime_events;
static osMessageQueueId_t motor_command_queue;
static osThreadId_t task_handles[APP_TASK_COUNT];
/*
 * runtime_snapshot 按字段组划分单一写入者，但整结构会被多个读取者复制。短临界区只用于
 * 内存复制或失败路径的安全标志更新，禁止在 taskENTER_CRITICAL 内调用可能阻塞的
 * HAL/RTOS 接口。安全任务的常规策略和通信 ESTOP 则分别在各自调度器挂起区串行化。
 * 执行器仲裁另用 vTaskSuspendAll，允许 TIM7 中断继续到达但阻止控制与安全任务互相切换。
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
  const AppMotorOpenLoopRequest *request);
static void AppTasks_LatchCommEstop(void);
static bool AppTasks_QueueMotorCommand(
  AppCommProtocol *protocol,
  const AppCommCommand *command,
  AppCommRuntimeSnapshot *communication);
static uint32_t AppTasks_AddSaturated(uint32_t value, uint32_t increment);
static uint32_t AppTasks_SumUartErrors(const BspUartStats *stats);
static uint32_t AppTasks_SumUartTxFaults(const BspUartStats *stats);
static void AppTasks_FillTelemetryInput(
  const AppRuntimeSnapshot *snapshot,
  uint32_t now_ms,
  AppTelemetryInput *input);

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

static void AppTasks_LatchCommEstop(void)
{
  /*
   * ESTOP 绕过普通命令队列和控制节拍，直接锁存共享安全状态并关闭 PWM。挂起调度器期间
   * TIM7 中断仍可运行，但控制任务不能在“写锁存—紧急空转”之间提交新的普通 PWM。
   */
  vTaskSuspendAll();
  runtime_snapshot.motion_inhibited = true;
  runtime_snapshot.fault_latched = true;
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

  AppMotorRequestType type;
  switch (command->type) {
    case APP_COMM_COMMAND_ARM:
      type = APP_MOTOR_REQUEST_ARM;
      break;
    case APP_COMM_COMMAND_PWM:
      type = APP_MOTOR_REQUEST_SET_PWM;
      break;
    case APP_COMM_COMMAND_STOP:
      type = APP_MOTOR_REQUEST_STOP;
      break;
    case APP_COMM_COMMAND_NONE:
    case APP_COMM_COMMAND_ESTOP:
    case APP_COMM_COMMAND_STATUS:
    default:
      return false;
  }

  const AppMotorOpenLoopRequest request = {
    .type = type,
    .sequence = command->sequence,
    .pwm = command->pwm,
  };
  if (osMessageQueuePut(motor_command_queue, &request, 0U, 0U) != osOK) {
    communication->command_queue_drop_count = AppTasks_AddSaturated(
      communication->command_queue_drop_count, 1U);
    return false;
  }
  /*
   * 两阶段提交把“命令副本进入队列”和“序号成为防重放基线”组成事务。若 Commit 在入队
   * 成功后仍失败，说明通信解析与队列状态发生内部不一致，必须按系统故障处理而非丢命令。
   */
  if (!AppCommProtocol_CommitSequence(protocol, command)) {
    AppTasks_FailCurrentThread();
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
  input->uart_error_count = AppTasks_SumUartErrors(
    &snapshot->communication.uart_ttl);
  input->uart_rx_overflow_count =
    snapshot->communication.uart_ttl.rx_buffer_overflow_count;
  input->uart_tx_fault_count = AppTasks_SumUartTxFaults(
    &snapshot->communication.uart_ttl);
  input->command_reject_count = AppCommProtocol_GetRejectedCount(
    &snapshot->communication.protocol);
  input->command_queue_drop_count =
    snapshot->communication.command_queue_drop_count;
  input->adc_error_count = snapshot->communication.adc_error_count;
  input->critical_tasks_alive = snapshot->critical_tasks_alive;
  input->motion_inhibited = snapshot->motion_inhibited;
  input->fault_latched = snapshot->fault_latched;
}

static BspStatus AppTasks_CommitControlMotorOutput(
  AppMotorOpenLoop *controller,
  uint32_t now_ms,
  const AppMotorOpenLoopRequest *request)
{
  if (controller == NULL || request == NULL) {
    return BSP_INVALID_ARG;
  }

  AppMotorOpenLoopOutput output;
  AppMotorOpenLoopSnapshot snapshot;
  BspStatus status = BSP_OK;

  /*
   * 任务级执行器仲裁区：不中断 TIM7 ISR，只阻止控制任务与安全任务在“读取最终安全门
   * →推进状态机→提交 PWM→发布电机快照”之间互相切换。这样安全任务不可能在控制任务
   * 读取旧的 motion_inhibited 后、真正写 PWM 前插入。区域内不得加入阻塞调用或格式化。
   */
  vTaskSuspendAll();
  const bool motion_inhibited = runtime_snapshot.motion_inhibited;
  const bool fault_latched = runtime_snapshot.fault_latched;
  if (!AppMotorOpenLoop_Step(
        controller,
        now_ms,
        motion_inhibited,
        fault_latched,
        request,
        &output) ||
      !AppMotorOpenLoop_GetSnapshot(controller, &snapshot)) {
    status = BSP_ERROR;
  } else {
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
  runtime_snapshot.motion_inhibited = true;

  runtime_events = osEventFlagsNew(NULL);
  if (runtime_events == NULL) {
    return BSP_ERROR;
  }

  motor_command_queue = osMessageQueueNew(
    ROBOT_CONFIG_MOTOR_COMMAND_QUEUE_DEPTH,
    sizeof(AppMotorOpenLoopRequest),
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
  const AppMotorOpenLoopRequest no_motor_request = {
    .type = APP_MOTOR_REQUEST_NONE,
    .sequence = 0U,
    .pwm = 0,
  };
  if (AppTasks_CommitControlMotorOutput(
        &motor_open_loop, motor_started_at_ms, &no_motor_request) != BSP_OK) {
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
    AppMotorOpenLoopRequest motor_request = no_motor_request;
    const osStatus_t command_status = osMessageQueueGet(
      motor_command_queue, &motor_request, NULL, 0U);
    if (command_status != osOK && command_status != osErrorResource) {
      AppTasks_FailCurrentThread();
    }
    if (AppTasks_CommitControlMotorOutput(
          &motor_open_loop, HAL_GetTick(), &motor_request) != BSP_OK) {
      AppTasks_FailCurrentThread();
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
    vTaskSuspendAll();
    /*
     * 在同一调度仲裁区读取 IMU/外部锁存、更新策略、发布安全状态并执行空转。控制任务无法
     * 在状态发布和硬件覆盖之间插入，但中断仍保持运行，时基可记录安全处理带来的任务延迟。
     */
    const bool imu_healthy = runtime_snapshot.imu.health == APP_IMU_HEALTH_HEALTHY;
    /*
     * 当前策略只按 health 判定，尚未同时检查 DATA_VALID/DATA_STALE/sample_age；持续无新
     * 样本但无总线错误时可能仍保持 HEALTHY，此处需要后续安全修复，不能视为完整数据门。
     */
    policy_updated = AppSafetyPolicy_Update(
      &safety_policy,
      tasks_healthy,
      imu_healthy,
      runtime_snapshot.fault_latched,
      ROBOT_CONFIG_HEALTH_MISS_LIMIT,
      &safety_output);
    if (policy_updated) {
      runtime_snapshot.critical_tasks_alive = tasks_healthy;
      runtime_snapshot.health_miss_count = safety_policy.critical_heartbeat_miss_count;
      runtime_snapshot.motion_inhibited = safety_policy.motion_inhibited;
      runtime_snapshot.fault_latched = safety_policy.fault_latched;

      if (safety_output.emergency_coast_request) {
        BspMotor_EmergencyCoastAll();
      } else if (safety_output.normal_coast_request) {
        BspMotor_CoastAll();
      }
    }
    (void)xTaskResumeAll();
    if (!policy_updated) {
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
  AppCommRuntimeSnapshot communication;
  memset(&communication, 0, sizeof(communication));
  uint32_t last_telemetry_ms = HAL_GetTick() - ROBOT_CONFIG_TELEMETRY_PERIOD_MS;
  bool force_telemetry = false;
  static AppRuntimeSnapshot telemetry_snapshot;
  static AppTelemetryInput telemetry_input;
  static uint8_t telemetry_frame[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];

  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    /* 每 2 ms 先服务 UART 恢复与发送，再一次性清空当前已接收字节，解析器本身不阻塞。 */
    BspUart_Service();

    uint8_t byte;
    while (BspUart_ReadByte(BSP_UART_TTL, &byte)) {
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
      } else {
        (void)AppTasks_QueueMotorCommand(
          &protocol, &command, &communication);
      }
    }

    const uint32_t now_ms = HAL_GetTick();
    const bool periodic_telemetry_due =
      (now_ms - last_telemetry_ms) >= ROBOT_CONFIG_TELEMETRY_PERIOD_MS;
    if (periodic_telemetry_due) {
      /*
       * ADC 是轮询式独占资源，只绑定周期遥测节拍以限制执行时间与采样负载。STATUS 强制
       * 遥测不会额外触发 ADC，避免命令洪泛把通信任务变成连续阻塞采样。
       */
      last_telemetry_ms = now_ms;
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

    if (periodic_telemetry_due || force_telemetry) {
      /*
       * 先在短临界区复制完整运行快照，再在临界区外构造和格式化帧。发送接口复制帧内容，
       * 因此静态 telemetry_frame 在成功入队后可由下一周期安全复用。
       */
      force_telemetry = false;
      (void)AppCommProtocol_GetStats(&protocol, &communication.protocol);
      (void)BspUart_GetStats(BSP_UART_TTL, &communication.uart_ttl);

      taskENTER_CRITICAL();
      runtime_snapshot.communication = communication;
      telemetry_snapshot = runtime_snapshot;
      taskEXIT_CRITICAL();

      AppTasks_FillTelemetryInput(
        &telemetry_snapshot, now_ms, &telemetry_input);
      uint16_t telemetry_length = 0U;
      if (!AppTelemetry_Format(
            &telemetry_input,
            telemetry_frame,
            (uint16_t)sizeof(telemetry_frame),
            &telemetry_length)) {
        communication.telemetry_format_error_count = AppTasks_AddSaturated(
          communication.telemetry_format_error_count, 1U);
      } else if (BspUart_WriteAsync(
                   BSP_UART_TTL, telemetry_frame, telemetry_length) != BSP_OK) {
        communication.telemetry_enqueue_drop_count = AppTasks_AddSaturated(
          communication.telemetry_enqueue_drop_count, 1U);
      } else {
        communication.telemetry_enqueued_count = AppTasks_AddSaturated(
          communication.telemetry_enqueued_count, 1U);
      }
    }

    (void)AppCommProtocol_GetStats(&protocol, &communication.protocol);
    (void)BspUart_GetStats(BSP_UART_TTL, &communication.uart_ttl);
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
    const BspStatus status = AppImu_Process(HAL_GetTick(), &output);

    taskENTER_CRITICAL();
    runtime_snapshot.imu = output;
    taskEXIT_CRITICAL();

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
     * 栈余量是 CMSIS-RTOS 报告的历史最小剩余字节，用于板测确认静态栈预算；LED1 翻转只
     * 表示低优先级监控任务仍得到调度，不参与关键心跳或安全判定。
     */
    uint32_t stack_free_bytes[APP_TASK_COUNT];
    for (uint32_t i = 0U; i < APP_TASK_COUNT; ++i) {
      stack_free_bytes[i] = osThreadGetStackSpace(task_handles[i]);
    }

    taskENTER_CRITICAL();
    memcpy(runtime_snapshot.stack_free_bytes, stack_free_bytes, sizeof(stack_free_bytes));
    taskEXIT_CRITICAL();

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONFIG_MONITOR_PERIOD_MS));
  }
}
