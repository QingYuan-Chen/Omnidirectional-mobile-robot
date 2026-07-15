#include "app_control_timebase.h"

#include "FreeRTOS.h"
#include "app_control_tick_buffer.h"
#include "app_control_timing.h"
#include "bsp_encoder.h"
#include "robot_config.h"
#include "task.h"
#include "tim.h"

#include <stdbool.h>
#include <stddef.h>

_Static_assert(
  ROBOT_CONFIG_CONTROL_TIMER_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
  "TIM7 interrupt priority may not call FreeRTOS FromISR APIs");

static AppControlTickBuffer tick_buffer;
static TaskHandle_t control_task_handle;
static volatile bool timebase_started;
static volatile uint32_t irq_entry_timestamp_cycles;
static uint32_t previous_irq_timestamp_cycles;
static uint32_t expected_period_cycles;
static uint32_t timer_irq_missed_period_count;
static uint64_t elapsed_cycles_since_start;
static uint64_t serviced_irq_count;

static void AppControlTimebase_ResetState(void)
{
  AppControlTickBuffer_Init(&tick_buffer);
  irq_entry_timestamp_cycles = 0U;
  previous_irq_timestamp_cycles = 0U;
  timer_irq_missed_period_count = 0U;
  elapsed_cycles_since_start = 0U;
  serviced_irq_count = 0U;
}

static bool AppControlTimebase_ConfigurationMatches(void)
{
  /* 启动前同时核对时钟树、TIM7 参数和中断优先级，避免局部改参后静默漂移。 */
  const uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
  const uint32_t actual_timer_clock_hz =
    (RCC->CFGR & RCC_CFGR_PPRE1) == 0U ? pclk1_hz : pclk1_hz * 2U;
  if (SystemCoreClock != ROBOT_CONFIG_CPU_CLOCK_HZ ||
      HAL_RCC_GetHCLKFreq() != ROBOT_CONFIG_CPU_CLOCK_HZ ||
      ROBOT_CONFIG_CONTROL_RATE_HZ == 0U ||
      (SystemCoreClock % ROBOT_CONFIG_CONTROL_RATE_HZ) != 0U ||
      actual_timer_clock_hz != ROBOT_CONFIG_CONTROL_TIMER_CLOCK_HZ ||
      htim7.Instance != TIM7 ||
      htim7.Init.Prescaler != ROBOT_CONFIG_CONTROL_TIMER_PRESCALER ||
      htim7.Init.Period != ROBOT_CONFIG_CONTROL_TIMER_AUTORELOAD ||
      NVIC_GetPriority(TIM7_IRQn) != ROBOT_CONFIG_CONTROL_TIMER_IRQ_PRIORITY) {
    return false;
  }

  const uint32_t timer_divider =
    (ROBOT_CONFIG_CONTROL_TIMER_PRESCALER + 1U) *
    (ROBOT_CONFIG_CONTROL_TIMER_AUTORELOAD + 1U);
  if (timer_divider == 0U ||
      (ROBOT_CONFIG_CONTROL_TIMER_CLOCK_HZ % timer_divider) != 0U ||
      (ROBOT_CONFIG_CONTROL_TIMER_CLOCK_HZ / timer_divider) != ROBOT_CONFIG_CONTROL_RATE_HZ) {
    return false;
  }

  expected_period_cycles = SystemCoreClock / ROBOT_CONFIG_CONTROL_RATE_HZ;
  return expected_period_cycles != 0U;
}

BspStatus AppControlTimebase_Start(void)
{
  if (timebase_started) {
    return BSP_BUSY;
  }
  if (!AppControlTimebase_ConfigurationMatches()) {
    return BSP_ERROR;
  }

  TaskHandle_t const current_task = xTaskGetCurrentTaskHandle();
  if (current_task == NULL) {
    return BSP_ERROR;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
    return BSP_ERROR;
  }

  (void)ulTaskNotifyTake(pdTRUE, 0U);
  __HAL_TIM_SET_COUNTER(&htim7, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);

  /* 先登记接收任务并置启动标志，再开启定时器，关闭首个通知无接收者的窗口。 */
  taskENTER_CRITICAL();
  AppControlTimebase_ResetState();
  previous_irq_timestamp_cycles = DWT->CYCCNT;
  control_task_handle = current_task;
  timebase_started = true;
  const HAL_StatusTypeDef start_status = HAL_TIM_Base_Start_IT(&htim7);
  if (start_status != HAL_OK) {
    timebase_started = false;
    control_task_handle = NULL;
  }
  taskEXIT_CRITICAL();

  if (start_status != HAL_OK) {
    return BSP_ERROR;
  }

  return BSP_OK;
}

BspStatus AppControlTimebase_Stop(void)
{
  taskENTER_CRITICAL();
  const bool was_started = timebase_started;
  timebase_started = false;
  control_task_handle = NULL;
  taskEXIT_CRITICAL();

  if (!was_started) {
    return BSP_OK;
  }
  return HAL_TIM_Base_Stop_IT(&htim7) == HAL_OK ? BSP_OK : BSP_ERROR;
}

BspStatus AppControlTimebase_Wait(AppControlTick *tick)
{
  if (tick == NULL || !timebase_started || xTaskGetCurrentTaskHandle() != control_task_handle) {
    return BSP_INVALID_ARG;
  }

  /* 计数型通知保留迟到周期数，带序号缓冲区再把通知与对应快照配对。 */
  const uint32_t notification_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  const uint32_t wake_cycles = DWT->CYCCNT;
  AppControlTickSample sample;

  taskENTER_CRITICAL();
  const bool sample_available =
    AppControlTickBuffer_Consume(&tick_buffer, notification_count, &sample);
  taskEXIT_CRITICAL();

  if (!sample_available) {
    return BSP_ERROR;
  }

  tick->tick_sequence = sample.tick_sequence;
  tick->irq_timestamp_cycles = sample.irq_timestamp_cycles;
  tick->irq_period_cycles = sample.irq_period_cycles;
  tick->timer_irq_missed_period_count = sample.timer_irq_missed_period_count;
  tick->task_wake_cycles = wake_cycles;
  tick->notification_count = notification_count;
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    tick->encoder_raw[i] = sample.encoder_raw[i];
  }
  return BSP_OK;
}

uint32_t AppControlTimebase_GetCycleCount(void)
{
  return DWT->CYCCNT;
}

void AppControlTimebase_OnTimerElapsedFromIsr(void)
{
  if (!timebase_started || control_task_handle == NULL) {
    return;
  }

  /* 中断内只做定长采集、发布和通知，不执行测速、状态机或控制律。 */
  const uint32_t now_cycles = irq_entry_timestamp_cycles;
  const uint32_t irq_period_cycles = now_cycles - previous_irq_timestamp_cycles;
  if (elapsed_cycles_since_start > (UINT64_MAX - (uint64_t)irq_period_cycles)) {
    elapsed_cycles_since_start = UINT64_MAX;
  } else {
    elapsed_cycles_since_start += (uint64_t)irq_period_cycles;
  }
  if (serviced_irq_count < UINT64_MAX) {
    serviced_irq_count++;
  }
  const uint32_t detected_missed_periods = AppControlTiming_CountMissedTimerPeriods(
    elapsed_cycles_since_start, expected_period_cycles, serviced_irq_count);
  if (detected_missed_periods > timer_irq_missed_period_count) {
    timer_irq_missed_period_count = detected_missed_periods;
  }

  uint16_t encoder_raw[BSP_MOTOR_COUNT];
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    encoder_raw[i] = BspEncoder_ReadRaw((BspMotorId)i);
  }
  AppControlTickBuffer_PublishFromIsr(
    &tick_buffer,
    now_cycles,
    irq_period_cycles,
    timer_irq_missed_period_count,
    encoder_raw);
  previous_irq_timestamp_cycles = now_cycles;

  BaseType_t higher_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(control_task_handle, &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void AppControlTimebase_OnTimerIrqEntryFromIsr(void)
{
  if (timebase_started) {
    irq_entry_timestamp_cycles = DWT->CYCCNT;
  }
}
