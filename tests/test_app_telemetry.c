#include "app_telemetry.h"

#include "robot_config.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define UART_8N1_BITS_PER_BYTE (10U)
#define UART_BYTES_PER_TELEMETRY_PERIOD \
  ((ROBOT_CONFIG_UART_BAUDRATE * ROBOT_CONFIG_TELEMETRY_PERIOD_MS) / \
   (UART_8N1_BITS_PER_BYTE * 1000U))

_Static_assert(
  ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH <= UART_BYTES_PER_TELEMETRY_PERIOD,
  "telemetry frame budget exceeds sustained UART4 throughput");

static void FillWorstCase(AppTelemetryInput *input)
{
  memset(input, 0, sizeof(*input));
  input->now_ms = UINT32_MAX;
  input->control_timing.tick_sequence = UINT32_MAX;
  input->control_timing.irq_timestamp_cycles = UINT32_MAX;
  input->control_timing.actual_dt_cycles = UINT32_MAX;
  input->control_timing.irq_period_cycles = UINT32_MAX;
  input->control_timing.irq_jitter_cycles = UINT32_MAX;
  input->control_timing.irq_jitter_max_cycles = UINT32_MAX;
  input->control_timing.wake_latency_cycles = UINT32_MAX;
  input->control_timing.wake_latency_max_cycles = UINT32_MAX;
  input->control_timing.wake_latency_p99_us = UINT32_MAX;
  input->control_timing.wcet_cycles = UINT32_MAX;
  input->control_timing.wcet_max_cycles = UINT32_MAX;
  input->control_timing.notification_coalesced_count = UINT32_MAX;
  input->control_timing.timer_irq_missed_period_count = UINT32_MAX;
  input->control_timing.task_iteration_missed_period_count = UINT32_MAX;
  input->control_timing.missed_period_count = UINT32_MAX;
  input->control_timing.deadline_miss_count = UINT32_MAX;
  input->control_timing.sample_count = UINT32_MAX;
  for (uint32_t i = 0U; i < (uint32_t)BSP_MOTOR_COUNT; ++i) {
    input->encoder_raw[i] = UINT16_MAX;
    input->encoder_delta[i] = INT16_MIN;
    input->encoder_total[i] = INT64_MIN;
  }
  input->motor.state = APP_MOTOR_OPEN_LOOP_ESTOP_LATCHED;
  input->motor.target_pwm = INT16_MIN;
  input->motor.applied_pwm = INT16_MIN;
  input->battery_millivolts = UINT16_MAX;
  input->imu_sample_age_ms = UINT32_MAX;
  input->imu_health = APP_IMU_HEALTH_ESTIMATOR_FAULT;
  input->uart_error_count = UINT32_MAX;
  input->uart_rx_overflow_count = UINT32_MAX;
  input->uart_tx_fault_count = UINT32_MAX;
  input->command_reject_count = UINT32_MAX;
  input->command_queue_drop_count = UINT32_MAX;
  input->motion_gate_reject_count = UINT32_MAX;
  input->invalidated_motor_command_count = UINT32_MAX;
  input->adc_error_count = UINT32_MAX;
  input->telemetry_enqueued_count = UINT32_MAX;
  input->telemetry_enqueue_drop_count = UINT32_MAX;
  input->telemetry_format_error_count = UINT32_MAX;
  for (uint32_t i = 0U; i < 4U; ++i) {
    input->telemetry_frame_failure_count[i] = UINT32_MAX;
  }
  input->capture_event_drop_count = UINT32_MAX;
  input->capture_export_error_count = UINT32_MAX;
  input->health_miss_count = UINT32_MAX;
  input->minimum_free_heap_bytes = UINT32_MAX;
  for (uint32_t i = 0U; i < 5U; ++i) {
    input->stack_free_bytes[i] = UINT32_MAX;
  }
  memset(&input->uart, 0xFF, sizeof(input->uart));
  memset(&input->imu, 0, sizeof(input->imu));
  input->imu.health = APP_IMU_HEALTH_ESTIMATOR_FAULT;
  input->imu.flags = UINT32_MAX;
  input->imu.sequence = UINT32_MAX;
  input->imu.sensor_timestamp = UINT32_MAX;
  input->imu.sample_age_ms = UINT32_MAX;
  input->imu.read_error_count = UINT32_MAX;
  input->imu.consecutive_error_count = UINT32_MAX;
  input->imu.backoff_count = UINT32_MAX;
  input->imu.retry_delay_ms = UINT32_MAX;
  input->imu.duplicate_count = UINT32_MAX;
  input->imu.dropped_sample_count = UINT32_MAX;
  input->imu.accel_update_accept_count = UINT32_MAX;
  input->imu.accel_update_reject_count = UINT32_MAX;
  input->imu.spike_reject_count = UINT32_MAX;
  input->imu.consecutive_spike_count = UINT32_MAX;
  input->imu.stable_sample_count = UINT32_MAX;
  input->imu.estimator_fault_count = UINT32_MAX;
  input->critical_tasks_alive = true;
  input->runtime_ready = true;
  input->motion_inhibited = true;
  input->fault_latched = true;
}

static void TestNormalFrame(void)
{
  AppTelemetryInput input;
  uint8_t frame[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
  uint16_t length = 0U;
  memset(&input, 0, sizeof(input));
  input.now_ms = 123U;
  input.control_timing.tick_sequence = 7U;
  input.control_timing.irq_timestamp_cycles = 456U;
  input.control_timing.actual_dt_cycles = 168001U;
  input.control_timing.irq_period_cycles = 168000U;
  input.control_timing.irq_jitter_cycles = 1U;
  input.control_timing.irq_jitter_max_cycles = 9U;
  input.control_timing.wake_latency_cycles = 20U;
  input.control_timing.wake_latency_max_cycles = 30U;
  input.control_timing.wake_latency_p99_us = 2U;
  input.control_timing.wcet_cycles = 40U;
  input.control_timing.wcet_max_cycles = 50U;
  input.control_timing.notification_coalesced_count = 3U;
  input.control_timing.timer_irq_missed_period_count = 4U;
  input.control_timing.task_iteration_missed_period_count = 5U;
  input.control_timing.missed_period_count = 6U;
  input.control_timing.deadline_miss_count = 7U;
  input.control_timing.sample_count = 8U;
  input.encoder_raw[BSP_MOTOR_MA] = 10U;
  input.encoder_delta[BSP_MOTOR_MA] = -2;
  input.encoder_total[BSP_MOTOR_MA] = -20;
  input.motor.target_pwm = 100;
  input.motor.applied_pwm = 50;
  input.battery_millivolts = 12000U;
  input.imu_sample_age_ms = 11U;
  input.imu_health = APP_IMU_HEALTH_RECOVERING;
  assert(AppTelemetry_Format(&input, frame, (uint16_t)sizeof(frame), &length));
  assert(length > 0U);
  assert(length < sizeof(frame));
  assert(frame[length - 1U] == (uint8_t)'\n');
  assert(frame[length] == 0U);
  assert(strstr((const char *)frame, "T,123,7,456,R,10") != NULL);
  assert(strstr((const char *)frame, ",D,-2") != NULL);
  assert(strstr((const char *)frame, ",E,-20") != NULL);
  assert(strstr((const char *)frame, ",P,100,50,B,12000") != NULL);
  assert(strstr((const char *)frame, ",S,0,0,0,0,0,0,I,") != NULL);
  assert(strstr((const char *)frame, ",I,11,4,J,1,9,L,20,50") != NULL);
  assert(strstr((const char *)frame, ",M,6,7,C,") != NULL);
  assert(strstr((const char *)frame, ",C,0,0,0,0,0,0,0,0\n") != NULL);
}

static void TestWorstCaseBudgetAndBoundary(void)
{
  AppTelemetryInput input;
  uint8_t frame[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
  uint16_t length = 0U;
  FillWorstCase(&input);
  assert(AppTelemetry_Format(&input, frame, (uint16_t)sizeof(frame), &length));
  assert(length <= ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH);
  assert((ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH - length) >= 6U);
  assert(length <= UART_BYTES_PER_TELEMETRY_PERIOD);
  assert(length > 1U);
  assert(frame[length - 1U] == (uint8_t)'\n');
  assert(strstr((const char *)frame, ",S,1,1,1,1,6,1,I,") != NULL);
  assert(strstr((const char *)frame, ",I,4294967295,5,J,") != NULL);

  uint8_t exact[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
  uint16_t exact_length = 0U;
  assert(AppTelemetry_Format(&input, exact, length, &exact_length));
  assert(exact_length == length);
  assert(!AppTelemetry_Format(&input, exact, (uint16_t)(length - 1U), &exact_length));
  assert(exact_length == 0U);
}

static void TestArguments(void)
{
  AppTelemetryInput input;
  uint8_t frame[8];
  uint16_t length;
  memset(&input, 0, sizeof(input));
  assert(!AppTelemetry_Format(NULL, frame, (uint16_t)sizeof(frame), &length));
  assert(!AppTelemetry_Format(&input, NULL, (uint16_t)sizeof(frame), &length));
  assert(!AppTelemetry_Format(&input, frame, 0U, &length));
  assert(!AppTelemetry_Format(&input, frame, (uint16_t)sizeof(frame), NULL));
}

static void TestTypedFramesAndBandwidth(void)
{
  AppTelemetryInput input;
  AppTelemetrySchedule schedule;
  uint8_t frame[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
  uint16_t lengths[4] = {0U, 0U, 0U, 0U};
  FillWorstCase(&input);
  AppTelemetrySchedule_Init(&schedule, 1000U);
  AppTelemetrySchedule_UpdateEvents(&schedule, &input);

  for (uint32_t type = 0U; type < 4U; ++type) {
    assert(AppTelemetry_FormatTyped(
      (AppTelemetryFrameType)type,
      &schedule,
      &input,
      frame,
      (uint16_t)sizeof(frame),
      &lengths[type]));
    assert(lengths[type] > 0U);
    assert(lengths[type] <= ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH);
    assert(frame[lengths[type] - 1U] == (uint8_t)'\n');
  }

  assert(memcmp(frame, "EVENT,1,", 8U) == 0);
  const uint32_t worst_bytes_per_second =
    ((uint32_t)lengths[APP_TELEMETRY_FRAME_STAT] * 1000U /
      ROBOT_CONFIG_TELEMETRY_PERIOD_MS) +
    ((uint32_t)lengths[APP_TELEMETRY_FRAME_IMUQ] * 1000U /
      ROBOT_CONFIG_IMUQ_TELEMETRY_PERIOD_MS) +
    ((uint32_t)lengths[APP_TELEMETRY_FRAME_RES] * 1000U /
      ROBOT_CONFIG_RES_TELEMETRY_PERIOD_MS) +
    ((uint32_t)lengths[APP_TELEMETRY_FRAME_EVENT] * 1000U /
      ROBOT_CONFIG_EVENT_TELEMETRY_MIN_PERIOD_MS);
  const uint32_t uart_bytes_per_second =
    ROBOT_CONFIG_UART_BAUDRATE / UART_8N1_BITS_PER_BYTE;
  assert((worst_bytes_per_second * 4U) <=
         (uart_bytes_per_second * 3U));

  uint16_t failed_length = 123U;
  assert(!AppTelemetry_FormatTyped(
    APP_TELEMETRY_FRAME_EVENT,
    &schedule,
    &input,
    frame,
    (uint16_t)(lengths[APP_TELEMETRY_FRAME_EVENT] - 1U),
    &failed_length));
  assert(failed_length == 0U);
}

static void TestTypedSchedule(void)
{
  AppTelemetryInput input;
  AppTelemetrySchedule schedule;
  AppTelemetryFrameType frame_type;
  memset(&input, 0, sizeof(input));
  AppTelemetrySchedule_Init(&schedule, UINT32_MAX - 20U);
  AppTelemetrySchedule_UpdateEvents(&schedule, &input);

  assert(AppTelemetrySchedule_Select(
    &schedule, UINT32_MAX - 20U, true, &frame_type));
  assert(frame_type == APP_TELEMETRY_FRAME_EVENT);
  AppTelemetrySchedule_MarkAttempt(
    &schedule, frame_type, UINT32_MAX - 20U, false);
  assert(schedule.pending_event_flags == APP_TELEMETRY_EVENT_BOOT);
  assert(AppTelemetrySchedule_Select(
    &schedule, UINT32_MAX - 19U, false, &frame_type));
  assert(frame_type == APP_TELEMETRY_FRAME_STAT);
  assert(AppTelemetrySchedule_Select(
    &schedule, 80U, true, &frame_type));
  assert(frame_type == APP_TELEMETRY_FRAME_EVENT);
  AppTelemetrySchedule_MarkAttempt(&schedule, frame_type, 80U, true);
  assert(schedule.pending_event_flags == 0U);
  assert(schedule.event_sequence == 1U);
  assert(AppTelemetrySchedule_Select(
    &schedule, 81U, true, &frame_type));
  assert(frame_type == APP_TELEMETRY_FRAME_STAT);
  AppTelemetrySchedule_MarkAttempt(&schedule, frame_type, 81U, true);

  input.runtime_ready = true;
  input.motor.state = APP_MOTOR_OPEN_LOOP_RUNNING;
  input.telemetry_format_error_count = 1U;
  AppTelemetrySchedule_UpdateEvents(&schedule, &input);
  assert((schedule.pending_event_flags &
          APP_TELEMETRY_EVENT_READY_CHANGED) != 0U);
  assert((schedule.pending_event_flags &
          APP_TELEMETRY_EVENT_MOTOR_STATE_CHANGED) != 0U);
  assert((schedule.pending_event_flags &
          APP_TELEMETRY_EVENT_DROP_CHANGED) != 0U);
}

int main(void)
{
  TestNormalFrame();
  TestWorstCaseBudgetAndBoundary();
  TestArguments();
  TestTypedFramesAndBandwidth();
  TestTypedSchedule();
  return 0;
}
