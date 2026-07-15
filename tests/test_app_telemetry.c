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
  input->adc_error_count = UINT32_MAX;
  input->critical_tasks_alive = true;
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
  assert(strstr((const char *)frame, ",S,0,0,0,0,0,I,") != NULL);
  assert(strstr((const char *)frame, ",I,11,4,J,1,9,L,20,50") != NULL);
  assert(strstr((const char *)frame, ",M,6,7,C,") != NULL);
  assert(strstr((const char *)frame, ",C,0,0,0,0,0,0\n") != NULL);
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
  assert(strstr((const char *)frame, ",S,1,1,1,6,1,I,") != NULL);
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

int main(void)
{
  TestNormalFrame();
  TestWorstCaseBudgetAndBoundary();
  TestArguments();
  return 0;
}
