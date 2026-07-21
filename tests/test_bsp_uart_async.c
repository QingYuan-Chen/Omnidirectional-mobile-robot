#include "bsp_uart.h"

#include "robot_config.h"
#include "usart.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TX_LOG_CAPACITY (16U)

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

static HAL_StatusTypeDef next_tx_status;
static uint32_t tx_attempt_count;
static uint32_t tx_success_count;
static uint16_t tx_lengths[TX_LOG_CAPACITY];
static uint8_t tx_frames[TX_LOG_CAPACITY][ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
static bool complete_during_start;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle);

HAL_StatusTypeDef HAL_UART_Receive_IT(
  UART_HandleTypeDef *handle,
  uint8_t *data,
  uint16_t length)
{
  (void)data;
  assert(handle != NULL);
  assert(length == 1U);
  handle->RxState = HAL_UART_STATE_BUSY_RX;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(
  UART_HandleTypeDef *handle,
  const uint8_t *data,
  uint16_t length)
{
  assert(handle != NULL);
  assert(data != NULL);
  assert(length > 0U);
  tx_attempt_count++;
  const HAL_StatusTypeDef status = next_tx_status;
  next_tx_status = HAL_OK;
  if (status != HAL_OK) {
    return status;
  }

  assert(tx_success_count < TX_LOG_CAPACITY);
  tx_lengths[tx_success_count] = length;
  memcpy(tx_frames[tx_success_count], data, length);
  tx_success_count++;
  handle->gState = HAL_UART_STATE_BUSY_TX;
  if (complete_during_start) {
    complete_during_start = false;
    handle->gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(handle);
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle)
{
  assert(handle != NULL);
  handle->RxState = HAL_UART_STATE_READY;
  return HAL_OK;
}

static void ResetHarness(void)
{
  memset(&huart1, 0, sizeof(huart1));
  memset(&huart2, 0, sizeof(huart2));
  memset(&huart4, 0, sizeof(huart4));
  huart1.gState = HAL_UART_STATE_READY;
  huart1.RxState = HAL_UART_STATE_READY;
  huart2.gState = HAL_UART_STATE_READY;
  huart2.RxState = HAL_UART_STATE_READY;
  huart4.gState = HAL_UART_STATE_READY;
  huart4.RxState = HAL_UART_STATE_READY;
  next_tx_status = HAL_OK;
  tx_attempt_count = 0U;
  tx_success_count = 0U;
  memset(tx_lengths, 0, sizeof(tx_lengths));
  memset(tx_frames, 0, sizeof(tx_frames));
  complete_during_start = false;
  assert(BspUart_Init() == BSP_OK);
}

static void Complete(UART_HandleTypeDef *handle)
{
  handle->gState = HAL_UART_STATE_READY;
  HAL_UART_TxCpltCallback(handle);
}

static void TestCallbackChainAndImmediateCompletion(void)
{
  static const uint8_t first[] = {1U, 2U};
  static const uint8_t second[] = {3U, 4U, 5U};
  BspUartStats stats;
  ResetHarness();
  assert(BspUart_WriteAsync(BSP_UART_TTL, first, (uint16_t)sizeof(first)) == BSP_OK);
  assert(BspUart_WriteAsync(BSP_UART_TTL, second, (uint16_t)sizeof(second)) == BSP_OK);
  assert(tx_success_count == 1U);
  BspUart_Service();
  assert(tx_success_count == 1U);
  Complete(&huart1);
  assert(tx_success_count == 2U);
  assert(tx_lengths[0] == sizeof(first));
  assert(tx_lengths[1] == sizeof(second));
  assert(memcmp(tx_frames[0], first, sizeof(first)) == 0);
  assert(memcmp(tx_frames[1], second, sizeof(second)) == 0);
  Complete(&huart1);
  assert(BspUart_GetStats(BSP_UART_TTL, &stats) == BSP_OK);
  assert(stats.tx_completed_frame_count == 2U);
  assert(stats.tx_queued_frame_count == 0U);

  ResetHarness();
  complete_during_start = true;
  assert(BspUart_WriteAsync(BSP_UART_TTL, first, (uint16_t)sizeof(first)) == BSP_OK);
  assert(BspUart_GetStats(BSP_UART_TTL, &stats) == BSP_OK);
  assert(stats.tx_completed_frame_count == 1U);
  assert(stats.tx_queued_frame_count == 0U);
}

static void TestBusyErrorQueueFullAndRecovery(void)
{
  static const uint8_t frame_a[] = {0xA1U};
  static const uint8_t frame_b[] = {0xB1U};
  BspUartStats stats;
  ResetHarness();
  next_tx_status = HAL_BUSY;
  assert(BspUart_WriteAsync(
           BSP_UART_TTL, frame_a, (uint16_t)sizeof(frame_a)) == BSP_OK);
  assert(tx_attempt_count == 1U);
  assert(tx_success_count == 0U);
  BspUart_Service();
  assert(tx_attempt_count == 2U);
  assert(tx_success_count == 1U);
  Complete(&huart1);

  ResetHarness();
  next_tx_status = HAL_ERROR;
  assert(BspUart_WriteAsync(
           BSP_UART_TTL, frame_a, (uint16_t)sizeof(frame_a)) == BSP_ERROR);
  assert(BspUart_GetStats(BSP_UART_TTL, &stats) == BSP_OK);
  assert(stats.tx_start_failure_count == 1U);
  assert(stats.tx_queued_frame_count == 0U);

  ResetHarness();
  assert(BspUart_WriteAsync(
           BSP_UART_TTL, frame_a, (uint16_t)sizeof(frame_a)) == BSP_OK);
  for (uint32_t i = 1U; i < ROBOT_CONFIG_UART_TX_QUEUE_DEPTH; ++i) {
    assert(BspUart_WriteAsync(
             BSP_UART_TTL, frame_b, (uint16_t)sizeof(frame_b)) == BSP_OK);
  }
  assert(BspUart_WriteAsync(
           BSP_UART_TTL, frame_b, (uint16_t)sizeof(frame_b)) == BSP_BUSY);
  assert(BspUart_GetStats(BSP_UART_TTL, &stats) == BSP_OK);
  assert(stats.tx_queue_full_count == 1U);
  assert(stats.tx_queued_frame_count == ROBOT_CONFIG_UART_TX_QUEUE_DEPTH);

  ResetHarness();
  assert(BspUart_WriteAsync(
           BSP_UART_TTL, frame_a, (uint16_t)sizeof(frame_a)) == BSP_OK);
  assert(BspUart_WriteAsync(
           BSP_UART_TTL, frame_b, (uint16_t)sizeof(frame_b)) == BSP_OK);
  huart1.gState = HAL_UART_STATE_READY;
  BspUart_Service();
  assert(tx_success_count == 2U);
  assert(tx_frames[0][0] == 0xA1U);
  assert(tx_frames[1][0] == 0xB1U);
  assert(BspUart_GetStats(BSP_UART_TTL, &stats) == BSP_OK);
  assert(stats.tx_completion_recovery_count == 1U);
  assert(stats.tx_queued_frame_count == 1U);
  Complete(&huart1);
  assert(BspUart_GetStats(BSP_UART_TTL, &stats) == BSP_OK);
  assert(stats.tx_completed_frame_count == 1U);
  assert(stats.tx_queued_frame_count == 0U);
}

static void TestSpuriousCompletionAndArguments(void)
{
  static const uint8_t frame[] = {1U};
  BspUartStats stats;
  ResetHarness();
  HAL_UART_TxCpltCallback(&huart1);
  assert(BspUart_GetStats(BSP_UART_TTL, &stats) == BSP_OK);
  assert(stats.tx_completion_recovery_count == 1U);
  assert(BspUart_WriteAsync(BSP_UART_COUNT, frame, 1U) == BSP_INVALID_ARG);
  assert(BspUart_WriteAsync(BSP_UART_TTL, NULL, 1U) == BSP_INVALID_ARG);
  assert(BspUart_WriteAsync(BSP_UART_TTL, frame, 0U) == BSP_INVALID_ARG);
}

int main(void)
{
  TestCallbackChainAndImmediateCompletion();
  TestBusyErrorQueueFullAndRecovery();
  TestSpuriousCompletionAndArguments();
  return 0;
}
