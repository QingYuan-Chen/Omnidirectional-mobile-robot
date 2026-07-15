#include "bsp_uart.h"

#include "bsp_uart_tx_queue.h"
#include "usart.h"

#include <limits.h>
#include <stdatomic.h>

#define BSP_UART_RX_BUFFER_SIZE (256U)
#define BSP_UART_TX_STATE_IDLE       (0U)
#define BSP_UART_TX_STATE_ACTIVE     (1U)
#define BSP_UART_TX_STATE_COMPLETING (2U)

typedef struct {
  UART_HandleTypeDef *handle;
  uint8_t irq_byte;
  uint8_t rx_buffer[BSP_UART_RX_BUFFER_SIZE];
  _Atomic uint16_t head;
  _Atomic uint16_t tail;
  _Atomic uint32_t overflow_count;
  _Atomic uint32_t parity_error_count;
  _Atomic uint32_t noise_error_count;
  _Atomic uint32_t framing_error_count;
  _Atomic uint32_t overrun_error_count;
  _Atomic uint32_t rx_recovery_attempt_count;
  _Atomic uint32_t rx_recovery_success_count;
  _Atomic uint32_t rx_restart_failure_count;
  _Atomic bool rx_recovery_pending;
  BspUartTxQueue tx_queue;
  _Atomic uint32_t tx_enqueued_frame_count;
  _Atomic uint32_t tx_completed_frame_count;
  _Atomic uint32_t tx_queue_full_count;
  _Atomic uint32_t tx_start_failure_count;
  _Atomic uint32_t tx_completion_recovery_count;
  _Atomic uint32_t tx_state;
} BspUartContext;

/* 上下文由任务和中断共同访问，共享索引、统计量和发送所有权均使用原子变量。 */
static BspUartContext uart_contexts[BSP_UART_COUNT];

static bool BspUart_IsValid(BspUartPort port)
{
  return (uint32_t)port < (uint32_t)BSP_UART_COUNT;
}

static BspStatus BspUart_FromHal(HAL_StatusTypeDef status)
{
  switch (status) {
    case HAL_OK:
      return BSP_OK;
    case HAL_BUSY:
      return BSP_BUSY;
    case HAL_TIMEOUT:
      return BSP_TIMEOUT;
    default:
      return BSP_ERROR;
  }
}

static void BspUart_IncrementSaturated(_Atomic uint32_t *counter)
{
  uint32_t current = atomic_load_explicit(counter, memory_order_relaxed);
  while (current != UINT32_MAX &&
         !atomic_compare_exchange_weak_explicit(
           counter,
           &current,
           current + 1U,
           memory_order_relaxed,
           memory_order_relaxed)) {
  }
}

static uint16_t BspUart_NextIndex(uint16_t index)
{
  return (uint16_t)((index + 1U) % BSP_UART_RX_BUFFER_SIZE);
}

static void BspUart_ResetContext(BspUartContext *context)
{
  atomic_store_explicit(&context->head, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->tail, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->overflow_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->parity_error_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->noise_error_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->framing_error_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->overrun_error_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->rx_recovery_attempt_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->rx_recovery_success_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->rx_restart_failure_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->rx_recovery_pending, false, memory_order_relaxed);
  BspUartTxQueue_Init(&context->tx_queue);
  atomic_store_explicit(&context->tx_enqueued_frame_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->tx_completed_frame_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->tx_queue_full_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->tx_start_failure_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&context->tx_completion_recovery_count, 0U, memory_order_relaxed);
  atomic_store_explicit(
    &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
}

static BspStatus BspUart_StartReceive(BspUartContext *context)
{
  const HAL_StatusTypeDef status = HAL_UART_Receive_IT(context->handle, &context->irq_byte, 1U);
  if (status != HAL_OK) {
    BspUart_IncrementSaturated(&context->rx_restart_failure_count);
    atomic_store_explicit(
      &context->rx_recovery_pending, true, memory_order_release);
  } else {
    atomic_store_explicit(
      &context->rx_recovery_pending, false, memory_order_release);
  }
  return BspUart_FromHal(status);
}

static void BspUart_RecordErrors(BspUartContext *context, uint32_t error_code)
{
  if ((error_code & HAL_UART_ERROR_PE) != 0U) {
    BspUart_IncrementSaturated(&context->parity_error_count);
  }
  if ((error_code & HAL_UART_ERROR_NE) != 0U) {
    BspUart_IncrementSaturated(&context->noise_error_count);
  }
  if ((error_code & HAL_UART_ERROR_FE) != 0U) {
    BspUart_IncrementSaturated(&context->framing_error_count);
  }
  if ((error_code & HAL_UART_ERROR_ORE) != 0U) {
    BspUart_IncrementSaturated(&context->overrun_error_count);
  }
}

static void BspUart_RecordAndClearPendingErrors(BspUartContext *context)
{
  const uint32_t error_code = context->handle->ErrorCode;
  if (error_code == HAL_UART_ERROR_NONE) {
    return;
  }

  BspUart_RecordErrors(context, error_code);
  __HAL_UART_CLEAR_PEFLAG(context->handle);
  context->handle->ErrorCode = HAL_UART_ERROR_NONE;
}

static void BspUart_TryRecover(BspUartContext *context)
{
  if (!atomic_load_explicit(
        &context->rx_recovery_pending, memory_order_acquire) ||
      context->handle->RxState != HAL_UART_STATE_READY) {
    return;
  }

  BspUart_RecordAndClearPendingErrors(context);
  BspUart_IncrementSaturated(&context->rx_recovery_attempt_count);
  if (BspUart_StartReceive(context) == BSP_OK) {
    BspUart_IncrementSaturated(&context->rx_recovery_success_count);
  }
}

static void BspUart_PushByte(BspUartContext *context, uint8_t byte)
{
  const uint16_t head = atomic_load_explicit(&context->head, memory_order_relaxed);
  const uint16_t next_head = BspUart_NextIndex(head);
  const uint16_t tail = atomic_load_explicit(&context->tail, memory_order_acquire);

  if (next_head == tail) {
    BspUart_IncrementSaturated(&context->overflow_count);
    return;
  }

  /* 先写数据再发布新队首，任务侧取得队首后才能读取完整字节。 */
  context->rx_buffer[head] = byte;
  atomic_store_explicit(&context->head, next_head, memory_order_release);
}

static BspStatus BspUart_StartNextTransmit(BspUartContext *context)
{
  /* 必须先取得发送所有权再读取队首，防止立即完成回调弹出后仍使用旧帧指针。 */
  uint32_t expected_state = BSP_UART_TX_STATE_IDLE;
  if (!atomic_compare_exchange_strong_explicit(
        &context->tx_state,
        &expected_state,
        BSP_UART_TX_STATE_ACTIVE,
        memory_order_acq_rel,
        memory_order_acquire)) {
    return BSP_BUSY;
  }

  const uint8_t *data = NULL;
  uint16_t length = 0U;
  if (!BspUartTxQueue_Peek(&context->tx_queue, &data, &length)) {
    atomic_store_explicit(
      &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
    return BSP_OK;
  }

  const HAL_StatusTypeDef hal_status = HAL_UART_Transmit_IT(
    context->handle, data, length);
  if (hal_status == HAL_OK) {
    return BSP_OK;
  }
  if (hal_status == HAL_BUSY) {
    /* 外设忙时保留队首并释放所有权，交给后续服务周期重试。 */
    atomic_store_explicit(
      &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
    return BSP_BUSY;
  }

  /* 不可重试的起发错误丢弃当前帧并计数，避免坏帧永久堵住队列。 */
  BspUart_IncrementSaturated(&context->tx_start_failure_count);
  (void)BspUartTxQueue_Pop(&context->tx_queue);
  atomic_store_explicit(
    &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
  return BspUart_FromHal(hal_status);
}

static void BspUart_ServiceTransmit(BspUartContext *context)
{
  /* HAL 已空闲但完成回调缺失时，由任务恢复路径独占弹出并继续发送。 */
  uint32_t expected_state = BSP_UART_TX_STATE_ACTIVE;
  if (context->handle->gState == HAL_UART_STATE_READY &&
      atomic_compare_exchange_strong_explicit(
        &context->tx_state,
        &expected_state,
        BSP_UART_TX_STATE_COMPLETING,
        memory_order_acq_rel,
        memory_order_acquire)) {
    BspUart_IncrementSaturated(&context->tx_completion_recovery_count);
    (void)BspUartTxQueue_Pop(&context->tx_queue);
    atomic_store_explicit(
      &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
  }
  (void)BspUart_StartNextTransmit(context);
}

BspStatus BspUart_Init(void)
{
  uart_contexts[BSP_UART_ROS].handle = &huart2;
  uart_contexts[BSP_UART_TTL].handle = &huart4;
  for (uint32_t i = 0; i < BSP_UART_COUNT; ++i) {
    BspUartContext *context = &uart_contexts[i];
    BspUart_ResetContext(context);

    const BspStatus status = BspUart_StartReceive(context);
    if (status != BSP_OK) {
      for (uint32_t started = 0U; started < i; ++started) {
        (void)HAL_UART_AbortReceive(uart_contexts[started].handle);
      }
      return status;
    }
  }

  return BSP_OK;
}

void BspUart_Service(void)
{
  for (uint32_t i = 0; i < BSP_UART_COUNT; ++i) {
    BspUartContext *context = &uart_contexts[i];
    BspUart_TryRecover(context);
    BspUart_ServiceTransmit(context);
  }
}

BspStatus BspUart_WriteAsync(BspUartPort port, const uint8_t *data, uint16_t length)
{
  if (!BspUart_IsValid(port) || data == NULL || length == 0U ||
      length > ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH) {
    return BSP_INVALID_ARG;
  }
  BspUartContext *context = &uart_contexts[port];
  if (!BspUartTxQueue_Enqueue(&context->tx_queue, data, length)) {
    BspUart_IncrementSaturated(&context->tx_queue_full_count);
    return BSP_BUSY;
  }
  BspUart_IncrementSaturated(&context->tx_enqueued_frame_count);
  const BspStatus start_status = BspUart_StartNextTransmit(context);
  return start_status == BSP_ERROR || start_status == BSP_TIMEOUT
           ? start_status
           : BSP_OK;
}

bool BspUart_ReadByte(BspUartPort port, uint8_t *byte)
{
  if (!BspUart_IsValid(port) || byte == NULL) {
    return false;
  }

  BspUartContext *context = &uart_contexts[port];
  const uint16_t tail = atomic_load_explicit(&context->tail, memory_order_relaxed);
  const uint16_t head = atomic_load_explicit(&context->head, memory_order_acquire);
  if (head == tail) {
    return false;
  }

  *byte = context->rx_buffer[tail];
  atomic_store_explicit(
    &context->tail, BspUart_NextIndex(tail), memory_order_release);

  return true;
}

uint16_t BspUart_Available(BspUartPort port)
{
  if (!BspUart_IsValid(port)) {
    return 0U;
  }

  const BspUartContext *context = &uart_contexts[port];
  const uint16_t tail = atomic_load_explicit(&context->tail, memory_order_relaxed);
  const uint16_t head = atomic_load_explicit(&context->head, memory_order_acquire);
  if (head >= tail) {
    return (uint16_t)(head - tail);
  }

  return (uint16_t)(BSP_UART_RX_BUFFER_SIZE - tail + head);
}

uint32_t BspUart_GetOverflowCount(BspUartPort port)
{
  if (!BspUart_IsValid(port)) {
    return 0U;
  }

  return atomic_load_explicit(
    &uart_contexts[port].overflow_count, memory_order_relaxed);
}

BspStatus BspUart_GetStats(BspUartPort port, BspUartStats *stats)
{
  if (!BspUart_IsValid(port) || stats == NULL) {
    return BSP_INVALID_ARG;
  }

  const BspUartContext *context = &uart_contexts[port];
  stats->parity_error_count = atomic_load_explicit(
    &context->parity_error_count, memory_order_relaxed);
  stats->noise_error_count = atomic_load_explicit(
    &context->noise_error_count, memory_order_relaxed);
  stats->framing_error_count = atomic_load_explicit(
    &context->framing_error_count, memory_order_relaxed);
  stats->overrun_error_count = atomic_load_explicit(
    &context->overrun_error_count, memory_order_relaxed);
  stats->rx_recovery_attempt_count = atomic_load_explicit(
    &context->rx_recovery_attempt_count, memory_order_relaxed);
  stats->rx_recovery_success_count = atomic_load_explicit(
    &context->rx_recovery_success_count, memory_order_relaxed);
  stats->rx_restart_failure_count = atomic_load_explicit(
    &context->rx_restart_failure_count, memory_order_relaxed);
  stats->rx_buffer_overflow_count = atomic_load_explicit(
    &context->overflow_count, memory_order_relaxed);
  stats->tx_enqueued_frame_count = atomic_load_explicit(
    &context->tx_enqueued_frame_count, memory_order_relaxed);
  stats->tx_completed_frame_count = atomic_load_explicit(
    &context->tx_completed_frame_count, memory_order_relaxed);
  stats->tx_queue_full_count = atomic_load_explicit(
    &context->tx_queue_full_count, memory_order_relaxed);
  stats->tx_start_failure_count = atomic_load_explicit(
    &context->tx_start_failure_count, memory_order_relaxed);
  stats->tx_completion_recovery_count = atomic_load_explicit(
    &context->tx_completion_recovery_count, memory_order_relaxed);
  stats->tx_queued_frame_count = BspUartTxQueue_Count(&context->tx_queue);
  return BSP_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  for (uint32_t i = 0; i < BSP_UART_COUNT; ++i) {
    BspUartContext *context = &uart_contexts[i];
    if (context->handle == huart) {
      if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
        atomic_store_explicit(
          &context->rx_recovery_pending, true, memory_order_release);
        return;
      }
      BspUart_PushByte(context, context->irq_byte);
      (void)BspUart_StartReceive(context);
      return;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  for (uint32_t i = 0U; i < (uint32_t)BSP_UART_COUNT; ++i) {
    BspUartContext *context = &uart_contexts[i];
    if (context->handle == huart) {
      /* 完成中断和任务恢复路径竞争同一三态所有权，只有胜者可以弹出队首。 */
      uint32_t expected_state = BSP_UART_TX_STATE_ACTIVE;
      if (!atomic_compare_exchange_strong_explicit(
            &context->tx_state,
            &expected_state,
            BSP_UART_TX_STATE_COMPLETING,
            memory_order_acq_rel,
            memory_order_acquire)) {
        BspUart_IncrementSaturated(&context->tx_completion_recovery_count);
        return;
      }
      if (BspUartTxQueue_Pop(&context->tx_queue)) {
        BspUart_IncrementSaturated(&context->tx_completed_frame_count);
      } else {
        BspUart_IncrementSaturated(&context->tx_completion_recovery_count);
      }
      atomic_store_explicit(
        &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
      (void)BspUart_StartNextTransmit(context);
      return;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  for (uint32_t i = 0; i < BSP_UART_COUNT; ++i) {
    BspUartContext *context = &uart_contexts[i];
    if (context->handle == huart) {
      BspUart_RecordAndClearPendingErrors(context);
      atomic_store_explicit(
        &context->rx_recovery_pending, true, memory_order_release);
      BspUart_TryRecover(context);
      return;
    }
  }
}
