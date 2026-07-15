#include "bsp_uart.h"

#include "bsp_uart_tx_queue.h"
#include "usart.h"

#include <limits.h>
#include <stdatomic.h>

/*
 * UART 实现为每个逻辑端口维护独立的接收环、发送队列、原子统计和恢复状态。
 * 接收中断是环形缓冲唯一生产者，通信任务是唯一消费者；发送通信任务是唯一生产者，
 * HAL 完成回调与任务恢复路径通过 tx_state 竞争同一逻辑消费者所有权。中断路径不循环
 * 重试，所有可能持续失败的恢复都交给周期 Service。
 */

#define BSP_UART_RX_BUFFER_SIZE (256U)
#define BSP_UART_TX_STATE_IDLE       (0U)
#define BSP_UART_TX_STATE_ACTIVE     (1U)
#define BSP_UART_TX_STATE_COMPLETING (2U)

typedef struct {
  /*
   * handle/irq_byte 在初始化后固定；rx_buffer 与 tx_queue 保存实际数据；所有会被任务和中断
   * 同时访问的索引、标志和计数均为 C11 原子。HAL 句柄内部状态由 HAL 驱动维护。
   */
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

/* 两个上下文按 BspUartPort 索引，逻辑端口与 HAL 句柄在 BspUart_Init 中一次绑定。 */
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
  /* CAS 循环允许任务与中断同时累计同一诊断，达到 UINT32_MAX 后保持饱和。 */
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
  /* 仅在收发尚未启动时调用；重置所有索引、统计、恢复标志、发送队列和所有权状态。 */
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
  /*
   * 每次只挂起一个字节的 HAL 中断接收。失败时增加 restart_failure 并保留 recovery_pending，
   * 后续 Service 会在 HAL RxState 回到 READY 后重试；成功才清除 pending。
   */
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
  /*
   * 先按 ErrorCode 位分别计数，再用 HAL 宏清除外设错误标志并复位句柄 ErrorCode。必须在
   * 重启接收前完成，否则 HAL 可能继续把新接收判为同一旧错误。
   */
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
  /* 只有 pending 且 HAL 接收状态 READY 时才尝试重启，避免在中断仍活动时重复调用 HAL。 */
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

  /*
   * 中断生产者 relaxed 读取自己的 head，acquire 读取任务 tail 判断是否满；写入字节后
   * release 发布 head。满时采用丢新字节策略并计数，不覆盖尚未消费的旧数据。
   */
  context->rx_buffer[head] = byte;
  atomic_store_explicit(&context->head, next_head, memory_order_release);
}

static BspStatus BspUart_StartNextTransmit(BspUartContext *context)
{
  /*
   * IDLE→ACTIVE 的 CAS 必须发生在 Peek 前。某些短帧可能在 HAL 起发后很快完成，若先取得
   * 队首指针再声明所有权，完成回调可能弹出槽位而本路径仍使用已经可复用的旧指针。
   */
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
    /* HAL_BUSY 不丢帧：保留 tail 槽位并把状态退回 IDLE，后续 Service 再尝试起发。 */
    atomic_store_explicit(
      &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
    return BSP_BUSY;
  }

  /*
   * 当前软件策略把 ERROR/TIMEOUT 记为起发失败并 Pop 本帧，避免同一队首永久堵塞整个
   * 发送队列；这不表示硬件不可恢复。调用者收到错误状态，后续帧仍可由 Service 处理。
   */
  BspUart_IncrementSaturated(&context->tx_start_failure_count);
  (void)BspUartTxQueue_Pop(&context->tx_queue);
  atomic_store_explicit(
    &context->tx_state, BSP_UART_TX_STATE_IDLE, memory_order_release);
  return BspUart_FromHal(hal_status);
}

static void BspUart_ServiceTransmit(BspUartContext *context)
{
  /*
   * 正常发送期间 tx_state=ACTIVE。若 HAL gState 已 READY 但完成回调没有推进队列，任务
   * CAS 到 COMPLETING 后代替回调 Pop；若回调先取得所有权，CAS 失败便不重复弹出。
   */
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
  /*
   * 先绑定固定硬件角色，再逐端口重置并启动首字节接收。若后一个端口失败，主动 Abort
   * 之前已启动端口，避免对上层返回失败后仍有接收中断写入半初始化上下文。
   */
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
  /* 入队成功即拥有稳定副本；HAL_BUSY 只表示当前无法起发，由 Service 重试，因此对调用者返回 OK。 */
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
  /* acquire 读取 head 后可见中断发布的字节；release 推进 tail 后生产者可安全复用该槽位。 */
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
  /*
   * 回调按 HAL 句柄定位逻辑端口。存在错误时不接收当前 irq_byte，只标记恢复；正常时先把
   * 字节发布到软件环，再立即挂起下一字节接收，保持连续流。
   */
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
      /*
       * ACTIVE→COMPLETING 的 CAS 决定唯一 Pop 所有者。若 CAS 失败，通常表示任务恢复路径
       * 已先处理该完成事件；只增加恢复诊断，绝不再次推进 tail。
       */
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
  /*
   * 错误回调记录并清理当前错误，再置 recovery_pending。TryRecover 只在 HAL 状态允许时
   * 起一次接收；若仍失败，pending 保持，交给下一个任务 Service 周期继续尝试。
   */
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
