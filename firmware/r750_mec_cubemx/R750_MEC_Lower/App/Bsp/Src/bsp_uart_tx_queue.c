#include "bsp_uart_tx_queue.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/* 发送队列复制完整帧，并通过原子索引在任务与中断之间发布所有权。 */

_Static_assert(ROBOT_CONFIG_UART_TX_QUEUE_DEPTH > 0U,
               "UART TX queue depth must be non-zero");
_Static_assert(ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH > 0U,
               "UART TX frame length must be non-zero");
_Static_assert(ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH <= UINT16_MAX,
               "UART TX frame length exceeds uint16_t");

static uint32_t BspUartTxQueue_Next(uint32_t index)
{
  return (index + 1U) % BSP_UART_TX_QUEUE_STORAGE_DEPTH;
}

void BspUartTxQueue_Init(BspUartTxQueue *queue)
{
  if (queue == NULL) {
    return;
  }
  memset(queue->frames, 0, sizeof(queue->frames));
  atomic_init(&queue->head, 0U);
  atomic_init(&queue->tail, 0U);
}

bool BspUartTxQueue_Enqueue(
  BspUartTxQueue *queue,
  const uint8_t *data,
  uint16_t length)
{
  if (queue == NULL || data == NULL || length == 0U ||
      length > ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH) {
    return false;
  }

  const uint32_t head = atomic_load_explicit(&queue->head, memory_order_relaxed);
  const uint32_t next_head = BspUartTxQueue_Next(head);
  const uint32_t tail = atomic_load_explicit(&queue->tail, memory_order_acquire);
  if (next_head == tail) {
    return false;
  }

  /* 先复制完整帧再发布队首，完成中断不会看到尚未写完的内容。 */
  memcpy(queue->frames[head].data, data, length);
  queue->frames[head].length = length;
  atomic_store_explicit(&queue->head, next_head, memory_order_release);
  return true;
}

bool BspUartTxQueue_Peek(
  const BspUartTxQueue *queue,
  const uint8_t **data,
  uint16_t *length)
{
  if (queue == NULL || data == NULL || length == NULL) {
    return false;
  }
  const uint32_t tail = atomic_load_explicit(&queue->tail, memory_order_relaxed);
  const uint32_t head = atomic_load_explicit(&queue->head, memory_order_acquire);
  if (tail == head) {
    return false;
  }
  *data = queue->frames[tail].data;
  *length = queue->frames[tail].length;
  return true;
}

bool BspUartTxQueue_Pop(BspUartTxQueue *queue)
{
  if (queue == NULL) {
    return false;
  }
  const uint32_t tail = atomic_load_explicit(&queue->tail, memory_order_relaxed);
  const uint32_t head = atomic_load_explicit(&queue->head, memory_order_acquire);
  if (tail == head) {
    return false;
  }
  /* 只有取得发送完成所有权的路径可以推进队尾。 */
  atomic_store_explicit(
    &queue->tail, BspUartTxQueue_Next(tail), memory_order_release);
  return true;
}

uint32_t BspUartTxQueue_Count(const BspUartTxQueue *queue)
{
  if (queue == NULL) {
    return 0U;
  }
  const uint32_t head = atomic_load_explicit(&queue->head, memory_order_acquire);
  const uint32_t tail = atomic_load_explicit(&queue->tail, memory_order_acquire);
  if (head >= tail) {
    return head - tail;
  }
  return BSP_UART_TX_QUEUE_STORAGE_DEPTH - tail + head;
}
