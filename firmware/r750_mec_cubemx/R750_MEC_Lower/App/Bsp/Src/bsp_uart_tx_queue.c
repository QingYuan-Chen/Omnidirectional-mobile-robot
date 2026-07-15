#include "bsp_uart_tx_queue.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * 发送队列在任务生产者与发送完成消费者之间传递“完整帧所有权”。
 * 每次入队都复制帧，HAL 发送期间使用的指针指向队列稳定槽位；只有完成路径 Pop 后该
 * 槽位才可复用。head 只由单任务生产者写；tail 由取得 COMPLETING 所有权的中断或任务
 * 恢复路径作为单一逻辑消费者写。原子 acquire/release 只负责跨上下文可见性，不把
 * 本结构扩展成多生产者或多个并行消费者队列。
 */

_Static_assert(ROBOT_CONFIG_UART_TX_QUEUE_DEPTH > 0U,
               "UART TX queue depth must be non-zero");
_Static_assert(ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH > 0U,
               "UART TX frame length must be non-zero");
_Static_assert(ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH <= UINT16_MAX,
               "UART TX frame length exceeds uint16_t");

static uint32_t BspUartTxQueue_Next(uint32_t index)
{
  /* 存储深度通常很小且不要求 2 的幂，使用取模推进环形索引。 */
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

  /*
   * 生产者 relaxed 读取自己拥有的 head，acquire 读取消费者 tail 判断空间；完成数据复制
   * 和 length 写入后，以 release 发布新 head。消费者 acquire 读取 head 后可见完整槽位。
   */
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
  /* Peek 不取得新所有权，只暴露当前 tail 槽位；真正的消费者互斥由 UART tx_state 保证。 */
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
  /*
   * Pop 的调用者必须已经通过 UART 三态 CAS 取得 COMPLETING 所有权，否则中断和任务恢复
   * 可能对同一帧弹出两次。release 推进 tail 后，生产者即可安全复用旧槽位。
   */
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
