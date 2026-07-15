#ifndef BSP_UART_TX_QUEUE_H
#define BSP_UART_TX_QUEUE_H

#include "robot_config.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

/* 单生产者、单消费者定长帧队列，负责隔离任务发送和完成中断。 */

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_UART_TX_QUEUE_STORAGE_DEPTH (ROBOT_CONFIG_UART_TX_QUEUE_DEPTH + 1U)

/* 队列拥有帧数据副本，调用者缓冲区可在入队后立即复用。 */
typedef struct {
  uint16_t length;
  uint8_t data[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
} BspUartTxFrame;

/* 额外一个存储槽用于区分队满和队空。 */
typedef struct {
  BspUartTxFrame frames[BSP_UART_TX_QUEUE_STORAGE_DEPTH];
  _Atomic uint32_t head;
  _Atomic uint32_t tail;
} BspUartTxQueue;

/* 清空帧存储并初始化原子索引。 */
void BspUartTxQueue_Init(BspUartTxQueue *queue);
/* 复制完整帧并在最后发布队首。 */
bool BspUartTxQueue_Enqueue(
  BspUartTxQueue *queue,
  const uint8_t *data,
  uint16_t length);
/* 返回队尾帧只读指针，不推进队列。 */
bool BspUartTxQueue_Peek(
  const BspUartTxQueue *queue,
  const uint8_t **data,
  uint16_t *length);
/* 发送完成所有权持有者调用，推进队尾。 */
bool BspUartTxQueue_Pop(BspUartTxQueue *queue);
/* 返回当前排队帧数的瞬时快照。 */
uint32_t BspUartTxQueue_Count(const BspUartTxQueue *queue);

#ifdef __cplusplus
}
#endif

#endif
