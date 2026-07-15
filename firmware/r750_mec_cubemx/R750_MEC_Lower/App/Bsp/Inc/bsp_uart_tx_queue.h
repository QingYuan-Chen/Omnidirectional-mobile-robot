#ifndef BSP_UART_TX_QUEUE_H
#define BSP_UART_TX_QUEUE_H

#include "robot_config.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_UART_TX_QUEUE_STORAGE_DEPTH (ROBOT_CONFIG_UART_TX_QUEUE_DEPTH + 1U)

typedef struct {
  uint16_t length;
  uint8_t data[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
} BspUartTxFrame;

typedef struct {
  BspUartTxFrame frames[BSP_UART_TX_QUEUE_STORAGE_DEPTH];
  _Atomic uint32_t head;
  _Atomic uint32_t tail;
} BspUartTxQueue;

void BspUartTxQueue_Init(BspUartTxQueue *queue);
bool BspUartTxQueue_Enqueue(
  BspUartTxQueue *queue,
  const uint8_t *data,
  uint16_t length);
bool BspUartTxQueue_Peek(
  const BspUartTxQueue *queue,
  const uint8_t **data,
  uint16_t *length);
bool BspUartTxQueue_Pop(BspUartTxQueue *queue);
uint32_t BspUartTxQueue_Count(const BspUartTxQueue *queue);

#ifdef __cplusplus
}
#endif

#endif
