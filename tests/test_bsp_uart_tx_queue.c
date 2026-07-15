#include "bsp_uart_tx_queue.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

static void TestCapacityAndFifo(void)
{
  BspUartTxQueue queue;
  uint8_t frames[ROBOT_CONFIG_UART_TX_QUEUE_DEPTH][4];
  BspUartTxQueue_Init(&queue);
  for (uint32_t i = 0U; i < ROBOT_CONFIG_UART_TX_QUEUE_DEPTH; ++i) {
    frames[i][0] = (uint8_t)i;
    frames[i][1] = (uint8_t)(i + 1U);
    assert(BspUartTxQueue_Enqueue(&queue, frames[i], 2U));
  }
  assert(BspUartTxQueue_Count(&queue) == ROBOT_CONFIG_UART_TX_QUEUE_DEPTH);
  assert(!BspUartTxQueue_Enqueue(&queue, frames[0], 2U));

  for (uint32_t i = 0U; i < ROBOT_CONFIG_UART_TX_QUEUE_DEPTH; ++i) {
    const uint8_t *data = NULL;
    uint16_t length = 0U;
    assert(BspUartTxQueue_Peek(&queue, &data, &length));
    assert(length == 2U);
    assert(memcmp(data, frames[i], length) == 0);
    assert(BspUartTxQueue_Pop(&queue));
  }
  assert(BspUartTxQueue_Count(&queue) == 0U);
  const uint8_t *data = NULL;
  uint16_t length = 0U;
  assert(!BspUartTxQueue_Peek(&queue, &data, &length));
  assert(!BspUartTxQueue_Pop(&queue));
}

static void TestWrapAndMaximumFrame(void)
{
  BspUartTxQueue queue;
  uint8_t frame[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
  memset(frame, 0xA5, sizeof(frame));
  BspUartTxQueue_Init(&queue);
  for (uint32_t cycle = 0U; cycle < 20U; ++cycle) {
    assert(BspUartTxQueue_Enqueue(&queue, frame, (uint16_t)sizeof(frame)));
    const uint8_t *data = NULL;
    uint16_t length = 0U;
    assert(BspUartTxQueue_Peek(&queue, &data, &length));
    assert(length == sizeof(frame));
    assert(data[0] == 0xA5U);
    assert(data[length - 1U] == 0xA5U);
    assert(BspUartTxQueue_Pop(&queue));
  }
}

static void TestArguments(void)
{
  BspUartTxQueue queue;
  uint8_t frame[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH + 1U] = {0};
  const uint8_t *data;
  uint16_t length;
  BspUartTxQueue_Init(NULL);
  BspUartTxQueue_Init(&queue);
  assert(!BspUartTxQueue_Enqueue(NULL, frame, 1U));
  assert(!BspUartTxQueue_Enqueue(&queue, NULL, 1U));
  assert(!BspUartTxQueue_Enqueue(&queue, frame, 0U));
  assert(!BspUartTxQueue_Enqueue(&queue, frame, (uint16_t)sizeof(frame)));
  assert(!BspUartTxQueue_Peek(NULL, &data, &length));
  assert(!BspUartTxQueue_Peek(&queue, NULL, &length));
  assert(!BspUartTxQueue_Peek(&queue, &data, NULL));
  assert(!BspUartTxQueue_Pop(NULL));
  assert(BspUartTxQueue_Count(NULL) == 0U);
}

int main(void)
{
  TestCapacityAndFifo();
  TestWrapAndMaximumFrame();
  TestArguments();
  return 0;
}
