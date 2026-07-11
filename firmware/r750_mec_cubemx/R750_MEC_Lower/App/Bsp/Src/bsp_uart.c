#include "bsp_uart.h"

#include "usart.h"

#define BSP_UART_RX_BUFFER_SIZE (256U)

typedef struct {
  UART_HandleTypeDef *handle;
  uint8_t irq_byte;
  uint8_t rx_buffer[BSP_UART_RX_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint32_t overflow_count;
  volatile uint32_t parity_error_count;
  volatile uint32_t noise_error_count;
  volatile uint32_t framing_error_count;
  volatile uint32_t overrun_error_count;
  volatile uint32_t rx_restart_failure_count;
  volatile bool rx_recovery_pending;
} BspUartContext;

static BspUartContext uart_contexts[BSP_UART_COUNT] = {
  [BSP_UART_ROS] = {.handle = &huart2},
  [BSP_UART_TTL] = {.handle = &huart4},
};

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

static uint16_t BspUart_NextIndex(uint16_t index)
{
  return (uint16_t)((index + 1U) % BSP_UART_RX_BUFFER_SIZE);
}

static void BspUart_ResetContext(BspUartContext *context)
{
  context->head = 0U;
  context->tail = 0U;
  context->overflow_count = 0U;
  context->parity_error_count = 0U;
  context->noise_error_count = 0U;
  context->framing_error_count = 0U;
  context->overrun_error_count = 0U;
  context->rx_restart_failure_count = 0U;
  context->rx_recovery_pending = false;
}

static BspStatus BspUart_StartReceive(BspUartContext *context)
{
  const HAL_StatusTypeDef status = HAL_UART_Receive_IT(context->handle, &context->irq_byte, 1U);
  if (status != HAL_OK) {
    context->rx_restart_failure_count++;
    context->rx_recovery_pending = true;
  } else {
    context->rx_recovery_pending = false;
  }
  return BspUart_FromHal(status);
}

static void BspUart_RecordErrors(BspUartContext *context, uint32_t error_code)
{
  if ((error_code & HAL_UART_ERROR_PE) != 0U) {
    context->parity_error_count++;
  }
  if ((error_code & HAL_UART_ERROR_NE) != 0U) {
    context->noise_error_count++;
  }
  if ((error_code & HAL_UART_ERROR_FE) != 0U) {
    context->framing_error_count++;
  }
  if ((error_code & HAL_UART_ERROR_ORE) != 0U) {
    context->overrun_error_count++;
  }
}

static void BspUart_PushByte(BspUartContext *context, uint8_t byte)
{
  const uint16_t next_head = BspUart_NextIndex(context->head);

  if (next_head == context->tail) {
    context->overflow_count++;
    return;
  }

  context->rx_buffer[context->head] = byte;
  context->head = next_head;
}

BspStatus BspUart_Init(void)
{
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
    if (context->rx_recovery_pending && context->handle->RxState == HAL_UART_STATE_READY) {
      (void)BspUart_StartReceive(context);
    }
  }
}

BspStatus BspUart_Write(BspUartPort port, const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
  if (!BspUart_IsValid(port) || data == NULL || length == 0U) {
    return BSP_INVALID_ARG;
  }

  return BspUart_FromHal(HAL_UART_Transmit(uart_contexts[port].handle, data, length, timeout_ms));
}

bool BspUart_ReadByte(BspUartPort port, uint8_t *byte)
{
  if (!BspUart_IsValid(port) || byte == NULL) {
    return false;
  }

  BspUartContext *context = &uart_contexts[port];
  if (context->head == context->tail) {
    return false;
  }

  *byte = context->rx_buffer[context->tail];
  context->tail = BspUart_NextIndex(context->tail);

  return true;
}

uint16_t BspUart_Available(BspUartPort port)
{
  if (!BspUart_IsValid(port)) {
    return 0U;
  }

  const BspUartContext *context = &uart_contexts[port];
  if (context->head >= context->tail) {
    return (uint16_t)(context->head - context->tail);
  }

  return (uint16_t)(BSP_UART_RX_BUFFER_SIZE - context->tail + context->head);
}

uint32_t BspUart_GetOverflowCount(BspUartPort port)
{
  if (!BspUart_IsValid(port)) {
    return 0U;
  }

  return uart_contexts[port].overflow_count;
}

BspStatus BspUart_GetStats(BspUartPort port, BspUartStats *stats)
{
  if (!BspUart_IsValid(port) || stats == NULL) {
    return BSP_INVALID_ARG;
  }

  const BspUartContext *context = &uart_contexts[port];
  stats->parity_error_count = context->parity_error_count;
  stats->noise_error_count = context->noise_error_count;
  stats->framing_error_count = context->framing_error_count;
  stats->overrun_error_count = context->overrun_error_count;
  stats->rx_restart_failure_count = context->rx_restart_failure_count;
  stats->rx_buffer_overflow_count = context->overflow_count;
  return BSP_OK;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  for (uint32_t i = 0; i < BSP_UART_COUNT; ++i) {
    BspUartContext *context = &uart_contexts[i];
    if (context->handle == huart) {
      if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
        return;
      }
      BspUart_PushByte(context, context->irq_byte);
      (void)BspUart_StartReceive(context);
      return;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  for (uint32_t i = 0; i < BSP_UART_COUNT; ++i) {
    BspUartContext *context = &uart_contexts[i];
    if (context->handle == huart) {
      BspUart_RecordErrors(context, huart->ErrorCode);
      __HAL_UART_CLEAR_PEFLAG(huart);
      if (huart->RxState == HAL_UART_STATE_READY) {
        (void)BspUart_StartReceive(context);
      }
      return;
    }
  }
}
