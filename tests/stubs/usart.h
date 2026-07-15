#ifndef TEST_STUB_USART_H
#define TEST_STUB_USART_H

#include <stdint.h>

typedef enum {
  HAL_OK = 0,
  HAL_ERROR,
  HAL_BUSY,
  HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef enum {
  HAL_UART_STATE_RESET = 0,
  HAL_UART_STATE_READY,
  HAL_UART_STATE_BUSY_TX,
  HAL_UART_STATE_BUSY_RX
} HAL_UART_StateTypeDef;

typedef struct {
  volatile HAL_UART_StateTypeDef gState;
  volatile HAL_UART_StateTypeDef RxState;
  volatile uint32_t ErrorCode;
} UART_HandleTypeDef;

#define HAL_UART_ERROR_NONE (0U)
#define HAL_UART_ERROR_PE   (1U << 0U)
#define HAL_UART_ERROR_NE   (1U << 1U)
#define HAL_UART_ERROR_FE   (1U << 2U)
#define HAL_UART_ERROR_ORE  (1U << 3U)

#define __HAL_UART_CLEAR_PEFLAG(handle) ((void)(handle))

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

HAL_StatusTypeDef HAL_UART_Receive_IT(
  UART_HandleTypeDef *handle,
  uint8_t *data,
  uint16_t length);
HAL_StatusTypeDef HAL_UART_Transmit_IT(
  UART_HandleTypeDef *handle,
  const uint8_t *data,
  uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *handle);

#endif
