#ifndef BSP_UART_H
#define BSP_UART_H

#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t parity_error_count;
  uint32_t noise_error_count;
  uint32_t framing_error_count;
  uint32_t overrun_error_count;
  uint32_t rx_recovery_attempt_count;
  uint32_t rx_recovery_success_count;
  uint32_t rx_restart_failure_count;
  uint32_t rx_buffer_overflow_count;
} BspUartStats;

BspStatus BspUart_Init(void);
void BspUart_Service(void);
BspStatus BspUart_Write(BspUartPort port, const uint8_t *data, uint16_t length, uint32_t timeout_ms);
bool BspUart_ReadByte(BspUartPort port, uint8_t *byte);
uint16_t BspUart_Available(BspUartPort port);
uint32_t BspUart_GetOverflowCount(BspUartPort port);
BspStatus BspUart_GetStats(BspUartPort port, BspUartStats *stats);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
