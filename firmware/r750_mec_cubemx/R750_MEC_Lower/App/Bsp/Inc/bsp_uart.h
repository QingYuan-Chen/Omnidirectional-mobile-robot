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
  uint32_t tx_enqueued_frame_count;
  uint32_t tx_completed_frame_count;
  uint32_t tx_queue_full_count;
  uint32_t tx_start_failure_count;
  uint32_t tx_completion_recovery_count;
  uint32_t tx_queued_frame_count;
} BspUartStats;

BspStatus BspUart_Init(void);
void BspUart_Service(void);
/* 每个端口只允许一个任务作为发送生产者，完成中断和任务恢复路径共同推进队列。 */
BspStatus BspUart_WriteAsync(BspUartPort port, const uint8_t *data, uint16_t length);
bool BspUart_ReadByte(BspUartPort port, uint8_t *byte);
uint16_t BspUart_Available(BspUartPort port);
uint32_t BspUart_GetOverflowCount(BspUartPort port);
BspStatus BspUart_GetStats(BspUartPort port, BspUartStats *stats);

#ifdef __cplusplus
}
#endif

#endif
