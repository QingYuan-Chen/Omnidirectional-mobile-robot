#ifndef BSP_UART_H
#define BSP_UART_H

#include "bsp_types.h"

/* UART BSP 统一管理逐字节中断接收、非阻塞帧发送、错误统计和恢复。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 所有计数均为饱和累计值，可直接复制到运行快照。 */
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

/* 绑定逻辑端口与 HAL 句柄并启动逐字节接收。 */
BspStatus BspUart_Init(void);
/* 任务周期服务接收恢复、发送起发和缺失完成回调恢复。 */
void BspUart_Service(void);
/* 每个端口只允许一个任务作为发送生产者，完成中断和任务恢复路径共同推进队列。 */
BspStatus BspUart_WriteAsync(BspUartPort port, const uint8_t *data, uint16_t length);
/* 从接收环形缓冲区非阻塞读取一个字节。 */
bool BspUart_ReadByte(BspUartPort port, uint8_t *byte);
/* 返回当前可读字节数的瞬时快照。 */
uint16_t BspUart_Available(BspUartPort port);
/* 返回接收环形缓冲区溢出计数。 */
uint32_t BspUart_GetOverflowCount(BspUartPort port);
/* 复制指定端口的完整诊断统计。 */
BspStatus BspUart_GetStats(BspUartPort port, BspUartStats *stats);

#ifdef __cplusplus
}
#endif

#endif
