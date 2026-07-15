#ifndef BSP_UART_H
#define BSP_UART_H

#include "bsp_types.h"

/*
 * USART2/UART4 的非阻塞收发与故障恢复封装。
 *
 * 每个逻辑端口用逐字节接收中断写入单生产者、单消费者环形缓冲区；任务通过 ReadByte
 * 非阻塞消费。发送端复制整帧到固定队列，再由 HAL 中断异步发送。完成中断和任务服务
 * 路径通过三态原子所有权协作，可在完成回调缺失时恢复队列。错误回调记录错误并在中断
 * 内最多尝试一次立即恢复，持续失败再由 BspUart_Service 在任务上下文周期重试，避免
 * 在中断中执行无界循环。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 单端口累计诊断快照。
 * parity/noise/framing/overrun 对应 HAL 接收错误；rx_recovery_* 描述重启尝试结果；
 * rx_buffer_overflow 表示软件环形缓冲区丢字节；tx_queue_full 表示生产速度超过发送能力；
 * tx_start_failure 表示当前策略把 HAL 起发失败帧丢弃；completion_recovery 表示完成
 * 回调与恢复路径竞争或回调缺失。计数均饱和，queued_frame_count 是读取瞬间的队列深度。
 */
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

/*
 * 绑定 ROS→USART2、TTL→UART4，清零两个上下文并启动各自首字节中断接收。
 * 任一路启动失败会中止此前已启动端口的接收并返回对应 BSP 状态；成功后仍需通信任务
 * 周期调用 BspUart_Service。
 */
BspStatus BspUart_Init(void);
/*
 * 在通信任务上下文周期调用。
 * 对每个端口清理待处理错误、尝试重启接收、恢复缺失的发送完成回调，并在发送空闲时
 * 启动下一帧。函数不主动等待外设，不应从中断调用。
 */
void BspUart_Service(void);
/*
 * 复制一帧到指定端口发送队列，并尽可能立即起发。
 * 每个端口只允许一个任务生产者。成功入队后，即使 HAL 暂忙也返回 BSP_OK，由后续
 * Service 重试；参数无效返回 BSP_INVALID_ARG，队满返回 BSP_BUSY；按当前策略丢弃的起发
 * 错误返回 BSP_ERROR 或 BSP_TIMEOUT。这里的丢帧不表示 UART 硬件永久不可恢复。
 */
BspStatus BspUart_WriteAsync(BspUartPort port, const uint8_t *data, uint16_t length);
/*
 * 从接收环形缓冲区非阻塞取出一个字节并推进 tail。
 * 成功返回 true；端口/指针无效或当前无数据返回 false。每个端口只允许一个任务消费者。
 */
bool BspUart_ReadByte(BspUartPort port, uint8_t *byte);
/* 返回当前可读字节数的瞬时快照；端口无效返回 0，不应用作线程同步条件。 */
uint16_t BspUart_Available(BspUartPort port);
/* 返回接收软件环形缓冲区的饱和溢出计数；端口无效返回 0。 */
uint32_t BspUart_GetOverflowCount(BspUartPort port);
/*
 * 用原子读取复制指定端口的完整统计快照。
 * 各字段可能来自相邻中断时刻，因此快照适合趋势诊断而非事务一致性判断。参数无效返回
 * BSP_INVALID_ARG，成功返回 BSP_OK。
 */
BspStatus BspUart_GetStats(BspUartPort port, BspUartStats *stats);

#ifdef __cplusplus
}
#endif

#endif
