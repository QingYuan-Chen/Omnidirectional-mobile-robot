#ifndef BSP_UART_TX_QUEUE_H
#define BSP_UART_TX_QUEUE_H

#include "robot_config.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * UART 异步发送使用的单生产者、单消费者定长帧环形队列。
 *
 * 任务生产者把完整帧复制进队列，发送完成中断或任务恢复路径作为同一逻辑消费者推进
 * 队尾。head/tail 使用 C11 原子变量和 acquire/release 顺序发布所有权，队列不使用锁、
 * 不分配内存，也不支持多个并发生产者。额外一个物理槽用于区分队空和队满。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_UART_TX_QUEUE_STORAGE_DEPTH (ROBOT_CONFIG_UART_TX_QUEUE_DEPTH + 1U)

/*
 * 单个发送槽位。length 是有效字节数，data 容量为编译期最大帧长。
 * 队列拥有数据副本，成功入队后调用者可立即修改或复用原缓冲区。
 */
typedef struct {
  uint16_t length;
  uint8_t data[ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH];
} BspUartTxFrame;

/* 队列存储与原子索引；有效可排队帧数仍为 ROBOT_CONFIG_UART_TX_QUEUE_DEPTH。 */
typedef struct {
  BspUartTxFrame frames[BSP_UART_TX_QUEUE_STORAGE_DEPTH];
  _Atomic uint32_t head;
  _Atomic uint32_t tail;
} BspUartTxQueue;

/* 清空全部槽位并把 head/tail 初始化为零；参数为空时不执行操作。 */
void BspUartTxQueue_Init(BspUartTxQueue *queue);
/*
 * 复制一帧并发布新 head。
 * length 必须位于 1 到最大帧长；队满或参数无效返回 false。函数先写槽位内容，再用
 * release 存储发布 head，消费者不会看到半写入帧。
 */
bool BspUartTxQueue_Enqueue(
  BspUartTxQueue *queue,
  const uint8_t *data,
  uint16_t length);
/*
 * 查看当前队尾帧但不弹出。
 * 成功返回队列内部只读数据指针和长度；该指针只在对应帧尚未 Pop 时有效。队空或参数
 * 无效返回 false。
 */
bool BspUartTxQueue_Peek(
  const BspUartTxQueue *queue,
  const uint8_t **data,
  uint16_t *length);
/*
 * 弹出当前队尾帧。只有取得 UART 发送完成所有权的路径可以调用；队空或参数为空返回
 * false。函数不清除槽位数据，只用 release 存储推进 tail。
 */
bool BspUartTxQueue_Pop(BspUartTxQueue *queue);
/* 返回当前排队帧数的瞬时快照；并发生产/消费时数值仅用于诊断，不用于同步。 */
uint32_t BspUartTxQueue_Count(const BspUartTxQueue *queue);

#ifdef __cplusplus
}
#endif

#endif
