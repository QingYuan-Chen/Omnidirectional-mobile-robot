#ifndef BSP_TYPES_H
#define BSP_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 板级支持包公共类型。
 *
 * BSP 对上层隐藏 HAL 句柄、GPIO 端口和定时器通道，应用代码只使用统一状态码和逻辑
 * 设备编号。逻辑枚举的顺序同时用于静态映射数组，新增设备或调整顺序时必须同步检查
 * 每个 BSP 映射表以及运行快照数组。
 */

/*
 * 板级接口统一返回值。
 * OK 表示本次操作完成；BUSY 表示暂时未就绪、调用者可稍后重试；TIMEOUT 表示等待超过
 * 明确上限；INVALID_ARG 表示调用契约错误；ERROR 表示其余硬件或状态异常。
 */
typedef enum {
  BSP_OK = 0,
  BSP_ERROR = 1,
  BSP_TIMEOUT = 2,
  BSP_INVALID_ARG = 3,
  BSP_BUSY = 4
} BspStatus;

/* 四路电机及其编码器的固定逻辑顺序：MA、MB、MC、MD。COUNT 仅用于数组边界。 */
typedef enum {
  BSP_MOTOR_MA = 0,
  BSP_MOTOR_MB,
  BSP_MOTOR_MC,
  BSP_MOTOR_MD,
  BSP_MOTOR_COUNT
} BspMotorId;

/*
 * 逻辑串口角色。
 * ROS 对应 USART2，最终用于 STM32 与其直接上位机树莓派之间的正式 X-Protocol 链路；
 * TTL 对应 USART1（PA9/PA10），由无线 DAPLink 的虚拟串口承载人类调试命令与遥测；
 * ROS 对应 USART2，保留给树莓派正式 X-Protocol 链路。UART4 是旧调试硬件通道，当前
 * 不再绑定应用逻辑端口。ASCII 调试协议与 X-Protocol 分属不同物理串口，不得交换角色。
 */
typedef enum {
  BSP_UART_ROS = 0,
  BSP_UART_TTL,
  BSP_UART_COUNT
} BspUartPort;

#endif
