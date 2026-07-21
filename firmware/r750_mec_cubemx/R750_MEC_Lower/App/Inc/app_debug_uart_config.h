#ifndef APP_DEBUG_UART_CONFIG_H
#define APP_DEBUG_UART_CONFIG_H

#include "bsp_types.h"

/*
 * 调试命令与遥测的唯一端口选择。
 * M2 板测由无线 DAPLink 虚拟串口对应的 USART1（PA9/PA10）承载大写 ASCII 调试
 * 协议，因此选择 TTL 逻辑端口。USART2 保留给树莓派正式 X-Protocol 链路；commTask
 * 不得按物理串口写死收发或统计，也不得在同一端口自动探测两种协议。
 */
#define ROBOT_CONFIG_DEBUG_UART_PORT (BSP_UART_TTL)

#endif
