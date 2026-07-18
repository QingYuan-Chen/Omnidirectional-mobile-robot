#ifndef APP_DEBUG_UART_CONFIG_H
#define APP_DEBUG_UART_CONFIG_H

#include "bsp_types.h"

/*
 * 调试命令与遥测的唯一端口选择。
 * M2 板测阶段由板载 Type-C 对应的 USART2 承载大写 ASCII 调试协议，因此选择 ROS
 * 逻辑端口。commTask 不得再按物理串口写死收发或统计。M5 启用正式 X-Protocol 前，
 * 必须先把调试端口迁回 TTL/UART4，禁止在 USART2 上同时自动探测 ASCII 与
 * X-Protocol。
 */
#define ROBOT_CONFIG_DEBUG_UART_PORT (BSP_UART_ROS)

#endif
