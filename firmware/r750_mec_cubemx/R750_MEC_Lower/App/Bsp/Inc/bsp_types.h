#ifndef BSP_TYPES_H
#define BSP_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* BSP 公共类型隔离 HAL 细节，使应用层使用统一状态码和逻辑设备编号。 */

/* 板级接口统一返回值。 */
typedef enum {
  BSP_OK = 0,
  BSP_ERROR = 1,
  BSP_TIMEOUT = 2,
  BSP_INVALID_ARG = 3,
  BSP_BUSY = 4
} BspStatus;

/* 四路电机及编码器的固定逻辑顺序。 */
typedef enum {
  BSP_MOTOR_MA = 0,
  BSP_MOTOR_MB,
  BSP_MOTOR_MC,
  BSP_MOTOR_MD,
  BSP_MOTOR_COUNT
} BspMotorId;

/* ROS 口连接树莓派，TTL口当前用于临时试验命令和遥测。 */
typedef enum {
  BSP_UART_ROS = 0,
  BSP_UART_TTL,
  BSP_UART_COUNT
} BspUartPort;

#endif
