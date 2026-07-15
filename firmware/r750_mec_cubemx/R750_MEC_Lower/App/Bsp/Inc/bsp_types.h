#ifndef BSP_TYPES_H
#define BSP_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  BSP_OK = 0,
  BSP_ERROR = 1,
  BSP_TIMEOUT = 2,
  BSP_INVALID_ARG = 3,
  BSP_BUSY = 4
} BspStatus;

typedef enum {
  BSP_MOTOR_MA = 0,
  BSP_MOTOR_MB,
  BSP_MOTOR_MC,
  BSP_MOTOR_MD,
  BSP_MOTOR_COUNT
} BspMotorId;

typedef enum {
  BSP_UART_ROS = 0,
  BSP_UART_TTL,
  BSP_UART_COUNT
} BspUartPort;

#endif
