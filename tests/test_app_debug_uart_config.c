#include "app_debug_uart_config.h"

#include <assert.h>

/* 锁定当前 M2 板测镜像：ASCII 调试链必须经无线 DAPLink 对应的 USART1。 */
int main(void)
{
  assert(ROBOT_CONFIG_DEBUG_UART_PORT == BSP_UART_TTL);
  return 0;
}
