#include "app_debug_uart_config.h"

#include <assert.h>

/* 锁定当前 M2 板测镜像：ASCII 调试链必须经板载 Type-C 对应的 USART2。 */
int main(void)
{
  assert(ROBOT_CONFIG_DEBUG_UART_PORT == BSP_UART_ROS);
  return 0;
}
