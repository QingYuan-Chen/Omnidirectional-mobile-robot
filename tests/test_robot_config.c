#include "robot_config.h"

#include <assert.h>

/*
 * 锁定 MA 三圈板测关闭的编码器倍率。后续修改编码器型号、减速比或计数模式时，必须先
 * 更新板测证据，再同步修改配置和本测试，禁止把 1024 PPR 误当成四倍频后的计数。
 */
int main(void)
{
  assert(ROBOT_CONFIG_ENCODER_PULSES_PER_MOTOR_REV == 1024U);
  assert(ROBOT_CONFIG_ENCODER_QUADRATURE_FACTOR == 4U);
  assert(ROBOT_CONFIG_MOTOR_REDUCTION_RATIO == 30U);
  assert(ROBOT_CONFIG_ENCODER_COUNTS_PER_WHEEL_REV == 122880U);
  return 0;
}
