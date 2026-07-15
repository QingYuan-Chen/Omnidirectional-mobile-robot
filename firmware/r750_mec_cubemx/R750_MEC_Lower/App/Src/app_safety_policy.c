#include "app_safety_policy.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/* 安全策略是纯状态机，不直接操作硬件，便于独立验证故障升级规则。 */

void AppSafetyPolicy_Init(AppSafetyPolicy *policy)
{
  if (policy != NULL) {
    memset(policy, 0, sizeof(*policy));
  }
}

bool AppSafetyPolicy_Update(
  AppSafetyPolicy *policy,
  bool critical_tasks_healthy,
  bool imu_healthy,
  bool external_fault_latched,
  uint32_t critical_miss_limit,
  AppSafetyPolicyOutput *output)
{
  if (policy == NULL || output == NULL || critical_miss_limit == 0U) {
    return false;
  }

  output->normal_coast_request = false;
  output->emergency_coast_request = false;
  if (!critical_tasks_healthy) {
    if (policy->critical_heartbeat_miss_count < UINT32_MAX) {
      policy->critical_heartbeat_miss_count++;
    }
  } else {
    policy->critical_heartbeat_miss_count = 0U;
  }

  const bool critical_fault =
    policy->critical_heartbeat_miss_count >= critical_miss_limit;
  /* 达到连续缺失门槛或已有外部硬故障才永久锁存；单次缺失只做可恢复禁止。 */
  if (!policy->fault_latched && (external_fault_latched || critical_fault)) {
    policy->fault_latched = true;
    output->emergency_coast_request = true;
  }

  /* IMU 非健康不会单独触发破坏性急停，但会持续禁止运动直到稳定恢复。 */
  policy->motion_inhibited =
    policy->fault_latched || !critical_tasks_healthy || !imu_healthy;
  output->normal_coast_request =
    policy->motion_inhibited && !policy->fault_latched;
  return true;
}
