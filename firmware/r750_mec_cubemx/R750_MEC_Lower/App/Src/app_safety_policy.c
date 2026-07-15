#include "app_safety_policy.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

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
  if (!policy->fault_latched && (external_fault_latched || critical_fault)) {
    policy->fault_latched = true;
    output->emergency_coast_request = true;
  }

  policy->motion_inhibited =
    policy->fault_latched || !critical_tasks_healthy || !imu_healthy;
  output->normal_coast_request =
    policy->motion_inhibited && !policy->fault_latched;
  return true;
}
