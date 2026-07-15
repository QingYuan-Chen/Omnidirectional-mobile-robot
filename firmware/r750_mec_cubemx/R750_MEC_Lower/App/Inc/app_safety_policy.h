#ifndef APP_SAFETY_POLICY_H
#define APP_SAFETY_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t critical_heartbeat_miss_count;
  bool motion_inhibited;
  bool fault_latched;
} AppSafetyPolicy;

typedef struct {
  bool normal_coast_request;
  bool emergency_coast_request;
} AppSafetyPolicyOutput;

void AppSafetyPolicy_Init(AppSafetyPolicy *policy);
bool AppSafetyPolicy_Update(
  AppSafetyPolicy *policy,
  bool critical_tasks_healthy,
  bool imu_healthy,
  bool external_fault_latched,
  uint32_t critical_miss_limit,
  AppSafetyPolicyOutput *output);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_POLICY_H */
