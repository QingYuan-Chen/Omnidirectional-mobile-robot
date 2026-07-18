#include "app_safety_policy.h"

#include <assert.h>
#include <limits.h>
#include <stddef.h>

static void Update(
  AppSafetyPolicy *policy,
  bool runtime_ready,
  bool critical_tasks_healthy,
  bool imu_motion_usable,
  bool external_fault_latched,
  uint32_t miss_limit,
  AppSafetyPolicyOutput *output)
{
  assert(AppSafetyPolicy_Update(
    policy,
    runtime_ready,
    critical_tasks_healthy,
    imu_motion_usable,
    external_fault_latched,
    miss_limit,
    output));
}

static void TestRecoverableInhibit(void)
{
  AppSafetyPolicy policy;
  AppSafetyPolicyOutput output;
  AppSafetyPolicy_Init(&policy);
  assert(policy.motion_inhibited);
  Update(&policy, false, true, true, false, 3U, &output);
  assert(policy.motion_inhibited);
  assert(output.normal_coast_request);
  assert(policy.critical_heartbeat_miss_count == 0U);

  Update(&policy, true, true, true, false, 3U, &output);
  assert(!policy.motion_inhibited);
  assert(!policy.fault_latched);
  assert(!output.normal_coast_request);

  Update(&policy, true, false, true, false, 3U, &output);
  assert(policy.critical_heartbeat_miss_count == 1U);
  assert(policy.motion_inhibited);
  assert(!policy.fault_latched);
  assert(!output.emergency_coast_request);
  assert(output.normal_coast_request);

  Update(&policy, true, true, true, false, 3U, &output);
  assert(policy.critical_heartbeat_miss_count == 0U);
  assert(!policy.motion_inhibited);
  assert(!output.normal_coast_request);

  Update(&policy, true, true, false, false, 3U, &output);
  assert(policy.critical_heartbeat_miss_count == 0U);
  assert(policy.motion_inhibited);
  assert(!policy.fault_latched);
  assert(!output.emergency_coast_request);
  assert(output.normal_coast_request);
  Update(&policy, true, true, true, false, 3U, &output);
  assert(!policy.motion_inhibited);
  assert(!output.normal_coast_request);
}

static void TestCriticalFaultLatches(void)
{
  AppSafetyPolicy policy;
  AppSafetyPolicyOutput output;
  AppSafetyPolicy_Init(&policy);
  Update(&policy, true, false, true, false, 3U, &output);
  assert(!output.emergency_coast_request);
  Update(&policy, true, false, true, false, 3U, &output);
  assert(!output.emergency_coast_request);
  Update(&policy, true, false, true, false, 3U, &output);
  assert(policy.critical_heartbeat_miss_count == 3U);
  assert(policy.fault_latched);
  assert(policy.motion_inhibited);
  assert(output.emergency_coast_request);
  assert(!output.normal_coast_request);

  Update(&policy, true, true, true, false, 3U, &output);
  assert(policy.critical_heartbeat_miss_count == 0U);
  assert(policy.fault_latched);
  assert(policy.motion_inhibited);
  assert(!output.emergency_coast_request);
  assert(!output.normal_coast_request);
}

static void TestExternalFaultAndSaturation(void)
{
  AppSafetyPolicy policy;
  AppSafetyPolicyOutput output;
  AppSafetyPolicy_Init(&policy);
  Update(&policy, true, true, true, true, 3U, &output);
  assert(policy.fault_latched);
  assert(output.emergency_coast_request);
  assert(!output.normal_coast_request);
  Update(&policy, true, true, true, false, 3U, &output);
  assert(policy.fault_latched);
  assert(!output.emergency_coast_request);

  AppSafetyPolicy_Init(&policy);
  policy.critical_heartbeat_miss_count = UINT32_MAX;
  Update(&policy, true, false, true, false, UINT32_MAX, &output);
  assert(policy.critical_heartbeat_miss_count == UINT32_MAX);
  assert(policy.fault_latched);
  assert(output.emergency_coast_request);
}

static void TestArguments(void)
{
  AppSafetyPolicy policy;
  AppSafetyPolicyOutput output;
  AppSafetyPolicy_Init(NULL);
  AppSafetyPolicy_Init(&policy);
  assert(!AppSafetyPolicy_Update(NULL, true, true, true, false, 3U, &output));
  assert(!AppSafetyPolicy_Update(&policy, true, true, true, false, 0U, &output));
  assert(!AppSafetyPolicy_Update(&policy, true, true, true, false, 3U, NULL));
}

int main(void)
{
  TestRecoverableInhibit();
  TestCriticalFaultLatches();
  TestExternalFaultAndSaturation();
  TestArguments();
  return 0;
}
