#include "app_safety_policy.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * 安全策略保持为纯状态机，不直接操作电机或 RTOS 对象。
 * 每次调用代表一个固定安全检查窗；调用者负责提供启动是否就绪、该窗是否收齐全部
 * 关键任务心跳、IMU 实时运动可用性以及其他模块已锁存的硬故障。分级逻辑因此可在
 * 主机测试中独立验证。
 */

void AppSafetyPolicy_Init(AppSafetyPolicy *policy)
{
  if (policy != NULL) {
    memset(policy, 0, sizeof(*policy));
    policy->motion_inhibited = true;
  }
}

bool AppSafetyPolicy_Update(
  AppSafetyPolicy *policy,
  bool runtime_ready,
  bool critical_tasks_healthy,
  bool imu_motion_usable,
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
    /* 只统计连续缺失窗；任一完整健康窗都会清零，防止零散调度抖动永久累积成硬故障。 */
    if (policy->critical_heartbeat_miss_count < UINT32_MAX) {
      policy->critical_heartbeat_miss_count++;
    }
  } else {
    policy->critical_heartbeat_miss_count = 0U;
  }

  const bool critical_fault =
    policy->critical_heartbeat_miss_count >= critical_miss_limit;
  /*
   * external_fault_latched（例如通信 ESTOP）无需等待心跳门槛。急停动作只在 fault_latched
   * 从 false 跳为 true 的首次更新发出；锁存后的每个周期仍保持 motion_inhibited。
   */
  if (!policy->fault_latched && (external_fault_latched || critical_fault)) {
    policy->fault_latched = true;
    output->emergency_coast_request = true;
  }

  /*
   * IMU 数据故障不等同于 IMU 任务失联：前者由统一实时可用判据触发普通可恢复空转，
   * 后者会缺失 IMU 心跳并在连续达到门槛后升级硬锁存。启动未就绪也保持普通禁止，但
   * 不计入心跳失联。这样退避期间持续心跳不会误触发硬急停，陈旧数据仍不能运动。
   */
  policy->motion_inhibited =
    policy->fault_latched || !runtime_ready || !critical_tasks_healthy || !imu_motion_usable;
  output->normal_coast_request =
    policy->motion_inhibited && !policy->fault_latched;
  return true;
}
