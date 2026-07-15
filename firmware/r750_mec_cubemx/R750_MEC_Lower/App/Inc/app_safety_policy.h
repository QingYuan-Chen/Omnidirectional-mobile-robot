#ifndef APP_SAFETY_POLICY_H
#define APP_SAFETY_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* 可恢复运动禁止与复位恢复型故障锁存分级处理，严重故障优先级最高。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 保存连续心跳缺失计数以及当前禁止、锁存状态。 */
typedef struct {
  uint32_t critical_heartbeat_miss_count;
  bool motion_inhibited;
  bool fault_latched;
} AppSafetyPolicy;

/* 输出区分可恢复普通空转和破坏性紧急空转。 */
typedef struct {
  bool normal_coast_request;
  bool emergency_coast_request;
} AppSafetyPolicyOutput;

/* 清零安全策略状态；首次健康检查前仍由运行快照保持运动禁止。 */
void AppSafetyPolicy_Init(AppSafetyPolicy *policy);
/* 根据关键心跳、IMU健康和外部故障更新分级安全决策。 */
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

#endif
