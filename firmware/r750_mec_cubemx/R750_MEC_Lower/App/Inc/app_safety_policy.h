#ifndef APP_SAFETY_POLICY_H
#define APP_SAFETY_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 整机安全分级的纯状态机。
 *
 * 安全策略把“启动尚未就绪、本检查窗心跳不全或 IMU 暂时不可用”处理为可恢复运动禁止，
 * 把“关键任务连续多窗失联或外部硬故障”升级为本次上电不可解除的故障锁存。模块只给出
 * 制动策略，不直接操作硬件，便于主机测试验证升级门槛；任务编排层负责提交最终输出。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 安全策略状态。
 * critical_heartbeat_miss_count 只统计连续不健康检查窗，健康一次即清零；fault_latched
 * 一旦置位不会由后续健康输入清除；motion_inhibited 是当前周期综合判断结果。
 */
typedef struct {
  uint32_t critical_heartbeat_miss_count;
  bool motion_inhibited;
  bool fault_latched;
} AppSafetyPolicy;

/*
 * 本次更新需要执行的安全动作。
 * normal_coast_request 表示普通空转并允许健康恢复后重新初始化控制状态；
 * emergency_coast_request 只在首次进入硬锁存时发出，要求关闭 PWM 基础设施。
 */
typedef struct {
  bool normal_coast_request;
  bool emergency_coast_request;
} AppSafetyPolicyOutput;

/*
 * 清零策略内部计数和锁存，并把 motion_inhibited 初始化为 true。只有后续更新同时确认
 * runtime_ready、关键心跳和 IMU 可用后才允许解除。
 */
void AppSafetyPolicy_Init(AppSafetyPolicy *policy);
/*
 * 按一个安全检查窗更新故障等级。
 * runtime_ready 是本次上电已通过启动门的单向锁；critical_tasks_healthy 表示该窗全部
 * 关键任务都到达过心跳；imu_motion_usable 使用统一实时 IMU 判据。external_fault_latched
 * 可把通信急停等外部硬故障并入锁存。miss_limit 必须大于零。
 */
bool AppSafetyPolicy_Update(
  AppSafetyPolicy *policy,
  bool runtime_ready,
  bool critical_tasks_healthy,
  bool imu_motion_usable,
  bool external_fault_latched,
  uint32_t critical_miss_limit,
  AppSafetyPolicyOutput *output);

#ifdef __cplusplus
}
#endif

#endif
