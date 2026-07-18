#ifndef APP_MOTION_GATE_H
#define APP_MOTION_GATE_H

#include "app_comm_protocol.h"
#include "app_motor_open_loop.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 整机运动许可和已排队命令失效的纯状态模块。
 *
 * 许可由任务编排层提供的启动就绪、全局禁止、硬故障和实时 IMU 可用性共同决定。本模块
 * 不读取全局快照、不调用 RTOS 或硬件，只维护许可代际并把普通通信命令转换成带代际的
 * 电机请求。许可从可用转为不可用时代际递增，使故障前排队、故障后才被控制任务取出的
 * 旧请求无法在系统恢复后执行。
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t generation;
  bool available;
} AppMotionGate;

typedef struct {
  AppMotorOpenLoopRequest request;
  uint32_t gate_generation;
} AppMotionGatedRequest;

typedef enum {
  APP_MOTION_PREPARE_INVALID = 0,
  APP_MOTION_PREPARE_NOT_MOTION_COMMAND,
  APP_MOTION_PREPARE_GATE_REJECTED,
  APP_MOTION_PREPARE_READY
} AppMotionPrepareResult;

/* 初始化为禁止运动、代际零；初始化不代表运行时已经通过任何安全门。 */
void AppMotionGate_Init(AppMotionGate *gate);
/*
 * 计算当前整机运动许可。四项条件必须全部满足；参数本身不含锁存逻辑，调用者应传入
 * 已经锁存的 fault_latched 和单向 runtime_ready。
 */
bool AppMotionGate_ComputeAvailable(
  bool runtime_ready,
  bool motion_inhibited,
  bool fault_latched,
  bool imu_motion_usable);
/*
 * 更新许可状态。只有 available 的下降沿递增代际；重复禁止和恢复可用都不递增。
 * 若下降沿发生时 generation 已到 UINT32_MAX，函数仍强制 available=false 但返回 false，
 * 调用者必须把不可再区分新旧请求的情况升级为不可恢复系统故障。
 */
bool AppMotionGate_Update(AppMotionGate *gate, bool available);
/*
 * 把 ARM/PWM/STOP 转换为带当前许可代际的电机请求。ESTOP/STATUS 返回
 * NOT_MOTION_COMMAND，由通信任务各自处理；许可关闭时普通命令返回 GATE_REJECTED，不应
 * 入队或提交协议序号。
 */
AppMotionPrepareResult AppMotionGate_PrepareCommand(
  const AppMotionGate *gate,
  const AppCommCommand *command,
  AppMotionGatedRequest *request);
/*
 * 控制任务执行前的最终队列代际检查。只有当前仍允许运动且请求代际等于当前代际时返回
 * true；故障期间被失效的旧请求不会因随后恢复可用而重新变成有效。
 */
bool AppMotionGate_IsRequestCurrent(
  const AppMotionGate *gate,
  const AppMotionGatedRequest *request);

#ifdef __cplusplus
}
#endif

#endif
