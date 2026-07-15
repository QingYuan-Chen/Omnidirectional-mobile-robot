#ifndef APP_ENCODER_ACCUMULATOR_H
#define APP_ENCODER_ACCUMULATOR_H

#include "bsp_types.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 编码器回绕解算与长期累计模块。
 *
 * 定时器只提供 16 位循环计数，本模块在固定控制节拍中把相邻原始值解释为有符号增量，
 * 并累加到 64 位总计数。算法要求任意两个相邻快照之间的真实变化量不超过 32767 个计数；
 * 超过半量程时方向将产生歧义，因此控制任务漏周期诊断必须与本模块结果结合判断。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 四路编码器状态，数组顺序与 BspMotorId 完全一致。
 * total_count 在达到 int64_t 上下限后保持饱和，避免长期运行回绕破坏里程计连续性。
 */
typedef struct {
  uint16_t previous_raw[BSP_MOTOR_COUNT];
  int64_t total_count[BSP_MOTOR_COUNT];
} AppEncoderAccumulator;

/*
 * 用当前四路原始计数建立差分基线，并把累计值清零。
 * 初始化本身不输出增量，因此不会把上电前定时器初值误认为运动；参数无效返回 false。
 */
bool AppEncoderAccumulator_Init(AppEncoderAccumulator *state, const uint16_t raw_count[BSP_MOTOR_COUNT]);
/*
 * 使用一个同步快照更新四路增量与累计计数。
 * 差分先在模 2^16 无符号域完成，再映射到 [-32768, 32767]；成功时同时更新内部基线、
 * delta_count 和 total_count。任一指针为空时返回 false 且不更新任何一路。
 */
bool AppEncoderAccumulator_Update(
  AppEncoderAccumulator *state,
  const uint16_t raw_count[BSP_MOTOR_COUNT],
  int16_t delta_count[BSP_MOTOR_COUNT],
  int64_t total_count[BSP_MOTOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
