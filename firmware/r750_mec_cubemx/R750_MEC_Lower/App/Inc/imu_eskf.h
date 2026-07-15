#ifndef IMU_ESKF_H
#define IMU_ESKF_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 六轴 IMU 姿态误差状态卡尔曼滤波器。
 *
 * 名义状态由单位四元数姿态和三维陀螺零偏组成；六维误差状态按“微小姿态误差、零偏
 * 误差”排列。陀螺用于连续预测，加速度只在模长、方向创新和归一化创新平方检验通过时
 * 作为重力方向观测。没有磁力计或外部航向观测，因此航向及对应协方差不可绝对收敛。
 * 调用者必须输入通过突变检测但未经普通低通的 SI 单位样本。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_ESKF_STATE_SIZE       (6U)
#define IMU_ESKF_COVARIANCE_SIZE  (IMU_ESKF_STATE_SIZE * IMU_ESKF_STATE_SIZE)
#define IMU_ESKF_MEASUREMENT_SIZE (3U)

/*
 * 完整滤波器上下文，由 IMU 任务独占。
 * quaternion 顺序为 [w,x,y,z]，gyro_bias_rad_s 为三轴零偏，covariance 按 6×6 行主序
 * 展开。其余数组是矩阵运算的固定工作区，放入结构体可避免栈峰值和动态分配；外部模块
 * 不应读取或修改这些工作区。accel_update_count 只统计实际接受的重力校正。
 */
typedef struct {
  float quaternion[4];
  float gyro_bias_rad_s[3];
  float covariance[IMU_ESKF_COVARIANCE_SIZE];
  uint32_t accel_update_count;
  bool initialized;
  float work_a[IMU_ESKF_COVARIANCE_SIZE];
  float work_b[IMU_ESKF_COVARIANCE_SIZE];
  float measurement_jacobian[IMU_ESKF_MEASUREMENT_SIZE * IMU_ESKF_STATE_SIZE];
  float covariance_times_h_transpose[IMU_ESKF_STATE_SIZE * IMU_ESKF_MEASUREMENT_SIZE];
  float kalman_gain[IMU_ESKF_STATE_SIZE * IMU_ESKF_MEASUREMENT_SIZE];
  float innovation_covariance[IMU_ESKF_MEASUREMENT_SIZE * IMU_ESKF_MEASUREMENT_SIZE];
  float innovation_covariance_inverse[IMU_ESKF_MEASUREMENT_SIZE * IMU_ESKF_MEASUREMENT_SIZE];
} ImuEskf;

/*
 * 用静止加速度均值和陀螺均值初始化滤波器。
 * acceleration_mps2 用于确定横滚、俯仰，初始航向固定为零；gyro_bias_rad_s 作为初始
 * 零偏。成功时清空工作区并建立初始协方差；参数无效、加速度非有限或模长过小返回 false。
 */
bool ImuEskf_Init(ImuEskf *filter, const float acceleration_mps2[3], const float gyro_bias_rad_s[3]);
/*
 * 执行一次“陀螺预测—可选重力校正”。
 * 输入角速度单位 rad/s、加速度单位 m/s²、delta_time_s 单位秒且必须位于 (0, 0.1]。
 * 返回 true 表示预测后状态仍为有限值；accel_update_used 单独表明本次加速度是否通过门控，
 * vibration_high 表明加速度模长偏离重力超过振动阈值。加速度被拒绝不等于滤波更新失败。
 */
bool ImuEskf_Update(ImuEskf *filter,
                    const float angular_rate_rad_s[3],
                    const float acceleration_mps2[3],
                    float delta_time_s,
                    bool *accel_update_used,
                    bool *vibration_high);
/*
 * 把名义四元数转换为 [横滚, 俯仰, 航向] 弧度数组，仅用于输出和诊断。
 * 函数会限制 asin 输入以抵抗浮点舍入；参数无效或滤波器未初始化时不写输出。
 */
void ImuEskf_GetEulerRad(const ImuEskf *filter, float euler_rad[3]);
/*
 * 当滤波器已初始化且累计接受的加速度校正达到固定门槛时返回 true。
 * 该结果表示倾角估计获得足够重力校正，不表示绝对航向有效。
 */
bool ImuEskf_IsConverged(const ImuEskf *filter);
/* 检查已初始化滤波器的四元数、零偏和全部协方差元素是否为有限值。 */
bool ImuEskf_IsStateFinite(const ImuEskf *filter);

#ifdef __cplusplus
}
#endif

#endif
