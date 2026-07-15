#ifndef IMU_ESKF_H
#define IMU_ESKF_H

#include <stdbool.h>
#include <stdint.h>

/* 六维误差状态由三维姿态误差和三维陀螺零偏组成，名义姿态使用四元数。 */

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_ESKF_STATE_SIZE       (6U)
#define IMU_ESKF_COVARIANCE_SIZE  (IMU_ESKF_STATE_SIZE * IMU_ESKF_STATE_SIZE)
#define IMU_ESKF_MEASUREMENT_SIZE (3U)

/* 滤波器状态包含名义状态、协方差以及静态工作区，不进行动态内存分配。 */
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

/* 用静止重力方向初始化横滚、俯仰和陀螺零偏；绝对航向不可观。 */
bool ImuEskf_Init(ImuEskf *filter, const float acceleration_mps2[3], const float gyro_bias_rad_s[3]);
/* 先用角速度预测，再按门限决定是否使用加速度重力观测校正。 */
bool ImuEskf_Update(ImuEskf *filter,
                    const float angular_rate_rad_s[3],
                    const float acceleration_mps2[3],
                    float delta_time_s,
                    bool *accel_update_used,
                    bool *vibration_high);
/* 将名义四元数转换为弧度制欧拉角，仅用于输出。 */
void ImuEskf_GetEulerRad(const ImuEskf *filter, float euler_rad[3]);
/* 加速度校正成功次数达到稳定门槛后报告收敛。 */
bool ImuEskf_IsConverged(const ImuEskf *filter);
/* 检查名义状态和协方差是否全部为有限值。 */
bool ImuEskf_IsStateFinite(const ImuEskf *filter);

#ifdef __cplusplus
}
#endif

#endif
