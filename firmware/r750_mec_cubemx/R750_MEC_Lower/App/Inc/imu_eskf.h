#ifndef IMU_ESKF_H
#define IMU_ESKF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_ESKF_STATE_SIZE       (6U)
#define IMU_ESKF_COVARIANCE_SIZE  (IMU_ESKF_STATE_SIZE * IMU_ESKF_STATE_SIZE)
#define IMU_ESKF_MEASUREMENT_SIZE (3U)

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

bool ImuEskf_Init(ImuEskf *filter, const float acceleration_mps2[3], const float gyro_bias_rad_s[3]);
bool ImuEskf_Update(ImuEskf *filter,
                    const float angular_rate_rad_s[3],
                    const float acceleration_mps2[3],
                    float delta_time_s,
                    bool *accel_update_used,
                    bool *vibration_high);
void ImuEskf_GetEulerRad(const ImuEskf *filter, float euler_rad[3]);
bool ImuEskf_IsConverged(const ImuEskf *filter);
bool ImuEskf_IsStateFinite(const ImuEskf *filter);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ESKF_H */
