#include "imu_eskf.h"

#include "robot_config.h"

#include <math.h>
#include <string.h>

/*
 * 姿态 ESKF 采用“名义四元数 + 陀螺零偏”和六维小误差状态。
 *
 * 每个样本先用未经过普通低通的角速度传播名义姿态与协方差，再把通过多级门控的归一化
 * 加速度当作重力方向观测。校正量注入名义状态后，用约瑟夫形式和误差重置雅可比更新
 * 协方差。所有矩阵按行主序存放在结构体固定工作区，无动态内存，也不在栈上创建大矩阵。
 */

#define IMU_ESKF_INDEX(row, column, width) ((row) * (width) + (column))
#define IMU_ESKF_GYRO_NOISE_VARIANCE       (4.0e-6f)
#define IMU_ESKF_BIAS_NOISE_VARIANCE       (1.0e-7f)
#define IMU_ESKF_ACCEL_NOISE_VARIANCE      (2.5e-3f)
#define IMU_ESKF_ACCEL_GATE_MPS2           (1.5f)
#define IMU_ESKF_VIBRATION_THRESHOLD_MPS2  (0.75f)
#define IMU_ESKF_INNOVATION_GATE           (0.70f)
#define IMU_ESKF_NIS_GATE                  (9.21f)
#define IMU_ESKF_MAX_CORRECTION_RAD         (0.20f)
#define IMU_ESKF_MIN_COVARIANCE             (1.0e-9f)
#define IMU_ESKF_MAX_COVARIANCE             (100.0f)
#define IMU_ESKF_CONVERGENCE_UPDATES        (100U)

/*
 * 噪声方差和门限当前是软件候选值，后续应结合静止噪声、振动工况与板测创新统计复核。
 * ACCEL_GATE 先按物理模长拒绝明显线加速度；INNOVATION_GATE 按重力方向差拒绝大姿态矛盾；
 * NIS_GATE 再结合当前协方差做统计一致性检查。三层门控都通过才允许校正。
 */

static float ImuEskf_VectorNorm(const float vector[3])
{
  return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

static void ImuEskf_QuaternionNormalize(float quaternion[4])
{
  /* 每次传播与误差注入后归一化；范数过小时回退单位姿态，避免除零并保持有限输出。 */
  const float norm = sqrtf(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                           quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
  if (norm > 1.0e-8f) {
    const float inverse_norm = 1.0f / norm;
    for (uint32_t i = 0U; i < 4U; ++i) {
      quaternion[i] *= inverse_norm;
    }
  } else {
    quaternion[0] = 1.0f;
    quaternion[1] = 0.0f;
    quaternion[2] = 0.0f;
    quaternion[3] = 0.0f;
  }
}

static void ImuEskf_QuaternionMultiply(const float left[4], const float right[4], float result[4])
{
  /* Hamilton 乘法，调用顺序为当前名义姿态右乘本周期增量四元数。 */
  result[0] = left[0] * right[0] - left[1] * right[1] - left[2] * right[2] - left[3] * right[3];
  result[1] = left[0] * right[1] + left[1] * right[0] + left[2] * right[3] - left[3] * right[2];
  result[2] = left[0] * right[2] - left[1] * right[3] + left[2] * right[0] + left[3] * right[1];
  result[3] = left[0] * right[3] + left[1] * right[2] - left[2] * right[1] + left[3] * right[0];
}

static void ImuEskf_MatrixMultiply6(const float left[36], const float right[36], float result[36])
{
  /* 固定 6×6 矩阵乘法，result 不得与输入别名；调用点使用结构体内不同工作区保证。 */
  for (uint32_t row = 0U; row < 6U; ++row) {
    for (uint32_t column = 0U; column < 6U; ++column) {
      float value = 0.0f;
      for (uint32_t k = 0U; k < 6U; ++k) {
        value += left[IMU_ESKF_INDEX(row, k, 6U)] * right[IMU_ESKF_INDEX(k, column, 6U)];
      }
      result[IMU_ESKF_INDEX(row, column, 6U)] = value;
    }
  }
}

static void ImuEskf_MatrixMultiplyRightTranspose6(const float left[36], const float right[36], float result[36])
{
  /* 计算 left·rightᵀ，供 FPFᵀ、Joseph 项和重置雅可比变换复用。 */
  for (uint32_t row = 0U; row < 6U; ++row) {
    for (uint32_t column = 0U; column < 6U; ++column) {
      float value = 0.0f;
      for (uint32_t k = 0U; k < 6U; ++k) {
        value += left[IMU_ESKF_INDEX(row, k, 6U)] * right[IMU_ESKF_INDEX(column, k, 6U)];
      }
      result[IMU_ESKF_INDEX(row, column, 6U)] = value;
    }
  }
}

static bool ImuEskf_Invert3(const float matrix[9], float inverse[9])
{
  /*
   * 创新协方差理论上为对称正定矩阵。三阶 Cholesky 分解在求逆的同时检查三个主元为
   * 有限正数；任一主元退化就拒绝本次观测，而不是继续用不稳定的通用伴随矩阵求逆。
   * 分解后对单位矩阵三列分别前代、回代得到 S⁻¹。
   */
  const float l00_squared = matrix[0];
  if (!isfinite(l00_squared) || l00_squared <= 1.0e-12f) {
    return false;
  }

  const float l00 = sqrtf(l00_squared);
  const float l10 = matrix[3] / l00;
  const float l20 = matrix[6] / l00;
  const float l11_squared = matrix[4] - l10 * l10;
  if (!isfinite(l11_squared) || l11_squared <= 1.0e-12f) {
    return false;
  }
  const float l11 = sqrtf(l11_squared);
  const float l21 = (matrix[7] - l20 * l10) / l11;
  const float l22_squared = matrix[8] - l20 * l20 - l21 * l21;
  if (!isfinite(l22_squared) || l22_squared <= 1.0e-12f) {
    return false;
  }
  const float l22 = sqrtf(l22_squared);

  for (uint32_t column = 0U; column < 3U; ++column) {
    const float rhs0 = column == 0U ? 1.0f : 0.0f;
    const float rhs1 = column == 1U ? 1.0f : 0.0f;
    const float rhs2 = column == 2U ? 1.0f : 0.0f;
    const float y0 = rhs0 / l00;
    const float y1 = (rhs1 - l10 * y0) / l11;
    const float y2 = (rhs2 - l20 * y0 - l21 * y1) / l22;
    const float x2 = y2 / l22;
    const float x1 = (y1 - l21 * x2) / l11;
    const float x0 = (y0 - l10 * x1 - l20 * x2) / l00;
    inverse[IMU_ESKF_INDEX(0U, column, 3U)] = x0;
    inverse[IMU_ESKF_INDEX(1U, column, 3U)] = x1;
    inverse[IMU_ESKF_INDEX(2U, column, 3U)] = x2;
  }
  return true;
}

static void ImuEskf_SymmetrizeCovariance(ImuEskf *filter)
{
  /*
   * 浮点矩阵乘法会逐步破坏 P 的严格对称性，这里用上下三角平均恢复对称；对角线再限制
   * 到有限正区间，抑制负方差、下溢和长期无观测方向的无界增长。
   */
  for (uint32_t row = 0U; row < 6U; ++row) {
    for (uint32_t column = row + 1U; column < 6U; ++column) {
      const float average = 0.5f * (filter->covariance[IMU_ESKF_INDEX(row, column, 6U)] +
                                    filter->covariance[IMU_ESKF_INDEX(column, row, 6U)]);
      filter->covariance[IMU_ESKF_INDEX(row, column, 6U)] = average;
      filter->covariance[IMU_ESKF_INDEX(column, row, 6U)] = average;
    }
    float *diagonal = &filter->covariance[IMU_ESKF_INDEX(row, row, 6U)];
    if (!isfinite(*diagonal) || *diagonal < IMU_ESKF_MIN_COVARIANCE) {
      *diagonal = IMU_ESKF_MIN_COVARIANCE;
    } else if (*diagonal > IMU_ESKF_MAX_COVARIANCE) {
      *diagonal = IMU_ESKF_MAX_COVARIANCE;
    }
  }
}

bool ImuEskf_IsStateFinite(const ImuEskf *filter)
{
  /* 工作区是中间量，不属于持久滤波状态；有限性检查覆盖名义状态和完整 6×6 协方差。 */
  if (filter == NULL || !filter->initialized) {
    return false;
  }
  for (uint32_t i = 0U; i < 4U; ++i) {
    if (!isfinite(filter->quaternion[i])) {
      return false;
    }
  }
  for (uint32_t i = 0U; i < 3U; ++i) {
    if (!isfinite(filter->gyro_bias_rad_s[i])) {
      return false;
    }
  }
  for (uint32_t i = 0U; i < IMU_ESKF_COVARIANCE_SIZE; ++i) {
    if (!isfinite(filter->covariance[i])) {
      return false;
    }
  }
  return true;
}

static void ImuEskf_Predict(ImuEskf *filter, const float angular_rate_rad_s[3], float delta_time_s)
{
  /*
   * 名义姿态使用去零偏角速度的指数映射传播。角度足够大时使用 sin/cos 精确增量；小角度
   * 时使用一阶四元数，避免 sin(x)/|omega| 在接近零时数值不稳。随后构造离散一阶状态
   * 转移 F：左上角为姿态误差的反对称角速度项，右上角把零偏误差耦合到姿态误差。
   */
  float omega[3];
  for (uint32_t i = 0U; i < 3U; ++i) {
    omega[i] = angular_rate_rad_s[i] - filter->gyro_bias_rad_s[i];
  }

  const float angle = ImuEskf_VectorNorm(omega) * delta_time_s;
  float delta_quaternion[4];
  if (angle > 1.0e-6f) {
    const float omega_norm = ImuEskf_VectorNorm(omega);
    const float half_angle = 0.5f * angle;
    const float scale = sinf(half_angle) / omega_norm;
    delta_quaternion[0] = cosf(half_angle);
    delta_quaternion[1] = omega[0] * scale;
    delta_quaternion[2] = omega[1] * scale;
    delta_quaternion[3] = omega[2] * scale;
  } else {
    delta_quaternion[0] = 1.0f;
    delta_quaternion[1] = 0.5f * omega[0] * delta_time_s;
    delta_quaternion[2] = 0.5f * omega[1] * delta_time_s;
    delta_quaternion[3] = 0.5f * omega[2] * delta_time_s;
  }

  float predicted_quaternion[4];
  ImuEskf_QuaternionMultiply(filter->quaternion, delta_quaternion, predicted_quaternion);
  memcpy(filter->quaternion, predicted_quaternion, sizeof(predicted_quaternion));
  ImuEskf_QuaternionNormalize(filter->quaternion);

  float *transition = filter->work_a;
  memset(transition, 0, sizeof(filter->work_a));
  for (uint32_t i = 0U; i < 6U; ++i) {
    transition[IMU_ESKF_INDEX(i, i, 6U)] = 1.0f;
  }

  transition[IMU_ESKF_INDEX(0U, 1U, 6U)] = omega[2] * delta_time_s;
  transition[IMU_ESKF_INDEX(0U, 2U, 6U)] = -omega[1] * delta_time_s;
  transition[IMU_ESKF_INDEX(1U, 0U, 6U)] = -omega[2] * delta_time_s;
  transition[IMU_ESKF_INDEX(1U, 2U, 6U)] = omega[0] * delta_time_s;
  transition[IMU_ESKF_INDEX(2U, 0U, 6U)] = omega[1] * delta_time_s;
  transition[IMU_ESKF_INDEX(2U, 1U, 6U)] = -omega[0] * delta_time_s;
  for (uint32_t i = 0U; i < 3U; ++i) {
    transition[IMU_ESKF_INDEX(i, i + 3U, 6U)] = -delta_time_s;
  }

  ImuEskf_MatrixMultiply6(transition, filter->covariance, filter->work_b);
  /* P=F·P·Fᵀ+Q·dt；陀螺白噪声进入姿态块，零偏随机游走进入零偏块。 */
  ImuEskf_MatrixMultiplyRightTranspose6(filter->work_b, transition, filter->covariance);
  for (uint32_t i = 0U; i < 3U; ++i) {
    filter->covariance[IMU_ESKF_INDEX(i, i, 6U)] += IMU_ESKF_GYRO_NOISE_VARIANCE * delta_time_s;
    filter->covariance[IMU_ESKF_INDEX(i + 3U, i + 3U, 6U)] += IMU_ESKF_BIAS_NOISE_VARIANCE * delta_time_s;
  }
  ImuEskf_SymmetrizeCovariance(filter);
}

static bool ImuEskf_CorrectAcceleration(ImuEskf *filter,
                                        const float acceleration_mps2[3],
                                        bool *vibration_high)
{
  /*
   * 校正阶段只使用加速度方向，不使用幅值估计姿态。先检查模长接近标准重力并给出振动
   * 标志，再归一化为单位观测；当前四元数预测车体系重力方向，二者之差构成创新。
   */
  const float acceleration_norm = ImuEskf_VectorNorm(acceleration_mps2);
  const float norm_error = fabsf(acceleration_norm - ROBOT_CONFIG_STANDARD_GRAVITY_MPS2);
  *vibration_high = norm_error > IMU_ESKF_VIBRATION_THRESHOLD_MPS2;
  if (!isfinite(acceleration_norm) || acceleration_norm < 1.0f || norm_error > IMU_ESKF_ACCEL_GATE_MPS2) {
    return false;
  }

  float measurement[3];
  for (uint32_t i = 0U; i < 3U; ++i) {
    measurement[i] = acceleration_mps2[i] / acceleration_norm;
  }

  const float qw = filter->quaternion[0];
  const float qx = filter->quaternion[1];
  const float qy = filter->quaternion[2];
  const float qz = filter->quaternion[3];
  const float predicted_gravity[3] = {
    2.0f * (qx * qz - qw * qy),
    2.0f * (qy * qz + qw * qx),
    1.0f - 2.0f * (qx * qx + qy * qy),
  };
  const float innovation[3] = {
    measurement[0] - predicted_gravity[0],
    measurement[1] - predicted_gravity[1],
    measurement[2] - predicted_gravity[2],
  };
  if (ImuEskf_VectorNorm(innovation) > IMU_ESKF_INNOVATION_GATE) {
    return false;
  }

  float *h = filter->measurement_jacobian;
  /* H=[[g]×, 0]：按本实现误差定义，重力方向对小姿态误差敏感，对零偏没有直接观测。 */
  memset(h, 0, sizeof(filter->measurement_jacobian));
  h[IMU_ESKF_INDEX(0U, 1U, 6U)] = -predicted_gravity[2];
  h[IMU_ESKF_INDEX(0U, 2U, 6U)] = predicted_gravity[1];
  h[IMU_ESKF_INDEX(1U, 0U, 6U)] = predicted_gravity[2];
  h[IMU_ESKF_INDEX(1U, 2U, 6U)] = -predicted_gravity[0];
  h[IMU_ESKF_INDEX(2U, 0U, 6U)] = -predicted_gravity[1];
  h[IMU_ESKF_INDEX(2U, 1U, 6U)] = predicted_gravity[0];

  float *pht = filter->covariance_times_h_transpose;
  for (uint32_t row = 0U; row < 6U; ++row) {
    for (uint32_t column = 0U; column < 3U; ++column) {
      float value = 0.0f;
      for (uint32_t k = 0U; k < 6U; ++k) {
        value += filter->covariance[IMU_ESKF_INDEX(row, k, 6U)] * h[IMU_ESKF_INDEX(column, k, 6U)];
      }
      pht[IMU_ESKF_INDEX(row, column, 3U)] = value;
    }
  }

  const float normalized_error = norm_error / ROBOT_CONFIG_STANDARD_GRAVITY_MPS2;
  /*
   * 即使模长仍在硬门限内，偏离重力越多，越可能含线加速度，因此按归一化模长误差平方
   * 放大测量噪声 R，使卡尔曼增益平滑减小而不是只靠硬开关。
   */
  const float measurement_variance = IMU_ESKF_ACCEL_NOISE_VARIANCE *
                                     (1.0f + 25.0f * normalized_error * normalized_error);
  float *innovation_covariance = filter->innovation_covariance;
  for (uint32_t row = 0U; row < 3U; ++row) {
    for (uint32_t column = 0U; column < 3U; ++column) {
      float value = 0.0f;
      for (uint32_t k = 0U; k < 6U; ++k) {
        value += h[IMU_ESKF_INDEX(row, k, 6U)] * pht[IMU_ESKF_INDEX(k, column, 3U)];
      }
      innovation_covariance[IMU_ESKF_INDEX(row, column, 3U)] =
        value + (row == column ? measurement_variance : 0.0f);
    }
  }
  for (uint32_t row = 0U; row < 3U; ++row) {
    for (uint32_t column = row + 1U; column < 3U; ++column) {
      const float average =
        0.5f * (innovation_covariance[IMU_ESKF_INDEX(row, column, 3U)] +
                innovation_covariance[IMU_ESKF_INDEX(column, row, 3U)]);
      innovation_covariance[IMU_ESKF_INDEX(row, column, 3U)] = average;
      innovation_covariance[IMU_ESKF_INDEX(column, row, 3U)] = average;
    }
  }
  if (!ImuEskf_Invert3(innovation_covariance, filter->innovation_covariance_inverse)) {
    return false;
  }

  float nis = 0.0f;
  /* NIS=创新ᵀS⁻¹创新，用当前不确定度归一化残差；超过三维卡方门限则拒绝观测。 */
  for (uint32_t row = 0U; row < 3U; ++row) {
    float weighted_innovation = 0.0f;
    for (uint32_t column = 0U; column < 3U; ++column) {
      weighted_innovation +=
        filter->innovation_covariance_inverse[IMU_ESKF_INDEX(row, column, 3U)] * innovation[column];
    }
    nis += innovation[row] * weighted_innovation;
  }
  if (!isfinite(nis) || nis > IMU_ESKF_NIS_GATE) {
    return false;
  }

  float *kalman_gain = filter->kalman_gain;
  for (uint32_t row = 0U; row < 6U; ++row) {
    for (uint32_t column = 0U; column < 3U; ++column) {
      float value = 0.0f;
      for (uint32_t k = 0U; k < 3U; ++k) {
        value += pht[IMU_ESKF_INDEX(row, k, 3U)] *
                 filter->innovation_covariance_inverse[IMU_ESKF_INDEX(k, column, 3U)];
      }
      kalman_gain[IMU_ESKF_INDEX(row, column, 3U)] = value;
    }
  }

  float correction[6] = {0.0f};
  for (uint32_t row = 0U; row < 6U; ++row) {
    for (uint32_t k = 0U; k < 3U; ++k) {
      correction[row] += kalman_gain[IMU_ESKF_INDEX(row, k, 3U)] * innovation[k];
    }
  }
  const float correction_norm = ImuEskf_VectorNorm(correction);
  /*
   * 只限制前三维姿态校正的模长，防止单次观测把名义四元数拉动过大；零偏校正随后还会
   * 单独限制在 ±0.5 rad/s 的物理保护范围。
   */
  if (correction_norm > IMU_ESKF_MAX_CORRECTION_RAD) {
    const float scale = IMU_ESKF_MAX_CORRECTION_RAD / correction_norm;
    for (uint32_t i = 0U; i < 3U; ++i) {
      correction[i] *= scale;
    }
  }

  const float delta_quaternion[4] = {
    1.0f,
    0.5f * correction[0],
    0.5f * correction[1],
    0.5f * correction[2],
  };
  float corrected_quaternion[4];
  ImuEskf_QuaternionMultiply(filter->quaternion, delta_quaternion, corrected_quaternion);
  memcpy(filter->quaternion, corrected_quaternion, sizeof(corrected_quaternion));
  ImuEskf_QuaternionNormalize(filter->quaternion);
  for (uint32_t i = 0U; i < 3U; ++i) {
    filter->gyro_bias_rad_s[i] += correction[i + 3U];
    if (filter->gyro_bias_rad_s[i] > 0.5f) {
      filter->gyro_bias_rad_s[i] = 0.5f;
    } else if (filter->gyro_bias_rad_s[i] < -0.5f) {
      filter->gyro_bias_rad_s[i] = -0.5f;
    }
  }

  /*
   * 约瑟夫形式 P=(I-KH)P(I-KH)ᵀ+KRKᵀ 比简化式 P=(I-KH)P 更能在单精度下保持
   * 对称半正定。误差已经注入名义四元数后，误差坐标原点改变，再用一阶 reset Jacobian
   * 把协方差变换到新的零误差切空间。
   */
  float *joseph = filter->work_a;
  memset(joseph, 0, sizeof(filter->work_a));
  for (uint32_t row = 0U; row < 6U; ++row) {
    joseph[IMU_ESKF_INDEX(row, row, 6U)] = 1.0f;
    for (uint32_t column = 0U; column < 6U; ++column) {
      for (uint32_t k = 0U; k < 3U; ++k) {
        joseph[IMU_ESKF_INDEX(row, column, 6U)] -=
          kalman_gain[IMU_ESKF_INDEX(row, k, 3U)] * h[IMU_ESKF_INDEX(k, column, 6U)];
      }
    }
  }
  ImuEskf_MatrixMultiply6(joseph, filter->covariance, filter->work_b);
  ImuEskf_MatrixMultiplyRightTranspose6(filter->work_b, joseph, filter->covariance);
  for (uint32_t row = 0U; row < 6U; ++row) {
    for (uint32_t column = 0U; column < 6U; ++column) {
      float noise_term = 0.0f;
      for (uint32_t k = 0U; k < 3U; ++k) {
        noise_term += kalman_gain[IMU_ESKF_INDEX(row, k, 3U)] *
                      kalman_gain[IMU_ESKF_INDEX(column, k, 3U)] * measurement_variance;
      }
      filter->covariance[IMU_ESKF_INDEX(row, column, 6U)] += noise_term;
    }
  }

  float *reset_jacobian = filter->work_a;
  memset(reset_jacobian, 0, sizeof(filter->work_a));
  for (uint32_t i = 0U; i < 6U; ++i) {
    reset_jacobian[IMU_ESKF_INDEX(i, i, 6U)] = 1.0f;
  }
  reset_jacobian[IMU_ESKF_INDEX(0U, 1U, 6U)] = 0.5f * correction[2];
  reset_jacobian[IMU_ESKF_INDEX(0U, 2U, 6U)] = -0.5f * correction[1];
  reset_jacobian[IMU_ESKF_INDEX(1U, 0U, 6U)] = -0.5f * correction[2];
  reset_jacobian[IMU_ESKF_INDEX(1U, 2U, 6U)] = 0.5f * correction[0];
  reset_jacobian[IMU_ESKF_INDEX(2U, 0U, 6U)] = 0.5f * correction[1];
  reset_jacobian[IMU_ESKF_INDEX(2U, 1U, 6U)] = -0.5f * correction[0];
  ImuEskf_MatrixMultiply6(reset_jacobian, filter->covariance, filter->work_b);
  ImuEskf_MatrixMultiplyRightTranspose6(filter->work_b, reset_jacobian, filter->covariance);
  ImuEskf_SymmetrizeCovariance(filter);
  if (filter->accel_update_count < UINT32_MAX) {
    filter->accel_update_count++;
  }
  return true;
}

bool ImuEskf_Init(ImuEskf *filter, const float acceleration_mps2[3], const float gyro_bias_rad_s[3])
{
  if (filter == NULL || acceleration_mps2 == NULL || gyro_bias_rad_s == NULL) {
    return false;
  }

  const float acceleration_norm = ImuEskf_VectorNorm(acceleration_mps2);
  if (!isfinite(acceleration_norm) || acceleration_norm < 1.0f) {
    return false;
  }

  /*
   * 静止重力只能确定 roll/pitch，初始 yaw 选为 0。姿态协方差中 yaw 给较大不确定度 1.0，
   * roll/pitch 与零偏从 1e-2 起步，明确表达航向不可观而不是伪装成已知。
   */
  memset(filter, 0, sizeof(*filter));
  const float roll = atan2f(acceleration_mps2[1], acceleration_mps2[2]);
  const float pitch = atan2f(-acceleration_mps2[0],
                            sqrtf(acceleration_mps2[1] * acceleration_mps2[1] +
                                  acceleration_mps2[2] * acceleration_mps2[2]));
  const float half_roll = 0.5f * roll;
  const float half_pitch = 0.5f * pitch;
  filter->quaternion[0] = cosf(half_roll) * cosf(half_pitch);
  filter->quaternion[1] = sinf(half_roll) * cosf(half_pitch);
  filter->quaternion[2] = cosf(half_roll) * sinf(half_pitch);
  filter->quaternion[3] = -sinf(half_roll) * sinf(half_pitch);
  ImuEskf_QuaternionNormalize(filter->quaternion);
  memcpy(filter->gyro_bias_rad_s, gyro_bias_rad_s, sizeof(filter->gyro_bias_rad_s));

  filter->covariance[IMU_ESKF_INDEX(0U, 0U, 6U)] = 1.0e-2f;
  filter->covariance[IMU_ESKF_INDEX(1U, 1U, 6U)] = 1.0e-2f;
  filter->covariance[IMU_ESKF_INDEX(2U, 2U, 6U)] = 1.0f;
  for (uint32_t i = 3U; i < 6U; ++i) {
    filter->covariance[IMU_ESKF_INDEX(i, i, 6U)] = 1.0e-2f;
  }
  filter->initialized = true;
  return true;
}

bool ImuEskf_Update(ImuEskf *filter,
                    const float angular_rate_rad_s[3],
                    const float acceleration_mps2[3],
                    float delta_time_s,
                    bool *accel_update_used,
                    bool *vibration_high)
{
  /*
   * delta_time_s 上限 0.1 s 是数值安全边界；应用层还会按正常 ODR 把实际使用 dt 限到更小。
   * 加速度校正被门控拒绝属于正常预测模式，因此函数仍返回 true；只有输入契约、预测后
   * 非有限或最终状态非有限才返回 false。
   */
  if (filter == NULL || angular_rate_rad_s == NULL || acceleration_mps2 == NULL || accel_update_used == NULL ||
      vibration_high == NULL || !filter->initialized || !isfinite(delta_time_s) || delta_time_s <= 0.0f ||
      delta_time_s > 0.1f) {
    return false;
  }

  *accel_update_used = false;
  *vibration_high = false;
  ImuEskf_Predict(filter, angular_rate_rad_s, delta_time_s);
  if (!ImuEskf_IsStateFinite(filter)) {
    return false;
  }
  *accel_update_used = ImuEskf_CorrectAcceleration(filter, acceleration_mps2, vibration_high);
  return ImuEskf_IsStateFinite(filter);
}

void ImuEskf_GetEulerRad(const ImuEskf *filter, float euler_rad[3])
{
  if (filter == NULL || euler_rad == NULL || !filter->initialized) {
    return;
  }

  const float qw = filter->quaternion[0];
  const float qx = filter->quaternion[1];
  const float qy = filter->quaternion[2];
  const float qz = filter->quaternion[3];
  euler_rad[0] = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
  /* pitch 的 asin 参数受浮点舍入影响可能略超 [-1,1]，先限幅避免生成 NaN。 */
  float pitch_argument = 2.0f * (qw * qy - qz * qx);
  if (pitch_argument > 1.0f) {
    pitch_argument = 1.0f;
  } else if (pitch_argument < -1.0f) {
    pitch_argument = -1.0f;
  }
  euler_rad[1] = asinf(pitch_argument);
  euler_rad[2] = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

bool ImuEskf_IsConverged(const ImuEskf *filter)
{
  /* 收敛门只统计被实际接受的重力校正，不把纯陀螺预测或被门控拒绝的样本计入。 */
  return filter != NULL && filter->initialized && filter->accel_update_count >= IMU_ESKF_CONVERGENCE_UPDATES;
}
