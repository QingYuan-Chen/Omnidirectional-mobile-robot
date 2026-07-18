#include "app_imu.h"

#include "cmsis_os.h"
#include "imu_eskf.h"
#include "main.h"
#include "robot_config.h"

#include <math.h>
#include <string.h>

/*
 * IMU 应用层把 QMI8658A 原始帧转换成可供整机安全与姿态估计使用的受控数据流。
 *
 * 数据路径严格按“时间戳检查→坐标/单位换算→突变拒绝→ESKF 原始输入→独立输出低通”
 * 执行。任何被判为重复、跳变过大或突变的样本都不会进入 ESKF。传感器总线故障与估计器
 * 数值故障分别计数和分级；恢复必须通过连续稳定样本门槛。退避只限制下一次总线访问，
 * 不阻塞任务，因此安全任务仍能区分“IMU 数据不健康”和“IMU 任务已经失联”。
 */

#define APP_IMU_PI                         (3.14159265358979323846f)
#define APP_IMU_TIMESTAMP_MASK             (0x00FFFFFFUL)
#define APP_IMU_MAX_CALIBRATION_READ_ERRORS (20U)
#define APP_IMU_CALIBRATION_DISCARD_SAMPLES  (3U)
#define APP_IMU_MAX_TIMESTAMP_STEP         (11U)
#define APP_IMU_MAX_FILTER_DELTA_TIME_S    (0.05f)
#define APP_IMU_CALIBRATION_ACCEL_NORM_TOLERANCE (0.15f * ROBOT_CONFIG_STANDARD_GRAVITY_MPS2)
#define APP_IMU_CALIBRATION_ACCEL_STD_LIMIT      (0.02f * ROBOT_CONFIG_STANDARD_GRAVITY_MPS2)
#define APP_IMU_CALIBRATION_GYRO_NORM_LIMIT      (3.0f * APP_IMU_PI / 180.0f)
#define APP_IMU_CALIBRATION_GYRO_STD_LIMIT       (0.2f * APP_IMU_PI / 180.0f)

/*
 * imu_filter 与 imu_output 由唯一 IMU 任务独占更新，其他任务只读取 runtime_snapshot 副本。
 * previous_sensor_timestamp 保存最近观察到的原始帧，用于时间戳连续性诊断；
 * accepted_* 只保存最近一次通过质量门的向量，作为突变比较基线。等待进入 ESKF 的
 * 时间戳步数单独累计，避免尖峰帧被拒绝后丢失实际积分时间。calibration_gyro_bias_rad_s
 * 是估计器重置时的可靠初始零偏，不随输出低通变化。
 */
static ImuEskf imu_filter;
static AppImuOutput imu_output;
static bool imu_calibrated;
static uint32_t previous_sensor_timestamp;
static uint32_t pending_filter_timestamp_steps;
static float accepted_acceleration_mps2[3];
static float accepted_angular_rate_rad_s[3];
static float calibration_gyro_bias_rad_s[3];
static uint32_t backoff_index;
static bool recovery_required;

static const uint32_t app_imu_backoff_ms[] = {20U, 50U, 100U, 200U, 500U};

static uint32_t AppImu_SaturatingIncrement(uint32_t value)
{
  return value < UINT32_MAX ? value + 1U : UINT32_MAX;
}

static bool AppImu_TickIsBefore(uint32_t now_ms, uint32_t target_ms)
{
  /* 有符号模差比较允许 HAL 毫秒计数自然回绕，目标距离必须小于 2^31 ms。 */
  return (int32_t)(now_ms - target_ms) < 0;
}

static bool AppImu_IsStaleAt(uint32_t now_ms, uint32_t last_good_tick_ms)
{
  /* 无符号减法同时覆盖正常计时和 HAL 毫秒计数回绕，20 ms 边界本身即视为陈旧。 */
  return (now_ms - last_good_tick_ms) >= ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS;
}

static void AppImu_StartBackoff(uint32_t now_ms)
{
  /*
   * 退避序列从 20 ms 逐级增长到 500 ms 并保持封顶。这里只记录截止时刻，绝不 osDelay；
   * IMU 任务仍按中断或等待超时醒来、刷新 sample_age_ms、发布健康状态并上报任务心跳。
   */
  const uint32_t delay_ms = app_imu_backoff_ms[backoff_index];
  imu_output.retry_delay_ms = delay_ms;
  imu_output.next_retry_tick_ms = now_ms + delay_ms;
  imu_output.backoff_count = AppImu_SaturatingIncrement(imu_output.backoff_count);
  if (backoff_index + 1U < (uint32_t)(sizeof(app_imu_backoff_ms) / sizeof(app_imu_backoff_ms[0]))) {
    backoff_index++;
  }
}

static void AppImu_ResetBackoff(void)
{
  backoff_index = 0U;
  imu_output.retry_delay_ms = 0U;
  imu_output.next_retry_tick_ms = 0U;
}

static void AppImu_MarkSensorFailure(bool persistent)
{
  /*
   * 传感器失败立即撤销 DATA_VALID 并要求重新稳定。persistent 只决定是否升级为持久故障；
   * 即使未升级，上层也会因 TRANSIENT_DEGRADED 暂时禁止运动。
   */
  recovery_required = true;
  imu_output.stable_sample_count = 0U;
  imu_output.flags |= APP_IMU_FLAG_SENSOR_DEGRADED;
  imu_output.flags &= ~((uint32_t)APP_IMU_FLAG_DATA_VALID | (uint32_t)APP_IMU_FLAG_RECOVERING);
  if (persistent || (imu_output.flags & APP_IMU_FLAG_SENSOR_FAULT) != 0U) {
    imu_output.health = APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT;
    imu_output.flags |= APP_IMU_FLAG_SENSOR_FAULT;
  } else if ((imu_output.flags & APP_IMU_FLAG_ESTIMATOR_FAULT) != 0U) {
    imu_output.health = APP_IMU_HEALTH_ESTIMATOR_FAULT;
  } else {
    imu_output.health = APP_IMU_HEALTH_TRANSIENT_DEGRADED;
  }
}

static void AppImu_MarkEstimatorFailure(const float acceleration_mps2[3])
{
  /*
   * 数值故障与总线故障分开记录。当前样本的加速度已通过时间戳和突变检查，可用于尝试
   * 重建初始倾角；零偏回退到启动静止标定值。重建成功也不能立即恢复 DATA_VALID。
   */
  recovery_required = true;
  imu_output.stable_sample_count = 0U;
  imu_output.estimator_fault_count = AppImu_SaturatingIncrement(imu_output.estimator_fault_count);
  imu_output.health = APP_IMU_HEALTH_ESTIMATOR_FAULT;
  imu_output.flags |= APP_IMU_FLAG_ESTIMATOR_FAULT;
  imu_output.flags &= ~((uint32_t)APP_IMU_FLAG_DATA_VALID | (uint32_t)APP_IMU_FLAG_RECOVERING |
                        (uint32_t)APP_IMU_FLAG_FILTER_CONVERGED | (uint32_t)APP_IMU_FLAG_TILT_VALID);
  if (ImuEskf_Init(&imu_filter, acceleration_mps2, calibration_gyro_bias_rad_s)) {
    imu_output.flags |= APP_IMU_FLAG_FILTER_INITIALIZED;
  } else {
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_FILTER_INITIALIZED;
  }
}

static void AppImu_UpdateRecovery(bool timestamp_continuous,
                                  bool accel_update_used,
                                  bool vibration_high)
{
  if (!recovery_required) {
    imu_output.health = APP_IMU_HEALTH_HEALTHY;
    imu_output.flags |= APP_IMU_FLAG_DATA_VALID;
    return;
  }

  /*
   * 本函数只在突变检查与 ESKF 更新都成功后调用；这两类失败会在各自分支直接清零恢复
   * 状态。剩余严格资格只用于故障恢复：时间戳必须连续、滤波器已初始化且全状态有限、
   * 当前加速度观测确实参与校正，并且没有高振动。任一条件失败都清零连续计数；正常
   * 健康运行不会因单次观测拒绝或高振动直接停车。
   */
  const bool stable_sample =
    timestamp_continuous && imu_filter.initialized && ImuEskf_IsStateFinite(&imu_filter) &&
    accel_update_used && !vibration_high;
  if (stable_sample) {
    imu_output.stable_sample_count = AppImu_SaturatingIncrement(imu_output.stable_sample_count);
  } else {
    imu_output.stable_sample_count = 0U;
  }

  if (imu_output.stable_sample_count >= ROBOT_CONFIG_IMU_RECOVERY_STABLE_SAMPLES) {
    recovery_required = false;
    imu_output.health = APP_IMU_HEALTH_HEALTHY;
    imu_output.flags |= APP_IMU_FLAG_DATA_VALID;
    imu_output.flags &= ~((uint32_t)APP_IMU_FLAG_SENSOR_DEGRADED | (uint32_t)APP_IMU_FLAG_SENSOR_FAULT |
                          (uint32_t)APP_IMU_FLAG_RECOVERING | (uint32_t)APP_IMU_FLAG_ESTIMATOR_FAULT);
  } else {
    imu_output.health = APP_IMU_HEALTH_RECOVERING;
    imu_output.flags |= APP_IMU_FLAG_RECOVERING;
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_DATA_VALID;
  }
}

static float AppImu_VectorNorm(const float vector[3])
{
  return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

static uint32_t AppImu_SaturatingAdd(uint32_t value, uint32_t increment)
{
  return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

static bool AppImu_IsSpike(const float acceleration_mps2[3], const float angular_rate_rad_s[3])
{
  /*
   * 在车体坐标系 SI 单位中比较相邻三维向量范数，而不是逐轴限幅。判据位于普通低通
   * 之前，避免低通把单点尖峰摊到多个周期；超过任一加速度或角速度门限即拒绝整帧。
   */
  float acceleration_delta[3];
  float angular_rate_delta[3];
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    acceleration_delta[axis] = acceleration_mps2[axis] - accepted_acceleration_mps2[axis];
    angular_rate_delta[axis] = angular_rate_rad_s[axis] - accepted_angular_rate_rad_s[axis];
  }

  return AppImu_VectorNorm(acceleration_delta) > ROBOT_CONFIG_IMU_ACCEL_SPIKE_LIMIT_MPS2 ||
         AppImu_VectorNorm(angular_rate_delta) > ROBOT_CONFIG_IMU_GYRO_SPIKE_LIMIT_RAD_S;
}

static void AppImu_UpdateFilteredOutput(const float acceleration_mps2[3],
                                        const float angular_rate_rad_s[3],
                                        float delta_time_s)
{
  /*
   * 一阶低通系数由实际样本 dt 计算，输出初值在标定结束时设置为静止均值，因此不会
   * 从零产生启动瞬态。该函数只能在 ESKF 成功处理样本之后调用，结果绝不回灌估计器。
   */
  const float time_constant_s = 1.0f / (2.0f * APP_IMU_PI * ROBOT_CONFIG_IMU_OUTPUT_FILTER_CUTOFF_HZ);
  const float alpha = delta_time_s / (time_constant_s + delta_time_s);
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    imu_output.filtered_acceleration_mps2[axis] +=
      alpha * (acceleration_mps2[axis] - imu_output.filtered_acceleration_mps2[axis]);
    imu_output.filtered_angular_rate_rad_s[axis] +=
      alpha * (angular_rate_rad_s[axis] - imu_output.filtered_angular_rate_rad_s[axis]);
  }
}

static void AppImu_ConvertToBodySi(const BspImuSample *sample,
                                   float acceleration_mps2[3],
                                   float angular_rate_rad_s[3])
{
  /*
   * 传感器安装方向与车体坐标系的对应为：车体 X=芯片 Y，车体 Y=-芯片 X，车体 Z=芯片 Z。
   * 加速度按 LSB/g 换算为 m/s²，角速度按 LSB/(°/s) 换算为 rad/s。若机械安装方向变化，
   * 必须在此统一修改并重新验证静止重力方向和电机运动方向。
   */
  const float acceleration_scale =
    ROBOT_CONFIG_STANDARD_GRAVITY_MPS2 / ROBOT_CONFIG_IMU_ACCEL_LSB_PER_G;
  const float angular_rate_scale = APP_IMU_PI / (180.0f * ROBOT_CONFIG_IMU_GYRO_LSB_PER_DPS);

  acceleration_mps2[0] = (float)sample->acceleration[1] * acceleration_scale;
  acceleration_mps2[1] = -(float)sample->acceleration[0] * acceleration_scale;
  acceleration_mps2[2] = (float)sample->acceleration[2] * acceleration_scale;
  angular_rate_rad_s[0] = (float)sample->angular_rate[1] * angular_rate_scale;
  angular_rate_rad_s[1] = -(float)sample->angular_rate[0] * angular_rate_scale;
  angular_rate_rad_s[2] = (float)sample->angular_rate[2] * angular_rate_scale;
}

static void AppImu_UpdateMeanAndVariance(float sample, uint32_t count, float *mean, float *m2)
{
  /* Welford 在线算法只保存均值和离均差平方和，避免累加平方造成精度损失和大数组占用。 */
  const float delta = sample - *mean;
  *mean += delta / (float)count;
  const float delta_after_mean = sample - *mean;
  *m2 += delta * delta_after_mean;
}

static void AppImu_CopyFilterState(void)
{
  /*
   * 统一复制名义状态和输出标志，防止调用点只更新四元数却遗漏零偏或收敛状态。
   * 六轴系统没有绝对航向参考，因此每次发布都显式清除 ABSOLUTE_YAW_VALID。
   */
  memcpy(imu_output.quaternion, imu_filter.quaternion, sizeof(imu_output.quaternion));
  memcpy(imu_output.gyro_bias_rad_s, imu_filter.gyro_bias_rad_s, sizeof(imu_output.gyro_bias_rad_s));
  ImuEskf_GetEulerRad(&imu_filter, imu_output.euler_rad);
  if (ImuEskf_IsConverged(&imu_filter)) {
    imu_output.flags |= APP_IMU_FLAG_FILTER_CONVERGED;
  } else {
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_FILTER_CONVERGED;
  }
  imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_ABSOLUTE_YAW_VALID;
}

static void AppImu_UpdateStaleState(uint32_t now_ms)
{
  /*
   * sample_age_ms 表示距最后一次完整接受样本的主机时间，不是芯片时间戳步数。退避、重复
   * 帧和 BUSY 都会继续增长；达到 20 ms 时 DATA_STALE 置位并强制撤销 DATA_VALID。
   * 陈旧首次出现时进入稳定恢复流程；重复轮询只刷新年龄，不反复清零已经开始的恢复计数。
   */
  imu_output.sample_age_ms = now_ms - imu_output.last_good_tick_ms;
  if (!imu_calibrated) {
    imu_output.flags |= APP_IMU_FLAG_DATA_STALE;
    imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_DATA_VALID;
    return;
  }

  if (AppImu_IsStaleAt(now_ms, imu_output.last_good_tick_ms)) {
    if ((imu_output.flags & APP_IMU_FLAG_DATA_STALE) == 0U) {
      recovery_required = true;
      imu_output.stable_sample_count = 0U;
      imu_output.flags |= APP_IMU_FLAG_SENSOR_DEGRADED;
      /*
       * 恢复阶段的 health 会暂时显示 RECOVERING，因此不能只看当前枚举保留严重度；
       * 根因标志在完整恢复前一直保留，再次陈旧时必须恢复为对应的持久或估计器故障。
       */
      if ((imu_output.flags & APP_IMU_FLAG_SENSOR_FAULT) != 0U) {
        imu_output.health = APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT;
      } else if ((imu_output.flags & APP_IMU_FLAG_ESTIMATOR_FAULT) != 0U) {
        imu_output.health = APP_IMU_HEALTH_ESTIMATOR_FAULT;
      } else {
        imu_output.health = APP_IMU_HEALTH_TRANSIENT_DEGRADED;
      }
    }
    imu_output.flags |= APP_IMU_FLAG_DATA_STALE;
    imu_output.flags &= ~((uint32_t)APP_IMU_FLAG_DATA_VALID | (uint32_t)APP_IMU_FLAG_RECOVERING);
  }
}

BspStatus AppImu_Calibrate(void)
{
  /*
   * 标定入口先彻底清除上一次运行的滤波器、退避和诊断基线，保证重试不会继承半初始化
   * 状态。SENSOR_PRESENT 表示 BSP 初始化阶段已成功发现设备，不代表数据已经可用。
   */
  memset(&imu_filter, 0, sizeof(imu_filter));
  memset(&imu_output, 0, sizeof(imu_output));
  imu_calibrated = false;
  previous_sensor_timestamp = 0U;
  pending_filter_timestamp_steps = 0U;
  backoff_index = 0U;
  memset(accepted_acceleration_mps2, 0, sizeof(accepted_acceleration_mps2));
  memset(accepted_angular_rate_rad_s, 0, sizeof(accepted_angular_rate_rad_s));
  memset(calibration_gyro_bias_rad_s, 0, sizeof(calibration_gyro_bias_rad_s));
  recovery_required = false;
  imu_output.flags = APP_IMU_FLAG_SENSOR_PRESENT;

  float acceleration_mean[3] = {0.0f};
  float acceleration_m2[3] = {0.0f};
  float angular_rate_mean[3] = {0.0f};
  float angular_rate_m2[3] = {0.0f};
  BspImuSample latest_sample = {0};
  uint32_t sample_count = 0U;
  uint32_t read_error_count = 0U;
  uint32_t discard_count = APP_IMU_CALIBRATION_DISCARD_SAMPLES;
  const uint32_t started_at = HAL_GetTick();

  while (sample_count < ROBOT_CONFIG_IMU_CALIBRATION_SAMPLES &&
         (HAL_GetTick() - started_at) < ROBOT_CONFIG_IMU_CALIBRATION_TIMEOUT_MS) {
    BspImuSample sample;
    const BspStatus status = BspImu_ReadSample(&sample);
    if (status == BSP_OK) {
      /*
       * 丢弃复位后最初三帧；之后用芯片时间戳去重。若瞬时重力模长或角速度表明机器人
       * 正在运动，则整段统计清零，要求重新连续取得一组静止样本，而不是混入旧均值。
       */
      if (discard_count > 0U) {
        discard_count--;
        osDelay(1U);
        continue;
      }
      if (sample_count > 0U && sample.sensor_timestamp == latest_sample.sensor_timestamp) {
        osDelay(1U);
        continue;
      }

      float acceleration_mps2[3];
      float angular_rate_rad_s[3];
      AppImu_ConvertToBodySi(&sample, acceleration_mps2, angular_rate_rad_s);
      if (fabsf(AppImu_VectorNorm(acceleration_mps2) - ROBOT_CONFIG_STANDARD_GRAVITY_MPS2) >
            APP_IMU_CALIBRATION_ACCEL_NORM_TOLERANCE ||
          AppImu_VectorNorm(angular_rate_rad_s) > APP_IMU_CALIBRATION_GYRO_NORM_LIMIT) {
        sample_count = 0U;
        memset(acceleration_mean, 0, sizeof(acceleration_mean));
        memset(acceleration_m2, 0, sizeof(acceleration_m2));
        memset(angular_rate_mean, 0, sizeof(angular_rate_mean));
        memset(angular_rate_m2, 0, sizeof(angular_rate_m2));
        memset(&latest_sample, 0, sizeof(latest_sample));
        osDelay(1U);
        continue;
      }
      sample_count++;
      for (uint32_t axis = 0U; axis < 3U; ++axis) {
        AppImu_UpdateMeanAndVariance(
          acceleration_mps2[axis], sample_count, &acceleration_mean[axis], &acceleration_m2[axis]);
        AppImu_UpdateMeanAndVariance(
          angular_rate_rad_s[axis], sample_count, &angular_rate_mean[axis], &angular_rate_m2[axis]);
      }
      latest_sample = sample;
    } else if (status != BSP_BUSY) {
      read_error_count++;
      if (read_error_count >= APP_IMU_MAX_CALIBRATION_READ_ERRORS) {
        imu_output.flags |= APP_IMU_FLAG_SENSOR_FAULT;
        return status;
      }
    }
    osDelay(1U);
  }

  if (sample_count < ROBOT_CONFIG_IMU_CALIBRATION_SAMPLES) {
    return BSP_TIMEOUT;
  }

  float acceleration_std[3];
  float angular_rate_std[3];
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    /* 样本数固定大于 1，使用无偏样本标准差检查三轴振动与陀螺噪声。 */
    acceleration_std[axis] = sqrtf(acceleration_m2[axis] / (float)(sample_count - 1U));
    angular_rate_std[axis] = sqrtf(angular_rate_m2[axis] / (float)(sample_count - 1U));
    if (acceleration_std[axis] > APP_IMU_CALIBRATION_ACCEL_STD_LIMIT ||
        angular_rate_std[axis] > APP_IMU_CALIBRATION_GYRO_STD_LIMIT) {
      return BSP_ERROR;
    }
  }

  if (fabsf(AppImu_VectorNorm(acceleration_mean) - ROBOT_CONFIG_STANDARD_GRAVITY_MPS2) >
        APP_IMU_CALIBRATION_ACCEL_NORM_TOLERANCE ||
      AppImu_VectorNorm(angular_rate_mean) > APP_IMU_CALIBRATION_GYRO_NORM_LIMIT) {
    return BSP_ERROR;
  }
  if (!ImuEskf_Init(&imu_filter, acceleration_mean, angular_rate_mean)) {
    return BSP_ERROR;
  }

  imu_calibrated = true;
  /*
   * 标定均值同时建立 ESKF、输出低通和突变比较的共同初值。对外角速度初值为零，因为
   * 静止均值已作为陀螺零偏；accepted_angular_rate 保留未扣偏均值以匹配后续原始差分。
   */
  memcpy(calibration_gyro_bias_rad_s, angular_rate_mean, sizeof(calibration_gyro_bias_rad_s));
  previous_sensor_timestamp = latest_sample.sensor_timestamp;
  imu_output.raw_sample = latest_sample;
  memcpy(imu_output.acceleration_mps2, acceleration_mean, sizeof(acceleration_mean));
  memcpy(imu_output.filtered_acceleration_mps2, acceleration_mean, sizeof(acceleration_mean));
  memcpy(accepted_acceleration_mps2, acceleration_mean, sizeof(acceleration_mean));
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    imu_output.angular_rate_rad_s[axis] = 0.0f;
    imu_output.filtered_angular_rate_rad_s[axis] = 0.0f;
    accepted_angular_rate_rad_s[axis] = angular_rate_mean[axis];
  }
  imu_output.temperature_celsius = (float)latest_sample.temperature / 256.0f;
  imu_output.sensor_timestamp = latest_sample.sensor_timestamp;
  imu_output.host_tick_ms = latest_sample.host_tick_ms;
  imu_output.last_good_tick_ms = latest_sample.host_tick_ms;
  imu_output.sample_age_ms = 0U;
  imu_output.read_error_count = read_error_count;
  imu_output.health = APP_IMU_HEALTH_HEALTHY;
  imu_output.stable_sample_count = ROBOT_CONFIG_IMU_RECOVERY_STABLE_SAMPLES;
  imu_output.flags |= APP_IMU_FLAG_CALIBRATED | APP_IMU_FLAG_DATA_VALID | APP_IMU_FLAG_FILTER_INITIALIZED |
                      APP_IMU_FLAG_TIMESTAMP_VALID | APP_IMU_FLAG_TILT_VALID;
  AppImu_CopyFilterState();
  return BSP_OK;
}

BspStatus AppImu_Process(uint32_t now_ms, AppImuOutput *output)
{
  if (output == NULL) {
    return BSP_INVALID_ARG;
  }
  if (!imu_calibrated) {
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_ERROR;
  }

  if (imu_output.retry_delay_ms != 0U && AppImu_TickIsBefore(now_ms, imu_output.next_retry_tick_ms)) {
    /*
     * 退避窗口内不访问 I2C，避免故障设备被高频轮询拖住总线；仍返回最新完整快照，
     * 让任务编排层继续发布心跳并让安全层通过 health/sample_age 判断数据不可用。
     */
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_BUSY;
  }

  BspImuSample sample;
  const BspStatus status = BspImu_ReadSample(&sample);
  if (status == BSP_BUSY) {
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_BUSY;
  }
  if (status != BSP_OK) {
    /*
     * BSP_BUSY 只是数据未就绪，不算读错也不启动退避；其余状态才累计连续错误。达到连续
     * 门槛后升级持久传感器故障，但退避序列从第一次真实读错即开始。
     */
    imu_output.read_error_count = AppImu_SaturatingIncrement(imu_output.read_error_count);
    imu_output.consecutive_error_count = AppImu_SaturatingIncrement(imu_output.consecutive_error_count);
    AppImu_StartBackoff(now_ms);
    AppImu_MarkSensorFailure(
      imu_output.consecutive_error_count >= ROBOT_CONFIG_IMU_PERSISTENT_FAULT_THRESHOLD);
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return status;
  }
  AppImu_ResetBackoff();

  const uint32_t timestamp_step = (sample.sensor_timestamp - previous_sensor_timestamp) & APP_IMU_TIMESTAMP_MASK;
  if (timestamp_step == 0U) {
    imu_output.duplicate_count = AppImu_SaturatingIncrement(imu_output.duplicate_count);
    if (recovery_required) {
      /*
       * 恢复门槛要求连续取得新的稳定帧。重复时间戳不是传感器总线故障，但会中断连续
       * 样本序列，因此必须清零恢复计数；重复帧本身没有通过质量链，不能把瞬时、持久或
       * 估计器故障改写为 RECOVERING。已经开始恢复时则保留原有恢复状态和标志。
       */
      imu_output.stable_sample_count = 0U;
      imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_DATA_VALID;
    }
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_BUSY;
  }

  if (timestamp_step > APP_IMU_MAX_TIMESTAMP_STEP) {
    /*
     * 芯片时间戳为 24 位，模差可自然处理回绕。step 2～11 记为有限丢样并继续处理；更大
     * 跳变说明 dt 和样本链已不可信，直接进入降级。丢样通过独立计数表达，不再把
     * UINT32_MAX 填入普通字段充当哨兵。
     */
    previous_sensor_timestamp = sample.sensor_timestamp;
    pending_filter_timestamp_steps =
      AppImu_SaturatingAdd(pending_filter_timestamp_steps, timestamp_step);
    imu_output.dropped_sample_count = AppImu_SaturatingAdd(imu_output.dropped_sample_count, timestamp_step - 1U);
    imu_output.flags |= APP_IMU_FLAG_DATA_STALE;
    imu_output.flags &=
      ~((uint32_t)APP_IMU_FLAG_DATA_VALID | (uint32_t)APP_IMU_FLAG_TIMESTAMP_VALID |
        (uint32_t)APP_IMU_FLAG_FILTER_CONVERGED);
    AppImu_MarkSensorFailure(false);
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_ERROR;
  }

  if (timestamp_step > 1U) {
    const uint32_t dropped = timestamp_step - 1U;
    imu_output.dropped_sample_count = AppImu_SaturatingAdd(imu_output.dropped_sample_count, dropped);
  }

  float acceleration_mps2[3];
  float angular_rate_rad_s[3];
  AppImu_ConvertToBodySi(&sample, acceleration_mps2, angular_rate_rad_s);
  previous_sensor_timestamp = sample.sensor_timestamp;
  pending_filter_timestamp_steps =
    AppImu_SaturatingAdd(pending_filter_timestamp_steps, timestamp_step);
  if (AppImu_IsSpike(acceleration_mps2, angular_rate_rad_s)) {
    imu_output.spike_reject_count = AppImu_SaturatingIncrement(imu_output.spike_reject_count);
    imu_output.consecutive_spike_count = AppImu_SaturatingIncrement(imu_output.consecutive_spike_count);
    /*
     * 被拒绝的尖峰只推进原始帧时间戳和待积分时间，不更新突变向量基线、输出或 ESKF。
     * 下一帧仍与最后接受样本比较，因此“正常→单点尖峰→正常”只拒绝中间异常一次。
     */
    imu_output.flags |= APP_IMU_FLAG_SAMPLE_SPIKE;
    AppImu_MarkSensorFailure(
      imu_output.consecutive_spike_count >= ROBOT_CONFIG_IMU_PERSISTENT_FAULT_THRESHOLD);
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_ERROR;
  }
  float delta_time_s = (float)pending_filter_timestamp_steps / ROBOT_CONFIG_IMU_ODR_HZ;
  if (delta_time_s > APP_IMU_MAX_FILTER_DELTA_TIME_S) {
    delta_time_s = APP_IMU_MAX_FILTER_DELTA_TIME_S;
  }

  bool accel_update_used = false;
  bool vibration_high = false;
  /*
   * ESKF 直接接收本帧未经过普通低通的换算值；内部只通过重力观测门控决定是否校正。
   * 预测或数值检查失败会重置估计器并进入恢复状态，不能继续发布旧姿态为有效数据。
   */
  if (!ImuEskf_Update(
        &imu_filter, angular_rate_rad_s, acceleration_mps2, delta_time_s, &accel_update_used, &vibration_high)) {
    AppImu_MarkEstimatorFailure(acceleration_mps2);
    /*
     * 估计器重建以当前加速度为新的时间起点；无论重建是否成功，旧累计时间都不能再次
     * 施加到下一次更新，否则会对同一间隔重复积分。
     */
    pending_filter_timestamp_steps = 0U;
    AppImu_UpdateStaleState(now_ms);
    *output = imu_output;
    return BSP_ERROR;
  }

  imu_output.raw_sample = sample;
  /*
   * 只有走到这里的样本才算“最后良好样本”：更新所有数值、主机时间、序号和诊断标志，
   * 清除连续读错/尖峰计数，再根据时间戳连续性推进稳定恢复。原始角速度送入 ESKF，
   * 对外 angular_rate 则扣除滤波器当前估计零偏。
   */
  memcpy(imu_output.acceleration_mps2, acceleration_mps2, sizeof(acceleration_mps2));
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    imu_output.angular_rate_rad_s[axis] = angular_rate_rad_s[axis] - imu_filter.gyro_bias_rad_s[axis];
  }
  AppImu_UpdateFilteredOutput(
    acceleration_mps2, imu_output.angular_rate_rad_s, delta_time_s);
  memcpy(accepted_acceleration_mps2, acceleration_mps2, sizeof(accepted_acceleration_mps2));
  memcpy(accepted_angular_rate_rad_s, angular_rate_rad_s, sizeof(accepted_angular_rate_rad_s));
  pending_filter_timestamp_steps = 0U;
  imu_output.temperature_celsius = (float)sample.temperature / 256.0f;
  imu_output.sensor_timestamp = sample.sensor_timestamp;
  imu_output.host_tick_ms = sample.host_tick_ms;
  imu_output.last_good_tick_ms = now_ms;
  imu_output.sample_age_ms = 0U;
  imu_output.sequence = AppImu_SaturatingIncrement(imu_output.sequence);
  imu_output.consecutive_error_count = 0U;
  imu_output.consecutive_spike_count = 0U;
  imu_output.flags |= APP_IMU_FLAG_SENSOR_PRESENT | APP_IMU_FLAG_CALIBRATED |
                      APP_IMU_FLAG_FILTER_INITIALIZED | APP_IMU_FLAG_TIMESTAMP_VALID | APP_IMU_FLAG_TILT_VALID;
  imu_output.flags &=
    ~((uint32_t)APP_IMU_FLAG_DATA_STALE | (uint32_t)APP_IMU_FLAG_ACCEL_UPDATE_USED |
      (uint32_t)APP_IMU_FLAG_VIBRATION_HIGH);
  imu_output.flags &= ~(uint32_t)APP_IMU_FLAG_SAMPLE_SPIKE;
  if (accel_update_used) {
    imu_output.flags |= APP_IMU_FLAG_ACCEL_UPDATE_USED;
    imu_output.accel_update_accept_count = AppImu_SaturatingIncrement(imu_output.accel_update_accept_count);
  } else {
    imu_output.accel_update_reject_count = AppImu_SaturatingIncrement(imu_output.accel_update_reject_count);
  }
  if (vibration_high) {
    imu_output.flags |= APP_IMU_FLAG_VIBRATION_HIGH;
  }
  AppImu_UpdateRecovery(timestamp_step == 1U, accel_update_used, vibration_high);
  AppImu_CopyFilterState();
  *output = imu_output;
  return BSP_OK;
}

bool AppImu_IsMotionUsable(const AppImuOutput *output, uint32_t now_ms)
{
  if (output == NULL || output->health != APP_IMU_HEALTH_HEALTHY ||
      AppImu_IsStaleAt(now_ms, output->last_good_tick_ms)) {
    return false;
  }

  const uint32_t required_flags =
    APP_IMU_FLAG_CALIBRATED | APP_IMU_FLAG_DATA_VALID | APP_IMU_FLAG_FILTER_INITIALIZED |
    APP_IMU_FLAG_FILTER_CONVERGED | APP_IMU_FLAG_TIMESTAMP_VALID | APP_IMU_FLAG_TILT_VALID;
  const uint32_t rejected_flags =
    APP_IMU_FLAG_SENSOR_FAULT | APP_IMU_FLAG_DATA_STALE | APP_IMU_FLAG_SENSOR_DEGRADED |
    APP_IMU_FLAG_RECOVERING | APP_IMU_FLAG_ESTIMATOR_FAULT | APP_IMU_FLAG_SAMPLE_SPIKE;

  /*
   * 这里不要求当前帧必须使用加速度校正，也不因 VIBRATION_HIGH 单独禁止运动；这两个
   * 条件只参与故障恢复阶段的连续稳定样本资格，避免正常加减速时把观测门控误当成故障。
   */
  return (output->flags & required_flags) == required_flags && (output->flags & rejected_flags) == 0U;
}
