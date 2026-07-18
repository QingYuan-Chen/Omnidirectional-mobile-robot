#ifndef APP_IMU_H
#define APP_IMU_H

#include "bsp_imu.h"
#include "bsp_types.h"

/*
 * IMU 应用处理与健康管理接口。
 *
 * 本层位于 QMI8658A 寄存器驱动和姿态 ESKF 之间，负责静止标定、坐标轴变换、SI 单位
 * 换算、时间戳连续性、突变拒绝、故障分级、非阻塞退避和稳定恢复。输出同时保留原始
 * 换算值与当前候选截止频率为 20 Hz 的独立低通值；ESKF 只接收未经过普通低通且通过质量门控的本次样本，
 * 避免额外相位延迟污染估计器输入。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 输出状态位，可组合出现。
 * DATA_VALID 是控制、估计和遥测等数据消费者的最终数值可用门；SENSOR_DEGRADED、
 * SENSOR_FAULT、RECOVERING 和 ESTIMATOR_FAULT 解释不可用原因。当前安全任务只检查
 * health，尚需单独修正陈旧样本与安全门的联动。ABSOLUTE_YAW_VALID 当前始终清零，因为
 * 六轴 IMU 没有绝对航向观测；TILT_VALID 只表示横滚和俯仰可用。
 */
typedef enum {
  APP_IMU_FLAG_SENSOR_PRESENT = (1UL << 0U),
  APP_IMU_FLAG_CALIBRATED = (1UL << 1U),
  APP_IMU_FLAG_DATA_VALID = (1UL << 2U),
  APP_IMU_FLAG_FILTER_INITIALIZED = (1UL << 3U),
  APP_IMU_FLAG_FILTER_CONVERGED = (1UL << 4U),
  APP_IMU_FLAG_ACCEL_UPDATE_USED = (1UL << 5U),
  APP_IMU_FLAG_VIBRATION_HIGH = (1UL << 6U),
  APP_IMU_FLAG_SENSOR_FAULT = (1UL << 7U),
  APP_IMU_FLAG_DATA_STALE = (1UL << 8U),
  APP_IMU_FLAG_ABSOLUTE_YAW_VALID = (1UL << 9U),
  APP_IMU_FLAG_TIMESTAMP_VALID = (1UL << 10U),
  APP_IMU_FLAG_TILT_VALID = (1UL << 11U),
  APP_IMU_FLAG_SAMPLE_SPIKE = (1UL << 12U),
  APP_IMU_FLAG_SENSOR_DEGRADED = (1UL << 13U),
  APP_IMU_FLAG_RECOVERING = (1UL << 14U),
  APP_IMU_FLAG_ESTIMATOR_FAULT = (1UL << 15U)
} AppImuFlags;

/*
 * 互斥健康等级，供安全任务作粗粒度决策。
 * 短暂总线失败或单次突变进入 TRANSIENT_DEGRADED；连续总线读错或连续突变升级为
 * PERSISTENT_SENSOR_FAULT。重复时间戳只返回 BUSY，过大时间戳跳变单次进入降级，不计入
 * 上述连续升级计数。重新取得样本后必须经过 RECOVERING 的稳定门槛，不能一帧就恢复。
 * ESTIMATOR_FAULT 独立于传感器总线故障。
 */
typedef enum {
  APP_IMU_HEALTH_UNINITIALIZED = 0,
  APP_IMU_HEALTH_HEALTHY,
  APP_IMU_HEALTH_TRANSIENT_DEGRADED,
  APP_IMU_HEALTH_PERSISTENT_SENSOR_FAULT,
  APP_IMU_HEALTH_RECOVERING,
  APP_IMU_HEALTH_ESTIMATOR_FAULT
} AppImuHealth;

/*
 * IMU 完整运行快照。
 *
 * raw_sample 是芯片原始帧；acceleration_mps2 与 angular_rate_rad_s 是车体坐标系 SI 值，
 * 其中对外角速度已扣除 ESKF 零偏；filtered_* 供控制和遥测等处理后消费者使用，不得
 * 替代 raw_sample 或回灌 ESKF；
 * quaternion 顺序为 [w,x,y,z]，euler_rad 顺序为横滚、俯仰、航向且单位为弧度。
 *
 * host_tick_ms 是样本读取时刻，last_good_tick_ms 是最后一次通过全部质量检查的处理时刻，
 * sample_age_ms 每次调用都由当前时刻减去 last_good_tick_ms 得到。sequence 只在接受新样本
 * 后递增。retry_delay_ms 与 next_retry_tick_ms 描述当前非阻塞退避窗口；退避期间任务仍
 * 调用 Process、发布这份快照并上报心跳。其余计数均为饱和累计诊断量。
 */
typedef struct {
  BspImuSample raw_sample;
  float acceleration_mps2[3];
  float angular_rate_rad_s[3];
  float filtered_acceleration_mps2[3];
  float filtered_angular_rate_rad_s[3];
  float quaternion[4];
  float euler_rad[3];
  float gyro_bias_rad_s[3];
  float temperature_celsius;
  AppImuHealth health;
  uint32_t flags;
  uint32_t sequence;
  uint32_t sensor_timestamp;
  uint32_t host_tick_ms;
  uint32_t last_good_tick_ms;
  uint32_t sample_age_ms;
  uint32_t read_error_count;
  uint32_t consecutive_error_count;
  uint32_t backoff_count;
  uint32_t retry_delay_ms;
  uint32_t next_retry_tick_ms;
  uint32_t duplicate_count;
  uint32_t dropped_sample_count;
  uint32_t accel_update_accept_count;
  uint32_t accel_update_reject_count;
  uint32_t spike_reject_count;
  uint32_t consecutive_spike_count;
  uint32_t stable_sample_count;
  uint32_t estimator_fault_count;
} AppImuOutput;

/*
 * 启动阶段静止标定。
 * 函数同步采集配置数量的互不重复样本，用在线均值/方差检查加速度重力一致性、陀螺
 * 静止程度和噪声上限，然后用平均重力方向和陀螺均值初始化 ESKF。标定期间机器人必须
 * 静止；运动会重置本轮统计。成功返回 BSP_OK，超时、总线错误或质量不合格返回相应状态。
 */
BspStatus AppImu_Calibrate(void);
/*
 * 在 IMU 任务中处理一次机会，now_ms 使用 HAL 毫秒时基。
 * 函数最多执行一次有界同步读取和一次估计更新；处于退避窗口或数据未就绪时返回 BSP_BUSY，
 * 质量、时间戳、传感器或估计器异常返回错误状态。只要 output 有效，无论返回值如何都会
 * 写入最新快照并更新 sample_age_ms，调用者不得因 BUSY 或错误而停止 IMU 任务心跳。
 */
BspStatus AppImu_Process(uint32_t now_ms, AppImuOutput *output);
/*
 * 使用调用时刻重新核验 IMU 是否可作为运动许可条件。
 * 判据要求健康、标定、有效、ESKF 初始化并收敛、时间戳与倾角有效，且按 last_good_tick_ms
 * 计算的实时年龄严格小于 20 ms。调用者不得只信任快照中的 sample_age_ms，因为快照发布
 * 与实际控制时刻之间仍可能经过数毫秒。正常健康运行不要求每帧都使用加速度观测，也不因
 * 单帧高振动直接禁止运动；严格加速度与振动条件仅用于故障恢复门槛。
 */
bool AppImu_IsMotionUsable(const AppImuOutput *output, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
