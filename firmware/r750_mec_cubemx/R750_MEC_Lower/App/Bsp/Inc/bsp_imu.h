#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "bsp_types.h"

/*
 * QMI8658A 六轴 IMU 板级驱动。
 *
 * 本模块只负责 I2C1 设备探测、寄存器配置、命令握手和一致原始帧读取，不做坐标轴变换、
 * 单位换算、低通滤波、突变检测或健康分级。BSP_BUSY 表示芯片数据尚未同时就绪，并非
 * 传感器故障；上层必须通过 AppImu_Process 统一处理时间戳和故障恢复。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_IMU_QMI8658_ADDRESS_7BIT (0x6AU)

/*
 * 一次锁存原始样本。
 * acceleration、angular_rate、temperature 保持芯片有符号原始计数；sensor_timestamp
 * 只使用 QMI8658A 的 24 位计数；host_tick_ms 在完整突发读取并解码后记录 HAL 毫秒时刻；
 * status 保存 STATUS0，供上层诊断本帧加速度计和陀螺仪是否都就绪。
 */
typedef struct {
  int16_t acceleration[3];
  int16_t angular_rate[3];
  int16_t temperature;
  uint32_t sensor_timestamp;
  uint32_t host_tick_ms;
  uint8_t status;
} BspImuSample;

/*
 * 完成设备探测、身份校验、软复位、复位状态校验、关键寄存器写后回读及 CTRL9 命令握手，
 * 最后启用加速度计和陀螺仪。返回 BSP_OK 后量程和 ODR 必须与 robot_config.h 换算参数一致；
 * 任一 I2C、超时或回读不一致都会停止流程并返回对应状态。
 */
BspStatus BspImu_Init(void);
/*
 * 使用 HAL 设备就绪探测确认 7 位地址 0x6A 可应答，不校验 WHO_AM_I 内容。
 * 适用于区分总线无响应和后续寄存器配置错误。
 */
BspStatus BspImu_Probe(void);
/*
 * 有界同步读取一帧数据。
 * 先检查可用位，必要时在 2 ms 上限内同步等待数据锁存，再从 STATUS0 起突发读取状态、时间戳、
 * 温度、三轴加速度和三轴角速度。数据未齐返回 BSP_BUSY；总线失败时尝试读末寄存器释放
 * 芯片锁并返回错误；sample 为空返回 BSP_INVALID_ARG。HAL I2C 事务会短时阻塞当前 IMU
 * 任务，只有故障退避策略本身不使用阻塞延时。
 */
BspStatus BspImu_ReadSample(BspImuSample *sample);
/*
 * 以下寄存器接口供本驱动初始化、故障诊断和受控板测使用。
 * 地址长度固定为 8 位，单次 I2C 超时为 2 ms；应用任务不得绕过 ReadSample 任意读写
 * 控制寄存器，否则可能破坏锁存、ODR 与换算参数的一致性。
 */
BspStatus BspImu_ReadReg(uint8_t reg, uint8_t *value);
BspStatus BspImu_WriteReg(uint8_t reg, uint8_t value);
BspStatus BspImu_ReadRegs(uint8_t start_reg, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
