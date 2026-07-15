#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "bsp_types.h"

/* QMI8658C 板级驱动只负责寄存器配置和原始样本读取，不承担滤波与健康判定。 */

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_IMU_QMI8658_ADDRESS_7BIT (0x6AU)

/* 一次传感器原始样本，同时记录芯片时间戳和主机读取时刻。 */
typedef struct {
  int16_t acceleration[3];
  int16_t angular_rate[3];
  int16_t temperature;
  uint32_t sensor_timestamp;
  uint32_t host_tick_ms;
  uint8_t status;
} BspImuSample;

/* 校验芯片身份并配置量程、输出数据率、中断和时间戳。 */
BspStatus BspImu_Init(void);
/* 仅读取身份寄存器确认设备在线。 */
BspStatus BspImu_Probe(void);
/* 数据未就绪返回忙，成功时一次性读取完整原始样本。 */
BspStatus BspImu_ReadSample(BspImuSample *sample);
/* 以下寄存器接口供初始化、诊断和受控板测使用。 */
BspStatus BspImu_ReadReg(uint8_t reg, uint8_t *value);
BspStatus BspImu_WriteReg(uint8_t reg, uint8_t value);
BspStatus BspImu_ReadRegs(uint8_t start_reg, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
