#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_IMU_QMI8658_ADDRESS_7BIT (0x6AU)

typedef struct {
  int16_t acceleration[3];
  int16_t angular_rate[3];
  int16_t temperature;
  uint32_t sensor_timestamp;
  uint32_t host_tick_ms;
  uint8_t status;
} BspImuSample;

BspStatus BspImu_Init(void);
BspStatus BspImu_Probe(void);
BspStatus BspImu_ReadSample(BspImuSample *sample);
BspStatus BspImu_ReadReg(uint8_t reg, uint8_t *value);
BspStatus BspImu_WriteReg(uint8_t reg, uint8_t value);
BspStatus BspImu_ReadRegs(uint8_t start_reg, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
