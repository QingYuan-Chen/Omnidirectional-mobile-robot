#include "bsp_imu.h"

#include "i2c.h"

/* QMI8658C 驱动严格按数据手册顺序完成复位、回读校验、命令应答和整帧读取。 */

#define BSP_IMU_I2C_TIMEOUT_MS (2U)
#define BSP_IMU_I2C_ADDRESS    (BSP_IMU_QMI8658_ADDRESS_7BIT << 1U)

#define BSP_IMU_REG_WHO_AM_I    (0x00U)
#define BSP_IMU_REG_CTRL1       (0x02U)
#define BSP_IMU_REG_CTRL2       (0x03U)
#define BSP_IMU_REG_CTRL3       (0x04U)
#define BSP_IMU_REG_CTRL5       (0x06U)
#define BSP_IMU_REG_CTRL7       (0x08U)
#define BSP_IMU_REG_CTRL9       (0x0AU)
#define BSP_IMU_REG_CAL1_LOW    (0x0BU)
#define BSP_IMU_REG_STATUS_INT  (0x2DU)
#define BSP_IMU_REG_STATUS0     (0x2EU)
#define BSP_IMU_REG_GYRO_Z_HIGH (0x40U)
#define BSP_IMU_REG_RESET_STATE (0x4DU)
#define BSP_IMU_REG_RESET       (0x60U)

#define BSP_IMU_SAMPLE_BYTE_COUNT (19U)
#define BSP_IMU_DATA_READY_MASK   (0x03U)

#define BSP_IMU_WHO_AM_I_VALUE    (0x05U)
#define BSP_IMU_SOFT_RESET_VALUE  (0xB0U)
#define BSP_IMU_RESET_READY_VALUE (0x80U)
#define BSP_IMU_CTRL9_ACK_VALUE    (0x00U)
#define BSP_IMU_CTRL9_AHB_VALUE    (0x12U)

#define BSP_IMU_CTRL1_VALUE (0x50U)
#define BSP_IMU_CTRL2_VALUE (0x15U)
#define BSP_IMU_CTRL3_VALUE (0x55U)
#define BSP_IMU_CTRL5_VALUE (0x77U)
#define BSP_IMU_CTRL7_VALUE (0x83U)

#define BSP_IMU_STATUS_INT_COMMAND_DONE (0x80U)
#define BSP_IMU_STATUS_INT_LOCKED       (0x02U)
#define BSP_IMU_STATUS_INT_AVAILABLE    (0x01U)

#define BSP_IMU_RESET_DELAY_MS   (20U)
#define BSP_IMU_STARTUP_DELAY_MS (170U)
#define BSP_IMU_CTRL9_TIMEOUT_MS (100U)
#define BSP_IMU_LOCK_TIMEOUT_MS  (2U)

static BspStatus BspImu_FromHal(HAL_StatusTypeDef status)
{
  switch (status) {
    case HAL_OK:
      return BSP_OK;
    case HAL_BUSY:
      return BSP_BUSY;
    case HAL_TIMEOUT:
      return BSP_TIMEOUT;
    default:
      return BSP_ERROR;
  }
}

static BspStatus BspImu_WriteChecked(uint8_t reg, uint8_t value)
{
  /* 关键控制寄存器写入后立即回读，拒绝静默配置失败。 */
  BspStatus status = BspImu_WriteReg(reg, value);
  if (status != BSP_OK) {
    return status;
  }

  uint8_t readback = 0U;
  status = BspImu_ReadReg(reg, &readback);
  if (status != BSP_OK) {
    return status;
  }

  return readback == value ? BSP_OK : BSP_ERROR;
}

static int16_t BspImu_DecodeInt16(const uint8_t *data)
{
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static BspStatus BspImu_WaitStatusInt(uint8_t mask, bool set, uint32_t timeout_ms)
{
  const uint32_t started_at = HAL_GetTick();
  do {
    uint8_t status_int = 0U;
    const BspStatus status = BspImu_ReadReg(BSP_IMU_REG_STATUS_INT, &status_int);
    if (status != BSP_OK) {
      return status;
    }
    if (((status_int & mask) != 0U) == set) {
      return BSP_OK;
    }
  } while ((HAL_GetTick() - started_at) < timeout_ms);

  return BSP_TIMEOUT;
}

static BspStatus BspImu_RunCtrl9Command(uint8_t command)
{
  /* 命令完成位必须经历置位、应答清零、再复位三个阶段。 */
  BspStatus status = BspImu_WriteReg(BSP_IMU_REG_CTRL9, command);
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_WaitStatusInt(BSP_IMU_STATUS_INT_COMMAND_DONE, true, BSP_IMU_CTRL9_TIMEOUT_MS);
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_WriteReg(BSP_IMU_REG_CTRL9, BSP_IMU_CTRL9_ACK_VALUE);
  if (status != BSP_OK) {
    return status;
  }
  return BspImu_WaitStatusInt(BSP_IMU_STATUS_INT_COMMAND_DONE, false, BSP_IMU_CTRL9_TIMEOUT_MS);
}

static BspStatus BspImu_DisableAhbClockGating(void)
{
  BspStatus status = BspImu_WriteReg(BSP_IMU_REG_CAL1_LOW, 0x01U);
  if (status != BSP_OK) {
    return status;
  }
  return BspImu_RunCtrl9Command(BSP_IMU_CTRL9_AHB_VALUE);
}

BspStatus BspImu_Init(void)
{
  /* 先探测和核对身份，再软复位；配置期间保持传感器输出关闭。 */
  BspStatus status = BspImu_Probe();
  if (status != BSP_OK) {
    return status;
  }

  uint8_t device_id = 0U;
  status = BspImu_ReadReg(BSP_IMU_REG_WHO_AM_I, &device_id);
  if (status != BSP_OK || device_id != BSP_IMU_WHO_AM_I_VALUE) {
    return status == BSP_OK ? BSP_ERROR : status;
  }

  status = BspImu_WriteReg(BSP_IMU_REG_RESET, BSP_IMU_SOFT_RESET_VALUE);
  if (status != BSP_OK) {
    return status;
  }
  HAL_Delay(BSP_IMU_RESET_DELAY_MS);

  uint8_t reset_state = 0U;
  status = BspImu_ReadReg(BSP_IMU_REG_RESET_STATE, &reset_state);
  if (status != BSP_OK || reset_state != BSP_IMU_RESET_READY_VALUE) {
    return status == BSP_OK ? BSP_ERROR : status;
  }

  status = BspImu_ReadReg(BSP_IMU_REG_WHO_AM_I, &device_id);
  if (status != BSP_OK || device_id != BSP_IMU_WHO_AM_I_VALUE) {
    return status == BSP_OK ? BSP_ERROR : status;
  }

  /* 逐项配置量程、输出率和滤波参数，最后才统一启用加速度计和陀螺仪。 */
  status = BspImu_WriteChecked(BSP_IMU_REG_CTRL7, 0U);
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_WriteChecked(BSP_IMU_REG_CTRL1, BSP_IMU_CTRL1_VALUE);
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_WriteChecked(BSP_IMU_REG_CTRL2, BSP_IMU_CTRL2_VALUE);
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_WriteChecked(BSP_IMU_REG_CTRL3, BSP_IMU_CTRL3_VALUE);
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_WriteChecked(BSP_IMU_REG_CTRL5, BSP_IMU_CTRL5_VALUE);
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_DisableAhbClockGating();
  if (status != BSP_OK) {
    return status;
  }
  status = BspImu_WriteChecked(BSP_IMU_REG_CTRL7, BSP_IMU_CTRL7_VALUE);
  if (status != BSP_OK) {
    return status;
  }

  HAL_Delay(BSP_IMU_STARTUP_DELAY_MS);
  return BSP_OK;
}

BspStatus BspImu_Probe(void)
{
  return BspImu_FromHal(HAL_I2C_IsDeviceReady(&hi2c1, BSP_IMU_I2C_ADDRESS, 2U, BSP_IMU_I2C_TIMEOUT_MS));
}

BspStatus BspImu_ReadSample(BspImuSample *sample)
{
  if (sample == NULL) {
    return BSP_INVALID_ARG;
  }

  uint8_t status_int = 0U;
  BspStatus status = BspImu_ReadReg(BSP_IMU_REG_STATUS_INT, &status_int);
  if (status != BSP_OK) {
    return status;
  }
  if ((status_int & BSP_IMU_STATUS_INT_AVAILABLE) == 0U) {
    return BSP_BUSY;
  }
  if ((status_int & BSP_IMU_STATUS_INT_LOCKED) == 0U) {
    /* 等待锁存保证本次突发读取中的各轴和时间戳属于同一采样时刻。 */
    status = BspImu_WaitStatusInt(BSP_IMU_STATUS_INT_LOCKED, true, BSP_IMU_LOCK_TIMEOUT_MS);
    if (status != BSP_OK) {
      return status;
    }
  }

  uint8_t raw[BSP_IMU_SAMPLE_BYTE_COUNT];
  status = BspImu_ReadRegs(BSP_IMU_REG_STATUS0, raw, sizeof(raw));
  if (status != BSP_OK) {
    /* 读取末寄存器可释放芯片数据锁，避免一次总线错误阻塞后续样本。 */
    uint8_t release_lock = 0U;
    (void)BspImu_ReadReg(BSP_IMU_REG_GYRO_Z_HIGH, &release_lock);
    return status;
  }

  sample->status = raw[0];
  sample->sensor_timestamp = (uint32_t)raw[2] | ((uint32_t)raw[3] << 8U) | ((uint32_t)raw[4] << 16U);
  sample->temperature = BspImu_DecodeInt16(&raw[5]);
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    sample->acceleration[axis] = BspImu_DecodeInt16(&raw[7U + axis * 2U]);
    sample->angular_rate[axis] = BspImu_DecodeInt16(&raw[13U + axis * 2U]);
  }
  sample->host_tick_ms = HAL_GetTick();

  return (sample->status & BSP_IMU_DATA_READY_MASK) == BSP_IMU_DATA_READY_MASK ? BSP_OK : BSP_BUSY;
}

BspStatus BspImu_ReadReg(uint8_t reg, uint8_t *value)
{
  if (value == NULL) {
    return BSP_INVALID_ARG;
  }

  return BspImu_FromHal(
    HAL_I2C_Mem_Read(&hi2c1, BSP_IMU_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, value, 1U, BSP_IMU_I2C_TIMEOUT_MS));
}

BspStatus BspImu_WriteReg(uint8_t reg, uint8_t value)
{
  return BspImu_FromHal(
    HAL_I2C_Mem_Write(&hi2c1, BSP_IMU_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, BSP_IMU_I2C_TIMEOUT_MS));
}

BspStatus BspImu_ReadRegs(uint8_t start_reg, uint8_t *data, uint16_t length)
{
  if (data == NULL || length == 0U) {
    return BSP_INVALID_ARG;
  }

  return BspImu_FromHal(
    HAL_I2C_Mem_Read(&hi2c1, BSP_IMU_I2C_ADDRESS, start_reg, I2C_MEMADD_SIZE_8BIT, data, length, BSP_IMU_I2C_TIMEOUT_MS));
}
