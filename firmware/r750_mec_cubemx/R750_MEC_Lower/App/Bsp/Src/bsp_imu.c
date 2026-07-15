#include "bsp_imu.h"

#include "i2c.h"

/*
 * QMI8658A 驱动按数据手册时序完成设备探测、软复位、关键配置回读、CTRL9 命令握手和
 * 锁存整帧读取。本层保持寄存器值与原始数据，不解释物理单位；robot_config.h 中的 ODR
 * 和量程换算必须与 CTRL2/CTRL3 固定值同步维护。所有 I2C 事务都有毫秒上限，错误原样
 * 映射为统一 BSP 状态，便于应用层决定退避和故障分级。
 */

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

/*
 * CTRL1 配置地址自动递增与中断行为，CTRL2/CTRL3 选择加速度/陀螺量程和 ODR，CTRL5
 * 配置芯片内部滤波，CTRL7 最后统一启用传感器。寄存器常量来自当前硬件方案，修改时需
 * 对照数据手册并同步上层 LSB 换算，不能只改其中一侧。
 */

static BspStatus BspImu_FromHal(HAL_StatusTypeDef status)
{
  /* 保留 BUSY 与 TIMEOUT 语义，上层才能区分可重试状态和一般硬件错误。 */
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
  /*
   * 对可直接回读的关键控制寄存器执行“写→读→逐字节相等”校验。I2C ACK 只证明总线事务
   * 完成，不能证明芯片实际接受了配置，因此回读不一致按 BSP_ERROR 处理。
   */
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
  /* QMI8658A 多字节数据按小端序排列，先组合无符号位型再解释为二补码有符号值。 */
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static BspStatus BspImu_WaitStatusInt(uint8_t mask, bool set, uint32_t timeout_ms)
{
  /* 使用 HAL 毫秒时基轮询指定位达到期望电平；循环内每次 I2C 读取自身也受 2 ms 上限约束。 */
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
  /*
   * CTRL9 命令协议不是普通寄存器写：写命令后等待 COMMAND_DONE=1，再写 0 确认，最后等待
   * COMMAND_DONE=0。缺少最后阶段可能让下一条命令误读上一次完成状态。
   */
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
  /* 按芯片推荐序列写 CAL1_LOW 并执行 CTRL9 0x12，保证后续数据寄存器访问时内部总线可用。 */
  BspStatus status = BspImu_WriteReg(BSP_IMU_REG_CAL1_LOW, 0x01U);
  if (status != BSP_OK) {
    return status;
  }
  return BspImu_RunCtrl9Command(BSP_IMU_CTRL9_AHB_VALUE);
}

BspStatus BspImu_Init(void)
{
  /*
   * IsDeviceReady 先确认地址应答，WHO_AM_I 再确认应答者确为目标芯片。软复位后等待固定
   * 20 ms，并同时检查 RESET_STATE 与 WHO_AM_I，防止在复位未完成或总线上有错误设备时配置。
   */
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

  /*
   * 先写 CTRL7=0 关闭加速度计/陀螺仪，再配置公共控制、两类量程/ODR 和内部滤波；完成
   * AHB 命令后才写最终 CTRL7 启用。每一步都立即回读，任一失败不继续执行后续步骤。
   */
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
    /* 没有可用帧是正常非阻塞状态，不计为总线错误。 */
    return BSP_BUSY;
  }
  if ((status_int & BSP_IMU_STATUS_INT_LOCKED) == 0U) {
    /*
     * AVAILABLE 表示数据可取，LOCKED 表示本帧寄存器已冻结。最多等待 2 ms 取得锁，确保
     * 随后的 19 字节突发读取不会跨越一次芯片内部数据更新。
     */
    status = BspImu_WaitStatusInt(BSP_IMU_STATUS_INT_LOCKED, true, BSP_IMU_LOCK_TIMEOUT_MS);
    if (status != BSP_OK) {
      return status;
    }
  }

  uint8_t raw[BSP_IMU_SAMPLE_BYTE_COUNT];
  status = BspImu_ReadRegs(BSP_IMU_REG_STATUS0, raw, sizeof(raw));
  if (status != BSP_OK) {
    /*
     * 正常突发读取覆盖到 GYRO_Z_HIGH，会自然释放数据锁。若中途 I2C 失败，额外读取末寄存器
     * 尝试释放锁；释放失败不覆盖原始错误，应用层会按原错误启动退避。
     */
    uint8_t release_lock = 0U;
    (void)BspImu_ReadReg(BSP_IMU_REG_GYRO_Z_HIGH, &release_lock);
    return status;
  }

  sample->status = raw[0];
  /* raw[1] 为保留/未使用字节；24 位时间戳、温度、加速度和角速度随后按固定偏移解码。 */
  sample->sensor_timestamp = (uint32_t)raw[2] | ((uint32_t)raw[3] << 8U) | ((uint32_t)raw[4] << 16U);
  sample->temperature = BspImu_DecodeInt16(&raw[5]);
  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    sample->acceleration[axis] = BspImu_DecodeInt16(&raw[7U + axis * 2U]);
    sample->angular_rate[axis] = BspImu_DecodeInt16(&raw[13U + axis * 2U]);
  }
  sample->host_tick_ms = HAL_GetTick();

  /* 只有加速度计与陀螺仪两类 data-ready 位同时有效，整帧才交给应用层。 */
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
