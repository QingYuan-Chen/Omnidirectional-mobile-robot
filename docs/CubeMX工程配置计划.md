# R750-MEC 下位机 CubeMX 工程配置计划

> 历史搭建文档：本文记录初期工程生成依据。当前控制架构、任务和频率决策以 `task_plan.md`、`plan_review_and_directives.md`、`FreeRTOS框架规划.md` 和 `m2_control_architecture_gate.md` 为准。

## 目标

为 R750 金属麦克纳姆轮底盘创建一个新的 STM32 下位机工程。原厂 Keil 源码只作为参考，不直接改造。

正式工程应由 STM32CubeMX 生成启动代码、HAL 初始化、FreeRTOS 配置、外设初始化和 CMake/Makefile 工程骨架。业务代码只放在用户模块中，避免污染 CubeMX 生成区。

## 已确认事实

- MCU: STM32F407VET6 / STM32F407VETx。
- 底盘: R750-MEC，四麦克纳姆轮。
- 轮子: 金属麦轮，直径 0.077 m。
- 轮距 W: 0.196 m。
- 轴距 H: 0.160 m。
- 整车质量: 空载 2.2 kg，最大额外载荷约 10 kg，最大总质量按约 12.2 kg 规划。
- 电池: LB350E，3S 21700，5000 mAh，标称 11.1 V，满电 12.6 V，最大持续/瞬时放电电流 8/16 A。
- 电机: 塔克 MC520P30_12V，12 V，1:30 减速比，1024 PPR 磁编码器版本。
- 编码器轮端分辨率：2026-07-18 MA 三圈完整平台实测平均 123927.7 counts / wheel rev，与原厂 1:30 四倍频的 122880 偏差约 0.85%，固件据此采用 122880 counts / wheel rev。
- ROS 串口波特率: 230400。
- 上位机通信: 当前阶段只做下位机，不构建 ROS2 节点。

## 工具状态

- 已找到 STM32CubeCLT:
  - `D:/STM32CubeCLT_1.20.0/CMake/bin/cmake.exe`
  - `D:/STM32CubeCLT_1.20.0/Ninja/bin/ninja.exe`
  - `D:/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe`
- 已找到 STM32CubeF4 固件包:
  - `C:/Users/MECHREU/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3`
- 已找到并确认后台打开 STM32CubeMX:
  - `E:/STM32CubeMax/STM32CubeMX.exe`
  - 当前版本: 6.15.0

## 正式工程目录建议

```text
firmware/
  vendor_reference/                  # 原厂 Keil 源码，只读参考
  r750_mec_cubemx/                   # CubeMX 正式工程
    R750_MEC_Lower.ioc               # 根配置文件
    cubemx_generate.txt              # CubeMX 命令行生成脚本
    R750_MEC_Lower/                  # CubeMX 生成的 CMake 工程
      R750_MEC_Lower.ioc
      Core/                          # CubeMX 生成
      Drivers/                       # CubeMX 生成
      Middlewares/                   # CubeMX 生成 FreeRTOS
      cmake/
      CMakeLists.txt
      App/                           # 后续用户业务代码建议放这里
        Inc/
        Src/
```

## 当前生成与验证状态

- 已用 STM32CubeMX 6.15.0 命令行生成 CMake/GCC 工程。
- 生成脚本: `firmware/r750_mec_cubemx/cubemx_generate.txt`。
- 生成工程: `firmware/r750_mec_cubemx/R750_MEC_Lower`。
- 已验证 `Debug` 构建通过，产物为:
  - `firmware/r750_mec_cubemx/R750_MEC_Lower/build/Debug/R750_MEC_Lower.elf`
  - `firmware/r750_mec_cubemx/R750_MEC_Lower/build/Debug/R750_MEC_Lower.map`
- 已补充 CLion 根目录入口:
  - `CMakeLists.txt`
  - `CMakePresets.json`
  - CLion 可直接打开项目根目录 `C:/Users/MECHREU/Desktop/Mobile_robot_lower_level_machine`。
- 已加入最小 `App` 层:
  - `App/Inc/app.h`
  - `App/Inc/robot_config.h`
  - `App/Src/app.c`
- 当前默认任务已经转发到 `App_DefaultTask()`，并以 LED1 作为 500 ms 心跳占位。
- 已加入第一版 BSP 编程框架:
  - `App/Bsp/Inc/bsp*.h`
  - `App/Bsp/Src/bsp*.c`
  - 已覆盖电机 PWM、编码器、UART、ADC、电池电压、蜂鸣器、IMU I2C 基础访问。
  - `App_DefaultTask()` 当前调用 `Bsp_Init()` 完成 BSP 初始化。

复现生成:

```powershell
cd E:\STM32CubeMax
.\jre\bin\java.exe -jar .\STM32CubeMX.exe -q "C:\Users\MECHREU\Desktop\移动机器人下位机\firmware\r750_mec_cubemx\cubemx_generate.txt"
```

复现构建:

```powershell
cd "C:\Users\MECHREU\Desktop\Mobile_robot_lower_level_machine"
$env:Path="D:\STM32CubeCLT_1.20.0\GNU-tools-for-STM32\bin;D:\STM32CubeCLT_1.20.0\Ninja\bin;$env:Path"
D:\STM32CubeCLT_1.20.0\CMake\bin\cmake.exe --preset Debug
D:\STM32CubeCLT_1.20.0\CMake\bin\cmake.exe --build --preset Debug
```

## CubeMX 基础设置

- MCU 选择器：`STM32F407VETx`。
- 芯片封装：LQFP100。
- 项目管理：
  - 工具链/IDE：优先选择 CMake；若 CubeMX 版本不支持 CMake，则选择 Makefile 后由 CubeCLT/CMake 适配。
  - 每个外设分别生成一对 `.c/.h` 初始化文件：启用。
  - 重新生成时保留用户代码：启用。
- 中间件：
  - FreeRTOS: CMSIS-V1 或 Native FreeRTOS 均可。建议 Native FreeRTOS，业务层更直接。

## 时钟树

需用 H60 原理图确认外部晶振频率后再在 CubeMX 固定。原厂 `system_stm32f4xx.c` 有多套分支，不能只凭源码片段断定。

目标运行频率:

- SYSCLK：168 MHz。
- HCLK：168 MHz。
- APB1：42 MHz。
- APB2：84 MHz。
- FreeRTOS 系统节拍：1 kHz。

## 外设配置

### 电机 PWM

参考原厂源码和 H60 手册，四路电机每路使用两个 PWM 输入控制 AT8236。

| 电机 | 功能 | 定时器通道 | 引脚 |
|---|---|---|---|
| MA | PWM IN1/IN2 | TIM1_CH1 / TIM1_CH2 | PE9 / PE11 |
| MB | PWM IN1/IN2 | TIM1_CH3 / TIM1_CH4 | PE13 / PE14 |
| MC | PWM IN1/IN2 | TIM9_CH1 / TIM9_CH2 | PE5 / PE6 |
| MD | PWM IN1/IN2 | TIM12_CH1 / TIM12_CH2 | PB14 / PB15 |

当前基线与候选:

- PWM 频率：当前 20 kHz；这是 M2 决策门的起测值，不是最终冻结值。
- PWM 周期：沿用原厂 4200 计数，或用 CubeMX 计算等效 20 kHz 周期。
- PWM 模式：PWM Generation CHx。
- 输出极性：高电平有效。

### 编码器

| 电机 | 编码器定时器 | 引脚 |
|---|---|---|
| MA | TIM2_CH1 / TIM2_CH2 | PA15 / PB3 |
| MB | TIM3_CH1 / TIM3_CH2 | PB4 / PB5 |
| MC | TIM5_CH1 / TIM5_CH2 | PA0 / PA1 |
| MD | TIM4_CH1 / TIM4_CH2 | PD12 / PD13 |

建议:

- 编码器模式：TI1 和 TI2。
- 计数器周期：最大值。
- 输入滤波：先参考原厂滤波值 6，在 CubeMX 中按可选项配置。
- 控制周期：历史 20 ms 建议已废止。当前软件占位为 10 ms；M2 以硬件定时器驱动的 1 kHz 作为首轮候选，最终由辨识和鲁棒性评测决定。

### ROS 串口

| 接口 | 用途 | 引脚 | 波特率 |
|---|---|---|---|
| USART2 | ROS USB 通信串口 | PD5 TX / PD6 RX | 230400 |
| UART4 | 人类调试、辨识采集和临时调参 | PC10 TX / PC11 RX | 230400 |

最终角色仍为 USART2 承载 ROS/X-Protocol、UART4 承载人类调试。M2 板测期间，应用层集中配置临时选择 Type-C 对应的 USART2 运行 ASCII 命令和遥测；本次不修改 CubeMX 引脚或波特率。M5 启用 X-Protocol 前必须把调试链迁回 UART4，不在 USART2 同时自动探测两种协议。

建议:

- 8 个数据位。
- 无奇偶校验。
- 1 个停止位。
- 启用 RX 接收中断。
- USART2/UART4 TX 均使用按端口独立的中断驱动非阻塞帧队列；DMA + IDLE 仍留到 M6 评审。

### IMU QMI8658A

| 功能 | 引脚 |
|---|---|
| I2C SCL | PB6 |
| I2C SDA | PB7 |
| INT | PD7 |

建议:

- I2C1。
- 400 kHz Fast Mode。
- QMI8658A 地址按原厂源码先使用 `0x6A`。
- 加速度量程: +/-4g。
- 陀螺仪量程: +/-512 dps。
- ODR: 500 Hz。
- 下位机任务中 50 Hz 读取并回传，内部可保留更高采样空间。

### 电池电压 ADC

| 功能 | 引脚 |
|---|---|
| VIN ADC | PC0 / ADC2_IN10 |

参考原厂 `ax_vin.c`，电池电压采样使用 ADC2 regular channel 10；H60 手册中该脚对应 PC0 / ADC123_IN10。电压分压比为 1/11。

建议:

- ADC2 regular channel IN10。
- 软件触发。
- 低频采样，100-500 ms 更新一次。
- 回传单位沿用原厂 centivolt: `12.34V -> 1234`。

### 蜂鸣器、LED、按键

| 功能 | 引脚 |
|---|---|
| 蜂鸣器 | PE0 |
| 用户按键 | PE1 |
| LED1 | PB0 |
| LED2 | PB1 |

第一版保留蜂鸣器和 LED 状态提示。按键可后续用于 IMU 校准或参数恢复。

## FreeRTOS 任务计划

| 任务 | 周期/触发 | 优先级建议 | 职责 |
|---|---:|---:|---|
| MotorTask | 20 ms | High | 编码器读取、麦轮正/逆解、PID、PWM 输出 |
| CommRxTask | UART RX interrupt + queue | High | 解析 X-Protocol，更新速度目标和心跳 |
| CommTxTask | 20 ms 或 50 ms | Normal | 回传 IMU、底盘速度、电池、电机状态 |
| ImuTask | 20 ms | Normal | 读取 QMI8658A，零偏处理，坐标转换 |
| SafetyTask | 20 ms | High | ROS 超时停车、限速、低电压保护 |
| BuzzerTask | event | Low | 状态提示音 |
| MonitorTask | 500 ms | Low | LED 心跳、电池低电压提示 |

第一版可合并为 2-3 个任务，避免过早复杂化:

- `RobotTask`: 20 ms，控制 + IMU + 安全。
- `CommTask`: 队列驱动，串口协议解析。
- `MonitorTask`: 500 ms，电池/LED/蜂鸣器。

## 业务模块边界

```text
App/Inc, App/Src
  robot_config        # R750-MEC 参数
  xprotocol           # AA 55 X-Protocol
  mecanum_kinematics  # 麦克纳姆正逆解
  encoder_service     # 编码器 delta -> m/s
  motor_control       # PID + PWM 输出
  imu_service         # QMI8658A 读取、零偏、坐标转换
  safety              # 心跳、限速、低电压、急停
  robot_task          # FreeRTOS 任务编排
```

规则:

- `App` 里的运动学和协议代码尽量不直接依赖 HAL。
- HAL 依赖集中在 `bsp_*` 或 CubeMX 生成的外设文件里。
- CubeMX 生成区只在 `USER CODE BEGIN/END` 内写必要挂钩。

## 通信协议第一版

沿用原厂 X-Protocol:

```text
AA 55 | frame_len | frame_id | payload | checksum
```

保留兼容帧:

- `0x10`: 下位机 -> 上位机，基础综合数据。
- `0x50`: 上位机 -> 下位机，速度 `vx/vy/wz`。
- `0x51`: 上位机 -> 下位机，IMU 零偏校准。
- `0x54`: 上位机 -> 下位机，蜂鸣器控制。

新增建议帧:

- `0x11`: 下位机 -> 上位机，四轮目标速度、实际速度、PWM。
- `0x12`: 下位机 -> 上位机，系统状态、错误码、超时状态。

## 安全策略

- ROS 速度帧超时 500 ms 自动停车。
- `vx/vy/wz` 限幅:
  - `vx`: +/-1.5 m/s。
  - `vy`: +/-1.2 m/s。
  - `wz`: +/-6.28 rad/s。
- PWM 限幅，默认沿用原厂 +/-4200。
- 低电压阈值沿用 R750 手册:
  - 11.19V: 低电提示。
  - 10.86V: 强提示。
  - 10.50V: 停车保护。

## 下一个实际动作

1. 加入 FreeRTOS 任务骨架和串口协议解析。
2. 做一次板上 bring-up: LED 心跳、蜂鸣器、串口回环、ADC 电压、编码器计数和 PWM 空载测试。
3. 在 bring-up 通过后再实现麦轮运动学、速度闭环和安全停车逻辑。
