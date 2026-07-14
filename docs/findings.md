# 项目发现与结论

## 代码库基线（2026-07-11）

### FreeRTOS 配置
- `configUSE_PREEMPTION=1`, `configTICK_RATE_HZ=1000`
- 当前选用的 GCC ARM_CM4F FreeRTOS 移植层不使用 `configENABLE_FPU=0`。工具链参数为 `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`；`port.c` 已启用 VFP 和浮点上下文惰性保存。该项已验证关闭。
- `configCHECK_FOR_STACK_OVERFLOW=2`
- `configMAX_PRIORITIES=56`，堆大小为 32 KB
- 已启用 CMSIS-RTOS v2 封装层

### 任务布局
| 任务 | 优先级 | 周期 | 栈大小 |
|------|----------|--------|-------|
| `controlTask` | 高 | 10 ms（当前占位） | 1536 B |
| `safetyTask` | 高 | 100 ms | 1024 B |
| `commTask` | 较高 | 5 ms | 1536 B |
| `imuTask` | 较高 | DRDY/10 ms | 1536 B |
| `monitorTask` | 低 | 500 ms | 1024 B |

### 当前 `AppImuOutput` 标志
- `SENSOR_PRESENT`、`CALIBRATED`、`DATA_VALID`、`FILTER_INITIALIZED`
- `FILTER_CONVERGED`、`ACCEL_UPDATE_USED`、`VIBRATION_HIGH`
- `SENSOR_FAULT`、`DATA_STALE`、`ABSOLUTE_YAW_VALID`、`TIMESTAMP_VALID`、`TILT_VALID`
- `SAMPLE_SPIKE`、`SENSOR_DEGRADED`、`RECOVERING`、`ESTIMATOR_FAULT`

### IMU 数据与标志语义

| 字段/标志 | 含义 | 控制链用途 |
|---|---|---|
| `raw_sample` | 未修改的 QMI8658A 同步寄存器原始值 | 不进入控制，仅用于诊断和遥测 |
| `acceleration_mps2` | 估计器使用的当前机体系 SI 加速度 | 仅在处理后有效性条件满足时使用 |
| `angular_rate_rad_s` | 已扣除估计陀螺零偏的当前机体系角速度 | 仅在处理后有效性条件满足时使用 |
| `quaternion`、`euler_rad` | ESKF 姿态输出 | 需要 `DATA_VALID`、`FILTER_INITIALIZED`、`TIMESTAMP_VALID` 及对应姿态有效标志 |
| `SENSOR_PRESENT` | 传感器已检测并初始化 | 仅是诊断前提，不代表数据有效 |
| `CALIBRATED` | 启动静止标定已完成 | 处理后输出的必要条件 |
| `DATA_VALID` | 当前处理样本及估计器状态可用 | 控制和安全消费者的必要条件 |
| `FILTER_INITIALIZED` | ESKF 名义状态和误差状态已初始化 | 姿态输出的必要条件 |
| `FILTER_CONVERGED` | 已达到最低有效重力更新次数 | 就绪和诊断门槛 |
| `ACCEL_UPDATE_USED` | 当前加速度样本通过重力观测门控 | 用于诊断；单次拒绝不构成传感器故障 |
| `VIBRATION_HIGH` | 加速度模长表明振动或动态运动较强 | 诊断和降级输入 |
| `SENSOR_FAULT` | 当前策略下出现持久的传感器访问或数据故障 | 禁止消费处理后数据 |
| `DATA_STALE` | 没有足够新的有效样本 | 禁止消费处理后数据 |
| `TIMESTAMP_VALID` | 传感器时间推进正常 | 处理后消费者的必要条件 |
| `TILT_VALID` | 横滚角和俯仰角可观且有效 | 使用倾角时的必要条件 |
| `ABSOLUTE_YAW_VALID` | 绝对航向可观 | 当前仅使用六轴 IMU 的估计器中始终为假 |

M1 已增加名称明确的滤波加速度和滤波角速度字段，供控制与遥测使用；这些字段不会替代 `raw_sample`，也不会进入 ESKF 输入链路。

### M1 故障与恢复语义

| 状态 | 含义 | 安全行为 |
|---|---|---|
| `HEALTHY` | 当前样本和已初始化的有限估计器通过处理后数据门控 | 允许使用处理后的 IMU 输出 |
| `TRANSIENT_DEGRADED` | 单次传感器访问、时间戳或突变错误影响当前样本 | 禁止运动，但不永久锁存 |
| `PERSISTENT_SENSOR_FAULT` | 连续传感器或突变失败达到配置门槛 | 保持运动禁止，同时继续重试和上报心跳 |
| `ESTIMATOR_FAULT` | ESKF 更新产生无效内部状态，并尝试从标定零偏重新初始化 | 保持运动禁止，并要求连续稳定恢复样本 |
| `RECOVERING` | 传感器读取和估计器更新已经恢复，但尚未满足稳定样本门槛 | 恢复完成前保持 `DATA_VALID` 清零 |

任务心跳状态通过 `AppRuntimeSnapshot.critical_tasks_alive` 独立发布。当前只有重复的关键任务心跳丢失会永久锁存；IMU 降级仅设置 `motion_inhibited`，同时保持重试和恢复。

### M2 电机控制决策状态

- 当前硬件没有 MCU 可见的电机电流反馈。AT8236 的 `ISEN` 分流电阻和固定 `VREF` 只提供硬件电流斩波，未连接到 STM32 ADC，因此 M2 禁止建立伪电流环。
- 当前 SYSCLK 为 168 MHz、PWM 约为 20 kHz、`controlTask` 为 100 Hz。仅保留 168 MHz 作为 MCU 计划运行点，PWM 和控制频率仍是实验候选值。
- PI + 前馈是基准控制器，不是最终选定控制器。LADRC 和 PI + DOB 是 `m2_control_architecture_gate.md` 定义的同级候选方案。
- 首轮速度环候选为 1 kHz，使用 TIM6/TIM7 触发任务通知；最终采样率和闭环带宽需要系统辨识、延迟/噪声测量及离散鲁棒性证据。
- 正式确定控制架构前必须完成电流反馈硬件决策。如果增加电流采样，需要重新评审电流环时序和负载转矩观测方案。

### M2 G0 电机身份（2026-07-15）

- 已购电机为塔克 MC520P30_12V，12 V、1:30、1024 PPR 磁编码器 AB 相版本。
- 原厂参数为：空载转速 360 ± 20 rpm、额定转速 290 ± 20 rpm、额定电流 0.3 A、堵转电流 3.2 A、额定转矩 1.5 kg·cm、堵转转矩 4.5 kg·cm、额定功率 4.32 W、质量约 150 g；同时给出电感 4.45 mH、电阻 2.3 ± 0.5 Ω。
- 电机标称堵转电流 3.2 A，高于控制板约 2.2 A 的 AT8236 硬件斩波阈值。辨识时必须显式检测斩波介入区域，不能假设能够达到目录堵转点。
- 原厂 1:30 四倍频示例为 `1024 * 30 * 4 = 122880` 轮端计数，当前固件使用 `1024 * 30 = 30720`。在输出轴整圈计数实测确认交付编码器的 1024 PPR 是四倍频前还是四倍频后之前，不修改固件常量。

### M2 G0 电池与机械参数（2026-07-15）

- 电池为 LB350E，采用 3 串 21700 电芯，容量 5000 mAh，标称电压 11.1 V，满电电压 12.6 V，推荐充电电流 2 A，最大持续放电电流 8 A，最大瞬时放电电流 16 A，质量约 230 g，尺寸约 75 × 64 × 22 mm；放电使用 T 型防反接插头，充电使用 DC5.5 × 2.1 接口。
- 规格图虽然标注“3C 电芯”，但整包参数表明确给出的最大持续放电电流是 8 A；安全设计按整包 8 A 限值执行，不按 `5 Ah * 3C = 15 A` 推算持续能力。
- 电池规格图声明具备过充、过流、过放、短路、过压保护和电芯安全阀，但没有给出 BMS 具体动作阈值、恢复条件和 16 A 允许持续时间；这些参数不能推测。
- 整车空载质量为 2.2 kg，最大额外载荷约 10 kg，最大总质量按约 12.2 kg 规划；轮径为 77 mm，底盘和主要机械结构为金属材质。
- 电机安装和出线方式与参考底盘及源码一致，M2 先沿用参考符号映射，不安排独立方向试验。MA 首次低功率开环斜坡必须顺带检查指令与速度符号；若不一致，在 BSP/配置层修正，其余三轮留到 M4 联调。
- 四台电机目录堵转电流合计 12.8 A，高于电池 8 A 持续能力；按每路约 2.2 A 的 AT8236 硬件斩波估算，四路理想总和仍约 8.8 A。G1 必须配置启动斜坡、PWM 上限和欠压/保护遥测，并通过单路到四路的分级试验确定实际边界。
- “金属材质、耐温高”只作为结构描述，不构成电机、驱动器或电池允许温度证据；量化热保护阈值仍需厂家资料和温升试验。

### GPIO 分配
- 按键：PE1（`USER_Btn`）
- LED1：PB0，LED2：PB1
- 蜂鸣器：PE0
- UART4：PC10 TX / PC11 RX（230400 baud）

### PWM 通道
- MA: TIM1_CH1/CH2 (PE9/PE11)
- MB: TIM1_CH3/CH4 (PE13/PE14)
- MC: TIM9_CH1/CH2 (PE5/PE6)
- MD: TIM12_CH1/CH2 (PB14/PB15)

### 编码器定时器
- MA: TIM2, MB: TIM3, MC: TIM5, MD: TIM4
