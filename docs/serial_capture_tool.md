# Windows 串口采集工具

## 用途与边界

`tools/capture_serial.ps1` 用于 M2 板测时直连调试串口，显式选择 COM 口并同时保存：

- `raw_uart.log`：串口收到的原始字节，不增加主机前缀；
- `telemetry.csv`：旧 `T,...` 或新 `STAT,1,...` 投影得到的 40 列兼容遥测；
- `stat.csv`：schema 1 状态帧，与 `telemetry.csv` 使用相同的 40 个值字段；
- `imuq.csv`：IMU 序号、时间戳、年龄、健康、标志及质量/恢复累计计数；
- `resources.csv`：控制时序、五任务栈水位、最小剩余堆、UART 和各类发送失败计数；
- `events.csv`：带板端时刻、序号、变化位掩码和当前安全状态快照的事件；
- `motor_capture.csv`：停车后导出的板内 1 kHz MA 电机高速样本；
- `motor_capture_events.csv`：高速记录开始、停止、导出边界和拒绝事件；
- `imu_capture.csv`：停车状态下导出的板内 IMU 质量链接受样本；
- `imu_capture_events.csv`：IMU 记录开始、停止、导出边界和拒绝事件；
- `commands.csv`：计划时间、实际发送时间、命令文本和发送结果；
- `metadata.json`：固件提交、工具哈希、仓库状态、主机和串口配置、起止时间、计数与产物名称。

采集目录默认为仓库根目录下的 `captures/`，该目录已加入 `.gitignore`，避免原始板测数据意外进入 Git。工具不会修改固件，也不会自动猜测 COM 口或已烧录的固件版本。

## 安全状态采集

在仓库根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\capture_serial.ps1 `
  -Port COM6 `
  -DurationSeconds 30 `
  -FirmwareCommit bfb60a7 `
  -SendStatusAtStart
```

`-Port` 与 `-FirmwareCommit` 均为必填项。默认配置是 230400 baud、8N1、无流控、DTR/RTS 关闭；`-SendStatusAtStart` 会在采集开始时发送一条 `STATUS` 并把实际发送时刻写入 `commands.csv`。

默认安全模式只允许 `STATUS`。这使总体系统上电的通信验收不会因为计划文件内容错误而发送运动命令。

新固件收到 `STATUS` 后调度一帧即时 `STAT`；若同时存在到期 `EVENT`，事件先发送且强制
状态请求保持到后续通信循环。`STAT` 仍包含纯状态安全门需要的电池毫伏、目标/实际 PWM、
运行就绪、运动禁止、锁存故障、ESTOP、电机状态和累计错误总览。主机没有收到有效
`STAT` 时只能判为状态门未通过，不能凭旧帧或其他类型帧解除运动限制。

## 分类型低频遥测

schema 1 使用四种独立行协议：

- `STAT,1,...`：20 Hz 状态、编码器、电机、安全和错误总览；
- `IMUQ,1,...`：10 Hz IMU 质量、退避、重复/丢样、突变和估计器计数；
- `RES,1,...`：2 Hz 控制时序、通信资源、五任务栈水位和最小剩余堆；
- `EVENT,1,...`：启动及安全/健康/错误变化事件，最多 10 Hz。

固件每个通信循环最多选择一个诊断帧，顺序为
`EVENT > STATUS 强制 STAT > 周期 STAT > IMUQ > RES`。四种帧按最坏长度和最高频率
合并后的预算不得超过 230400/8N1 有效字节带宽的 75%。每种帧的格式化或入队失败在
`RES` 中有独立累计计数；`EVENT` 失败后保留待发变化位，但限频机制仍生效。

这套 ASCII schema 仍是板级诊断协议，不是 M5 的正式 ROS2/X-Protocol。事件允许在
100 ms 窗内合并，不能替代原始日志或被控制链当作安全输入。

## 定时命令计划

命令计划是包含以下两列的 CSV：

```csv
elapsed_ms,command
0,STATUS
1000,STATUS
```

通过 `-CommandSchedulePath` 加载。`elapsed_ms` 必须是小于采集总时长的非负整数；工具会按时间排序并记录计划时间与实际发送时间。

非 `STATUS` 命令会被默认安全门拒绝。只有在当前板测步骤已经明确授权、供电和机械安全条件均已复核时，才可额外使用 `-AllowNonStatusCommands`。该开关只解除采集工具的软件限制，不代替项目安全门、固件运动许可或人工现场检查。

## 板内高速电机采集

高速记录器使用独立 CCMRAM 固定缓冲，在候选 1 kHz TIM7 周期记录 2200 个、每个
28 B 的 MA 样本，覆盖约 2.2 s。支持以下诊断命令：

- `CAPTURE START`：仅在 MA 为 `DISARMED`、目标/实际 PWM 均为 0 且无锁存故障时清空并开始；
- `CAPTURE STOP`：停止记录并保留当前样本；
- `CAPTURE STATUS`：返回状态、样本数、容量和丢弃数；
- `CAPTURE EXPORT`：仅在记录完成且 MA 完全停机后开始异步导出。

缓冲区满后的下一样本会使记录器停止并增加 `dropped_sample_count`，旧样本不会被覆盖。
从 EXPORT 开始到 END 发出期间，普通 `ARM/PWM/STOP` 不执行且不提交序号，发送端可在
导出结束后用原序号重试；`ESTOP` 始终保持立即可用。
导出行使用 `MCAP,1,...` 边界帧和 `MC,1,...` 样本帧；串口工具严格解析并分别落盘。
样本中的 `previous_wcet_cycles` 属于当前行前一个控制周期，离线分析器会保留这一对齐
限制。

无动力时序计划可通过下列命令生成，生成器不打开串口，且清单默认未授权：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\new_motor_capture_timing_plan.ps1 `
  -FirmwareCommit <本次固件提交>
```

现场明确授权后，使用生成目录中的 `command_schedule.pending.csv` 执行约 20 s 采集。
该计划只包含 `STATUS` 和 `CAPTURE` 诊断命令，不包含 `ARM/PWM/STOP/ESTOP`，并明确要求
电机动力保持关闭。采集完成后执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\analyze_motor_capture.ps1 `
  -CaptureDirectory <采集目录>
```

分析器要求 `BEGIN/END`、CSV 行数和元数据闭合，检查 1 kHz 索引/tick 连续性、IRQ 抖动、
唤醒延迟、上一周期 WCET、缓冲丢弃，以及遥测中的漏周期、截止期、UART、命令队列、
运动门和 ADC 计数增量。门槛为 P99 不超过周期 5%、最大值不超过 10%、WCET 不超过
25%，且全部丢失、截止期、缓冲丢弃和错误增量为 0；即使高速样本完整，只要导出期间
`uart_tx_fault_count` 等任一错误增加，`accepted` 也必须为 `false`。

## 板内 IMU 新接受样本采集

IMU 记录器使用同一 CCMRAM union 的独立视图，固定保存 1700 个、每个 36 B 的样本。
它与 `MOTOR`、`G3_SPEED` 两种模式互斥，支持：

- `CAPTURE IMU START`：仅在 MA 完全停机、无锁存故障且没有其他记录器正在记录或导出时开始；
- `CAPTURE IMU STOP`：停止并保留已接受样本；
- `CAPTURE IMU STATUS`：输出状态、样本数、固定容量及三种丢失/断点计数；
- `CAPTURE IMU EXPORT`：仅在记录完成且 MA 完全停机后异步导出。

样本行 `IC,1,...` 保存应用接受序号、24 位传感器时间戳、主机毫秒时刻、质量标志、
源丢样累计值、原始六轴、温度、STATUS0 和健康级；事件行使用 `ICAP,1,...`。这里的
“高速”是按 IMU 新接受样本触发，不按 IMU 任务唤醒次数填行：`BSP_BUSY`、退避、总线
错误、重复时间戳、突变拒绝和估计器失败不会产生 `IC` 行，其诊断必须结合 `IMUQ`。

`tools/analyze_imu_capture.ps1` 要求唯一且一致的 `BEGIN/END`、固定容量 1700、至少
100 个样本、连续索引和接受序号、1 至 11 的传感器时间戳步长、前进的主机时刻、
STATUS0 加速度计/陀螺仪就绪位，以及记录器和源丢样计数均为 0。它还按配置的
224.2 Hz 比较传感器步数理论时长与主机累计时长，容差为 `max(10 ms, 2%)`。这能拒绝
明显不一致的离线证据，但最终 ODR、轴向和真实导出链仍必须以无动力板测关闭；完整流程
见 `imu_capture_procedure.md`。

## CSV 字段与失败处理

`telemetry.csv` 的前两列是 `capture_elapsed_ms` 和 `host_received_utc`，其后固定为既有
40 个值字段。旧 `T,...` 直接进入该文件；新 `STAT,1,...` 同时进入 `stat.csv` 和
`telemetry.csv`，从而保持既有 G2 分析器兼容。`STAT` 额外严格限制 schema、电机状态、
IMU 健康和 int16 PWM 范围。

`imuq.csv`、`resources.csv` 和 `events.csv` 同样带主机接收时间。四种新帧分别维护
`*_rows` 与 `*_parse_errors`；metadata schema 3 还记录各产物名称。字段数、固定标签、
整数边界、枚举、布尔值、事件位掩码或 UART 队列深度不合法时，该行不进入 CSV。任何
完整性分析都必须同时检查四类解析错误，不能只读取旧 `telemetry_parse_errors`。

`motor_capture.csv`、`motor_capture_events.csv`、`imu_capture.csv` 与
`imu_capture_events.csv` 同样带有主机接收时间。未知版本、字段数量、整数边界、电机
状态、安全位、IMU 健康级或事件类型分别累计到对应的 `*_parse_errors`，不得作为完整
高速证据。既有 G2 捕获、工作点和死区入口已把 IMU 行/事件纳入总行数闭合并拒绝 IMU
解析错误；新增 schema 不改变原有 G2 数值字段或结论。

如果串口不存在、被占用或运行中发生异常，脚本返回失败，同时尽可能关闭串口和文件，并在已创建的采集目录中保留 `outcome=failed` 的 `metadata.json`。因此失败尝试也能追溯其端口、时间和错误原因。

## 离线验证

串口解析、四类 schema/数值边界、兼容视图、CSV 转义、命令安全门和计划排序测试已接入统一入口：

```powershell
.\tools\run_host_tests.ps1 -Clean
```

当前统一主机测试为 27/27；串口解析在 PowerShell 7 和 Windows PowerShell 5.1 下均为
55 项断言通过，IMU 高速分析专项在两个版本下均为 8 项断言通过。测试不需要连接开发板；
真正的调试 UART 周期、`STATUS` 响应、四类低频行数、IMU 实际 ODR/时间戳/STATUS0、队列
失败计数和原始字节保存仍必须通过总体系统上电实机采集验收。2026-07-21 起阶段 3 调试 UART
为无线 DAPLink 对应的 USART1 PA9/PA10、Windows 端口 COM10；USART2 恢复为树莓派正式通信口。
首个 COM10 纯 `STATUS` 因基线固件仍选择 USART2 而得到 0 字节，失败证据位于
`captures/20260721-143006157_COM10/`，不得删除或用迁移后的成功采集覆盖。

阶段 3 的最终权威板测证据为 `captures/20260721-145813693_COM10/`：提交 `7aaaeb4`、
COM10 230400/8N1、10 s，只发送一条 `STATUS`，得到 174/87/18/4 条
`STAT/IMUQ/RES/EVENT`，四类解析错误、非遥测行和尾残片均为 0；前三类频率
19.871/9.935/1.998 Hz，EVENT 间隔至少 100 ms。此前 `145501089`、`145602046`、
`145717478` 的积压、首帧边界和复位前在途帧诊断目录与基线 0 字节目录一并保留。
末行 `IMUQ dropped_sample_count=1128` 不得误写为全诊断计数为 0；它属于阶段 4/5 的
真实 ODR/丢样债务，阶段 3 只据通信、解析、格式、入队和时序门关闭。

阶段 4 在 `captures/20260721-153409644_COM10/` 导出 1,224 条 IMU 高速记录时暴露了
Windows `SerialPort` 默认 4,096 B 接收缓冲边界：板端记录器和 UART 诊断为 0，但主机
出现四段整块字节缺失、IC/STAT 拼接并缺少 END。工具因此固定使用 65,536 B 接收缓冲，
并在 `metadata.serial.read_buffer_bytes` 记录实际配置；原始读取数组仍为 4,096 B，避免放大
单次PowerShell逐行解析成本。该失败目录保留，不能用后续成功结果覆盖。

唯一 64 KiB 复验 `captures/20260721-154902693_COM10/` 仍只保存 635/1,221 条 IC，并有
3 个解析错误；板端 STOP/STATUS/BEGIN/END、记录器三计数、源 drop、UART/export 均闭合为
零错误。该结果证明扩大接收缓冲不能消除在线解析和多 CSV 写入阻塞，失败目录继续保留。

当前工具采用 raw-first/offline-parse：在线循环只调度命令、读取串口、原样写入 `raw_uart.log`
并向 `raw_chunk_timing.bin` 追加 24 B 定长记录。每条记录依次保存该块累计结束偏移、采集相对
毫秒和 UTC ticks；固定时长到期后只按一次 `BytesToRead` 快照有界 drain，不等待持续遥测归零，
随即关闭串口。离线 reader 以包含换行符的块时刻恢复 `capture_elapsed_ms` 和
`host_received_utc`，再运行原有各 schema 解析与 CSV 分派，因此采集时间不会被 PowerShell
解析吞吐反压，既有主机时间语义也不改写。

metadata 升级为 schema 4：保留原字段，并新增 `raw_reader_sha256`、
`processing.mode=raw_first_offline_parse`、独立 `offline_parse_duration_ms`、有界 drain 字节数、
24 B timing 记录宽度/哈希/数量和 sidecar 产物名。`actual_duration_ms`、`ended_utc` 仍只描述
在线采集窗，不包含离线解析。超过 64 KiB、1,221 条 IC、跨块/CRLF/尾残片、时刻单调和 raw
哈希不变的测试在 PowerShell 7/5.1 下各 1,286 项断言通过，统一主机测试 27/27、全部脚本
两版本解析错误为 0。该软件提交推送后才允许一次新的阶段 4 零运动复验；若仍丢块，不得继续
追加相同实采。

raw-first 架构的唯一正式复验为 `captures/20260721-160532766_COM10/`：固件 `3c6a714`、
仓库 `62f1799`、COM10 230400/8N1、30 s，无 `ARM/PWM`。metadata schema 4 记录在线
30,003 ms、离线解析 3,027 ms、raw 211,123 B、timing 1,152 条/27,648 B、drain=0。
主机完整得到 1,223 条 IC 和 5 条 ICAP，IMU parse=0；BEGIN/END 与板端计数闭合，严格分析
13/13、`accepted=true`。该证据关闭阶段 4 的板内 IMU 高速采集/完整导出门。

原始流开头约 20 ms 有无线桥旧缓存造成的 1 个 STAT 解析错误和 2 个非协议残段，发生在
CAPTURE START 实际 501 ms 前。该事实必须保留，不能把权威目录描述为“全流解析 0”；它与
IMU 严格链自身零解析错误是两个不同范围。此前 4 KiB 与 64 KiB 两份块丢失证据也继续保留，
不能由最终成功目录覆盖。
