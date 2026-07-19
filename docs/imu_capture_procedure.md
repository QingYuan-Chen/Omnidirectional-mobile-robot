# 阶段 4：IMU 新接受样本高速采集流程

## 路线位置与边界

- 原十阶段路线位置：阶段 4 板内高速采集。
- 当前结果：电机模式已经通过无动力和带动力板测；IMU 模式软件、主机工具和离线严格门已完成，整阶段仍待无动力板测。
- 当前执行不需要电机动力；本软件切片没有连接主控板、没有打开 COM6、没有刷板或发送命令。
- 阶段 7 的 G2/G3 上板验证继续冻结，当前连接配置的 `±840` 正反各三次完整保留。
- G4 维持“固定现有硬件、不增加真实电流反馈”的已决策状态。

## 已实现的软件契约

`AppImuCaptureSample` 固定为 36 B，容量固定为 1700，最多保存约 7.58 s 的 224.2 Hz
接受样本。它与既有 `MOTOR`、`G3_SPEED` 记录器通过同一个 CCMRAM union 互斥复用，
链接结果保持 CCMRAM 61,628 B（94.04%），没有同时分配三份大缓冲。

记录资格不是 IMU 任务醒来一次就写一行。只有 `AppImu_Process` 返回 `BSP_OK`、质量链
真正接受并推进 sequence 的输出才进入 `IC`：

- 保存 sequence、24 位传感器时间戳和主机毫秒时刻；
- 保存 flags、源丢样累计值、原始六轴、温度、STATUS0 和健康级；
- `BSP_BUSY`、退避、总线错误、重复时间戳、突变拒绝和估计器失败不写样本；
- 被拒绝原始帧只能通过 `IMUQ` 的错误、重复、丢样和突变计数诊断，不能从 `IC` 还原。

START 以最后完整发布且已经记录的 sequence 为基线。IMU 快照发布、运动门刷新和本次
36 B 记录位于同一个不可调度事务，避免通信任务恰好在发布与记录之间 START 而把正常
首样本误记为重复序号。

## 协议与主机产物

板端命令为：

- `CAPTURE IMU START`
- `CAPTURE IMU STOP`
- `CAPTURE IMU STATUS`
- `CAPTURE IMU EXPORT`

事件帧为
`ICAP,1,event,state,sample_count,capacity,dropped,duplicate,source_gap`；样本帧为
`IC,1,index,sequence,sensor_timestamp,host_tick,flags,source_dropped,ax,ay,az,gx,gy,gz,temp,status,health`。

Windows 采集器分别保存 `imu_capture.csv`、`imu_capture_events.csv`，并在
`metadata.json` 中记录行数、事件数、解析错误和产物名称。既有 G2 工具仍读取原来的
电机字段，但会把 IMU 行/事件纳入总行数闭合并拒绝 IMU 解析错误，防止新增 schema
污染完整性判断。

离线分析命令：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\analyze_imu_capture.ps1 `
  -CaptureDirectory <采集目录>
```

严格门要求：

- metadata 完成，CSV 行数和事件数闭合，解析错误为 0；
- 唯一 `BEGIN/END` 均为 COMPLETE，样本数一致，容量固定 1700；
- 记录器 dropped/duplicate/source_gap 均为 0；
- 至少 100 个样本，索引与接受 sequence 连续；
- 传感器时间戳每步为 1 至 11，主机时刻每步大于 0 且不超过 1000 ms；
- STATUS0 的加速度计和陀螺仪就绪位均有效，源丢样累计值不增加；
- 传感器步数按 224.2 Hz 得到的理论时长与主机累计时长误差不超过
  `max(10 ms, 2%)`。

## 待执行的无动力板测

本节只是后续验收流程，本软件切片没有执行：

1. 物理断开电机动力，连接 ST-Link 和 USART2 USB-COM；刷写绑定已提交仓库版本的固件。
2. 先做独立纯 `STATUS`，核对 PWM 0、`motor_state=0`、ready、无故障/ESTOP 和全部错误计数。无动力 ADC 只记基线，不得误当运动电压。
3. 静置采集约 5 至 6 s：`CAPTURE IMU START` 后等待，再 `STOP`、`STATUS`、`EXPORT`；全程不得发送 `ARM/PWM`，也不需要接电机动力。
4. 检查 `ICAP BEGIN/END`、真实接受频率、24 位时间戳回绕/步长、STATUS0、轴向和静止原始值；核对 `IMUQ` 中被拒绝帧与错误计数。
5. 运行严格分析器，确认无解析错误、无记录器/源丢样、无接受序号断点、时长一致且导出闭合。

只有真实证据通过后，阶段 4 才能从“软件完成、待无动力板测”改为整阶段完成。阶段 3
低频分型遥测、阶段 5 的 DRDY/故障注入/栈堆水位和阶段 6 的 500 ms 超时、
Brake/Coast/旧队列失效仍是独立债务，不能由本次 IMU 记录器代替。

## 离线验证记录

- 统一主机测试：27/27。
- 串口解析：PowerShell 7 与 Windows PowerShell 5.1 各 55 项断言。
- IMU 分析专项：两个 PowerShell 版本各 8 项断言。
- 38 个 PowerShell 脚本：两个版本解析错误均为 0。
- ARM Debug/Release 构建通过：RAM 47,792 B，CCMRAM 61,628 B；Flash
  102,836/57,136 B。
- `app_comm_protocol.c`、`app_imu_capture.c`、`app_tasks.c` 通过严格告警与
  GCC `-fanalyzer`。
