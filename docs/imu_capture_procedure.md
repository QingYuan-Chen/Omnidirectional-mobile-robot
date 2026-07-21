# 阶段 4：IMU 新接受样本高速采集流程

## 路线位置与边界

- 原十阶段路线位置：阶段 4 板内高速采集。
- 当前结果：电机模式的既有静态与运动板测已经通过；IMU 模式软件、主机工具和离线严格门已完成。2026-07-21 首轮总体系统上电正式采集已完成，但严格门因源丢样累计增加而拒绝，整阶段仍未完成。
- 当前执行统一使用“总体系统上电/总体系统非上电”口径；阶段 4 只做静置 IMU 诊断，全程禁止 `ARM/PWM` 和运动。
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

## 首轮总体系统上电板测结果

- 独立纯 `STATUS` 证据为 `captures/20260721-150911328_COM10/`：最终电池 11.497 V，PWM 0、ready=1、inhibit/fault/motor_state/ESTOP=0，UART、控制和 ADC 错误为 0。`nontelemetry=1`、`trailing=128` 只记录为本次串口开闭边界，不能作为正式高速导出证据。
- 正式计划 `captures/stage4_imu_capture_schedule_20260721.csv` 只含 `STATUS`、`CAPTURE IMU START/STOP/STATUS/EXPORT` 和结束 `STATUS`，无 `ARM/PWM`。正式证据为 `captures/20260721-151011421_COM10/`：30 s、155,735 B、6 条命令全部发送，502 条 `IC`、5 条 `ICAP`，全部解析错误为 0；STARTED/STOPPED/STATUS/BEGIN/END 完整，BEGIN/END 均为 COMPLETE、502 样本、容量 1700，记录器 dropped/duplicate/source_gap 均为 0，导出约 1.92 s 闭合。
- 严格分析 `accepted=false`，唯一失败项为 `source_drop_counter_unchanged=false`；其余 12 项门均通过。源丢样累计从 22,230 增至 22,948，窗口增加 718；接受 sequence 15,499 至 16,000 连续，传感器时间戳 38,393 至 39,612、步长 2 至 3，总计 1,219 步（5,437.11 ms）；主机时刻 173,617 至 179,128、每步固定 11 ms，总计 5,511 ms，误差 73.89 ms，小于 108.74 ms 容差。502 个 STATUS0 均为 3。
- 整个 30 s `IMUQ dropped_sample_count` 增加 3,969；read/consecutive/backoff/duplicate/spike/estimator fault 均为 0。静置原始均值/标准差为 accel X 260.09/4.70、Y 1422.81/4.21、Z 8621.05/6.19，gyro X -40.31/4.07、Y -147.20/4.46、Z 28.59/3.68；temperature 全为 0，作为待查疑点保留。最终安全门、UART、格式、入队、导出、控制和 ADC 错误均为 0。
- 只读定位发现 `imuTask` 等待 DRDY、10 ms 超时兜底，而全部接受样本主机间隔固定为 11 ms。结合传感器时间戳每次推进 2 至 3，证据强烈指向任务持续从超时路径唤醒，而不是由 DRDY 及时唤醒；PD7 EXTI 上升沿、回调和原理图 INT1 连线的软件/图纸配置虽存在，但 DRDY 是否由 QMI8658A 正确输出并到达 MCU 尚未实测，不能提前定论为单一硬件故障。

上述一条保留首轮结束时的证据边界。随后目视核对 QMI8658A 数据手册原页和 H60 图纸，已确认普通 DRDY 固定从 INT2 输出，而图纸仅把 INT1 接到 PD7；旧配置 `CTRL1=0x50`、`CTRL7=0x83` 启用未连接的 INT2 输出并让 INT1 保持 High-Z。因此首轮 11 ms 周期已定位为端点不匹配导致的 10 ms 超时兜底，不是芯片或板线损坏。修复软件改为 `CTRL1=0x40`、3 ms 绝对周期轮询 STATUSINT/SyncSample 锁存帧，删除 thread flag/EXTI callback，并把 PD7 改为下拉普通输入、关闭 EXTI9_5；当前尚未刷写复验。

## 轮询修复后的两次主机接收失败

提交 `3c6a714` 已刷写。`captures/20260721-153329990_COM10/` 的纯 `STATUS` 与
`captures/20260721-153409644_COM10/` 的板端状态均证明源丢样保持为 0，STOP/STATUS/BEGIN
一致报告 1,224 个完整记录；但主机因默认 4,096 B 串口缓冲只保存 691 条有效 IC，缺 END
并出现四段整块字节缺失，严格门仍拒绝。该目录作为主机接收失败证据永久保留。

随后只执行了一次有修改依据的 64 KiB 复验 `captures/20260721-154902693_COM10/`。板端
STOP/STATUS/BEGIN/END 均为 1,221/1,700，记录器 dropped/duplicate/source_gap、源 drop、
UART/export 错误均为 0；主机仍只得到 635 条 IC、5 条 ICAP 和 3 个解析错误，严格结果
`accepted=false`。缺块条件下总 sensor steps=1,220，传感器理论时长与主机板端 tick 时长
一致；这只进一步定位主机接收损失，不能把不完整证据改判为通过。64 KiB 缓冲方案至此停止。

## raw-first/offline-parse 新架构复验

新工具在固定在线采集窗内不再解析行或写入数据 CSV，只保存 raw 字节和每块 24 B 的结束偏移、
相对毫秒、UTC ticks。到期后只读取一次 `BytesToRead` 快照并有界 drain 该批字节，随后关闭串口；
离线阶段再按块时刻重放 raw、恢复原有主机时间列并生成全部 CSV。metadata schema 4 分别记录
`actual_duration_ms`、`ended_utc` 与离线解析耗时，同时记录 processing mode、raw reader/timing
sidecar 哈希、24 B 宽度、记录数和 drain 字节数。

只有该工具修改完成独立提交并推送后，才执行一次新架构复验；不重复修改 IMU 固件，仍保持
总体系统上电、零运动：

1. 先做独立纯 `STATUS`，核对 PWM 0、`motor_state=0`、ready、无故障/ESTOP 和全部错误计数。
2. 清除 COM10 旧积压并等待在途帧排净；按同一计划静置采集约 5 至 6 s并导出，全程不得发送 `ARM/PWM`。
3. metadata 必须为 schema 4、`processing.mode=raw_first_offline_parse`，并记录 24 B timing sidecar、哈希、块数、在线时长、离线耗时和有界 drain；ICAP BEGIN/END、约 1,221 条 IC、解析零错误和索引连续必须同时成立。
4. 运行严格分析器，确认记录器/源丢样均为 0、时间戳步长和约 224.2 Hz 频率合理、时长一致且导出闭合。时间戳回绕若本次短窗未发生，继续保留到后续长时板测。
5. 若 raw-first 新架构仍有整块主机丢字节，不再重复实采，转为独立串口/驱动层诊断后再评审。

## 最终验收结果

`captures/20260721-160532766_COM10/` 已按上述新架构完成唯一复验：固件 `3c6a714`、仓库
`62f1799`、30 s、无 `ARM/PWM`，metadata schema 4 的在线/离线时长分别为 30,003/3,027 ms，
raw 211,123 B，timing 1,152 条/27,648 B，drain=0。IC=1,223、ICAP=5、IMU parse=0；
STARTED=0，STOPPED/STATUS/BEGIN/END 均为 1,223/1,700且三种记录器计数为 0。

严格分析 `accepted=true`、13/13。index/sequence/STATUS0/source drop 全部通过；sensor timestamp
每步为 1，总 1,222 steps=5,450.491 ms，host tick=5,520 ms、单步 3-6 ms，误差
69.509 ms<109.010 ms。IMUQ 与板端相关错误最大值均为 0；结束 11.470 V、PWM/状态为 0、
ready 且无 inhibit/fault/ESTOP。temperature raw 8,148-8,164，约 31.828-31.891 °C，
此前温度恒 0 疑点关闭。

raw 在约 20 ms 处仍有 1 个 STAT 解析错误和 2 个非协议残段，来源是无线桥旧缓存边界，早于
CAPTURE START 实际 501 ms。该异常必须保留，因此不能写“全流解析 0”；IMU 严格链自身仍为
零解析错误且13/13通过。`153409644`、`154902693` 两份失败目录也继续保留。

据此阶段 4 按“板内 IMU 高速采集/完整导出严格门”定义完成，当前路线移动到阶段 5但不在本次
开始阶段 5。轴向/量程因缺少已知姿态基准继续留在阶段 5；静置模长约 8,738.873 counts
（约 1.067 g）只作观测。24 位时间戳短窗未发生回绕，继续留给后续长时测试。阶段 5 的
故障注入/栈堆水位和阶段 6 的 500 ms 超时、Brake/Coast/旧队列失效仍是独立债务。

## 离线验证记录

- 统一主机测试：27/27。
- 串口与 raw 离线重组：PowerShell 7 与 Windows PowerShell 5.1 各 1,286 项断言。
- IMU 分析专项：两个 PowerShell 版本各 8 项断言。
- 全部 PowerShell 脚本：两个版本解析错误均为 0。
- DRDY 端点修复后 ARM Debug/Release 构建各 72/72 通过：RAM 47,864 B，CCMRAM 61,628 B；Flash
  101,608/56,292 B。
- `app_comm_protocol.c`、`app_imu_capture.c`、`app_tasks.c`、`bsp_imu.c` 通过严格告警与
  GCC `-fanalyzer`。
