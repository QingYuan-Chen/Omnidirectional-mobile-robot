# M2 G2/G3 单电机试验流程

## 当前阶段

本阶段先完成不需要电机动力的试验计划和分析工具。只有在电机动力供电、接线和机械安全条件由操作者确认后，才允许把待执行计划交给串口采集工具。

当前仍不能宣称 G2/G3 完成：

- 尚未让 MA 电机实际运动；
- 1024 PPR 对应的轮端整圈计数仍有 30,720/122,880 歧义；
- 当前遥测只提供原始计数、单周期增量和累计计数，不提供未经验证的 RPM；
- 当前没有脉冲周期数据，不能完成 T 法或 M/T 融合评审；
- 50 Hz 遥测快照可检查累计最大抖动和 WCET，但不能代替完整 1 kHz 分布的最终 P99 验收。

## 首动计划生成

`new_g2_first_motion_plan.ps1` 只生成文件，不打开串口：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\new_g2_first_motion_plan.ps1 `
  -ExperimentId ma_positive_first_motion `
  -FirmwareCommit bfb60a7 `
  -Direction Positive `
  -PeakPwm 100
```

输出目录默认为 `experiments/generated/<ExperimentId>/`，包含：

- `command_schedule.pending.csv`：待执行命令计划；
- `experiment_manifest.json`：参数、安全检查和授权状态。

首次运动计划的软件临时上限为 200，低于固件 G1.2 的 840 硬上限。正转和反转必须分别生成、分别观察，首个计划不会在一次运行内直接换向。工具按不超过 400 ms 的间隔重复 PWM 命令，低于固件 500 ms 命令超时；结束使用受控 `STOP`，不生成需要复位恢复的 `ESTOP`。

生成后的清单固定为 `execution_state=not_authorized`，供电、极性/共地、MA 单通道、车轮悬空、紧急断电和人工授权初值均为 `false`。文件生成成功不代表允许执行。

## 电机动力执行门

真正执行前必须暂停并由操作者确认：

1. 12 V 电机动力来源、极性和可用电流符合当前单电机试验；
2. 控制板逻辑地与电机动力地连接正确；
3. 只允许 MA 运动，MB/MC/MD 保持 Coast；
4. 车轮悬空、周围无干涉，机器人被可靠固定；
5. 可立即切断电机动力，且操作者已了解 `ESTOP` 后必须复位；
6. 串口、ST-Link 和遥测仍正常；
7. 明确本次方向、PWM、持续时间和全部待发送命令。

上述条件未关闭前，不得把 `.pending.csv` 传给带 `-AllowNonStatusCommands` 的采集命令。

## 采集后的离线分析

对提交二生成的完整采集目录运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\analyze_g2_capture.ps1 `
  -CaptureDirectory .\captures\<capture-directory>
```

分析器先核对原始字节、遥测行数、命令行数、解析错误、非遥测行和尾部残片。证据不闭合时拒绝继续。通过后生成：

- `g2_analysis_summary.json`：采样周期、时序门槛、PWM、电池、四路编码器计数率和错误最大值；
- `g2_telemetry_derived.csv`：相邻帧板端时间差、控制 tick 差、四路累计计数差和 counts/s。

分析结果故意保留 counts/s，不使用当前未验证的每圈计数换算 RPM。P99 抖动来自 50 Hz 快照，只作为初步检查；累计最大抖动、WCET、漏周期和截止期计数仍可直接发现明显阻断问题。

## 动力接入后的执行顺序

1. 正向低 PWM 首动，确认 MA 通道、编码器符号、STOP 和电池压降；
2. 单独执行负向低 PWM 首动，确认方向与编码器符号对称；
3. 测量输出轴整圈计数，关闭编码器倍率冲突；
4. 验收 TIM7 零丢周期、最大抖动和 WCET，再补完整 P99 分布；
5. 逐步扩展死区、线性区、阶跃、斜坡、零点穿越和 Brake/Coast 工况；
6. 每个工况至少重复三次，保留电池状态、温升和机械配置；
7. 比较一阶加纯延迟、二阶加纯延迟模型；
8. 增加脉冲周期测量后比较 M、T 与 M/T 测速，再决定 RPM 实现。
