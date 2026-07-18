# Windows 串口采集工具

## 用途与边界

`tools/capture_serial.ps1` 用于 M2 板测时直连调试串口，显式选择 COM 口并同时保存：

- `raw_uart.log`：串口收到的原始字节，不增加主机前缀；
- `telemetry.csv`：按当前 `T/R/D/E/P/B/S/I/J/L/M/C` 协议解析的结构化遥测；
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

默认安全模式只允许 `STATUS`。这使无动力通信验收不会因为计划文件内容错误而发送运动命令。

## 定时命令计划

命令计划是包含以下两列的 CSV：

```csv
elapsed_ms,command
0,STATUS
1000,STATUS
```

通过 `-CommandSchedulePath` 加载。`elapsed_ms` 必须是小于采集总时长的非负整数；工具会按时间排序并记录计划时间与实际发送时间。

非 `STATUS` 命令会被默认安全门拒绝。只有在当前板测步骤已经明确授权、供电和机械安全条件均已复核时，才可额外使用 `-AllowNonStatusCommands`。该开关只解除采集工具的软件限制，不代替项目安全门、固件运动许可或人工现场检查。

## CSV 字段与失败处理

`telemetry.csv` 的前两列是 `capture_elapsed_ms` 和 `host_received_utc`，其后按固件线协议展开 40 个字段。整数范围、固定标签、字段数量和系统布尔值均会校验；格式不合法的 `T,...` 行不进入 CSV，并累计到 `telemetry_parse_errors`。

如果串口不存在、被占用或运行中发生异常，脚本返回失败，同时尽可能关闭串口和文件，并在已创建的采集目录中保留 `outcome=failed` 的 `metadata.json`。因此失败尝试也能追溯其端口、时间和错误原因。

## 离线验证

串口解析、数值边界、CSV 转义、命令安全门和计划排序测试已接入统一入口：

```powershell
.\tools\run_host_tests.ps1 -Clean
```

测试不需要连接开发板；真正的端口打开、原始字节保存和实时解析仍必须通过一次实机采集验收。
