# 规划审批意见与执行指令（给 codex）

> 审批人: Claude（2026-07-11）。本文档是对 `task_plan.md` 三阶段计划的审批结论和修订指令。
> codex 执行时以本文档为准，与 `task_plan.md` 冲突处按本文档执行，并同步更新 `task_plan.md`。

## 审批结论

三阶段划分（IMU → 单电机 PID → HMI）方向正确，予以通过，但附带以下修订：

1. **顺序错误**：PID 调参（Phase 2）依赖"下发目标转速 + 回传实际转速"的通道，而串口命令解析被排在 Phase 3.4。没有遥测就没法调 PID。→ 必须把**最小命令/遥测通道**前置到 Phase 2 开头。
2. **三个已知代码缺陷未纳入计划**（见 P0 清单），其中两个与 Phase 1 的目标直接冲突，必须先修。
3. **缺少验收纪律**：上一轮 18 个子项 0 完成但无任何进度记录。本轮起每个子项必须附构建证据并实时勾选。

## P0：开工前必须完成的前置项

- [ ] **P0.1 初始化版本控制**。当前项目不是 git 仓库，任何改动都不可回退。
  在项目根目录执行 `git init`，添加 `.gitignore`（排除 `build/`、`.idea/workspace.xml`），首个 commit 固化当前基线。此后每完成一个子项 commit 一次。
- [ ] **P0.2 修复 UART 接收停摆缺陷**：`App/Bsp/Src/bsp_uart.c` 的 `HAL_UART_RxCpltCallback` 在 `ErrorCode != NONE` 时直接 return，既不重启接收也不置 `rx_recovery_pending`。若 ErrorCallback 未触发，该口 RX 永久静默。Phase 2/3 的命令通道都依赖它。
- [ ] **P0.3 重定义 IMU 故障恢复策略**（与 Phase 1.5 合并设计）：
  - 现状 A：`app_imu.c` 时间戳跳变 > 11 步即置 `SENSOR_FAULT`，且 `dropped_sample_count = UINT32_MAX` 拿计数器当哨兵。
  - 现状 B：`app_tasks.c` safety 任务对 IMU 不健康**永久锁存** fault → 电机永久 coast，只能重启。
  - 一次 I2C 毛刺或 >50ms 的调度延迟就会让机器人瘫痪，这与 Phase 1"故障退避后恢复"的目标相反。
  - 要求：区分**瞬时降级**（数据标 STALE/INVALID、控制层降速或停车，恢复后自动回归）与**持久故障**（连续 N 次退避失败才锁存）。锁存仅保留给不可恢复场景。策略写进 `findings.md` 再实现。

## Phase 1 修订（IMU 数据有效性）

按原计划 1.1–1.7 执行，补充以下量化规格（实现前如需调整，先在 findings.md 记录理由）：

- **1.2 低通滤波**：一阶 IIR，`y += alpha * (x - y)`。加速度 fc≈30 Hz，角速度 fc≈50 Hz（ODR 224.2 Hz 下折算 alpha）。滤波作用于"processed"输出，raw_sample 保持原样。
- **1.3 突变剔除**：相邻样本差超过阈值（加速度 4g/样本、角速度 500 dps/样本，可调）时丢弃该样本并计数 `spike_reject_count`，连续 ≥5 个 spike 才升级为故障。
- **1.4 `sample_age_ms`**：在 `AppImu_Process` 出口计算 `now_ms - last_good_tick_ms` 写入输出结构，消费方不得自己算。
- **1.5 指数退避**：初始 50 ms，倍增，上限 1 s；成功读取一次即复位。替换现在固定 50 ms 的做法。配合 P0.3 的恢复策略。
- **1.1/1.7 标志拆分**：新增 `APP_IMU_FLAG_RAW_VALID` 与 `APP_IMU_FLAG_PROCESSED_VALID`；控制/安全路径只允许消费 processed 侧标志，raw 仅供遥测调试。

**Phase 1 验收**：编译通过 + 板上跑 ≥5 分钟：人为拔插 IMU 中断线或短暂干扰 I2C，系统降级后能自动恢复，不永久锁存；`docs/progress.md` 记录测试现象。

## Phase 2 修订（单电机 PID）

新增两个前置子项，其余沿用 2.1–2.6：

- **2.0a 最小调试通道（从 Phase 3.4/3.5 前移）**：UART4 收纯文本命令：`t <rpm>`（设目标）、`s`（停止）、`p/i/d <val>`（PID 参数）；周期 50 ms 回传一行 `target_rpm, actual_rpm, pwm, err` 文本。不做二进制协议、不做 CRC——那是后续 ROS 协议（X-Protocol）的事，本通道仅调试用。
- **2.0b 开环方向标定**：闭环前先开环给 MA +1000/-1000 PWM 各 2 s，确认电机转向与编码器计数符号一致并记录进 findings.md。符号不一致时在 bsp 层统一翻转，禁止在 PID 里加负号。
- **2.1 PID 模块**：新建 `App/Inc/pid.h` + `App/Src/pid.c`，通用结构体（kp/ki/kd、积分限幅、输出限幅、目标斜坡步长），4 电机可复用。float 实现即可（CM4F 硬件 FPU）。
- **2.2 转速换算**：`rpm = delta * (60000 / (30720 * period_ms))`。10 ms 周期下低速编码器量化噪声大，实测速度需再过一层低通（复用 1.2 的 IIR）。
- **2.3 抗饱和**：用积分钳位（conditional integration / clamping），不用简单积分限幅。
- **2.5 安全路径**：目标为 0 → 斜坡减速到停 → Brake；`fault_latched` 或命令超时 → 立即按 safety 策略处理。PID 只在 controlTask 里跑，其他任务不许碰电机（遵守 FreeRTOS框架规划.md 的所有权规则）。
- **2.6 命令超时**：500 ms 无新命令即目标归零（受控停车，不是 coast）。用现有 `ROBOT_CONFIG_CMD_TIMEOUT_MS`，别再定义新宏。

**Phase 2 验收**：板上 MA 电机 100/200/300 RPM 阶跃，回传曲线稳态误差 <5%、无持续振荡；拔掉串口 500 ms 后电机受控停车。测试记录进 progress.md。

## Phase 3 修订（人机交互）

- **3.1 按键**：PE1，10 ms 轮询消抖（放 controlTask 或 monitorTask，不新开任务）；短按 <1 s = 启动/停止，长按 ≥3 s = 急停锁存。先查原理图确认按键极性和上下拉再写代码。
- **3.2 LED**：LED1 = 运行心跳（现有 500 ms 翻转保留），LED2 = 状态：常灭=正常、慢闪=就绪未运行、快闪=故障。用查表状态机，不要 if 链。
- **3.3 蜂鸣器**：非阻塞状态机（pattern = 若干 {on_ms, off_ms} 段），挂在 monitorTask 周期里驱动；**同时删除 `app.c:68` 的阻塞 `osDelay(500)` 响铃**，改为提交一个 pattern。
- **3.4/3.5**：把 2.0a 的文本通道扩展为完整调试命令集 + 监控上报（栈余量、堆余量、电池 mV、IMU 标志、PID 状态）。仍是文本协议；二进制 X-Protocol 留给下一期 ROS 对接。
- **顺带清理**（仅限这几处，不许扩大范围）：删除 `robot_config.h` 里重复的 `BATTERY_DIVIDER_RATIO`（与 NUM/DEN 二选一）；确认 `TELEMETRY_PERIOD_MS` 被 3.5 实际使用，否则删除。

## 过程纪律（不可协商）

1. **每完成一个子项**：勾选 `task_plan.md` 对应项 → `docs/progress.md` 追加一行（日期、子项号、构建命令及结果摘要）→ git commit。
2. **构建验证**：每个子项完成后必须跑 `cmake --build --preset Debug` 并确认成功；说"完成"必须能贴出构建输出。不许在多个子项堆积后才第一次编译。
3. **边界**：不修改 CubeMX 生成区（只允许在 `USER CODE BEGIN/END` 内挂钩）；新业务代码一律放 `App/`；不顺手重构与本期无关的代码。
4. **需要重新生成 CubeMX**（如启用 IWDG、改引脚）时，先在 progress.md 说明原因，重新生成后立即构建验证再继续。
5. **板上验证做不了时**（无硬件在手）：如实标注"未上板验证"，不得声称通过。

## 明确不做（本期范围外，防止跑偏）

- 麦克纳姆四轮运动学与四轮联动闭环。
- 二进制 X-Protocol / ROS2 对接。
- IWDG 看门狗（列入下一期，与 FreeRTOS框架规划.md 第 44-51 行的落地顺序合并）。
- UART DMA + IDLE 改造（记入 backlog，当前 230400 波特率逐字节中断可先用）。
