# Progress Log

## 2026-07-11: Planning session started
- User specified 3-phase implementation plan
- Created task_plan.md, findings.md
- Baseline code reviewed

## 2026-07-11: M0 repository baseline
- Initialized the repository on branch `main` and configured `origin` for `QingYuan-Chen/Omnidirectional-mobile-robot`.
- Added `.gitignore`; excluded build output, IDE state, CubeMX temporary files, local reference material, and the vendor source copy.
- Created and pushed baseline commit `51c2ed5` (`chore: establish STM32 firmware baseline`).
- Reclassified the task plan into baseline, board-validation, and TODO states; consolidated the approved execution directives into v2.

## 2026-07-11: M0 UART receive recovery
- Verified the STM32F4 HAL interrupt ordering before changing the callback flow: the HAL normally follows an errored receive-complete callback with `HAL_UART_ErrorCallback`, but the BSP no longer relies on that implicit ordering as its only recovery path.
- An errored receive-complete callback now marks recovery pending. The error callback records/clears errors and attempts one restart only when `RxState` is Ready; `BspUart_Service()` provides the task-context fallback.
- Added UART recovery attempt/success counters while retaining restart-failure and hardware error counters.
- CubeMX CLI regeneration completed without overwriting the BSP change. CubeMX emitted unrelated local third-party pack warnings.
- `cmake --build --preset Debug --clean-first`: passed 57/57; RAM 40,768 B, Flash 65,776 B.
- `cmake --build --preset Release --clean-first`: passed 57/57; RAM 40,760 B, Flash 39,276 B.
- `bsp_uart.c` passed `-Wall -Wextra -Wshadow -Wconversion -Werror` syntax checking and GCC `-fanalyzer`.
- Hardware fault injection has not yet been performed; UART parity/framing/noise/overrun recovery remains a board-validation item.

## 2026-07-11: M1 IMU data-quality outputs
- Added independent first-order filtered acceleration and angular-rate fields for control/telemetry. The ESKF continues to consume the validity-gated, unfiltered SI sample.
- Added producer-owned `sample_age_ms`, configurable vector-delta spike rejection, and spike diagnostic counters.
- Replaced the `UINT32_MAX` timestamp-jump sentinel with the real saturating dropped-sample total. A single timestamp jump now invalidates only the affected sample.
- Preliminary filter cutoff and spike thresholds remain board-tuning items; M1.3 hardware acceptance is deferred to the M2 single-motor bring-up stage.
- Debug and Release builds passed. RAM: 40,864/40,856 B; Flash: 66,660/39,696 B.
- `app_imu.c` passed `-Wall -Wextra -Wshadow -Wconversion -Werror` syntax checking and GCC `-fanalyzer`.

## 2026-07-11: M1 IMU non-blocking retry backoff
- Replaced the IMU task's fixed blocking delay with an application-owned `20/50/100/200/500 ms` retry schedule, capped at 500 ms and reset by the first successful sensor read.
- During backoff, `AppImu_Process()` skips I2C access but the IMU task still wakes at its normal bounded interval, publishes the current age/stale state, and sets its heartbeat every pass.
- Added observable backoff count, current delay, and next-retry tick fields to `AppImuOutput`.
- Debug and Release builds passed. RAM: 40,888 B; Flash: 66,900/39,764 B.
- `app_imu.c` and `app_tasks.c` passed `-Wall -Wextra -Wshadow -Wconversion -Werror` syntax checking and GCC `-fanalyzer`.

## 2026-07-11: M1 IMU fault classification and stable recovery
- Added explicit healthy, transient-degraded, persistent-sensor-fault, recovering, and estimator-fault states. Critical-task heartbeat status is published separately from IMU health.
- A single I2C error, timestamp discontinuity, or rejected spike invalidates only the affected output. Three consecutive sensor/spike failures escalate to persistent sensor fault.
- ESKF failure is classified separately and triggers reinitialization from the stationary calibration bias. `DATA_VALID` is restored only after eight timestamp-continuous samples with an initialized finite estimator.
- Safety now uses recoverable IMU health to inhibit motion without permanently latching the robot. Repeated critical-task heartbeat loss remains latching.
- M1.3 board acceptance is deferred to the M2 single-motor bring-up. At this point M2 was recorded as speed PI + feedforward with no pseudo current loop; the controller/frequency selection was explicitly reopened and superseded by the 2026-07-14 M2 decision gate below.
- Debug and Release clean builds passed 57/57, followed by successful incremental rebuilds after the final state-severity correction. RAM: 40,928 B; Flash: 67,552/40,116 B.
- `app_imu.c`, `app_tasks.c`, and `imu_eskf.c` passed `-Wall -Wextra -Wshadow -Wconversion -Werror` syntax checking and GCC `-fanalyzer`.

## 2026-07-14: M2 control architecture decision reopened
- Reclassified the reference firmware's 100 Hz control task and the current 20 kHz PWM as implementation baselines rather than final design decisions. SYSCLK remains 168 MHz, while PWM, optional current-loop, speed-loop and chassis-loop timing are managed as separate domains.
- Added `m2_control_architecture_gate.md` with G0-G7 gates covering motor/hardware confirmation, deterministic TIM6/TIM7-triggered timing, open-loop identification, M/T speed estimation, current-feedback hardware review, unified candidate controllers, board comparison and a final architecture decision record.
- PI + feedforward is now the benchmark rather than the selected controller. LADRC and PI + DOB are peer candidates until they are compared with the same data, constraints, safety behavior and quantitative metrics.
- The current H60 board has AT8236 hardware current chopping but no MCU-visible motor-current feedback. A pseudo current loop remains prohibited; adding real current feedback requires an explicit hardware decision.
- No `.ioc` or firmware behavior was changed in this documentation-only step. The existing 10 ms `controlTask` remains an explicitly labeled placeholder until G0 and the G1 test-bench design review are complete.

## 2026-07-15: M2 G0 motor identity confirmed

- Confirmed the purchased motor as the XTARK MC520P30_12V, 12 V, 1:30, 1024 PPR magnetic-encoder variant and recorded the vendor-rated speed, current, torque, power, resistance and inductance data.
- Identified that the 3.2 A catalog stall current is above the board's approximately 2.2 A AT8236 hardware current-chopping threshold; the chopping region remains a required board measurement.
- Identified a factor-of-four encoder-definition conflict: the vendor's 1:30 quadrature example gives 122880 counts per wheel revolution, while the current firmware configuration gives 30720. The firmware constant remains unchanged until a full-turn count test resolves the delivered encoder semantics.
- G0 remains open for encoder count/direction measurement, allowable temperature, battery/mechanical configuration and AT8236 decay/Brake/Coast/chopping characterization. No firmware or `.ioc` behavior changed in this step.
