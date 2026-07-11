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
