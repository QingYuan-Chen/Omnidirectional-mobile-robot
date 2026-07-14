# Findings

## Codebase Baseline (2026-07-11)

### FreeRTOS Config
- `configUSE_PREEMPTION=1`, `configTICK_RATE_HZ=1000`
- `configENABLE_FPU=0` is not used by the selected GCC ARM_CM4F FreeRTOS port. The toolchain uses `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`; `port.c` enables VFP and lazy floating-point context preservation. This item is verified and closed.
- `configCHECK_FOR_STACK_OVERFLOW=2`
- `configMAX_PRIORITIES=56`, heap=32KB
- CMSIS-RTOS v2 wrapper active

### Task Layout
| Task | Priority | Period | Stack |
|------|----------|--------|-------|
| controlTask | High | 10ms（当前占位） | 1536B |
| safetyTask | High | 100ms | 1024B |
| commTask | AboveNormal | 5ms | 1536B |
| imuTask | AboveNormal | DRDY/10ms | 1536B |
| monitorTask | Low | 500ms | 1024B |

### Current AppImuOutput Flags
- SENSOR_PRESENT, CALIBRATED, DATA_VALID, FILTER_INITIALIZED
- FILTER_CONVERGED, ACCEL_UPDATE_USED, VIBRATION_HIGH
- SENSOR_FAULT, DATA_STALE, ABSOLUTE_YAW_VALID, TIMESTAMP_VALID, TILT_VALID
- SAMPLE_SPIKE, SENSOR_DEGRADED, RECOVERING, ESTIMATOR_FAULT

### IMU Data and Flag Semantics

| Field / flag | Meaning | Control-path use |
|---|---|---|
| `raw_sample` | Unmodified synchronized QMI8658A register values | No; diagnostics/telemetry only |
| `acceleration_mps2` | Current body-frame SI acceleration used by the estimator | Only while processed validity requirements hold |
| `angular_rate_rad_s` | Current body-frame angular rate with estimated gyro bias removed | Only while processed validity requirements hold |
| `quaternion`, `euler_rad` | ESKF attitude output | Requires `DATA_VALID`, `FILTER_INITIALIZED`, `TIMESTAMP_VALID`, and the relevant attitude-valid flag |
| `SENSOR_PRESENT` | The sensor was detected and initialized | Diagnostic prerequisite, not sufficient validity |
| `CALIBRATED` | Startup stationary calibration completed | Required for processed output |
| `DATA_VALID` | Current processed sample and estimator state are consumable | Required by control/safety consumers |
| `FILTER_INITIALIZED` | ESKF nominal/error state was initialized | Required for attitude output |
| `FILTER_CONVERGED` | Minimum accepted gravity updates reached | Readiness/diagnostic gate |
| `ACCEL_UPDATE_USED` | This sample's acceleration passed gravity-observation gates | Diagnostic; a single rejection is not a sensor fault |
| `VIBRATION_HIGH` | Acceleration norm suggests elevated vibration/dynamic motion | Diagnostic/degradation input |
| `SENSOR_FAULT` | Persistent sensor access/data failure under the current policy | Processed data must not be consumed |
| `DATA_STALE` | No sufficiently recent good sample | Processed data must not be consumed |
| `TIMESTAMP_VALID` | Sensor time progression is acceptable | Required by processed consumers |
| `TILT_VALID` | Roll/pitch are observable and valid | Required when consuming tilt |
| `ABSOLUTE_YAW_VALID` | Absolute heading is observable | Always false for the current six-axis IMU-only estimator |

The M1 implementation added explicitly named filtered acceleration/angular-rate fields for control and telemetry. They do not replace `raw_sample` or the ESKF input path.

### M1 Fault and Recovery Semantics

| State | Meaning | Safety behavior |
|---|---|---|
| `HEALTHY` | Current sample and finite initialized estimator passed the processed-data gates | Processed IMU output may be consumed |
| `TRANSIENT_DEGRADED` | A single sensor access, timestamp, or spike failure affected the current sample | Inhibit motion without permanent latching |
| `PERSISTENT_SENSOR_FAULT` | Consecutive sensor/spike failures reached the configured threshold | Keep motion inhibited and continue retry/heartbeat |
| `ESTIMATOR_FAULT` | ESKF update produced an invalid internal state; reinitialization is attempted from calibration bias | Keep motion inhibited; require stable recovery samples |
| `RECOVERING` | Sensor reads and estimator updates have resumed, but the stable-sample threshold is not complete | Keep `DATA_VALID` clear until recovery completes |

Task heartbeat status remains separate in `AppRuntimeSnapshot.critical_tasks_alive`. Only repeated task-heartbeat loss is permanently latched at this stage; IMU degradation drives `motion_inhibited` while retry and recovery continue.

### M2 Motor-Control Decision Status

- The present hardware has no MCU-visible motor-current feedback. The AT8236 `ISEN` shunt and fixed `VREF` provide hardware current chopping but are not routed to the STM32 ADC, so M2 must not create a pseudo current loop.
- SYSCLK is currently 168 MHz, PWM is approximately 20 kHz, and `controlTask` is currently 100 Hz. Only SYSCLK is retained as the planned MCU operating point; PWM and control rates remain experimental candidates.
- PI + feedforward is the benchmark controller, not the selected final controller. LADRC and PI + DOB are peer candidates under `m2_control_architecture_gate.md`.
- The first speed-loop candidate is 1 kHz using a TIM6/TIM7-triggered task notification; the final sampling rate and closed-loop bandwidth require plant identification, delay/noise measurements and discrete robustness evidence.
- A hardware-current-feedback decision is required before final architecture selection. If current sensing is added, current-loop timing and load-torque-observer options must be reevaluated.

### M2 G0 Motor Identity (2026-07-15)

- The purchased motor is the XTARK MC520P30_12V, 12 V, 1:30, 1024 PPR magnetic AB-encoder variant.
- Vendor ratings are 360 ± 20 rpm no-load, 290 ± 20 rpm rated, 0.3 A rated current, 3.2 A stall current, 1.5 kg·cm rated torque, 4.5 kg·cm stall torque, 4.32 W rated power and approximately 150 g mass. The vendor also lists 4.45 mH inductance and 2.3 ± 0.5 Ω resistance.
- The motor's 3.2 A nominal stall current exceeds the board's approximately 2.2 A AT8236 hardware-chopping threshold, so identification must explicitly detect the chopping region instead of assuming the catalog stall point is reachable.
- The vendor's 1:30 quadrature example computes `1024 * 30 * 4 = 122880` wheel counts, while the firmware currently uses `1024 * 30 = 30720`. No firmware constant will change until an output-shaft full-turn count test resolves whether the vendor's 1024 PPR is pre- or post-quadrature for the delivered unit.

### GPIO Assignments
- Button: PE1 (USER_Btn)
- LED1: PB0, LED2: PB1
- Buzzer: PE0
- UART4: PC10 TX / PC11 RX (230400 baud)

### PWM Channels
- MA: TIM1_CH1/CH2 (PE9/PE11)
- MB: TIM1_CH3/CH4 (PE13/PE14)
- MC: TIM9_CH1/CH2 (PE5/PE6)
- MD: TIM12_CH1/CH2 (PB14/PB15)

### Encoder Timers
- MA: TIM2, MB: TIM3, MC: TIM5, MD: TIM4
