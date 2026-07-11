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
| controlTask | High | 10ms | 1536B |
| safetyTask | High | 100ms | 1024B |
| commTask | AboveNormal | 5ms | 1536B |
| imuTask | AboveNormal | DRDY/10ms | 1536B |
| monitorTask | Low | 500ms | 1024B |

### Current AppImuOutput Flags
- SENSOR_PRESENT, CALIBRATED, DATA_VALID, FILTER_INITIALIZED
- FILTER_CONVERGED, ACCEL_UPDATE_USED, VIBRATION_HIGH
- SENSOR_FAULT, DATA_STALE, ABSOLUTE_YAW_VALID, TIMESTAMP_VALID, TILT_VALID

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

The M1 implementation will add explicitly named filtered acceleration/angular-rate fields for control and telemetry. They will not replace `raw_sample` or the ESKF input path.

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
