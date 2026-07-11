# Findings

## Codebase Baseline (2026-07-11)

### FreeRTOS Config
- `configUSE_PREEMPTION=1`, `configTICK_RATE_HZ=1000`
- `configENABLE_FPU=0` — **needs verification** with hard-float ABI
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
