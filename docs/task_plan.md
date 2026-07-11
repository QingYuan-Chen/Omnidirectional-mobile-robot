# Task Plan: IMU Data Refactor + Single Motor PID + HMI

## Phases

### Phase 1: IMU Data Validity Model & Fault Backoff
- [ ] 1.1 Refactor `AppImuOutput` flags: split raw vs processed validity
- [ ] 1.2 Add low-pass filter for acceleration and angular rate
- [ ] 1.3 Add anomaly jump detection (spike rejection)
- [ ] 1.4 Add `sample_age_ms` field (time since last good sample)
- [ ] 1.5 Improve I2C fault backoff: exponential backoff with max cap
- [ ] 1.6 Carry timestamp, sequence, valid_flags, error_count explicitly
- [ ] 1.7 Ensure raw data is debug-only; control uses processed data

### Phase 2: Single Motor Closed-Loop PID
- [ ] 2.1 Design generic PID controller struct (reusable for 4 motors)
- [ ] 2.2 Implement RPM/RPS conversion from encoder delta
- [ ] 2.3 Implement PID with anti-windup, target ramp, deadband
- [ ] 2.4 Wire MA motor: encoder read → PID → PWM output
- [ ] 2.5 Zero target → Brake; fault → Coast safety path
- [ ] 2.6 Add command timeout (500ms) for motor commands

### Phase 3: Human-Machine Interaction
- [ ] 3.1 Button: short press start/stop, long press emergency stop
- [ ] 3.2 LED: multi-state (startup/ready/running/fault)
- [ ] 3.3 Buzzer: non-blocking state machine with tone patterns
- [ ] 3.4 UART4: command parser for target RPM, start/stop, PID params
- [ ] 3.5 Monitor: extend to report stack, heap, battery, IMU, PID status

## Decisions
- Control period: 10ms (100 Hz)
- PID output range: -4200 to +4200
- Encoder resolution: 30720 counts/wheel revolution
- MA motor as default test motor
- No mecanum kinematics in this phase
