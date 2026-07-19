function Get-AppTelemetryColumnNames {
    return @(
        'board_now_ms',
        'control_tick_sequence',
        'irq_timestamp_cycles',
        'encoder_raw_1',
        'encoder_raw_2',
        'encoder_raw_3',
        'encoder_raw_4',
        'encoder_delta_1',
        'encoder_delta_2',
        'encoder_delta_3',
        'encoder_delta_4',
        'encoder_total_1',
        'encoder_total_2',
        'encoder_total_3',
        'encoder_total_4',
        'target_pwm',
        'applied_pwm',
        'battery_mv',
        'critical_tasks_alive',
        'runtime_ready',
        'motion_inhibited',
        'fault_latched',
        'motor_state',
        'estop_latched',
        'imu_sample_age_ms',
        'imu_health',
        'irq_jitter_cycles',
        'irq_jitter_max_cycles',
        'wake_latency_cycles',
        'wcet_max_cycles',
        'missed_period_count',
        'deadline_miss_count',
        'uart_error_count',
        'uart_rx_overflow_count',
        'uart_tx_fault_count',
        'command_reject_count',
        'command_queue_drop_count',
        'motion_gate_reject_count',
        'invalidated_motor_command_count',
        'adc_error_count'
    )
}

function ConvertTo-AppTelemetryUnsigned {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text,
        [Parameter(Mandatory = $true)]
        [uint64]$Maximum,
        [Parameter(Mandatory = $true)]
        [string]$FieldName
    )

    [uint64]$value = 0
    $parsed = [uint64]::TryParse(
        $Text,
        [Globalization.NumberStyles]::None,
        [Globalization.CultureInfo]::InvariantCulture,
        [ref]$value)
    if (-not $parsed -or $value -gt $Maximum) {
        throw "遥测字段 $FieldName 不是有效无符号整数：$Text"
    }
    return $value
}

function ConvertTo-AppTelemetrySigned {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text,
        [Parameter(Mandatory = $true)]
        [int64]$Minimum,
        [Parameter(Mandatory = $true)]
        [int64]$Maximum,
        [Parameter(Mandatory = $true)]
        [string]$FieldName
    )

    [int64]$value = 0
    $parsed = [int64]::TryParse(
        $Text,
        [Globalization.NumberStyles]::AllowLeadingSign,
        [Globalization.CultureInfo]::InvariantCulture,
        [ref]$value)
    if (-not $parsed -or $value -lt $Minimum -or $value -gt $Maximum) {
        throw "遥测字段 $FieldName 不是有效有符号整数：$Text"
    }
    return $value
}

function ConvertFrom-AppTelemetryLine {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Line
    )

    $normalizedLine = $Line.TrimEnd([char]13)
    $tokens = $normalizedLine.Split(
        [char[]]@(','),
        [StringSplitOptions]::None)
    if ($tokens.Count -ne 52) {
        throw "遥测字段数量应为 52，实际为 $($tokens.Count)"
    }

    $expectedLabels = [ordered]@{
        0 = 'T'
        4 = 'R'
        9 = 'D'
        14 = 'E'
        19 = 'P'
        22 = 'B'
        24 = 'S'
        31 = 'I'
        34 = 'J'
        37 = 'L'
        40 = 'M'
        43 = 'C'
    }
    foreach ($entry in $expectedLabels.GetEnumerator()) {
        if ($tokens[[int]$entry.Key] -cne [string]$entry.Value) {
            throw "遥测标签位置 $($entry.Key) 应为 $($entry.Value)，实际为 $($tokens[[int]$entry.Key])"
        }
    }

    $record = [ordered]@{}
    $u32Maximum = [uint64][uint32]::MaxValue
    $u16Maximum = [uint64][uint16]::MaxValue

    $record.board_now_ms = ConvertTo-AppTelemetryUnsigned $tokens[1] $u32Maximum 'board_now_ms'
    $record.control_tick_sequence = ConvertTo-AppTelemetryUnsigned $tokens[2] $u32Maximum 'control_tick_sequence'
    $record.irq_timestamp_cycles = ConvertTo-AppTelemetryUnsigned $tokens[3] $u32Maximum 'irq_timestamp_cycles'

    for ($index = 0; $index -lt 4; $index++) {
        $record["encoder_raw_$($index + 1)"] =
            ConvertTo-AppTelemetryUnsigned $tokens[5 + $index] $u16Maximum "encoder_raw_$($index + 1)"
        $record["encoder_delta_$($index + 1)"] =
            ConvertTo-AppTelemetrySigned $tokens[10 + $index] ([int16]::MinValue) ([int16]::MaxValue) "encoder_delta_$($index + 1)"
        $record["encoder_total_$($index + 1)"] =
            ConvertTo-AppTelemetrySigned $tokens[15 + $index] ([int64]::MinValue) ([int64]::MaxValue) "encoder_total_$($index + 1)"
    }

    $record.target_pwm = ConvertTo-AppTelemetrySigned $tokens[20] ([int32]::MinValue) ([int32]::MaxValue) 'target_pwm'
    $record.applied_pwm = ConvertTo-AppTelemetrySigned $tokens[21] ([int32]::MinValue) ([int32]::MaxValue) 'applied_pwm'
    $record.battery_mv = ConvertTo-AppTelemetryUnsigned $tokens[23] $u16Maximum 'battery_mv'

    $systemNames = @(
        'critical_tasks_alive',
        'runtime_ready',
        'motion_inhibited',
        'fault_latched',
        'motor_state',
        'estop_latched'
    )
    for ($index = 0; $index -lt $systemNames.Count; $index++) {
        $fieldName = $systemNames[$index]
        $maximum = if ($fieldName -eq 'motor_state') { $u32Maximum } else { [uint64]1 }
        $record[$fieldName] =
            ConvertTo-AppTelemetryUnsigned $tokens[25 + $index] $maximum $fieldName
    }

    $record.imu_sample_age_ms = ConvertTo-AppTelemetryUnsigned $tokens[32] $u32Maximum 'imu_sample_age_ms'
    $record.imu_health = ConvertTo-AppTelemetryUnsigned $tokens[33] $u32Maximum 'imu_health'
    $record.irq_jitter_cycles = ConvertTo-AppTelemetryUnsigned $tokens[35] $u32Maximum 'irq_jitter_cycles'
    $record.irq_jitter_max_cycles = ConvertTo-AppTelemetryUnsigned $tokens[36] $u32Maximum 'irq_jitter_max_cycles'
    $record.wake_latency_cycles = ConvertTo-AppTelemetryUnsigned $tokens[38] $u32Maximum 'wake_latency_cycles'
    $record.wcet_max_cycles = ConvertTo-AppTelemetryUnsigned $tokens[39] $u32Maximum 'wcet_max_cycles'
    $record.missed_period_count = ConvertTo-AppTelemetryUnsigned $tokens[41] $u32Maximum 'missed_period_count'
    $record.deadline_miss_count = ConvertTo-AppTelemetryUnsigned $tokens[42] $u32Maximum 'deadline_miss_count'

    $communicationNames = @(
        'uart_error_count',
        'uart_rx_overflow_count',
        'uart_tx_fault_count',
        'command_reject_count',
        'command_queue_drop_count',
        'motion_gate_reject_count',
        'invalidated_motor_command_count',
        'adc_error_count'
    )
    for ($index = 0; $index -lt $communicationNames.Count; $index++) {
        $fieldName = $communicationNames[$index]
        $record[$fieldName] =
            ConvertTo-AppTelemetryUnsigned $tokens[44 + $index] $u32Maximum $fieldName
    }

    return [pscustomobject]$record
}

function ConvertFrom-AppStatLine {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Line
    )

    $normalizedLine = $Line.TrimEnd([char]13)
    if (-not $normalizedLine.StartsWith('STAT,1,', [StringComparison]::Ordinal)) {
        throw 'STAT 遥测必须使用已知的 schema 1'
    }
    $record = ConvertFrom-AppTelemetryLine -Line ('T,' + $normalizedLine.Substring(7))
    if ([uint64]$record.motor_state -gt 6) {
        throw "STAT motor_state 超出已知状态范围：$($record.motor_state)"
    }
    if ([uint64]$record.imu_health -gt 5) {
        throw "STAT imu_health 超出已知健康范围：$($record.imu_health)"
    }
    if ([int64]$record.target_pwm -lt [int16]::MinValue -or
        [int64]$record.target_pwm -gt [int16]::MaxValue -or
        [int64]$record.applied_pwm -lt [int16]::MinValue -or
        [int64]$record.applied_pwm -gt [int16]::MaxValue) {
        throw 'STAT PWM 超出固件 int16 范围'
    }
    return $record
}

function Get-AppImuqColumnNames {
    return @(
        'schema_version', 'board_now_ms', 'imu_sequence',
        'sensor_timestamp', 'sample_age_ms', 'health', 'flags',
        'read_error_count', 'consecutive_error_count', 'backoff_count',
        'retry_delay_ms', 'duplicate_count', 'dropped_sample_count',
        'accel_update_accept_count', 'accel_update_reject_count',
        'spike_reject_count', 'consecutive_spike_count',
        'stable_sample_count', 'estimator_fault_count'
    )
}

function ConvertFrom-AppImuqLine {
    param([Parameter(Mandatory = $true)][string]$Line)

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','), [StringSplitOptions]::None)
    if ($tokens.Count -ne 23 -or $tokens[0] -cne 'IMUQ' -or
        $tokens[1] -cne '1' -or $tokens[3] -cne 'Q' -or
        $tokens[9] -cne 'R' -or $tokens[16] -cne 'A') {
        throw 'IMUQ 遥测必须是 23 字段的 IMUQ,1 帧'
    }
    $u32Maximum = [uint64][uint32]::MaxValue
    $record = [ordered]@{
        schema_version = [uint64]1
        board_now_ms = ConvertTo-AppTelemetryUnsigned $tokens[2] $u32Maximum 'board_now_ms'
    }
    $names = @(
        'imu_sequence', 'sensor_timestamp', 'sample_age_ms', 'health', 'flags',
        'read_error_count', 'consecutive_error_count', 'backoff_count',
        'retry_delay_ms', 'duplicate_count', 'dropped_sample_count',
        'accel_update_accept_count', 'accel_update_reject_count',
        'spike_reject_count', 'consecutive_spike_count',
        'stable_sample_count', 'estimator_fault_count'
    )
    $indexes = @(4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 22)
    for ($index = 0; $index -lt $names.Count; $index++) {
        $maximum = if ($names[$index] -eq 'health') { [uint64]5 } else { $u32Maximum }
        $record[$names[$index]] = ConvertTo-AppTelemetryUnsigned `
            $tokens[$indexes[$index]] $maximum $names[$index]
    }
    return [pscustomobject]$record
}

function Get-AppResourceColumnNames {
    return @(
        'schema_version', 'board_now_ms', 'irq_jitter_cycles',
        'irq_jitter_max_cycles', 'wake_latency_cycles',
        'wake_latency_max_cycles', 'wake_latency_p99_us', 'wcet_cycles',
        'wcet_max_cycles', 'timer_irq_missed_period_count',
        'task_iteration_missed_period_count', 'deadline_miss_count',
        'control_stack_free_bytes', 'safety_stack_free_bytes',
        'comm_stack_free_bytes', 'imu_stack_free_bytes',
        'monitor_stack_free_bytes', 'minimum_free_heap_bytes',
        'health_miss_count', 'uart_tx_queued_frame_count',
        'uart_error_count', 'uart_rx_overflow_count', 'uart_tx_fault_count',
        'telemetry_enqueued_count', 'telemetry_enqueue_drop_count',
        'telemetry_format_error_count', 'capture_event_drop_count',
        'capture_export_error_count', 'command_queue_drop_count',
        'motion_gate_reject_count', 'invalidated_motor_command_count',
        'adc_error_count', 'stat_frame_failure_count',
        'imuq_frame_failure_count', 'res_frame_failure_count',
        'event_frame_failure_count'
    )
}

function ConvertFrom-AppResourceLine {
    param([Parameter(Mandatory = $true)][string]$Line)

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','), [StringSplitOptions]::None)
    if ($tokens.Count -ne 42 -or $tokens[0] -cne 'RES' -or
        $tokens[1] -cne '1' -or $tokens[3] -cne 'T' -or
        $tokens[14] -cne 'S' -or $tokens[21] -cne 'H' -or
        $tokens[23] -cne 'U' -or $tokens[37] -cne 'F') {
        throw 'RES 遥测必须是 42 字段的 RES,1 帧'
    }
    $names = @(Get-AppResourceColumnNames)
    $indexes = @(1, 2) + @(4..13) + @(15..20) + @(22) + @(24..36) + @(38..41)
    $u32Maximum = [uint64][uint32]::MaxValue
    $record = [ordered]@{}
    for ($index = 0; $index -lt $names.Count; $index++) {
        $maximum = if ($names[$index] -eq 'uart_tx_queued_frame_count') {
            [uint64]4
        } else {
            $u32Maximum
        }
        $record[$names[$index]] = ConvertTo-AppTelemetryUnsigned `
            $tokens[$indexes[$index]] $maximum $names[$index]
    }
    return [pscustomobject]$record
}

function Get-AppEventColumnNames {
    return @(
        'schema_version', 'board_now_ms', 'event_sequence', 'event_flags',
        'runtime_ready', 'motion_inhibited', 'fault_latched', 'motor_state',
        'estop_latched', 'imu_health', 'uart_error_count',
        'telemetry_enqueue_drop_count', 'telemetry_format_error_count',
        'adc_error_count', 'invalidated_motor_command_count'
    )
}

function ConvertFrom-AppEventLine {
    param([Parameter(Mandatory = $true)][string]$Line)

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','), [StringSplitOptions]::None)
    if ($tokens.Count -ne 18 -or $tokens[0] -cne 'EVENT' -or
        $tokens[1] -cne '1' -or $tokens[5] -cne 'S' -or
        $tokens[12] -cne 'C') {
        throw 'EVENT 遥测必须是 18 字段的 EVENT,1 帧'
    }
    $names = @(Get-AppEventColumnNames)
    $indexes = @(1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17)
    $u32Maximum = [uint64][uint32]::MaxValue
    $record = [ordered]@{}
    for ($index = 0; $index -lt $names.Count; $index++) {
        $name = $names[$index]
        $maximum = switch ($name) {
            { $_ -in @('runtime_ready', 'motion_inhibited', 'fault_latched', 'estop_latched') } { [uint64]1; break }
            'motor_state' { [uint64]6; break }
            'imu_health' { [uint64]5; break }
            default { $u32Maximum }
        }
        $record[$name] = ConvertTo-AppTelemetryUnsigned `
            $tokens[$indexes[$index]] $maximum $name
    }
    if ($record.event_flags -eq 0 -or $record.event_flags -gt 2047) {
        throw "EVENT event_flags 必须是已知非零位掩码：$($record.event_flags)"
    }
    return [pscustomobject]$record
}

function Get-MotorCaptureColumnNames {
    return @(
        'capture_index',
        'control_tick_sequence',
        'irq_timestamp_cycles',
        'wake_latency_cycles',
        'previous_wcet_cycles',
        'encoder_raw_ma',
        'encoder_delta_ma',
        'target_pwm',
        'applied_pwm',
        'battery_mv',
        'motor_state',
        'safety_flags'
    )
}

function ConvertFrom-MotorCaptureSampleLine {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Line
    )

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','),
        [StringSplitOptions]::None)
    if ($tokens.Count -ne 14 -or $tokens[0] -cne 'MC' -or $tokens[1] -cne '1') {
        throw '高速电机样本必须是 14 字段的 MC,1 帧'
    }

    $u32Maximum = [uint64][uint32]::MaxValue
    $u16Maximum = [uint64][uint16]::MaxValue
    $record = [ordered]@{}
    $record.capture_index =
        ConvertTo-AppTelemetryUnsigned $tokens[2] $u32Maximum 'capture_index'
    $record.control_tick_sequence =
        ConvertTo-AppTelemetryUnsigned $tokens[3] $u32Maximum 'control_tick_sequence'
    $record.irq_timestamp_cycles =
        ConvertTo-AppTelemetryUnsigned $tokens[4] $u32Maximum 'irq_timestamp_cycles'
    $record.wake_latency_cycles =
        ConvertTo-AppTelemetryUnsigned $tokens[5] $u32Maximum 'wake_latency_cycles'
    $record.previous_wcet_cycles =
        ConvertTo-AppTelemetryUnsigned $tokens[6] $u32Maximum 'previous_wcet_cycles'
    $record.encoder_raw_ma =
        ConvertTo-AppTelemetryUnsigned $tokens[7] $u16Maximum 'encoder_raw_ma'
    $record.encoder_delta_ma =
        ConvertTo-AppTelemetrySigned $tokens[8] ([int16]::MinValue) ([int16]::MaxValue) 'encoder_delta_ma'
    $record.target_pwm =
        ConvertTo-AppTelemetrySigned $tokens[9] ([int16]::MinValue) ([int16]::MaxValue) 'target_pwm'
    $record.applied_pwm =
        ConvertTo-AppTelemetrySigned $tokens[10] ([int16]::MinValue) ([int16]::MaxValue) 'applied_pwm'
    $record.battery_mv =
        ConvertTo-AppTelemetryUnsigned $tokens[11] $u16Maximum 'battery_mv'
    $record.motor_state =
        ConvertTo-AppTelemetryUnsigned $tokens[12] 6 'motor_state'
    $record.safety_flags =
        ConvertTo-AppTelemetryUnsigned $tokens[13] 31 'safety_flags'
    return [pscustomobject]$record
}

function Get-MotorCaptureEventColumnNames {
    return @(
        'event',
        'state',
        'sample_count',
        'capacity',
        'dropped_sample_count'
    )
}

function ConvertFrom-MotorCaptureEventLine {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Line
    )

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','),
        [StringSplitOptions]::None)
    if ($tokens.Count -ne 7 -or $tokens[0] -cne 'MCAP' -or $tokens[1] -cne '1') {
        throw '高速电机事件必须是 7 字段的 MCAP,1 帧'
    }
    $allowedEvents = @('STATUS', 'STARTED', 'STOPPED', 'BEGIN', 'END', 'REJECTED')
    if ($tokens[2] -cnotin $allowedEvents) {
        throw "未知高速电机事件：$($tokens[2])"
    }

    $u32Maximum = [uint64][uint32]::MaxValue
    return [pscustomobject][ordered]@{
        event = $tokens[2]
        state =
            ConvertTo-AppTelemetryUnsigned $tokens[3] 2 'capture_state'
        sample_count =
            ConvertTo-AppTelemetryUnsigned $tokens[4] $u32Maximum 'sample_count'
        capacity =
            ConvertTo-AppTelemetryUnsigned $tokens[5] $u32Maximum 'capacity'
        dropped_sample_count =
            ConvertTo-AppTelemetryUnsigned $tokens[6] $u32Maximum 'dropped_sample_count'
    }
}

function Get-SpeedCaptureColumnNames {
    return @(
        'capture_index',
        'control_tick_sequence',
        'irq_timestamp_cycles',
        'encoder_delta_ma',
        'applied_pwm',
        'period_sum_cycles',
        'period_count',
        'last_edge_age_cycles',
        'event_sequence',
        'direction',
        'period_flags'
    )
}

function ConvertFrom-SpeedCaptureSampleLine {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Line
    )

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','),
        [StringSplitOptions]::None)
    if ($tokens.Count -ne 13 -or $tokens[0] -cne 'SC' -or $tokens[1] -cne '1') {
        throw 'G3 轮速样本必须是 13 字段的 SC,1 帧'
    }

    $u32Maximum = [uint64][uint32]::MaxValue
    $u16Maximum = [uint64][uint16]::MaxValue
    return [pscustomobject][ordered]@{
        capture_index =
            ConvertTo-AppTelemetryUnsigned $tokens[2] $u32Maximum 'capture_index'
        control_tick_sequence =
            ConvertTo-AppTelemetryUnsigned $tokens[3] $u32Maximum 'control_tick_sequence'
        irq_timestamp_cycles =
            ConvertTo-AppTelemetryUnsigned $tokens[4] $u32Maximum 'irq_timestamp_cycles'
        encoder_delta_ma =
            ConvertTo-AppTelemetrySigned $tokens[5] ([int16]::MinValue) ([int16]::MaxValue) 'encoder_delta_ma'
        applied_pwm =
            ConvertTo-AppTelemetrySigned $tokens[6] ([int16]::MinValue) ([int16]::MaxValue) 'applied_pwm'
        period_sum_cycles =
            ConvertTo-AppTelemetryUnsigned $tokens[7] $u32Maximum 'period_sum_cycles'
        period_count =
            ConvertTo-AppTelemetryUnsigned $tokens[8] $u16Maximum 'period_count'
        last_edge_age_cycles =
            ConvertTo-AppTelemetryUnsigned $tokens[9] $u32Maximum 'last_edge_age_cycles'
        event_sequence =
            ConvertTo-AppTelemetryUnsigned $tokens[10] $u32Maximum 'event_sequence'
        direction =
            ConvertTo-AppTelemetrySigned $tokens[11] -1 1 'direction'
        period_flags =
            ConvertTo-AppTelemetryUnsigned $tokens[12] 31 'period_flags'
    }
}

function Get-SpeedCaptureEventColumnNames {
    return @(
        'event',
        'state',
        'sample_count',
        'capacity',
        'dropped_sample_count',
        'invalid_direction_count',
        'zero_period_count',
        'aggregate_drop_count',
        'direction_reset_count'
    )
}

function ConvertFrom-SpeedCaptureEventLine {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Line
    )

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','),
        [StringSplitOptions]::None)
    if ($tokens.Count -ne 11 -or $tokens[0] -cne 'SCAP' -or $tokens[1] -cne '1') {
        throw 'G3 轮速事件必须是 11 字段的 SCAP,1 帧'
    }
    $allowedEvents = @('STATUS', 'STARTED', 'STOPPED', 'BEGIN', 'END', 'REJECTED')
    if ($tokens[2] -cnotin $allowedEvents) {
        throw "未知 G3 轮速事件：$($tokens[2])"
    }

    $u32Maximum = [uint64][uint32]::MaxValue
    return [pscustomobject][ordered]@{
        event = $tokens[2]
        state = ConvertTo-AppTelemetryUnsigned $tokens[3] 2 'capture_state'
        sample_count =
            ConvertTo-AppTelemetryUnsigned $tokens[4] $u32Maximum 'sample_count'
        capacity =
            ConvertTo-AppTelemetryUnsigned $tokens[5] $u32Maximum 'capacity'
        dropped_sample_count =
            ConvertTo-AppTelemetryUnsigned $tokens[6] $u32Maximum 'dropped_sample_count'
        invalid_direction_count =
            ConvertTo-AppTelemetryUnsigned $tokens[7] $u32Maximum 'invalid_direction_count'
        zero_period_count =
            ConvertTo-AppTelemetryUnsigned $tokens[8] $u32Maximum 'zero_period_count'
        aggregate_drop_count =
            ConvertTo-AppTelemetryUnsigned $tokens[9] $u32Maximum 'aggregate_drop_count'
        direction_reset_count =
            ConvertTo-AppTelemetryUnsigned $tokens[10] $u32Maximum 'direction_reset_count'
    }
}

function Get-ImuCaptureColumnNames {
    return @(
        'capture_index', 'imu_sequence', 'sensor_timestamp',
        'host_tick_ms', 'flags', 'source_dropped_sample_count',
        'raw_accel_x', 'raw_accel_y', 'raw_accel_z',
        'raw_gyro_x', 'raw_gyro_y', 'raw_gyro_z',
        'raw_temperature', 'sensor_status', 'health'
    )
}

function ConvertFrom-ImuCaptureSampleLine {
    param([Parameter(Mandatory = $true)][string]$Line)

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','), [StringSplitOptions]::None)
    if ($tokens.Count -ne 17 -or $tokens[0] -cne 'IC' -or
        $tokens[1] -cne '1') {
        throw 'IMU 高速样本必须是 17 字段的 IC,1 帧'
    }
    $u32Maximum = [uint64][uint32]::MaxValue
    $record = [ordered]@{
        capture_index =
            ConvertTo-AppTelemetryUnsigned $tokens[2] $u32Maximum 'capture_index'
        imu_sequence =
            ConvertTo-AppTelemetryUnsigned $tokens[3] $u32Maximum 'imu_sequence'
        sensor_timestamp =
            ConvertTo-AppTelemetryUnsigned $tokens[4] 16777215 'sensor_timestamp'
        host_tick_ms =
            ConvertTo-AppTelemetryUnsigned $tokens[5] $u32Maximum 'host_tick_ms'
        flags =
            ConvertTo-AppTelemetryUnsigned $tokens[6] 65535 'flags'
        source_dropped_sample_count =
            ConvertTo-AppTelemetryUnsigned $tokens[7] $u32Maximum 'source_dropped_sample_count'
    }
    $rawNames = @(
        'raw_accel_x', 'raw_accel_y', 'raw_accel_z',
        'raw_gyro_x', 'raw_gyro_y', 'raw_gyro_z', 'raw_temperature')
    for ($index = 0; $index -lt $rawNames.Count; $index++) {
        $record[$rawNames[$index]] = ConvertTo-AppTelemetrySigned `
            $tokens[8 + $index] ([int16]::MinValue) ([int16]::MaxValue) $rawNames[$index]
    }
    $record.sensor_status =
        ConvertTo-AppTelemetryUnsigned $tokens[15] 255 'sensor_status'
    $record.health =
        ConvertTo-AppTelemetryUnsigned $tokens[16] 5 'health'
    return [pscustomobject]$record
}

function Get-ImuCaptureEventColumnNames {
    return @(
        'event', 'state', 'sample_count', 'capacity',
        'dropped_sample_count', 'duplicate_sequence_count',
        'source_gap_count'
    )
}

function ConvertFrom-ImuCaptureEventLine {
    param([Parameter(Mandatory = $true)][string]$Line)

    $tokens = $Line.TrimEnd([char]13).Split(
        [char[]]@(','), [StringSplitOptions]::None)
    if ($tokens.Count -ne 9 -or $tokens[0] -cne 'ICAP' -or
        $tokens[1] -cne '1') {
        throw 'IMU 高速事件必须是 9 字段的 ICAP,1 帧'
    }
    $allowedEvents = @('STATUS', 'STARTED', 'STOPPED', 'BEGIN', 'END', 'REJECTED')
    if ($tokens[2] -cnotin $allowedEvents) {
        throw "未知 IMU 高速事件：$($tokens[2])"
    }
    $u32Maximum = [uint64][uint32]::MaxValue
    return [pscustomobject][ordered]@{
        event = $tokens[2]
        state = ConvertTo-AppTelemetryUnsigned $tokens[3] 2 'capture_state'
        sample_count =
            ConvertTo-AppTelemetryUnsigned $tokens[4] $u32Maximum 'sample_count'
        capacity =
            ConvertTo-AppTelemetryUnsigned $tokens[5] $u32Maximum 'capacity'
        dropped_sample_count =
            ConvertTo-AppTelemetryUnsigned $tokens[6] $u32Maximum 'dropped_sample_count'
        duplicate_sequence_count =
            ConvertTo-AppTelemetryUnsigned $tokens[7] $u32Maximum 'duplicate_sequence_count'
        source_gap_count =
            ConvertTo-AppTelemetryUnsigned $tokens[8] $u32Maximum 'source_gap_count'
    }
}

function ConvertTo-CaptureCsvField {
    param(
        [AllowNull()]
        [object]$Value
    )

    $text = if ($null -eq $Value) {
        ''
    } else {
        [Convert]::ToString($Value, [Globalization.CultureInfo]::InvariantCulture)
    }
    return '"' + $text.Replace('"', '""') + '"'
}

function ConvertTo-CaptureCsvLine {
    param(
        [Parameter(Mandatory = $true)]
        [AllowNull()]
        [AllowEmptyCollection()]
        [object[]]$Values
    )

    return (($Values | ForEach-Object { ConvertTo-CaptureCsvField $_ }) -join ',')
}

function Assert-SerialCaptureCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Command,
        [switch]$AllowNonStatusCommands
    )

    if ([string]::IsNullOrWhiteSpace($Command) -or $Command -cne $Command.Trim()) {
        throw '命令不能为空，也不能包含首尾空白'
    }
    if ($Command.IndexOfAny([char[]]@([char]10, [char]13)) -ge 0) {
        throw '单条命令不能包含 CR 或 LF'
    }
    foreach ($character in $Command.ToCharArray()) {
        if ([int]$character -lt 32 -or [int]$character -gt 126) {
            throw '命令只能包含可打印 ASCII 字符'
        }
    }
    if ([Text.Encoding]::ASCII.GetByteCount($Command) -gt 64) {
        throw '命令在 LF 前不能超过 64 字节'
    }
    if (-not $AllowNonStatusCommands -and $Command -cne 'STATUS') {
        throw "默认安全模式只允许 STATUS；发送其他命令必须显式使用 -AllowNonStatusCommands：$Command"
    }
}

function Get-SerialCaptureCommandSchedule {
    param(
        [AllowEmptyString()]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [uint64]$DurationMilliseconds,
        [switch]$SendStatusAtStart,
        [switch]$AllowNonStatusCommands
    )

    $entries = [Collections.Generic.List[object]]::new()
    if ($SendStatusAtStart) {
        $entries.Add([pscustomobject]@{
            planned_elapsed_ms = [uint64]0
            command = 'STATUS'
        })
    }

    if (-not [string]::IsNullOrWhiteSpace($Path)) {
        if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
            throw "命令计划文件不存在：$Path"
        }
        $rows = @(Import-Csv -LiteralPath $Path)
        foreach ($row in $rows) {
            if ($null -eq $row.PSObject.Properties['elapsed_ms'] -or
                $null -eq $row.PSObject.Properties['command']) {
                throw '命令计划 CSV 必须包含 elapsed_ms 和 command 两列'
            }
            [uint64]$elapsed = 0
            $parsed = [uint64]::TryParse(
                [string]$row.elapsed_ms,
                [Globalization.NumberStyles]::None,
                [Globalization.CultureInfo]::InvariantCulture,
                [ref]$elapsed)
            if (-not $parsed -or $elapsed -ge $DurationMilliseconds) {
                throw "命令计划 elapsed_ms 必须小于采集时长 $DurationMilliseconds ms：$($row.elapsed_ms)"
            }
            $command = [string]$row.command
            Assert-SerialCaptureCommand -Command $command -AllowNonStatusCommands:$AllowNonStatusCommands
            $entries.Add([pscustomobject]@{
                planned_elapsed_ms = $elapsed
                command = $command
            })
        }
    }

    return @($entries | Sort-Object planned_elapsed_ms)
}
