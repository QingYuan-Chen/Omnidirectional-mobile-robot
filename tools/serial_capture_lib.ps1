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
