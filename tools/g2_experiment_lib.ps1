function New-G2FirstMotionSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [ValidateRange(1, 200)]
        [int]$PeakPwm,

        [ValidateRange(250, 10000)]
        [int]$HoldMilliseconds = 1000,

        [ValidateRange(50, 400)]
        [int]$KeepAliveMilliseconds = 250,

        [ValidateRange(1, 4294967295)]
        [uint32]$SequenceStart = 1
    )

    $drivePwm = if ($Direction -ceq 'Positive') { $PeakPwm } else { -$PeakPwm }
    $schedule = [Collections.Generic.List[object]]::new()
    $schedule.Add([pscustomobject]@{
        elapsed_ms = [uint64]0
        command = 'STATUS'
    })

    [uint64]$sequence = $SequenceStart
    $armAt = [uint64]1000
    $driveAt = [uint64]1250
    $driveEnd = $driveAt + [uint64]$HoldMilliseconds
    $schedule.Add([pscustomobject]@{
        elapsed_ms = $armAt
        command = "ARM $sequence"
    })
    $sequence++

    for ($sendAt = $driveAt;
         $sendAt -lt $driveEnd;
         $sendAt += [uint64]$KeepAliveMilliseconds) {
        if ($sequence -gt [uint64][uint32]::MaxValue) {
            throw '命令序号超过 uint32 范围'
        }
        $schedule.Add([pscustomobject]@{
            elapsed_ms = $sendAt
            command = "PWM $sequence $drivePwm"
        })
        $sequence++
    }

    if ($sequence -gt [uint64][uint32]::MaxValue) {
        throw '命令序号超过 uint32 范围'
    }
    $schedule.Add([pscustomobject]@{
        elapsed_ms = $driveEnd
        command = "STOP $sequence"
    })

    $statusAt = $driveEnd + [uint64]$PeakPwm + [uint64]500
    $schedule.Add([pscustomobject]@{
        elapsed_ms = $statusAt
        command = 'STATUS'
    })

    [void](Test-G2FirstMotionSchedule `
        -Schedule $schedule `
        -Direction $Direction `
        -PeakPwm $PeakPwm `
        -MaximumKeepAliveMilliseconds 400)
    return @($schedule)
}

function New-G2DeadzoneProbeSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [ValidateRange(1, 840)]
        [int]$PeakPwm,

        [ValidateRange(250, 10000)]
        [int]$HoldMilliseconds = 1000,

        [ValidateRange(50, 400)]
        [int]$KeepAliveMilliseconds = 250,

        [ValidateRange(1, 4294967295)]
        [uint32]$SequenceStart = 1
    )

    $drivePwm = if ($Direction -ceq 'Positive') { $PeakPwm } else { -$PeakPwm }
    $schedule = [Collections.Generic.List[object]]::new()
    $schedule.Add([pscustomobject]@{
        elapsed_ms = [uint64]0
        command = 'STATUS'
    })

    [uint64]$sequence = $SequenceStart
    $armAt = [uint64]1000
    $driveAt = [uint64]1250
    $driveEnd = $driveAt + [uint64]$HoldMilliseconds
    $schedule.Add([pscustomobject]@{
        elapsed_ms = $armAt
        command = "ARM $sequence"
    })
    $sequence++

    for ($sendAt = $driveAt;
         $sendAt -lt $driveEnd;
         $sendAt += [uint64]$KeepAliveMilliseconds) {
        if ($sequence -gt [uint64][uint32]::MaxValue) {
            throw '命令序号超过 uint32 范围'
        }
        $schedule.Add([pscustomobject]@{
            elapsed_ms = $sendAt
            command = "PWM $sequence $drivePwm"
        })
        $sequence++
    }

    if ($sequence -gt [uint64][uint32]::MaxValue) {
        throw '命令序号超过 uint32 范围'
    }
    $schedule.Add([pscustomobject]@{
        elapsed_ms = $driveEnd
        command = "STOP $sequence"
    })

    $statusAt = $driveEnd + [uint64]$PeakPwm + [uint64]500
    $schedule.Add([pscustomobject]@{
        elapsed_ms = $statusAt
        command = 'STATUS'
    })

    [void](Test-G2FirstMotionSchedule `
        -Schedule $schedule `
        -Direction $Direction `
        -PeakPwm $PeakPwm `
        -MaximumKeepAliveMilliseconds 400)
    return @($schedule)
}

function Test-G2FirstMotionSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Schedule,

        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [ValidateRange(1, 840)]
        [int]$PeakPwm,

        [ValidateRange(50, 499)]
        [int]$MaximumKeepAliveMilliseconds = 400
    )

    if ($Schedule.Count -lt 5) {
        throw '首动计划至少需要 STATUS、ARM、PWM、STOP 和结束 STATUS'
    }
    $previousElapsed = [uint64]0
    [uint64]$previousSequence = 0
    $hasSequence = $false
    $armCount = 0
    $stopCount = 0
    $pwmCount = 0
    $lastMotionCommandAt = $null
    $expectedPwm = if ($Direction -ceq 'Positive') { $PeakPwm } else { -$PeakPwm }

    for ($index = 0; $index -lt $Schedule.Count; $index++) {
        $entry = $Schedule[$index]
        [uint64]$elapsed = $entry.elapsed_ms
        $command = [string]$entry.command
        if ($index -gt 0 -and $elapsed -le $previousElapsed) {
            throw '计划时间必须严格递增'
        }
        if ($command -ceq 'ESTOP') {
            throw '首动计划禁止包含 ESTOP'
        }

        if ($command -match '^(ARM|PWM|STOP) ([0-9]+)(?: (-?[0-9]+))?$') {
            [uint64]$sequence = 0
            if (-not [uint64]::TryParse($Matches[2], [ref]$sequence) -or
                $sequence -gt [uint64][uint32]::MaxValue) {
                throw "非法命令序号：$command"
            }
            if ($hasSequence -and $sequence -ne $previousSequence + 1) {
                throw '首动计划的运动命令序号必须连续递增'
            }
            $previousSequence = $sequence
            $hasSequence = $true

            if ($Matches[1] -ceq 'ARM') {
                $armCount++
            } elseif ($Matches[1] -ceq 'STOP') {
                $stopCount++
            } else {
                $pwmCount++
                if ([int]$Matches[3] -ne $expectedPwm) {
                    throw "PWM 方向或幅值与计划不一致：$command"
                }
            }

            if ($null -ne $lastMotionCommandAt -and
                ($elapsed - [uint64]$lastMotionCommandAt) -gt
                [uint64]$MaximumKeepAliveMilliseconds) {
                throw 'ARM 至 STOP 之间的运动命令间隔超过保活安全余量'
            }
            $lastMotionCommandAt = $elapsed
        } elseif ($command -cne 'STATUS') {
            throw "首动计划包含未知命令：$command"
        }
        $previousElapsed = $elapsed
    }

    if ($Schedule[0].command -cne 'STATUS' -or
        $Schedule[-1].command -cne 'STATUS' -or
        $armCount -ne 1 -or
        $stopCount -ne 1 -or
        $pwmCount -lt 1) {
        throw '首动计划结构不完整'
    }
    return $true
}

function Measure-G2DeadzoneMotion {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Rows,

        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [ValidateRange(1, 1000000)]
        [int64]$MotionThresholdCounts = 1000,

        [ValidateRange(1, 1000000)]
        [int64]$OtherChannelLimitCounts = 1000
    )

    if ($Rows.Count -lt 2) {
        throw '死区判定至少需要两行遥测'
    }
    foreach ($channel in 1..4) {
        $propertyName = "encoder_total_$channel"
        if ($null -eq $Rows[0].PSObject.Properties[$propertyName]) {
            throw "遥测缺少字段：$propertyName"
        }
    }

    $directionSign = if ($Direction -ceq 'Positive') { [int64]1 } else { [int64]-1 }
    $baseline = [int64]$Rows[0].encoder_total_1
    [int64]$expectedExcursion = 0
    [int64]$oppositeExcursion = 0
    [int64]$otherChannelMaximum = 0

    foreach ($row in $Rows) {
        $signedDisplacement = $directionSign * ([int64]$row.encoder_total_1 - $baseline)
        if ($signedDisplacement -gt $expectedExcursion) {
            $expectedExcursion = $signedDisplacement
        }
        if (-$signedDisplacement -gt $oppositeExcursion) {
            $oppositeExcursion = -$signedDisplacement
        }

        foreach ($channel in 2..4) {
            $propertyName = "encoder_total_$channel"
            $channelBaseline = [int64]$Rows[0].$propertyName
            $channelExcursion = [Math]::Abs([int64]$row.$propertyName - $channelBaseline)
            if ($channelExcursion -gt $otherChannelMaximum) {
                $otherChannelMaximum = $channelExcursion
            }
        }
    }

    $expectedMotionDetected = $expectedExcursion -ge $MotionThresholdCounts
    $wrongDirectionDetected = $oppositeExcursion -ge $MotionThresholdCounts
    $otherChannelMotionDetected = $otherChannelMaximum -ge $OtherChannelLimitCounts

    return [pscustomobject][ordered]@{
        direction = $Direction
        motion_threshold_counts = $MotionThresholdCounts
        other_channel_limit_counts = $OtherChannelLimitCounts
        expected_direction_excursion_counts = $expectedExcursion
        opposite_direction_excursion_counts = $oppositeExcursion
        other_channel_maximum_excursion_counts = $otherChannelMaximum
        expected_motion_detected = $expectedMotionDetected
        wrong_direction_detected = $wrongDirectionDetected
        other_channel_motion_detected = $otherChannelMotionDetected
        moved = ($expectedMotionDetected -and
            -not $wrongDirectionDetected -and
            -not $otherChannelMotionDetected)
    }
}

function Get-G2UInt32Delta {
    param(
        [Parameter(Mandatory = $true)]
        [uint32]$Previous,
        [Parameter(Mandatory = $true)]
        [uint32]$Current
    )

    if ($Current -ge $Previous) {
        return [uint64]($Current - $Previous)
    }
    return ([uint64][uint32]::MaxValue - [uint64]$Previous) + 1 + [uint64]$Current
}

function Get-G2Percentile {
    param(
        [Parameter(Mandatory = $true)]
        [double[]]$Values,
        [Parameter(Mandatory = $true)]
        [ValidateRange(0.0, 1.0)]
        [double]$Probability
    )

    if ($Values.Count -eq 0) {
        throw '百分位输入不能为空'
    }
    $sorted = @($Values | Sort-Object)
    $rank = [Math]::Ceiling($Probability * $sorted.Count)
    $index = [Math]::Max(0, [int]$rank - 1)
    return [double]$sorted[$index]
}

function Test-G2PreliminaryTimingThresholds {
    param(
        [Parameter(Mandatory = $true)]
        [object]$Thresholds
    )

    $values = if ($Thresholds -is [Collections.IDictionary]) {
        @($Thresholds.Values)
    } else {
        @($Thresholds.PSObject.Properties | ForEach-Object { $_.Value })
    }
    return @($values | Where-Object { -not [bool]$_ }).Count -eq 0
}

function Measure-G2CaptureRows {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Rows
    )

    if ($Rows.Count -lt 2) {
        throw '至少需要两行遥测才能计算时间差和计数率'
    }

    $requiredColumns = @(
        'board_now_ms',
        'control_tick_sequence',
        'target_pwm',
        'applied_pwm',
        'battery_mv',
        'irq_jitter_cycles',
        'irq_jitter_max_cycles',
        'wake_latency_cycles',
        'wcet_max_cycles',
        'missed_period_count',
        'deadline_miss_count',
        'encoder_total_1',
        'encoder_total_2',
        'encoder_total_3',
        'encoder_total_4',
        'uart_error_count',
        'uart_rx_overflow_count',
        'uart_tx_fault_count',
        'command_reject_count',
        'command_queue_drop_count',
        'motion_gate_reject_count',
        'invalidated_motor_command_count',
        'adc_error_count'
    )
    foreach ($column in $requiredColumns) {
        if ($null -eq $Rows[0].PSObject.Properties[$column]) {
            throw "遥测 CSV 缺少字段：$column"
        }
    }

    $derivedRows = [Collections.Generic.List[object]]::new()
    $samplePeriods = [Collections.Generic.List[double]]::new()
    $jitterSamples = [Collections.Generic.List[double]]::new()
    $wakeSamples = [Collections.Generic.List[double]]::new()
    $targetPwmValues = [Collections.Generic.List[double]]::new()
    $appliedPwmValues = [Collections.Generic.List[double]]::new()
    $batteryValues = [Collections.Generic.List[double]]::new()
    $countRates = @(
        [Collections.Generic.List[double]]::new(),
        [Collections.Generic.List[double]]::new(),
        [Collections.Generic.List[double]]::new(),
        [Collections.Generic.List[double]]::new()
    )
    [uint64]$durationMilliseconds = 0

    foreach ($row in $Rows) {
        $jitterSamples.Add([double]$row.irq_jitter_cycles)
        $wakeSamples.Add([double]$row.wake_latency_cycles)
        $targetPwmValues.Add([double]$row.target_pwm)
        $appliedPwmValues.Add([double]$row.applied_pwm)
        $batteryValues.Add([double]$row.battery_mv)
    }

    for ($index = 1; $index -lt $Rows.Count; $index++) {
        $previous = $Rows[$index - 1]
        $current = $Rows[$index]
        $boardDelta = Get-G2UInt32Delta `
            -Previous ([uint32]$previous.board_now_ms) `
            -Current ([uint32]$current.board_now_ms)
        $tickDelta = Get-G2UInt32Delta `
            -Previous ([uint32]$previous.control_tick_sequence) `
            -Current ([uint32]$current.control_tick_sequence)
        if ($boardDelta -eq 0) {
            throw "第 $index 行板端时间未推进"
        }
        $durationMilliseconds += $boardDelta
        $samplePeriods.Add([double]$boardDelta)

        $derived = [ordered]@{
            board_now_ms = [uint32]$current.board_now_ms
            board_delta_ms = $boardDelta
            control_tick_delta = $tickDelta
            target_pwm = [int32]$current.target_pwm
            applied_pwm = [int32]$current.applied_pwm
            battery_mv = [uint32]$current.battery_mv
        }
        for ($channel = 1; $channel -le 4; $channel++) {
            $propertyName = "encoder_total_$channel"
            $countDelta = [int64]$current.$propertyName - [int64]$previous.$propertyName
            $rate = ([double]$countDelta * 1000.0) / [double]$boardDelta
            $derived["encoder_count_delta_$channel"] = $countDelta
            $derived["encoder_count_rate_cps_$channel"] = $rate
            $countRates[$channel - 1].Add($rate)
        }
        $derivedRows.Add([pscustomobject]$derived)
    }

    $cpuCyclesPerControlPeriod = 168000.0
    $p99Jitter = Get-G2Percentile -Values $jitterSamples.ToArray() -Probability 0.99
    $p99Wake = Get-G2Percentile -Values $wakeSamples.ToArray() -Probability 0.99
    $maxReportedJitter = [double](($Rows | Measure-Object -Property irq_jitter_max_cycles -Maximum).Maximum)
    $maxReportedWcet = [double](($Rows | Measure-Object -Property wcet_max_cycles -Maximum).Maximum)
    $missedDelta = Get-G2UInt32Delta `
        -Previous ([uint32]$Rows[0].missed_period_count) `
        -Current ([uint32]$Rows[-1].missed_period_count)
    $deadlineDelta = Get-G2UInt32Delta `
        -Previous ([uint32]$Rows[0].deadline_miss_count) `
        -Current ([uint32]$Rows[-1].deadline_miss_count)

    $errorNames = @(
        'uart_error_count',
        'uart_rx_overflow_count',
        'uart_tx_fault_count',
        'command_reject_count',
        'command_queue_drop_count',
        'motion_gate_reject_count',
        'invalidated_motor_command_count',
        'adc_error_count'
    )
    $errorMaxima = [ordered]@{}
    foreach ($name in $errorNames) {
        $errorMaxima[$name] = [uint64](($Rows | Measure-Object -Property $name -Maximum).Maximum)
    }

    $encoderSummary = [ordered]@{}
    for ($channel = 1; $channel -le 4; $channel++) {
        $propertyName = "encoder_total_$channel"
        $rates = $countRates[$channel - 1].ToArray()
        $encoderSummary["channel_$channel"] = [ordered]@{
            total_count_change = [int64]$Rows[-1].$propertyName - [int64]$Rows[0].$propertyName
            minimum_count_rate_cps = [double](($rates | Measure-Object -Minimum).Minimum)
            maximum_count_rate_cps = [double](($rates | Measure-Object -Maximum).Maximum)
        }
    }

    $summary = [ordered]@{
        row_count = $Rows.Count
        derived_row_count = $derivedRows.Count
        board_duration_ms = $durationMilliseconds
        sample_period_ms = [ordered]@{
            minimum = [double](($samplePeriods | Measure-Object -Minimum).Minimum)
            average = [double](($samplePeriods | Measure-Object -Average).Average)
            maximum = [double](($samplePeriods | Measure-Object -Maximum).Maximum)
        }
        timing = [ordered]@{
            p99_irq_jitter_cycles_50hz_snapshot = $p99Jitter
            p99_irq_jitter_percent = ($p99Jitter / $cpuCyclesPerControlPeriod) * 100.0
            maximum_reported_irq_jitter_cycles = $maxReportedJitter
            maximum_reported_irq_jitter_percent = ($maxReportedJitter / $cpuCyclesPerControlPeriod) * 100.0
            p99_wake_latency_cycles_50hz_snapshot = $p99Wake
            maximum_reported_wcet_cycles = $maxReportedWcet
            maximum_reported_wcet_percent = ($maxReportedWcet / $cpuCyclesPerControlPeriod) * 100.0
            missed_period_delta = $missedDelta
            deadline_miss_delta = $deadlineDelta
            preliminary_thresholds = [ordered]@{
                zero_missed_periods = ($missedDelta -eq 0)
                p99_jitter_at_most_5_percent = ($p99Jitter -le 8400.0)
                maximum_jitter_at_most_10_percent = ($maxReportedJitter -le 16800.0)
                maximum_wcet_at_most_25_percent = ($maxReportedWcet -le 42000.0)
            }
            limitation = 'P99 uses 50 Hz telemetry snapshots; final acceptance requires a complete on-board distribution.'
        }
        pwm = [ordered]@{
            minimum_target = [int32](($targetPwmValues | Measure-Object -Minimum).Minimum)
            maximum_target = [int32](($targetPwmValues | Measure-Object -Maximum).Maximum)
            minimum_applied = [int32](($appliedPwmValues | Measure-Object -Minimum).Minimum)
            maximum_applied = [int32](($appliedPwmValues | Measure-Object -Maximum).Maximum)
        }
        battery_mv = [ordered]@{
            minimum = [uint32](($batteryValues | Measure-Object -Minimum).Minimum)
            maximum = [uint32](($batteryValues | Measure-Object -Maximum).Maximum)
        }
        encoder = $encoderSummary
        error_maxima = $errorMaxima
        g3_readiness = [ordered]@{
            rpm_reported = $false
            counts_per_wheel_revolution_verified = $true
            counts_per_wheel_revolution = 122880
            pulse_period_data_available = $false
            conclusion = 'Wheel counts/rev is verified. Keep count-rate units until pulse-period data and the M/T estimator are implemented.'
        }
    }

    return [pscustomobject]@{
        summary = [pscustomobject]$summary
        derived_rows = @($derivedRows)
    }
}
