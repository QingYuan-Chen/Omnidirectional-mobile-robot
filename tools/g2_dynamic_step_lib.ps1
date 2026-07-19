function New-G2DynamicStepSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [ValidateSet(240, 400, 600, 840)]
        [int]$PeakPwm,

        [ValidateRange(1, 4294967290)]
        [uint32]$SequenceStart = 1
    )

    $signedPwm = if ($Direction -ceq 'Positive') { $PeakPwm } else { -$PeakPwm }
    [uint64]$sequence = $SequenceStart
    $schedule = [Collections.Generic.List[object]]::new()

    foreach ($entry in @(
        [pscustomobject]@{ elapsed_ms = [uint64]0; command = 'STATUS' },
        [pscustomobject]@{ elapsed_ms = [uint64]500; command = 'CAPTURE STATUS' },
        [pscustomobject]@{ elapsed_ms = [uint64]750; command = 'CAPTURE START' },
        [pscustomobject]@{ elapsed_ms = [uint64]800; command = "ARM $sequence" }
    )) {
        $schedule.Add($entry)
    }
    $sequence++

    foreach ($sendAt in @(850, 1150, 1450, 1750)) {
        $schedule.Add([pscustomobject]@{
            elapsed_ms = [uint64]$sendAt
            command = "PWM $sequence $signedPwm"
        })
        $sequence++
    }
    $schedule.Add([pscustomobject]@{
        elapsed_ms = [uint64]1800
        command = "STOP $sequence"
    })
    foreach ($entry in @(
        [pscustomobject]@{ elapsed_ms = [uint64]2750; command = 'CAPTURE STOP' },
        [pscustomobject]@{ elapsed_ms = [uint64]3000; command = 'CAPTURE STATUS' },
        [pscustomobject]@{ elapsed_ms = [uint64]3250; command = 'CAPTURE EXPORT' },
        [pscustomobject]@{ elapsed_ms = [uint64]12000; command = 'STATUS' }
    )) {
        $schedule.Add($entry)
    }

    [void](Test-G2DynamicStepSchedule `
        -Schedule $schedule `
        -Direction $Direction `
        -PeakPwm $PeakPwm)
    return @($schedule)
}

function Test-G2DynamicStepSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Schedule,

        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [ValidateSet(240, 400, 600, 840)]
        [int]$PeakPwm
    )

    if ($Schedule.Count -ne 13) {
        throw '动态阶跃计划必须恰好包含13条命令'
    }
    $expectedPwm = if ($Direction -ceq 'Positive') { $PeakPwm } else { -$PeakPwm }
    $expectedFixedCommands = @{
        0 = 'STATUS'
        500 = 'CAPTURE STATUS'
        750 = 'CAPTURE START'
        2750 = 'CAPTURE STOP'
        3000 = 'CAPTURE STATUS'
        3250 = 'CAPTURE EXPORT'
        12000 = 'STATUS'
    }
    $motionTimes = @(800, 850, 1150, 1450, 1750, 1800)
    [uint64]$previousElapsed = 0
    [uint64]$previousSequence = 0
    $hasSequence = $false
    $armCount = 0
    $pwmCount = 0
    $stopCount = 0

    for ($index = 0; $index -lt $Schedule.Count; $index++) {
        [uint64]$elapsed = $Schedule[$index].elapsed_ms
        $command = [string]$Schedule[$index].command
        if ($index -gt 0 -and $elapsed -le $previousElapsed) {
            throw '动态阶跃计划时间必须严格递增'
        }
        if ($command -ceq 'ESTOP') {
            throw '动态阶跃计划禁止包含ESTOP'
        }
        if ($expectedFixedCommands.ContainsKey([int]$elapsed)) {
            if ($command -cne $expectedFixedCommands[[int]$elapsed]) {
                throw "动态阶跃固定命令不匹配：$elapsed ms"
            }
        } elseif ($motionTimes -notcontains [int]$elapsed) {
            throw "动态阶跃计划包含未知时刻：$elapsed ms"
        }

        if ($command -match '^(ARM|PWM|STOP) ([0-9]+)(?: (-?[0-9]+))?$') {
            [uint64]$sequence = 0
            if (-not [uint64]::TryParse($Matches[2], [ref]$sequence) -or
                $sequence -gt [uint64][uint32]::MaxValue) {
                throw "动态阶跃命令序号非法：$command"
            }
            if ($hasSequence -and $sequence -ne $previousSequence + 1) {
                throw '动态阶跃运动命令序号必须连续'
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
                    throw "动态阶跃PWM方向或幅值错误：$command"
                }
            }
        } elseif (-not $expectedFixedCommands.ContainsKey([int]$elapsed)) {
            throw "动态阶跃计划包含未知命令：$command"
        }
        $previousElapsed = $elapsed
    }

    if ($armCount -ne 1 -or $pwmCount -ne 4 -or $stopCount -ne 1) {
        throw '动态阶跃计划必须包含一次ARM、四次PWM和一次STOP'
    }
    if ((2750 - 1800) -lt ($PeakPwm + 100)) {
        throw '采集窗口不足以覆盖安全斜坡降零和100 ms停车尾段'
    }
    return $true
}

function Measure-G2DynamicStep {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Samples,

        [Parameter(Mandatory = $true)]
        [object[]]$Telemetry,

        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [ValidateSet(240, 400, 600, 840)]
        [int]$PeakPwm,

        [ValidateRange(1, 1000000)]
        [int64]$MotionThresholdCounts = 1000,

        [ValidateRange(1, 1000000)]
        [int64]$OtherChannelLimitCounts = 1000,

        [ValidateRange(1, 65535)]
        [uint16]$MinimumBatteryMillivolts = 10500,

        [ValidateRange(1, 10000000)]
        [int64]$CountsPerWheelRevolution = 122880
    )

    if ($Samples.Count -lt 1000 -or $Telemetry.Count -lt 2) {
        throw '动态阶跃分析需要至少1000个高速样本和两行遥测'
    }
    $sampleColumns = @(
        'capture_index', 'encoder_delta_ma', 'target_pwm', 'applied_pwm',
        'battery_mv', 'motor_state'
    )
    $telemetryColumns = @(
        'encoder_total_1', 'encoder_total_2', 'encoder_total_3', 'encoder_total_4',
        'target_pwm', 'applied_pwm', 'battery_mv', 'critical_tasks_alive',
        'runtime_ready', 'motion_inhibited', 'fault_latched', 'motor_state',
        'estop_latched', 'missed_period_count', 'deadline_miss_count',
        'uart_error_count', 'uart_rx_overflow_count', 'uart_tx_fault_count',
        'command_reject_count', 'command_queue_drop_count',
        'motion_gate_reject_count', 'invalidated_motor_command_count',
        'adc_error_count'
    )
    foreach ($column in $sampleColumns) {
        if ($Samples[0].PSObject.Properties.Name -notcontains $column) {
            throw "高速样本缺少字段：$column"
        }
    }
    foreach ($column in $telemetryColumns) {
        if ($Telemetry[0].PSObject.Properties.Name -notcontains $column) {
            throw "遥测缺少字段：$column"
        }
    }

    $directionSign = if ($Direction -ceq 'Positive') { [int64]1 } else { [int64]-1 }
    $expectedPwm = [int32]($directionSign * $PeakPwm)
    $firstTargetIndex = -1
    $firstAppliedIndex = -1
    $peakAppliedIndex = -1
    $targetStopIndex = -1
    $appliedZeroIndex = -1

    for ($index = 0; $index -lt $Samples.Count; $index++) {
        [int32]$target = [int32]$Samples[$index].target_pwm
        [int32]$applied = [int32]$Samples[$index].applied_pwm
        if ($firstTargetIndex -lt 0 -and $target -eq $expectedPwm) {
            $firstTargetIndex = $index
        }
        if ($firstTargetIndex -ge 0 -and $firstAppliedIndex -lt 0 -and
            $applied -ne 0) {
            $firstAppliedIndex = $index
        }
        if ($peakAppliedIndex -lt 0 -and $applied -eq $expectedPwm) {
            $peakAppliedIndex = $index
        }
        if ($firstTargetIndex -ge 0 -and $index -gt $firstTargetIndex -and
            $targetStopIndex -lt 0 -and $target -eq 0) {
            $targetStopIndex = $index
        }
        if ($targetStopIndex -ge 0 -and $index -gt $targetStopIndex -and
            $appliedZeroIndex -lt 0 -and $applied -eq 0) {
            $appliedZeroIndex = $index
        }
    }
    if (@($firstTargetIndex, $firstAppliedIndex, $peakAppliedIndex,
          $targetStopIndex, $appliedZeroIndex) -contains -1) {
        throw '高速样本未形成完整的目标跳变、达到峰值和停车降零过程'
    }

    $targetValues = @(
        $Samples | ForEach-Object { [int32]$_.target_pwm } | Sort-Object -Unique)
    $appliedValues = @(
        $Samples | ForEach-Object { [int32]$_.applied_pwm } | Sort-Object -Unique)
    [int64]$relativeCounts = 0
    [int64]$minimumSignedCounts = 0
    [int64]$maximumSignedCounts = 0
    $motionThresholdIndex = -1
    for ($index = $firstTargetIndex; $index -lt $Samples.Count; $index++) {
        $relativeCounts += [int16]$Samples[$index].encoder_delta_ma
        $signedCounts = $directionSign * $relativeCounts
        $minimumSignedCounts = [Math]::Min($minimumSignedCounts, $signedCounts)
        $maximumSignedCounts = [Math]::Max($maximumSignedCounts, $signedCounts)
        if ($motionThresholdIndex -lt 0 -and
            $signedCounts -ge $MotionThresholdCounts) {
            $motionThresholdIndex = $index
        }
    }

    $peakRows = @(
        $Samples[$peakAppliedIndex..($targetStopIndex - 1)] |
            Where-Object {
                [int32]$_.target_pwm -eq $expectedPwm -and
                [int32]$_.applied_pwm -eq $expectedPwm
            })
    $signedPeakRate = $null
    $peakRpm = $null
    if ($peakRows.Count -gt 0) {
        $signedPeakRate = $directionSign * [double]((
            $peakRows | Measure-Object -Property encoder_delta_ma -Average).Average) * 1000.0
        $peakRpm = ($signedPeakRate * 60.0) / [double]$CountsPerWheelRevolution
    }

    $activeRows = @($Samples | Where-Object {
        [int32]$_.target_pwm -ne 0 -or [int32]$_.applied_pwm -ne 0
    })
    $activeBatteryMinimum = [uint16]((
        $activeRows | Measure-Object -Property battery_mv -Minimum).Minimum)
    $otherChannelRanges = [ordered]@{}
    $otherChannelsQuiet = $true
    foreach ($channel in 2..4) {
        $field = "encoder_total_$channel"
        $values = @($Telemetry | ForEach-Object { [int64]$_.$field })
        $measure = $values | Measure-Object -Minimum -Maximum
        [int64]$range = [int64]$measure.Maximum - [int64]$measure.Minimum
        $otherChannelRanges["channel_$channel"] = $range
        if ($range -gt $OtherChannelLimitCounts) {
            $otherChannelsQuiet = $false
        }
    }

    $errorFields = @(
        'missed_period_count', 'deadline_miss_count', 'uart_error_count',
        'uart_rx_overflow_count', 'uart_tx_fault_count', 'command_reject_count',
        'command_queue_drop_count', 'motion_gate_reject_count',
        'invalidated_motor_command_count', 'adc_error_count'
    )
    $errorMaxima = [ordered]@{}
    $absoluteErrorsZero = $true
    foreach ($field in $errorFields) {
        [uint64]$maximum =
            [uint64](($Telemetry | Measure-Object -Property $field -Maximum).Maximum)
        $errorMaxima[$field] = $maximum
        if ($maximum -ne 0) {
            $absoluteErrorsZero = $false
        }
    }

    $finalTelemetry = $Telemetry[-1]
    $finalStopped =
        [int32]$Samples[-1].target_pwm -eq 0 -and
        [int32]$Samples[-1].applied_pwm -eq 0 -and
        [byte]$Samples[-1].motor_state -eq 0 -and
        [int32]$finalTelemetry.target_pwm -eq 0 -and
        [int32]$finalTelemetry.applied_pwm -eq 0 -and
        [byte]$finalTelemetry.motor_state -eq 0 -and
        [byte]$finalTelemetry.critical_tasks_alive -eq 1 -and
        [byte]$finalTelemetry.runtime_ready -eq 1 -and
        [byte]$finalTelemetry.motion_inhibited -eq 0 -and
        [byte]$finalTelemetry.fault_latched -eq 0 -and
        [byte]$finalTelemetry.estop_latched -eq 0

    $rampUpMilliseconds = $peakAppliedIndex - $firstAppliedIndex
    $rampDownMilliseconds = $appliedZeroIndex - $targetStopIndex
    $expectedRampMilliseconds = $PeakPwm - 1
    $gates = [ordered]@{
        at_least_1000_samples = ($Samples.Count -ge 1000)
        at_least_50_baseline_samples = ($firstTargetIndex -ge 50)
        target_values_only_zero_and_expected =
            ($targetValues.Count -eq 2 -and
             $targetValues -contains 0 -and
             $targetValues -contains $expectedPwm)
        applied_values_keep_expected_sign =
            (($Direction -ceq 'Positive' -and ($appliedValues | Measure-Object -Minimum).Minimum -ge 0) -or
             ($Direction -ceq 'Negative' -and ($appliedValues | Measure-Object -Maximum).Maximum -le 0))
        applied_reaches_expected_peak = ($appliedValues -contains $expectedPwm)
        ramp_up_matches_1_count_per_ms =
            ([Math]::Abs($rampUpMilliseconds - $expectedRampMilliseconds) -le 2)
        ramp_down_matches_1_count_per_ms =
            ([Math]::Abs($rampDownMilliseconds - $expectedRampMilliseconds) -le 2)
        at_least_50_peak_samples = ($peakRows.Count -ge 50)
        expected_motion_threshold_reached = ($motionThresholdIndex -ge 0)
        wrong_direction_excursion_within_limit =
            ((-$minimumSignedCounts) -le $MotionThresholdCounts)
        other_channels_within_limit = $otherChannelsQuiet
        active_battery_above_minimum =
            ($activeBatteryMinimum -ge $MinimumBatteryMillivolts)
        final_state_stopped_and_ready = $finalStopped
        absolute_error_counters_zero = $absoluteErrorsZero
    }

    return [pscustomobject]@{
        Summary = [pscustomobject][ordered]@{
            direction = $Direction
            peak_pwm = $PeakPwm
            signed_peak_pwm = $expectedPwm
            sample_count = $Samples.Count
            baseline_sample_count = $firstTargetIndex
            first_target_index = $firstTargetIndex
            first_applied_index = $firstAppliedIndex
            peak_applied_index = $peakAppliedIndex
            target_stop_index = $targetStopIndex
            applied_zero_index = $appliedZeroIndex
            ramp_up_ms = $rampUpMilliseconds
            ramp_down_ms = $rampDownMilliseconds
            motion_threshold_counts = $MotionThresholdCounts
            motion_threshold_delay_ms =
                if ($motionThresholdIndex -ge 0) {
                    $motionThresholdIndex - $firstTargetIndex
                } else {
                    $null
                }
            signed_total_displacement_counts = $directionSign * $relativeCounts
            maximum_signed_displacement_counts = $maximumSignedCounts
            maximum_wrong_direction_excursion_counts = -$minimumSignedCounts
            peak_window_sample_count = $peakRows.Count
            peak_window_signed_count_rate_cps = $signedPeakRate
            peak_window_wheel_rpm = $peakRpm
            active_battery_minimum_mv = $activeBatteryMinimum
            other_channel_ranges = $otherChannelRanges
            error_counter_maxima = $errorMaxima
        }
        Gates = [pscustomobject]$gates
        Accepted = -not ($gates.Values -contains $false)
    }
}
