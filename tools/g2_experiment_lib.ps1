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

function Measure-G2OperatingPoint {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Rows,

        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [ValidateRange(1, 840)]
        [int]$PeakPwm,

        [ValidateRange(1, 10000000)]
        [int64]$CountsPerWheelRevolution = 122880,

        [ValidateRange(0, 5000)]
        [int]$SteadySettleMilliseconds = 500,

        [ValidateRange(1, 1000000)]
        [int64]$MotionThresholdCounts = 1000,

        [ValidateRange(1, 1000000)]
        [int64]$OtherChannelLimitCounts = 1000
    )

    if ($Rows.Count -lt 3) {
        throw '工作点分析至少需要三行遥测'
    }
    $requiredColumns = @(
        'board_now_ms',
        'target_pwm',
        'applied_pwm',
        'battery_mv',
        'encoder_total_1',
        'encoder_total_2',
        'encoder_total_3',
        'encoder_total_4'
    )
    foreach ($column in $requiredColumns) {
        if ($null -eq $Rows[0].PSObject.Properties[$column]) {
            throw "遥测缺少字段：$column"
        }
    }

    $directionSign = if ($Direction -ceq 'Positive') { [int64]1 } else { [int64]-1 }
    $expectedPwm = [int32]($directionSign * $PeakPwm)
    $targetStartIndex = -1
    $steadyIndices = [Collections.Generic.List[int]]::new()
    for ($index = 0; $index -lt $Rows.Count; $index++) {
        if ($targetStartIndex -lt 0 -and [int32]$Rows[$index].target_pwm -eq $expectedPwm) {
            $targetStartIndex = $index
        }
        if ([int32]$Rows[$index].target_pwm -eq $expectedPwm -and
            [int32]$Rows[$index].applied_pwm -eq $expectedPwm) {
            $steadyIndices.Add($index)
        }
    }
    if ($targetStartIndex -lt 0) {
        throw "遥测中没有找到目标 PWM $expectedPwm"
    }
    if ($steadyIndices.Count -lt 2) {
        throw "PWM $expectedPwm 的稳态窗口不足两行"
    }

    $appliedPeakStartIndex = $steadyIndices[0]
    $appliedPeakStart = $Rows[$appliedPeakStartIndex]
    $steadyStartIndex = -1
    foreach ($index in $steadyIndices) {
        $elapsedAtPeak = Get-G2UInt32Delta `
            -Previous ([uint32]$appliedPeakStart.board_now_ms) `
            -Current ([uint32]$Rows[$index].board_now_ms)
        if ($elapsedAtPeak -ge [uint64]$SteadySettleMilliseconds) {
            $steadyStartIndex = $index
            break
        }
    }
    if ($steadyStartIndex -lt 0) {
        throw "PWM $expectedPwm 达到目标后没有足够的稳态等待窗口"
    }
    $steadyEndIndex = $steadyIndices[$steadyIndices.Count - 1]
    if ($steadyEndIndex -le $steadyStartIndex) {
        throw "PWM $expectedPwm 排除稳态等待时间后不足两行"
    }
    $steadyStart = $Rows[$steadyStartIndex]
    $steadyEnd = $Rows[$steadyEndIndex]
    $steadyDuration = Get-G2UInt32Delta `
        -Previous ([uint32]$steadyStart.board_now_ms) `
        -Current ([uint32]$steadyEnd.board_now_ms)
    if ($steadyDuration -eq 0) {
        throw '稳态窗口板端时长为零'
    }

    $steadySignedCountChange = $directionSign * (
        [int64]$steadyEnd.encoder_total_1 - [int64]$steadyStart.encoder_total_1)
    $steadySignedRate = ([double]$steadySignedCountChange * 1000.0) /
        [double]$steadyDuration
    $steadyRpm = ($steadySignedRate * 60.0) / [double]$CountsPerWheelRevolution
    $intervalRates = [Collections.Generic.List[double]]::new()
    for ($index = $steadyStartIndex + 1; $index -le $steadyEndIndex; $index++) {
        $previous = $Rows[$index - 1]
        $current = $Rows[$index]
        if ([int32]$previous.target_pwm -ne $expectedPwm -or
            [int32]$previous.applied_pwm -ne $expectedPwm -or
            [int32]$current.target_pwm -ne $expectedPwm -or
            [int32]$current.applied_pwm -ne $expectedPwm) {
            continue
        }
        $deltaMs = Get-G2UInt32Delta `
            -Previous ([uint32]$previous.board_now_ms) `
            -Current ([uint32]$current.board_now_ms)
        if ($deltaMs -eq 0) {
            continue
        }
        $signedDelta = $directionSign * (
            [int64]$current.encoder_total_1 - [int64]$previous.encoder_total_1)
        $intervalRates.Add(([double]$signedDelta * 1000.0) / [double]$deltaMs)
    }
    if ($intervalRates.Count -eq 0) {
        throw '稳态窗口没有可用的相邻采样计数率'
    }

    $targetStart = $Rows[$targetStartIndex]
    $timeToAppliedPeak = Get-G2UInt32Delta `
        -Previous ([uint32]$targetStart.board_now_ms) `
        -Current ([uint32]$appliedPeakStart.board_now_ms)
    $baselineIndex = [Math]::Max(0, $targetStartIndex - 1)
    $baselineCount = [int64]$Rows[$baselineIndex].encoder_total_1
    $motionStartDelay = $null
    for ($index = $targetStartIndex; $index -lt $Rows.Count; $index++) {
        $signedDisplacement = $directionSign * (
            [int64]$Rows[$index].encoder_total_1 - $baselineCount)
        if ($signedDisplacement -ge $MotionThresholdCounts) {
            $motionStartDelay = Get-G2UInt32Delta `
                -Previous ([uint32]$targetStart.board_now_ms) `
                -Current ([uint32]$Rows[$index].board_now_ms)
            break
        }
    }

    $baselineRows = @($Rows[0..$baselineIndex])
    $activeRows = @($Rows | Where-Object {
        [int32]$_.target_pwm -ne 0 -or [int32]$_.applied_pwm -ne 0
    })
    $baselineBatteryAverage = [double]((
        $baselineRows | Measure-Object -Property battery_mv -Average).Average)
    $activeBatteryMinimum = [uint32]((
        $activeRows | Measure-Object -Property battery_mv -Minimum).Minimum)
    $activeBatteryAverage = [double]((
        $activeRows | Measure-Object -Property battery_mv -Average).Average)
    $motion = Measure-G2DeadzoneMotion `
        -Rows $Rows `
        -Direction $Direction `
        -MotionThresholdCounts $MotionThresholdCounts `
        -OtherChannelLimitCounts $OtherChannelLimitCounts

    return [pscustomobject][ordered]@{
        direction = $Direction
        peak_pwm = $PeakPwm
        signed_peak_pwm = $expectedPwm
        counts_per_wheel_revolution = $CountsPerWheelRevolution
        steady_window = [pscustomobject][ordered]@{
            settling_excluded_ms = $SteadySettleMilliseconds
            row_count = $steadyEndIndex - $steadyStartIndex + 1
            duration_ms = $steadyDuration
            signed_count_change = $steadySignedCountChange
            signed_count_rate_cps = $steadySignedRate
            wheel_rpm = $steadyRpm
            interval_count_rate_cps = [pscustomobject][ordered]@{
                minimum = [double](($intervalRates | Measure-Object -Minimum).Minimum)
                median = Get-G2Percentile -Values $intervalRates.ToArray() -Probability 0.5
                maximum = [double](($intervalRates | Measure-Object -Maximum).Maximum)
            }
        }
        response = [pscustomobject][ordered]@{
            time_to_applied_peak_ms = $timeToAppliedPeak
            motion_threshold_counts = $MotionThresholdCounts
            motion_start_delay_ms = $motionStartDelay
        }
        battery_mv = [pscustomobject][ordered]@{
            baseline_average = $baselineBatteryAverage
            active_average = $activeBatteryAverage
            active_minimum = $activeBatteryMinimum
            baseline_to_active_minimum_drop = $baselineBatteryAverage - $activeBatteryMinimum
        }
        motion = $motion
    }
}

function Measure-G2Repeatability {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Rows,

        [ValidateRange(2, 20)]
        [int]$RequiredRepeatCount = 3,

        [ValidateRange(0.1, 100.0)]
        [double]$PassCvPercent = 5.0,

        [ValidateRange(0.1, 100.0)]
        [double]$RetestCvPercent = 10.0,

        [ValidateRange(0.1, 100.0)]
        [double]$MaximumSingleDeviationPercent = 10.0
    )

    if ($Rows.Count -lt 2) {
        throw '重复性汇总至少需要两条工作点结果'
    }
    if ($RetestCvPercent -lt $PassCvPercent) {
        throw '复验 CV 门槛不能低于通过门槛'
    }
    $requiredColumns = @(
        'direction',
        'peak_pwm',
        'steady_count_rate_cps',
        'wheel_rpm',
        'battery_minimum_mv',
        'accepted'
    )
    foreach ($column in $requiredColumns) {
        if ($null -eq $Rows[0].PSObject.Properties[$column]) {
            throw "重复性输入缺少字段：$column"
        }
    }

    $groups = [Collections.Generic.List[object]]::new()
    $groupedRows = @($Rows |
        Group-Object -Property direction, peak_pwm |
        Sort-Object @{
            Expression = {
                if ($_.Group[0].direction -ceq 'Positive') { 0 } else { 1 }
            }
        }, @{
            Expression = { [int]$_.Group[0].peak_pwm }
        })
    foreach ($group in $groupedRows) {
        $samples = @($group.Group)
        $rpmValues = @($samples | ForEach-Object { [double]$_.wheel_rpm })
        $rateValues = @($samples | ForEach-Object {
            [double]$_.steady_count_rate_cps
        })
        $rpmMean = [double](($rpmValues | Measure-Object -Average).Average)
        $rateMean = [double](($rateValues | Measure-Object -Average).Average)
        $sumSquared = 0.0
        $maximumDeviation = 0.0
        foreach ($value in $rpmValues) {
            $difference = $value - $rpmMean
            $sumSquared += $difference * $difference
            if ([Math]::Abs($rpmMean) -gt 0.0) {
                $deviation = ([Math]::Abs($difference) /
                    [Math]::Abs($rpmMean)) * 100.0
                if ($deviation -gt $maximumDeviation) {
                    $maximumDeviation = $deviation
                }
            }
        }
        $standardDeviation = if ($samples.Count -gt 1) {
            [Math]::Sqrt($sumSquared / [double]($samples.Count - 1))
        } else {
            0.0
        }
        $cvPercent = if ([Math]::Abs($rpmMean) -gt 0.0) {
            ($standardDeviation / [Math]::Abs($rpmMean)) * 100.0
        } else {
            [double]::PositiveInfinity
        }
        $allAccepted = @($samples | Where-Object {
            -not [bool]$_.accepted
        }).Count -eq 0
        $screening = if (-not $allAccepted) {
            'blocked'
        } elseif ($samples.Count -lt $RequiredRepeatCount) {
            'incomplete'
        } elseif ($cvPercent -gt $RetestCvPercent) {
            'blocked'
        } elseif ($cvPercent -gt $PassCvPercent -or
            $maximumDeviation -gt $MaximumSingleDeviationPercent) {
            'retest'
        } else {
            'passed'
        }

        $groups.Add([pscustomobject][ordered]@{
            direction = [string]$samples[0].direction
            peak_pwm = [int]$samples[0].peak_pwm
            repeat_count = $samples.Count
            all_captures_accepted = $allAccepted
            mean_steady_count_rate_cps = $rateMean
            mean_wheel_rpm = $rpmMean
            sample_standard_deviation_rpm = $standardDeviation
            coefficient_of_variation_percent = $cvPercent
            minimum_wheel_rpm = [double]((
                $rpmValues | Measure-Object -Minimum).Minimum)
            maximum_wheel_rpm = [double]((
                $rpmValues | Measure-Object -Maximum).Maximum)
            maximum_single_deviation_percent = $maximumDeviation
            minimum_battery_mv = [uint32]((
                $samples | Measure-Object -Property battery_minimum_mv -Minimum
            ).Minimum)
            screening = $screening
        })
    }

    $monotonicByDirection = [ordered]@{}
    foreach ($direction in @('Positive', 'Negative')) {
        $directionGroups = @($groups |
            Where-Object { $_.direction -ceq $direction } |
            Sort-Object peak_pwm)
        $monotonic = $directionGroups.Count -gt 1
        for ($index = 1; $index -lt $directionGroups.Count; $index++) {
            if ($directionGroups[$index].mean_wheel_rpm -le
                $directionGroups[$index - 1].mean_wheel_rpm) {
                $monotonic = $false
            }
        }
        $monotonicByDirection[$direction] = $monotonic
    }
    $allGroupsPassed = @($groups | Where-Object {
        $_.screening -cne 'passed'
    }).Count -eq 0
    $allMonotonic = @($monotonicByDirection.Values | Where-Object {
        -not [bool]$_
    }).Count -eq 0

    return [pscustomobject][ordered]@{
        required_repeat_count = $RequiredRepeatCount
        thresholds = [pscustomobject][ordered]@{
            pass_cv_percent = $PassCvPercent
            retest_cv_percent = $RetestCvPercent
            maximum_single_deviation_percent =
                $MaximumSingleDeviationPercent
        }
        groups = @($groups)
        monotonic_by_direction = [pscustomobject]$monotonicByDirection
        all_groups_passed = $allGroupsPassed
        all_directions_monotonic = $allMonotonic
        accepted = ($allGroupsPassed -and $allMonotonic)
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
