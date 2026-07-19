. (Join-Path $PSScriptRoot 'g2_dynamic_step_lib.ps1')

function Get-G2SingleCaptureAnalysisProfile {
    param(
        [Parameter(Mandatory = $true)]
        [object]$Manifest,

        [Parameter(Mandatory = $true)]
        [object[]]$Schedule
    )

    $requiredAnalysisFields = @(
        'counts_per_wheel_revolution',
        'minimum_battery_mv',
        'motion_threshold_counts',
        'other_channel_limit_counts'
    )
    foreach ($field in $requiredAnalysisFields) {
        if ($Manifest.analysis.PSObject.Properties.Name -notcontains $field) {
            throw "单次分析清单缺少字段：analysis.$field"
        }
    }

    $experimentType = [string]$Manifest.experiment_type
    if ($experimentType -ceq 'g2_dynamic_bounded_step') {
        [void](Test-G2DynamicStepSchedule `
            -Schedule $Schedule `
            -Direction ([string]$Manifest.direction) `
            -PeakPwm ([int]$Manifest.peak_pwm))
        return [pscustomobject][ordered]@{
            experiment_type = $experimentType
            low_speed_steady_validation = $false
            minimum_full_pwm_plateau_ms = 50
        }
    }
    if ($experimentType -ceq 'g2_low_speed_steady_validation') {
        [void](Test-G2LowSpeedValidationSchedule -Schedule $Schedule)
        if ($Manifest.analysis.PSObject.Properties.Name -notcontains
            'minimum_full_pwm_plateau_ms') {
            throw '低速独立验证清单缺少字段：analysis.minimum_full_pwm_plateau_ms'
        }
        return [pscustomobject][ordered]@{
            experiment_type = $experimentType
            low_speed_steady_validation = $true
            minimum_full_pwm_plateau_ms =
                [int]$Manifest.analysis.minimum_full_pwm_plateau_ms
        }
    }
    throw "不支持的G2单次分析试验类型：$experimentType"
}

function New-G2LowSpeedValidationSchedule {
    param(
        [ValidateRange(1, 4294967289)]
        [uint32]$SequenceStart = 1
    )

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

    foreach ($sendAt in @(850, 1150, 1450, 1750, 2050)) {
        $schedule.Add([pscustomobject]@{
            elapsed_ms = [uint64]$sendAt
            command = "PWM $sequence -240"
        })
        $sequence++
    }
    $schedule.Add([pscustomobject]@{
        elapsed_ms = [uint64]2300
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

    [void](Test-G2LowSpeedValidationSchedule -Schedule $schedule)
    return @($schedule)
}

function Test-G2LowSpeedValidationSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Schedule
    )

    if ($Schedule.Count -ne 14) {
        throw '低速独立验证计划必须恰好包含14条命令'
    }
    $expectedFixedCommands = @{
        0 = 'STATUS'
        500 = 'CAPTURE STATUS'
        750 = 'CAPTURE START'
        2750 = 'CAPTURE STOP'
        3000 = 'CAPTURE STATUS'
        3250 = 'CAPTURE EXPORT'
        12000 = 'STATUS'
    }
    $motionTimes = @(800, 850, 1150, 1450, 1750, 2050, 2300)
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
            throw '低速独立验证计划时间必须严格递增'
        }
        if ($command -ceq 'ESTOP') {
            throw '低速独立验证计划禁止包含ESTOP'
        }
        if ($expectedFixedCommands.ContainsKey([int]$elapsed)) {
            if ($command -cne $expectedFixedCommands[[int]$elapsed]) {
                throw "低速独立验证固定命令不匹配：$elapsed ms"
            }
        } elseif ($motionTimes -notcontains [int]$elapsed) {
            throw "低速独立验证计划包含未知时刻：$elapsed ms"
        }

        if ($command -match '^(ARM|PWM|STOP) ([0-9]+)(?: (-?[0-9]+))?$') {
            [uint64]$sequence = 0
            if (-not [uint64]::TryParse($Matches[2], [ref]$sequence) -or
                $sequence -gt [uint64][uint32]::MaxValue) {
                throw "低速独立验证命令序号非法：$command"
            }
            if ($hasSequence -and $sequence -ne $previousSequence + 1) {
                throw '低速独立验证运动命令序号必须连续'
            }
            $previousSequence = $sequence
            $hasSequence = $true

            if ($Matches[1] -ceq 'ARM') {
                $armCount++
            } elseif ($Matches[1] -ceq 'STOP') {
                $stopCount++
            } else {
                $pwmCount++
                if ([int]$Matches[3] -ne -240) {
                    throw "低速独立验证只允许反向240：$command"
                }
            }
        } elseif (-not $expectedFixedCommands.ContainsKey([int]$elapsed)) {
            throw "低速独立验证计划包含未知命令：$command"
        }
        $previousElapsed = $elapsed
    }

    if ($armCount -ne 1 -or $pwmCount -ne 5 -or $stopCount -ne 1) {
        throw '低速独立验证必须包含一次ARM、五次PWM和一次STOP'
    }
    if ((2750 - 750) -gt 2200) {
        throw '低速独立验证超过2200样本高速缓冲预算'
    }
    if ((2750 - 2300) -lt (240 + 100)) {
        throw '低速独立验证停车后窗口不足'
    }
    return $true
}

function Measure-G2LowSpeedRun {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Samples,

        [Parameter(Mandatory = $true)]
        [string]$ExperimentId,

        [Parameter(Mandatory = $true)]
        [int]$FirstTargetIndex,

        [Parameter(Mandatory = $true)]
        [int]$PeakAppliedIndex,

        [Parameter(Mandatory = $true)]
        [int]$TargetStopIndex,

        [Parameter(Mandatory = $true)]
        [int]$MotionThresholdDelayMs,

        [double]$ReportedPeakWindowRpm = 0.0,

        [ValidateRange(1, 10000000)]
        [int]$CountsPerWheelRevolution = 122880
    )

    if ($Samples.Count -eq 0) {
        throw '低速诊断样本不能为空'
    }
    foreach ($column in @('encoder_delta_ma', 'applied_pwm')) {
        if ($Samples[0].PSObject.Properties.Name -notcontains $column) {
            throw "低速诊断样本缺少字段：$column"
        }
    }
    if ($FirstTargetIndex -lt 0 -or
        $PeakAppliedIndex -lt $FirstTargetIndex -or
        $TargetStopIndex -le $PeakAppliedIndex -or
        $TargetStopIndex -gt $Samples.Count) {
        throw '低速诊断索引范围非法'
    }

    $firstMoveIndex = -1
    for ($index = $FirstTargetIndex; $index -lt $TargetStopIndex; $index++) {
        if ([int]$Samples[$index].encoder_delta_ma -lt 0) {
            $firstMoveIndex = $index
            break
        }
    }
    if ($firstMoveIndex -lt 0) {
        throw '低速诊断平台内没有期望负向运动'
    }

    $measurement = [ordered]@{
        experiment_id = $ExperimentId
        direction = 'Negative'
        peak_pwm = 240
        first_move_delay_ms = $firstMoveIndex - $FirstTargetIndex
        motion_threshold_delay_ms = $MotionThresholdDelayMs
        full_pwm_plateau_ms = $TargetStopIndex - $PeakAppliedIndex
        reported_peak_window_rpm = $ReportedPeakWindowRpm
    }
    foreach ($windowMs in @(200, 300)) {
        $windowStart = $TargetStopIndex - $windowMs
        if ($windowStart -lt $PeakAppliedIndex) {
            throw "低速诊断平台不足以覆盖末端${windowMs}ms窗口"
        }
        [int64]$signedCounts = 0
        for ($index = $windowStart; $index -lt $TargetStopIndex; $index++) {
            if ([int]$Samples[$index].applied_pwm -ne -240) {
                throw "低速诊断末端${windowMs}ms窗口未保持实际PWM -240"
            }
            $signedCounts -= [int]$Samples[$index].encoder_delta_ma
        }
        if ($signedCounts -le 0) {
            throw "低速诊断末端${windowMs}ms窗口没有期望位移"
        }
        $countsPerSecond = [double]$signedCounts * 1000.0 / $windowMs
        $measurement["tail_${windowMs}ms_signed_counts"] = $signedCounts
        $measurement["tail_${windowMs}ms_wheel_rpm"] =
            $countsPerSecond * 60.0 / $CountsPerWheelRevolution
    }
    return [pscustomobject]$measurement
}

function Measure-G2LowSpeedDiagnosticBatch {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Rows,

        [ValidateSet('exploratory', 'independent_validation')]
        [string]$EvidenceRole = 'exploratory',

        [ValidateRange(3, 5)]
        [int]$RequiredRepeatCount = 3,

        [ValidateRange(0.1, 100.0)]
        [double]$StableSpeedCvLimitPercent = 5.0,

        [ValidateRange(0.1, 100.0)]
        [double]$StableSpeedDeviationLimitPercent = 10.0,

        [ValidateRange(0.1, 100.0)]
        [double]$StartupDelayCvLimitPercent = 10.0
    )

    if ($Rows.Count -eq 0) {
        throw '低速诊断批次不能为空'
    }
    $requiredColumns = @(
        'experiment_id', 'accepted', 'direction', 'peak_pwm',
        'first_move_delay_ms', 'motion_threshold_delay_ms',
        'tail_200ms_wheel_rpm', 'tail_300ms_wheel_rpm'
    )
    foreach ($column in $requiredColumns) {
        if ($Rows[0].PSObject.Properties.Name -notcontains $column) {
            throw "低速诊断批次缺少字段：$column"
        }
    }
    if (@($Rows | Where-Object {
        [string]$_.direction -cne 'Negative' -or [int]$_.peak_pwm -ne 240
    }).Count -ne 0) {
        throw '低速诊断批次只允许同工况反向240'
    }

    $tail200 = Get-G2DynamicSampleStatistics -Values @(
        $Rows | ForEach-Object { [double]$_.tail_200ms_wheel_rpm })
    $tail300 = Get-G2DynamicSampleStatistics -Values @(
        $Rows | ForEach-Object { [double]$_.tail_300ms_wheel_rpm })
    $firstMove = Get-G2DynamicSampleStatistics -Values @(
        $Rows | ForEach-Object { [double]$_.first_move_delay_ms })
    $thresholdDelay = Get-G2DynamicSampleStatistics -Values @(
        $Rows | ForEach-Object { [double]$_.motion_threshold_delay_ms })
    $allAccepted = -not @($Rows | Where-Object { -not [bool]$_.accepted }).Count
    $enoughRepeats = $Rows.Count -ge $RequiredRepeatCount
    $stableSpeedAccepted =
        $allAccepted -and
        $enoughRepeats -and
        [double]$tail200.coefficient_of_variation_percent -le
            $StableSpeedCvLimitPercent -and
        [double]$tail300.coefficient_of_variation_percent -le
            $StableSpeedCvLimitPercent -and
        [double]$tail200.maximum_single_deviation_percent -le
            $StableSpeedDeviationLimitPercent -and
        [double]$tail300.maximum_single_deviation_percent -le
            $StableSpeedDeviationLimitPercent
    $startupDelayAccepted =
        $allAccepted -and
        $enoughRepeats -and
        [double]$firstMove.coefficient_of_variation_percent -le
            $StartupDelayCvLimitPercent -and
        [double]$thresholdDelay.coefficient_of_variation_percent -le
            $StartupDelayCvLimitPercent
    $classification = if (-not $allAccepted -or -not $enoughRepeats) {
        'insufficient_or_rejected'
    } elseif (-not $stableSpeedAccepted) {
        'unstable_running_speed'
    } elseif ($startupDelayAccepted) {
        'stable_speed_and_startup'
    } else {
        'stable_speed_variable_startup'
    }
    $independentValidationAccepted =
        $EvidenceRole -ceq 'independent_validation' -and $stableSpeedAccepted

    return [pscustomobject][ordered]@{
        evidence_role = $EvidenceRole
        capture_count = $Rows.Count
        required_repeat_count = $RequiredRepeatCount
        all_captures_accepted = $allAccepted
        tail_200ms_wheel_rpm = $tail200
        tail_300ms_wheel_rpm = $tail300
        first_move_delay_ms = $firstMove
        motion_threshold_delay_ms = $thresholdDelay
        stable_speed_repeatability_accepted = $stableSpeedAccepted
        startup_delay_repeatability_accepted = $startupDelayAccepted
        classification = $classification
        independent_validation_accepted = $independentValidationAccepted
        original_short_window_gate_closed = $false
        model_ready = $false
        decision = if ($independentValidationAccepted) {
            '稳态速度独立验证通过；240仍作为启动/死区非线性验证点，不直接并入普通线性模型。'
        } elseif ($stableSpeedAccepted) {
            '探索性证据支持稳态速度可重复，但必须用预注册独立批次验证。'
        } else {
            '稳态速度重复性未通过，保持机械/驱动诊断阻断。'
        }
    }
}
