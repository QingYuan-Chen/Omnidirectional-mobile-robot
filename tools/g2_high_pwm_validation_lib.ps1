. (Join-Path $PSScriptRoot 'g2_low_speed_diagnostic_lib.ps1')

function New-G2HighPwmValidationSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [ValidateRange(1, 4294967290)]
        [uint32]$SequenceStart = 1
    )

    $signedPwm = if ($Direction -ceq 'Positive') { 840 } else { -840 }
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
        elapsed_ms = [uint64]1990
        command = "STOP $sequence"
    })
    foreach ($entry in @(
        [pscustomobject]@{ elapsed_ms = [uint64]2900; command = 'CAPTURE STOP' },
        [pscustomobject]@{ elapsed_ms = [uint64]3150; command = 'CAPTURE STATUS' },
        [pscustomobject]@{ elapsed_ms = [uint64]3400; command = 'CAPTURE EXPORT' },
        [pscustomobject]@{ elapsed_ms = [uint64]12000; command = 'STATUS' }
    )) {
        $schedule.Add($entry)
    }

    [void](Test-G2HighPwmValidationSchedule `
        -Schedule $schedule `
        -Direction $Direction)
    return @($schedule)
}

function Test-G2HighPwmValidationSchedule {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Schedule,

        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction
    )

    if ($Schedule.Count -ne 13) {
        throw '840稳态验证计划必须恰好包含13条命令'
    }
    $expectedPwm = if ($Direction -ceq 'Positive') { 840 } else { -840 }
    $expectedFixedCommands = @{
        0 = 'STATUS'
        500 = 'CAPTURE STATUS'
        750 = 'CAPTURE START'
        2900 = 'CAPTURE STOP'
        3150 = 'CAPTURE STATUS'
        3400 = 'CAPTURE EXPORT'
        12000 = 'STATUS'
    }
    $motionTimes = @(800, 850, 1150, 1450, 1750, 1990)
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
            throw '840稳态验证计划时间必须严格递增'
        }
        if ($command -ceq 'ESTOP') {
            throw '840稳态验证计划禁止包含ESTOP'
        }
        if ($expectedFixedCommands.ContainsKey([int]$elapsed)) {
            if ($command -cne $expectedFixedCommands[[int]$elapsed]) {
                throw "840稳态验证固定命令不匹配：$elapsed ms"
            }
        } elseif ($motionTimes -notcontains [int]$elapsed) {
            throw "840稳态验证计划包含未知时刻：$elapsed ms"
        }

        if ($command -match '^(ARM|PWM|STOP) ([0-9]+)(?: (-?[0-9]+))?$') {
            [uint64]$sequence = 0
            if (-not [uint64]::TryParse($Matches[2], [ref]$sequence) -or
                $sequence -gt [uint64][uint32]::MaxValue) {
                throw "840稳态验证命令序号非法：$command"
            }
            if ($hasSequence -and $sequence -ne $previousSequence + 1) {
                throw '840稳态验证运动命令序号必须连续'
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
                    throw "840稳态验证PWM方向或幅值不匹配：$command"
                }
            }
        } elseif (-not $expectedFixedCommands.ContainsKey([int]$elapsed)) {
            throw "840稳态验证计划包含未知命令：$command"
        }
        $previousElapsed = $elapsed
    }

    if ($armCount -ne 1 -or $pwmCount -ne 4 -or $stopCount -ne 1) {
        throw '840稳态验证必须包含一次ARM、四次PWM和一次STOP'
    }
    if ((2900 - 750) -gt 2200) {
        throw '840稳态验证超过2200样本高速缓冲预算'
    }
    if ((1990 - (850 + 839)) -lt 300) {
        throw '840稳态验证满PWM平台不足300 ms'
    }
    if ((2900 - (1990 + 839)) -lt 50) {
        throw '840稳态验证降零后记录余量不足50 ms'
    }
    if ((1990 - 1750) -ge 500) {
        throw '840稳态验证最后PWM与STOP间隔触发命令超时'
    }
    return $true
}

function Measure-G2HighPwmRun {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Samples,

        [Parameter(Mandatory = $true)]
        [string]$ExperimentId,

        [Parameter(Mandatory = $true)]
        [ValidateSet('Positive', 'Negative')]
        [string]$Direction,

        [Parameter(Mandatory = $true)]
        [int]$PeakAppliedIndex,

        [Parameter(Mandatory = $true)]
        [int]$TargetStopIndex,

        [ValidateRange(1, 10000000)]
        [int]$CountsPerWheelRevolution = 122880
    )

    if ($Samples.Count -eq 0) {
        throw '840稳态诊断样本不能为空'
    }
    foreach ($column in @('encoder_delta_ma', 'applied_pwm')) {
        if ($Samples[0].PSObject.Properties.Name -notcontains $column) {
            throw "840稳态诊断样本缺少字段：$column"
        }
    }
    if ($PeakAppliedIndex -lt 0 -or
        $TargetStopIndex -le $PeakAppliedIndex -or
        $TargetStopIndex -gt $Samples.Count) {
        throw '840稳态诊断索引范围非法'
    }

    $directionSign = if ($Direction -ceq 'Positive') { 1 } else { -1 }
    $expectedPwm = $directionSign * 840
    $measurement = [ordered]@{
        experiment_id = $ExperimentId
        direction = $Direction
        peak_pwm = 840
        full_pwm_plateau_ms = $TargetStopIndex - $PeakAppliedIndex
    }
    foreach ($windowMs in @(200, 300)) {
        $windowStart = $TargetStopIndex - $windowMs
        if ($windowStart -lt $PeakAppliedIndex) {
            throw "840稳态平台不足以覆盖末端${windowMs}ms窗口"
        }
        [int64]$signedCounts = 0
        for ($index = $windowStart; $index -lt $TargetStopIndex; $index++) {
            if ([int]$Samples[$index].applied_pwm -ne $expectedPwm) {
                throw "840稳态末端${windowMs}ms窗口未保持实际PWM $expectedPwm"
            }
            $signedCounts +=
                $directionSign * [int]$Samples[$index].encoder_delta_ma
        }
        if ($signedCounts -le 0) {
            throw "840稳态末端${windowMs}ms窗口没有期望位移"
        }
        $countsPerSecond = [double]$signedCounts * 1000.0 / $windowMs
        $measurement["tail_${windowMs}ms_signed_counts"] = $signedCounts
        $measurement["tail_${windowMs}ms_wheel_rpm"] =
            $countsPerSecond * 60.0 / $CountsPerWheelRevolution
    }
    return [pscustomobject]$measurement
}

function Measure-G2HighPwmValidationBatch {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Rows,

        [ValidateRange(3, 5)]
        [int]$RequiredRepeatCount = 3,

        [ValidateRange(0.1, 100.0)]
        [double]$StableSpeedCvLimitPercent = 5.0,

        [ValidateRange(0.1, 100.0)]
        [double]$StableSpeedDeviationLimitPercent = 10.0
    )

    if ($Rows.Count -eq 0) {
        throw '840稳态验证批次不能为空'
    }
    $requiredColumns = @(
        'experiment_id', 'accepted', 'direction', 'peak_pwm',
        'tail_200ms_wheel_rpm', 'tail_300ms_wheel_rpm'
    )
    foreach ($column in $requiredColumns) {
        if ($Rows[0].PSObject.Properties.Name -notcontains $column) {
            throw "840稳态验证批次缺少字段：$column"
        }
    }
    if (@($Rows | Where-Object {
        [string]$_.direction -cnotin @('Positive', 'Negative') -or
        [int]$_.peak_pwm -ne 840
    }).Count -ne 0) {
        throw '840稳态验证批次只允许正反向840'
    }

    $groups = [Collections.Generic.List[object]]::new()
    foreach ($direction in @('Positive', 'Negative')) {
        $groupRows = @($Rows | Where-Object {
            [string]$_.direction -ceq $direction
        })
        $tail200 = Get-G2DynamicSampleStatistics -Values @(
            $groupRows | ForEach-Object {
                [double]$_.tail_200ms_wheel_rpm
            })
        $tail300 = Get-G2DynamicSampleStatistics -Values @(
            $groupRows | ForEach-Object {
                [double]$_.tail_300ms_wheel_rpm
            })
        $allAccepted =
            -not @($groupRows | Where-Object { -not [bool]$_.accepted }).Count
        $enoughRepeats = $groupRows.Count -ge $RequiredRepeatCount
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
        $groups.Add([pscustomobject][ordered]@{
            direction = $direction
            peak_pwm = 840
            repeat_count = $groupRows.Count
            all_captures_accepted = $allAccepted
            enough_repeats = $enoughRepeats
            tail_200ms_wheel_rpm = $tail200
            tail_300ms_wheel_rpm = $tail300
            stable_speed_repeatability_accepted = $stableSpeedAccepted
        })
    }
    $captureEvidenceAccepted =
        -not @($Rows | Where-Object { -not [bool]$_.accepted }).Count
    $repeatabilityAccepted =
        -not @($groups | Where-Object {
            -not [bool]$_.stable_speed_repeatability_accepted
        }).Count

    return [pscustomobject][ordered]@{
        capture_count = $Rows.Count
        required_repeat_count = $RequiredRepeatCount
        capture_evidence_accepted = $captureEvidenceAccepted
        repeatability_accepted = $repeatabilityAccepted
        groups = $groups.ToArray()
        model_ready = $false
    }
}
