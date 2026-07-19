function Get-G3SpeedU32Delta {
    param(
        [Parameter(Mandatory = $true)]
        [uint64]$Current,
        [Parameter(Mandatory = $true)]
        [uint64]$Previous
    )

    return [uint64](($Current - $Previous) -band [uint64][uint32]::MaxValue)
}

function Get-G3SpeedStatistics {
    param(
        [Parameter(Mandatory = $true)]
        [AllowEmptyCollection()]
        [double[]]$Values
    )

    if ($Values.Count -eq 0) {
        return [pscustomobject][ordered]@{
            count = 0
            mean = $null
            standard_deviation = $null
            cv_percent = $null
        }
    }
    $mean = [double](($Values | Measure-Object -Average).Average)
    $sumSquares = 0.0
    foreach ($value in $Values) {
        $difference = $value - $mean
        $sumSquares += $difference * $difference
    }
    $standardDeviation = if ($Values.Count -gt 1) {
        [Math]::Sqrt($sumSquares / ($Values.Count - 1))
    } else {
        0.0
    }
    return [pscustomobject][ordered]@{
        count = $Values.Count
        mean = $mean
        standard_deviation = $standardDeviation
        cv_percent = if ([Math]::Abs($mean) -gt 0.0) {
            100.0 * $standardDeviation / [Math]::Abs($mean)
        } else {
            $null
        }
    }
}

function Measure-G3SpeedCaptureRows {
    param(
        [Parameter(Mandatory = $true)]
        [AllowEmptyCollection()]
        [object[]]$Rows,
        [uint32]$EncoderCountsPerWheelRevolution = 122880,
        [uint32]$PeriodEventsPerWheelRevolution = 30720,
        [uint32]$TimestampClockHz = 168000000,
        [uint32]$ControlRateHz = 1000,
        [ValidateRange(1, 50)]
        [uint32]$MWindowTicks = 10
    )

    if ($Rows.Count -eq 0) {
        throw 'G3 轮速分析至少需要一个样本'
    }
    if ($EncoderCountsPerWheelRevolution -eq 0 -or
        $PeriodEventsPerWheelRevolution -eq 0 -or
        $TimestampClockHz -eq 0 -or
        $ControlRateHz -eq 0) {
        throw 'G3 轮速换算参数必须非零'
    }

    [uint64]$indexBreaks = 0
    [uint64]$tickBreaks = 0
    [uint64]$nonzeroPwmRows = 0
    [int64]$encoderCountSum = 0
    [uint64]$periodCountSum = 0
    [uint64]$aggregateDropFlagRows = 0
    [uint64]$zeroPeriodFlagRows = 0
    [uint64]$directionResetFlagRows = 0
    $mValues = [Collections.Generic.List[double]]::new()
    $tValues = [Collections.Generic.List[double]]::new()
    $window = [Collections.Generic.Queue[int]]::new()
    [int64]$windowSum = 0

    for ($index = 0; $index -lt $Rows.Count; $index++) {
        $row = $Rows[$index]
        if ([int64]$row.applied_pwm -ne 0) {
            $nonzeroPwmRows++
        }
        $delta = [int]$row.encoder_delta_ma
        $encoderCountSum += $delta
        $periodCountSum += [uint64]$row.period_count
        $flags = [uint32]$row.period_flags
        if (($flags -band 8) -ne 0) {
            $aggregateDropFlagRows++
        }
        if (($flags -band 16) -ne 0) {
            $zeroPeriodFlagRows++
        }
        if (($flags -band 4) -ne 0) {
            $directionResetFlagRows++
        }
        if ($index -gt 0) {
            if ((Get-G3SpeedU32Delta `
                    -Current ([uint64]$row.capture_index) `
                    -Previous ([uint64]$Rows[$index - 1].capture_index)) -ne 1) {
                $indexBreaks++
            }
            if ((Get-G3SpeedU32Delta `
                    -Current ([uint64]$row.control_tick_sequence) `
                    -Previous ([uint64]$Rows[$index - 1].control_tick_sequence)) -ne 1) {
                $tickBreaks++
            }
        }

        $window.Enqueue($delta)
        $windowSum += $delta
        if ($window.Count -gt $MWindowTicks) {
            $windowSum -= $window.Dequeue()
        }
        if ($window.Count -eq $MWindowTicks) {
            $mValues.Add(
                60.0 * $ControlRateHz * $windowSum /
                ($EncoderCountsPerWheelRevolution * $MWindowTicks))
        }

        $periodCount = [uint64]$row.period_count
        $periodSum = [uint64]$row.period_sum_cycles
        $direction = [int]$row.direction
        if ($periodCount -gt 0 -and $periodSum -gt 0 -and
            ($direction -eq -1 -or $direction -eq 1)) {
            $tValues.Add(
                $direction * 60.0 * $TimestampClockHz * $periodCount /
                ($PeriodEventsPerWheelRevolution * $periodSum))
        }
    }

    $firstEvent = [uint64]$Rows[0].event_sequence
    $lastEvent = [uint64]$Rows[$Rows.Count - 1].event_sequence
    $eventDelta = Get-G3SpeedU32Delta -Current $lastEvent -Previous $firstEvent
    return [pscustomobject][ordered]@{
        sample_count = $Rows.Count
        index_break_count = $indexBreaks
        tick_break_count = $tickBreaks
        nonzero_pwm_row_count = $nonzeroPwmRows
        encoder_count_sum = $encoderCountSum
        event_sequence_delta = $eventDelta
        period_count_sum = $periodCountSum
        encoder_counts_per_observed_event = if ($eventDelta -gt 0) {
            [Math]::Abs([double]$encoderCountSum) / $eventDelta
        } else {
            $null
        }
        aggregate_drop_flag_row_count = $aggregateDropFlagRows
        zero_period_flag_row_count = $zeroPeriodFlagRows
        direction_reset_flag_row_count = $directionResetFlagRows
        m_speed_rpm = Get-G3SpeedStatistics -Values @($mValues)
        t_speed_rpm = Get-G3SpeedStatistics -Values @($tValues)
        model_ready = $false
    }
}
