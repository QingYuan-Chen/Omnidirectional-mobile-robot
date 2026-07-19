function Get-MotorCapturePercentile {
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
    $index = [Math]::Ceiling($Probability * $sorted.Count) - 1
    if ($index -lt 0) {
        $index = 0
    }
    return [double]$sorted[$index]
}

function Get-MotorCaptureU32Delta {
    param(
        [Parameter(Mandatory = $true)]
        [uint32]$Current,
        [Parameter(Mandatory = $true)]
        [uint32]$Previous
    )

    if ($Current -ge $Previous) {
        return [uint64]($Current - $Previous)
    }
    return ([uint64][uint32]::MaxValue - [uint64]$Previous) +
        [uint64]$Current + 1
}

function Measure-MotorCaptureRows {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Rows,
        [uint32]$CpuClockHz = 168000000,
        [uint32]$ControlRateHz = 1000
    )

    if ($Rows.Count -lt 2) {
        throw '高速采集至少需要两行样本'
    }
    if ($CpuClockHz -eq 0 -or $ControlRateHz -eq 0 -or
        ($CpuClockHz % $ControlRateHz) -ne 0) {
        throw 'CPU频率必须能被控制频率整除'
    }

    [uint64]$expectedPeriod = [uint64]$CpuClockHz / [uint64]$ControlRateHz
    [int64]$relativeEncoder = 0
    [uint64]$tickGapCount = 0
    [uint64]$indexGapCount = 0
    $irqJitter = [Collections.Generic.List[double]]::new()
    $wakeLatency = [Collections.Generic.List[double]]::new()
    $wcet = [Collections.Generic.List[double]]::new()
    $derived = [Collections.Generic.List[object]]::new()

    for ($index = 0; $index -lt $Rows.Count; $index++) {
        $row = $Rows[$index]
        [uint32]$captureIndex = [uint32]$row.capture_index
        [uint32]$tick = [uint32]$row.control_tick_sequence
        [uint32]$irq = [uint32]$row.irq_timestamp_cycles
        [int16]$encoderDelta = [int16]$row.encoder_delta_ma
        [uint32]$wake = [uint32]$row.wake_latency_cycles
        [uint32]$previousWcet = [uint32]$row.previous_wcet_cycles

        if ($captureIndex -ne [uint32]$index) {
            $indexGapCount++
        }
        $relativeEncoder += [int64]$encoderDelta
        $wakeLatency.Add([double]$wake)
        $wcet.Add([double]$previousWcet)

        $irqPeriod = $null
        $jitter = $null
        if ($index -gt 0) {
            [uint32]$previousTick = [uint32]$Rows[$index - 1].control_tick_sequence
            [uint32]$previousIrq = [uint32]$Rows[$index - 1].irq_timestamp_cycles
            [uint64]$tickDelta = Get-MotorCaptureU32Delta -Current $tick -Previous $previousTick
            if ($tickDelta -ne 1) {
                $tickGapCount += [Math]::Max([uint64]1, $tickDelta - 1)
            }
            [uint64]$irqPeriodValue = Get-MotorCaptureU32Delta -Current $irq -Previous $previousIrq
            [uint64]$jitterValue = if ($irqPeriodValue -ge $expectedPeriod) {
                $irqPeriodValue - $expectedPeriod
            } else {
                $expectedPeriod - $irqPeriodValue
            }
            $irqPeriod = $irqPeriodValue
            $jitter = $jitterValue
            $irqJitter.Add([double]$jitterValue)
        }

        $derived.Add([pscustomobject][ordered]@{
            capture_index = $captureIndex
            control_tick_sequence = $tick
            irq_timestamp_cycles = $irq
            irq_period_cycles = $irqPeriod
            irq_jitter_cycles = $jitter
            wake_latency_cycles = $wake
            previous_wcet_cycles = $previousWcet
            encoder_raw_ma = [uint16]$row.encoder_raw_ma
            encoder_delta_ma = $encoderDelta
            encoder_relative_counts = $relativeEncoder
            target_pwm = [int16]$row.target_pwm
            applied_pwm = [int16]$row.applied_pwm
            battery_mv = [uint16]$row.battery_mv
            motor_state = [byte]$row.motor_state
            safety_flags = [byte]$row.safety_flags
        })
    }

    $p99IrqJitter = Get-MotorCapturePercentile -Values $irqJitter.ToArray() -Probability 0.99
    $p99Wake = Get-MotorCapturePercentile -Values $wakeLatency.ToArray() -Probability 0.99
    $maximumIrqJitter = [double](($irqJitter | Measure-Object -Maximum).Maximum)
    $maximumWake = [double](($wakeLatency | Measure-Object -Maximum).Maximum)
    $maximumWcet = [double](($wcet | Measure-Object -Maximum).Maximum)

    return [pscustomobject]@{
        Summary = [pscustomobject][ordered]@{
            sample_count = $Rows.Count
            expected_period_cycles = $expectedPeriod
            capture_index_gap_count = $indexGapCount
            control_tick_gap_count = $tickGapCount
            p99_irq_jitter_cycles = $p99IrqJitter
            maximum_irq_jitter_cycles = $maximumIrqJitter
            p99_wake_latency_cycles = $p99Wake
            maximum_wake_latency_cycles = $maximumWake
            maximum_previous_wcet_cycles = $maximumWcet
            p99_irq_jitter_percent = ($p99IrqJitter / $expectedPeriod) * 100.0
            p99_wake_latency_percent = ($p99Wake / $expectedPeriod) * 100.0
            maximum_wcet_percent = ($maximumWcet / $expectedPeriod) * 100.0
            encoder_relative_counts = $relativeEncoder
            first_tick_sequence = [uint32]$Rows[0].control_tick_sequence
            last_tick_sequence = [uint32]$Rows[-1].control_tick_sequence
        }
        DerivedRows = $derived.ToArray()
    }
}
