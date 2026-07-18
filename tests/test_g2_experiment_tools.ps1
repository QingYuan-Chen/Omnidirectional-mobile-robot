$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\tools\g2_experiment_lib.ps1')

$assertionCount = 0

function Assert-True {
    param(
        [Parameter(Mandatory = $true)]
        [bool]$Condition,
        [Parameter(Mandatory = $true)]
        [string]$Message
    )
    $script:assertionCount++
    if (-not $Condition) {
        throw "断言失败：$Message"
    }
}

function Assert-Throws {
    param(
        [Parameter(Mandatory = $true)]
        [scriptblock]$Action,
        [Parameter(Mandatory = $true)]
        [string]$Message
    )
    $script:assertionCount++
    try {
        & $Action
    } catch {
        return
    }
    throw "断言失败，预期抛出异常：$Message"
}

$positive = @(New-G2FirstMotionSchedule `
    -Direction Positive `
    -PeakPwm 100 `
    -HoldMilliseconds 1000 `
    -KeepAliveMilliseconds 250 `
    -SequenceStart 10)
Assert-True ($positive[0].command -ceq 'STATUS') '计划以 STATUS 开始'
Assert-True ($positive[-1].command -ceq 'STATUS') '计划以 STATUS 结束'
Assert-True (@($positive | Where-Object { $_.command -ceq 'ARM 10' }).Count -eq 1) '只生成一次 ARM'
Assert-True (@($positive | Where-Object { $_.command -like 'PWM * 100' }).Count -eq 4) '按 250 ms 生成正向保活'
Assert-True (@($positive | Where-Object { $_.command -ceq 'STOP 15' }).Count -eq 1) '生成连续序号 STOP'
Assert-True (-not (@($positive | Where-Object { $_.command -ceq 'ESTOP' }).Count -gt 0)) '计划不含 ESTOP'

$negative = @(New-G2FirstMotionSchedule `
    -Direction Negative `
    -PeakPwm 80 `
    -HoldMilliseconds 500 `
    -KeepAliveMilliseconds 250)
Assert-True (@($negative | Where-Object { $_.command -like 'PWM * -80' }).Count -eq 2) '生成负向 PWM'
Assert-Throws {
    New-G2FirstMotionSchedule -Direction Positive -PeakPwm 201
} '首动软件上限为 200'
Assert-True ((Get-G2UInt32Delta -Previous 4294967294 -Current 1) -eq 3) 'uint32 回绕差值'

function New-TestTelemetryRow {
    param(
        [uint32]$Now,
        [uint32]$Tick,
        [int64]$Encoder1,
        [uint32]$Jitter,
        [uint32]$JitterMax,
        [uint32]$Wake,
        [uint32]$Wcet
    )
    return [pscustomobject]@{
        board_now_ms = $Now
        control_tick_sequence = $Tick
        target_pwm = 100
        applied_pwm = 100
        battery_mv = 12000
        irq_jitter_cycles = $Jitter
        irq_jitter_max_cycles = $JitterMax
        wake_latency_cycles = $Wake
        wcet_max_cycles = $Wcet
        missed_period_count = 0
        deadline_miss_count = 0
        encoder_total_1 = $Encoder1
        encoder_total_2 = 0
        encoder_total_3 = 0
        encoder_total_4 = 0
        uart_error_count = 0
        uart_rx_overflow_count = 0
        uart_tx_fault_count = 0
        command_reject_count = 0
        command_queue_drop_count = 0
        motion_gate_reject_count = 0
        invalidated_motor_command_count = 0
        adc_error_count = 0
    }
}

$rows = @(
    (New-TestTelemetryRow 1000 900 0 10 100 20 1000),
    (New-TestTelemetryRow 1020 920 10 20 100 30 1000),
    (New-TestTelemetryRow 1040 940 30 30 100 40 1000)
)
$measurement = Measure-G2CaptureRows -Rows $rows
Assert-True ($measurement.summary.board_duration_ms -eq 40) '累计板端时长'
Assert-True ($measurement.derived_rows.Count -eq 2) '派生行数量'
Assert-True ($measurement.summary.encoder.channel_1.total_count_change -eq 30) '编码器累计变化'
Assert-True ($measurement.summary.encoder.channel_1.maximum_count_rate_cps -eq 1000) '编码器最大计数率'
Assert-True ($measurement.summary.timing.preliminary_thresholds.zero_missed_periods) '零丢周期门槛'
Assert-True ((Test-G2PreliminaryTimingThresholds `
    -Thresholds $measurement.summary.timing.preliminary_thresholds)) '初步时序门槛汇总'
Assert-True (-not $measurement.summary.g3_readiness.rpm_reported) '未验证每圈计数前不报告 RPM'

$temporaryRoot = Join-Path ([IO.Path]::GetTempPath()) ('g2-tools-test-' + [Guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporaryRoot)
try {
    $generator = Join-Path $PSScriptRoot '..\tools\new_g2_first_motion_plan.ps1'
    $generated = & $generator `
        -ExperimentId test_first_motion `
        -FirmwareCommit 85ec922 `
        -Direction Positive `
        -PeakPwm 100 `
        -OutputRoot $temporaryRoot
    $manifest = Get-Content -LiteralPath $generated.Manifest -Raw | ConvertFrom-Json
    $scheduleRows = @(Import-Csv -LiteralPath $generated.PendingSchedule)
    Assert-True ($manifest.execution_state -ceq 'not_authorized') '生成计划默认未授权'
    Assert-True (-not $manifest.safety_checks.operator_execution_authorized) '人工执行确认默认为 false'
    Assert-True ($scheduleRows.Count -eq $positive.Count) '落盘计划行数'
} finally {
    if (Test-Path -LiteralPath $temporaryRoot -PathType Container) {
        Remove-Item -LiteralPath $temporaryRoot -Recurse -Force
    }
}

Write-Host "g2_experiment_tools：$assertionCount 项断言通过"
