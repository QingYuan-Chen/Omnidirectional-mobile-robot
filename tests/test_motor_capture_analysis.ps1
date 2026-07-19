$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\tools\motor_capture_analysis_lib.ps1')

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

$rows = @(
    [pscustomobject]@{
        capture_index = 0
        control_tick_sequence = [uint32]::MaxValue
        irq_timestamp_cycles = 4294883295
        wake_latency_cycles = 100
        previous_wcet_cycles = 1000
        encoder_raw_ma = 10
        encoder_delta_ma = 2
        target_pwm = 0
        applied_pwm = 0
        battery_mv = 11600
        motor_state = 0
        safety_flags = 25
    },
    [pscustomobject]@{
        capture_index = 1
        control_tick_sequence = 0
        irq_timestamp_cycles = 83999
        wake_latency_cycles = 200
        previous_wcet_cycles = 2000
        encoder_raw_ma = 13
        encoder_delta_ma = 3
        target_pwm = 400
        applied_pwm = 1
        battery_mv = 11590
        motor_state = 2
        safety_flags = 25
    },
    [pscustomobject]@{
        capture_index = 2
        control_tick_sequence = 1
        irq_timestamp_cycles = 252009
        wake_latency_cycles = 300
        previous_wcet_cycles = 3000
        encoder_raw_ma = 17
        encoder_delta_ma = 4
        target_pwm = 400
        applied_pwm = 2
        battery_mv = 11590
        motor_state = 2
        safety_flags = 25
    }
)

$measurement = Measure-MotorCaptureRows -Rows $rows
Assert-True ($measurement.Summary.capture_index_gap_count -eq 0) '索引连续'
Assert-True ($measurement.Summary.control_tick_gap_count -eq 0) 'tick回绕连续'
Assert-True ($measurement.Summary.encoder_relative_counts -eq 9) '累计编码器增量'
Assert-True ($measurement.Summary.maximum_irq_jitter_cycles -eq 10) '计算IRQ最大抖动'
Assert-True ($measurement.Summary.p99_wake_latency_cycles -eq 300) '计算唤醒P99'
Assert-True ($measurement.Summary.maximum_previous_wcet_cycles -eq 3000) '计算WCET最大值'
Assert-True ($measurement.DerivedRows[1].irq_period_cycles -eq 168000) '处理DWT回绕'

$rows[2].capture_index = 4
$rows[2].control_tick_sequence = 3
$gapped = Measure-MotorCaptureRows -Rows $rows
Assert-True ($gapped.Summary.capture_index_gap_count -eq 1) '识别采集索引断裂'
Assert-True ($gapped.Summary.control_tick_gap_count -eq 2) '识别控制tick丢失'

$temporaryDirectory = Join-Path (
    [IO.Path]::GetTempPath()) ('motor-capture-plan-' + [Guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporaryDirectory)
try {
    $plan = & (Join-Path $PSScriptRoot '..\tools\new_motor_capture_timing_plan.ps1') `
        -FirmwareCommit 1234567 `
        -OutputRoot $temporaryDirectory
    $schedule = @(Import-Csv -LiteralPath $plan.SchedulePath)
    $manifest = Get-Content -LiteralPath $plan.ManifestPath -Raw | ConvertFrom-Json
    Assert-True ($schedule.Count -eq 7) '生成七条无动力时序计划命令'
    Assert-True (($schedule.command -contains 'CAPTURE START') -and
                 ($schedule.command -contains 'CAPTURE EXPORT')) '计划包含开始和停车后导出'
    Assert-True (-not ($schedule.command -match '^(ARM|PWM|STOP|ESTOP)').Count) '计划不含电机命令'
    Assert-True ($manifest.execution_state -ceq 'not_authorized' -and
                 -not $manifest.capture.motor_power_required) '计划默认未授权且禁止电机动力'

    $captureDirectory = Join-Path $temporaryDirectory 'synthetic_capture'
    $analysisDirectory = Join-Path $temporaryDirectory 'synthetic_analysis'
    [void][IO.Directory]::CreateDirectory($captureDirectory)
    $syntheticRows = for ($index = 0; $index -lt 1000; $index++) {
        [pscustomobject][ordered]@{
            capture_elapsed_ms = $index
            host_received_utc = '2026-07-19T00:00:00Z'
            capture_index = $index
            control_tick_sequence = $index
            irq_timestamp_cycles = [uint32]($index * 168000)
            wake_latency_cycles = 100
            previous_wcet_cycles = 1000
            encoder_raw_ma = $index
            encoder_delta_ma = 1
            target_pwm = 0
            applied_pwm = 0
            battery_mv = 11600
            motor_state = 0
            safety_flags = 25
        }
    }
    $syntheticRows |
        Export-Csv -LiteralPath (Join-Path $captureDirectory 'motor_capture.csv') `
            -NoTypeInformation -Encoding utf8
    @(
        [pscustomobject]@{
            capture_elapsed_ms = 0
            host_received_utc = '2026-07-19T00:00:00Z'
            event = 'BEGIN'
            state = 2
            sample_count = 1000
            capacity = 2200
            dropped_sample_count = 0
        },
        [pscustomobject]@{
            capture_elapsed_ms = 10000
            host_received_utc = '2026-07-19T00:00:10Z'
            event = 'END'
            state = 2
            sample_count = 1000
            capacity = 2200
            dropped_sample_count = 0
        }
    ) | Export-Csv -LiteralPath (
        Join-Path $captureDirectory 'motor_capture_events.csv') `
        -NoTypeInformation -Encoding utf8
    @(
        [pscustomobject]@{ missed_period_count = 0; deadline_miss_count = 0 },
        [pscustomobject]@{ missed_period_count = 0; deadline_miss_count = 0 }
    ) | Export-Csv -LiteralPath (Join-Path $captureDirectory 'telemetry.csv') `
        -NoTypeInformation -Encoding utf8
    [IO.File]::WriteAllText(
        (Join-Path $captureDirectory 'metadata.json'),
        ([ordered]@{
            outcome = 'completed'
            firmware_commit = '1234567'
            counts = [ordered]@{
                motor_capture_parse_errors = 0
                motor_capture_rows = 1000
                motor_capture_events = 2
            }
        } | ConvertTo-Json -Depth 4),
        [Text.UTF8Encoding]::new($false))

    $analysis = & (Join-Path $PSScriptRoot '..\tools\analyze_motor_capture.ps1') `
        -CaptureDirectory $captureDirectory `
        -OutputDirectory $analysisDirectory
    $analysisSummary =
        Get-Content -LiteralPath $analysis.SummaryPath -Raw | ConvertFrom-Json
    Assert-True ($analysis.Accepted -and $analysisSummary.accepted) '端到端分析接受完整合成采集'
    Assert-True ($analysisSummary.timing.sample_count -eq 1000 -and
                 $analysisSummary.timing.control_tick_gap_count -eq 0) '端到端分析保持样本连续'
} finally {
    if (Test-Path -LiteralPath $temporaryDirectory -PathType Container) {
        Remove-Item -LiteralPath $temporaryDirectory -Recurse -Force
    }
}

Write-Host "motor_capture_analysis：$assertionCount 项断言通过"
