$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\tools\g2_dynamic_step_lib.ps1')

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
    $threw = $false
    try {
        & $Action
    } catch {
        $threw = $true
    }
    Assert-True $threw $Message
}

$negativeSchedule = @(New-G2DynamicStepSchedule `
    -Direction Negative `
    -PeakPwm 840 `
    -SequenceStart 100)
Assert-True ($negativeSchedule.Count -eq 13) '固定生成13条动态计划命令'
Assert-True ($negativeSchedule[2].command -ceq 'CAPTURE START' -and
             $negativeSchedule[-2].command -ceq 'CAPTURE EXPORT') `
    '计划包含停车前记录与停车后导出'
Assert-True (@($negativeSchedule | Where-Object {
    $_.command -match '^PWM [0-9]+ -840$'
}).Count -eq 4) '负向840使用四条保活PWM'
Assert-True (-not @($negativeSchedule | Where-Object {
    $_.command -ceq 'ESTOP'
}).Count) '动态计划不含ESTOP'

$brokenSchedule = @($negativeSchedule | ForEach-Object {
    [pscustomobject]@{
        elapsed_ms = $_.elapsed_ms
        command = $_.command
    }
})
$brokenSchedule[5].command = 'PWM 102 840'
Assert-Throws {
    Test-G2DynamicStepSchedule `
        -Schedule $brokenSchedule `
        -Direction Negative `
        -PeakPwm 840
} '拒绝方向错误的动态PWM'

$temporaryDirectory = Join-Path (
    [IO.Path]::GetTempPath()) ('g2-dynamic-step-' + [Guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporaryDirectory)
try {
    $planRoot = Join-Path $temporaryDirectory 'plans'
    $plan = & (Join-Path $PSScriptRoot '..\tools\new_g2_dynamic_step_plan.ps1') `
        -ExperimentId synthetic_negative_240 `
        -FirmwareCommit 1234567 `
        -Direction Negative `
        -PeakPwm 240 `
        -SequenceStart 100 `
        -OutputRoot $planRoot
    $manifest =
        Get-Content -LiteralPath $plan.ManifestPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    $plannedCommands = @(Import-Csv -LiteralPath $plan.SchedulePath)
    Assert-True ($manifest.execution_state -ceq 'not_authorized' -and
                 $manifest.safety.motor_power_required) `
        '生成计划默认未授权且明确需要动力'
    Assert-True (-not $manifest.safety.reversal_present -and
                 -not $manifest.safety.estop_present) `
        '单次动态计划不换向且不含ESTOP'
    Assert-True ($manifest.capture_duration_seconds -eq 14) `
        '采集时长覆盖停车后完整导出'

    $batch = & (Join-Path $PSScriptRoot '..\tools\new_g2_dynamic_step_batch.ps1') `
        -BatchId synthetic_midrange_batch `
        -FirmwareCommit 1234567 `
        -PeakPwms @(400) `
        -Repetitions 3 `
        -SequenceStart 1000 `
        -OutputRoot $planRoot
    $batchRows = @(Import-Csv -LiteralPath $batch.CsvPath)
    $batchManifest =
        Get-Content -LiteralPath $batch.JsonPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    Assert-True ($batchRows.Count -eq 6 -and $batch.ExperimentCount -eq 6) `
        '中档双向三轮批次生成六个独立计划'
    Assert-True ($batchRows[0].direction -ceq 'Positive' -and
                 $batchRows[1].direction -ceq 'Negative' -and
                 $batchRows[2].direction -ceq 'Negative' -and
                 $batchRows[3].direction -ceq 'Positive') `
        '批次按轮次交替方向先后顺序'
    Assert-True ([uint32]$batchRows[0].sequence_start -eq 1000 -and
                 [uint32]$batchRows[-1].sequence_start -eq 1030 -and
                 $batch.NextUnusedSequence -eq 1036) `
        '批次为每次试验分配六个连续且互不重叠的序号'
    Assert-True ($batchManifest.safety.abort_batch_on_any_rejected_capture -and
                 -not $batchManifest.analysis.model_ready_after_batch) `
        '批次遇到拒绝立即停止且不宣称模型就绪'

    $captureDirectory = Join-Path $temporaryDirectory 'capture'
    [void][IO.Directory]::CreateDirectory($captureDirectory)
    $sampleCount = 2000
    $cumulative = [int64[]]::new($sampleCount)
    [int64]$totalCounts = 0
    $samples = for ($index = 0; $index -lt $sampleCount; $index++) {
        $target = if ($index -ge 100 -and $index -lt 1050) { -240 } else { 0 }
        $applied = 0
        if ($index -ge 100 -and $index -le 339) {
            $applied = -($index - 99)
        } elseif ($index -gt 339 -and $index -lt 1050) {
            $applied = -240
        } elseif ($index -ge 1050 -and $index -le 1289) {
            $applied = -240 + ($index - 1049)
        }
        $delta = if ($index -ge 180 -and $index -lt 1290) {
            -10
        } elseif ($index -ge 1290 -and $index -lt 1350) {
            -3
        } else {
            0
        }
        $totalCounts += $delta
        $cumulative[$index] = $totalCounts
        $state = if ($index -lt 50) {
            0
        } elseif ($index -lt 100) {
            1
        } elseif ($index -lt 1050) {
            2
        } elseif ($index -le 1289) {
            3
        } elseif ($index -lt 1550) {
            1
        } else {
            0
        }
        [pscustomobject][ordered]@{
            capture_elapsed_ms = $index
            host_received_utc = '2026-07-19T00:00:00Z'
            capture_index = $index
            control_tick_sequence = $index
            irq_timestamp_cycles = [uint32]($index * 168000)
            wake_latency_cycles = 100
            previous_wcet_cycles = 1000
            encoder_raw_ma = [uint16]($totalCounts -band 0xFFFF)
            encoder_delta_ma = $delta
            target_pwm = $target
            applied_pwm = $applied
            battery_mv = if ($target -ne 0 -or $applied -ne 0) { 11550 } else { 11600 }
            motor_state = $state
            safety_flags = 25
        }
    }
    $samples |
        Export-Csv -LiteralPath (Join-Path $captureDirectory 'motor_capture.csv') `
        -NoTypeInformation -Encoding utf8

    @(
        [pscustomobject]@{
            capture_elapsed_ms = 0
            host_received_utc = '2026-07-19T00:00:00Z'
            event = 'BEGIN'
            state = 2
            sample_count = $sampleCount
            capacity = 2200
            dropped_sample_count = 0
        },
        [pscustomobject]@{
            capture_elapsed_ms = 10000
            host_received_utc = '2026-07-19T00:00:10Z'
            event = 'END'
            state = 2
            sample_count = $sampleCount
            capacity = 2200
            dropped_sample_count = 0
        }
    ) | Export-Csv -LiteralPath (
        Join-Path $captureDirectory 'motor_capture_events.csv') `
        -NoTypeInformation -Encoding utf8

    $telemetry = for ($rowIndex = 0; $rowIndex -lt 100; $rowIndex++) {
        $sampleIndex = [Math]::Min($sampleCount - 1, $rowIndex * 20)
        $sample = $samples[$sampleIndex]
        [pscustomobject][ordered]@{
            board_now_ms = $rowIndex * 20
            encoder_total_1 = $cumulative[$sampleIndex]
            encoder_total_2 = 0
            encoder_total_3 = 0
            encoder_total_4 = 0
            target_pwm = $sample.target_pwm
            applied_pwm = $sample.applied_pwm
            battery_mv = $sample.battery_mv
            critical_tasks_alive = 1
            runtime_ready = 1
            motion_inhibited = 0
            fault_latched = 0
            motor_state = $sample.motor_state
            estop_latched = 0
            missed_period_count = 0
            deadline_miss_count = 0
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
    $telemetry |
        Export-Csv -LiteralPath (Join-Path $captureDirectory 'telemetry.csv') `
        -NoTypeInformation -Encoding utf8

    $actualCommands = for ($index = 0; $index -lt $plannedCommands.Count; $index++) {
        [pscustomobject][ordered]@{
            planned_elapsed_ms = $plannedCommands[$index].elapsed_ms
            actual_elapsed_ms = [uint64]$plannedCommands[$index].elapsed_ms + 5
            host_sent_utc = '2026-07-19T00:00:00Z'
            command = $plannedCommands[$index].command
            result = 'sent'
        }
    }
    $actualCommands |
        Export-Csv -LiteralPath (Join-Path $captureDirectory 'commands.csv') `
        -NoTypeInformation -Encoding utf8

    [IO.File]::WriteAllText(
        (Join-Path $captureDirectory 'metadata.json'),
        ([ordered]@{
            outcome = 'completed'
            firmware_commit = '1234567'
            counts = [ordered]@{
                motor_capture_parse_errors = 0
                motor_capture_rows = $sampleCount
                motor_capture_events = 2
            }
        } | ConvertTo-Json -Depth 4),
        [Text.UTF8Encoding]::new($false))

    $analysis = & (Join-Path $PSScriptRoot '..\tools\analyze_g2_dynamic_step.ps1') `
        -CaptureDirectory $captureDirectory `
        -PlanDirectory $plan.ExperimentDirectory
    $summary =
        Get-Content -LiteralPath $analysis.SummaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    Assert-True ($analysis.Accepted -and $summary.accepted) `
        '端到端接受完整合成动态阶跃'
    Assert-True (-not $analysis.ModelReady -and -not $summary.model_ready) `
        '单次阶跃不得宣称模型就绪'
    Assert-True ($summary.dynamics.ramp_up_ms -eq 239 -and
                 $summary.dynamics.ramp_down_ms -eq 239) `
        '识别1 count/ms双向安全斜坡'
    Assert-True ($summary.dynamics.signed_total_displacement_counts -gt 1000 -and
                 $summary.dynamics.active_battery_minimum_mv -eq 11550) `
        '识别期望方向位移和带载最低电压'
    Assert-True ($summary.command_dispatch.maximum_absolute_error_ms -eq 5) `
        '记录主机命令调度误差'

    $telemetry[-1].encoder_total_2 = 2000
    $telemetry |
        Export-Csv -LiteralPath (Join-Path $captureDirectory 'telemetry.csv') `
        -NoTypeInformation -Encoding utf8
    $rejectedOutput = Join-Path $temporaryDirectory 'rejected'
    $rejected = & (Join-Path $PSScriptRoot '..\tools\analyze_g2_dynamic_step.ps1') `
        -CaptureDirectory $captureDirectory `
        -PlanDirectory $plan.ExperimentDirectory `
        -OutputDirectory $rejectedOutput
    $rejectedSummary =
        Get-Content -LiteralPath $rejected.SummaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    Assert-True (-not $rejected.Accepted -and
                 -not $rejectedSummary.gates.other_channels_within_limit) `
        '其他电机编码器越界必须拒绝动态阶跃'
} finally {
    if (Test-Path -LiteralPath $temporaryDirectory -PathType Container) {
        Remove-Item -LiteralPath $temporaryDirectory -Recurse -Force
    }
}

Write-Host "g2_dynamic_step：$assertionCount 项断言通过"
