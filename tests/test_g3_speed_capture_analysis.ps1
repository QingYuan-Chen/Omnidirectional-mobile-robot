$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot '..\tools\g3_speed_capture_analysis_lib.ps1')

$assertionCount = 0
function Assert-True {
    param([bool]$Condition, [string]$Message)
    $script:assertionCount++
    if (-not $Condition) {
        throw "断言失败：$Message"
    }
}

$rows = [Collections.Generic.List[object]]::new()
for ($index = 0; $index -lt 100; $index++) {
    $rows.Add([pscustomobject]@{
        capture_index = [uint32]$index
        control_tick_sequence = [uint32](1000 + $index)
        irq_timestamp_cycles = [uint32](168000 * $index)
        encoder_delta_ma = 4
        applied_pwm = 0
        period_sum_cycles = if ($index -eq 0) { 0 } else { 168000 }
        period_count = if ($index -eq 0) { 0 } else { 1 }
        last_edge_age_cycles = 0
        event_sequence = [uint32]$index
        direction = 1
        period_flags = if ($index -eq 0) { 1 } else { 3 }
    })
}
$measurement = Measure-G3SpeedCaptureRows -Rows @($rows)
Assert-True ($measurement.sample_count -eq 100) '样本计数'
Assert-True ($measurement.index_break_count -eq 0 -and
             $measurement.tick_break_count -eq 0) '索引与 tick 连续'
Assert-True ($measurement.encoder_count_sum -eq 400 -and
             $measurement.event_sequence_delta -eq 99) '累计编码器与事件'
Assert-True ([Math]::Abs($measurement.encoder_counts_per_observed_event - 4.040404) -lt 0.00001) '每事件编码器计数'
Assert-True ([Math]::Abs($measurement.m_speed_rpm.mean - 1.953125) -lt 0.000001) '10 ms M 法'
Assert-True ([Math]::Abs($measurement.t_speed_rpm.mean - 1.953125) -lt 0.000001) '周期 T 法'
Assert-True (-not $measurement.model_ready) '分析不提前放行模型'

$rows[50].capture_index = 52
$rows[50].control_tick_sequence = 1052
$rows[50].applied_pwm = 1
$rows[50].period_flags = 8
$fault = Measure-G3SpeedCaptureRows -Rows @($rows)
Assert-True ($fault.index_break_count -eq 2 -and
             $fault.tick_break_count -eq 2) '连续性断点双边计数'
Assert-True ($fault.nonzero_pwm_row_count -eq 1 -and
             $fault.aggregate_drop_flag_row_count -eq 1) 'PWM 与聚合故障'

$wrapRows = @(
    [pscustomobject]@{
        capture_index = [uint32]::MaxValue
        control_tick_sequence = [uint32]::MaxValue
        irq_timestamp_cycles = 0
        encoder_delta_ma = -4
        applied_pwm = 0
        period_sum_cycles = 168000
        period_count = 1
        last_edge_age_cycles = 0
        event_sequence = [uint32]::MaxValue
        direction = -1
        period_flags = 3
    },
    [pscustomobject]@{
        capture_index = 0
        control_tick_sequence = 0
        irq_timestamp_cycles = 168000
        encoder_delta_ma = -4
        applied_pwm = 0
        period_sum_cycles = 168000
        period_count = 1
        last_edge_age_cycles = 0
        event_sequence = 0
        direction = -1
        period_flags = 3
    }
)
$wrap = Measure-G3SpeedCaptureRows -Rows $wrapRows -MWindowTicks 2
Assert-True ($wrap.index_break_count -eq 0 -and
             $wrap.tick_break_count -eq 0 -and
             $wrap.event_sequence_delta -eq 1) 'uint32 回绕连续'
Assert-True ($wrap.m_speed_rpm.mean -lt 0 -and
             $wrap.t_speed_rpm.mean -lt 0) '反向 M/T 符号'

$temporaryDirectory = Join-Path (
    [IO.Path]::GetTempPath()) ('g3-speed-analysis-' + [Guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporaryDirectory)
try {
    @($rows | ForEach-Object {
        $_.capture_index = [uint32]$rows.IndexOf($_)
        $_.control_tick_sequence = [uint32](1000 + $rows.IndexOf($_))
        $_.applied_pwm = 0
        $_.period_flags = if ($_.capture_index -eq 0) { 1 } else { 3 }
        $_
    }) | Export-Csv -LiteralPath (
        Join-Path $temporaryDirectory 'speed_capture.csv') -NoTypeInformation -Encoding utf8
    @(
        [pscustomobject]@{
            event = 'BEGIN'; state = 2; sample_count = 100; capacity = 2200
            dropped_sample_count = 0; invalid_direction_count = 0
            zero_period_count = 0; aggregate_drop_count = 0
            direction_reset_count = 0
        },
        [pscustomobject]@{
            event = 'END'; state = 2; sample_count = 100; capacity = 2200
            dropped_sample_count = 0; invalid_direction_count = 0
            zero_period_count = 0; aggregate_drop_count = 0
            direction_reset_count = 0
        }
    ) | Export-Csv -LiteralPath (
        Join-Path $temporaryDirectory 'speed_capture_events.csv') -NoTypeInformation -Encoding utf8
    $metadata = [pscustomobject]@{
        outcome = 'completed'
        counts = [pscustomobject]@{
            speed_capture_rows = 100
            speed_capture_events = 2
            speed_capture_parse_errors = 0
        }
    }
    [IO.File]::WriteAllText(
        (Join-Path $temporaryDirectory 'metadata.json'),
        ($metadata | ConvertTo-Json -Depth 4),
        [Text.UTF8Encoding]::new($false))
    $expectedTurns = 99.0 / 30720.0
    $summary = & (Join-Path $PSScriptRoot '..\tools\analyze_g3_speed_capture.ps1') `
        -CaptureDirectory $temporaryDirectory `
        -ExpectedWheelTurns $expectedTurns `
        -OutputDirectory (Join-Path $temporaryDirectory 'analysis')
    Assert-True ($summary.accepted -and -not $summary.model_ready) '端到端分析接受合成证据但不放行模型'
    Assert-True ((Test-Path -LiteralPath (
        Join-Path $temporaryDirectory 'analysis\g3_speed_capture_summary.json'))) '写入分析摘要'
} finally {
    if (Test-Path -LiteralPath $temporaryDirectory -PathType Container) {
        Remove-Item -LiteralPath $temporaryDirectory -Recurse -Force
    }
}

Write-Host "g3_speed_capture_analysis：$assertionCount 项断言通过"
