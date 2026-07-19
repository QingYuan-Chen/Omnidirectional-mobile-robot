param(
    [Parameter(Mandatory = $true)]
    [string]$CaptureDirectory,
    [Parameter(Mandatory = $true)]
    [ValidateRange(0.0001, 100.0)]
    [double]$ExpectedWheelTurns,
    [ValidateRange(0.1, 25.0)]
    [double]$EventCountTolerancePercent = 5.0,
    [string]$OutputDirectory
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'g3_speed_capture_analysis_lib.ps1')

$resolvedCapture = (Resolve-Path -LiteralPath $CaptureDirectory).Path
$metadataPath = Join-Path $resolvedCapture 'metadata.json'
$samplesPath = Join-Path $resolvedCapture 'speed_capture.csv'
$eventsPath = Join-Path $resolvedCapture 'speed_capture_events.csv'
foreach ($path in @($metadataPath, $samplesPath, $eventsPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "缺少 G3 轮速分析输入：$path"
    }
}

$metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
$samples = @(Import-Csv -LiteralPath $samplesPath)
$events = @(Import-Csv -LiteralPath $eventsPath)
$measurement = Measure-G3SpeedCaptureRows -Rows $samples
$endEvents = @($events | Where-Object { $_.event -ceq 'END' })
$beginEvents = @($events | Where-Object { $_.event -ceq 'BEGIN' })
$expectedEvents = 30720.0 * $ExpectedWheelTurns
$eventErrorPercent = if ($expectedEvents -gt 0.0) {
    100.0 * [Math]::Abs($measurement.event_sequence_delta - $expectedEvents) /
        $expectedEvents
} else {
    [double]::PositiveInfinity
}

$checks = [ordered]@{
    metadata_completed = [string]$metadata.outcome -ceq 'completed'
    metadata_counts_match =
        [uint64]$metadata.counts.speed_capture_rows -eq $samples.Count -and
        [uint64]$metadata.counts.speed_capture_events -eq $events.Count
    parse_errors_zero = [uint64]$metadata.counts.speed_capture_parse_errors -eq 0
    begin_end_present = $beginEvents.Count -eq 1 -and $endEvents.Count -eq 1
    end_complete_no_drop =
        $endEvents.Count -eq 1 -and
        [uint32]$endEvents[0].state -eq 2 -and
        [uint64]$endEvents[0].dropped_sample_count -eq 0
    period_errors_zero =
        $endEvents.Count -eq 1 -and
        [uint64]$endEvents[0].invalid_direction_count -eq 0 -and
        [uint64]$endEvents[0].zero_period_count -eq 0 -and
        [uint64]$endEvents[0].aggregate_drop_count -eq 0
    samples_contiguous =
        $measurement.index_break_count -eq 0 -and
        $measurement.tick_break_count -eq 0
    no_motor_output = $measurement.nonzero_pwm_row_count -eq 0
    one_direction = $measurement.direction_reset_flag_row_count -eq 0
    edge_events_present = $measurement.event_sequence_delta -gt 0
    candidate_event_count_matches =
        $eventErrorPercent -le $EventCountTolerancePercent
}
$accepted = @($checks.Values | Where-Object { -not $_ }).Count -eq 0
$summary = [pscustomobject][ordered]@{
    analysis_type = 'g3_speed_hand_turn_validation'
    capture_directory = $resolvedCapture
    expected_wheel_turns = $ExpectedWheelTurns
    candidate_events_per_wheel_revolution = 30720
    observed_events_per_wheel_revolution =
        $measurement.event_sequence_delta / $ExpectedWheelTurns
    event_count_error_percent = $eventErrorPercent
    event_count_tolerance_percent = $EventCountTolerancePercent
    measurement = $measurement
    checks = $checks
    accepted = $accepted
    model_ready = $false
}

if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $resolvedCapture 'analysis_g3_speed'
}
[void][IO.Directory]::CreateDirectory($OutputDirectory)
$summaryPath = Join-Path $OutputDirectory 'g3_speed_capture_summary.json'
[IO.File]::WriteAllText(
    $summaryPath,
    ($summary | ConvertTo-Json -Depth 8),
    [Text.UTF8Encoding]::new($false))
$summary
