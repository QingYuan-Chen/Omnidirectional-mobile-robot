[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CaptureDirectory,

    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'

$resolvedCapture = (Resolve-Path -LiteralPath $CaptureDirectory).Path
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $resolvedCapture
} else {
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
    [void][IO.Directory]::CreateDirectory($resolvedOutput)
}

$metadataPath = Join-Path $resolvedCapture 'metadata.json'
$samplesPath = Join-Path $resolvedCapture 'imu_capture.csv'
$eventsPath = Join-Path $resolvedCapture 'imu_capture_events.csv'
foreach ($path in @($metadataPath, $samplesPath, $eventsPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "IMU 高速采集证据文件不存在：$path"
    }
}

$metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
$samples = @(Import-Csv -LiteralPath $samplesPath)
$events = @(Import-Csv -LiteralPath $eventsPath)
$beginEvents = @($events | Where-Object { $_.event -ceq 'BEGIN' })
$endEvents = @($events | Where-Object { $_.event -ceq 'END' })

$indexBreakCount = [uint64]0
$sequenceBreakCount = [uint64]0
$sensorTimestampInvalidCount = [uint64]0
$hostTickInvalidCount = [uint64]0
$statusNotReadyCount = [uint64]0
$sourceDropDelta = [uint64]0
$totalSensorSteps = [uint64]0
$hostElapsedMs = [uint64]0
$hostDeltas = [Collections.Generic.List[uint64]]::new()
$sensorDeltas = [Collections.Generic.List[uint64]]::new()

for ($index = 0; $index -lt $samples.Count; $index++) {
    $row = $samples[$index]
    if ([uint64]$row.capture_index -ne [uint64]$index) {
        $indexBreakCount++
    }
    if (([uint64]$row.sensor_status -band 3) -ne 3) {
        $statusNotReadyCount++
    }
    if ($index -eq 0) {
        continue
    }
    $previous = $samples[$index - 1]
    $sequenceDelta =
        (([uint64]$row.imu_sequence + 4294967296) -
         [uint64]$previous.imu_sequence) % 4294967296
    if ($sequenceDelta -ne 1) {
        $sequenceBreakCount++
    }
    $sensorDelta =
        (([uint64]$row.sensor_timestamp + 16777216) -
         [uint64]$previous.sensor_timestamp) % 16777216
    $sensorDeltas.Add($sensorDelta)
    $totalSensorSteps += $sensorDelta
    if ($sensorDelta -eq 0 -or $sensorDelta -gt 11) {
        $sensorTimestampInvalidCount++
    }
    $hostDelta =
        (([uint64]$row.host_tick_ms + 4294967296) -
         [uint64]$previous.host_tick_ms) % 4294967296
    $hostDeltas.Add($hostDelta)
    $hostElapsedMs += $hostDelta
    if ($hostDelta -eq 0 -or $hostDelta -gt 1000) {
        $hostTickInvalidCount++
    }
}

if ($samples.Count -gt 1) {
    $firstDrop = [uint64]$samples[0].source_dropped_sample_count
    $lastDrop = [uint64]$samples[-1].source_dropped_sample_count
    $sourceDropDelta =
        (($lastDrop + 4294967296) - $firstDrop) % 4294967296
}

$beginStatusValid =
    $beginEvents.Count -eq 1 -and
    [uint64]$beginEvents[0].state -eq 2 -and
    [uint64]$beginEvents[0].sample_count -eq [uint64]$samples.Count -and
    [uint64]$beginEvents[0].capacity -eq 1700 -and
    [uint64]$beginEvents[0].dropped_sample_count -eq 0 -and
    [uint64]$beginEvents[0].duplicate_sequence_count -eq 0 -and
    [uint64]$beginEvents[0].source_gap_count -eq 0
$endStatusValid =
    $endEvents.Count -eq 1 -and
    [uint64]$endEvents[0].state -eq 2 -and
    [uint64]$endEvents[0].sample_count -eq [uint64]$samples.Count -and
    [uint64]$endEvents[0].capacity -eq 1700 -and
    [uint64]$endEvents[0].dropped_sample_count -eq 0 -and
    [uint64]$endEvents[0].duplicate_sequence_count -eq 0 -and
    [uint64]$endEvents[0].source_gap_count -eq 0
$sensorDurationMs = [double]$totalSensorSteps * 1000.0 / 224.2
$durationErrorMs = [Math]::Abs([double]$hostElapsedMs - $sensorDurationMs)
$durationToleranceMs = [Math]::Max(10.0, $sensorDurationMs * 0.02)
$durationConsistent =
    $samples.Count -ge 100 -and
    $durationErrorMs -le $durationToleranceMs

$checks = [ordered]@{
    metadata_completed = [string]$metadata.outcome -ceq 'completed'
    metadata_counts_match =
        [uint64]$metadata.counts.imu_capture_rows -eq [uint64]$samples.Count -and
        [uint64]$metadata.counts.imu_capture_events -eq [uint64]$events.Count
    parse_errors_zero =
        [uint64]$metadata.counts.imu_capture_parse_errors -eq 0
    begin_end_present = $beginEvents.Count -eq 1 -and $endEvents.Count -eq 1
    begin_end_complete_no_drop = $beginStatusValid -and $endStatusValid
    samples_sufficient_for_timing = $samples.Count -ge 100
    indexes_contiguous = $indexBreakCount -eq 0
    accepted_sequence_contiguous = $sequenceBreakCount -eq 0
    sensor_timestamp_progresses = $sensorTimestampInvalidCount -eq 0
    host_tick_progresses = $hostTickInvalidCount -eq 0
    sensor_host_duration_consistent = $durationConsistent
    status0_accel_gyro_ready = $statusNotReadyCount -eq 0
    source_drop_counter_unchanged = $sourceDropDelta -eq 0
}
$accepted = @($checks.Values | Where-Object { -not $_ }).Count -eq 0

$hostDeltaSummary = [ordered]@{
    minimum_ms = if ($hostDeltas.Count -gt 0) {
        [uint64](($hostDeltas | Measure-Object -Minimum).Minimum)
    } else { $null }
    maximum_ms = if ($hostDeltas.Count -gt 0) {
        [uint64](($hostDeltas | Measure-Object -Maximum).Maximum)
    } else { $null }
    count_4ms = [uint64]@($hostDeltas | Where-Object { $_ -eq 4 }).Count
    count_5ms = [uint64]@($hostDeltas | Where-Object { $_ -eq 5 }).Count
}
$sensorDeltaSummary = [ordered]@{
    minimum_ticks = if ($sensorDeltas.Count -gt 0) {
        [uint64](($sensorDeltas | Measure-Object -Minimum).Minimum)
    } else { $null }
    maximum_ticks = if ($sensorDeltas.Count -gt 0) {
        [uint64](($sensorDeltas | Measure-Object -Maximum).Maximum)
    } else { $null }
}

$summary = [ordered]@{
    schema_version = 1
    analyzed_utc = [DateTimeOffset]::UtcNow.ToString('O')
    source_capture_directory = $resolvedCapture
    firmware_commit = [string]$metadata.firmware_commit
    accepted = $accepted
    checks = $checks
    metrics = [ordered]@{
        sample_count = [uint64]$samples.Count
        event_count = [uint64]$events.Count
        index_break_count = $indexBreakCount
        sequence_break_count = $sequenceBreakCount
        sensor_timestamp_invalid_count = $sensorTimestampInvalidCount
        host_tick_invalid_count = $hostTickInvalidCount
        status_not_ready_count = $statusNotReadyCount
        source_drop_delta = $sourceDropDelta
        total_sensor_steps = $totalSensorSteps
        sensor_duration_ms_at_224_2_hz = $sensorDurationMs
        host_elapsed_ms = $hostElapsedMs
        sensor_host_duration_error_ms = $durationErrorMs
        sensor_host_duration_tolerance_ms = $durationToleranceMs
        host_tick_delta = $hostDeltaSummary
        sensor_timestamp_delta = $sensorDeltaSummary
    }
    limitations = @(
        'IC records only AppImu_Process outputs that passed the current acceptance chain.',
        'Rejected spikes, duplicate timestamps and bus failures are visible only through IMUQ counters.',
        'The duration gate compares sensor timestamp steps at configured 224.2 Hz with host ticks using max(10 ms, 2%) tolerance.',
        'Host millisecond deltas remain quantized; final ODR acceptance still requires a real no-power board capture.'
    )
}

$summaryPath = Join-Path $resolvedOutput 'imu_capture_analysis.json'
[IO.File]::WriteAllText(
    $summaryPath,
    ($summary | ConvertTo-Json -Depth 8),
    [Text.UTF8Encoding]::new($false))

if (-not $accepted) {
    throw "IMU 高速采集严格门未通过，摘要已保存：$summaryPath"
}

[pscustomobject]@{
    Summary = $summaryPath
    Accepted = $accepted
    SampleCount = $samples.Count
}
