[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CaptureDirectory,

    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_experiment_lib.ps1')

$resolvedCapture = (Resolve-Path -LiteralPath $CaptureDirectory).Path
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $resolvedCapture
} else {
    if (-not [IO.Path]::IsPathRooted($OutputDirectory)) {
        $OutputDirectory = Join-Path (Get-Location).Path $OutputDirectory
    }
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
    [void][IO.Directory]::CreateDirectory($resolvedOutput)
}

$rawPath = Join-Path $resolvedCapture 'raw_uart.log'
$telemetryPath = Join-Path $resolvedCapture 'telemetry.csv'
$commandsPath = Join-Path $resolvedCapture 'commands.csv'
$metadataPath = Join-Path $resolvedCapture 'metadata.json'
foreach ($path in @($rawPath, $telemetryPath, $commandsPath, $metadataPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "采集目录缺少必要文件：$path"
    }
}

$metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
if ($metadata.outcome -cne 'completed') {
    throw "只分析完成的采集，当前 outcome=$($metadata.outcome)"
}
$telemetry = @(Import-Csv -LiteralPath $telemetryPath)
$commands = @(Import-Csv -LiteralPath $commandsPath)
$rawBytes = [uint64](Get-Item -LiteralPath $rawPath).Length
$additionalTypedRows = [uint64]0
$typedParseErrors = [uint64]0
foreach ($name in @('imuq_rows', 'resource_rows', 'event_rows')) {
    if ($null -ne $metadata.counts.PSObject.Properties[$name]) {
        $additionalTypedRows += [uint64]$metadata.counts.$name
    }
}
foreach ($name in @(
    'motor_capture_rows', 'motor_capture_events',
    'speed_capture_rows', 'speed_capture_events',
    'imu_capture_rows', 'imu_capture_events')) {
    if ($null -ne $metadata.counts.PSObject.Properties[$name]) {
        $additionalTypedRows += [uint64]$metadata.counts.$name
    }
}
foreach ($name in @(
    'stat_parse_errors', 'imuq_parse_errors',
    'resource_parse_errors', 'event_parse_errors',
    'motor_capture_parse_errors', 'speed_capture_parse_errors',
    'imu_capture_parse_errors')) {
    if ($null -ne $metadata.counts.PSObject.Properties[$name]) {
        $typedParseErrors += [uint64]$metadata.counts.$name
    }
}

$consistency = [ordered]@{
    raw_bytes_match = ($rawBytes -eq [uint64]$metadata.counts.raw_bytes)
    telemetry_rows_match = ($telemetry.Count -eq [int]$metadata.counts.telemetry_rows)
    command_rows_match = ($commands.Count -eq [int]$metadata.counts.commands_sent)
    complete_line_partition_match =
        ([uint64]$metadata.counts.complete_lines -eq
         ([uint64]$metadata.counts.telemetry_rows +
          $additionalTypedRows +
          [uint64]$metadata.counts.non_telemetry_lines))
    parse_errors_zero =
        ([uint64]$metadata.counts.telemetry_parse_errors -eq 0 -and
         $typedParseErrors -eq 0)
}
if (@($consistency.Values | Where-Object { -not $_ }).Count -gt 0) {
    throw '采集产物计数不一致或包含解析异常，拒绝生成 G2 摘要'
}

$measurement = Measure-G2CaptureRows -Rows $telemetry
$analysis = [ordered]@{
    schema_version = 1
    analyzed_utc = [DateTimeOffset]::UtcNow.ToString('O')
    source_capture_directory = $resolvedCapture
    firmware_commit = [string]$metadata.firmware_commit
    capture_repository_commit = [string]$metadata.repository_commit
    artifact_consistency = $consistency
    capture_boundaries = [ordered]@{
        non_telemetry_lines = [uint64]$metadata.counts.non_telemetry_lines
        trailing_partial_characters = [uint64]$metadata.counts.trailing_partial_characters
        note = 'A serial capture may begin or end inside a frame; raw bytes remain authoritative.'
    }
    metrics = $measurement.summary
}

$summaryPath = Join-Path $resolvedOutput 'g2_analysis_summary.json'
$derivedPath = Join-Path $resolvedOutput 'g2_telemetry_derived.csv'
$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
[IO.File]::WriteAllText(
    $summaryPath,
    ($analysis | ConvertTo-Json -Depth 10),
    $utf8WithoutBom)
$measurement.derived_rows |
    Export-Csv -LiteralPath $derivedPath -NoTypeInformation -Encoding UTF8

[pscustomobject]@{
    Summary = $summaryPath
    DerivedTelemetry = $derivedPath
    TelemetryRows = $telemetry.Count
    BoardDurationMs = $measurement.summary.board_duration_ms
    PreliminaryTimingThresholdsPass =
        Test-G2PreliminaryTimingThresholds `
            -Thresholds $measurement.summary.timing.preliminary_thresholds
    G3Ready = $measurement.summary.g3_readiness.rpm_reported
}
