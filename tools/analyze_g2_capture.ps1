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

$consistency = [ordered]@{
    raw_bytes_match = ($rawBytes -eq [uint64]$metadata.counts.raw_bytes)
    telemetry_rows_match = ($telemetry.Count -eq [int]$metadata.counts.telemetry_rows)
    command_rows_match = ($commands.Count -eq [int]$metadata.counts.commands_sent)
    parse_errors_zero = ([uint64]$metadata.counts.telemetry_parse_errors -eq 0)
    non_telemetry_lines_zero = ([uint64]$metadata.counts.non_telemetry_lines -eq 0)
    trailing_characters_zero = ([uint64]$metadata.counts.trailing_partial_characters -eq 0)
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
