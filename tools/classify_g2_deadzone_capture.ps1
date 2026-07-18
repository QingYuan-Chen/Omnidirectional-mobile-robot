[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CaptureDirectory,

    [Parameter(Mandatory = $true)]
    [ValidateSet('Positive', 'Negative')]
    [string]$Direction,

    [ValidateRange(1, 1000000)]
    [int64]$MotionThresholdCounts = 1000,

    [ValidateRange(1, 1000000)]
    [int64]$OtherChannelLimitCounts = 1000
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_experiment_lib.ps1')

$resolvedCapture = (Resolve-Path -LiteralPath $CaptureDirectory).Path
$metadataPath = Join-Path $resolvedCapture 'metadata.json'
$telemetryPath = Join-Path $resolvedCapture 'telemetry.csv'
$outputPath = Join-Path $resolvedCapture 'g2_deadzone_classification.json'

$metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
$telemetry = @(Import-Csv -LiteralPath $telemetryPath)
if ([string]$metadata.outcome -cne 'completed' -or
    [uint64]$metadata.counts.telemetry_parse_errors -ne 0 -or
    $telemetry.Count -ne [int]$metadata.counts.telemetry_rows) {
    throw '采集未完整完成、包含解析错误或遥测行数不闭合，拒绝死区判定'
}

$motion = Measure-G2DeadzoneMotion `
    -Rows $telemetry `
    -Direction $Direction `
    -MotionThresholdCounts $MotionThresholdCounts `
    -OtherChannelLimitCounts $OtherChannelLimitCounts

$last = $telemetry[-1]
$errorNames = @(
    'uart_error_count',
    'uart_rx_overflow_count',
    'uart_tx_fault_count',
    'command_reject_count',
    'command_queue_drop_count',
    'motion_gate_reject_count',
    'invalidated_motor_command_count',
    'adc_error_count'
)
$errorMaxima = [ordered]@{}
foreach ($name in $errorNames) {
    $errorMaxima[$name] = [uint64](($telemetry | Measure-Object -Property $name -Maximum).Maximum)
}
$errorsZero = @($errorMaxima.Values | Where-Object { $_ -ne 0 }).Count -eq 0
$safeFinalState = (
    [int32]$last.target_pwm -eq 0 -and
    [int32]$last.applied_pwm -eq 0 -and
    [uint32]$last.motor_state -eq 0 -and
    [uint32]$last.fault_latched -eq 0 -and
    [uint32]$last.estop_latched -eq 0)

$result = [ordered]@{
    schema_version = 1
    source_capture_directory = $resolvedCapture
    firmware_commit = [string]$metadata.firmware_commit
    direction = $Direction
    classification = $motion
    safety = [ordered]@{
        battery_minimum_mv = [uint32](($telemetry | Measure-Object -Property battery_mv -Minimum).Minimum)
        final_state_safe = $safeFinalState
        error_maxima = $errorMaxima
        errors_zero = $errorsZero
    }
    accepted = ($motion.moved -and $safeFinalState -and $errorsZero)
}

$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
[IO.File]::WriteAllText(
    $outputPath,
    ($result | ConvertTo-Json -Depth 6),
    $utf8WithoutBom)

[pscustomobject]@{
    Classification = $outputPath
    Moved = [bool]$motion.moved
    ExpectedExcursionCounts = [int64]$motion.expected_direction_excursion_counts
    OtherChannelMaximumCounts = [int64]$motion.other_channel_maximum_excursion_counts
    FinalStateSafe = $safeFinalState
    ErrorsZero = $errorsZero
    Accepted = [bool]$result.accepted
}
