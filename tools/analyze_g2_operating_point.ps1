[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CaptureDirectory,

    [Parameter(Mandatory = $true)]
    [ValidateSet('Positive', 'Negative')]
    [string]$Direction,

    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 840)]
    [int]$PeakPwm,

    [ValidateRange(1, 10000000)]
    [int64]$CountsPerWheelRevolution = 122880,

    [ValidateRange(0, 5000)]
    [int]$SteadySettleMilliseconds = 500,

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
$outputPath = Join-Path $resolvedCapture 'g2_operating_point_summary.json'

$metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
$telemetry = @(Import-Csv -LiteralPath $telemetryPath)
$typedParseErrors = [uint64]0
foreach ($name in @(
    'stat_parse_errors', 'imuq_parse_errors',
    'resource_parse_errors', 'event_parse_errors',
    'motor_capture_parse_errors', 'speed_capture_parse_errors',
    'imu_capture_parse_errors')) {
    if ($null -ne $metadata.counts.PSObject.Properties[$name]) {
        $typedParseErrors += [uint64]$metadata.counts.$name
    }
}
if ([string]$metadata.outcome -cne 'completed' -or
    [uint64]$metadata.counts.telemetry_parse_errors -ne 0 -or
    $typedParseErrors -ne 0 -or
    $telemetry.Count -ne [int]$metadata.counts.telemetry_rows) {
    throw '采集未完整完成、包含解析错误或遥测行数不闭合，拒绝工作点分析'
}

$measurement = Measure-G2OperatingPoint `
    -Rows $telemetry `
    -Direction $Direction `
    -PeakPwm $PeakPwm `
    -CountsPerWheelRevolution $CountsPerWheelRevolution `
    -SteadySettleMilliseconds $SteadySettleMilliseconds `
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
    $errorMaxima[$name] = [uint64]((
        $telemetry | Measure-Object -Property $name -Maximum).Maximum)
}
$errorsZero = @($errorMaxima.Values | Where-Object { $_ -ne 0 }).Count -eq 0
$safeFinalState = (
    [int32]$last.target_pwm -eq 0 -and
    [int32]$last.applied_pwm -eq 0 -and
    [uint32]$last.motor_state -eq 0 -and
    [uint32]$last.fault_latched -eq 0 -and
    [uint32]$last.estop_latched -eq 0)
$accepted = (
    $measurement.motion.moved -and
    -not $measurement.motion.wrong_direction_detected -and
    -not $measurement.motion.other_channel_motion_detected -and
    $measurement.steady_window.signed_count_rate_cps -gt 0 -and
    $safeFinalState -and
    $errorsZero)

$result = [ordered]@{
    schema_version = 1
    source_capture_directory = $resolvedCapture
    firmware_commit = [string]$metadata.firmware_commit
    measurement = $measurement
    safety = [ordered]@{
        final_state_safe = $safeFinalState
        error_maxima = $errorMaxima
        errors_zero = $errorsZero
    }
    accepted = $accepted
}

$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
[IO.File]::WriteAllText(
    $outputPath,
    ($result | ConvertTo-Json -Depth 8),
    $utf8WithoutBom)

[pscustomobject]@{
    Summary = $outputPath
    Direction = $Direction
    PeakPwm = $PeakPwm
    SteadyCountRateCps = [Math]::Round(
        [double]$measurement.steady_window.signed_count_rate_cps, 1)
    WheelRpm = [Math]::Round(
        [double]$measurement.steady_window.wheel_rpm, 3)
    MotionStartDelayMs = $measurement.response.motion_start_delay_ms
    BatteryMinimumMv = $measurement.battery_mv.active_minimum
    FinalStateSafe = $safeFinalState
    ErrorsZero = $errorsZero
    Accepted = $accepted
}
