[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CaptureDirectory,

    [Parameter(Mandatory = $true)]
    [string]$PlanDirectory,

    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_low_speed_diagnostic_lib.ps1')

$resolvedCapture = (Resolve-Path -LiteralPath $CaptureDirectory).Path
$resolvedPlan = (Resolve-Path -LiteralPath $PlanDirectory).Path
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $resolvedCapture
} else {
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
    [void][IO.Directory]::CreateDirectory($resolvedOutput)
}

$manifestPath = Join-Path $resolvedPlan 'experiment_manifest.json'
$schedulePath = Join-Path $resolvedPlan 'command_schedule.pending.csv'
$captureMetadataPath = Join-Path $resolvedCapture 'metadata.json'
$samplesPath = Join-Path $resolvedCapture 'motor_capture.csv'
$telemetryPath = Join-Path $resolvedCapture 'telemetry.csv'
$commandsPath = Join-Path $resolvedCapture 'commands.csv'
foreach ($path in @(
    $manifestPath, $schedulePath, $captureMetadataPath,
    $samplesPath, $telemetryPath, $commandsPath
)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "动态阶跃证据文件不存在：$path"
    }
}

$manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 |
    ConvertFrom-Json
$captureMetadata =
    Get-Content -LiteralPath $captureMetadataPath -Raw -Encoding UTF8 |
        ConvertFrom-Json
$samples = @(Import-Csv -LiteralPath $samplesPath)
$telemetry = @(Import-Csv -LiteralPath $telemetryPath)
$plannedCommands = @(Import-Csv -LiteralPath $schedulePath)
$actualCommands = @(Import-Csv -LiteralPath $commandsPath)
$analysisProfile = Get-G2SingleCaptureAnalysisProfile `
    -Manifest $manifest `
    -Schedule $plannedCommands

$baseAnalysis = & (Join-Path $PSScriptRoot 'analyze_motor_capture.ps1') `
    -CaptureDirectory $resolvedCapture `
    -OutputDirectory $resolvedOutput
$baseSummary =
    Get-Content -LiteralPath $baseAnalysis.SummaryPath -Raw -Encoding UTF8 |
        ConvertFrom-Json

$scheduleMatches = $plannedCommands.Count -eq $actualCommands.Count
[uint64]$maximumDispatchErrorMs = 0
if ($scheduleMatches) {
    for ($index = 0; $index -lt $plannedCommands.Count; $index++) {
        [uint64]$planned = [uint64]$plannedCommands[$index].elapsed_ms
        [uint64]$recordedPlan = [uint64]$actualCommands[$index].planned_elapsed_ms
        [uint64]$actual = [uint64]$actualCommands[$index].actual_elapsed_ms
        $dispatchError = if ($actual -ge $planned) {
            $actual - $planned
        } else {
            $planned - $actual
        }
        $maximumDispatchErrorMs = [Math]::Max(
            $maximumDispatchErrorMs, $dispatchError)
        if ($planned -ne $recordedPlan -or
            [string]$plannedCommands[$index].command -cne
                [string]$actualCommands[$index].command -or
            [string]$actualCommands[$index].result -cne 'sent') {
            $scheduleMatches = $false
            break
        }
    }
}

$measurement = Measure-G2DynamicStep `
    -Samples $samples `
    -Telemetry $telemetry `
    -Direction ([string]$manifest.direction) `
    -PeakPwm ([int]$manifest.peak_pwm) `
    -MotionThresholdCounts ([int64]$manifest.analysis.motion_threshold_counts) `
    -OtherChannelLimitCounts ([int64]$manifest.analysis.other_channel_limit_counts) `
    -MinimumBatteryMillivolts ([uint16]$manifest.analysis.minimum_battery_mv) `
    -CountsPerWheelRevolution (
        [int64]$manifest.analysis.counts_per_wheel_revolution)

$gates = [ordered]@{
    base_high_speed_capture_accepted = [bool]$baseSummary.accepted
    firmware_commit_matches_plan =
        ([string]$captureMetadata.firmware_commit -ceq
         [string]$manifest.firmware_commit)
    command_schedule_matches = $scheduleMatches
    maximum_command_dispatch_error_at_most_100_ms =
        ($maximumDispatchErrorMs -le 100)
}
foreach ($property in $measurement.Gates.PSObject.Properties) {
    $gates[$property.Name] = [bool]$property.Value
}
$fullPwmPlateauMs =
    [int]$measurement.Summary.target_stop_index -
    [int]$measurement.Summary.peak_applied_index
$gates.full_pwm_plateau_meets_plan_minimum =
    ($fullPwmPlateauMs -ge $analysisProfile.minimum_full_pwm_plateau_ms)
$lowSpeedDiagnostic = $null
if ($analysisProfile.low_speed_steady_validation) {
    $lowSpeedDiagnostic = Measure-G2LowSpeedRun `
        -Samples $samples `
        -ExperimentId ([string]$manifest.experiment_id) `
        -FirstTargetIndex ([int]$measurement.Summary.first_target_index) `
        -PeakAppliedIndex ([int]$measurement.Summary.peak_applied_index) `
        -TargetStopIndex ([int]$measurement.Summary.target_stop_index) `
        -MotionThresholdDelayMs (
            [int]$measurement.Summary.motion_threshold_delay_ms) `
        -ReportedPeakWindowRpm (
            [double]$measurement.Summary.peak_window_wheel_rpm) `
        -CountsPerWheelRevolution (
            [int]$manifest.analysis.counts_per_wheel_revolution)
}
$accepted = -not ($gates.Values -contains $false)

$summary = [ordered]@{
    schema_version = 1
    experiment_id = [string]$manifest.experiment_id
    experiment_type = [string]$analysisProfile.experiment_type
    source_capture_directory = $resolvedCapture
    source_plan_directory = $resolvedPlan
    source_firmware_commit = [string]$captureMetadata.firmware_commit
    plan_repository_commit = [string]$manifest.repository_commit
    execution_state_recorded_in_plan = [string]$manifest.execution_state
    command_dispatch = [ordered]@{
        planned_count = $plannedCommands.Count
        sent_count = $actualCommands.Count
        maximum_absolute_error_ms = $maximumDispatchErrorMs
    }
    dynamics = $measurement.Summary
    low_speed_diagnostic = $lowSpeedDiagnostic
    high_speed_timing = $baseSummary.timing
    telemetry_counter_increments = $baseSummary.telemetry_counter_increments
    gates = $gates
    accepted = $accepted
    model_ready = $false
    model_readiness_reason =
        '单次受控阶跃只验证数据质量；必须完成多档双向重复、独立验证集和残差评估。'
}

$summaryPath = Join-Path $resolvedOutput 'g2_dynamic_step_summary.json'
[IO.File]::WriteAllText(
    $summaryPath,
    ($summary | ConvertTo-Json -Depth 10),
    [Text.UTF8Encoding]::new($false))

[pscustomobject]@{
    Accepted = $accepted
    ModelReady = $false
    SummaryPath = $summaryPath
    BaseSummaryPath = $baseAnalysis.SummaryPath
}
