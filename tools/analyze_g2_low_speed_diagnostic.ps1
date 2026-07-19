[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string[]]$CaptureDirectories,

    [ValidateSet('exploratory', 'independent_validation')]
    [string]$EvidenceRole = 'exploratory',

    [ValidateRange(3, 5)]
    [int]$RequiredRepeatCount = 3,

    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_low_speed_diagnostic_lib.ps1')

if ($CaptureDirectories.Count -lt 3 -or $CaptureDirectories.Count -gt 5) {
    throw '低速诊断只接受3至5个采集目录'
}
$resolvedCaptures = [Collections.Generic.List[string]]::new()
$seenCaptures = @{}
$rows = [Collections.Generic.List[object]]::new()
$firmwareCommits = [Collections.Generic.List[string]]::new()
foreach ($captureDirectory in $CaptureDirectories) {
    $resolvedCapture = (Resolve-Path -LiteralPath $captureDirectory).Path
    if ($seenCaptures.ContainsKey($resolvedCapture)) {
        throw "低速诊断采集目录重复：$resolvedCapture"
    }
    $seenCaptures[$resolvedCapture] = $true
    $resolvedCaptures.Add($resolvedCapture)

    $summaryPath = Join-Path $resolvedCapture 'g2_dynamic_step_summary.json'
    $samplesPath = Join-Path $resolvedCapture 'motor_capture.csv'
    foreach ($path in @($summaryPath, $samplesPath)) {
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
            throw "低速诊断输入不存在：$path"
        }
    }
    $summary =
        Get-Content -LiteralPath $summaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    if ([string]$summary.dynamics.direction -cne 'Negative' -or
        [int]$summary.dynamics.peak_pwm -ne 240) {
        throw "低速诊断只接受反向240摘要：$summaryPath"
    }
    $firmwareCommit = [string]$summary.source_firmware_commit
    if ($firmwareCommits -notcontains $firmwareCommit) {
        $firmwareCommits.Add($firmwareCommit)
    }
    $samples = @(Import-Csv -LiteralPath $samplesPath)
    $measurement = Measure-G2LowSpeedRun `
        -Samples $samples `
        -ExperimentId ([string]$summary.experiment_id) `
        -FirstTargetIndex ([int]$summary.dynamics.first_target_index) `
        -PeakAppliedIndex ([int]$summary.dynamics.peak_applied_index) `
        -TargetStopIndex ([int]$summary.dynamics.target_stop_index) `
        -MotionThresholdDelayMs (
            [int]$summary.dynamics.motion_threshold_delay_ms) `
        -ReportedPeakWindowRpm (
            [double]$summary.dynamics.peak_window_wheel_rpm)
    $rows.Add([pscustomobject][ordered]@{
        experiment_id = $measurement.experiment_id
        capture_directory = $resolvedCapture
        accepted = [bool]$summary.accepted
        direction = $measurement.direction
        peak_pwm = $measurement.peak_pwm
        first_move_delay_ms = $measurement.first_move_delay_ms
        motion_threshold_delay_ms =
            $measurement.motion_threshold_delay_ms
        full_pwm_plateau_ms = $measurement.full_pwm_plateau_ms
        reported_peak_window_rpm =
            $measurement.reported_peak_window_rpm
        tail_200ms_signed_counts =
            $measurement.tail_200ms_signed_counts
        tail_200ms_wheel_rpm =
            $measurement.tail_200ms_wheel_rpm
        tail_300ms_signed_counts =
            $measurement.tail_300ms_signed_counts
        tail_300ms_wheel_rpm =
            $measurement.tail_300ms_wheel_rpm
    })
}
if ($firmwareCommits.Count -ne 1) {
    throw '低速诊断采集必须来自同一固件提交'
}

$batch = Measure-G2LowSpeedDiagnosticBatch `
    -Rows @($rows) `
    -EvidenceRole $EvidenceRole `
    -RequiredRepeatCount $RequiredRepeatCount
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $resolvedCaptures[0]
} elseif ([IO.Path]::IsPathRooted($OutputDirectory)) {
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
} else {
    $resolvedOutput = [IO.Path]::GetFullPath((
        Join-Path (Get-Location).Path $OutputDirectory))
}
[void][IO.Directory]::CreateDirectory($resolvedOutput)
$rowsPath = Join-Path $resolvedOutput 'g2_low_speed_diagnostic_rows.csv'
$summaryOutputPath =
    Join-Path $resolvedOutput 'g2_low_speed_diagnostic_summary.json'
$rows | Export-Csv -LiteralPath $rowsPath -NoTypeInformation -Encoding utf8
$output = [ordered]@{
    schema_version = 1
    diagnostic_type = 'g2_low_speed_startup_vs_running_speed'
    source_firmware_commit = $firmwareCommits[0]
    source_capture_directories = @($resolvedCaptures)
    fixed_tail_windows_ms = @(200, 300)
    stable_speed_cv_limit_percent = 5.0
    stable_speed_maximum_deviation_limit_percent = 10.0
    startup_delay_cv_limit_percent = 10.0
    batch = $batch
    files = [ordered]@{
        rows_csv = 'g2_low_speed_diagnostic_rows.csv'
        summary_json = 'g2_low_speed_diagnostic_summary.json'
    }
}
[IO.File]::WriteAllText(
    $summaryOutputPath,
    ($output | ConvertTo-Json -Depth 10),
    [Text.UTF8Encoding]::new($false))

[pscustomobject]@{
    Classification = $batch.classification
    StableSpeedAccepted = $batch.stable_speed_repeatability_accepted
    IndependentValidationAccepted = $batch.independent_validation_accepted
    RowsPath = $rowsPath
    SummaryPath = $summaryOutputPath
}
