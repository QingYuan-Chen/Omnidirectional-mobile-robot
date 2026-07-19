[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$BatchDirectory,

    [Parameter(Mandatory = $true)]
    [string[]]$CaptureDirectories,

    [string]$OutputDirectory = '',

    [ValidateRange(3, 5)]
    [int]$RequiredRepeatCount = 3,

    [ValidateRange(0.1, 100.0)]
    [double]$PassCvPercent = 5.0,

    [ValidateRange(0.1, 100.0)]
    [double]$RetestCvPercent = 10.0
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_dynamic_step_lib.ps1')

$resolvedBatch = (Resolve-Path -LiteralPath $BatchDirectory).Path
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $resolvedBatch
} elseif ([IO.Path]::IsPathRooted($OutputDirectory)) {
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
} else {
    $resolvedOutput = [IO.Path]::GetFullPath((
        Join-Path (Get-Location).Path $OutputDirectory))
}
[void][IO.Directory]::CreateDirectory($resolvedOutput)

$batchManifestPath = Join-Path $resolvedBatch 'batch_manifest.csv'
$batchManifestJsonPath = Join-Path $resolvedBatch 'batch_manifest.json'
foreach ($path in @($batchManifestPath, $batchManifestJsonPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "动态阶跃批次清单不存在：$path"
    }
}
$batchRows = @(Import-Csv -LiteralPath $batchManifestPath)
$batchManifest =
    Get-Content -LiteralPath $batchManifestJsonPath -Raw -Encoding UTF8 |
        ConvertFrom-Json
if ($batchManifest.batch_type -cne 'g2_dynamic_bounded_step_batch') {
    throw '目录不是受控动态阶跃批次'
}
if ($CaptureDirectories.Count -ne $batchRows.Count) {
    throw '采集目录数量必须与批次计划数量一致'
}

$planByExperiment = @{}
foreach ($row in $batchRows) {
    $experimentId = [string]$row.experiment_id
    if ($planByExperiment.ContainsKey($experimentId)) {
        throw "批次清单实验ID重复：$experimentId"
    }
    $planByExperiment[$experimentId] = $row
}

$measurements = [Collections.Generic.List[object]]::new()
$seenCaptures = @{}
foreach ($captureDirectory in $CaptureDirectories) {
    $resolvedCapture = (Resolve-Path -LiteralPath $captureDirectory).Path
    if ($seenCaptures.ContainsKey($resolvedCapture)) {
        throw "动态阶跃采集目录重复：$resolvedCapture"
    }
    $seenCaptures[$resolvedCapture] = $true
    $summaryPath = Join-Path $resolvedCapture 'g2_dynamic_step_summary.json'
    if (-not (Test-Path -LiteralPath $summaryPath -PathType Leaf)) {
        throw "动态阶跃单次摘要不存在：$summaryPath"
    }
    $summary =
        Get-Content -LiteralPath $summaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    $experimentId = [string]$summary.experiment_id
    if (-not $planByExperiment.ContainsKey($experimentId)) {
        throw "单次摘要不属于当前批次：$experimentId"
    }
    $plan = $planByExperiment[$experimentId]
    if ([string]$summary.dynamics.direction -cne [string]$plan.direction -or
        [int]$summary.dynamics.peak_pwm -ne [int]$plan.peak_pwm) {
        throw "单次摘要与批次计划工况不一致：$experimentId"
    }
    $measurements.Add([pscustomobject][ordered]@{
        experiment_id = $experimentId
        direction = [string]$plan.direction
        peak_pwm = [int]$plan.peak_pwm
        repetition = [int]$plan.repetition
        capture_directory = $resolvedCapture
        accepted = [bool]$summary.accepted
        peak_window_wheel_rpm =
            [double]$summary.dynamics.peak_window_wheel_rpm
        signed_total_displacement_counts =
            [int64]$summary.dynamics.signed_total_displacement_counts
        motion_threshold_delay_ms =
            [int]$summary.dynamics.motion_threshold_delay_ms
        active_battery_minimum_mv =
            [uint32]$summary.dynamics.active_battery_minimum_mv
    })
}
if ($measurements.Count -ne $planByExperiment.Count) {
    throw '动态阶跃批次存在未绑定采集结果的计划'
}

$measurement = Measure-G2DynamicStepBatch `
    -Rows $measurements.ToArray() `
    -RequiredRepeatCount $RequiredRepeatCount `
    -PassCvPercent $PassCvPercent `
    -RetestCvPercent $RetestCvPercent

$summaryPath = Join-Path $resolvedOutput 'g2_dynamic_batch_summary.json'
$groupsPath = Join-Path $resolvedOutput 'g2_dynamic_batch_groups.csv'
$result = [ordered]@{
    schema_version = 1
    source_batch_directory = $resolvedBatch
    source_firmware_commit = [string]$batchManifest.firmware_commit
    source_repository_commit = [string]$batchManifest.repository_commit
    required_repeat_count = $RequiredRepeatCount
    pass_cv_percent = $PassCvPercent
    retest_cv_percent = $RetestCvPercent
    measurements = $measurements.ToArray()
    measurement = $measurement
}
[IO.File]::WriteAllText(
    $summaryPath,
    ($result | ConvertTo-Json -Depth 10),
    [Text.UTF8Encoding]::new($false))
$measurement.groups |
    Select-Object direction,
        peak_pwm,
        repeat_count,
        all_captures_accepted,
        @{ Name = 'mean_peak_window_wheel_rpm'
           Expression = { $_.peak_window_wheel_rpm.mean } },
        @{ Name = 'sample_standard_deviation_rpm'
           Expression = {
               $_.peak_window_wheel_rpm.sample_standard_deviation
           } },
        @{ Name = 'coefficient_of_variation_percent'
           Expression = {
               $_.peak_window_wheel_rpm.coefficient_of_variation_percent
           } },
        @{ Name = 'maximum_single_deviation_percent'
           Expression = {
               $_.peak_window_wheel_rpm.maximum_single_deviation_percent
           } },
        minimum_battery_mv,
        screening |
    Export-Csv -LiteralPath $groupsPath -NoTypeInformation -Encoding UTF8

[pscustomobject]@{
    SummaryPath = $summaryPath
    GroupsPath = $groupsPath
    CaptureCount = $measurement.capture_count
    CaptureEvidenceAccepted = $measurement.capture_evidence_accepted
    RepeatabilityAccepted = $measurement.repeatability_accepted
    ModelReady = $false
}
