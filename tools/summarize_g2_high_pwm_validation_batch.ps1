[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$BatchDirectory,

    [Parameter(Mandatory = $true)]
    [string[]]$CaptureDirectories,

    [string]$OutputDirectory = '',

    [ValidateRange(3, 3)]
    [int]$RequiredRepeatCount = 3,

    [ValidateRange(0.1, 100.0)]
    [double]$StableSpeedCvLimitPercent = 5.0,

    [ValidateRange(0.1, 100.0)]
    [double]$StableSpeedDeviationLimitPercent = 10.0
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_high_pwm_validation_lib.ps1')

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
        throw "840稳态验证批次清单不存在：$path"
    }
}
$batchRows = @(Import-Csv -LiteralPath $batchManifestPath)
$batchManifest =
    Get-Content -LiteralPath $batchManifestJsonPath -Raw -Encoding UTF8 |
        ConvertFrom-Json
if ($batchManifest.batch_type -cne
    'g2_high_pwm_steady_validation_batch') {
    throw '目录不是840稳态验证批次'
}
if ($CaptureDirectories.Count -ne $batchRows.Count) {
    throw '采集目录数量必须与840稳态验证计划数量一致'
}

$planByExperiment = @{}
foreach ($row in $batchRows) {
    $experimentId = [string]$row.experiment_id
    if ($planByExperiment.ContainsKey($experimentId)) {
        throw "840稳态验证实验ID重复：$experimentId"
    }
    $planByExperiment[$experimentId] = $row
}

$measurements = [Collections.Generic.List[object]]::new()
$seenCaptures = @{}
$seenExperiments = @{}
foreach ($captureDirectory in $CaptureDirectories) {
    $resolvedCapture = (Resolve-Path -LiteralPath $captureDirectory).Path
    if ($seenCaptures.ContainsKey($resolvedCapture)) {
        throw "840稳态验证采集目录重复：$resolvedCapture"
    }
    $seenCaptures[$resolvedCapture] = $true
    $summaryPath = Join-Path $resolvedCapture 'g2_dynamic_step_summary.json'
    if (-not (Test-Path -LiteralPath $summaryPath -PathType Leaf)) {
        throw "840稳态验证单次摘要不存在：$summaryPath"
    }
    $summary =
        Get-Content -LiteralPath $summaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    $experimentId = [string]$summary.experiment_id
    if (-not $planByExperiment.ContainsKey($experimentId)) {
        throw "单次摘要不属于当前840批次：$experimentId"
    }
    if ($seenExperiments.ContainsKey($experimentId)) {
        throw "840稳态验证实验结果重复：$experimentId"
    }
    $seenExperiments[$experimentId] = $true
    $plan = $planByExperiment[$experimentId]
    $expectedPlanDirectory = (Resolve-Path -LiteralPath (
        Join-Path $resolvedBatch ([string]$plan.relative_plan_directory)
    )).Path
    $actualPlanDirectory = (Resolve-Path -LiteralPath (
        [string]$summary.source_plan_directory
    )).Path
    if ([string]$summary.experiment_type -cne
        'g2_high_pwm_steady_validation' -or
        [string]$summary.dynamics.direction -cne [string]$plan.direction -or
        [int]$summary.dynamics.peak_pwm -ne 840 -or
        $actualPlanDirectory -cne $expectedPlanDirectory) {
        throw "840单次摘要与批次计划不一致：$experimentId"
    }
    if ($summary.PSObject.Properties.Name -notcontains
        'high_pwm_diagnostic' -or $null -eq $summary.high_pwm_diagnostic) {
        throw "840单次摘要缺少稳态双窗口：$experimentId"
    }
    $measurements.Add([pscustomobject][ordered]@{
        experiment_id = $experimentId
        direction = [string]$plan.direction
        peak_pwm = 840
        repetition = [int]$plan.repetition
        capture_directory = $resolvedCapture
        accepted = [bool]$summary.accepted
        tail_200ms_wheel_rpm =
            [double]$summary.high_pwm_diagnostic.tail_200ms_wheel_rpm
        tail_300ms_wheel_rpm =
            [double]$summary.high_pwm_diagnostic.tail_300ms_wheel_rpm
        active_battery_minimum_mv =
            [uint32]$summary.dynamics.active_battery_minimum_mv
    })
}
if ($measurements.Count -ne $planByExperiment.Count) {
    throw '840稳态验证批次存在未绑定采集结果的计划'
}

$measurement = Measure-G2HighPwmValidationBatch `
    -Rows $measurements.ToArray() `
    -RequiredRepeatCount $RequiredRepeatCount `
    -StableSpeedCvLimitPercent $StableSpeedCvLimitPercent `
    -StableSpeedDeviationLimitPercent $StableSpeedDeviationLimitPercent

$summaryPath = Join-Path $resolvedOutput 'g2_high_pwm_batch_summary.json'
$groupsPath = Join-Path $resolvedOutput 'g2_high_pwm_batch_groups.csv'
$result = [ordered]@{
    schema_version = 1
    source_batch_directory = $resolvedBatch
    source_firmware_commit = [string]$batchManifest.firmware_commit
    source_repository_commit = [string]$batchManifest.repository_commit
    required_repeat_count = $RequiredRepeatCount
    stable_speed_cv_limit_percent = $StableSpeedCvLimitPercent
    stable_speed_deviation_limit_percent =
        $StableSpeedDeviationLimitPercent
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
        enough_repeats,
        @{ Name = 'tail_200ms_mean_rpm'
           Expression = { $_.tail_200ms_wheel_rpm.mean } },
        @{ Name = 'tail_200ms_cv_percent'
           Expression = {
               $_.tail_200ms_wheel_rpm.coefficient_of_variation_percent
           } },
        @{ Name = 'tail_300ms_mean_rpm'
           Expression = { $_.tail_300ms_wheel_rpm.mean } },
        @{ Name = 'tail_300ms_cv_percent'
           Expression = {
               $_.tail_300ms_wheel_rpm.coefficient_of_variation_percent
           } },
        stable_speed_repeatability_accepted |
    Export-Csv -LiteralPath $groupsPath -NoTypeInformation -Encoding UTF8

[pscustomobject]@{
    SummaryPath = $summaryPath
    GroupsPath = $groupsPath
    CaptureCount = $measurement.capture_count
    CaptureEvidenceAccepted = $measurement.capture_evidence_accepted
    RepeatabilityAccepted = $measurement.repeatability_accepted
    ModelReady = $false
}
