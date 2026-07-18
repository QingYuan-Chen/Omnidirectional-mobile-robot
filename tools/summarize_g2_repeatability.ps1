[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$ManifestPath,

    [string]$OutputDirectory = '',

    [ValidateRange(2, 20)]
    [int]$RequiredRepeatCount = 3,

    [ValidateRange(0.1, 100.0)]
    [double]$PassCvPercent = 5.0,

    [ValidateRange(0.1, 100.0)]
    [double]$RetestCvPercent = 10.0,

    [ValidateRange(0.1, 100.0)]
    [double]$MaximumSingleDeviationPercent = 10.0
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_experiment_lib.ps1')

$resolvedManifest = (Resolve-Path -LiteralPath $ManifestPath).Path
$manifestDirectory = Split-Path -Parent $resolvedManifest
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $manifestDirectory
} elseif ([IO.Path]::IsPathRooted($OutputDirectory)) {
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
} else {
    $resolvedOutput = [IO.Path]::GetFullPath((
        Join-Path (Get-Location).Path $OutputDirectory))
}
[void][IO.Directory]::CreateDirectory($resolvedOutput)

$manifestRows = @(Import-Csv -LiteralPath $resolvedManifest)
if ($manifestRows.Count -eq 0) {
    throw '重复性清单不能为空'
}
$manifestColumns = @(
    'capture_directory',
    'direction',
    'peak_pwm',
    'repeat_index'
)
foreach ($column in $manifestColumns) {
    if ($null -eq $manifestRows[0].PSObject.Properties[$column]) {
        throw "重复性清单缺少字段：$column"
    }
}

$measurements = [Collections.Generic.List[object]]::new()
$seenRepeats = @{}
foreach ($row in $manifestRows) {
    $direction = [string]$row.direction
    if ($direction -cne 'Positive' -and $direction -cne 'Negative') {
        throw "重复性清单包含非法方向：$direction"
    }
    [int]$peakPwm = 0
    [int]$repeatIndex = 0
    if (-not [int]::TryParse([string]$row.peak_pwm, [ref]$peakPwm) -or
        $peakPwm -lt 1 -or $peakPwm -gt 840) {
        throw "重复性清单包含非法 PWM：$($row.peak_pwm)"
    }
    if (-not [int]::TryParse(
        [string]$row.repeat_index,
        [ref]$repeatIndex) -or $repeatIndex -lt 1) {
        throw "重复性清单包含非法重复序号：$($row.repeat_index)"
    }
    $repeatKey = "$direction|$peakPwm|$repeatIndex"
    if ($seenRepeats.ContainsKey($repeatKey)) {
        throw "重复性清单存在重复工况序号：$repeatKey"
    }
    $seenRepeats[$repeatKey] = $true

    $capturePath = [string]$row.capture_directory
    if (-not [IO.Path]::IsPathRooted($capturePath)) {
        $capturePath = Join-Path $manifestDirectory $capturePath
    }
    $resolvedCapture = (Resolve-Path -LiteralPath $capturePath).Path
    $summaryPath = Join-Path $resolvedCapture 'g2_operating_point_summary.json'
    $summary = Get-Content -LiteralPath $summaryPath -Raw | ConvertFrom-Json
    if ([string]$summary.measurement.direction -cne $direction -or
        [int]$summary.measurement.peak_pwm -ne $peakPwm) {
        throw "采集摘要与清单工况不一致：$resolvedCapture"
    }

    $measurements.Add([pscustomobject][ordered]@{
        capture_directory = $resolvedCapture
        direction = $direction
        peak_pwm = $peakPwm
        repeat_index = $repeatIndex
        steady_count_rate_cps =
            [double]$summary.measurement.steady_window.signed_count_rate_cps
        wheel_rpm = [double]$summary.measurement.steady_window.wheel_rpm
        battery_minimum_mv =
            [uint32]$summary.measurement.battery_mv.active_minimum
        accepted = [bool]$summary.accepted
    })
}

$measurement = Measure-G2Repeatability `
    -Rows $measurements.ToArray() `
    -RequiredRepeatCount $RequiredRepeatCount `
    -PassCvPercent $PassCvPercent `
    -RetestCvPercent $RetestCvPercent `
    -MaximumSingleDeviationPercent $MaximumSingleDeviationPercent

$jsonPath = Join-Path $resolvedOutput 'g2_repeatability_summary.json'
$csvPath = Join-Path $resolvedOutput 'g2_repeatability_groups.csv'
$result = [ordered]@{
    schema_version = 1
    source_manifest = $resolvedManifest
    capture_count = $measurements.Count
    measurement = $measurement
}
$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
[IO.File]::WriteAllText(
    $jsonPath,
    ($result | ConvertTo-Json -Depth 8),
    $utf8WithoutBom)
$measurement.groups |
    Select-Object direction,
        peak_pwm,
        repeat_count,
        mean_steady_count_rate_cps,
        mean_wheel_rpm,
        sample_standard_deviation_rpm,
        coefficient_of_variation_percent,
        minimum_wheel_rpm,
        maximum_wheel_rpm,
        maximum_single_deviation_percent,
        minimum_battery_mv,
        screening |
    Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8

[pscustomobject]@{
    Summary = $jsonPath
    Groups = $csvPath
    CaptureCount = $measurements.Count
    AllGroupsPassed = [bool]$measurement.all_groups_passed
    AllDirectionsMonotonic = [bool]$measurement.all_directions_monotonic
    Accepted = [bool]$measurement.accepted
}
