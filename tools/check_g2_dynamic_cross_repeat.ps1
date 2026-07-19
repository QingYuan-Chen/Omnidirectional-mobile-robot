[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CurrentCaptureDirectory,

    [Parameter(Mandatory = $true)]
    [string[]]$PreviousCaptureDirectories,

    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_dynamic_step_lib.ps1')

function Read-G2DynamicGateRow {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CaptureDirectory
    )

    $resolvedCapture = (Resolve-Path -LiteralPath $CaptureDirectory).Path
    $summaryPath = Join-Path $resolvedCapture 'g2_dynamic_step_summary.json'
    if (-not (Test-Path -LiteralPath $summaryPath -PathType Leaf)) {
        throw "动态阶跃摘要不存在：$summaryPath"
    }
    $summary =
        Get-Content -LiteralPath $summaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    return [pscustomobject][ordered]@{
        experiment_id = [string]$summary.experiment_id
        direction = [string]$summary.dynamics.direction
        peak_pwm = [int]$summary.dynamics.peak_pwm
        capture_directory = $resolvedCapture
        accepted = [bool]$summary.accepted
        peak_window_wheel_rpm =
            [double]$summary.dynamics.peak_window_wheel_rpm
        signed_total_displacement_counts =
            [int64]$summary.dynamics.signed_total_displacement_counts
        motion_threshold_delay_ms =
            [int]$summary.dynamics.motion_threshold_delay_ms
    }
}

$current = Read-G2DynamicGateRow `
    -CaptureDirectory $CurrentCaptureDirectory
$previousRows = @($PreviousCaptureDirectories | ForEach-Object {
    Read-G2DynamicGateRow -CaptureDirectory $_
})
$measurement = Measure-G2DynamicCrossRepeatGate `
    -PreviousRows $previousRows `
    -CurrentRow $current

if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $current.capture_directory
} elseif ([IO.Path]::IsPathRooted($OutputDirectory)) {
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
} else {
    $resolvedOutput = [IO.Path]::GetFullPath((
        Join-Path (Get-Location).Path $OutputDirectory))
}
[void][IO.Directory]::CreateDirectory($resolvedOutput)
$summaryPath = Join-Path $resolvedOutput 'g2_dynamic_cross_repeat_gate.json'
[IO.File]::WriteAllText(
    $summaryPath,
    ($measurement | ConvertTo-Json -Depth 8),
    [Text.UTF8Encoding]::new($false))

[pscustomobject]@{
    Evaluated = $measurement.evaluated
    Passed = $measurement.passed
    SummaryPath = $summaryPath
}
