[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')]
    [string]$ExperimentId,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[0-9a-fA-F]{7,40}$')]
    [string]$FirmwareCommit,

    [Parameter(Mandatory = $true)]
    [ValidateSet('Positive', 'Negative')]
    [string]$Direction,

    [Parameter(Mandatory = $true)]
    [ValidateSet(240, 400, 600, 840)]
    [int]$PeakPwm,

    [ValidateRange(1, 4294967290)]
    [uint32]$SequenceStart = 1,

    [string]$OutputRoot = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_dynamic_step_lib.ps1')
. (Join-Path $PSScriptRoot 'serial_capture_lib.ps1')

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $repositoryRoot 'experiments\generated'
} elseif (-not [IO.Path]::IsPathRooted($OutputRoot)) {
    $OutputRoot = Join-Path (Get-Location).Path $OutputRoot
}
$resolvedOutputRoot = [IO.Path]::GetFullPath($OutputRoot)
[void][IO.Directory]::CreateDirectory($resolvedOutputRoot)
$experimentDirectory = Join-Path $resolvedOutputRoot $ExperimentId
if (Test-Path -LiteralPath $experimentDirectory) {
    throw "拒绝覆盖已有实验目录：$experimentDirectory"
}
[void][IO.Directory]::CreateDirectory($experimentDirectory)

$schedule = @(New-G2DynamicStepSchedule `
    -Direction $Direction `
    -PeakPwm $PeakPwm `
    -SequenceStart $SequenceStart)
foreach ($entry in $schedule) {
    Assert-SerialCaptureCommand -Command $entry.command -AllowNonStatusCommands
}

$schedulePath = Join-Path $experimentDirectory 'command_schedule.pending.csv'
$manifestPath = Join-Path $experimentDirectory 'experiment_manifest.json'
$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
$scheduleLines = [Collections.Generic.List[string]]::new()
$scheduleLines.Add((ConvertTo-CaptureCsvLine -Values @('elapsed_ms', 'command')))
foreach ($entry in $schedule) {
    $scheduleLines.Add((ConvertTo-CaptureCsvLine -Values @(
        $entry.elapsed_ms,
        $entry.command
    )))
}
[IO.File]::WriteAllLines($schedulePath, $scheduleLines, $utf8WithoutBom)

$repositoryCommit = (& git -C $repositoryRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $repositoryCommit -notmatch '^[0-9a-fA-F]{40}$') {
    throw '无法读取仓库提交'
}
$signedPwm = if ($Direction -ceq 'Positive') { $PeakPwm } else { -$PeakPwm }
$manifest = [ordered]@{
    schema_version = 1
    experiment_id = $ExperimentId
    experiment_type = 'g2_dynamic_bounded_step'
    execution_state = 'not_authorized'
    firmware_commit = $FirmwareCommit
    repository_commit = $repositoryCommit
    direction = $Direction
    peak_pwm = $PeakPwm
    signed_peak_pwm = $signedPwm
    sequence_start = $SequenceStart
    capture_duration_seconds = 14
    schedule = [ordered]@{
        capture_start_ms = 750
        arm_ms = 800
        target_step_ms = 850
        stop_ms = 1800
        capture_stop_ms = 2750
        export_ms = 3250
        final_status_ms = 12000
        keepalive_interval_ms = 300
    }
    safety = [ordered]@{
        motor = 'MA'
        motor_power_required = $true
        wheels_must_be_suspended = $true
        other_motors_required_coast = $true
        pure_status_preflight_required = $true
        explicit_operator_authorization_required = $true
        emergency_power_disconnect_required = $true
        estop_present = $false
        reversal_present = $false
        target_is_bounded_by_1_count_per_ms_ramp = $true
    }
    analysis = [ordered]@{
        tool = 'tools/analyze_g2_dynamic_step.ps1'
        counts_per_wheel_revolution = 122880
        minimum_battery_mv = 10500
        motion_threshold_counts = 1000
        other_channel_limit_counts = 1000
        model_ready_after_single_capture = $false
    }
    files = [ordered]@{
        command_schedule = 'command_schedule.pending.csv'
        manifest = 'experiment_manifest.json'
    }
}
[IO.File]::WriteAllText(
    $manifestPath,
    ($manifest | ConvertTo-Json -Depth 8),
    $utf8WithoutBom)

[pscustomobject]@{
    ExperimentDirectory = $experimentDirectory
    SchedulePath = $schedulePath
    ManifestPath = $manifestPath
    CaptureDurationSeconds = 14
}
