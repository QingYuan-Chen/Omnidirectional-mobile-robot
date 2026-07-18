[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]{2,63}$')]
    [string]$ExperimentId,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[0-9a-fA-F]{7,40}$')]
    [string]$FirmwareCommit,

    [Parameter(Mandatory = $true)]
    [ValidateSet('Positive', 'Negative')]
    [string]$Direction,

    [Parameter(Mandatory = $true)]
    [ValidateRange(1, 840)]
    [int]$PeakPwm,

    [ValidateRange(250, 10000)]
    [int]$HoldMilliseconds = 1000,

    [ValidateRange(50, 400)]
    [int]$KeepAliveMilliseconds = 250,

    [ValidateRange(1, 4294967295)]
    [uint32]$SequenceStart = 1,

    [string]$OutputRoot = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_experiment_lib.ps1')
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

$schedule = @(New-G2DeadzoneProbeSchedule `
    -Direction $Direction `
    -PeakPwm $PeakPwm `
    -HoldMilliseconds $HoldMilliseconds `
    -KeepAliveMilliseconds $KeepAliveMilliseconds `
    -SequenceStart $SequenceStart)
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

$lastScheduleMs = [uint64]$schedule[-1].elapsed_ms
$captureDurationSeconds = [int][Math]::Ceiling(([double]$lastScheduleMs + 500.0) / 1000.0)
$manifest = [ordered]@{
    schema_version = 1
    experiment_id = $ExperimentId
    experiment_type = 'g2_deadzone_probe'
    generated_utc = [DateTimeOffset]::UtcNow.ToString('O')
    execution_state = 'not_authorized'
    requires_motor_power = $true
    firmware_commit = $FirmwareCommit.ToLowerInvariant()
    motor_channel = 'MA'
    direction = $Direction
    requested_peak_pwm = $PeakPwm
    deadzone_probe_pwm_cap = 840
    firmware_open_loop_pwm_cap = 840
    firmware_pwm_ramp_counts_per_ms = 1
    firmware_command_timeout_ms = 500
    keepalive_ms = $KeepAliveMilliseconds
    hold_ms = $HoldMilliseconds
    recommended_capture_duration_seconds = $captureDurationSeconds
    schedule_file = 'command_schedule.pending.csv'
    safety_checks = [ordered]@{
        schedule_generation_opens_serial_port = $false
        motor_power_connected_and_voltage_verified = $false
        ma_only_motion_path_verified = $false
        wheel_suspended_and_clear = $false
        emergency_power_disconnect_ready = $false
        operator_execution_authorized = $false
    }
    notes = @(
        'This directory only prepares one bounded PWM command plan and does not open a serial port.',
        'Do not execute while execution_state is not_authorized.',
        'ESTOP is intentionally absent because it requires a system reset.'
    )
}
[IO.File]::WriteAllText(
    $manifestPath,
    ($manifest | ConvertTo-Json -Depth 6),
    $utf8WithoutBom)

[pscustomobject]@{
    ExperimentDirectory = $experimentDirectory
    Manifest = $manifestPath
    PendingSchedule = $schedulePath
    CommandCount = $schedule.Count
    RecommendedCaptureDurationSeconds = $captureDurationSeconds
    ExecutionState = 'not_authorized'
}
