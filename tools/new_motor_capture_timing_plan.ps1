[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[0-9a-fA-F]{7,40}$')]
    [string]$FirmwareCommit,

    [string]$OutputRoot = '',

    [ValidateRange(1000, 2100)]
    [int]$RecordDurationMilliseconds = 2000
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'serial_capture_lib.ps1')

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $repositoryRoot 'experiments\motor_capture'
} elseif (-not [IO.Path]::IsPathRooted($OutputRoot)) {
    $OutputRoot = Join-Path (Get-Location).Path $OutputRoot
}
$resolvedOutputRoot = [IO.Path]::GetFullPath($OutputRoot)
[void][IO.Directory]::CreateDirectory($resolvedOutputRoot)

$captureStartMs = 750
$captureStopMs = $captureStartMs + $RecordDurationMilliseconds
$exportStartMs = $captureStopMs + 500
$finalStatusMs = 18000
$captureDurationSeconds = 20

$schedule = @(
    [pscustomobject]@{ elapsed_ms = 0; command = 'STATUS' },
    [pscustomobject]@{ elapsed_ms = 500; command = 'CAPTURE STATUS' },
    [pscustomobject]@{ elapsed_ms = $captureStartMs; command = 'CAPTURE START' },
    [pscustomobject]@{ elapsed_ms = $captureStopMs; command = 'CAPTURE STOP' },
    [pscustomobject]@{ elapsed_ms = $captureStopMs + 250; command = 'CAPTURE STATUS' },
    [pscustomobject]@{ elapsed_ms = $exportStartMs; command = 'CAPTURE EXPORT' },
    [pscustomobject]@{ elapsed_ms = $finalStatusMs; command = 'STATUS' }
)
foreach ($entry in $schedule) {
    Assert-SerialCaptureCommand -Command $entry.command -AllowNonStatusCommands
}

$name = 'timing_{0}' -f (Get-Date -Format 'yyyyMMdd-HHmmssfff')
$planDirectory = Join-Path $resolvedOutputRoot $name
[void][IO.Directory]::CreateDirectory($planDirectory)
$schedulePath = Join-Path $planDirectory 'command_schedule.pending.csv'
$manifestPath = Join-Path $planDirectory 'plan_manifest.json'
$utf8WithoutBom = [Text.UTF8Encoding]::new($false)

$scheduleLines = [Collections.Generic.List[string]]::new()
$scheduleLines.Add((ConvertTo-CaptureCsvLine -Values @('elapsed_ms', 'command')))
foreach ($entry in $schedule) {
    $scheduleLines.Add(
        (ConvertTo-CaptureCsvLine -Values @($entry.elapsed_ms, $entry.command)))
}
[IO.File]::WriteAllLines($schedulePath, $scheduleLines, $utf8WithoutBom)

$manifest = [ordered]@{
    schema_version = 1
    plan_type = 'motor_capture_no_power_timing'
    execution_state = 'not_authorized'
    firmware_commit = $FirmwareCommit.ToLowerInvariant()
    repository_commit = (& git -C $repositoryRoot rev-parse HEAD 2>$null)
    created_utc = [DateTimeOffset]::UtcNow.ToString('O')
    serial = [ordered]@{
        baud_rate = 230400
        data_bits = 8
        parity = 'None'
        stop_bits = 1
    }
    capture = [ordered]@{
        requested_duration_seconds = $captureDurationSeconds
        record_duration_ms = $RecordDurationMilliseconds
        expected_sample_count_approximately = $RecordDurationMilliseconds
        capacity_samples = 2200
        motor_power_required = $false
        motor_commands_present = $false
    }
    safety = [ordered]@{
        pure_status_preflight_required = $true
        motor_power_must_be_off = $true
        arm_pwm_stop_estop_forbidden = $true
        explicit_operator_authorization_required = $true
    }
    execution = [ordered]@{
        tool = 'tools/capture_serial.ps1'
        schedule_file = 'command_schedule.pending.csv'
        required_switch = '-AllowNonStatusCommands'
        analyzer = 'tools/analyze_motor_capture.ps1'
    }
}
[IO.File]::WriteAllText(
    $manifestPath,
    ($manifest | ConvertTo-Json -Depth 8),
    $utf8WithoutBom)

[pscustomobject]@{
    PlanDirectory = $planDirectory
    SchedulePath = $schedulePath
    ManifestPath = $manifestPath
    ExecutionState = 'not_authorized'
}
