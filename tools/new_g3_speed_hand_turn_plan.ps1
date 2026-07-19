param(
    [string]$OutputDirectory,
    [string]$FirmwareCommit
)

$ErrorActionPreference = 'Stop'
$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$repositoryCommit = (& git -C $repositoryRoot rev-parse HEAD 2>$null)
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($repositoryCommit)) {
    throw '无法读取当前仓库提交'
}
$blockingChanges = @(
    & git -C $repositoryRoot status --porcelain --untracked-files=all 2>$null |
        Where-Object { $_ -notmatch '^\?\? tmp[\\/]' }
)
if ($blockingChanges.Count -gt 0) {
    throw '生成正式手转计划前仓库必须干净'
}
if ([string]::IsNullOrWhiteSpace($FirmwareCommit)) {
    $FirmwareCommit = $repositoryCommit
}
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $repositoryRoot 'experiments\generated\g3_speed_hand_turn'
}
[void][IO.Directory]::CreateDirectory($OutputDirectory)

$commands = @(
    [pscustomobject]@{ elapsed_ms = 0; command = 'STATUS' },
    [pscustomobject]@{ elapsed_ms = 500; command = 'CAPTURE SPEED STATUS' },
    [pscustomobject]@{ elapsed_ms = 1000; command = 'CAPTURE SPEED START' },
    [pscustomobject]@{ elapsed_ms = 3000; command = 'CAPTURE SPEED STOP' },
    [pscustomobject]@{ elapsed_ms = 3250; command = 'CAPTURE SPEED STATUS' },
    [pscustomobject]@{ elapsed_ms = 3500; command = 'CAPTURE SPEED EXPORT' },
    [pscustomobject]@{ elapsed_ms = 11000; command = 'STATUS' }
)
$schedulePath = Join-Path $OutputDirectory 'commands.csv'
$commands | Export-Csv -LiteralPath $schedulePath -NoTypeInformation -Encoding utf8

$manifest = [pscustomobject][ordered]@{
    plan_type = 'g3_speed_hand_turn_validation'
    execution_state = 'not_authorized'
    route_stage = 7
    requires_firmware_flash = $true
    requires_board_connection = $true
    requires_motor_power = $false
    firmware_commit = $FirmwareCommit.ToLowerInvariant()
    repository_commit = $repositoryCommit
    duration_seconds = 12
    expected_wheel_turns = 1.0
    operator_window_ms = [ordered]@{
        capture_start = 1000
        begin_one_direction_turn_immediately_after = 1000
        complete_turn_before = 2850
        capture_stop = 3000
    }
    candidate_events_per_wheel_revolution = 30720
    event_count_tolerance_percent = 5.0
    safety = [ordered]@{
        independent_status_required_before_plan = $true
        motor_power_must_remain_disconnected = $true
        target_pwm_must_be_zero = $true
        applied_pwm_must_be_zero = $true
        motor_state_must_be_disarmed = $true
        ready_required = $true
        faults_estop_and_all_errors_must_be_zero = $true
        turn_exactly_one_marked_revolution_in_one_direction = $true
        reverse_or_overshoot_correction_forbidden = $true
    }
    analyzer = 'tools/analyze_g3_speed_capture.ps1'
    command_schedule = 'commands.csv'
}
$manifestPath = Join-Path $OutputDirectory 'plan_manifest.json'
[IO.File]::WriteAllText(
    $manifestPath,
    ($manifest | ConvertTo-Json -Depth 8),
    [Text.UTF8Encoding]::new($false))
$manifest
