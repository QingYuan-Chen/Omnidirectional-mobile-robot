[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')]
    [string]$BatchId,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[0-9a-fA-F]{7,40}$')]
    [string]$FirmwareCommit,

    [ValidateRange(1, 4294967275)]
    [uint32]$SequenceStart = 1,

    [string]$OutputRoot = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'g2_low_speed_diagnostic_lib.ps1')
. (Join-Path $PSScriptRoot 'serial_capture_lib.ps1')

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $repositoryRoot 'experiments\generated'
} elseif (-not [IO.Path]::IsPathRooted($OutputRoot)) {
    $OutputRoot = Join-Path (Get-Location).Path $OutputRoot
}
$resolvedOutputRoot = [IO.Path]::GetFullPath($OutputRoot)
[void][IO.Directory]::CreateDirectory($resolvedOutputRoot)
$batchDirectory = Join-Path $resolvedOutputRoot $BatchId
if (Test-Path -LiteralPath $batchDirectory) {
    throw "拒绝覆盖已有低速独立验证批次：$batchDirectory"
}
[void][IO.Directory]::CreateDirectory($batchDirectory)

$repositoryCommit = (& git -C $repositoryRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or
    $repositoryCommit -notmatch '^[0-9a-fA-F]{40}$') {
    throw '无法读取仓库提交'
}
$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
$rows = [Collections.Generic.List[object]]::new()
[uint64]$nextSequence = $SequenceStart
for ($repetition = 1; $repetition -le 3; $repetition++) {
    $experimentId = 'neg240_steady_validation_r{0}' -f $repetition
    $experimentDirectory = Join-Path $batchDirectory $experimentId
    [void][IO.Directory]::CreateDirectory($experimentDirectory)
    $schedule = @(New-G2LowSpeedValidationSchedule `
        -SequenceStart ([uint32]$nextSequence))
    foreach ($entry in $schedule) {
        Assert-SerialCaptureCommand `
            -Command ([string]$entry.command) `
            -AllowNonStatusCommands
    }

    $schedulePath =
        Join-Path $experimentDirectory 'command_schedule.pending.csv'
    $scheduleLines = [Collections.Generic.List[string]]::new()
    $scheduleLines.Add((
        ConvertTo-CaptureCsvLine -Values @('elapsed_ms', 'command')))
    foreach ($entry in $schedule) {
        $scheduleLines.Add((ConvertTo-CaptureCsvLine -Values @(
            $entry.elapsed_ms,
            $entry.command
        )))
    }
    [IO.File]::WriteAllLines($schedulePath, $scheduleLines, $utf8WithoutBom)

    $manifest = [ordered]@{
        schema_version = 1
        experiment_id = $experimentId
        experiment_type = 'g2_low_speed_steady_validation'
        execution_state = 'not_authorized'
        evidence_role = 'independent_validation'
        firmware_commit = $FirmwareCommit
        repository_commit = $repositoryCommit
        repetition = $repetition
        direction = 'Negative'
        peak_pwm = 240
        signed_peak_pwm = -240
        sequence_start = [uint32]$nextSequence
        capture_duration_seconds = 14
        controlled_conditions = [ordered]@{
            initial_wheel_mark = '0deg'
            approach_to_mark_from_same_physical_direction = $true
            overshoot_then_reverse_to_mark_forbidden = $true
            minimum_cooldown_before_plan_seconds = 60
            battery_preflight_minimum_mv = 11500
            battery_preflight_maximum_mv = 11750
            temperature_measurement_required = $false
            temperature_record_required = $false
            temperature_is_acceptance_gate = $false
        }
        schedule = [ordered]@{
            capture_start_ms = 750
            arm_ms = 800
            target_step_ms = 850
            stop_ms = 2300
            capture_stop_ms = 2750
            export_ms = 3250
            final_status_ms = 12000
            keepalive_interval_ms = 300
            expected_full_pwm_plateau_ms = 1211
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
            abort_batch_on_any_rejected_capture = $true
        }
        analysis = [ordered]@{
            tool = 'tools/analyze_g2_low_speed_diagnostic.ps1'
            fixed_tail_windows_ms = @(200, 300)
            stable_speed_cv_limit_percent = 5.0
            stable_speed_maximum_deviation_limit_percent = 10.0
            startup_delay_cv_limit_percent = 10.0
            original_short_window_gate_is_not_retroactively_replaced = $true
            model_ready_after_validation = $false
        }
        files = [ordered]@{
            command_schedule = 'command_schedule.pending.csv'
            manifest = 'experiment_manifest.json'
        }
    }
    $manifestPath = Join-Path $experimentDirectory 'experiment_manifest.json'
    [IO.File]::WriteAllText(
        $manifestPath,
        ($manifest | ConvertTo-Json -Depth 10),
        $utf8WithoutBom)
    $rows.Add([pscustomobject][ordered]@{
        execution_order = $repetition
        experiment_id = $experimentId
        repetition = $repetition
        sequence_start = [uint32]$nextSequence
        relative_plan_directory = $experimentId
        execution_state = 'not_authorized'
    })
    $nextSequence += [uint64]7
}

$csvPath = Join-Path $batchDirectory 'batch_manifest.csv'
$jsonPath = Join-Path $batchDirectory 'batch_manifest.json'
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding utf8
$batchManifest = [ordered]@{
    schema_version = 1
    batch_id = $BatchId
    batch_type = 'g2_low_speed_steady_validation_batch'
    execution_state = 'not_authorized'
    evidence_role = 'independent_validation'
    firmware_commit = $FirmwareCommit
    repository_commit = $repositoryCommit
    experiment_count = 3
    sequence_start = $SequenceStart
    next_unused_sequence = [uint32]$nextSequence
    exploratory_source_captures = @(
        'captures/20260719-152304066_COM6'
        'captures/20260719-152525386_COM6'
        'captures/20260719-152818633_COM6'
        'captures/20260719-154240198_COM6'
        'captures/20260719-154450829_COM6'
    )
    preregistered_decision = [ordered]@{
        stable_speed_requires_both_tail_windows = $true
        tail_200ms_cv_limit_percent = 5.0
        tail_300ms_cv_limit_percent = 5.0
        maximum_single_deviation_limit_percent = 10.0
        startup_delay_cv_limit_percent = 10.0
        maximum_repetitions = 3
        no_post_hoc_window_selection = $true
        original_short_window_gate_remains_recorded_as_failed = $true
        temperature_baseline_enabled = $false
    }
    safety = [ordered]@{
        execute_one_plan_at_a_time = $true
        pure_status_preflight_before_every_plan = $true
        explicit_operator_authorization_before_every_plan = $true
        wheels_must_be_suspended = $true
        emergency_power_disconnect_required = $true
        abort_batch_on_any_rejected_capture = $true
        minimum_cooldown_between_plans_seconds = 60
    }
    files = [ordered]@{
        batch_manifest_csv = 'batch_manifest.csv'
        batch_manifest_json = 'batch_manifest.json'
    }
}
[IO.File]::WriteAllText(
    $jsonPath,
    ($batchManifest | ConvertTo-Json -Depth 10),
    $utf8WithoutBom)

[pscustomobject]@{
    BatchDirectory = $batchDirectory
    CsvPath = $csvPath
    JsonPath = $jsonPath
    ExperimentCount = 3
    NextUnusedSequence = [uint32]$nextSequence
}
