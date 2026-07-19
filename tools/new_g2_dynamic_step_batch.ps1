[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$')]
    [string]$BatchId,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[0-9a-fA-F]{7,40}$')]
    [string]$FirmwareCommit,

    [int[]]$PeakPwms = @(400),

    [ValidateRange(3, 5)]
    [int]$Repetitions = 3,

    [ValidateRange(1, 4294967000)]
    [uint32]$SequenceStart = 1,

    [string]$OutputRoot = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'serial_capture_lib.ps1')

$allowedPeaks = @(240, 400, 600, 840)
$normalizedPeaks = @($PeakPwms | Sort-Object -Unique)
if ($normalizedPeaks.Count -eq 0 -or
    @($normalizedPeaks | Where-Object { $allowedPeaks -notcontains $_ }).Count -ne 0) {
    throw '动态阶跃批次只允许240/400/600/840 PWM'
}

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
    throw "拒绝覆盖已有动态阶跃批次：$batchDirectory"
}
[void][IO.Directory]::CreateDirectory($batchDirectory)

$rows = [Collections.Generic.List[object]]::new()
[uint64]$nextSequence = $SequenceStart
$order = 0
for ($repetition = 1; $repetition -le $Repetitions; $repetition++) {
    foreach ($peak in $normalizedPeaks) {
        $directions = if (($repetition % 2) -eq 1) {
            @('Positive', 'Negative')
        } else {
            @('Negative', 'Positive')
        }
        foreach ($direction in $directions) {
            if ($nextSequence + [uint64]5 -gt [uint64][uint32]::MaxValue) {
                throw '动态阶跃批次命令序号超过uint32范围'
            }
            $order++
            $directionSlug = if ($direction -ceq 'Positive') { 'pos' } else { 'neg' }
            $experimentId = 'pwm{0}_r{1}_{2}' -f $peak, $repetition, $directionSlug
            $plan = & (Join-Path $PSScriptRoot 'new_g2_dynamic_step_plan.ps1') `
                -ExperimentId $experimentId `
                -FirmwareCommit $FirmwareCommit `
                -Direction $direction `
                -PeakPwm $peak `
                -SequenceStart ([uint32]$nextSequence) `
                -OutputRoot $batchDirectory
            $rows.Add([pscustomobject][ordered]@{
                execution_order = $order
                experiment_id = $experimentId
                repetition = $repetition
                direction = $direction
                peak_pwm = $peak
                sequence_start = [uint32]$nextSequence
                relative_plan_directory = $experimentId
                capture_duration_seconds = $plan.CaptureDurationSeconds
                execution_state = 'not_authorized'
            })
            $nextSequence += [uint64]6
        }
    }
}

$repositoryCommit = (& git -C $repositoryRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $repositoryCommit -notmatch '^[0-9a-fA-F]{40}$') {
    throw '无法读取仓库提交'
}
$csvPath = Join-Path $batchDirectory 'batch_manifest.csv'
$jsonPath = Join-Path $batchDirectory 'batch_manifest.json'
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding utf8
$manifest = [ordered]@{
    schema_version = 1
    batch_id = $BatchId
    batch_type = 'g2_dynamic_bounded_step_batch'
    execution_state = 'not_authorized'
    firmware_commit = $FirmwareCommit
    repository_commit = $repositoryCommit
    peak_pwms = $normalizedPeaks
    repetitions = $Repetitions
    experiment_count = $rows.Count
    sequence_start = $SequenceStart
    next_unused_sequence = [uint32]$nextSequence
    ordering = '奇数轮正向先行，偶数轮反向先行，以降低固定方向顺序偏差。'
    safety = [ordered]@{
        execute_one_plan_at_a_time = $true
        pure_status_preflight_before_every_plan = $true
        explicit_operator_authorization_before_every_plan = $true
        wheels_must_be_suspended = $true
        emergency_power_disconnect_required = $true
        abort_batch_on_any_rejected_capture = $true
        minimum_cooldown_between_plans_seconds = 30
    }
    analysis = [ordered]@{
        per_capture_tool = 'tools/analyze_g2_dynamic_step.ps1'
        minimum_accepted_repetitions_per_direction_and_peak = 3
        model_ready_after_batch = $false
    }
    files = [ordered]@{
        batch_manifest_csv = 'batch_manifest.csv'
        batch_manifest_json = 'batch_manifest.json'
    }
}
[IO.File]::WriteAllText(
    $jsonPath,
    ($manifest | ConvertTo-Json -Depth 8),
    [Text.UTF8Encoding]::new($false))

[pscustomobject]@{
    BatchDirectory = $batchDirectory
    CsvPath = $csvPath
    JsonPath = $jsonPath
    ExperimentCount = $rows.Count
    NextUnusedSequence = [uint32]$nextSequence
}
