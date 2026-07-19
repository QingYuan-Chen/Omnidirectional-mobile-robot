$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\tools\g2_high_pwm_validation_lib.ps1')

$script:assertionCount = 0
function Assert-True {
    param(
        [Parameter(Mandatory = $true)]
        [bool]$Condition,

        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    $script:assertionCount++
    if (-not $Condition) {
        throw "断言失败：$Message"
    }
}

function Assert-Throws {
    param(
        [Parameter(Mandatory = $true)]
        [scriptblock]$Action,

        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    $threw = $false
    try {
        & $Action
    } catch {
        $threw = $true
    }
    Assert-True $threw $Message
}

$temporaryDirectory = Join-Path ([IO.Path]::GetTempPath()) (
    'g2-high-pwm-validation-' + [guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporaryDirectory)

try {
    $positiveSchedule = @(New-G2HighPwmValidationSchedule `
        -Direction Positive `
        -SequenceStart 4486)
    $negativeSchedule = @(New-G2HighPwmValidationSchedule `
        -Direction Negative `
        -SequenceStart 4492)
    Assert-True ($positiveSchedule.Count -eq 13 -and
                 $negativeSchedule.Count -eq 13) `
        '正反840计划均固定13条命令'
    Assert-True (
        $positiveSchedule[3].command -ceq 'ARM 4486' -and
        $positiveSchedule[4].command -ceq 'PWM 4487 840' -and
        $positiveSchedule[8].command -ceq 'STOP 4491') `
        '正向840计划使用连续六个运动序号'
    Assert-True (
        $negativeSchedule[4].command -ceq 'PWM 4493 -840' -and
        $negativeSchedule[8].command -ceq 'STOP 4497') `
        '反向840计划固定负向幅值和连续序号'
    Assert-True (
        @($positiveSchedule | Where-Object {
            $_.elapsed_ms -eq 1990 -and $_.command -like 'STOP *'
        }).Count -eq 1 -and
        @($positiveSchedule | Where-Object {
            $_.elapsed_ms -eq 2900 -and
            $_.command -ceq 'CAPTURE STOP'
        }).Count -eq 1) `
        '840计划固定301ms平台与2150样本时序'
    Assert-True (
        Test-G2HighPwmValidationSchedule `
            -Schedule $positiveSchedule `
            -Direction Positive) `
        '正向840计划通过严格结构检查'

    $invalidSchedule = @($positiveSchedule | ForEach-Object {
        [pscustomobject]@{
            elapsed_ms = $_.elapsed_ms
            command = $_.command
        }
    })
    $invalidSchedule[8].elapsed_ms = 1800
    Assert-Throws {
        Test-G2HighPwmValidationSchedule `
            -Schedule $invalidSchedule `
            -Direction Positive
    } '旧1800ms STOP计划不能冒充840稳态验证'

    $planRoot = Join-Path $temporaryDirectory 'plans'
    $plan = & (
        Join-Path $PSScriptRoot '..\tools\new_g2_high_pwm_validation_plan.ps1'
    ) `
        -ExperimentId pwm840_steady_r1_pos `
        -FirmwareCommit cc1b43a `
        -Direction Positive `
        -SequenceStart 4486 `
        -OutputRoot $planRoot
    $manifest =
        Get-Content -LiteralPath $plan.ManifestPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    $plannedCommands = @(Import-Csv -LiteralPath $plan.SchedulePath)
    Assert-True (
        $manifest.experiment_type -ceq
            'g2_high_pwm_steady_validation' -and
        $manifest.execution_state -ceq 'not_authorized' -and
        $manifest.peak_pwm -eq 840 -and
        $manifest.analysis.minimum_battery_mv -eq 11300 -and
        $manifest.analysis.minimum_full_pwm_plateau_ms -eq 300) `
        '840单计划冻结试验类型、未授权状态、动力和300ms门'
    $profile = Get-G2SingleCaptureAnalysisProfile `
        -Manifest $manifest `
        -Schedule $plannedCommands
    Assert-True (
        $profile.high_pwm_steady_validation -and
        -not $profile.low_speed_steady_validation -and
        $profile.minimum_full_pwm_plateau_ms -eq 300) `
        '单次严格分析识别840稳态验证配置'
    Assert-Throws {
        & (
            Join-Path $PSScriptRoot `
                '..\tools\new_g2_high_pwm_validation_plan.ps1'
        ) `
            -ExperimentId pwm840_steady_r1_pos `
            -FirmwareCommit cc1b43a `
            -Direction Positive `
            -SequenceStart 4486 `
            -OutputRoot $planRoot
    } '840单计划拒绝覆盖已有目录'

    $sampleCount = 2150
    $samples = for ($index = 0; $index -lt $sampleCount; $index++) {
        $applied = 0
        if ($index -ge 100 -and $index -le 939) {
            $applied = $index - 99
        } elseif ($index -gt 939 -and $index -lt 1240) {
            $applied = 840
        } elseif ($index -ge 1240 -and $index -le 2079) {
            $applied = 840 - ($index - 1239)
        }
        [pscustomobject]@{
            encoder_delta_ma = if ($index -ge 100 -and
                                    $index -lt 2080) {
                100
            } else {
                0
            }
            applied_pwm = $applied
        }
    }
    $run = Measure-G2HighPwmRun `
        -Samples $samples `
        -ExperimentId pwm840_steady_r1_pos `
        -Direction Positive `
        -PeakAppliedIndex 939 `
        -TargetStopIndex 1240
    Assert-True (
        $run.full_pwm_plateau_ms -eq 301 -and
        $run.tail_200ms_signed_counts -eq 20000 -and
        $run.tail_300ms_signed_counts -eq 30000) `
        '840单次诊断固定200/300ms双窗口'
    Assert-True (
        [Math]::Abs($run.tail_200ms_wheel_rpm - 48.828125) -lt
            0.000001 -and
        [Math]::Abs($run.tail_300ms_wheel_rpm - 48.828125) -lt
            0.000001) `
        '840单次诊断按122880 counts/rev换算双窗口'
    Assert-Throws {
        Measure-G2HighPwmRun `
            -Samples $samples `
            -ExperimentId too_short `
            -Direction Positive `
            -PeakAppliedIndex 939 `
            -TargetStopIndex 1238
    } '不足300ms满幅平台必须拒绝840双窗口诊断'

    $batchRoot = Join-Path $temporaryDirectory 'batches'
    $batch = & (
        Join-Path $PSScriptRoot '..\tools\new_g2_high_pwm_validation_batch.ps1'
    ) `
        -BatchId g2_840_test `
        -FirmwareCommit cc1b43a `
        -SequenceStart 4486 `
        -OutputRoot $batchRoot
    $batchRows = @(Import-Csv -LiteralPath $batch.CsvPath)
    $batchManifest =
        Get-Content -LiteralPath $batch.JsonPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    Assert-True (
        $batch.ExperimentCount -eq 6 -and
        $batch.NextUnusedSequence -eq 4522 -and
        $batchManifest.execution_state -ceq 'not_authorized') `
        '840批次固定六计划、序号4522和未授权状态'
    Assert-True (
        (@($batchRows | ForEach-Object { $_.direction }) -join '/') -ceq
            'Positive/Negative/Negative/Positive/Positive/Negative') `
        '840批次顺序固定为正负负正正负'
    Assert-True (
        (@($batchRows | ForEach-Object {
            [int]$_.sequence_start
        }) -join '/') -ceq '4486/4492/4498/4504/4510/4516') `
        '840批次每个计划消耗六个连续序号'

    $captureRoot = Join-Path $temporaryDirectory 'captures'
    [void][IO.Directory]::CreateDirectory($captureRoot)
    $captureDirectories = [Collections.Generic.List[string]]::new()
    $positiveRpms = @(50.0, 50.5, 49.5)
    $negativeRpms = @(48.0, 48.5, 47.5)
    foreach ($row in $batchRows) {
        $captureDirectory = Join-Path $captureRoot $row.experiment_id
        [void][IO.Directory]::CreateDirectory($captureDirectory)
        $repeatIndex = [int]$row.repetition - 1
        $rpm = if ($row.direction -ceq 'Positive') {
            $positiveRpms[$repeatIndex]
        } else {
            $negativeRpms[$repeatIndex]
        }
        $sourcePlan = Join-Path $batch.BatchDirectory `
            $row.relative_plan_directory
        [IO.File]::WriteAllText(
            (Join-Path $captureDirectory 'g2_dynamic_step_summary.json'),
            ([ordered]@{
                experiment_id = $row.experiment_id
                experiment_type = 'g2_high_pwm_steady_validation'
                source_plan_directory = $sourcePlan
                accepted = $true
                dynamics = [ordered]@{
                    direction = $row.direction
                    peak_pwm = 840
                    active_battery_minimum_mv = 11550
                }
                high_pwm_diagnostic = [ordered]@{
                    tail_200ms_wheel_rpm = $rpm
                    tail_300ms_wheel_rpm = $rpm + 0.1
                }
            } | ConvertTo-Json -Depth 5),
            [Text.UTF8Encoding]::new($false))
        $captureDirectories.Add($captureDirectory)
    }
    $summaryOutput = Join-Path $temporaryDirectory 'summary'
    $summary = & (
        Join-Path $PSScriptRoot `
            '..\tools\summarize_g2_high_pwm_validation_batch.ps1'
    ) `
        -BatchDirectory $batch.BatchDirectory `
        -CaptureDirectories $captureDirectories.ToArray() `
        -OutputDirectory $summaryOutput
    $summaryJson =
        Get-Content -LiteralPath $summary.SummaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    Assert-True (
        $summary.CaptureCount -eq 6 -and
        $summary.CaptureEvidenceAccepted -and
        $summary.RepeatabilityAccepted -and
        -not $summary.ModelReady) `
        '840双向三次双窗口批次通过但模型仍未就绪'
    Assert-True (
        $summaryJson.measurement.groups.Count -eq 2 -and
        -not $summaryJson.measurement.model_ready) `
        '840批次摘要分别保留正反向组和模型边界'

    $badSummaryPath = Join-Path $captureDirectories[0] `
        'g2_dynamic_step_summary.json'
    $badSummary =
        Get-Content -LiteralPath $badSummaryPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    $badSummary.high_pwm_diagnostic.tail_200ms_wheel_rpm = 35.0
    [IO.File]::WriteAllText(
        $badSummaryPath,
        ($badSummary | ConvertTo-Json -Depth 5),
        [Text.UTF8Encoding]::new($false))
    $rejectedOutput = Join-Path $temporaryDirectory 'rejected-summary'
    $rejected = & (
        Join-Path $PSScriptRoot `
            '..\tools\summarize_g2_high_pwm_validation_batch.ps1'
    ) `
        -BatchDirectory $batch.BatchDirectory `
        -CaptureDirectories $captureDirectories.ToArray() `
        -OutputDirectory $rejectedOutput
    Assert-True (
        $rejected.CaptureEvidenceAccepted -and
        -not $rejected.RepeatabilityAccepted) `
        '840批次双窗口离散超门时保持证据可用但拒绝重复性'
} finally {
    if (Test-Path -LiteralPath $temporaryDirectory -PathType Container) {
        Remove-Item -LiteralPath $temporaryDirectory -Recurse -Force
    }
}

Write-Host "g2_high_pwm_validation：$script:assertionCount 项断言通过"
