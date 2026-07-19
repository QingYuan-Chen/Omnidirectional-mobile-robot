$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\tools\g2_low_speed_diagnostic_lib.ps1')

$assertionCount = 0
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

$schedule = @(New-G2LowSpeedValidationSchedule -SequenceStart 100)
Assert-True ($schedule.Count -eq 14) '独立验证固定生成14条命令'
Assert-True (@($schedule | Where-Object {
    $_.command -match '^PWM [0-9]+ -240$'
}).Count -eq 5) '独立验证只生成五条反向240保活命令'
Assert-True (@($schedule | Where-Object {
    $_.command -match '^STOP [0-9]+$' -and $_.elapsed_ms -eq 2300
}).Count -eq 1) '独立验证延长平台后在2300 ms停车'
Assert-True (-not @($schedule | Where-Object {
    $_.command -ceq 'ESTOP'
}).Count) '独立验证不含ESTOP'

$brokenSchedule = @($schedule | ForEach-Object {
    [pscustomobject]@{
        elapsed_ms = $_.elapsed_ms
        command = $_.command
    }
})
$brokenSchedule[6].command = 'PWM 103 240'
Assert-Throws {
    Test-G2LowSpeedValidationSchedule -Schedule $brokenSchedule
} '拒绝正向或其他幅值PWM'

$samples = [Collections.Generic.List[object]]::new()
for ($index = 0; $index -lt 1500; $index++) {
    $applied = if ($index -ge 339 -and $index -lt 1300) { -240 } else { 0 }
    $delta = if ($index -ge 500 -and $index -lt 1300) { -20 } else { 0 }
    $samples.Add([pscustomobject]@{
        encoder_delta_ma = $delta
        applied_pwm = $applied
    })
}
$run = Measure-G2LowSpeedRun `
    -Samples @($samples) `
    -ExperimentId synthetic `
    -FirstTargetIndex 100 `
    -PeakAppliedIndex 339 `
    -TargetStopIndex 1300 `
    -MotionThresholdDelayMs 250 `
    -ReportedPeakWindowRpm 8.0
Assert-True ($run.first_move_delay_ms -eq 400) `
    '单次诊断找到首个期望方向编码器增量'
Assert-True ([Math]::Abs($run.tail_200ms_wheel_rpm - 9.765625) -lt 0.000001) `
    '末端200 ms按122880 counts/rev换算RPM'
Assert-True ([Math]::Abs($run.tail_300ms_wheel_rpm - 9.765625) -lt 0.000001) `
    '末端300 ms按固定窗口独立换算RPM'

$stableRows = @(
    [pscustomobject]@{
        experiment_id = 'r1'; accepted = $true
        direction = 'Negative'; peak_pwm = 240
        first_move_delay_ms = 70; motion_threshold_delay_ms = 240
        tail_200ms_wheel_rpm = 9.6; tail_300ms_wheel_rpm = 9.5
    },
    [pscustomobject]@{
        experiment_id = 'r2'; accepted = $true
        direction = 'Negative'; peak_pwm = 240
        first_move_delay_ms = 90; motion_threshold_delay_ms = 380
        tail_200ms_wheel_rpm = 9.8; tail_300ms_wheel_rpm = 9.7
    },
    [pscustomobject]@{
        experiment_id = 'r3'; accepted = $true
        direction = 'Negative'; peak_pwm = 240
        first_move_delay_ms = 80; motion_threshold_delay_ms = 250
        tail_200ms_wheel_rpm = 9.7; tail_300ms_wheel_rpm = 9.6
    }
)
$exploratory = Measure-G2LowSpeedDiagnosticBatch `
    -Rows $stableRows `
    -EvidenceRole exploratory
Assert-True ($exploratory.stable_speed_repeatability_accepted) `
    '双固定窗口均通过时接受稳态速度重复性'
Assert-True (-not $exploratory.startup_delay_repeatability_accepted) `
    '启动延迟离散超过门槛时保持未接受'
Assert-True ($exploratory.classification -ceq
             'stable_speed_variable_startup') `
    '区分稳定运行速度与离散启动过程'
Assert-True (-not $exploratory.independent_validation_accepted) `
    '探索性数据不能冒充独立验证'

$validation = Measure-G2LowSpeedDiagnosticBatch `
    -Rows $stableRows `
    -EvidenceRole independent_validation
Assert-True ($validation.independent_validation_accepted) `
    '预注册独立批次可关闭稳态速度验证'
Assert-True (-not $validation.original_short_window_gate_closed -and
             -not $validation.model_ready) `
    '新诊断不追溯改写旧门且不宣称模型就绪'

$unstableRows = @($stableRows | ForEach-Object {
    [pscustomobject]@{
        experiment_id = $_.experiment_id
        accepted = $_.accepted
        direction = $_.direction
        peak_pwm = $_.peak_pwm
        first_move_delay_ms = $_.first_move_delay_ms
        motion_threshold_delay_ms = $_.motion_threshold_delay_ms
        tail_200ms_wheel_rpm = $_.tail_200ms_wheel_rpm
        tail_300ms_wheel_rpm = $_.tail_300ms_wheel_rpm
    }
})
$unstableRows[2].tail_200ms_wheel_rpm = 7.0
$unstable = Measure-G2LowSpeedDiagnosticBatch -Rows $unstableRows
Assert-True ($unstable.classification -ceq 'unstable_running_speed') `
    '任一固定窗口重复性失败时保持稳态阻断'

$temporaryDirectory = Join-Path (
    [IO.Path]::GetTempPath()) (
        'g2-low-speed-' + [Guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporaryDirectory)
try {
    $batch = & (
        Join-Path $PSScriptRoot '..\tools\new_g2_low_speed_validation_batch.ps1') `
        -BatchId synthetic_low_speed `
        -FirmwareCommit 1234567 `
        -SequenceStart 100 `
        -OutputRoot $temporaryDirectory
    $batchRows = @(Import-Csv -LiteralPath $batch.CsvPath)
    $batchManifest =
        Get-Content -LiteralPath $batch.JsonPath -Raw -Encoding UTF8 |
            ConvertFrom-Json
    Assert-True ($batch.ExperimentCount -eq 3 -and
                 $batchRows.Count -eq 3) `
        '独立验证批次固定为三次'
    Assert-True ([uint32]$batchRows[0].sequence_start -eq 100 -and
                 [uint32]$batchRows[2].sequence_start -eq 114 -and
                 $batch.NextUnusedSequence -eq 121) `
        '三次计划使用连续且不重叠的七个运动序号'
    Assert-True ($batchManifest.execution_state -ceq 'not_authorized' -and
                 $batchManifest.evidence_role -ceq
                    'independent_validation') `
        '批次默认未授权且预注册为独立验证'
    Assert-True (
        $batchManifest.preregistered_decision.no_post_hoc_window_selection -and
        $batchManifest.preregistered_decision.maximum_repetitions -eq 3) `
        '批次冻结窗口且禁止三次后追加'
    $firstPlanDirectory =
        Join-Path $batch.BatchDirectory $batchRows[0].relative_plan_directory
    $firstPlan =
        Get-Content -LiteralPath (
            Join-Path $firstPlanDirectory 'experiment_manifest.json') `
            -Raw -Encoding UTF8 |
            ConvertFrom-Json
    $firstSchedule = @(Import-Csv -LiteralPath (
        Join-Path $firstPlanDirectory 'command_schedule.pending.csv'))
    $analysisProfile = Get-G2SingleCaptureAnalysisProfile `
        -Manifest $firstPlan `
        -Schedule $firstSchedule
    Assert-True (
        $firstPlan.controlled_conditions.minimum_cooldown_before_plan_seconds -eq
            60 -and
        $firstPlan.schedule.expected_full_pwm_plateau_ms -eq 1211) `
        '子计划冻结冷却时间和延长后的满PWM平台'
    Assert-True (
        $analysisProfile.low_speed_steady_validation -and
        $analysisProfile.minimum_full_pwm_plateau_ms -eq 1200 -and
        $firstPlan.analysis.single_capture_tool -ceq
            'tools/analyze_g2_dynamic_step.ps1' -and
        $firstPlan.analysis.batch_tool -ceq
            'tools/analyze_g2_low_speed_diagnostic.ps1') `
        '低速清单可由单次分析器验收并绑定批次诊断工具'
    Assert-True (
        $firstPlan.analysis.minimum_battery_mv -eq 11500 -and
        $firstPlan.analysis.counts_per_wheel_revolution -eq 122880 -and
        $firstPlan.analysis.motion_threshold_counts -eq 1000 -and
        $firstPlan.analysis.other_channel_limit_counts -eq 1000) `
        '低速清单显式冻结单次严格门参数'
    Assert-True (
        -not $firstPlan.controlled_conditions.temperature_measurement_required -and
        -not $firstPlan.controlled_conditions.temperature_record_required -and
        -not $firstPlan.controlled_conditions.temperature_is_acceptance_gate -and
        -not $batchManifest.preregistered_decision.temperature_baseline_enabled) `
        '独立验证明确关闭测温、温度记录和温度接受门'
    $unsupportedPlan = $firstPlan.PSObject.Copy()
    $unsupportedPlan.experiment_type = 'unsupported'
    Assert-Throws {
        Get-G2SingleCaptureAnalysisProfile `
            -Manifest $unsupportedPlan `
            -Schedule $firstSchedule
    } '单次分析器拒绝未知试验类型'
} finally {
    if (Test-Path -LiteralPath $temporaryDirectory) {
        Remove-Item -LiteralPath $temporaryDirectory -Recurse -Force
    }
}

Write-Host "g2_low_speed_diagnostic：$assertionCount 项断言通过"
