[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$CaptureDirectory,

    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'motor_capture_analysis_lib.ps1')

$resolvedCapture = (Resolve-Path -LiteralPath $CaptureDirectory).Path
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $resolvedOutput = $resolvedCapture
} else {
    $resolvedOutput = [IO.Path]::GetFullPath($OutputDirectory)
    [void][IO.Directory]::CreateDirectory($resolvedOutput)
}

$metadataPath = Join-Path $resolvedCapture 'metadata.json'
$samplesPath = Join-Path $resolvedCapture 'motor_capture.csv'
$eventsPath = Join-Path $resolvedCapture 'motor_capture_events.csv'
$telemetryPath = Join-Path $resolvedCapture 'telemetry.csv'
foreach ($path in @($metadataPath, $samplesPath, $eventsPath, $telemetryPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "高速采集证据文件不存在：$path"
    }
}

$metadata = Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
$samples = @(Import-Csv -LiteralPath $samplesPath)
$events = @(Import-Csv -LiteralPath $eventsPath)
$telemetry = @(Import-Csv -LiteralPath $telemetryPath)

if ($metadata.outcome -cne 'completed' -or
    [uint64]$metadata.counts.motor_capture_parse_errors -ne 0 -or
    $samples.Count -ne [int]$metadata.counts.motor_capture_rows -or
    $events.Count -ne [int]$metadata.counts.motor_capture_events) {
    throw '高速采集元数据、解析计数或CSV行数不闭合'
}

$beginEvents = @($events | Where-Object { $_.event -ceq 'BEGIN' })
$endEvents = @($events | Where-Object { $_.event -ceq 'END' })
if ($beginEvents.Count -ne 1 -or $endEvents.Count -ne 1) {
    throw '一次分析必须恰好包含一个BEGIN和一个END事件'
}
$begin = $beginEvents[0]
$end = $endEvents[0]
if ([uint32]$begin.sample_count -ne $samples.Count -or
    [uint32]$end.sample_count -ne $samples.Count -or
    [uint32]$begin.capacity -ne [uint32]$end.capacity -or
    [uint32]$begin.dropped_sample_count -ne [uint32]$end.dropped_sample_count) {
    throw 'BEGIN/END声明与高速样本数量不一致'
}

$measurement = Measure-MotorCaptureRows -Rows $samples
$telemetryCounterFields = @(
    'missed_period_count',
    'deadline_miss_count',
    'uart_error_count',
    'uart_rx_overflow_count',
    'uart_tx_fault_count',
    'command_reject_count',
    'command_queue_drop_count',
    'motion_gate_reject_count',
    'invalidated_motor_command_count',
    'adc_error_count'
)
$telemetryCounterIncrements = [ordered]@{}
foreach ($field in $telemetryCounterFields) {
    if ($telemetry.Count -lt 2 -or
        $telemetry[0].PSObject.Properties.Name -notcontains $field) {
        throw "遥测缺少高速采集安全计数：$field"
    }
    [uint64]$minimum =
        [uint64](($telemetry | Measure-Object -Property $field -Minimum).Minimum)
    [uint64]$maximum =
        [uint64](($telemetry | Measure-Object -Property $field -Maximum).Maximum)
    $telemetryCounterIncrements[$field] = $maximum - $minimum
}

$gates = [ordered]@{
    at_least_1000_samples = ($samples.Count -ge 1000)
    no_capture_drop = ([uint32]$begin.dropped_sample_count -eq 0)
    contiguous_capture_index = ($measurement.Summary.capture_index_gap_count -eq 0)
    contiguous_control_tick = ($measurement.Summary.control_tick_gap_count -eq 0)
    zero_reported_missed_period_increment =
        ($telemetryCounterIncrements.missed_period_count -eq 0)
    zero_reported_deadline_miss_increment =
        ($telemetryCounterIncrements.deadline_miss_count -eq 0)
    p99_irq_jitter_at_most_5_percent =
        ($measurement.Summary.p99_irq_jitter_cycles -le 8400)
    p99_wake_latency_at_most_5_percent =
        ($measurement.Summary.p99_wake_latency_cycles -le 8400)
    maximum_irq_jitter_at_most_10_percent =
        ($measurement.Summary.maximum_irq_jitter_cycles -le 16800)
    maximum_wake_latency_at_most_10_percent =
        ($measurement.Summary.maximum_wake_latency_cycles -le 16800)
    maximum_wcet_at_most_25_percent =
        ($measurement.Summary.maximum_previous_wcet_cycles -le 42000)
}
foreach ($field in $telemetryCounterFields | Select-Object -Skip 2) {
    $gates["zero_reported_${field}_increment"] =
        ($telemetryCounterIncrements[$field] -eq 0)
}
$accepted = -not ($gates.Values -contains $false)

$summary = [ordered]@{
    schema_version = 2
    source_capture_directory = $resolvedCapture
    source_firmware_commit = $metadata.firmware_commit
    sample_count = $samples.Count
    capacity = [uint32]$begin.capacity
    dropped_sample_count = [uint32]$begin.dropped_sample_count
    telemetry_counter_increments = $telemetryCounterIncrements
    timing = $measurement.Summary
    gates = $gates
    accepted = $accepted
    limitation =
        'previous_wcet_cycles belongs to the immediately preceding control cycle; the first row may predate CAPTURE START.'
}

$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
$summaryPath = Join-Path $resolvedOutput 'motor_capture_summary.json'
$derivedPath = Join-Path $resolvedOutput 'motor_capture_derived.csv'
[IO.File]::WriteAllText(
    $summaryPath,
    ($summary | ConvertTo-Json -Depth 8),
    $utf8WithoutBom)
$measurement.DerivedRows |
    Export-Csv -LiteralPath $derivedPath -NoTypeInformation -Encoding utf8

[pscustomobject]@{
    Accepted = $accepted
    SampleCount = $samples.Count
    SummaryPath = $summaryPath
    DerivedPath = $derivedPath
}
