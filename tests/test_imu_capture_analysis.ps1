$ErrorActionPreference = 'Stop'

$assertionCount = 0
function Assert-True {
    param([Parameter(Mandatory = $true)][bool]$Condition,
          [Parameter(Mandatory = $true)][string]$Message)
    $script:assertionCount++
    if (-not $Condition) {
        throw "断言失败：$Message"
    }
}

$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$analyzer = Join-Path $root 'tools\analyze_imu_capture.ps1'
$temporary = Join-Path ([IO.Path]::GetTempPath()) (
    'imu-capture-analysis-test-' + [Guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporary)

try {
    $samples = @(
        for ($index = 0; $index -le 100; $index++) {
            [pscustomobject]@{
                capture_elapsed_ms = [uint64][Math]::Round($index * 1000.0 / 224.2)
                host_received_utc = '2026-07-19T00:00:00Z'
                capture_index = $index
                imu_sequence = ([uint64]4294967245 + [uint64]$index) % 4294967296
                sensor_timestamp = ([uint64]16777180 + [uint64]$index) % 16777216
                host_tick_ms = [uint64](100 + [Math]::Round($index * 1000.0 / 224.2))
                flags = 7
                source_dropped_sample_count = 2
                raw_accel_x = 1; raw_accel_y = 2; raw_accel_z = 3
                raw_gyro_x = 4; raw_gyro_y = 5; raw_gyro_z = 6
                raw_temperature = 7; sensor_status = 3; health = 1
            }
        }
    )
    $events = @(
        [pscustomobject]@{
            capture_elapsed_ms = 10; host_received_utc = '2026-07-19T00:00:01Z'
            event = 'BEGIN'; state = 2; sample_count = 101; capacity = 1700
            dropped_sample_count = 0; duplicate_sequence_count = 0
            source_gap_count = 0
        },
        [pscustomobject]@{
            capture_elapsed_ms = 20; host_received_utc = '2026-07-19T00:00:02Z'
            event = 'END'; state = 2; sample_count = 101; capacity = 1700
            dropped_sample_count = 0; duplicate_sequence_count = 0
            source_gap_count = 0
        }
    )
    $samples | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture.csv') -NoTypeInformation
    $events | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture_events.csv') -NoTypeInformation
    [IO.File]::WriteAllText(
        (Join-Path $temporary 'metadata.json'),
        (@{
            outcome = 'completed'
            firmware_commit = '1234567'
            counts = @{
                imu_capture_rows = 101
                imu_capture_events = 2
                imu_capture_parse_errors = 0
            }
        } | ConvertTo-Json -Depth 4),
        [Text.UTF8Encoding]::new($false))

    $result = & $analyzer -CaptureDirectory $temporary
    Assert-True ([bool]$result.Accepted) '正常IMU高速证据通过'
    $summary = Get-Content -LiteralPath $result.Summary -Raw | ConvertFrom-Json
    Assert-True ($summary.metrics.host_tick_delta.count_4ms -gt 0 -and
                 $summary.metrics.host_tick_delta.count_5ms -gt 0) '统计4/5ms主机量化间隔'
    Assert-True ($summary.metrics.sensor_timestamp_invalid_count -eq 0) '接受24位时间戳回绕'

    $samples[100].imu_sequence = 4
    $samples | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture.csv') -NoTypeInformation
    $rejected = $false
    try {
        & $analyzer -CaptureDirectory $temporary | Out-Null
    } catch {
        $rejected = $true
    }
    Assert-True $rejected '拒绝接受序号断点'

    $samples[100].imu_sequence =
        ([uint64]4294967245 + [uint64]100) % 4294967296
    $samples[100].sensor_timestamp =
        ([uint64]$samples[99].sensor_timestamp + 20) % 16777216
    $samples | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture.csv') -NoTypeInformation
    $rejected = $false
    try {
        & $analyzer -CaptureDirectory $temporary | Out-Null
    } catch {
        $rejected = $true
    }
    Assert-True $rejected 'Reject sensor timestamp step above the quality-chain limit'

    $samples[100].sensor_timestamp =
        ([uint64]16777180 + [uint64]100) % 16777216
    $samples[100].host_tick_ms = $samples[99].host_tick_ms
    $samples | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture.csv') -NoTypeInformation
    $rejected = $false
    try {
        & $analyzer -CaptureDirectory $temporary | Out-Null
    } catch {
        $rejected = $true
    }
    Assert-True $rejected 'Reject a host timestamp that does not advance'

    $samples[100].host_tick_ms =
        [uint64](100 + [Math]::Round(100 * 1000.0 / 224.2) + 100)
    $samples | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture.csv') -NoTypeInformation
    $rejected = $false
    try {
        & $analyzer -CaptureDirectory $temporary | Out-Null
    } catch {
        $rejected = $true
    }
    Assert-True $rejected 'Reject sensor and host duration mismatch'

    $samples[100].host_tick_ms =
        [uint64](100 + [Math]::Round(100 * 1000.0 / 224.2))
    $samples | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture.csv') -NoTypeInformation
    $events[1].capacity = 1699
    $events | Export-Csv -LiteralPath (Join-Path $temporary 'imu_capture_events.csv') -NoTypeInformation
    $rejected = $false
    try {
        & $analyzer -CaptureDirectory $temporary | Out-Null
    } catch {
        $rejected = $true
    }
    Assert-True $rejected 'Reject an unexpected fixed capture capacity'
} finally {
    if (Test-Path -LiteralPath $temporary -PathType Container) {
        Remove-Item -LiteralPath $temporary -Recurse -Force
    }
}

Write-Host "imu_capture_analysis: $assertionCount assertions passed"
