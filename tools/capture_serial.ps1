[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^COM[0-9]+$')]
    [string]$Port,

    [ValidateRange(1, 86400)]
    [int]$DurationSeconds = 10,

    [ValidateRange(1200, 3000000)]
    [int]$BaudRate = 230400,

    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[0-9a-fA-F]{7,40}$')]
    [string]$FirmwareCommit,

    [string]$OutputRoot = '',

    [switch]$SendStatusAtStart,

    [string]$CommandSchedulePath = '',

    [switch]$AllowNonStatusCommands
)

$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot 'serial_capture_lib.ps1')
. (Join-Path $PSScriptRoot 'serial_capture_raw.ps1')

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $repositoryRoot 'captures'
} elseif (-not [IO.Path]::IsPathRooted($OutputRoot)) {
    $OutputRoot = Join-Path (Get-Location).Path $OutputRoot
}
$resolvedOutputRoot = [IO.Path]::GetFullPath($OutputRoot)
[void][IO.Directory]::CreateDirectory($resolvedOutputRoot)

$durationMilliseconds = [uint64]$DurationSeconds * 1000
$schedule = @(Get-SerialCaptureCommandSchedule `
    -Path $CommandSchedulePath `
    -DurationMilliseconds $durationMilliseconds `
    -SendStatusAtStart:$SendStatusAtStart `
    -AllowNonStatusCommands:$AllowNonStatusCommands)

$captureName = '{0}_{1}' -f (Get-Date -Format 'yyyyMMdd-HHmmssfff'), $Port.ToUpperInvariant()
$captureDirectory = Join-Path $resolvedOutputRoot $captureName
if (Test-Path -LiteralPath $captureDirectory) {
    $captureName += '_' + [Guid]::NewGuid().ToString('N').Substring(0, 8)
    $captureDirectory = Join-Path $resolvedOutputRoot $captureName
}
[void][IO.Directory]::CreateDirectory($captureDirectory)

$rawPath = Join-Path $captureDirectory 'raw_uart.log'
$rawTimingPath = Join-Path $captureDirectory 'raw_chunk_timing.bin'
$telemetryPath = Join-Path $captureDirectory 'telemetry.csv'
$statPath = Join-Path $captureDirectory 'stat.csv'
$imuqPath = Join-Path $captureDirectory 'imuq.csv'
$resourcePath = Join-Path $captureDirectory 'resources.csv'
$eventsPath = Join-Path $captureDirectory 'events.csv'
$motorCapturePath = Join-Path $captureDirectory 'motor_capture.csv'
$motorCaptureEventsPath = Join-Path $captureDirectory 'motor_capture_events.csv'
$speedCapturePath = Join-Path $captureDirectory 'speed_capture.csv'
$speedCaptureEventsPath = Join-Path $captureDirectory 'speed_capture_events.csv'
$imuCapturePath = Join-Path $captureDirectory 'imu_capture.csv'
$imuCaptureEventsPath = Join-Path $captureDirectory 'imu_capture_events.csv'
$commandsPath = Join-Path $captureDirectory 'commands.csv'
$metadataPath = Join-Path $captureDirectory 'metadata.json'
$utf8WithoutBom = [Text.UTF8Encoding]::new($false)
$serialReadBufferBytes = 65536

$serial = [IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [IO.Ports.Parity]::None,
    8,
    [IO.Ports.StopBits]::One)
$serial.Handshake = [IO.Ports.Handshake]::None
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$serial.ReadBufferSize = $serialReadBufferBytes
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.Encoding = [Text.Encoding]::ASCII

$rawStream = $null
$rawTimingStream = $null
$rawTimingWriter = $null
$telemetryWriter = $null
$statWriter = $null
$imuqWriter = $null
$resourceWriter = $null
$eventsWriter = $null
$motorCaptureWriter = $null
$motorCaptureEventsWriter = $null
$speedCaptureWriter = $null
$speedCaptureEventsWriter = $null
$imuCaptureWriter = $null
$imuCaptureEventsWriter = $null
$commandsWriter = $null
$stopwatch = [Diagnostics.Stopwatch]::new()
$startedUtc = [DateTimeOffset]::UtcNow
$endedUtc = $startedUtc
$actualDurationMs = [uint64]0
$offlineParseDurationMs = [uint64]0
$outcome = 'failed'
$failureMessage = $null
$rawByteCount = [uint64]0
$rawTimingRecordCount = [uint64]0
$endDrainByteCount = [uint64]0
$trailingPartialCharacterCount = [uint64]0
$completeLineCount = [uint64]0
$telemetryRowCount = [uint64]0
$telemetryParseErrorCount = [uint64]0
$statRowCount = [uint64]0
$statParseErrorCount = [uint64]0
$imuqRowCount = [uint64]0
$imuqParseErrorCount = [uint64]0
$resourceRowCount = [uint64]0
$resourceParseErrorCount = [uint64]0
$eventRowCount = [uint64]0
$eventParseErrorCount = [uint64]0
$motorCaptureRowCount = [uint64]0
$motorCaptureEventCount = [uint64]0
$motorCaptureParseErrorCount = [uint64]0
$speedCaptureRowCount = [uint64]0
$speedCaptureEventCount = [uint64]0
$speedCaptureParseErrorCount = [uint64]0
$imuCaptureRowCount = [uint64]0
$imuCaptureEventCount = [uint64]0
$imuCaptureParseErrorCount = [uint64]0
$nonTelemetryLineCount = [uint64]0
$commandSentCount = [uint64]0
$nextCommandIndex = 0
$readBuffer = [byte[]]::new(4096)

try {
    $rawStream = [IO.File]::Open(
        $rawPath,
        [IO.FileMode]::CreateNew,
        [IO.FileAccess]::Write,
        [IO.FileShare]::Read)
    $rawTimingStream = [IO.File]::Open(
        $rawTimingPath,
        [IO.FileMode]::CreateNew,
        [IO.FileAccess]::Write,
        [IO.FileShare]::Read)
    $rawTimingWriter = [IO.BinaryWriter]::new($rawTimingStream)
    $telemetryWriter = [IO.StreamWriter]::new($telemetryPath, $false, $utf8WithoutBom)
    $statWriter = [IO.StreamWriter]::new($statPath, $false, $utf8WithoutBom)
    $imuqWriter = [IO.StreamWriter]::new($imuqPath, $false, $utf8WithoutBom)
    $resourceWriter = [IO.StreamWriter]::new($resourcePath, $false, $utf8WithoutBom)
    $eventsWriter = [IO.StreamWriter]::new($eventsPath, $false, $utf8WithoutBom)
    $motorCaptureWriter = [IO.StreamWriter]::new(
        $motorCapturePath, $false, $utf8WithoutBom)
    $motorCaptureEventsWriter = [IO.StreamWriter]::new(
        $motorCaptureEventsPath, $false, $utf8WithoutBom)
    $speedCaptureWriter = [IO.StreamWriter]::new(
        $speedCapturePath, $false, $utf8WithoutBom)
    $speedCaptureEventsWriter = [IO.StreamWriter]::new(
        $speedCaptureEventsPath, $false, $utf8WithoutBom)
    $imuCaptureWriter = [IO.StreamWriter]::new(
        $imuCapturePath, $false, $utf8WithoutBom)
    $imuCaptureEventsWriter = [IO.StreamWriter]::new(
        $imuCaptureEventsPath, $false, $utf8WithoutBom)
    $commandsWriter = [IO.StreamWriter]::new($commandsPath, $false, $utf8WithoutBom)

    $telemetryColumns = @('capture_elapsed_ms', 'host_received_utc') + @(Get-AppTelemetryColumnNames)
    $telemetryWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $telemetryColumns))
    $statWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $telemetryColumns))
    $imuqColumns = @('capture_elapsed_ms', 'host_received_utc') + @(Get-AppImuqColumnNames)
    $imuqWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $imuqColumns))
    $resourceColumns = @('capture_elapsed_ms', 'host_received_utc') + @(Get-AppResourceColumnNames)
    $resourceWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $resourceColumns))
    $eventColumns = @('capture_elapsed_ms', 'host_received_utc') + @(Get-AppEventColumnNames)
    $eventsWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $eventColumns))
    $motorCaptureColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-MotorCaptureColumnNames)
    $motorCaptureWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $motorCaptureColumns))
    $motorCaptureEventColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-MotorCaptureEventColumnNames)
    $motorCaptureEventsWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $motorCaptureEventColumns))
    $speedCaptureColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-SpeedCaptureColumnNames)
    $speedCaptureWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $speedCaptureColumns))
    $speedCaptureEventColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-SpeedCaptureEventColumnNames)
    $speedCaptureEventsWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $speedCaptureEventColumns))
    $imuCaptureColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-ImuCaptureColumnNames)
    $imuCaptureWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $imuCaptureColumns))
    $imuCaptureEventColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-ImuCaptureEventColumnNames)
    $imuCaptureEventsWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $imuCaptureEventColumns))
    $commandsWriter.WriteLine((ConvertTo-CaptureCsvLine -Values @(
        'planned_elapsed_ms',
        'actual_elapsed_ms',
        'host_sent_utc',
        'command',
        'result'
    )))

    $serial.Open()
    $startedUtc = [DateTimeOffset]::UtcNow
    $stopwatch.Start()

    while ([uint64]$stopwatch.ElapsedMilliseconds -lt $durationMilliseconds) {
        while ($nextCommandIndex -lt $schedule.Count -and
               [uint64]$stopwatch.ElapsedMilliseconds -ge
               [uint64]$schedule[$nextCommandIndex].planned_elapsed_ms) {
            $scheduledCommand = $schedule[$nextCommandIndex]
            $actualElapsed = [uint64]$stopwatch.ElapsedMilliseconds
            $sentUtc = [DateTimeOffset]::UtcNow.ToString('O')
            $commandResult = 'sent'
            try {
                $serial.Write(([string]$scheduledCommand.command + "`n"))
                $commandSentCount++
            } catch {
                $commandResult = 'failed: ' + $_.Exception.Message
                $commandsWriter.WriteLine((ConvertTo-CaptureCsvLine -Values @(
                    $scheduledCommand.planned_elapsed_ms,
                    $actualElapsed,
                    $sentUtc,
                    $scheduledCommand.command,
                    $commandResult
                )))
                $commandsWriter.Flush()
                throw
            }
            $commandsWriter.WriteLine((ConvertTo-CaptureCsvLine -Values @(
                $scheduledCommand.planned_elapsed_ms,
                $actualElapsed,
                $sentUtc,
                $scheduledCommand.command,
                $commandResult
            )))
            $nextCommandIndex++
        }

        $available = $serial.BytesToRead
        if ($available -le 0) {
            Start-Sleep -Milliseconds 5
            continue
        }

        $requested = [Math]::Min($available, $readBuffer.Length)
        $readCount = $serial.Read($readBuffer, 0, $requested)
        if ($readCount -le 0) {
            continue
        }
        $rawStream.Write($readBuffer, 0, $readCount)
        $rawByteCount += [uint64]$readCount
        $chunkReceivedUtc = [DateTimeOffset]::UtcNow
        $chunkElapsedMs = [uint64]$stopwatch.ElapsedMilliseconds
        $rawTimingWriter.Write([uint64]$rawByteCount)
        $rawTimingWriter.Write($chunkElapsedMs)
        $rawTimingWriter.Write([int64]$chunkReceivedUtc.UtcDateTime.Ticks)
        $rawTimingRecordCount++
    }

    $actualDurationMs = [uint64]$stopwatch.ElapsedMilliseconds
    $endedUtc = [DateTimeOffset]::UtcNow
    $stopwatch.Stop()
    $endDrainRemaining = [int]$serial.BytesToRead
    while ($endDrainRemaining -gt 0) {
        $requested = [Math]::Min($endDrainRemaining, $readBuffer.Length)
        $readCount = $serial.Read($readBuffer, 0, $requested)
        if ($readCount -le 0) {
            break
        }
        $rawStream.Write($readBuffer, 0, $readCount)
        $rawByteCount += [uint64]$readCount
        $endDrainByteCount += [uint64]$readCount
        $rawTimingWriter.Write([uint64]$rawByteCount)
        $rawTimingWriter.Write($actualDurationMs)
        $rawTimingWriter.Write([int64]$endedUtc.UtcDateTime.Ticks)
        $rawTimingRecordCount++
        $endDrainRemaining -= $readCount
    }
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $rawStream.Flush()
    $rawStream.Dispose()
    $rawStream = $null
    $rawTimingWriter.Flush()
    $rawTimingWriter.Dispose()
    $rawTimingWriter = $null
    $rawTimingStream = $null

    $offlineParseStopwatch = [Diagnostics.Stopwatch]::StartNew()
    $parsedTimingRecordCount = [uint64]0
    Read-SerialCaptureRawLine -RawPath $rawPath -TimingPath $rawTimingPath |
        ForEach-Object {
            $lineRecord = $_
            if ($lineRecord.kind -ceq 'summary') {
                $trailingPartialCharacterCount =
                    [uint64]$lineRecord.trailing_partial_characters
                $parsedTimingRecordCount = [uint64]$lineRecord.timing_records
                return
            }

            $line = [string]$lineRecord.line
            $lineCaptureElapsedMs = [uint64]$lineRecord.capture_elapsed_ms
            $lineHostReceivedUtc = [string]$lineRecord.host_received_utc
            $completeLineCount++
            if ($line.StartsWith('T,', [StringComparison]::Ordinal)) {
                try {
                    $telemetry = ConvertFrom-AppTelemetryLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-AppTelemetryColumnNames)) {
                        $rowValues += $telemetry.$column
                    }
                    $telemetryWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $rowValues))
                    $telemetryRowCount++
                } catch {
                    $telemetryParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('STAT,', [StringComparison]::Ordinal)) {
                try {
                    $stat = ConvertFrom-AppStatLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-AppTelemetryColumnNames)) {
                        $rowValues += $stat.$column
                    }
                    $csvRow = ConvertTo-CaptureCsvLine -Values $rowValues
                    $statWriter.WriteLine($csvRow)
                    $telemetryWriter.WriteLine($csvRow)
                    $statRowCount++
                    $telemetryRowCount++
                } catch {
                    $statParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('IMUQ,', [StringComparison]::Ordinal)) {
                try {
                    $imuq = ConvertFrom-AppImuqLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-AppImuqColumnNames)) {
                        $rowValues += $imuq.$column
                    }
                    $imuqWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $imuqRowCount++
                } catch {
                    $imuqParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('RES,', [StringComparison]::Ordinal)) {
                try {
                    $resource = ConvertFrom-AppResourceLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-AppResourceColumnNames)) {
                        $rowValues += $resource.$column
                    }
                    $resourceWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $resourceRowCount++
                } catch {
                    $resourceParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('EVENT,', [StringComparison]::Ordinal)) {
                try {
                    $event = ConvertFrom-AppEventLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-AppEventColumnNames)) {
                        $rowValues += $event.$column
                    }
                    $eventsWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $eventRowCount++
                } catch {
                    $eventParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('MC,', [StringComparison]::Ordinal)) {
                try {
                    $sample = ConvertFrom-MotorCaptureSampleLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-MotorCaptureColumnNames)) {
                        $rowValues += $sample.$column
                    }
                    $motorCaptureWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $motorCaptureRowCount++
                } catch {
                    $motorCaptureParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('MCAP,', [StringComparison]::Ordinal)) {
                try {
                    $event = ConvertFrom-MotorCaptureEventLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-MotorCaptureEventColumnNames)) {
                        $rowValues += $event.$column
                    }
                    $motorCaptureEventsWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $motorCaptureEventCount++
                } catch {
                    $motorCaptureParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('SC,', [StringComparison]::Ordinal)) {
                try {
                    $sample = ConvertFrom-SpeedCaptureSampleLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-SpeedCaptureColumnNames)) {
                        $rowValues += $sample.$column
                    }
                    $speedCaptureWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $speedCaptureRowCount++
                } catch {
                    $speedCaptureParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('SCAP,', [StringComparison]::Ordinal)) {
                try {
                    $event = ConvertFrom-SpeedCaptureEventLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-SpeedCaptureEventColumnNames)) {
                        $rowValues += $event.$column
                    }
                    $speedCaptureEventsWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $speedCaptureEventCount++
                } catch {
                    $speedCaptureParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('IC,', [StringComparison]::Ordinal)) {
                try {
                    $sample = ConvertFrom-ImuCaptureSampleLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-ImuCaptureColumnNames)) {
                        $rowValues += $sample.$column
                    }
                    $imuCaptureWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $imuCaptureRowCount++
                } catch {
                    $imuCaptureParseErrorCount++
                }
                return
            }
            if ($line.StartsWith('ICAP,', [StringComparison]::Ordinal)) {
                try {
                    $event = ConvertFrom-ImuCaptureEventLine -Line $line
                    $rowValues = @(
                        $lineCaptureElapsedMs,
                        $lineHostReceivedUtc
                    )
                    foreach ($column in (Get-ImuCaptureEventColumnNames)) {
                        $rowValues += $event.$column
                    }
                    $imuCaptureEventsWriter.WriteLine(
                        (ConvertTo-CaptureCsvLine -Values $rowValues))
                    $imuCaptureEventCount++
                } catch {
                    $imuCaptureParseErrorCount++
                }
                return
            }
            if (-not [string]::IsNullOrEmpty($line)) {
                $nonTelemetryLineCount++
            }
        }
    $offlineParseStopwatch.Stop()
    $offlineParseDurationMs = [uint64]$offlineParseStopwatch.ElapsedMilliseconds
    if ($parsedTimingRecordCount -ne $rawTimingRecordCount) {
        throw "在线与离线串口块时刻记录数不一致：$rawTimingRecordCount / $parsedTimingRecordCount"
    }

    $outcome = 'completed'
} catch {
    $failureMessage = $_.Exception.Message
    throw
} finally {
    if ($actualDurationMs -eq 0) {
        $actualDurationMs = [uint64]$stopwatch.ElapsedMilliseconds
        $endedUtc = [DateTimeOffset]::UtcNow
    }
    $stopwatch.Stop()
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
    if ($null -ne $rawStream) {
        $rawStream.Flush()
        $rawStream.Dispose()
    }
    if ($null -ne $rawTimingWriter) {
        $rawTimingWriter.Flush()
        $rawTimingWriter.Dispose()
    } elseif ($null -ne $rawTimingStream) {
        $rawTimingStream.Dispose()
    }
    if ($null -ne $telemetryWriter) {
        $telemetryWriter.Flush()
        $telemetryWriter.Dispose()
    }
    foreach ($typedWriter in @($statWriter, $imuqWriter, $resourceWriter, $eventsWriter)) {
        if ($null -ne $typedWriter) {
            $typedWriter.Flush()
            $typedWriter.Dispose()
        }
    }
    if ($null -ne $motorCaptureWriter) {
        $motorCaptureWriter.Flush()
        $motorCaptureWriter.Dispose()
    }
    if ($null -ne $motorCaptureEventsWriter) {
        $motorCaptureEventsWriter.Flush()
        $motorCaptureEventsWriter.Dispose()
    }
    if ($null -ne $speedCaptureWriter) {
        $speedCaptureWriter.Flush()
        $speedCaptureWriter.Dispose()
    }
    if ($null -ne $speedCaptureEventsWriter) {
        $speedCaptureEventsWriter.Flush()
        $speedCaptureEventsWriter.Dispose()
    }
    if ($null -ne $imuCaptureWriter) {
        $imuCaptureWriter.Flush()
        $imuCaptureWriter.Dispose()
    }
    if ($null -ne $imuCaptureEventsWriter) {
        $imuCaptureEventsWriter.Flush()
        $imuCaptureEventsWriter.Dispose()
    }
    if ($null -ne $commandsWriter) {
        $commandsWriter.Flush()
        $commandsWriter.Dispose()
    }

    $metadata = [ordered]@{
        schema_version = 4
        tool = 'tools/capture_serial.ps1'
        tool_sha256 = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant()
        library_sha256 = (Get-FileHash `
            -LiteralPath (Join-Path $PSScriptRoot 'serial_capture_lib.ps1') `
            -Algorithm SHA256).Hash.ToLowerInvariant()
        raw_reader_sha256 = (Get-FileHash `
            -LiteralPath (Join-Path $PSScriptRoot 'serial_capture_raw.ps1') `
            -Algorithm SHA256).Hash.ToLowerInvariant()
        outcome = $outcome
        failure_message = $failureMessage
        firmware_commit = $FirmwareCommit.ToLowerInvariant()
        repository_commit = (& git -C $repositoryRoot rev-parse HEAD 2>$null)
        repository_dirty = [bool](@(& git -C $repositoryRoot status --porcelain 2>$null).Count -gt 0)
        started_utc = $startedUtc.ToString('O')
        ended_utc = $endedUtc.ToString('O')
        requested_duration_seconds = $DurationSeconds
        actual_duration_ms = $actualDurationMs
        host = [ordered]@{
            computer_name = [Environment]::MachineName
            os_version = [Environment]::OSVersion.VersionString
            powershell_version = $PSVersionTable.PSVersion.ToString()
        }
        serial = [ordered]@{
            port = $Port.ToUpperInvariant()
            baud_rate = $BaudRate
            data_bits = 8
            parity = 'None'
            stop_bits = 1
            handshake = 'None'
            read_buffer_bytes = $serialReadBufferBytes
            dtr_enabled = $false
            rts_enabled = $false
        }
        safety = [ordered]@{
            non_status_commands_allowed = [bool]$AllowNonStatusCommands
        }
        processing = [ordered]@{
            mode = 'raw_first_offline_parse'
            offline_parse_duration_ms = $offlineParseDurationMs
            bounded_end_drain_bytes = $endDrainByteCount
            raw_chunk_timing_record_bytes = 24
            raw_chunk_timing_sha256 = if (Test-Path -LiteralPath $rawTimingPath) {
                (Get-FileHash -LiteralPath $rawTimingPath -Algorithm SHA256).Hash.ToLowerInvariant()
            } else {
                $null
            }
        }
        counts = [ordered]@{
            raw_bytes = $rawByteCount
            raw_chunk_timing_records = $rawTimingRecordCount
            complete_lines = $completeLineCount
            telemetry_rows = $telemetryRowCount
            telemetry_parse_errors = $telemetryParseErrorCount
            stat_rows = $statRowCount
            stat_parse_errors = $statParseErrorCount
            imuq_rows = $imuqRowCount
            imuq_parse_errors = $imuqParseErrorCount
            resource_rows = $resourceRowCount
            resource_parse_errors = $resourceParseErrorCount
            event_rows = $eventRowCount
            event_parse_errors = $eventParseErrorCount
            motor_capture_rows = $motorCaptureRowCount
            motor_capture_events = $motorCaptureEventCount
            motor_capture_parse_errors = $motorCaptureParseErrorCount
            speed_capture_rows = $speedCaptureRowCount
            speed_capture_events = $speedCaptureEventCount
            speed_capture_parse_errors = $speedCaptureParseErrorCount
            imu_capture_rows = $imuCaptureRowCount
            imu_capture_events = $imuCaptureEventCount
            imu_capture_parse_errors = $imuCaptureParseErrorCount
            non_telemetry_lines = $nonTelemetryLineCount
            commands_scheduled = $schedule.Count
            commands_sent = $commandSentCount
            trailing_partial_characters = $trailingPartialCharacterCount
        }
        artifacts = [ordered]@{
            raw_uart_log = 'raw_uart.log'
            raw_chunk_timing = 'raw_chunk_timing.bin'
            telemetry_csv = 'telemetry.csv'
            stat_csv = 'stat.csv'
            imuq_csv = 'imuq.csv'
            resources_csv = 'resources.csv'
            events_csv = 'events.csv'
            motor_capture_csv = 'motor_capture.csv'
            motor_capture_events_csv = 'motor_capture_events.csv'
            speed_capture_csv = 'speed_capture.csv'
            speed_capture_events_csv = 'speed_capture_events.csv'
            imu_capture_csv = 'imu_capture.csv'
            imu_capture_events_csv = 'imu_capture_events.csv'
            commands_csv = 'commands.csv'
            metadata_json = 'metadata.json'
        }
    }
    [IO.File]::WriteAllText(
        $metadataPath,
        ($metadata | ConvertTo-Json -Depth 6),
        $utf8WithoutBom)
}

[pscustomobject]@{
    CaptureDirectory = $captureDirectory
    Outcome = $outcome
    RawBytes = $rawByteCount
    TelemetryRows = $telemetryRowCount
    TelemetryParseErrors = $telemetryParseErrorCount
    StatRows = $statRowCount
    ImuqRows = $imuqRowCount
    ResourceRows = $resourceRowCount
    EventRows = $eventRowCount
    MotorCaptureRows = $motorCaptureRowCount
    MotorCaptureEvents = $motorCaptureEventCount
    MotorCaptureParseErrors = $motorCaptureParseErrorCount
    SpeedCaptureRows = $speedCaptureRowCount
    SpeedCaptureEvents = $speedCaptureEventCount
    SpeedCaptureParseErrors = $speedCaptureParseErrorCount
    ImuCaptureRows = $imuCaptureRowCount
    ImuCaptureEvents = $imuCaptureEventCount
    ImuCaptureParseErrors = $imuCaptureParseErrorCount
    CommandsSent = $commandSentCount
}
