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
$telemetryPath = Join-Path $captureDirectory 'telemetry.csv'
$motorCapturePath = Join-Path $captureDirectory 'motor_capture.csv'
$motorCaptureEventsPath = Join-Path $captureDirectory 'motor_capture_events.csv'
$commandsPath = Join-Path $captureDirectory 'commands.csv'
$metadataPath = Join-Path $captureDirectory 'metadata.json'
$utf8WithoutBom = [Text.UTF8Encoding]::new($false)

$serial = [IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [IO.Ports.Parity]::None,
    8,
    [IO.Ports.StopBits]::One)
$serial.Handshake = [IO.Ports.Handshake]::None
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.Encoding = [Text.Encoding]::ASCII

$rawStream = $null
$telemetryWriter = $null
$motorCaptureWriter = $null
$motorCaptureEventsWriter = $null
$commandsWriter = $null
$stopwatch = [Diagnostics.Stopwatch]::new()
$pendingText = [Text.StringBuilder]::new()
$startedUtc = [DateTimeOffset]::UtcNow
$endedUtc = $startedUtc
$outcome = 'failed'
$failureMessage = $null
$rawByteCount = [uint64]0
$completeLineCount = [uint64]0
$telemetryRowCount = [uint64]0
$telemetryParseErrorCount = [uint64]0
$motorCaptureRowCount = [uint64]0
$motorCaptureEventCount = [uint64]0
$motorCaptureParseErrorCount = [uint64]0
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
    $telemetryWriter = [IO.StreamWriter]::new($telemetryPath, $false, $utf8WithoutBom)
    $motorCaptureWriter = [IO.StreamWriter]::new(
        $motorCapturePath, $false, $utf8WithoutBom)
    $motorCaptureEventsWriter = [IO.StreamWriter]::new(
        $motorCaptureEventsPath, $false, $utf8WithoutBom)
    $commandsWriter = [IO.StreamWriter]::new($commandsPath, $false, $utf8WithoutBom)

    $telemetryColumns = @('capture_elapsed_ms', 'host_received_utc') + @(Get-AppTelemetryColumnNames)
    $telemetryWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $telemetryColumns))
    $motorCaptureColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-MotorCaptureColumnNames)
    $motorCaptureWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $motorCaptureColumns))
    $motorCaptureEventColumns =
        @('capture_elapsed_ms', 'host_received_utc') + @(Get-MotorCaptureEventColumnNames)
    $motorCaptureEventsWriter.WriteLine(
        (ConvertTo-CaptureCsvLine -Values $motorCaptureEventColumns))
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
        [void]$pendingText.Append([Text.Encoding]::ASCII.GetString($readBuffer, 0, $readCount))

        while ($true) {
            $pendingSnapshot = $pendingText.ToString()
            $lineEnd = $pendingSnapshot.IndexOf("`n", [StringComparison]::Ordinal)
            if ($lineEnd -lt 0) {
                break
            }

            $line = $pendingSnapshot.Substring(0, $lineEnd).TrimEnd([char]13)
            [void]$pendingText.Remove(0, $lineEnd + 1)
            $completeLineCount++
            if ($line.StartsWith('T,', [StringComparison]::Ordinal)) {
                try {
                    $telemetry = ConvertFrom-AppTelemetryLine -Line $line
                    $rowValues = @(
                        [uint64]$stopwatch.ElapsedMilliseconds,
                        [DateTimeOffset]::UtcNow.ToString('O')
                    )
                    foreach ($column in (Get-AppTelemetryColumnNames)) {
                        $rowValues += $telemetry.$column
                    }
                    $telemetryWriter.WriteLine((ConvertTo-CaptureCsvLine -Values $rowValues))
                    $telemetryRowCount++
                } catch {
                    $telemetryParseErrorCount++
                }
                continue
            }
            if ($line.StartsWith('MC,', [StringComparison]::Ordinal)) {
                try {
                    $sample = ConvertFrom-MotorCaptureSampleLine -Line $line
                    $rowValues = @(
                        [uint64]$stopwatch.ElapsedMilliseconds,
                        [DateTimeOffset]::UtcNow.ToString('O')
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
                continue
            }
            if ($line.StartsWith('MCAP,', [StringComparison]::Ordinal)) {
                try {
                    $event = ConvertFrom-MotorCaptureEventLine -Line $line
                    $rowValues = @(
                        [uint64]$stopwatch.ElapsedMilliseconds,
                        [DateTimeOffset]::UtcNow.ToString('O')
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
                continue
            }
            if (-not [string]::IsNullOrEmpty($line)) {
                $nonTelemetryLineCount++
            }
        }
    }

    $outcome = 'completed'
} catch {
    $failureMessage = $_.Exception.Message
    throw
} finally {
    $stopwatch.Stop()
    $endedUtc = [DateTimeOffset]::UtcNow
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
    if ($null -ne $rawStream) {
        $rawStream.Flush()
        $rawStream.Dispose()
    }
    if ($null -ne $telemetryWriter) {
        $telemetryWriter.Flush()
        $telemetryWriter.Dispose()
    }
    if ($null -ne $motorCaptureWriter) {
        $motorCaptureWriter.Flush()
        $motorCaptureWriter.Dispose()
    }
    if ($null -ne $motorCaptureEventsWriter) {
        $motorCaptureEventsWriter.Flush()
        $motorCaptureEventsWriter.Dispose()
    }
    if ($null -ne $commandsWriter) {
        $commandsWriter.Flush()
        $commandsWriter.Dispose()
    }

    $metadata = [ordered]@{
        schema_version = 2
        tool = 'tools/capture_serial.ps1'
        tool_sha256 = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant()
        library_sha256 = (Get-FileHash `
            -LiteralPath (Join-Path $PSScriptRoot 'serial_capture_lib.ps1') `
            -Algorithm SHA256).Hash.ToLowerInvariant()
        outcome = $outcome
        failure_message = $failureMessage
        firmware_commit = $FirmwareCommit.ToLowerInvariant()
        repository_commit = (& git -C $repositoryRoot rev-parse HEAD 2>$null)
        repository_dirty = [bool](@(& git -C $repositoryRoot status --porcelain 2>$null).Count -gt 0)
        started_utc = $startedUtc.ToString('O')
        ended_utc = $endedUtc.ToString('O')
        requested_duration_seconds = $DurationSeconds
        actual_duration_ms = [uint64]$stopwatch.ElapsedMilliseconds
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
            dtr_enabled = $false
            rts_enabled = $false
        }
        safety = [ordered]@{
            non_status_commands_allowed = [bool]$AllowNonStatusCommands
        }
        counts = [ordered]@{
            raw_bytes = $rawByteCount
            complete_lines = $completeLineCount
            telemetry_rows = $telemetryRowCount
            telemetry_parse_errors = $telemetryParseErrorCount
            motor_capture_rows = $motorCaptureRowCount
            motor_capture_events = $motorCaptureEventCount
            motor_capture_parse_errors = $motorCaptureParseErrorCount
            non_telemetry_lines = $nonTelemetryLineCount
            commands_scheduled = $schedule.Count
            commands_sent = $commandSentCount
            trailing_partial_characters = $pendingText.Length
        }
        artifacts = [ordered]@{
            raw_uart_log = 'raw_uart.log'
            telemetry_csv = 'telemetry.csv'
            motor_capture_csv = 'motor_capture.csv'
            motor_capture_events_csv = 'motor_capture_events.csv'
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
    MotorCaptureRows = $motorCaptureRowCount
    MotorCaptureEvents = $motorCaptureEventCount
    MotorCaptureParseErrors = $motorCaptureParseErrorCount
    CommandsSent = $commandSentCount
}
