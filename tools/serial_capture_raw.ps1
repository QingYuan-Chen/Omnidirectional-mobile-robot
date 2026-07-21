function Read-SerialCaptureRawLine {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$RawPath,

        [Parameter(Mandatory = $true)]
        [string]$TimingPath
    )

    $resolvedRawPath = (Resolve-Path -LiteralPath $RawPath).Path
    $resolvedTimingPath = (Resolve-Path -LiteralPath $TimingPath).Path
    $rawLength = [uint64](Get-Item -LiteralPath $resolvedRawPath).Length
    $timingLength = [uint64](Get-Item -LiteralPath $resolvedTimingPath).Length
    $timingRecordBytes = [uint64]24

    if (($timingLength % $timingRecordBytes) -ne 0) {
        throw "串口块时刻文件长度不是 24 字节记录的整数倍：$timingLength"
    }
    if (($rawLength -eq 0) -ne ($timingLength -eq 0)) {
        throw '原始串口文件与块时刻文件的空文件状态不一致'
    }

    $rawStream = $null
    $timingStream = $null
    $timingReader = $null
    $pendingText = [Text.StringBuilder]::new()
    $rawBuffer = [byte[]]::new(65536)
    $consumedByteCount = [uint64]0
    $timingRecordCount = [uint64]0
    $currentTimingEndOffset = [uint64]0
    $currentTimingElapsedMs = [uint64]0
    $currentTimingUtcTicks = [int64]0
    $previousTimingEndOffset = [uint64]0
    $previousTimingElapsedMs = [uint64]0
    $previousTimingUtcTicks = [int64]0

    try {
        $rawStream = [IO.File]::OpenRead($resolvedRawPath)
        $timingStream = [IO.File]::OpenRead($resolvedTimingPath)
        $timingReader = [IO.BinaryReader]::new($timingStream)

        if ($rawLength -gt 0) {
            $currentTimingEndOffset = $timingReader.ReadUInt64()
            $currentTimingElapsedMs = $timingReader.ReadUInt64()
            $currentTimingUtcTicks = $timingReader.ReadInt64()
            $timingRecordCount++
            if ($currentTimingEndOffset -eq 0 -or
                $currentTimingEndOffset -gt $rawLength) {
                throw "首个串口块结束偏移非法：$currentTimingEndOffset / $rawLength"
            }
            if ($currentTimingUtcTicks -lt [DateTime]::MinValue.Ticks -or
                $currentTimingUtcTicks -gt [DateTime]::MaxValue.Ticks) {
                throw "首个串口块 UTC ticks 非法：$currentTimingUtcTicks"
            }
            $previousTimingEndOffset = $currentTimingEndOffset
            $previousTimingElapsedMs = $currentTimingElapsedMs
            $previousTimingUtcTicks = $currentTimingUtcTicks
        }

        while (($readCount = $rawStream.Read($rawBuffer, 0, $rawBuffer.Length)) -gt 0) {
            [void]$pendingText.Append(
                [Text.Encoding]::ASCII.GetString($rawBuffer, 0, $readCount))

            while ($true) {
                $pendingSnapshot = $pendingText.ToString()
                $lineEnd = $pendingSnapshot.IndexOf("`n", [StringComparison]::Ordinal)
                if ($lineEnd -lt 0) {
                    break
                }

                $consumedByteCount += [uint64]($lineEnd + 1)
                while ($currentTimingEndOffset -lt $consumedByteCount) {
                    if ($timingStream.Position -ge $timingStream.Length) {
                        throw "串口块时刻未覆盖原始行结束偏移：$consumedByteCount"
                    }
                    $currentTimingEndOffset = $timingReader.ReadUInt64()
                    $currentTimingElapsedMs = $timingReader.ReadUInt64()
                    $currentTimingUtcTicks = $timingReader.ReadInt64()
                    $timingRecordCount++
                    if ($currentTimingEndOffset -le $previousTimingEndOffset -or
                        $currentTimingEndOffset -gt $rawLength) {
                        throw "串口块结束偏移不递增或越界：$currentTimingEndOffset"
                    }
                    if ($currentTimingElapsedMs -lt $previousTimingElapsedMs) {
                        throw "串口块 elapsed_ms 倒退：$currentTimingElapsedMs"
                    }
                    if ($currentTimingUtcTicks -lt $previousTimingUtcTicks -or
                        $currentTimingUtcTicks -gt [DateTime]::MaxValue.Ticks) {
                        throw "串口块 UTC ticks 倒退或越界：$currentTimingUtcTicks"
                    }
                    $previousTimingEndOffset = $currentTimingEndOffset
                    $previousTimingElapsedMs = $currentTimingElapsedMs
                    $previousTimingUtcTicks = $currentTimingUtcTicks
                }

                $line = $pendingSnapshot.Substring(0, $lineEnd).TrimEnd([char]13)
                [void]$pendingText.Remove(0, $lineEnd + 1)
                [pscustomobject]@{
                    kind = 'line'
                    line = $line
                    capture_elapsed_ms = $currentTimingElapsedMs
                    host_received_utc = [DateTimeOffset]::new(
                        $currentTimingUtcTicks,
                        [TimeSpan]::Zero).ToString('O')
                }
            }
        }

        while ($timingStream.Position -lt $timingStream.Length) {
            $currentTimingEndOffset = $timingReader.ReadUInt64()
            $currentTimingElapsedMs = $timingReader.ReadUInt64()
            $currentTimingUtcTicks = $timingReader.ReadInt64()
            $timingRecordCount++
            if ($currentTimingEndOffset -le $previousTimingEndOffset -or
                $currentTimingEndOffset -gt $rawLength) {
                throw "串口块结束偏移不递增或越界：$currentTimingEndOffset"
            }
            if ($currentTimingElapsedMs -lt $previousTimingElapsedMs) {
                throw "串口块 elapsed_ms 倒退：$currentTimingElapsedMs"
            }
            if ($currentTimingUtcTicks -lt $previousTimingUtcTicks -or
                $currentTimingUtcTicks -gt [DateTime]::MaxValue.Ticks) {
                throw "串口块 UTC ticks 倒退或越界：$currentTimingUtcTicks"
            }
            $previousTimingEndOffset = $currentTimingEndOffset
            $previousTimingElapsedMs = $currentTimingElapsedMs
            $previousTimingUtcTicks = $currentTimingUtcTicks
        }

        if ($rawLength -gt 0 -and $currentTimingEndOffset -ne $rawLength) {
            throw "串口块结束偏移未闭合原始字节数：$currentTimingEndOffset / $rawLength"
        }

        [pscustomobject]@{
            kind = 'summary'
            raw_bytes = $rawLength
            timing_records = $timingRecordCount
            trailing_partial_characters = [uint64]$pendingText.Length
        }
    } finally {
        if ($null -ne $timingReader) {
            $timingReader.Dispose()
        } elseif ($null -ne $timingStream) {
            $timingStream.Dispose()
        }
        if ($null -ne $rawStream) {
            $rawStream.Dispose()
        }
    }
}
