$ErrorActionPreference = 'Stop'

. (Join-Path $PSScriptRoot '..\tools\serial_capture_lib.ps1')

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

    $script:assertionCount++
    try {
        & $Action
    } catch {
        return
    }
    throw "断言失败，预期抛出异常：$Message"
}

$validLine = 'T,178946,176930,3954438795,R,2,65535,1,2,D,0,-1,32767,-32768,E,0,-2,9223372036854775807,-9223372036854775808,P,-840,840,B,4192,S,1,1,0,0,0,0,I,0,1,J,0,225,L,2396,11916,M,0,0,C,0,0,0,0,0,0,0,0'
$parsed = ConvertFrom-AppTelemetryLine -Line ($validLine + "`r")

Assert-True ($parsed.board_now_ms -eq 178946) '解析板端时间'
Assert-True ($parsed.encoder_raw_2 -eq 65535) '解析 uint16 上界'
Assert-True ($parsed.encoder_delta_4 -eq -32768) '解析 int16 下界'
Assert-True ($parsed.encoder_total_3 -eq [int64]::MaxValue) '解析 int64 上界'
Assert-True ($parsed.target_pwm -eq -840 -and $parsed.applied_pwm -eq 840) '解析有符号 PWM'
Assert-True ((Get-AppTelemetryColumnNames).Count -eq 40) 'CSV 遥测列数量固定'

$validMotorCaptureLine = 'MC,1,7,17,1176000,27,47,65535,-32768,-840,840,11600,6,31'
$motorCapture = ConvertFrom-MotorCaptureSampleLine -Line $validMotorCaptureLine
Assert-True ($motorCapture.capture_index -eq 7) '解析高速采集索引'
Assert-True ($motorCapture.encoder_raw_ma -eq 65535 -and
             $motorCapture.encoder_delta_ma -eq -32768) '解析高速编码器边界'
Assert-True ($motorCapture.target_pwm -eq -840 -and
             $motorCapture.applied_pwm -eq 840) '解析高速 PWM'
Assert-True ($motorCapture.wake_latency_cycles -eq 27 -and
             $motorCapture.previous_wcet_cycles -eq 47) '解析完整时序字段'
Assert-True ((Get-MotorCaptureColumnNames).Count -eq 12) '高速采集 CSV 列数量固定'
Assert-Throws {
    ConvertFrom-MotorCaptureSampleLine -Line ($validMotorCaptureLine + ',0')
} '拒绝高速采集额外字段'
Assert-Throws {
    ConvertFrom-MotorCaptureSampleLine -Line ($validMotorCaptureLine -replace '^MC,1', 'MC,2')
} '拒绝未知高速采集版本'
Assert-Throws {
    ConvertFrom-MotorCaptureSampleLine -Line ($validMotorCaptureLine -replace ',6,31$', ',7,31')
} '拒绝未知电机状态'

$motorCaptureEvent = ConvertFrom-MotorCaptureEventLine -Line 'MCAP,1,BEGIN,2,2200,2200,1'
Assert-True ($motorCaptureEvent.event -ceq 'BEGIN' -and
             $motorCaptureEvent.state -eq 2 -and
             $motorCaptureEvent.sample_count -eq 2200 -and
             $motorCaptureEvent.dropped_sample_count -eq 1) '解析高速采集事件'
Assert-True ((Get-MotorCaptureEventColumnNames).Count -eq 5) '高速事件 CSV 列数量固定'
Assert-Throws {
    ConvertFrom-MotorCaptureEventLine -Line 'MCAP,1,UNKNOWN,2,0,2200,0'
} '拒绝未知高速采集事件'

Assert-Throws { ConvertFrom-AppTelemetryLine -Line ($validLine + ',0') } '拒绝额外字段'
Assert-Throws { ConvertFrom-AppTelemetryLine -Line ($validLine -replace ',R,', ',X,') } '拒绝错误标签'
Assert-Throws { ConvertFrom-AppTelemetryLine -Line ($validLine -replace '^T,178946', 'T,-1') } '拒绝负无符号数'
Assert-Throws { ConvertFrom-AppTelemetryLine -Line ($validLine -replace ',S,1,1,0,0,0,0,', ',S,2,1,0,0,0,0,') } '拒绝非法布尔值'

$csvLine = ConvertTo-CaptureCsvLine -Values @('plain', 'a"b', $null)
Assert-True ($csvLine -ceq '"plain","a""b",""') 'CSV 引号转义'

$statusSchedule = @(Get-SerialCaptureCommandSchedule `
    -Path '' `
    -DurationMilliseconds 1000 `
    -SendStatusAtStart)
Assert-True ($statusSchedule.Count -eq 1 -and
             $statusSchedule[0].command -ceq 'STATUS' -and
             $statusSchedule[0].planned_elapsed_ms -eq 0) '生成启动 STATUS 计划'

Assert-Throws {
    Assert-SerialCaptureCommand -Command 'PWM 1 10'
} '默认拒绝非 STATUS 命令'

Assert-SerialCaptureCommand -Command 'PWM 1 10' -AllowNonStatusCommands
$assertionCount++

$temporaryDirectory = Join-Path ([IO.Path]::GetTempPath()) ('serial-capture-test-' + [Guid]::NewGuid().ToString('N'))
[void][IO.Directory]::CreateDirectory($temporaryDirectory)
try {
    $schedulePath = Join-Path $temporaryDirectory 'schedule.csv'
    [IO.File]::WriteAllText(
        $schedulePath,
        "elapsed_ms,command`r`n200,STATUS`r`n100,STATUS`r`n",
        [Text.UTF8Encoding]::new($false))
    $fileSchedule = @(Get-SerialCaptureCommandSchedule `
        -Path $schedulePath `
        -DurationMilliseconds 1000)
    Assert-True ($fileSchedule.Count -eq 2 -and
                 $fileSchedule[0].planned_elapsed_ms -eq 100 -and
                 $fileSchedule[1].planned_elapsed_ms -eq 200) '命令计划按时间排序'
} finally {
    if (Test-Path -LiteralPath $temporaryDirectory -PathType Container) {
        Remove-Item -LiteralPath $temporaryDirectory -Recurse -Force
    }
}

Write-Host "serial_capture_tools：$assertionCount 项断言通过"
