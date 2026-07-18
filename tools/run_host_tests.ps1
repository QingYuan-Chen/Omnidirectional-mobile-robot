param(
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'

function Find-Executable {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Name,
        [Parameter(Mandatory = $true)]
        [string[]]$Candidates
    )

    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    foreach ($candidate in $Candidates) {
        if (-not [string]::IsNullOrWhiteSpace($candidate) -and
            (Test-Path -LiteralPath $candidate -PathType Leaf)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    throw "未找到可执行文件：$Name"
}

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,
        [Parameter(Mandatory = $true)]
        [string[]]$ArgumentList
    )

    & $FilePath @ArgumentList
    if ($LASTEXITCODE -ne 0) {
        throw "命令执行失败，退出码 $LASTEXITCODE：$FilePath $($ArgumentList -join ' ')"
    }
}

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$testSourceDirectory = Join-Path $repositoryRoot 'tests'
$testBuildDirectory = Join-Path $repositoryRoot 'build\HostTests'

if ($Clean -and (Test-Path -LiteralPath $testBuildDirectory)) {
    $resolvedBuildDirectory = (Resolve-Path -LiteralPath $testBuildDirectory).Path
    $resolvedBuildRoot = (Resolve-Path -LiteralPath (Join-Path $repositoryRoot 'build')).Path
    if (-not $resolvedBuildDirectory.StartsWith(
            $resolvedBuildRoot + [IO.Path]::DirectorySeparatorChar,
            [StringComparison]::OrdinalIgnoreCase)) {
        throw "拒绝清理仓库 build 目录以外的路径：$resolvedBuildDirectory"
    }
    Remove-Item -LiteralPath $resolvedBuildDirectory -Recurse -Force
}

$cmake = Find-Executable -Name 'cmake' -Candidates @(
    'D:\STM32CubeCLT_1.20.0\CMake\bin\cmake.exe'
)
$ctest = Join-Path (Split-Path -Parent $cmake) 'ctest.exe'
if (-not (Test-Path -LiteralPath $ctest -PathType Leaf)) {
    throw "未找到与 CMake 配套的 ctest：$ctest"
}
$ninja = Find-Executable -Name 'ninja' -Candidates @(
    'C:\mingw64\bin\ninja.exe',
    'D:\STM32CubeCLT_1.20.0\Ninja\bin\ninja.exe'
)
$gccCandidates = @('C:\mingw64\bin\gcc.exe')
if (-not [string]::IsNullOrWhiteSpace($env:CC)) {
    $gccCandidates = @($env:CC) + $gccCandidates
}
$gcc = Find-Executable -Name 'gcc' -Candidates $gccCandidates

Write-Host "主机测试编译器：$gcc"
Write-Host "主机测试构建器：$ninja"

Invoke-Checked -FilePath $cmake -ArgumentList @(
    '-S', $testSourceDirectory,
    '-B', $testBuildDirectory,
    '-G', 'Ninja',
    "-DCMAKE_C_COMPILER=$($gcc.Replace('\', '/'))",
    "-DCMAKE_MAKE_PROGRAM=$($ninja.Replace('\', '/'))"
)
Invoke-Checked -FilePath $cmake -ArgumentList @(
    '--build', $testBuildDirectory,
    '--clean-first'
)
Invoke-Checked -FilePath $ctest -ArgumentList @(
    '--test-dir', $testBuildDirectory,
    '--no-tests=error',
    '--output-on-failure'
)
