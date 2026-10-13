# Offline tests only: no board connections or motor commands.
[CmdletBinding()]
param([string]$ZigPath = '')
$ErrorActionPreference = 'Stop'
$sorterRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$sorterOutput = Join-Path $sorterRoot '.build'
New-Item -ItemType Directory -Path $sorterOutput -Force | Out-Null
$env:ZIG_GLOBAL_CACHE_DIR = Join-Path $sorterOutput 'zig-global'
$env:ZIG_LOCAL_CACHE_DIR = Join-Path $sorterOutput 'zig-local'
$sorterReport = [ordered]@{status='running'; hardware_tested=$false; results=@(); error=$null}
$sorterReportPath = Join-Path $sorterOutput 'test-results.json'
function Save-Report {
    $sorterReport | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $sorterReportPath -Encoding utf8
}
Save-Report
try {
    if (-not $ZigPath) {
        $sorterZigCommand = Get-Command zig -ErrorAction SilentlyContinue
        if ($sorterZigCommand) { $ZigPath = $sorterZigCommand.Source }
    }
    if (-not $ZigPath -or -not (Test-Path -LiteralPath $ZigPath -PathType Leaf)) { throw 'Zig not found; specify -ZigPath.' }
    foreach ($sorterSuite in @('protocol','vision','jog','sorter')) {
        $sorterExe = Join-Path $sorterOutput "test_$sorterSuite.exe"
        $sorterArgs = @('c++','-target','x86_64-windows-gnu','-std=c++17','-Wall','-Wextra','-Werror',
            '-I',(Join-Path $sorterRoot 'libraries/SorterCore/src'))
        if ($sorterSuite -eq 'sorter') { $sorterArgs += @('-I',(Join-Path $sorterRoot 'tests/arduino_fake')) }
        $sorterArgs += @((Join-Path $sorterRoot "tests/test_$sorterSuite.cpp"),'-o',$sorterExe)
        & $ZigPath @sorterArgs
        if ($LASTEXITCODE -ne 0) { throw "Test compilation failed: $sorterSuite" }
        & $sorterExe
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $sorterSuite" }
        $sorterReport.results += [ordered]@{suite=$sorterSuite; status='passed'}
        Save-Report
    }
    $sorterReport.status = 'passed'
    Save-Report
    Write-Output 'All four test suites passed. Hardware was not operated.'
} catch {
    $sorterReport.status = 'failed'
    $sorterReport.error = $_.Exception.Message
    Save-Report
    throw
}
