<#
Offline checks only; no serial ports or motor commands.
Example (from sorter):
  ./scripts/test.ps1 -PythonPath python -ZigPath C:/Tools/zig/zig.exe
Requires Python 3.10+ with numpy/Pillow, Zig 0.13.0 (C++17 frontend), and Node.js.
All generated files are kept under .build. No downloads are automatic.
#>
[CmdletBinding()]
param([string]$PythonPath = 'python', [string]$ZigPath = '', [string]$NodePath = 'node')
$ErrorActionPreference = 'Stop'
$sorterRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildRoot = Join-Path $sorterRoot '.build'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$env:ZIG_GLOBAL_CACHE_DIR = Join-Path $buildRoot 'zig-global'
$env:ZIG_LOCAL_CACHE_DIR = Join-Path $buildRoot 'zig-local'
$start = [DateTime]::UtcNow
$results = @()
$reportPath = Join-Path $buildRoot 'test-results.json'
$pipelineReport = Join-Path $buildRoot 'pipeline_run.json'
function Write-TestReport([string]$Status, [string]$Failure = '') {
    [ordered]@{status=$Status; started_utc=$start.ToString('o'); updated_utc=[DateTime]::UtcNow.ToString('o');
        physical_hardware_tested=$false; failure=$Failure; results=$results} | ConvertTo-Json -Depth 4 |
        Set-Content -LiteralPath $reportPath -Encoding utf8
}
Write-TestReport 'running'
# Replace an old successful simulation record before attempting the new run.
[ordered]@{status='pending'; hardware_executed=$false; started_utc=$start.ToString('o')} |
    ConvertTo-Json | Set-Content -LiteralPath $pipelineReport -Encoding utf8
Push-Location $sorterRoot
try {
    if (-not $ZigPath) {
        $localZig = Join-Path $sorterRoot '.tools/zig-windows-x86_64-0.13.0/zig.exe'
        if (Test-Path -LiteralPath $localZig) { $ZigPath = $localZig }
        else { $ZigPath = (Get-Command zig -ErrorAction Stop).Source }
    }
    foreach ($suite in @('protocol', 'vision', 'motion', 'jog', 'pipeline')) {
        Write-Host "Compile native $suite"
        $exe = Join-Path $buildRoot "test_$suite.exe"
        & $ZigPath c++ -target x86_64-windows-gnu -std=c++17 -Wall -Wextra -Werror `
            -I (Join-Path $sorterRoot 'libraries/SorterCore/src') `
            (Join-Path $sorterRoot "tests/test_$suite.cpp") -o $exe
        if ($LASTEXITCODE -ne 0) { throw "Native compilation failed: $suite" }
        if ($suite -eq 'pipeline') { & $exe --report $pipelineReport }
        else { & $exe }
        if ($LASTEXITCODE -ne 0) { throw "Native test failed: $suite" }
        $results += [ordered]@{name="native_$suite"; passed=$true}
        Write-TestReport 'running'
    }
    $bridge = Join-Path $buildRoot 'protocol_bridge.exe'
    & $ZigPath c++ -target x86_64-windows-gnu -std=c++17 -Wall -Wextra -Werror `
        -I (Join-Path $sorterRoot 'libraries/SorterCore/src') `
        (Join-Path $sorterRoot 'tests/protocol_bridge.cpp') -o $bridge
    if ($LASTEXITCODE -ne 0) { throw 'Protocol bridge compilation failed' }
    & $PythonPath tests/check_protocol_interop.py $bridge
    if ($LASTEXITCODE -ne 0) { throw 'Protocol interoperability failed' }
    $results += [ordered]@{name='protocol_interop'; passed=$true}
    & $PythonPath -m unittest discover -s tests -p test_tools.py -v
    if ($LASTEXITCODE -ne 0) { throw 'Python tests failed' }
    $results += [ordered]@{name='python_tools'; passed=$true}
    & $NodePath tools/check_picker.js
    if ($LASTEXITCODE -ne 0) { throw 'Picker JavaScript check failed' }
    $results += [ordered]@{name='picker_javascript'; passed=$true}
    Write-TestReport 'passed'
    Write-Host 'All offline checks passed. No physical hardware was tested.'
} catch {
    Write-TestReport 'failed' $_.Exception.Message
    throw
} finally { Pop-Location }
