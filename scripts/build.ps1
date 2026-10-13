# Compiles production firmware only. Never uploads or opens a serial port.
[CmdletBinding()]
param(
    [string]$ArduinoCli = '',
    [string]$ConfigFile = (Join-Path $env:USERPROFILE '.arduinoIDE/arduino-cli.yaml'),
    [string]$BuildRoot = (Join-Path $env:TEMP 'sorter-final-build'),
    [ValidateSet('all','sorter_node','camera_node')][string]$Sketch = 'all',
    [int]$Jobs = 2
)
$ErrorActionPreference = 'Stop'
$sorterRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$sorterOutput = Join-Path $sorterRoot '.build'
New-Item -ItemType Directory -Path $sorterOutput -Force | Out-Null
$sorterReport = [ordered]@{status='running'; core='esp32:esp32@3.3.12'; uploaded=$false; results=@(); error=$null}
$sorterReportPath = Join-Path $sorterOutput 'build-results.json'
function Save-Report {
    $sorterReport | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $sorterReportPath -Encoding utf8
}
Save-Report
try {
    if (-not $ArduinoCli) {
        $sorterCliCommand = Get-Command arduino-cli -ErrorAction SilentlyContinue
        if ($sorterCliCommand) { $ArduinoCli = $sorterCliCommand.Source }
        else { $ArduinoCli = Join-Path $env:ProgramFiles 'Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe' }
    }
    if (-not (Test-Path -LiteralPath $ArduinoCli -PathType Leaf)) { throw 'Arduino CLI not found; specify -ArduinoCli.' }
    if (-not (Test-Path -LiteralPath $ConfigFile -PathType Leaf)) { throw 'Arduino CLI configuration not found; specify -ConfigFile.' }
    if ($Jobs -lt 1 -or $Jobs -gt 16) { throw 'Jobs must be between 1 and 16.' }
    $sorterCoreJson = & $ArduinoCli --config-file $ConfigFile core list --format json
    if ($LASTEXITCODE -ne 0) { throw 'Could not read installed Arduino cores.' }
    $sorterCore = @(($sorterCoreJson | ConvertFrom-Json).platforms | Where-Object { $_.id -eq 'esp32:esp32' })
    if ($sorterCore.Count -ne 1 -or $sorterCore[0].installed_version -ne '3.3.12') {
        throw 'Install esp32 by Espressif Systems version 3.3.12 in Arduino IDE first.'
    }
    $sorterBoards = [ordered]@{sorter_node='esp32:esp32:esp32'; camera_node='esp32:esp32:esp32cam'}
    foreach ($sorterName in $sorterBoards.Keys) {
        if ($Sketch -ne 'all' -and $Sketch -ne $sorterName) { continue }
        $sorterArgs = @('--config-file', $ConfigFile, 'compile', '--fqbn', $sorterBoards[$sorterName],
            '--library', (Join-Path $sorterRoot 'libraries/SorterCore'), '--warnings', 'all', '--jobs', $Jobs,
            '--build-path', (Join-Path $BuildRoot $sorterName),
            '--output-dir', (Join-Path $sorterOutput "firmware/$sorterName"),
            (Join-Path $sorterRoot "firmware/$sorterName"))
        & $ArduinoCli @sorterArgs
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $sorterName" }
        $sorterReport.results += [ordered]@{sketch=$sorterName; fqbn=$sorterBoards[$sorterName]; status='passed'}
        Save-Report
    }
    $sorterReport.status = 'passed'
    Save-Report
    Write-Output 'Requested compile checks completed. No board was flashed.'
} catch {
    $sorterReport.status = 'failed'
    $sorterReport.error = $_.Exception.Message
    Save-Report
    throw
}
