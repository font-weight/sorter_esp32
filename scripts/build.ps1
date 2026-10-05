<#
Compiles five sketches, never uploads or opens a serial port.
  ./scripts/build.ps1 -InstallCore       # first use; downloads ESP32 core/tools
  ./scripts/build.ps1                    # uses installed local core
  ./scripts/build.ps1 -ArduinoCli C:/Tools/arduino-cli.exe -Sketch camera_node
  ./scripts/build.ps1 -BuildRoot "$env:TEMP/sorter32" -InstallCore # short Windows SDK path
Board profiles below are compile targets, not detected physical boards.
#>
[CmdletBinding()]
param(
    [string]$ArduinoCli = '',
    [string]$BuildRoot = '',
    [switch]$InstallCore,
    [ValidateSet('all','camera_diagnostic','camera_node','servo_diagnostic','motion_node','arm_calibration')]
    [string]$Sketch = 'all',
    [int]$Jobs = 2
)
$ErrorActionPreference = 'Stop'
$sorterRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildRoot = if ($BuildRoot) { [IO.Path]::GetFullPath($BuildRoot) } else { Join-Path $sorterRoot '.build' }
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$results = @()
$buildRecord = [ordered]@{status='running'; started_utc=[DateTime]::UtcNow.ToString('o');
    finished_utc=$null; expected_core='esp32:esp32@3.3.12'; core=$null; uploaded=$false;
    requested_sketch=$Sketch; results=@(); error=$null}
function Save-BuildState([string]$State, [string]$ErrorText = '') {
    $buildRecord.status = $State
    $buildRecord.results = $results
    if ($State -ne 'running') { $buildRecord.finished_utc = [DateTime]::UtcNow.ToString('o') }
    if ($ErrorText) { $buildRecord.error = $ErrorText }
    $buildRecord | ConvertTo-Json -Depth 5 |
        Set-Content -LiteralPath (Join-Path $buildRoot 'build-results.json') -Encoding utf8
}
Save-BuildState 'running'
try {
if (-not $ArduinoCli) {
    $cliCommand = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($cliCommand) { $ArduinoCli = $cliCommand.Source }
    else {
        $ArduinoCli = Join-Path $env:ProgramFiles 'Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'
        if (-not (Test-Path -LiteralPath $ArduinoCli)) { throw 'Pass -ArduinoCli with the path to arduino-cli.exe' }
    }
}
if ($Jobs -lt 1 -or $Jobs -gt 16) { throw 'Jobs must be 1..16' }
$config = Join-Path $buildRoot 'arduino-cli.yaml'
# YAML single quotes are escaped by doubling, not by shell interpolation.
$dataDir = (Join-Path $buildRoot 'arduino-data').Replace("'", "''")
$downloadDir = (Join-Path $buildRoot 'downloads').Replace("'", "''")
$userDir = (Join-Path $buildRoot 'sketchbook').Replace("'", "''")
@"
board_manager:
  additional_urls:
    - https://espressif.github.io/arduino-esp32/package_esp32_index.json
directories:
  data: '$dataDir'
  downloads: '$downloadDir'
  user: '$userDir'
"@ | Set-Content -LiteralPath $config -Encoding utf8
if ($InstallCore) {
    & $ArduinoCli --config-file $config core update-index
    if ($LASTEXITCODE -ne 0) { throw 'Arduino index update failed' }
    & $ArduinoCli --config-file $config core install 'esp32:esp32@3.3.12'
    if ($LASTEXITCODE -ne 0) { throw 'ESP32 core installation failed' }
}
$installedJson = & $ArduinoCli --config-file $config core list --format json
if ($LASTEXITCODE -ne 0) { throw 'Unable to verify the installed ESP32 core.' }
$installed = @(($installedJson | ConvertFrom-Json).platforms | Where-Object id -eq 'esp32:esp32')
if ($installed.Count -ne 1 -or $installed[0].installed_version -ne '3.3.12') {
    throw 'This project requires Arduino-ESP32 3.3.12. Run with -InstallCore or the documented subset installer.'
}
$buildRecord.core = 'esp32:esp32@' + $installed[0].installed_version
$targets = [ordered]@{
    camera_diagnostic='esp32:esp32:esp32cam'; camera_node='esp32:esp32:esp32cam';
    servo_diagnostic='esp32:esp32:esp32'; motion_node='esp32:esp32:esp32'; arm_calibration='esp32:esp32:esp32'
}
foreach ($name in $targets.Keys) {
    if ($Sketch -ne 'all' -and $Sketch -ne $name) { continue }
    Write-Host "Compile $name for $($targets[$name])"
    & $ArduinoCli --config-file $config compile --fqbn $targets[$name] `
        --library (Join-Path $sorterRoot 'libraries/SorterCore') --warnings all --jobs $Jobs `
        --build-path (Join-Path $buildRoot "arduino/$name") `
        --output-dir (Join-Path $buildRoot "firmware/$name") `
        (Join-Path $sorterRoot "firmware/$name")
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $name. On first use pass -InstallCore." }
    $results += [ordered]@{sketch=$name; fqbn=$targets[$name]; compiled=$true}
}
Save-BuildState 'passed'
Write-Host 'Compilation complete. Nothing uploaded; board identities still require confirmation.'
} catch {
    Save-BuildState 'failed' $_.Exception.Message
    throw
}
