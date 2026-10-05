<#
Compile-only link check of hardware paths that false defaults may optimize out.
Creates modified copies under BuildRoot, never edits delivered firmware, never
uploads, and never opens a serial port. Generated copies/binaries MUST NOT be
flashed: their confirmations are compiler-test placeholders, not measurements.
Run after build.ps1 has compiled the default source profiles.
#>
[CmdletBinding()]
param(
    [string]$ArduinoCli = '',
    [string]$BuildRoot = '',
    [int]$Jobs = 2
)
$ErrorActionPreference = 'Stop'
$sorterRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildRoot = if ($BuildRoot) { [IO.Path]::GetFullPath($BuildRoot) } else { Join-Path $sorterRoot '.build' }
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$results = @()
$buildRecord = [ordered]@{status='running'; started_utc=[DateTime]::UtcNow.ToString('o');
    finished_utc=$null; expected_core='esp32:esp32@3.3.12'; core=$null;
    profile='compile-only hardware paths; do not flash'; uploaded=$false; results=@(); error=$null}
function Save-BuildState([string]$State, [string]$ErrorText = '') {
    $buildRecord.status = $State
    $buildRecord.results = $results
    if ($State -ne 'running') { $buildRecord.finished_utc = [DateTime]::UtcNow.ToString('o') }
    if ($ErrorText) { $buildRecord.error = $ErrorText }
    $buildRecord | ConvertTo-Json -Depth 6 |
        Set-Content -LiteralPath (Join-Path $buildRoot 'build-hardware-paths.json') -Encoding utf8
}
Save-BuildState 'running'
try {
if (-not $ArduinoCli) {
    $cliCommand = Get-Command arduino-cli -ErrorAction SilentlyContinue
    $ArduinoCli = if ($cliCommand) { $cliCommand.Source } else {
        Join-Path $env:ProgramFiles 'Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'
    }
}
$config = Join-Path $buildRoot 'arduino-cli.yaml'
if (-not (Test-Path -LiteralPath $config)) { throw 'Run build.ps1 for this BuildRoot first.' }
if ($Jobs -lt 1 -or $Jobs -gt 16) { throw 'Jobs must be 1..16' }
$installedJson = & $ArduinoCli --config-file $config core list --format json
if ($LASTEXITCODE -ne 0) { throw 'Unable to verify the installed ESP32 core.' }
$installed = @(($installedJson | ConvertFrom-Json).platforms | Where-Object id -eq 'esp32:esp32')
if ($installed.Count -ne 1 -or $installed[0].installed_version -ne '3.3.12') {
    throw 'Hardware-path compile check requires Arduino-ESP32 3.3.12.'
}
$buildRecord.core = 'esp32:esp32@' + $installed[0].installed_version
$runRoot = Join-Path $buildRoot ('hardware-paths-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss'))
if (Test-Path -LiteralPath $runRoot) { throw 'Compile-only directory already exists; wait a second and retry.' }
New-Item -ItemType Directory -Path $runRoot | Out-Null
'COMPILER TEST COPIES ONLY. DO NOT FLASH. All enabled confirmation flags are unmeasured placeholders.' |
    Set-Content -LiteralPath (Join-Path $runRoot 'DO_NOT_FLASH.txt') -Encoding utf8
$profiles = @(
    @{name='camera_node'; board='esp32:esp32:esp32cam'; file='CameraConfig.h'; flags=@{
        CAMERA_PROFILE_CONFIRMED=@('false','true'); CAMERA_UART_PINS_CONFIRMED=@('false','true');
        VISION_CONFIG_CONFIRMED=@('false','true')}},
    @{name='motion_node'; board='esp32:esp32:esp32'; file='LocalConfig.h'; flags=@{
        VIRTUAL_MODE=@('true','false'); MOTOR_OUTPUTS_ENABLED=@('false','true');
        PINOUT_CONFIRMED=@('false','true'); UART_PROFILE_CONFIRMED=@('false','true')}},
    @{name='arm_calibration'; board='esp32:esp32:esp32'; file='LocalConfig.h'; flags=@{
        OUTPUTS_ENABLED=@('false','true'); PINOUT_CONFIRMED=@('false','true');
        LIMITS_CONFIRMED=@('false','true')}}
)
foreach ($p in $profiles) {
    $copy = Join-Path $runRoot $p.name
    Copy-Item -LiteralPath (Join-Path $sorterRoot ('firmware/' + $p.name)) -Destination $copy -Recurse
    $settings = Join-Path $copy $p.file
    $content = Get-Content -LiteralPath $settings -Raw
    foreach ($flag in $p.flags.Keys) {
        $old = 'constexpr bool ' + $flag + ' = ' + $p.flags[$flag][0] + ';'
        $new = 'constexpr bool ' + $flag + ' = ' + $p.flags[$flag][1] + ';'
        if ([regex]::Matches($content, [regex]::Escape($old)).Count -ne 1) {
            throw ('Expected exactly one guarded default: ' + $flag)
        }
        $content = $content.Replace($old, $new)
    }
    Set-Content -LiteralPath $settings -Value $content -Encoding utf8
    $targetBuild = Join-Path $runRoot ('build-' + $p.name)
    Write-Host ('COMPILE-ONLY hardware paths: ' + $p.name)
    & $ArduinoCli --config-file $config compile --fqbn $p.board `
        --library (Join-Path $sorterRoot 'libraries/SorterCore') --warnings all --jobs $Jobs `
        --build-path $targetBuild $copy
    if ($LASTEXITCODE -ne 0) { throw ('Compile-only hardware path failed: ' + $p.name) }
    $results += [ordered]@{sketch=$p.name; fqbn=$p.board; compiled=$true; flags=$p.flags;
        build_directory=$targetBuild; uploaded=$false}
}
Save-BuildState 'passed'
Write-Host 'Hardware-path compilation complete. Source defaults unchanged. Nothing uploaded.'
} catch {
    Save-BuildState 'failed' $_.Exception.Message
    throw
}
