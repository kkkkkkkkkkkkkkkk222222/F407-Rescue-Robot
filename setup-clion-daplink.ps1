[CmdletBinding()]
param(
    [string]$OpenOcd,
    [string]$ClionOptions
)

$ErrorActionPreference = 'Stop'

if (Get-Process clion64 -ErrorAction SilentlyContinue) {
    throw 'Close CLion before running this script.'
}

$projectRoot = (Resolve-Path $PSScriptRoot).Path
$dapConfig = Join-Path $projectRoot 'dap.cfg'

if (-not (Test-Path -LiteralPath $dapConfig)) {
    throw "Missing $dapConfig. Extract the complete package into the project root."
}

function Find-Tool {
    param(
        [string]$ExplicitPath,
        [string]$CommandName,
        [string[]]$CandidatePatterns
    )

    if ($ExplicitPath) {
        if (Test-Path -LiteralPath $ExplicitPath -PathType Leaf) {
            return (Resolve-Path -LiteralPath $ExplicitPath).Path
        }
        throw "Tool does not exist: $ExplicitPath"
    }

    $command = Get-Command $CommandName -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($command) {
        return $command.Source
    }

    foreach ($pattern in $CandidatePatterns) {
        $candidate = Get-Item -Path $pattern -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending |
            Select-Object -First 1
        if ($candidate) {
            return $candidate.FullName
        }
    }

    throw "Cannot find $CommandName. Add it to PATH or pass an explicit path."
}

$OpenOcd = Find-Tool -ExplicitPath $OpenOcd -CommandName 'openocd.exe' -CandidatePatterns @(
    "$env:USERPROFILE\Desktop\Toolchain\OpenOCD*\bin\openocd.exe",
    "$env:USERPROFILE\scoop\apps\openocd\current\bin\openocd.exe",
    'C:\Program Files\OpenOCD*\bin\openocd.exe',
    'C:\xpack-openocd*\bin\openocd.exe'
)

if (-not $ClionOptions) {
    $ClionOptions = Get-ChildItem (Join-Path $env:APPDATA 'JetBrains') -Directory -ErrorAction SilentlyContinue |
        Where-Object Name -Match '^CLion\d' |
        Sort-Object LastWriteTime -Descending |
        ForEach-Object { Join-Path $_.FullName 'options\windows' } |
        Where-Object { Test-Path -LiteralPath $_ } |
        Select-Object -First 1
}

if (-not $ClionOptions) {
    throw 'Cannot find CLion settings. Start and close CLion once, or pass -ClionOptions.'
}

# Both profile-specific OpenOCD configurations are tracked in the project.

$settingsFile = Join-Path $ClionOptions 'embedded-support.xml'
$settings = [System.Xml.XmlDocument]::new()
$settings.PreserveWhitespace = $true
if (Test-Path -LiteralPath $settingsFile) {
    $settings.Load($settingsFile)
} else {
    $settings.LoadXml('<application><component name="EmbeddedDevelopment" /></application>')
}

$component = $settings.SelectSingleNode('/application/component[@name="EmbeddedDevelopment"]')
if (-not $component) {
    $component = $settings.CreateElement('component')
    $component.SetAttribute('name', 'EmbeddedDevelopment')
    [void]$settings.DocumentElement.AppendChild($component)
}
$option = $component.SelectSingleNode('option[@name="openOcdLocation"]')
if (-not $option) {
    $option = $settings.CreateElement('option')
    $option.SetAttribute('name', 'openOcdLocation')
    [void]$component.AppendChild($option)
}
$option.SetAttribute('value', $OpenOcd)
$settings.Save($settingsFile)

Write-Host 'CLion DAPLink setup completed.' -ForegroundColor Green
Write-Host "Project:        $projectRoot"
Write-Host "OpenOCD:        $OpenOcd"
Write-Host 'Debugger:       CLion Bundled GDB (with Python support)'
Write-Host "CLion settings: $settingsFile"
Write-Host 'Reopen the project and select DAPLink MotionDebug or DAPLink NormalRun to run or debug.'
