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
$cmakeFile = Join-Path $projectRoot 'CMakeLists.txt'

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

$projectName = 'WWW'
if (Test-Path -LiteralPath $cmakeFile) {
    $cmakeText = Get-Content -LiteralPath $cmakeFile -Raw
    $match = [regex]::Match($cmakeText, 'set\s*\(\s*CMAKE_PROJECT_NAME\s+([^\s\)]+)', 'IgnoreCase')
    if ($match.Success) {
        $projectName = $match.Groups[1].Value.Trim('"')
    }
}

$configName = 'Debug'
$workspaceFile = Join-Path $projectRoot '.idea\workspace.xml'
if (Test-Path -LiteralPath $workspaceFile) {
    try {
        [xml]$workspace = Get-Content -LiteralPath $workspaceFile -Raw
        $cmakeConfig = $workspace.project.component |
            Where-Object name -eq 'RunManager' |
            Select-Object -ExpandProperty configuration |
            Where-Object { $_.TARGET_NAME -eq $projectName } |
            Select-Object -First 1
        if ($cmakeConfig.CONFIG_NAME) {
            $configName = [string]$cmakeConfig.CONFIG_NAME
        }
    } catch {
        Write-Warning 'Cannot read workspace.xml. Using the Debug profile name.'
    }
}

$runConfigDir = Join-Path $projectRoot '.idea\runConfigurations'
[void](New-Item -ItemType Directory -Path $runConfigDir -Force)
$runConfigFile = Join-Path $runConfigDir 'DAPLink_OpenOCD.xml'

$runConfig = @'
<component name="ProjectRunConfigurationManager">
  <configuration default="false" name="DAPLink OpenOCD" type="com.jetbrains.cidr.embedded.openocd.conf.type" factoryName="com.jetbrains.cidr.embedded.openocd.conf.factory" REDIRECT_INPUT="false" ELEVATE="false" USE_EXTERNAL_CONSOLE="false" PASS_PARENT_ENVS_2="true" PROJECT_NAME="__PROJECT__" TARGET_NAME="__PROJECT__" CONFIG_NAME="__CONFIG__" version="1" RUN_TARGET_PROJECT_NAME="__PROJECT__" RUN_TARGET_NAME="__PROJECT__">
    <openocd version="1" gdb-port="3333" telnet-port="4444" board-config="$PROJECT_DIR$/dap.cfg" reset-type="INIT" download-type="ALWAYS">
      <debugger kind="GDB" isBundled="true" />
    </openocd>
    <method v="2">
      <option name="CLION.COMPOUND.BUILD" enabled="true" />
    </method>
  </configuration>
</component>
'@
$runConfig = $runConfig.Replace('__PROJECT__', $projectName).Replace('__CONFIG__', $configName)
[System.IO.File]::WriteAllText($runConfigFile, $runConfig, [System.Text.UTF8Encoding]::new($false))

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
Write-Host 'Reopen the project and select DAPLink OpenOCD to run or debug.'
