param([string]$Destination)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$utf8 = [System.Text.UTF8Encoding]::new($true)
$config = [System.IO.File]::ReadAllText((Join-Path $repo 'Main/Inc/app_config.h'))
$console = [System.IO.File]::ReadAllText((Join-Path $repo 'Main/Src/DebugConsole.c'))
$template = [System.IO.File]::ReadAllText((Join-Path $repo 'docs/F405_SERIAL_GUIDE.txt'))

function Read-Angle([string]$Token) {
    if ($Token -match '^[+-]?\d+[uUlL]*$') {
        return [int]($Token -replace '[uUlL]+$', '')
    }
    $pattern = '(?m)^\s*#define\s+' + [regex]::Escape($Token) + '\s+([+-]?\d+)[uUlL]*(?=\s|$)'
    $match = [regex]::Match($config, $pattern)
    if (!$match.Success) { throw "Cannot read angle $Token in app_config.h" }
    return [int]$match.Groups[1].Value
}

if (!$Destination) {
    $name = ($template -split '\r?\n', 2)[0].Trim()
    $Destination = Join-Path ([Environment]::GetFolderPath('DesktopDirectory')) ($name + '.txt')
}

$offset = @{}
foreach ($servo in 1..4) { $offset[$servo] = Read-Angle "APP_SERVO_S${servo}_OFFSET_DEG" }
$touch = Read-Angle APP_CLAW_LEFT_TOUCH_ANGLE
$open = Read-Angle APP_CLAW_LEFT_OPEN_ANGLE
$presets = [regex]::Matches($console, '\{\s*"(?<name>[A-Z0-9_]+)"\s*,\s*(?<lift>Lift_Set\w+Position)\s*,\s*(?<cam>\w+)\s*,\s*(?<claw>Claw_\w+)\s*\}')
if ($presets.Count -eq 0) { throw 'No F405 presets found in DebugConsole.c.' }

$lines = @(
    '[[F405-AUTO-BEGIN]]'
    ('Offsets S1={0:+0;-0;0}, S2={1:+0;-0;0}, S3={2:+0;-0;0}, S4={3:+0;-0;0}' -f $offset[1], $offset[2], $offset[3], $offset[4])
    ''
    'Preset       S4/S2 logical -> PWM       S1/S3 logical'
)
foreach ($preset in $presets) {
    $name = $preset.Groups['name'].Value
    $camera = Read-Angle $preset.Groups['cam'].Value
    $lift = switch ($preset.Groups['lift'].Value) {
        'Lift_SetStartPosition' { Read-Angle APP_LIFT_START_ANGLE }
        'Lift_SetTravelPosition' { Read-Angle APP_LIFT_TRAVEL_ANGLE }
        default { throw "Unrecognized lift action in P $name" }
    }
    $claw = $preset.Groups['claw'].Value
    $left = switch ($claw) {
        'Claw_Retract' { Read-Angle APP_CLAW_LEFT_RETRACT_ANGLE }
        'Claw_Open' { $open }
        'Claw_Touch' { $touch }
        'Claw_HalfTouch' { $touch + (Read-Angle APP_INJURY_CLAW_RELAX_DEG) }
        default { throw "Unrecognized claw action in P $name" }
    }
    $right = if ($claw -eq 'Claw_Retract') { Read-Angle APP_CLAW_RIGHT_RETRACT_ANGLE } else { 180 - $left }
    $leftPwm = [Math]::Min(180, [Math]::Max(0, $left + $offset[4]))
    $rightPwm = [Math]::Min(180, [Math]::Max(0, $right + $offset[2]))
    $lines += 'P {0,-10} {1,3}/{2,-3} -> {3,3}/{4,-3}           {5,3}/{6}' -f $name, $left, $right, $leftPwm, $rightPwm, $lift, $camera
}
$runInScript = Join-Path $PSScriptRoot 'Run-F405ClawRunIn.ps1'
$sweepScript = Join-Path $PSScriptRoot 'Test-F405ClawSweep.ps1'
$lines += @(
    ''
    'Refresh in PowerShell'
    ('powershell.exe -NoProfile -ExecutionPolicy Bypass -File "{0}"' -f $PSCommandPath)
    'Claw run-in in PowerShell'
    ('powershell.exe -NoProfile -ExecutionPolicy Bypass -File "{0}"' -f $runInScript)
    'Claw fine sweep desktop installer'
    ('powershell.exe -NoProfile -ExecutionPolicy Bypass -File "{0}" -Install' -f $sweepScript)
    '[[F405-AUTO-END]]'
)

$exists = Test-Path -LiteralPath $Destination -PathType Leaf
$current = if ($exists) { [System.IO.File]::ReadAllText($Destination) } else { $template }
$newline = if ($current.Contains("`r`n") -or !$exists) { "`r`n" } else { "`n" }
$pattern = '(?ms)^\[\[F405-AUTO-BEGIN\]\]\r?\n.*?^\[\[F405-AUTO-END\]\](?=\r?$)'
$block = $lines -join $newline
$blockRegex = [regex]::new($pattern)
if ($blockRegex.Matches($current).Count -ne 1) { throw 'Expected one F405 auto-update block. Existing manual was kept.' }
$updated = $blockRegex.Replace($current, [System.Text.RegularExpressions.MatchEvaluator]{ param($m) $block }, 1)
if (!$exists -or $updated -cne $current) { [System.IO.File]::WriteAllText($Destination, $updated, $utf8) }
Write-Output $Destination
