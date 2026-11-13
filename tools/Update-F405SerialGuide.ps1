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
    $mirror = [regex]::Match($Token, '^APP_CLAW_RIGHT_FROM_LEFT\((\w+)\)$')
    if ($mirror.Success) { return 180 - (Read-Angle $mirror.Groups[1].Value) }
    $pattern = '(?m)^\s*#define\s+' + [regex]::Escape($Token) + '\s+([^\r\n]+)'
    $match = [regex]::Match($config, $pattern)
    if (!$match.Success) { throw "Cannot read angle $Token in app_config.h" }
    $value = ([regex]::Replace($match.Groups[1].Value, '/\*.*?\*/|//.*', '')).Trim()
    return Read-Angle $value
}

if (!$Destination) {
    $name = ($template -split '\r?\n', 2)[0].Trim()
    $Destination = Join-Path ([Environment]::GetFolderPath('DesktopDirectory')) ($name + '.txt')
}

$offset = @{}
$offsetUs = @{}
foreach ($servo in 1..4) { $offset[$servo] = Read-Angle "APP_SERVO_S${servo}_OFFSET_DEG" }
foreach ($servo in 1..4) { $offsetUs[$servo] = Read-Angle "APP_SERVO_S${servo}_OFFSET_US" }
$clawsInverted = (Read-Angle APP_CLAW_SERVOS_INVERTED) -eq 1
$leftTouch = Read-Angle APP_CLAW_LEFT_TOUCH_ANGLE
$rightTouch = Read-Angle APP_CLAW_RIGHT_TOUCH_ANGLE
$halfRelax = Read-Angle APP_INJURY_CLAW_RELAX_DEG
$open = Read-Angle APP_CLAW_LEFT_OPEN_ANGLE
$presets = [regex]::Matches($console, '\{\s*"(?<name>[A-Z0-9_]+)"\s*,\s*(?<lift>Lift_Set\w+Position)\s*,\s*(?<cam>\w+)\s*,\s*(?<claw>Claw_\w+)\s*\}')
if ($presets.Count -eq 0) { throw 'No F405 presets found in DebugConsole.c.' }

$lines = @(
    '[[F405-AUTO-BEGIN]]'
    ('S2/S4 inverted mounting: {0}' -f $clawsInverted)
    ('Offsets S1={0:+0;-0;0}, S2={1:+0;-0;0}, S3={2:+0;-0;0}, S4={3:+0;-0;0}' -f $offset[1], $offset[2], $offset[3], $offset[4])
    ('Fine offsets(us) S1={0}, S2={1}, S3={2}, S4={3}' -f $offsetUs[1], $offsetUs[2], $offsetUs[3], $offsetUs[4])
    ''
    'Preset       S4/S2 logical -> PWM(us)        S1/S3 logical'
)
foreach ($preset in $presets) {
    $name = $preset.Groups['name'].Value
    $camera = Read-Angle $preset.Groups['cam'].Value
    $lift = switch ($preset.Groups['lift'].Value) {
        'Lift_SetStartPosition' { Read-Angle APP_LIFT_START_ANGLE }
        'Lift_SetTravelPosition' { Read-Angle APP_LIFT_TRAVEL_ANGLE }
        'Lift_SetFirstGreenFilterPosition' { Read-Angle APP_LIFT_FIRST_GREEN_FILTER_ANGLE }
        default { throw "Unrecognized lift action in P $name" }
    }
    $claw = $preset.Groups['claw'].Value
    $left = switch ($claw) {
        'Claw_Retract' { Read-Angle APP_CLAW_LEFT_RETRACT_ANGLE }
        'Claw_Open' { $open }
        'Claw_Touch' { $leftTouch }
        'Claw_HalfTouch' { $leftTouch + $halfRelax }
        default { throw "Unrecognized claw action in P $name" }
    }
    $right = switch ($claw) {
        'Claw_Retract' { Read-Angle APP_CLAW_RIGHT_RETRACT_ANGLE }
        'Claw_Open' { 180 - $open }
        'Claw_Touch' { $rightTouch }
        'Claw_HalfTouch' { $rightTouch - $halfRelax }
        default { throw "Unrecognized claw action in P $name" }
    }
    $mountedLeft = if ($clawsInverted) { 180 - $left } else { $left }
    $mountedRight = if ($clawsInverted) { 180 - $right } else { $right }
    $leftWhole = [Math]::Min(180, [Math]::Max(0, $mountedLeft + $offset[4]))
    $rightWhole = [Math]::Min(180, [Math]::Max(0, $mountedRight + $offset[2]))
    $leftPwm = [Math]::Min(2500, [Math]::Max(500, 500 + [Math]::Floor($leftWhole * 2000 / 180) + $offsetUs[4]))
    $rightPwm = [Math]::Min(2500, [Math]::Max(500, 500 + [Math]::Floor($rightWhole * 2000 / 180) + $offsetUs[2]))
    $lines += 'P {0,-10} {1,3}/{2,-3} -> {3,4}/{4,-4}          {5,3}/{6}' -f $name, $left, $right, $leftPwm, $rightPwm, $lift, $camera
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
# Migrate only exact sentences emitted by older versions; retain personal notes.
$knownLegacy = @{
    'S 1 88 设升降逻辑角。S 2 90 设右爪逻辑角。S 3 140 设摄像头逻辑角。' = 'S1起步/退出61°、水平85°、首绿静止预设82°；合爪S4=86°、S2=94°；S3=140°复审。'
    'S 4 18 设左爪逻辑角。O 2 2 和 O 4 5 分别设右爪、左爪总偏移，单位度。' = '收纳先S4=32°后S2=132°。当前微调为S4+0us、S2+24us，O只调额外整数度补偿。'
    'S 4 18 设左爪逻辑角。O 2 0 和 O 4 0 分别清零右爪、左爪安装偏移，单位度。' = '收纳先S4=32°后S2=132°。O 2 0/O 4 0不清固定24us/0us微调。'
    'RDK 使用 JP506 的 UART4，230400 8N1。' = 'RDK 使用 JP506 的 UART4，921600 8N1。'
}
foreach ($entry in $knownLegacy.GetEnumerator()) { $current = $current.Replace($entry.Key, $entry.Value) }
if (!$current.Contains('P FILTER')) {
    $current = $current.Replace('P DROP     投送时双开', "P DROP     投送时双开`r`nP FILTER   首绿抬升82°静止预设，不启动Task或后退")
}
$newline = if ($current.Contains("`r`n") -or !$exists) { "`r`n" } else { "`n" }
$pattern = '(?ms)^\[\[F405-AUTO-BEGIN\]\]\r?\n.*?^\[\[F405-AUTO-END\]\](?=\r?$)'
$block = $lines -join $newline
$blockRegex = [regex]::new($pattern)
if ($blockRegex.Matches($current).Count -ne 1) { throw 'Expected one F405 auto-update block. Existing manual was kept.' }
$updated = $blockRegex.Replace($current, [System.Text.RegularExpressions.MatchEvaluator]{ param($m) $block }, 1)
if (!$exists -or $updated -cne [System.IO.File]::ReadAllText($Destination)) { [System.IO.File]::WriteAllText($Destination, $updated, $utf8) }
Write-Output $Destination
