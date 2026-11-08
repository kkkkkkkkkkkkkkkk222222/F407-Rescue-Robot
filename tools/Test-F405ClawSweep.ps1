param(
    [string]$ConfigPath = (Join-Path $PSScriptRoot 'F405ClawSweep.ini'),
    [string]$OpenOcdExe,
    [switch]$Preview,
    [switch]$Install
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

function Get-OpenOcdPath {
    if ($OpenOcdExe) { return (Get-Item -LiteralPath $OpenOcdExe).FullName }
    return (Get-Command openocd.exe -CommandType Application).Source
}

if ($Install) {
    $desktopFolder = Join-Path ([Environment]::GetFolderPath('DesktopDirectory')) 'F405 Claw Test'
    [void][IO.Directory]::CreateDirectory($desktopFolder)
    $settingsPath = Join-Path $desktopFolder 'settings.ini'
    if (!(Test-Path -LiteralPath $settingsPath)) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'F405ClawSweep.ini') -Destination $settingsPath
    }
    $exe = Get-OpenOcdPath
    $launcher = @"
@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$PSCommandPath" -ConfigPath "%~dp0settings.ini" -OpenOcdExe "$exe" %*
set "result=%errorlevel%"
if /I "%~1"=="-Preview" exit /b %result%
pause
exit /b %result%
"@
    [IO.File]::WriteAllText((Join-Path $desktopFolder 'Start.cmd'), $launcher, [Text.Encoding]::ASCII)
    $stopper = @'
@echo off
>"%~dp0stop.request" echo STOP
echo Stop requested. The test window will show STOPPED.
'@
    [IO.File]::WriteAllText((Join-Path $desktopFolder 'Stop.cmd'), $stopper, [Text.Encoding]::ASCII)
    Write-Output $desktopFolder
    return
}

$ConfigPath = (Get-Item -LiteralPath $ConfigPath).FullName
$settings = ConvertFrom-StringData ([IO.File]::ReadAllText($ConfigPath))
$keys = @('Port','StartDeg','EndDeg','LeftTrimDeg','StepDeg','IntervalMs','HoldMs','Cycles')
foreach ($key in $settings.Keys) { if ($key -notin $keys) { throw "Unknown setting $key" } }
if ($settings.Port -notmatch '^COM\d+$') { throw 'Port must be a COM port, for example COM18.' }

function Read-Number([string]$Name, [decimal]$Min, [decimal]$Max) {
    [decimal]$number = 0
    if (![decimal]::TryParse($settings[$Name], [Globalization.NumberStyles]::AllowDecimalPoint -bor [Globalization.NumberStyles]::AllowLeadingSign,
                             [Globalization.CultureInfo]::InvariantCulture, [ref]$number) -or
        $number -lt $Min -or $number -gt $Max) {
        throw "Set $Name to a number in $Min..$Max. Use a decimal point."
    }
    return $number
}

function Read-Integer([string]$Name, [int]$Min, [int]$Max) {
    $number = Read-Number $Name $Min $Max
    if ($number -ne [Math]::Floor($number)) { throw "$Name must be an integer." }
    return [int]$number
}

$startDeg = Read-Number StartDeg 0 180
$endDeg = Read-Number EndDeg 0 180
$trimDeg = Read-Integer LeftTrimDeg -180 180
$stepDeg = Read-Number StepDeg 0.09 0.5
if ($stepDeg -ge 0.5) { throw 'Set StepDeg below 0.5.' }
$intervalMs = Read-Integer IntervalMs 40 10000
$holdMs = Read-Integer HoldMs 0 60000
$cycles = Read-Integer Cycles 1 10000

function Convert-ToPulse([decimal]$Angle) {
    $outputAngle = $Angle + $trimDeg
    if ($outputAngle -lt 0 -or $outputAngle -gt 180) { throw 'Angle plus LeftTrimDeg must be within 0..180.' }
    return (500 + [int][Math]::Floor($outputAngle * 2000 / 180))
}

$startPulse = Convert-ToPulse $startDeg
$endPulse = Convert-ToPulse $endDeg
$stepUs = [int][Math]::Floor($stepDeg * 2000 / 180)
$actualStep = [decimal]$stepUs * 180 / 2000
$stepsPerLeg = [Math]::Ceiling([Math]::Abs($endPulse - $startPulse) / $stepUs)
$sweepSeconds = $cycles * 2.0 * ($stepsPerLeg * $intervalMs + $holdMs) / 1000.0
Write-Output "S4 $startDeg <-> $endDeg deg, test trim $trimDeg deg"
Write-Output "PWM $startPulse <-> $endPulse us, step $stepUs us ($actualStep deg), interval $intervalMs ms"
Write-Output "Hold $holdMs ms per end, $cycles round trips, about $sweepSeconds s plus travel to and from the start"
if ($Preview) { return }

$exe = Get-OpenOcdPath
$testFolder = Split-Path -Parent $ConfigPath
$stopFile = Join-Path $testFolder 'stop.request'
$logFile = Join-Path $testFolder 'last-test.log'
$tclStopFile = $stopFile.Replace('\', '/')
if ($tclStopFile -match '[{}]') { throw 'Use a test folder without braces in its path.' }
$serial = [System.IO.Ports.SerialPort]::new($settings.Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.WriteTimeout = 700
$lock = $null

function Send-ConsoleLine([string]$Command, [string]$Pattern) {
    $serial.DiscardInBuffer()
    $serial.Write($Command + "`r`n")
    $clock = [Diagnostics.Stopwatch]::StartNew()
    $response = ''
    while ($clock.ElapsedMilliseconds -lt 1600) {
        if (Test-Path -LiteralPath $stopFile -PathType Leaf) { throw 'Stopped during setup.' }
        $response += $serial.ReadExisting()
        if ($response -match $Pattern) { return }
        if ($response -match '(?m)^ERR(?:[ \t\r\n]|$)') { throw "Board rejected $Command" }
        Start-Sleep -Milliseconds 15
    }
    throw "No expected reply for $Command. Check NormalRun and the console connection."
}

try {
    $lock = [IO.File]::Open((Join-Path $testFolder 'run.lock'), [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
    if (Test-Path -LiteralPath $stopFile -PathType Leaf) { Remove-Item -LiteralPath $stopFile }
    try { $serial.Open() } catch { throw "Cannot open $($settings.Port). Close the serial assistant connection and retry. $($_.Exception.Message)" }
    Send-ConsoleLine 'PING' 'PONG NORMALDEBUG F405'
    Send-ConsoleLine 'DEBUG' 'DEBUG OK'
    Send-ConsoleLine 'S 2 90' 'OK\r?\n'
    Start-Sleep -Milliseconds 1000
    Send-ConsoleLine 'TSTAT' 'SERVO_TARGET=\d+,90,\d+,\d+'

    $savedPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $exe -f (Join-Path $repo 'dap.cfg') `
            -c "set start_pulse $startPulse" -c "set end_pulse $endPulse" `
            -c "set step_us $stepUs" -c "set interval_ms $intervalMs" `
            -c "set hold_ms $holdMs" -c "set cycles $cycles" `
            -c "set stop_file {$tclStopFile}" -f (Join-Path $PSScriptRoot 'F405ClawSweep.tcl') 2>&1 |
            ForEach-Object { $_.ToString() } | Tee-Object -FilePath $logFile
        $status = $LASTEXITCODE
    } finally { $ErrorActionPreference = $savedPreference }
    if ($status -ne 0) { throw "OpenOCD exited with code $status. See $logFile." }
    Write-Output "Log: $logFile"
} finally {
    $serial.Dispose()
    if ($null -ne $lock) { $lock.Dispose() }
}
