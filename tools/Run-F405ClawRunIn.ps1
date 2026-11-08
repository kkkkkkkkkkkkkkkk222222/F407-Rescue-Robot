param(
    [string]$Port = 'COM18',
    [ValidateRange(1, 3600)][int]$DurationSeconds = 600,
    [string]$StopFile
)

$ErrorActionPreference = 'Stop'
$intervalMs = 500
$inAngle = 18
$outAngle = 120
$durationMs = $DurationSeconds * 1000L
$configPath = Join-Path (Split-Path -Parent $PSScriptRoot) 'Main/Inc/app_config.h'
$config = [System.IO.File]::ReadAllText($configPath)
$offsets = @{}
foreach ($match in [regex]::Matches($config, '(?m)^\s*#define\s+APP_SERVO_S([24])_OFFSET_DEG\s+([+-]?\d+)(?=\s|$)')) {
    $offsets[[int]$match.Groups[1].Value] = [int]$match.Groups[2].Value
}
if ($offsets.Count -ne 2) { throw 'The S2 and S4 offsets could not be read from app_config.h.' }

function Test-StopRequested {
    return ($StopFile -and (Test-Path -LiteralPath $StopFile -PathType Leaf))
}

$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.WriteTimeout = 500

function Send-RunInCommand([string]$Command, [string]$ReplyPattern, [int]$TimeoutMs = 400) {
    $serial.DiscardInBuffer()
    $serial.Write($Command + "`r`n")
    $timeout = [Diagnostics.Stopwatch]::StartNew()
    $reply = ''
    while ($timeout.ElapsedMilliseconds -lt $TimeoutMs) {
        $reply += $serial.ReadExisting()
        if ($reply -match '(?m)^ERR(?:[ \t\r\n]|$)') { throw "F405 rejected $Command" }
        if ($reply -match $ReplyPattern) { return }
        Start-Sleep -Milliseconds 5
    }
    throw "No acknowledgement for $Command on $Port"
}

function Wait-RunInTime([Diagnostics.Stopwatch]$Clock, [long]$TargetMs) {
    while ($Clock.ElapsedMilliseconds -lt $TargetMs) {
        if (Test-StopRequested) { return }
        $remaining = $TargetMs - $Clock.ElapsedMilliseconds
        Start-Sleep -Milliseconds ([int][Math]::Max(1, [Math]::Min(50, $remaining)))
    }
}

try {
    if (Test-StopRequested) { throw "Stop file already exists: $StopFile" }
    $serial.Open()
    Send-RunInCommand 'PING' 'PONG NORMALDEBUG F405' 1200
    Send-RunInCommand 'DEBUG' 'DEBUG OK\r?\n' 1200
    Send-RunInCommand 'S 2 90' '\bOK\r?\n'
    Send-RunInCommand "O 2 $($offsets[2])" '\bOK\r?\n'
    $settleClock = [Diagnostics.Stopwatch]::StartNew()
    Wait-RunInTime $settleClock 1000
    if (Test-StopRequested) { Write-Output 'STOPPED during right claw setup'; return }
    Send-RunInCommand "O 4 $($offsets[4])" '\bOK\r?\n'

    Write-Output "START S2=90 S4=$inAngle/$outAngle INTERVAL_MS=$intervalMs DURATION_S=$DurationSeconds OFFSETS=$($offsets[2]),$($offsets[4])"
    $clock = [Diagnostics.Stopwatch]::StartNew()
    $sent = 0
    $lastAngle = $null
    $nextAt = 0L
    $nextReport = 60000L
    while ($clock.ElapsedMilliseconds -lt $durationMs -and !(Test-StopRequested)) {
        Wait-RunInTime $clock ([Math]::Min($nextAt, $durationMs))
        if ($clock.ElapsedMilliseconds -ge $durationMs -or (Test-StopRequested)) { break }
        $lastAngle = if ($sent % 2 -eq 0) { $inAngle } else { $outAngle }
        $sentAt = $clock.ElapsedMilliseconds
        Send-RunInCommand "S 4 $lastAngle" '\bOK\r?\n'
        $sent++
        $nextAt = $sentAt + $intervalMs
        if ($clock.ElapsedMilliseconds -ge $nextReport) {
            Write-Output ('PROGRESS ELAPSED_S={0:F1} COMMANDS={1} S4={2}' -f $clock.Elapsed.TotalSeconds, $sent, $lastAngle)
            $nextReport += 60000L
        }
    }

    if (Test-StopRequested) {
        Write-Output ('STOPPED ELAPSED_S={0:F1} COMMANDS={1} S4={2}' -f $clock.Elapsed.TotalSeconds, $sent, $lastAngle)
    } else {
        if ($lastAngle -ne $inAngle) { Send-RunInCommand "S 4 $inAngle" '\bOK\r?\n' }
        Send-RunInCommand 'TSTAT' "SERVO_TARGET=\d+,90,\d+,$inAngle\b" 1200
        Write-Output ('DONE ELAPSED_S={0:F1} COMMANDS={1} S2=90 S4={2} MODE=DEBUG' -f $clock.Elapsed.TotalSeconds, $sent, $inAngle)
    }
} finally {
    $serial.Dispose()
}
