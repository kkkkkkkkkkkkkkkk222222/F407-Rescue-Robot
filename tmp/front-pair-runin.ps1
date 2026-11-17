param(
    [ValidateRange(1, 1800)][int]$Seconds = 1800,
    [ValidateRange(0, 1799999)][int]$CompletedMs = 0
)

$ErrorActionPreference = 'Stop'
$stopFile = Join-Path $PSScriptRoot 'front-pair-runin.stop'
$logPath = Join-Path $PSScriptRoot ((Get-Date -Format 'yyyyMMdd-HHmmss') + '-front-pair-runin.log')
$log = [IO.StreamWriter]::new($logPath)
$log.AutoFlush = $true
$port = [IO.Ports.SerialPort]::new('COM18', 115200, [IO.Ports.Parity]::None, 8, [IO.Ports.StopBits]::One)
$port.WriteTimeout = 750
$port.DtrEnable = $false
$port.RtsEnable = $false
$verified = $false
$pendingLine = ''
$targetMs = $Seconds * 1000
$samplePattern = '(?m)^CAL_PAIR_DATA STEP=(?<step>\d+) PWM=-?\d+,-?\d+ MS=(?<tick>\d+) AGE_MS=(?<age>\d+) DT_MS=(?<dt>\d+) RAW=-?\d+,-?\d+,-?\d+ FB=(?<right>-?\d+),(?<left>-?\d+) ACTIVE=(?<active>[01])\r?$'

function Send-Command([string]$Command) {
    $port.Write($Command + "`r`n")
    $log.WriteLine("HOST_TX $Command")
}

function Read-Reply([int]$Milliseconds, [switch]$IgnoreStop) {
    $reply = [Text.StringBuilder]::new()
    $clock = [Diagnostics.Stopwatch]::StartNew()
    while ($clock.ElapsedMilliseconds -lt $Milliseconds) {
        Start-Sleep -Milliseconds 10
        $chunk = $port.ReadExisting()
        [void]$reply.Append($chunk)
        if (!$IgnoreStop -and (Test-Path -LiteralPath $stopFile)) { break }
    }
    $received = $script:pendingLine + $reply.ToString()
    $length = $received.LastIndexOf("`n") + 1
    $script:pendingLine = $received.Substring($length)
    $lines = $received.Substring(0, $length)
    if ($lines.Length -gt 0) { $log.Write($lines) }
    return $lines
}

function Write-ProgressLine([string]$Line) {
    $log.WriteLine($Line)
    Write-Output $Line
}

function Stop-Wheels {
    Send-Command 'STOP'
    $reply = Read-Reply 300 -IgnoreStop
    if ($reply -notmatch '(?m)^STOP OK\r?$') {
        throw 'STOP acknowledgement missing.'
    }
    return $reply
}

function Wait-WheelsStopped([switch]$IgnoreStop) {
    $clock = [Diagnostics.Stopwatch]::StartNew()
    $stableSince = $null
    $previous = $null
    while ($clock.ElapsedMilliseconds -lt 8000) {
        if (!$IgnoreStop -and (Test-Path -LiteralPath $stopFile)) { return $false }
        Send-Command 'ENC'
        $reply = Read-Reply 140 -IgnoreStop:$IgnoreStop
        $sample = [regex]::Match($reply, '(?m)^RXCMD ENC\r?\nENC RAW32=(-?\d+),(-?\d+),-?\d+ DELTA10=(-?\d+),(-?\d+),-?\d+\r?$')
        if (!$sample.Success -or $sample.Groups[3].Value -ne '0' -or $sample.Groups[4].Value -ne '0') {
            $previous = $stableSince = $null
            continue
        }
        $position = $sample.Groups[1].Value + ',' + $sample.Groups[2].Value
        $now = $clock.ElapsedMilliseconds
        if ($position -ne $previous) {
            $previous = $position
            $stableSince = $now
        } elseif ($now - $stableSince -ge 350) {
            $log.WriteLine("WHEELS_SETTLED RAW=$position STABLE_MS=$($now - $stableSince) WAIT_MS=$now")
            return $true
        }
    }
    throw 'Front wheel encoders did not confirm a stop within 8 seconds.'
}

function Get-ElapsedMs([long]$End, [long]$Start) {
    return ($End - $Start + 4294967296L) % 4294967296L
}

try {
    if (Test-Path -LiteralPath $stopFile) { throw "Stop file present: $stopFile" }
    $port.Open()
    [void](Read-Reply 200)
    for ($attempt = 0; $attempt -lt 3; ++$attempt) {
        Send-Command 'STAT'
        $status = Read-Reply 550
        $mode = [regex]::Match($status, 'BOARD=F405_ROBO SERVICE=MOTIONDEBUG[^\r\n]*\bCALPAIR=1 CAL_MAX=(\d+) CAL=1 MODE=5\b')
        if ($mode.Success) { break }
    }
    if (!$mode.Success) { throw 'Board did not identify as F405 MotionDebug with CALPAIR.' }
    $limit = [int]$mode.Groups[1].Value
    if ($limit -ne 600) { throw "Unexpected competition PWM limit: $limit" }
    $verified = $true
    Write-ProgressLine $mode.Value
    Write-ProgressLine "LOG=$logPath TARGET_MS=$targetMs PREVIOUS_MS=$CompletedMs"
    [void](Stop-Wheels)

    $ramp = @(250, 350, 450, 525, $limit)
    $trim = 0
    $step = 0
    $poweredMs = $CompletedMs
    $recoveries = 0
    while ($poweredMs -lt $targetMs -and (Wait-WheelsStopped) -and !(Test-Path -LiteralPath $stopFile)) {
        $batchMs = [Math]::Min(25000, $targetMs - $poweredMs)
        $batchStart = $null
        $batchElapsed = 0L
        $interrupted = $false
        while ($batchElapsed -lt $batchMs -and !(Test-Path -LiteralPath $stopFile)) {
            $base = $ramp[[Math]::Min($step, $ramp.Length - 1)]
            $right = -($base - [Math]::Max(0, $trim))
            $left = $base + [Math]::Min(0, $trim)
            Send-Command "CALPAIR $right $left 1500"
            $reply = Read-Reply 420
            $samples = [regex]::Matches($reply, $samplePattern)
            if ($samples.Count -gt 0) {
                if ($null -eq $batchStart) {
                    $first = $samples[0]
                    $batchStart = Get-ElapsedMs ([long]$first.Groups['tick'].Value) ([long]$first.Groups['age'].Value)
                }
                $batchElapsed = Get-ElapsedMs ([long]$samples[$samples.Count - 1].Groups['tick'].Value) $batchStart
            }
            if (Test-Path -LiteralPath $stopFile) { break }
            $ack = [regex]::Match($reply, "CAL_PAIR_BEGIN STEP=(\d+) PWM=$right,$left LEASE_MS=1500\b")
            if (!$ack.Success -or $reply -match '(?m)^(ERR |BUSY|CAL_END)') {
                $interrupted = $true
                break
            }
            $sampleId = $ack.Groups[1].Value
            $dt = $rightCounts = $leftCounts = 0
            foreach ($sample in $samples) {
                if ($sample.Groups['step'].Value -ne $sampleId -or $sample.Groups['active'].Value -ne '1' -or
                    [int]$sample.Groups['age'].Value -lt 150) { continue }
                $dt += [int]$sample.Groups['dt'].Value
                $rightCounts += [int]$sample.Groups['right'].Value
                $leftCounts += [int]$sample.Groups['left'].Value
            }
            if ($dt -eq 0) {
                $interrupted = $true
                break
            }
            if ($rightCounts -ge 0 -or $leftCounts -le 0) {
                throw 'Both front encoders must report the expected direction.'
            }
            $rpmScale = 60000 / (1061.268 * $dt)
            $rightRpm = -$rightCounts * $rpmScale
            $leftRpm = $leftCounts * $rpmScale
            $rpmLine = 'RPM RIGHT={0:F1} LEFT={1:F1} PWM={2},{3} RUN_S={4:F1}' -f $rightRpm,$leftRpm,$right,$left,(($poweredMs + $batchElapsed) / 1000)
            if ($step -le 7 -or $step % 20 -eq 0) { Write-ProgressLine $rpmLine } else { $log.WriteLine($rpmLine) }
            if ($base -eq $limit -and [Math]::Abs($rightRpm - $leftRpm) -gt 2) {
                $trim += [int][Math]::Round(0.45 * ($rightRpm - $leftRpm))
                $trim = [Math]::Max(250 - $limit, [Math]::Min($limit - 250, $trim))
            }
            ++$step
        }
        $stopReply = Stop-Wheels
        $endSamples = [regex]::Matches($stopReply, $samplePattern)
        if ($null -ne $batchStart -and $endSamples.Count -gt 0) {
            $batchElapsed = Get-ElapsedMs ([long]$endSamples[$endSamples.Count - 1].Groups['tick'].Value) $batchStart
        }
        if ($batchElapsed -gt 31000) { throw 'Invalid board time for the current batch.' }
        $poweredMs += $batchElapsed
        Write-ProgressLine ('BATCH_END STOPPED BATCH_MS={0} RUN_MS={1}' -f $batchElapsed,$poweredMs)
        if ($interrupted) {
            ++$recoveries
            if ($recoveries -ge 3) { throw 'Three consecutive batches were interrupted. Wheels stopped.' }
            $step = $trim = 0
            Write-ProgressLine "BATCH_INTERRUPTED RETRY=$recoveries/2 AFTER WHEELS_SETTLED"
        } else { $recoveries = 0 }
    }
    if ($poweredMs -ge $targetMs -and (Wait-WheelsStopped -IgnoreStop)) {
        Send-Command 'MOTOR'
        $final = Read-Reply 300 -IgnoreStop
        if ($final -notmatch '(?m)^DRIVE TARGET_MM_S=0,0,0 ACTUAL_MM_S=0,0,0 PWM_PERMILLE=0,0,0\r?$') {
            throw 'Final zero-speed and PWM confirmation missing.'
        }
        Write-ProgressLine "RUN_COMPLETE RUN_MS=$poweredMs SESSION_MS=$($poweredMs - $CompletedMs) STOPPED=1"
    }
} finally {
    try {
        if ($verified -and $port.IsOpen) { [void](Stop-Wheels) }
    } finally {
        $port.Dispose()
        $log.Dispose()
    }
}
