param(
    [switch]$WheelsSuspended,
    [ValidateRange(0, 72)][int]$CompletedBatches = 0,
    [switch]$RampToCalLimit
)

$ErrorActionPreference = 'Stop'
$portName = 'COM18'
$wheel = 2
$batchCount = 72
$batchMs = 25000
$leaseMs = 1500
$batchesPerDirection = $batchCount / 2
[int[]]$dutySteps = if ($RampToCalLimit) { 120,150,180,210,230,250 } else { 120 }
$countsPerRevolution = 1061.268

$remainingSeconds = ($batchCount - $CompletedBatches) * $batchMs / 1000
$dutyLabel = ($dutySteps | ForEach-Object { "$($_ / 10)%" }) -join ' -> '
Write-Output "$portName 115200/8N1 M$wheel PWM_STEPS=$dutyLabel BATCH_MS=$batchMs COMPLETED=$CompletedBatches/$batchCount REMAINING_SECONDS=$remainingSeconds"
if (!$WheelsSuspended -or $CompletedBatches -eq $batchCount) { return }

$portHandle = [IO.Ports.SerialPort]::new(
    $portName, 115200, [IO.Ports.Parity]::None, 8, [IO.Ports.StopBits]::One)
$portHandle.WriteTimeout = 1000
$portHandle.DtrEnable = $false
$portHandle.RtsEnable = $false
$log = [Text.StringBuilder]::new()
$lastStep = 0
$motionDebugConfirmed = $false

function Send-Command([string]$Command) {
    [void]$log.AppendLine("HOST_TX $Command")
    foreach ($character in ($Command + "`r`n").ToCharArray()) {
        $portHandle.Write([string]$character)
        Start-Sleep -Milliseconds 2
    }
}

function Read-For([int]$Milliseconds) {
    $reply = [Text.StringBuilder]::new()
    $clock = [Diagnostics.Stopwatch]::StartNew()
    while ($clock.ElapsedMilliseconds -lt $Milliseconds) {
        Start-Sleep -Milliseconds 10
        $chunk = $portHandle.ReadExisting()
        [void]$reply.Append($chunk)
        [void]$log.Append($chunk)
    }
    return $reply.ToString()
}

function Stop-Wheel {
    Send-Command 'STOP'
    $reply = Read-For 400
    if ($reply -notmatch 'STOP OK') { throw 'STOP acknowledgement missing.' }
}

try {
    $portHandle.Open()
    [void](Read-For 200)
    Send-Command 'STAT'
    $status = Read-For 600
    $mode = [regex]::Match($status, 'BOARD=F405_ROBO SERVICE=MOTIONDEBUG[^\r\n]*\bCAL=1 MODE=5\b')
    if (!$mode.Success) {
        throw 'F405 MotionDebug with CAL was not confirmed.'
    }
    $motionDebugConfirmed = $true
    Write-Output $mode.Value
    Stop-Wheel

    for ($batch = $CompletedBatches + 1; $batch -le $batchCount; ++$batch) {
        $direction = if ($batch -le $batchesPerDirection) { 1 } else { -1 }
        $previousDirectionBatches = if ($direction -gt 0) { 0 } else { $batchesPerDirection }
        $rampOrigin = [math]::Max($CompletedBatches, $previousDirectionBatches)
        $dutyStep = [math]::Min($batch - $rampOrigin - 1, $dutySteps.Length - 1)
        $pwm = $direction * $dutySteps[$dutyStep]
        $clock = [Diagnostics.Stopwatch]::StartNew()
        while ($clock.ElapsedMilliseconds -lt $batchMs) {
            Send-Command "CAL $wheel $pwm $leaseMs"
            $readMs = [int][math]::Min(800, [math]::Max(200, $batchMs - $clock.ElapsedMilliseconds))
            $reply = Read-For $readMs
            $ack = [regex]::Match($reply, "CAL_BEGIN STEP=(\d+) ID=$wheel PWM=$pwm LEASE_MS=$leaseMs\b")
            if (!$ack.Success -or $reply -match 'ERR |BUSY|CAL_END') {
                throw "CAL was interrupted in batch $batch."
            }
            $step = [int]$ack.Groups[1].Value
            $sample = [regex]::Match($reply,
                "CAL_DATA STEP=$step ID=$wheel PWM=$pwm [^\r\n]* FB=-?[1-9]\d* ACTIVE=1")
            if ($step -le $lastStep -or !$sample.Success) {
                throw "No new encoder feedback for M$wheel in batch $batch."
            }
            if ($lastStep -eq 0) {
                Write-Output $ack.Value
                Write-Output $sample.Value
            }
            $lastStep = $step
        }
        Stop-Wheel
        Write-Output ("BATCH={0}/{1} PWM={2} CUMULATIVE_SECONDS={3} STOPPED" -f
            $batch, $batchCount, $pwm, ($batch * $batchMs / 1000))
        if ($RampToCalLimit) {
            $samples = [regex]::Matches($reply,
                "CAL_DATA STEP=$step ID=$wheel PWM=$pwm [^\r\n]* DT_MS=(\d+) [^\r\n]* FB=(-?\d+) ACTIVE=1")
            $sampleMs = 0
            $sampleCounts = 0
            foreach ($entry in $samples) {
                $sampleMs += [int]$entry.Groups[1].Value
                $sampleCounts += [int]$entry.Groups[2].Value
            }
            if ($sampleMs -gt 0) {
                $rpm = [math]::Abs($sampleCounts) * 60000 / ($countsPerRevolution * $sampleMs)
                Write-Output ("SPEED BATCH={0} PWM={1} RPM={2:F1}" -f $batch,$pwm,$rpm)
            }
        }
        if ($batch -lt $batchCount) { [void](Read-For 1600) }
    }

    Send-Command 'STAT'
    Write-Output (Read-For 600)
} finally {
    try {
        if ($portHandle.IsOpen) {
            try {
                if ($motionDebugConfirmed) { Stop-Wheel }
            } finally { $portHandle.Close() }
        }
    } finally {
        $portHandle.Dispose()
        $path = Join-Path $PSScriptRoot ((Get-Date -Format 'yyyyMMdd-HHmmss') + '-left-wheel-runin.log')
        [IO.File]::WriteAllText($path, $log.ToString())
        Write-Output "LOG=$path"
    }
}
