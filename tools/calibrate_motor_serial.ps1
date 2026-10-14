param(
  [string]$Port = 'COM18',
  [int[]]$Wheels = @(1, 2, 3),
  [int[]]$Directions = @(1, -1),
  [switch]$WheelsSuspended,
  [switch]$Fine,
  [ValidateRange(0, 5)][int]$StartTrials = 0,
  [string]$OutputDirectory = 'build/MotionDebug/calibration'
)

$ErrorActionPreference = 'Stop'
if (!$WheelsSuspended) { throw 'Confirm all wheels are suspended with -WheelsSuspended.' }
if ($Fine -and $StartTrials -gt 0) { throw 'Choose Fine or StartTrials, not both.' }
if ($Wheels | Where-Object { $_ -lt 1 -or $_ -gt 3 }) { throw 'Wheel IDs must be 1..3.' }
if ($Directions | Where-Object { $_ -ne 1 -and $_ -ne -1 }) { throw 'Directions must be +/-1.' }
$null = New-Item -ItemType Directory -Force -Path $OutputDirectory
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$base = Join-Path (Resolve-Path $OutputDirectory) ($stamp + $(if ($StartTrials -gt 0) { '-starts' } elseif ($Fine) { '-fine' } else { '-coarse' }))
$log = [Text.StringBuilder]::new()
$rows = [Collections.Generic.List[object]]::new()
$portHandle = [IO.Ports.SerialPort]::new($Port, 115200)
$portHandle.WriteTimeout = 1000
$portHandle.Encoding = [Text.Encoding]::GetEncoding(28591)
$lastStep = 0

function Read-For([int]$Milliseconds) {
  $text = [Text.StringBuilder]::new()
  $clock = [Diagnostics.Stopwatch]::StartNew()
  while ($clock.ElapsedMilliseconds -lt $Milliseconds) {
    Start-Sleep -Milliseconds 10
    $part = $portHandle.ReadExisting()
    [void]$text.Append($part)
    [void]$log.Append($part)
  }
  return $text.ToString()
}

function Send-Command([string]$Command) {
  [void]$log.AppendLine("HOST_TX $Command")
  foreach ($character in ($Command + "`r`n").ToCharArray()) {
    $portHandle.Write([string]$character)
    Start-Sleep -Milliseconds 2
  }
}

try {
  $portHandle.Open()
  [void](Read-For 200)
  Send-Command 'STOP'
  [void](Read-For 300)
  Send-Command 'STAT'
  $status = Read-For 500
  if ($status -notmatch 'CAL=1 MODE=2') { throw "Calibration firmware not confirmed: $status" }
  foreach ($wheel in $Wheels) {
    foreach ($direction in $Directions) {
      Send-Command 'STOP'
      $stopped = Read-For 400
      if ($stopped -notmatch 'STOP OK') { throw 'STOP was not acknowledged.' }
      $phases = if ($StartTrials -gt 0) { @(1..$StartTrials | ForEach-Object { "START_$_" }) } else { @('UP', 'DOWN') }
      foreach ($phase in $phases) {
        $levels = if ($phase -eq 'START_2') {
          @(80, 70, 60, 50)
        } elseif ($phase -eq 'START_3') {
          @(60, 80, 50, 70)
        } elseif ($StartTrials -gt 0) {
          @(50, 60, 70, 80)
        } elseif ($Fine -and $wheel -eq 3 -and $phase -eq 'UP') {
          @(40, 45, 50, 55, 60, 65, 70, 75, 80, 85)
        } elseif ($Fine -and $wheel -eq 3) {
          @(80, 75, 70, 65, 60, 55, 50, 45, 40, 35)
        } elseif ($Fine -and $phase -eq 'UP') {
          @(30, 35, 40, 45, 50, 55, 60, 65)
        } elseif ($Fine) {
          @(60, 55, 50, 45, 40, 35, 30, 25)
        } elseif ($phase -eq 'UP') {
          @(20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 150, 180, 220)
        } else {
          @(180, 150, 120, 100, 90, 80, 70, 60, 50, 40, 30, 20)
        }
        foreach ($level in $levels) {
          if ($StartTrials -gt 0) {
            Send-Command 'STOP'
            $stopped = Read-For 500
            if ($stopped -notmatch 'STOP OK') { throw 'STOP was not acknowledged before start trial.' }
          }
          $pwm = $direction * $level
          Send-Command "CAL $wheel $pwm 1500"
          $reply = Read-For 900
          if ($reply -match 'ERR |BUSY|CAL_END') { throw "Calibration interrupted: $reply" }
          $begin = [regex]::Match($reply, 'CAL_BEGIN STEP=(\d+) ID=(\d+) PWM=(-?\d+)')
          if (!$begin.Success) { throw 'Missing CAL_BEGIN; no further drive commands will be sent.' }
          $step = [int]$begin.Groups[1].Value
          if ($step -le $lastStep -or [int]$begin.Groups[2].Value -ne $wheel -or
              [int]$begin.Groups[3].Value -ne $pwm) { throw 'Unexpected step, motor, or reset.' }
          $lastStep = $step
          $samples = [regex]::Matches($reply,
              'CAL_DATA STEP=(\d+) ID=(\d+) PWM=(-?\d+) MS=(\d+) AGE_MS=(\d+) DT_MS=(\d+) RAW=(-?\d+),(-?\d+),(-?\d+) FB=(-?\d+) ACTIVE=(\d+)')
          $currentRows = @()
          foreach ($sample in $samples) {
            $g = $sample.Groups
            if ([int]$g[1].Value -ne $step) { continue }
            $dt = [int]$g[6].Value
            $fb = [int]$g[10].Value
            $row = [pscustomobject]@{
              Wheel=$wheel; Direction=$direction; Phase=$phase; Step=$step; PwmPermille=$pwm
              Ms=[int]$g[4].Value; AgeMs=[int]$g[5].Value; DtMs=$dt
              Raw1=[int]$g[7].Value; Raw2=[int]$g[8].Value; Raw3=[int]$g[9].Value
              Feedback=$fb; Active=[int]$g[11].Value
              SpeedMmS=($fb * [math]::PI * 70 * 1000 / (1061.268 * $dt))
              Rpm=($fb * 60000 / (1061.268 * $dt)); AssumedSupplyMv=12000
            }
            $rows.Add($row)
            $currentRows += $row
          }
          $steady = @($currentRows | Where-Object { $_.AgeMs -ge 500 -and $_.Active -eq 1 })
          if ($steady.Count -lt 4) { throw 'Too few complete steady-state samples.' }
          $counts = ($steady | Measure-Object Feedback -Sum).Sum
          $ms = ($steady | Measure-Object DtMs -Sum).Sum
          $speed = $counts * [math]::PI * 70 * 1000 / (1061.268 * $ms)
          $zero = @($steady | Where-Object Feedback -eq 0).Count
          'W{0} DIR={1} {2} PWM={3:F1}% SPEED={4:F1}mm/s ZERO={5}/{6}' -f
              $wheel, $direction, $phase, ($level/10.0), $speed, $zero, $steady.Count
        }
      }
      Send-Command 'STOP'
      [void](Read-For 400)
      Send-Command 'PING'
      $pong = Read-For 300
      if ($pong -notmatch 'PONG MS=') { throw 'Lost communication after sweep.' }
    }
  }
  'CALIBRATION COMPLETE'
} finally {
  if ($portHandle.IsOpen) {
    try { Send-Command 'STOP'; [void](Read-For 300) } finally { $portHandle.Close() }
  }
  $portHandle.Dispose()
  [IO.File]::WriteAllText("$base.log", $log.ToString())
  $rows | Export-Csv -NoTypeInformation -Encoding utf8 -LiteralPath "$base.csv"
  "SAVED $base.csv"
}
