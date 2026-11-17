param([ValidateRange(1,3)][int]$Wheel = 3)
$ErrorActionPreference = 'Stop'
$p = [IO.Ports.SerialPort]::new('COM18',115200)
$p.WriteTimeout = 1000
$log = [Text.StringBuilder]::new()
function Send([string]$s) {
  [void]$log.AppendLine("HOST_TX $s")
  foreach ($c in ($s+"`r`n").ToCharArray()) { $p.Write([string]$c); Start-Sleep -Milliseconds 2 }
}
function ReadFor([int]$ms) {
  $b = [Text.StringBuilder]::new()
  $t = [Diagnostics.Stopwatch]::StartNew()
  while ($t.ElapsedMilliseconds -lt $ms) {
    Start-Sleep -Milliseconds 10
    $s = $p.ReadExisting()
    [void]$b.Append($s); [void]$log.Append($s)
  }
  return $b.ToString()
}
try {
  $p.Open(); [void](ReadFor 200)
  Send 'STOP'; $s = ReadFor 300
  if ($s -notmatch 'STOP OK') { throw 'No STOP acknowledgement' }
  Send 'STAT'; $s = ReadFor 500
  if ($s -notmatch 'CAL=1 MODE=2') { throw 'Wrong firmware' }
  for ($batch=1; $batch -le 24; $batch++) {
    $pwm = if ($batch -le 12) {120} else {-120}
    $clock = [Diagnostics.Stopwatch]::StartNew()
    while ($clock.ElapsedMilliseconds -lt 25000) {
      Send "CAL $Wheel $pwm 1500"
      $s = ReadFor ([int][math]::Min(800,[math]::Max(200,25000-$clock.ElapsedMilliseconds)))
      if ($s -match 'ERR |BUSY|CAL_END' -or $s -notmatch "CAL_BEGIN STEP=\d+ ID=$Wheel PWM=$pwm\b") {
        throw "Unexpected response: $s"
      }
      if ($s -notmatch 'FB=-?[1-9]\d* ACTIVE=1') { throw 'No motion feedback' }
    }
    Send 'STOP'; $s = ReadFor 400
    if ($s -notmatch 'STOP OK') { throw 'No STOP acknowledgement' }
    "BATCH=$batch/24 PWM=$pwm STOPPED ELAPSED_MS=$($clock.ElapsedMilliseconds)"
    [void](ReadFor 1600)
  }
  Send 'STAT'; Write-Output (ReadFor 600)
} finally {
  if ($p.IsOpen) {
    try { Send 'STOP'; [void](ReadFor 300) } finally { $p.Close() }
  }
  $p.Dispose()
  $path = Join-Path $PSScriptRoot ((Get-Date -Format 'yyyyMMdd-HHmmss')+'-rear-run.log')
  [IO.File]::WriteAllText($path,$log.ToString())
  "LOG=$path PORT_RELEASED"
}
