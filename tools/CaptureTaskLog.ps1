param(
    [ValidatePattern('^COM[0-9]+$')][string]$Port = 'COM18',
    [ValidateRange(1200, 1000000)][int]$Baud = 115200,
    [string]$OutputDirectory = (Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'F405TaskLogs')
)
$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$writer = $null
try {
    $serial.Open()
    [IO.Directory]::CreateDirectory($OutputDirectory) | Out-Null
    $file = Join-Path $OutputDirectory ('task_{0}_{1}.log' -f (Get-Date -Format 'yyyyMMdd_HHmmss_fff'), $Port)
    $writer = [IO.StreamWriter]::new($file, $false, [Text.UTF8Encoding]::new($false))
    $writer.AutoFlush = $true
    $writer.WriteLine('# Capture started {0:o}; port={1}; baud={2}; receive-only', (Get-Date), $Port, $Baud)
    Write-Host "Recording to $file"
    Write-Host 'Receive only. Ctrl+C stops recording; it does NOT stop the robot.'
    while ($true) {
        $text = $serial.ReadExisting()
        if ($text.Length -gt 0) {
            $writer.Write($text)
            [Console]::Write($text)
        }
        Start-Sleep -Milliseconds 50
    }
} finally {
    if ($null -ne $writer) { $writer.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
