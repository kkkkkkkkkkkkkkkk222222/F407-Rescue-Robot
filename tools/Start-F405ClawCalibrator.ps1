param(
    [string]$DataDirectory,
    [string]$OpenOcdExe,
    [ValidatePattern('^COM[0-9]+$')][string]$Port = 'COM18',
    [switch]$Install,
    [switch]$BuildOnly
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
if (!$DataDirectory) {
    $DataDirectory = Join-Path ([Environment]::GetFolderPath('DesktopDirectory')) 'F405 Claw Calibration'
}
[void][IO.Directory]::CreateDirectory($DataDirectory)
$DataDirectory = (Get-Item -LiteralPath $DataDirectory).FullName.TrimEnd('\')
if (!$OpenOcdExe) { $OpenOcdExe = (Get-Command openocd.exe -CommandType Application).Source }
$OpenOcdExe = (Get-Item -LiteralPath $OpenOcdExe).FullName
$csc = Join-Path $env:WINDIR 'Microsoft.NET/Framework64/v4.0.30319/csc.exe'
if (!(Test-Path -LiteralPath $csc -PathType Leaf)) {
    $csc = Join-Path $env:WINDIR 'Microsoft.NET/Framework/v4.0.30319/csc.exe'
}
if (!(Test-Path -LiteralPath $csc -PathType Leaf)) { throw '.NET Framework C# compiler not found.' }
$source = Join-Path $PSScriptRoot 'F405ClawCalibrator.cs'
$app = Join-Path $DataDirectory 'F405ClawCalibrator.exe'
$hashPath = Join-Path $DataDirectory 'build-source.sha256'
$hash = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash
$previous = if (Test-Path -LiteralPath $hashPath) { [IO.File]::ReadAllText($hashPath).Trim() } else { '' }
if (!(Test-Path -LiteralPath $app) -or $previous -ne $hash) {
    & $csc /nologo /target:winexe /platform:anycpu /codepage:65001 `
        /reference:System.Windows.Forms.dll /reference:System.Drawing.dll `
        /reference:System.Web.Extensions.dll "/out:$app" $source
    if ($LASTEXITCODE -ne 0) { throw 'Calibration window build failed; no board action was sent.' }
    [IO.File]::WriteAllText($hashPath, $hash, [Text.Encoding]::ASCII)
}

if ($Install) {
    $launcher = @"
@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$PSCommandPath" -DataDirectory "%~dp0." -OpenOcdExe "$OpenOcdExe" -Port "$Port"
if errorlevel 1 pause
"@
    [IO.File]::WriteAllText((Join-Path $DataDirectory 'Start.cmd'), $launcher, [Text.Encoding]::ASCII)
    $readme = @'
F405 双爪微调窗口

双击 Start.cmd。仅调双爪，不发车轮、升降、相机指令，不烧录。
保留程序器的USB、SWD和COM18接线。窗口开启时不会自动连接或移动。

1. 点“连接并读取”，读取当前两爪脉宽，不把它当实际角度反馈。
   MotionDebug保持原服务；NormalRun会发DEBUG进入维护。
2. 当前从旧合爪S4=86、S2=90开始。需要重回起点，点“恢复旧合爪”。
3. “双爪内收/外展”改变统一动作开度；左右offset保持。
   左右各自的“内收/外展”只调该侧安装offset。
4. 步长默认1微秒，约等于0.09度指令分度；也能选2/5/10/20微秒。
   舵机实际最小响应由实物决定。0度仅是内收参考，不代表机械可达。
5. 点“保存标定”保存calibration.json；每次调节另记JSONL。
   保存不会自动改固件。调好后把这个文件交给Codex固化。
6. “断开并保持”或关窗口都保持最后脉宽，释放COM18与SWD；不自动归位。

S命令含义未变。这里的统一开度两侧都是小值内收、大值外展。
窗口分开显示统一开度与左右offset。初次读入的offset只是维持当前姿态的起始值，
不能由软件自动证明安装已校准。可以在任意开度检查两爪实际对称性。

临时微调会被新的S指令、重启或恢复Task覆盖；保存文件可用于再次应用。
原S/Task目标角不会随临时PWM表覆盖更新，请看窗口脉宽读回及保存文件。
需要重新连接/启用PWM时，可点“恢复旧合爪”，它只发原S2/S4命令。
25齿齿距14.4度；具体offset要按当前实物调，不能拿齿距代替实测。
'@
    $readmePath = Join-Path $DataDirectory '使用说明.txt'
    if (!(Test-Path -LiteralPath $readmePath)) {
        [IO.File]::WriteAllText($readmePath, $readme, [Text.UTF8Encoding]::new($true))
    }
}
Write-Output ('APP=' + $app)
Write-Output ('CALIBRATION=' + (Join-Path $DataDirectory 'calibration.json'))
if ($BuildOnly) { return }

# This is the user-requested interactive window. Its OpenOCD helper is hidden.
$arguments = '"{0}" "{1}" "{2}" "{3}"' -f $repo, $OpenOcdExe, $DataDirectory, $Port
$process = Start-Process -FilePath $app -ArgumentList $arguments -PassThru
Write-Output ('WINDOW_PID=' + $process.Id)
