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
[[F405-CALIBRATOR-BEGIN]]
F405 四路舵机滚轮微调

双击 Start.cmd，点一次“连接并读取”。以后鼠标停在哪一行，滚轮就调哪一行。
保留程序器USB、SWD和COM18接线。打开窗口不自动连接、移动或烧录，不发车轮动作。
MotionDebug保持原服务；NormalRun会使用原DEBUG进入维护。

滚轮默认每格1微秒，约0.09度指令分度，不需要点增减按钮。
鼠标停在：
- 统一动作开度行：同时微调双爪，上滚外展、下滚内收。
- 左S4或右S2行：只微调该侧安装offset，上滚外展、下滚内收。
- S1升降或S3相机行：只微调该路PWM，上滚增加输出、下滚减小输出。
  未启用的S1/S3在滚动时会按显示值启用本路，先看清目标。

空格：全开请求180度，再回到当前合爪目标；默认一次，可设置重复次数。
F2：全开并保持。F3或Esc：回到面板合爪，并结束复测。
Ctrl+S：保存标定到calibration.json，同时生成当前标定.txt。
已按人工确认更正：合爪84度、左S4补偿8微秒、右S2补偿32微秒，不因切换预设重算offset。
全开180度直接使用最大张开的物理端点，不加任何offset；安装offset只用于非全开姿态。
当前倒装：左全开500微秒、右全开2500微秒。单侧分离只给保留侧叠加offset，全开侧不加。

两爪设计是90度各内收6度，即统一动作开度84度。左右offset单独调整。
窗口上方可直接点击：起步收纳、高/低位搜索、开爪观察、合爪复审、半闭后退、
安全区预备、投送开爪、首绿抬升82度，以及两侧分离夹紧20/10/5度。
预设来自当前源码，S1起步/退出61度、水平85度、首绿预设82度。
起步收纳先收左爪再收右爪。所有预设仅运行机构姿态，不动车、不执行后退或比赛Task。
首绿预设不代表自动过滤流程已接入。选择首绿预设后，再选合爪复审可回水平85度。
查看开爪等预设不会改变已标定的合爪84度；F3始终回合爪，保留offset。
每次先全开、再从外侧回到合爪，观察重复性，不把极小PWM步长当机械精度。
复测按0.13秒/60度乘1.5估时，再加可设的到位后停留；不改比赛的等待时间。
全开期间面板保留合爪目标，滚轮修改将用于下一次合爪。

保存schema3分别记合爪标定和当前预设姿态、四路读回及启用状态，兼容旧版双爪JSON。
保存不自动修改比赛固件。当前源码参数已更正，但未烧录不会自动更新板端。
“应用已保存姿态”恢复保存的双爪及当时已启用的S1/S3；旧版文件只影响双爪。
关窗口或断开保持最后输出，释放COM18和本窗口的SWD进程，不自动归位。
新S命令、重启或恢复Task可能覆盖临时微调；S/Task角度记录不会随直接PWM覆盖更新。
S命令含义保持原样，S1/S3的窗口角度标为PWM等效角，不冒充原S逻辑角或机械实测角。
[[F405-CALIBRATOR-END]]
'@
    $readmePath = Join-Path $DataDirectory '使用说明.txt'
    $currentReadme = if (Test-Path -LiteralPath $readmePath) { [IO.File]::ReadAllText($readmePath) } else { '' }
    $blockPattern = '(?s)\[\[F405-CALIBRATOR-BEGIN\]\].*?\[\[F405-CALIBRATOR-END\]\]'
    if ([regex]::IsMatch($currentReadme, $blockPattern)) {
        $updatedReadme = [regex]::Replace($currentReadme, $blockPattern, [Text.RegularExpressions.MatchEvaluator]{ param($match) $readme })
    } else {
        if ($currentReadme.Length -gt 0) {
            $oldReadme = Join-Path $DataDirectory '使用说明_升级前备份.txt'
            if (!(Test-Path -LiteralPath $oldReadme)) { [IO.File]::WriteAllText($oldReadme, $currentReadme, [Text.UTF8Encoding]::new($true)) }
        }
        $updatedReadme = $readme + "`r`n`r`n个人笔记（标记之外的文字后续保留）：`r`n"
    }
    if ($updatedReadme -cne $currentReadme) { [IO.File]::WriteAllText($readmePath, $updatedReadme, [Text.UTF8Encoding]::new($true)) }
}
Write-Output ('APP=' + $app)
Write-Output ('CALIBRATION=' + (Join-Path $DataDirectory 'calibration.json'))
if ($BuildOnly) { return }

# This is the user-requested interactive window. Its OpenOCD helper is hidden.
$arguments = '"{0}" "{1}" "{2}" "{3}"' -f $repo, $OpenOcdExe, $DataDirectory, $Port
$process = Start-Process -FilePath $app -ArgumentList $arguments -PassThru
Write-Output ('WINDOW_PID=' + $process.Id)
