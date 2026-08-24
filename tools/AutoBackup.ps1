[CmdletBinding()]
param(
    [string]$Repository = (Join-Path $PSScriptRoot '..'),
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
$repositoryPath = (Resolve-Path -LiteralPath $Repository).Path
$safeDirectory = $repositoryPath.Replace('\', '/')
$gitPrefix = @('-c', "safe.directory=$safeDirectory", '-C', $repositoryPath)
$mutex = New-Object System.Threading.Mutex($false, 'F407_GitHub_AutoBackup')
$hasMutex = $false

function Get-ChangeSummary {
    param([string[]]$Paths)

    $categories = New-Object System.Collections.Generic.List[string]
    $rules = @(
        @{ Pattern = '(^|/)(imu\.[ch])$';                         Name = 'IMU' },
        @{ Pattern = '(^|/)(motor|encoder|pid)\.[ch]$';           Name = 'motor/PID' },
        @{ Pattern = '(^|/)(Task|vision)\.[ch]$';                 Name = 'task/vision' },
        @{ Pattern = '(^|/)(mechanism|servo)\.[ch]$';             Name = 'mechanism/servo' },
        @{ Pattern = '(^|/)(Robot|Lcd)\.[ch]$';                   Name = 'scheduler/LCD' },
        @{ Pattern = '(^Core/|WWW\.ioc$|CMakeLists\.txt$|cmake/)'; Name = 'board/build config' },
        @{ Pattern = '(^|/)(README|CHANGELOG).*\.md$|\.md$';     Name = 'documentation' },
        @{ Pattern = '^tools/';                                   Name = 'tools' }
    )

    foreach ($rule in $rules) {
        if ($Paths -match $rule.Pattern) {
            $categories.Add($rule.Name)
        }
    }
    if ($categories.Count -eq 0) {
        $categories.Add('source files')
    }
    return ($categories -join ', ')
}

function Push-Backup {
    $env:GIT_TERMINAL_PROMPT = '0'

    & git @gitPrefix rev-parse --abbrev-ref --symbolic-full-name '@{upstream}' *> $null
    if ($LASTEXITCODE -eq 0) {
        $pushOutput = @(& git @gitPrefix push 2>&1)
        $pushExitCode = $LASTEXITCODE
        $pushOutput | ForEach-Object { Write-Host $_ }
        return $pushExitCode
    }

    $branch = (& git @gitPrefix branch --show-current).Trim()
    if (($LASTEXITCODE -ne 0) -or [string]::IsNullOrWhiteSpace($branch)) {
        Write-Warning 'Auto-backup could not determine the current Git branch.'
        return 1
    }
    $pushOutput = @(& git @gitPrefix push --set-upstream origin $branch 2>&1)
    $pushExitCode = $LASTEXITCODE
    $pushOutput | ForEach-Object { Write-Host $_ }
    return $pushExitCode
}

try {
    $hasMutex = $mutex.WaitOne(0)
    if (-not $hasMutex) {
        Write-Host 'GitHub auto-backup is already running; this request was skipped.'
        return
    }

    $statusLines = @(& git @gitPrefix status --porcelain=v1 --untracked-files=all)
    if ($LASTEXITCODE -ne 0) {
        Write-Warning 'GitHub auto-backup could not read the repository status.'
        return
    }

    if ($statusLines.Count -eq 0) {
        $ahead = (& git @gitPrefix rev-list --count '@{upstream}..HEAD' 2>$null)
        if (($LASTEXITCODE -eq 0) -and ([int]$ahead -gt 0) -and (-not $DryRun)) {
            if ((Push-Backup) -ne 0) {
                Write-Warning 'Local backup exists, but GitHub is unavailable. It will be retried after the next successful build.'
            }
        }
        return
    }

    $changedPaths = foreach ($line in $statusLines) {
        if ($line.Length -lt 4) {
            continue
        }
        $path = $line.Substring(3).Trim('"')
        if ($path.Contains(' -> ')) {
            $path = $path.Split(@(' -> '), [System.StringSplitOptions]::None)[-1]
        }
        $path.Replace('\', '/')
    }

    $blockedPath = $changedPaths | Where-Object {
        $_ -match '(^|/)(\.env($|\.)|credentials?($|\.)|secrets?($|\.))' -or
        $_ -match '\.(pem|pfx|p12|key)$'
    } | Select-Object -First 1
    if ($null -ne $blockedPath) {
        Write-Warning "Auto-backup skipped because a possible credential file changed: $blockedPath"
        return
    }

    $timestamp = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
    $summary = Get-ChangeSummary -Paths $changedPaths
    $message = "auto-backup: $timestamp | $summary"

    if ($DryRun) {
        Write-Host "Dry run: $message"
        Write-Host ($changedPaths -join [Environment]::NewLine)
        return
    }

    & git @gitPrefix add --all
    if ($LASTEXITCODE -ne 0) {
        Write-Warning 'GitHub auto-backup could not stage the changed files.'
        return
    }

    & git @gitPrefix diff --cached --quiet
    if ($LASTEXITCODE -eq 0) {
        return
    }

    & git @gitPrefix commit -m $message
    if ($LASTEXITCODE -ne 0) {
        Write-Warning 'GitHub auto-backup could not create the local commit.'
        return
    }

    if ((Push-Backup) -ne 0) {
        Write-Warning 'The local backup was created, but GitHub is unavailable. It will be retried after the next successful build.'
        return
    }
    Write-Host "GitHub backup complete: $message"
}
catch {
    Write-Warning "GitHub auto-backup failed without blocking the firmware build: $($_.Exception.Message)"
}
finally {
    if ($hasMutex) {
        $mutex.ReleaseMutex()
    }
    $mutex.Dispose()
}
