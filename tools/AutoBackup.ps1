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
        & git @gitPrefix push
        return $LASTEXITCODE
    }

    $branch = (& git @gitPrefix branch --show-current).Trim()
    if (($LASTEXITCODE -ne 0) -or [string]::IsNullOrWhiteSpace($branch)) {
        return 1
    }
    & git @gitPrefix push --set-upstream origin $branch
    return $LASTEXITCODE
}

function Assert-GitHubSynchronized {
    $localHead = (& git @gitPrefix rev-parse HEAD).Trim()
    if ($LASTEXITCODE -ne 0) {
        throw 'Could not read the local HEAD after push.'
    }
    $upstreamHead = (& git @gitPrefix rev-parse '@{upstream}').Trim()
    if ($LASTEXITCODE -ne 0) {
        throw 'Could not read the upstream branch after push.'
    }
    if ($localHead -ne $upstreamHead) {
        throw "GitHub verification failed: local HEAD $localHead differs from upstream $upstreamHead."
    }
    Write-Host "GitHub synchronized: $localHead"
}

$backupFailed = $false
try {
    $hasMutex = $mutex.WaitOne(0)
    if (-not $hasMutex) {
        throw 'Another GitHub backup is still running.'
    }

    $statusLines = @(& git @gitPrefix status --porcelain=v1 --untracked-files=all)
    if ($LASTEXITCODE -ne 0) {
        throw 'Could not read the repository status.'
    }

    if ($statusLines.Count -eq 0) {
        if ($DryRun) {
            Write-Host 'Dry run: working tree is clean.'
            return
        }
        if ((Push-Backup) -ne 0) {
            throw 'GitHub push failed; build/flash is blocked.'
        }
        Assert-GitHubSynchronized
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
        throw "Possible credential file changed: $blockedPath. Build/flash is blocked."
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
        throw 'Could not stage the changed files.'
    }

    & git @gitPrefix diff --cached --quiet
    if ($LASTEXITCODE -eq 0) {
        return
    }

    & git @gitPrefix commit -m $message
    if ($LASTEXITCODE -ne 0) {
        throw 'Could not create the local backup commit.'
    }

    if ((Push-Backup) -ne 0) {
        throw 'GitHub push failed after the local commit; build/flash is blocked.'
    }
    Assert-GitHubSynchronized
    Write-Host "GitHub backup complete: $message"
}
catch {
    $backupFailed = $true
    Write-Error "GitHub auto-backup failed: $($_.Exception.Message)"
}
finally {
    if ($hasMutex) {
        $mutex.ReleaseMutex()
    }
    $mutex.Dispose()
}

if ($backupFailed) {
    exit 1
}
