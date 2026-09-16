[CmdletBinding()]
param(
    [string] $HostName,
    [string] $User,
    [string] $RemoteDir,
    [ValidateRange(1, 65535)]
    [int] $Port,
    [string] $Identity,
    [string] $Preset,
    [switch] $Delete,
    [switch] $DryRun,
    [switch] $Build,
    [switch] $Test,
    [switch] $Restart,
    [switch] $InteractiveAuth,
    [switch] $DetailedLog,
    [string] $LogPath
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
$TranscriptStarted = $false
$WslExe = Join-Path $env:SystemRoot 'System32\wsl.exe'

try {
    if ($DetailedLog) {
        if (-not $LogPath) {
            $LogDirectory = Join-Path $ProjectRoot 'out\deploy-logs'
            $LogPath = Join-Path $LogDirectory ("deploy-{0}.log" -f (Get-Date -Format 'yyyyMMdd-HHmmss'))
        }
        else {
            if (-not [System.IO.Path]::IsPathRooted($LogPath)) {
                $LogPath = Join-Path $ProjectRoot $LogPath
            }
            $LogPath = [System.IO.Path]::GetFullPath($LogPath)
            $LogDirectory = Split-Path -Parent $LogPath
        }

        New-Item -ItemType Directory -Force -Path $LogDirectory | Out-Null
        Start-Transcript -LiteralPath $LogPath -Force | Out-Null
        $TranscriptStarted = $true
        Write-Host "Detailed deployment log: $LogPath"
    }

    if (-not (Test-Path -LiteralPath $WslExe)) {
        throw 'WSL is required for rsync deployment from Windows. Install rsync and openssh-client inside WSL.'
    }

    $DeployArguments = @()
    if ($HostName) { $DeployArguments += @('--host', $HostName) }
    if ($User) { $DeployArguments += @('--user', $User) }
    if ($RemoteDir) { $DeployArguments += @('--remote-dir', $RemoteDir) }
    if ($Port) { $DeployArguments += @('--port', $Port.ToString()) }
    if ($Identity) {
        $ResolvedIdentity = (Resolve-Path -LiteralPath $Identity).Path
        $WslIdentity = (& $WslExe wslpath -a $ResolvedIdentity).Trim()
        if ($LASTEXITCODE -ne 0 -or -not $WslIdentity) {
            throw "Could not translate SSH identity path for WSL: $ResolvedIdentity"
        }
        $DeployArguments += @('--identity', $WslIdentity)
    }
    if ($Preset) { $DeployArguments += @('--preset', $Preset) }
    if ($Delete) { $DeployArguments += '--delete' }
    if ($DryRun) { $DeployArguments += '--dry-run' }
    if ($Build) { $DeployArguments += '--build' }
    if ($Test) { $DeployArguments += '--test' }
    if ($Restart) { $DeployArguments += '--restart' }
    if ($InteractiveAuth) { $DeployArguments += '--interactive-auth' }
    if ($DetailedLog) { $DeployArguments += '--verbose' }

    Write-Host "Starting Jetson deployment at $(Get-Date -Format o)"
    & $WslExe --cd $ProjectRoot bash ./scripts/jetson/deploy.sh @DeployArguments 2>&1 |
        ForEach-Object { Write-Host $_ }
    $DeployExitCode = $LASTEXITCODE
    if ($DeployExitCode -eq 3) {
        # Sync/build/restart and RGB/depth streaming succeeded; only the mandatory IMU
        # acceptance is unmet. Surface it as its own outcome and preserve the exit code.
        Write-Warning "Jetson deployment finished with IMU ACCEPTANCE FAILED: service restarted, RGB/depth RTSP live, accel/gyro data missing."
    }
    elseif ($DeployExitCode -ne 0) {
        throw "Jetson deployment failed with exit code $DeployExitCode."
    }
    else {
        Write-Host "Jetson deployment completed at $(Get-Date -Format o)"
    }
}
finally {
    if ($TranscriptStarted) {
        Stop-Transcript | Out-Null
        Write-Host "Deployment log saved to: $LogPath"
    }
}

if ($DeployExitCode -eq 3) {
    exit 3
}
