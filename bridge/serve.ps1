<#
.SYNOPSIS
  Start / stop / inspect the DS101 bridge as an INDEPENDENT background process.

.DESCRIPTION
  Why this exists. `arm_launch` starts the bridge through DSH's subprocess service, which makes
  it a CHILD of the DSH process. Restarting DSH therefore silently kills the bridge, and the arm
  panel then correctly reports "bridge 离线" -- while the operator goes looking for a bug in the
  panel. A whole debugging session was spent on exactly that.

  This script starts the bridge through the WMI process provider instead. The created process's
  parent is the WMI service, not this shell and not DSH, so it survives both. It also means the
  bridge no longer dies with the GUI it happens to be serving.

  The bridge owns the serial port and both cameras, so exactly one instance may run. `start`
  refuses rather than racing, and `stop` is always the bridge's own graceful /shutdown -- never
  a hard kill, because killing it wedges COM10 until the USB adapter is physically replugged.

.EXAMPLE
  pwsh -File bridge/serve.ps1 status
  pwsh -File bridge/serve.ps1 start
  pwsh -File bridge/serve.ps1 stop
  pwsh -File bridge/serve.ps1 restart
#>
[CmdletBinding()]
param(
    [ValidateSet('start', 'stop', 'status', 'restart')]
    [string]$Action = 'status',

    # Extra bridge arguments, e.g. -ExtraArgs '--allow-motion'
    [string[]]$ExtraArgs = @(),

    [int]$HttpPort = 8731,
    [string]$SerialPort = 'COM10',

    # Run against the in-memory plant and a synthetic camera; touches no hardware.
    [switch]$Simulate
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot          # repo root, one level above bridge/
$venvPython = Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $venvPython)) { $venvPython = 'python' }
$logDir = Join-Path $root 'calib\captures\bridge'
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$outLog = Join-Path $logDir 'bridge.out.log'
$errLog = Join-Path $logDir 'bridge.err.log'

function Get-BridgeHealth {
    try {
        return Invoke-RestMethod -Uri "http://127.0.0.1:$HttpPort/health" -TimeoutSec 3
    } catch {
        return $null
    }
}

function Get-BridgeProcess {
    # Identify by command line, never by name: the venv python.exe spawns a launcher plus a real
    # child, so several python processes exist and only one of them owns the bridge.
    Get-CimInstance Win32_Process -Filter "Name = 'python.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -and $_.CommandLine -match 'bridge[\\/]server\.py' }
}

function Start-Bridge {
    if (Get-BridgeHealth) {
        Write-Host "bridge already answering on $HttpPort - refusing to start a second instance."
        return
    }
    $existing = Get-BridgeProcess
    if ($existing) {
        Write-Warning ("a bridge process is already running (pid {0}) but is not answering; " +
                       "stop it before starting another." -f ($existing.ProcessId -join ','))
        return
    }

    $argv = @(
        "`"$venvPython`"",
        'bridge/server.py',
        '--http-port', $HttpPort,
        '--port', $SerialPort,
        '--table-z-mm', '-115',
        '--camera-forward-mm', '440',
        '--camera-height-mm', '625',
        '--camera-pitch-deg', '16'
    )
    if ($Simulate) { $argv += '--simulate' }
    if ($ExtraArgs.Count -gt 0) { $argv += $ExtraArgs }

    # The bridge writes its own log file (`--log-file`) instead of being wrapped in a shell
    # redirection. `cmd /c` quoting inside a WMI command line is a minefield: the first version
    # of this script emitted a command line containing literal backticks, so nothing ran and no
    # log was ever created. The bridge's startup lines matter -- they name the wrist camera it
    # resolved and the index it found -- so this has to actually work.
    $argv += @('--log-file', $outLog)

    $commandLine = ($argv -join ' ')

    $created = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{
        CommandLine      = $commandLine
        CurrentDirectory = $root
    }
    if ($created.ReturnValue -ne 0) {
        throw "Win32_Process.Create failed with code $($created.ReturnValue)"
    }
    Write-Host "bridge starting as an independent process (pid $($created.ProcessId), parent = WMI)"
    Write-Host "  log: $outLog"

    for ($i = 0; $i -lt 30; $i++) {
        Start-Sleep -Milliseconds 700
        $h = Get-BridgeHealth
        if ($h) {
            Write-Host "bridge is answering: port=$($h.port) motion_allowed=$($h.motion_allowed)"
            return
        }
    }
    Write-Warning "the bridge has not answered after ~21s; check $errLog and $outLog"
}

function Stop-Bridge {
    $h = Get-BridgeHealth
    if ($h) {
        # Always the bridge's own shutdown: it releases the serial port and the cameras in the
        # right order. A hard kill leaves COM10 wedged until the adapter is physically replugged.
        try {
            Invoke-RestMethod -Uri "http://127.0.0.1:$HttpPort/shutdown" -Method Post `
                -Body '{}' -ContentType 'application/json' -TimeoutSec 20 | Out-Null
            Write-Host 'bridge acknowledged shutdown'
        } catch {
            Write-Warning "shutdown request failed: $($_.Exception.Message)"
        }
        for ($i = 0; $i -lt 20; $i++) {
            Start-Sleep -Milliseconds 500
            if (-not (Get-BridgeHealth)) { break }
        }
    } else {
        Write-Host 'bridge is not answering'
    }

    # Wait for the process to actually EXIT, not merely to stop answering. /shutdown returns
    # before the process has released the serial port and the cameras, and starting a second
    # bridge during that window is how COM10 gets contended. (This loop also fixes a formatting
    # bug: "-f" binds tighter than "+", so the old warning printed a literal "pid {0}".)
    $left = Get-BridgeProcess
    for ($i = 0; $i -lt 24 -and $left; $i++) {
        Start-Sleep -Milliseconds 500
        $left = Get-BridgeProcess
    }
    if ($left) {
        $ids = ($left.ProcessId) -join ','
        Write-Warning ("still running after 12s: pid $ids. Shutdown was requested but the " +
                       "process is alive - do NOT force-kill it unless you accept having to " +
                       "replug the USB adapter; COM10 wedges.")
    } else {
        Write-Host 'no bridge process remains'
    }
}

switch ($Action) {
    'status' {
        $h = Get-BridgeHealth
        if ($h) {
            Write-Host "bridge ONLINE on $HttpPort"
            $h | ConvertTo-Json -Depth 5
        } else {
            Write-Host "bridge OFFLINE on $HttpPort"
        }
        $p = Get-BridgeProcess
        if ($p) { $p | Select-Object ProcessId, CreationDate, CommandLine | Format-List }
    }
    'start'   { Start-Bridge }
    'stop'    { Stop-Bridge }
    'restart' { Stop-Bridge; Start-Sleep -Seconds 2; Start-Bridge }
}
