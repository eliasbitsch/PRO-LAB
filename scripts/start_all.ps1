# PRO-LAB one-shot launcher (Windows).
#
# Idempotent: safe to run multiple times. Stops stale processes first,
# then brings everything up in dependency order with readiness checks
# instead of fixed sleeps. Aborts loudly on the first failure.
#
# Pipeline:
#   1) Stop any old gz sim, ros2 launch, container processes.
#   2) Start native Gazebo headlessly in WSL2 (GPU server).
#      Wait until /world/<world>/scene/info answers (= sim is alive).
#   3) Start docker stack (prolab + foxglove_ui + ws_publish).
#   4) Build workspace inside the prolab container.
#   5) Launch wrong_init_experiment ROS stack inside the container.
#      Wait until /pf/pose actually publishes (= filters connected to GZ).
#   6) Open Lichtblick in default browser.
#
# Anything broken? -> output points at the right log:
#     ~/gz_sim.log        (Gazebo)
#     /tmp/build.log      (colcon build inside container)
#     /tmp/launch.log     (ros2 launch inside container)
#
# Usage:
#   pwsh scripts\start_all.ps1                          # default scenario
#   pwsh scripts\start_all.ps1 -Scenario offset_5m
#   pwsh scripts\start_all.ps1 -SkipBuild               # don't rebuild
#   pwsh scripts\start_all.ps1 -SkipBrowser
param(
    [string]$Scenario   = "correct_init",
    [switch]$SkipBuild  = $false,
    [switch]$SkipBrowser = $false
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot

# ---------- helpers ----------------------------------------------------------

function Wsl-Bash([string]$cmd) {
    # Run a bash one-liner in WSL. We use single-quoted body always to avoid
    # PowerShell variable interpolation surprises. Caller escapes its own quotes.
    wsl -- bash -lc $cmd
}

function In-Container([string]$cmd) {
    # Run a bash one-liner inside the prolab_jazzy container, sourced.
    $sourced = "source /opt/ros/jazzy/setup.bash && " +
               "[ -f /home/ros/ws/install/setup.bash ] && source /home/ros/ws/install/setup.bash; " +
               $cmd
    return Wsl-Bash "docker exec prolab_jazzy bash -lc `"$sourced`""
}

function Wait-For([scriptblock]$Probe, [string]$What, [int]$TimeoutSec = 60) {
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    $tries = 0
    while ((Get-Date) -lt $deadline) {
        $tries++
        try {
            if (& $Probe) {
                Write-Host "      ready after $tries tries" -ForegroundColor DarkGray
                return $true
            }
        } catch { }
        Start-Sleep -Seconds 2
    }
    throw "timeout waiting for: $What (after ${TimeoutSec}s)"
}

# ---------- 1) cleanup -------------------------------------------------------

Write-Host "[1/6] Cleaning up stale processes..." -ForegroundColor Cyan
Wsl-Bash "pkill -9 -f 'gz sim'        2>/dev/null; true" | Out-Null
Wsl-Bash "pkill -9 -f 'ros2 launch'   2>/dev/null; true" | Out-Null
# Kill any stale ros2 launch *inside* the container too:
Wsl-Bash "docker ps -q -f name=prolab_jazzy | xargs -r -I{} docker exec {} bash -lc `"pkill -9 -f wrong_init_experiment 2>/dev/null; true`"" | Out-Null
Start-Sleep -Seconds 2

# ---------- 2) Gazebo --------------------------------------------------------

Write-Host "[2/6] Starting native Gazebo in WSL..." -ForegroundColor Cyan
Wsl-Bash "setsid nohup /mnt/c/git/PRO-LAB/scripts/run_gazebo_native.sh > /home/`$USER/gz_sim.log 2>&1 < /dev/null & disown; echo gz-spawned" | Out-Null

Write-Host "      waiting for gz scene info on partition 'prolab'..." -ForegroundColor DarkGray
Wait-For -TimeoutSec 90 -What "Gazebo scene info" -Probe {
    $out = Wsl-Bash "GZ_PARTITION=prolab timeout 2 gz topic -l 2>/dev/null | grep -c '/scene/info' || true"
    return ($out -match '^\s*[1-9]')
}

# ---------- 3) Docker stack --------------------------------------------------

Write-Host "[3/6] Starting docker stack (prolab, foxglove_ui, ws_publish)..." -ForegroundColor Cyan
Wsl-Bash "cd /mnt/c/git/PRO-LAB/docker && docker compose up -d prolab foxglove_ui ws_publish" | Out-Null

Wait-For -TimeoutSec 30 -What "prolab_jazzy container running" -Probe {
    $state = Wsl-Bash "docker inspect -f '{{.State.Running}}' prolab_jazzy 2>/dev/null || echo false"
    return ($state -match 'true')
}

# ---------- 4) Build ---------------------------------------------------------

if (-not $SkipBuild) {
    Write-Host "[4/6] Building workspace (colcon)..." -ForegroundColor Cyan
    $buildCmd = "cd /home/ros/ws && source /opt/ros/jazzy/setup.bash && " +
                "colcon build --symlink-install --packages-select pro_lab_filters > /tmp/build.log 2>&1 && echo BUILD_OK"
    $out = Wsl-Bash "docker exec prolab_jazzy bash -lc `"$buildCmd`""
    if ($out -notmatch 'BUILD_OK') {
        Write-Host "BUILD FAILED — last lines of /tmp/build.log:" -ForegroundColor Red
        Wsl-Bash "docker exec prolab_jazzy bash -lc 'tail -30 /tmp/build.log'"
        throw "colcon build failed"
    }
} else {
    Write-Host "[4/6] Skipping build (-SkipBuild)" -ForegroundColor DarkGray
}

# ---------- 5) Launch ROS stack ---------------------------------------------

Write-Host "[5/6] Launching wrong_init_experiment (scenario=$Scenario)..." -ForegroundColor Cyan
$launchCmd = "cd /home/ros/ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && " +
             "ros2 launch pro_lab_filters wrong_init_experiment.launch.py " +
             "scenario:=$Scenario use_rviz:=false use_foxglove:=true start_gz:=false " +
             "> /tmp/launch.log 2>&1"
Wsl-Bash "docker exec -d prolab_jazzy bash -lc `"$launchCmd`"" | Out-Null

Write-Host "      waiting for /pf/pose to publish (= filters fed by GZ)..." -ForegroundColor DarkGray
try {
    Wait-For -TimeoutSec 60 -What "/pf/pose publishing" -Probe {
        $out = In-Container "timeout 2 ros2 topic hz /pf/pose 2>&1 | grep -c 'average rate' || true"
        return ($out -match '^\s*[1-9]')
    }
} catch {
    Write-Host "LAUNCH NOT READY — last lines of /tmp/launch.log:" -ForegroundColor Red
    Wsl-Bash "docker exec prolab_jazzy bash -lc 'tail -30 /tmp/launch.log'"
    throw
}

# ---------- 6) Browser -------------------------------------------------------

if (-not $SkipBrowser) {
    Write-Host "[6/6] Opening Lichtblick in browser..." -ForegroundColor Cyan
    Start-Process "http://localhost:8082/?ds=foxglove-websocket&ds.url=ws://127.0.0.1:8767"
} else {
    Write-Host "[6/6] Skipping browser (-SkipBrowser)" -ForegroundColor DarkGray
}

Write-Host ""
Write-Host "Stack is up. scenario=$Scenario" -ForegroundColor Green
Write-Host "Tail logs:" -ForegroundColor Gray
Write-Host "  wsl tail -f /home/`$USER/gz_sim.log" -ForegroundColor Gray
Write-Host "  wsl docker exec prolab_jazzy tail -f /tmp/launch.log" -ForegroundColor Gray
