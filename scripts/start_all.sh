#!/usr/bin/env bash
# PRO-LAB one-shot launcher.
#
# Idempotent: safe to run multiple times. Stops stale processes first,
# then brings everything up in dependency order with readiness checks
# instead of fixed sleeps. Aborts loudly on the first failure.
#
# Pipeline:
#   1) Stop any old gz sim, ros2 launch, container processes.
#   2) Start native Gazebo headlessly (GPU server).
#      Wait until GZ scene info is reachable on partition 'prolab'.
#   3) Start docker stack (prolab + foxglove_ui + ws_publish).
#   4) Build workspace inside the prolab container.
#   5) Launch wrong_init_experiment ROS stack inside the container.
#      Wait until /pf/pose actually publishes (= filters connected to GZ).
#   6) Open Lichtblick in default browser (works in WSL via cmd.exe, in
#      pure Linux via xdg-open).
#
# Anything broken? -> output points at the right log:
#     ~/gz_sim.log        (Gazebo)
#     /tmp/build.log      (colcon build inside container)
#     /tmp/launch.log     (ros2 launch inside container)
#
# Usage:
#   bash scripts/start_all.sh
#   bash scripts/start_all.sh --scenario offset_5m
#   bash scripts/start_all.sh --skip-build
#   bash scripts/start_all.sh --skip-browser

set -uo pipefail

# ---------- args -------------------------------------------------------------

SCENARIO="correct_init"
VIZ="foxglove"           # foxglove | rviz | none
SKIP_BUILD=0
SKIP_BROWSER=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)      SCENARIO="$2"; shift 2 ;;
    --viz)           VIZ="$2"; shift 2 ;;
    --rviz)          VIZ="rviz"; shift ;;
    --foxglove)      VIZ="foxglove"; shift ;;
    --skip-build)    SKIP_BUILD=1; shift ;;
    --skip-browser)  SKIP_BROWSER=1; shift ;;
    -h|--help)
      sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "$VIZ" in
  foxglove|rviz|none) ;;
  *) echo "--viz must be foxglove|rviz|none, got: $VIZ" >&2; exit 2 ;;
esac

USE_RVIZ=false
USE_FOX=false
[[ "$VIZ" == "rviz" ]]     && USE_RVIZ=true
[[ "$VIZ" == "foxglove" ]] && USE_FOX=true

REPO="$(cd "$(dirname "$0")/.." && pwd)"
GZ_LOG="$HOME/gz_sim.log"
URL="http://localhost:8082/?ds=foxglove-websocket&ds.url=ws://127.0.0.1:8767"

# ---------- helpers ----------------------------------------------------------

c_cyan()  { printf '\033[36m%s\033[0m\n' "$*"; }
c_gray()  { printf '\033[90m%s\033[0m\n' "$*"; }
c_green() { printf '\033[32m%s\033[0m\n' "$*"; }
c_red()   { printf '\033[31m%s\033[0m\n' "$*" >&2; }

in_container() {
  # Run a bash one-liner inside prolab_jazzy, ROS sourced.
  docker exec prolab_jazzy bash -lc "
    source /opt/ros/jazzy/setup.bash
    [ -f /home/ros/ws/install/setup.bash ] && source /home/ros/ws/install/setup.bash
    $*
  "
}

wait_for() {
  # wait_for <timeout_sec> <description> <probe_cmd...>
  local timeout="$1"; shift
  local what="$1"; shift
  local deadline=$(( $(date +%s) + timeout ))
  local tries=0
  while (( $(date +%s) < deadline )); do
    tries=$((tries+1))
    if eval "$@" >/dev/null 2>&1; then
      c_gray "      ready after $tries tries"
      return 0
    fi
    sleep 2
  done
  c_red "timeout waiting for: $what (after ${timeout}s)"
  return 1
}

open_browser() {
  if grep -qi microsoft /proc/version 2>/dev/null; then
    # WSL -> open in Windows default browser
    cmd.exe /c start "" "$URL" >/dev/null 2>&1 || true
  elif command -v xdg-open >/dev/null; then
    xdg-open "$URL" >/dev/null 2>&1 &
  else
    echo "open this in your browser: $URL"
  fi
}

# ---------- 1) cleanup -------------------------------------------------------

c_cyan "[1/6] Cleaning up stale processes..."
# Native-gz remnants from older runs (we now run gz inside the container,
# but a leftover wsl-host gz can race for /clock and confuse things).
pkill -9 -f 'gz sim'      2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
# Pkill from inside the container often misses launch-supervised respawns
# and leaks bridges/map_servers across runs, which causes duplicate /clock
# publishers and disconnected TF trees. Restarting the container itself is
# the only reliable nuke. Cheap (~3s), idempotent, and required.
if docker ps -q -f name=prolab_jazzy | grep -q .; then
  docker restart prolab_jazzy >/dev/null
fi
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 3

# ---------- 2) Gazebo --------------------------------------------------------

c_cyan "[2/6] Starting native Gazebo..."
setsid nohup "$REPO/scripts/run_gazebo_native.sh" > "$GZ_LOG" 2>&1 < /dev/null &
disown || true

c_gray "      waiting for 'World [...] initialized' in $GZ_LOG..."
if ! wait_for 90 "Gazebo world init" \
     "grep -q 'initialized with' '$GZ_LOG'"
then
  c_red "Gazebo did not come up — last lines of $GZ_LOG:"
  tail -30 "$GZ_LOG" >&2 || true
  exit 1
fi

# ---------- 3) Docker stack --------------------------------------------------

c_cyan "[3/6] Starting docker stack..."
( cd "$REPO/docker" && docker compose up -d prolab foxglove_ui ws_publish ) >/dev/null

wait_for 30 "prolab_jazzy container running" \
  "[ \"\$(docker inspect -f '{{.State.Running}}' prolab_jazzy 2>/dev/null)\" = true ]" \
  || exit 1

# ---------- 4) Build ---------------------------------------------------------

if (( SKIP_BUILD == 0 )); then
  c_cyan "[4/6] Building workspace (colcon)..."
  if ! in_container "
        cd /home/ros/ws &&
        colcon build --symlink-install --packages-select pro_lab_filters \
          > /tmp/build.log 2>&1 && echo BUILD_OK
      " | grep -q BUILD_OK
  then
    c_red "BUILD FAILED — last lines of /tmp/build.log:"
    in_container "tail -30 /tmp/build.log" >&2 || true
    exit 1
  fi
else
  c_gray "[4/6] Skipping build (--skip-build)"
fi

# ---------- 5) Launch ROS stack ---------------------------------------------

c_cyan "[5/6] Launching wrong_init_experiment (scenario=$SCENARIO, viz=$VIZ)..."
docker exec -d prolab_jazzy bash -lc "
  cd /home/ros/ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=$SCENARIO use_rviz:=$USE_RVIZ use_foxglove:=$USE_FOX \
    start_gz:=true use_nav2:=false \
    > /tmp/launch.log 2>&1
"

c_gray "      waiting for /pf/pose to publish (non-fatal)..."
if ! wait_for 30 "/pf/pose publishing" \
     "in_container 'timeout 2 ros2 topic hz /pf/pose 2>&1 | grep -q \"average rate\"'"
then
  c_red "WARN: /pf/pose did not publish in 30s — opening viz anyway so you can debug."
  c_gray "      tail -f /tmp/launch.log to investigate."
fi

# ---------- 6) Browser -------------------------------------------------------

if (( SKIP_BROWSER == 0 )) && [[ "$VIZ" == "foxglove" ]]; then
  c_cyan "[6/6] Opening Lichtblick in browser..."
  open_browser
elif [[ "$VIZ" == "rviz" ]]; then
  c_gray "[6/6] viz=rviz — RViz window opened by launch file"
else
  c_gray "[6/6] Skipping browser"
fi

echo
c_green "Stack is up. scenario=$SCENARIO"
c_gray "Tail logs:"
c_gray "  tail -f $GZ_LOG"
c_gray "  docker exec prolab_jazzy tail -f /tmp/launch.log"
