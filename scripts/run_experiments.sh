#!/usr/bin/env bash
# Run every (scenario × filter) combination for `duration` seconds, dropping
# CSVs into out_dir for later RMSE / convergence analysis. The launch file's
# auto-shutdown (duration_s arg) handles teardown, so each run is fully
# self-contained.
#
# Usage:
#   bash scripts/run_experiments.sh                       # default 60s, all
#   bash scripts/run_experiments.sh --duration 30
#   bash scripts/run_experiments.sh --filter pf
#   bash scripts/run_experiments.sh --scenario offset_5m
#
# Stack must already be up (start_all.sh --skip-browser). Gazebo + bridges
# stay running across runs; only the filter / metrics / csv_logger nodes
# restart per scenario.
set -uo pipefail

DURATION=60
OUT_DIR="/tmp/pro_lab_results"
SCENARIOS_FILTER="*"
FILTERS_FILTER="*"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)  DURATION="$2"; shift 2 ;;
    --out)       OUT_DIR="$2";  shift 2 ;;
    --scenario)  SCENARIOS_FILTER="$2"; shift 2 ;;
    --filter)    FILTERS_FILTER="$2";   shift 2 ;;
    -h|--help)   sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

c_cyan() { printf '\033[36m%s\033[0m\n' "$*"; }
c_green(){ printf '\033[32m%s\033[0m\n' "$*"; }
c_red()  { printf '\033[31m%s\033[0m\n' "$*" >&2; }

# Discover scenarios from the installed share/ tree (single source of truth).
SCENARIOS=$(docker exec prolab_jazzy bash -lc \
  'ls /home/ros/ws/install/pro_lab_filters/share/pro_lab_filters/config/scenarios/ \
    | sed "s/\.yaml$//"' | tr -d '\r')
FILTERS="kf ekf pf"

total=0; done_=0
for s in $SCENARIOS; do
  [[ "$SCENARIOS_FILTER" != "*" && "$s" != "$SCENARIOS_FILTER" ]] && continue
  for f in $FILTERS; do
    [[ "$FILTERS_FILTER" != "*" && "$f" != "$FILTERS_FILTER" ]] && continue
    total=$((total+1))
  done
done

c_cyan "Running $total experiments  (each ${DURATION}s)"
c_cyan "Outputs -> $OUT_DIR (inside container; mapped if you have a volume)"
echo

for s in $SCENARIOS; do
  [[ "$SCENARIOS_FILTER" != "*" && "$s" != "$SCENARIOS_FILTER" ]] && continue
  for f in $FILTERS; do
    [[ "$FILTERS_FILTER" != "*" && "$f" != "$FILTERS_FILTER" ]] && continue
    done_=$((done_+1))
    c_cyan "[$done_/$total] scenario=$s  filter=$f"
    # Tear down the whole previous launch (gz_server included) so each
    # scenario starts from a clean slate. start_gz:=true means each
    # launch spawns its own gz; collisions if we leave the old one alive.
    docker exec prolab_jazzy bash -lc '
      pkill -9 -f wrong_init_experiment 2>/dev/null
      pkill -9 -f gz                  2>/dev/null
      pkill -9 -f parameter_bridge    2>/dev/null
      pkill -9 -f image_bridge        2>/dev/null
      pkill -9 -f kf_node             2>/dev/null
      pkill -9 -f ekf_node            2>/dev/null
      pkill -9 -f pf_node             2>/dev/null
      pkill -9 -f truth_relay         2>/dev/null
      pkill -9 -f map_odom_tf         2>/dev/null
      pkill -9 -f map_server          2>/dev/null
      pkill -9 -f lifecycle_manager   2>/dev/null
      pkill -9 -f csv_logger          2>/dev/null
      pkill -9 -f metrics_node        2>/dev/null
      pkill -9 -f landmark_detector   2>/dev/null
      pkill -9 -f teleop_marker       2>/dev/null
      pkill -9 -f cmd_vel_watchdog    2>/dev/null
      pkill -9 -f map_republisher     2>/dev/null
      pkill -9 -f robot_teleporter    2>/dev/null
      pkill -9 -f robot_state         2>/dev/null
      rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
      sleep 2; true
    ' >/dev/null 2>&1
    docker exec prolab_jazzy bash -lc "
      cd /home/ros/ws &&
      source /opt/ros/jazzy/setup.bash &&
      source install/setup.bash &&
      ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
        scenario:=$s filter:=$f duration_s:=$DURATION \
        use_rviz:=false use_foxglove:=false start_gz:=true use_nav2:=false \
        out_dir:=$OUT_DIR \
        > /tmp/exp_${s}_${f}.log 2>&1
    " || c_red "  run failed — see /tmp/exp_${s}_${f}.log inside container"
  done
done

c_green "Done. CSVs in $OUT_DIR (inside prolab_jazzy)."
echo "Copy out:  docker cp prolab_jazzy:$OUT_DIR ./results"
