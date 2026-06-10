#!/bin/bash
# Run all wrong-init scenarios sequentially, each for `DURATION` seconds.
# Outputs CSV time-series and summary per scenario into RESULTS_DIR.
#
# Run inside the prolab container (or any host with the workspace sourced):
#   bash run_wrong_init_batch.sh
# Optional env:
#   DURATION=60       seconds per scenario
#   RESULTS_DIR=/tmp/pro_lab_results
#   USE_RVIZ=false    (headless)
#
# Note: `set -e` is intentionally OFF. nav2_container and csv_logger
# occasionally throw SIGSEGV (-11) or SIGABRT (-6) at the end of a
# scenario shutdown after their CSVs are already flushed to disk. Those
# exit codes propagate out of `ros2 launch` and would abort the loop
# under `set -e`, costing 30+ min of compute. The flushed CSVs are fine.

DURATION=${DURATION:-40}   # 12s warmup + 15s scripted path + ~13s settle
RESULTS_DIR=${RESULTS_DIR:-/tmp/pro_lab_results}
USE_RVIZ=${USE_RVIZ:-true}
# gz GUI follows USE_RVIZ by default. When headless (USE_RVIZ=false) we also
# drop the gz GUI, which makes the launch start gz with -s --headless-rendering
# so the gpu_lidar renders offscreen via EGL on the NVIDIA dGPU (not the iGPU).
GZ_GUI=${GZ_GUI:-$USE_RVIZ}

SCENARIOS=(
  correct_init
  offset_1m
  offset_5m
  wrong_yaw_pi2
  overconfident_wrong
  underconfident
  kidnapped
)

mkdir -p "$RESULTS_DIR"
echo "Running ${#SCENARIOS[@]} scenarios @ ${DURATION}s each → $RESULTS_DIR"

for s in "${SCENARIOS[@]}"; do
  echo "─── $s ───"
  ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=$s \
    duration_s:=$DURATION \
    out_dir:=$RESULTS_DIR \
    use_rviz:=$USE_RVIZ \
    gz_gui:=$GZ_GUI
  echo "→ $RESULTS_DIR/${s}_summary.csv"
done

echo "Done. Aggregating summaries:"
echo "scenario,kf_rmse_xy,ekf_rmse_xy,pf_rmse_xy,kf_ttc,ekf_ttc,pf_ttc"
for s in "${SCENARIOS[@]}"; do
  f="$RESULTS_DIR/${s}_summary.csv"
  if [ -f "$f" ]; then
    awk -F, 'NR==2{printf "%s,%s,%s,%s,%s,%s,%s\n", $1, $3, $7, $11, $6, $10, $14}' "$f"
  fi
done
