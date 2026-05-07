#!/bin/bash
# Run all wrong-init scenarios sequentially, each for `DURATION` seconds.
# Outputs CSV time-series and summary per scenario into RESULTS_DIR.
#
# Run inside the prolab container (or any host with the workspace sourced):
#   bash run_wrong_init_batch.sh
# Optional env:
#   DURATION=60       seconds per scenario
#   RESULTS_DIR=/tmp/pro_lab_results
#   USE_RVIZ=false USE_FOXGLOVE=false   (headless)
set -e

DURATION=${DURATION:-60}
RESULTS_DIR=${RESULTS_DIR:-/tmp/pro_lab_results}
USE_RVIZ=${USE_RVIZ:-false}
USE_FOXGLOVE=${USE_FOXGLOVE:-false}

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
    use_foxglove:=$USE_FOXGLOVE
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
