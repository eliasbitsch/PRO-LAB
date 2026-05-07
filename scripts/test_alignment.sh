#!/usr/bin/env bash
# Alignment-Diagnose-Test:
#  - Startet den vollen Stack mit `correct_init` Szenario
#    (PF-Init exakt auf GZ-Spawn, sehr enger Spread)
#  - Kein Kidnap, kein /initialpose-Klick nötig
#
# Was du dann in Foxglove pruefst:
#   1) Map-Panel: aligned die OccupancyGrid mit den /scan Punkten?
#   2) TF-Panel: ist map -> odom ungefaehr Identity (alle ~0)?
#
# Beantworte danach mit ja/nein:
#   ja  -> alignment OK, das urspruengliche Problem war Filter-Konvergenz
#          aus einem Wrong-Init-Szenario (erwartet)
#   nein -> echtes Frame-Problem (Map-Origin oder truth_relay frame)
set -euo pipefail

cd "$(dirname "$0")/.."

source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash

echo "========================================================="
echo " Alignment test: scenario=correct_init"
echo " PF init: x=2.12, y=-21.3, yaw=1.57, spread=0.10m / 0.05rad"
echo " (kein Kidnap, kein /initialpose noetig)"
echo "========================================================="
echo

ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=correct_init \
    duration_s:=0
