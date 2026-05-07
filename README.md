# PRO Lab — Probabilistic Robotics with ROS 2 & TurtleBot 4

KF, EKF and PF state estimation on a simulated TurtleBot 4 in the Nav2
warehouse world, focused on the **Wrong Initialization** task.

> Student: Elias Bitsch · Group 2B1 · Task ID 2510331007

---

## What's in here

```
ws/src/pro_lab_filters/
├── include/pro_lab_filters/
│   ├── KF.h, EKF.h, PF.h           # standalone (ROS-free) filter classes
│   ├── LikelihoodField.h           # 2-pass Euclidean DT for /scan likelihood
│   ├── teleop_panel.hpp            # Foxglove-style RViz teleop dpad
│   └── kidnap_tool.hpp             # RViz tool: click-to-teleport
├── src/
│   ├── kf_node.cpp, ekf_node.cpp, pf_node.cpp     # 3 filter nodes
│   ├── truth_relay_node.cpp                       # ground truth from TF
│   ├── metrics_node.cpp                           # RMSE / convergence
│   ├── map_odom_tf_publisher.cpp                  # AMCL-style map→odom
│   ├── robot_teleporter.cpp                       # GZ live-teleport
│   ├── landmark_detector_node.cpp                 # self-defined landmarks
│   ├── teleop_panel.cpp, kidnap_tool.cpp          # RViz plugins
│   └── teleop_marker_node.cpp                     # interactive_markers teleop
├── launch/
│   ├── wrong_init_experiment.launch.py    # master launch (filter:= arg)
│   ├── kf_only.launch.py                  # single-filter wrappers
│   ├── ekf_only.launch.py
│   └── pf_only.launch.py
├── config/
│   ├── scenarios/                         # wrong-init + Q/R YAMLs
│   ├── landmarks.yaml
│   ├── wrong_init.rviz
│   └── gz_bridge.yaml
└── scripts/csv_logger.py                  # per-run CSV writer
docker/                                    # WSL2 + Docker setup
scripts/
├── start_all.sh                           # one-shot bringup (gz + ros + viz)
├── run_experiments.sh                     # batch all scenarios × all filters
└── analyze_results.py                     # RMSE plots from CSVs
```

---

## Assignment ↔ Code mapping

| Requirement                        | Where                                                                |
| ---------------------------------- | -------------------------------------------------------------------- |
| **Implement KF**                   | `src/kf_node.cpp` + `include/pro_lab_filters/KF.h`                   |
| **Implement EKF**                  | `src/ekf_node.cpp` + `include/pro_lab_filters/EKF.h`                 |
| **Implement PF**                   | `src/pf_node.cpp` + `include/pro_lab_filters/PF.h`                   |
| Common system setup                | All filters consume same `/cmd_vel`, `/imu`, `/pose`, `/scan`, `/map`. Same TF tree (`map → odom → base_footprint`). Same scenario YAMLs. |
| **Q variation**                    | `config/scenarios/q_low.yaml`, `q_high.yaml`                         |
| **R variation**                    | `config/scenarios/r_low.yaml`, `r_high.yaml`                         |
| **Runtime / performance**          | metrics_node + per-filter rates from `analyze_results.py`            |
| **Ground-truth evaluation (RMSE)** | `truth_relay_node` + `metrics_node` → CSV → analyze script           |
| **Landmark detection**             | `src/landmark_detector_node.cpp` + `config/landmarks.yaml`           |
| **Wrong Init task**                | scenarios `correct_init / offset_1m / offset_5m / wrong_yaw_pi2 / overconfident_wrong / underconfident / kidnapped` |

PF additionally implements (textbook AMCL):

- Likelihood-field scan update (Probabilistic Robotics §6.4)
- Augmented-MCL kidnap recovery (§8.3.5)
- Velocity motion model with α₁..α₆ noise (§5.3 Table 5.3)
- Beam skip for dynamic obstacles (AMCL params)
- KLD-sampling for adaptive particle count (§4.3.4)

---

## Running

### Bring up the full stack

```bash
bash scripts/start_all.sh --rviz                       # gz + filters + rviz
bash scripts/start_all.sh --foxglove                   # gz + filters + lichtblick
bash scripts/start_all.sh --rviz --scenario offset_5m
```

### Single-filter testing

```bash
ros2 launch pro_lab_filters kf_only.launch.py  scenario:=offset_5m
ros2 launch pro_lab_filters ekf_only.launch.py scenario:=overconfident_wrong
ros2 launch pro_lab_filters pf_only.launch.py  scenario:=kidnapped
```

### Run all experiments → CSVs

```bash
bash scripts/run_experiments.sh --duration 60       # ~9 scenarios × 3 filters
bash scripts/run_experiments.sh --scenario q_low    # one scenario, all filters
bash scripts/run_experiments.sh --filter pf         # one filter, all scenarios
docker cp prolab_jazzy:/tmp/pro_lab_results ./results
```

### Analyse → plots

```bash
python3 scripts/analyze_results.py --in ./results --out ./results
# → ./results/<scenario>_error.png      per-scenario RMSE timelines
# → ./results/rmse_comparison.png       cross-scenario bar chart
# → ./results/all_summaries.csv         table for the paper
```

### Live UI

- **RViz**: full layout incl. teleop panel, kidnap tool, landmark markers,
  particle cloud, all 3 filter pose arrows.
- **Lichtblick** (Foxglove fork): same setup at <http://localhost:8082>.

---

## Scenarios

| scenario              | what it tests                                       |
| --------------------- | --------------------------------------------------- |
| `correct_init`        | baseline — init at truth, tight spread              |
| `offset_1m`           | mild wrong init, spread covers truth                |
| `offset_5m`           | severe wrong init, spread does **not** cover truth  |
| `wrong_yaw_pi2`       | yaw rotated 90° — non-linear failure mode           |
| `overconfident_wrong` | wrong + tiny spread — particle deprivation for PF   |
| `underconfident`      | right pose, huge covariance — slow KF/EKF           |
| `kidnapped`           | uniform global init, PF only feasible               |
| `q_low / q_high`      | process noise variation                             |
| `r_low / r_high`      | measurement noise variation                         |

---

## Setup notes

- ROS 2 Jazzy + Gazebo Harmonic, native gz on WSL2 + ROS in Docker for GPU
  consistency. See `docker/docker-compose.yml`.
- Map origin in `nav2_bringup/maps/warehouse.yaml` is set up for spawn
  `(-8.0, -0.50, 0)` — that's our default. All wrong-init scenarios express
  their `init_x/y/yaw` as deltas from this baseline.
