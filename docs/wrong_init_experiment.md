# Wrong Initialization — Experiment Guide

Task assigned to student group **2B1: 2510331021** / **2B2: 2510331007**:

> *Initialize the filter with an incorrect starting pose and/or uncertainty.*

This document describes the experimental setup, the scenarios, the metrics
captured, and how to reproduce the results.

---

## What we measure

Each scenario starts the simulation with the robot at a known truth pose
**(x=2.12, y=−21.3, yaw=1.57)** in the warehouse world. All three filters
(KF, EKF, PF) are initialized **at the same pose** — the *believed* initial
pose, which may or may not match the truth.

We compare:

| Metric | Description |
|---|---|
| `error_xy(t)` | Euclidean distance between estimate and truth, per filter |
| `error_yaw(t)` | wrapped yaw difference |
| `rmse_xy`, `rmse_yaw` | running RMSE up to time *t* |
| `converged` | true once `error_xy < 0.20 m` for ≥ 2 s continuously |
| `time_to_converge` | seconds from experiment start to first convergence |
| `pf_ess` | PF effective sample size (1 / Σ wᵢ²) — collapses with degeneracy |

Truth is read from TF (`odom → base_footprint`) by `truth_relay_node` and
republished as `/ground_truth/pose` (PoseStamped). The metrics node compares
each filter pose to this signal.

---

## Scenarios (config/scenarios/*.yaml)

| File | Init pose error | Init spread | Notes |
|---|---|---|---|
| `correct_init.yaml`        | 0 | tight (10 cm) | baseline — should converge instantly |
| `offset_1m.yaml`           | +1 m in X | 0.5 m | mild bias, spread covers truth |
| `offset_5m.yaml`           | +5 m in X | 1.0 m, N=1000 | bigger bias, PF needs more particles |
| `wrong_yaw_pi2.yaml`       | +π/2 yaw | 0.2 m / 0.3 rad | rotational error, hard for linearised filters |
| `overconfident_wrong.yaml` | +3 m, +2 m | tight 10 cm | **PF particle deprivation** — headline finding |
| `underconfident.yaml`      | 0 | huge (3 m, 1.5 rad) | slow convergence for KF/EKF |
| `kidnapped.yaml`           | uniform across map | N=3000 | global localization, KF/EKF expected to fail |

Edit any YAML to sweep parameters; they are loaded as ROS 2 parameters by
all three filter nodes.

---

## Running a single scenario

Inside the `prolab_jazzy` container (or any sourced ROS 2 Jazzy environment):

```bash
ros2 launch pro_lab_filters wrong_init_experiment.launch.py \
    scenario:=overconfident_wrong \
    duration_s:=60 \
    out_dir:=/tmp/pro_lab_results
```

Launch args:

| Arg | Default | Purpose |
|---|---|---|
| `scenario`     | `correct_init` | YAML file basename (without `.yaml`) |
| `duration_s`   | `0` (forever) | auto-shutdown after N seconds |
| `out_dir`      | `/tmp/pro_lab_results` | CSV destination |
| `world`        | `warehouse` | gz world (warehouse / depot / maze) |
| `x_pose`, `y_pose`, `yaw` | warehouse defaults | spawn pose (truth) |
| `use_rviz`     | `true` | start RViz |
| `use_foxglove` | `true` | start Foxglove bridge on :8765 |
| `use_nav2`     | `true` | bring up Nav2 (provides `/pose` measurement) |

Outputs in `out_dir`:

- `<scenario>_timeseries.csv` — per-step truth, estimates, errors, RMSE, ESS
- `<scenario>_summary.csv` — final RMSE, convergence flag, time-to-converge

---

## Running all scenarios in one go

```bash
DURATION=60 RESULTS_DIR=/tmp/pro_lab_results \
USE_RVIZ=false USE_FOXGLOVE=false \
bash $(ros2 pkg prefix pro_lab_filters)/share/pro_lab_filters/../../lib/pro_lab_filters/run_wrong_init_batch.sh
```

(or just `bash ws/src/pro_lab_filters/scripts/run_wrong_init_batch.sh`)

The batch script prints a CSV summary of all scenarios at the end.

---

## Visualization

### RViz

Launched automatically (unless `use_rviz:=false`). The bundled config
[`config/wrong_init.rviz`](../ws/src/pro_lab_filters/config/wrong_init.rviz)
shows:

- Green axes → ground truth
- Blue arrow → KF estimate
- Greenish arrow → EKF estimate
- Red arrow → PF estimate (with covariance ellipse)
- Red flat arrows → PF particle cloud
- Map (Nav2) + RobotModel + LaserScan

### Foxglove Studio

Launched automatically (unless `use_foxglove:=false`). The `foxglove_bridge`
exposes `ws://<container-host>:8765`.

1. Open Foxglove Studio (web at <https://app.foxglove.dev> or desktop).
2. Add connection → Foxglove WebSocket → `ws://localhost:8765`.
3. Layout → Import from file → pick
   [`config/foxglove_layout.json`](../ws/src/pro_lab_filters/config/foxglove_layout.json).

The pre-built layout includes:

- 3D scene with truth (green) + KF/EKF/PF poses (with covariance) + particle cloud
- Time-series plot of `error_xy` per filter (instant)
- Time-series plot of running `rmse_xy` per filter
- Time-series plot of `error_yaw` per filter
- Time-series plot of PF Effective Sample Size
- Three convergence-state indicators (KF / EKF / PF)
- Gauge for PF time-to-converge

Both UIs run in parallel — pick whichever you prefer for any moment.

---

## Expected findings (hypotheses to verify in the report)

1. **`correct_init`** — all three filters track truth from t=0; RMSE ≈ sensor
   noise floor.
2. **`offset_1m`** — KF/EKF: smooth exponential pull-in (~1–2 s). PF: similar
   if the spread covers truth.
3. **`offset_5m`** — convergence times grow; PF needs N≥1000 particles to
   keep the truth in the support of the prior, otherwise tail-events
   dominate weighting.
4. **`wrong_yaw_pi2`** — KF (linear constant-velocity assumption) drifts
   noticeably; EKF and PF, both nonlinear-aware, recover faster.
5. **`overconfident_wrong`** — *headline result for this study*:
   - KF/EKF still converge eventually because the Kalman gain pulls the
     estimate toward each `/pose` update (asymptotically consistent).
   - PF cannot recover: all particles are far from truth, every weight is
     ≈ 0, resampling perpetuates the wrong cloud → divergent or stuck. PF
     ESS drops to 1.
6. **`underconfident`** — convergence is slow but reliable for all filters.
   PF benefits from the wide prior (truth is in support).
7. **`kidnapped`** — only PF (uniform init, N=3000) can globally localise;
   KF/EKF stay biased toward the (wrong) Gaussian prior centre.

These hypotheses become the experimental analysis section of the paper.

---

## Files added/changed for this experiment

| Path | Purpose |
|---|---|
| `ws/src/pro_lab_filters/include/pro_lab_filters/PF.h` | added Uniform init + ESS |
| `ws/src/pro_lab_filters/src/pf_node.cpp` | publish `/pf/particles`, `/pf/ess`, init_distribution param |
| `ws/src/pro_lab_filters/src/truth_relay_node.cpp` | TF → `/ground_truth/pose` |
| `ws/src/pro_lab_filters/src/metrics_node.cpp` | per-filter error / RMSE / convergence |
| `ws/src/pro_lab_filters/scripts/csv_logger.py` | CSV time-series + summary |
| `ws/src/pro_lab_filters/scripts/run_wrong_init_batch.sh` | batch driver |
| `ws/src/pro_lab_filters/config/scenarios/*.yaml` | 7 scenarios |
| `ws/src/pro_lab_filters/config/wrong_init.rviz` | RViz layout |
| `ws/src/pro_lab_filters/config/foxglove_layout.json` | Foxglove Studio layout |
| `ws/src/pro_lab_filters/launch/wrong_init_experiment.launch.py` | experiment launcher |
| `ws/src/pro_lab_filters/launch/all_in_one.launch.py` | also starts foxglove_bridge |
| `docker/Dockerfile` | adds `ros-jazzy-foxglove-bridge` |
| `ws/src/pro_lab_filters/CMakeLists.txt` | builds new executables |
| `ws/src/pro_lab_filters/package.xml` | adds tf2_ros, rclpy, foxglove_bridge deps |
