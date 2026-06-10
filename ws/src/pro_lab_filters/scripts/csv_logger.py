#!/usr/bin/env python3
"""Log metrics + filter outputs + ground truth to two CSV files for offline
analysis (RMSE, plots, table for the paper).

Outputs:
  <out_dir>/<scenario>_timeseries.csv
      time, truth_x, truth_y, truth_yaw,
      kf_x, kf_y, kf_yaw, kf_err_xy, kf_err_yaw, kf_rmse_xy, kf_rmse_yaw,
      ekf_x, ekf_y, ekf_yaw, ekf_err_xy, ekf_err_yaw, ekf_rmse_xy, ekf_rmse_yaw,
      pf_x,  pf_y,  pf_yaw,  pf_err_xy,  pf_err_yaw,  pf_rmse_xy,  pf_rmse_yaw,
      pf_ess
  <out_dir>/<scenario>_summary.csv  (one row, written on shutdown)
      scenario, duration_s,
      kf_final_rmse_xy, kf_final_rmse_yaw, kf_converged, kf_time_to_converge,
      ekf_..., pf_...

Run as:
    ros2 run pro_lab_filters csv_logger.py \
        --ros-args -p scenario:=offset_1m -p out_dir:=/tmp/results
"""
from __future__ import annotations
import csv
import math
import os
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float32MultiArray, Float64


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CsvLogger(Node):
    DEFAULT_FILTERS = ('kf', 'ekf', 'pf')

    def __init__(self):
        super().__init__('csv_logger')
        self.declare_parameter('scenario', 'unnamed')
        self.declare_parameter('out_dir', '/tmp/pro_lab_results')
        self.declare_parameter('flush_every', 10)
        self.declare_parameter('seed', 0)
        # Filters to log. AMCL is added when the launch enables use_amcl —
        # /amcl_pose is relayed onto /amcl/pose so the same topic convention
        # holds for every tracked estimator.
        self.declare_parameter('filters', list(self.DEFAULT_FILTERS))

        self.scenario  = self.get_parameter('scenario').value
        self.out_dir   = Path(self.get_parameter('out_dir').value)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.flush_every = int(self.get_parameter('flush_every').value)
        self.seed = int(self.get_parameter('seed').value)
        self.FILTERS = tuple(self.get_parameter('filters').value)

        self._lock = threading.Lock()
        self._t0: Optional[float] = None
        self._latest: Dict[str, float] = {}
        # Per-filter cumulative state for the summary
        self._summary = {f: {'rmse_xy': float('nan'),
                             'rmse_yaw': float('nan'),
                             'converged': False,
                             'ttc': float('nan'),
                             'runtime_mean_us': 0.0,
                             'runtime_max_us':  0.0,
                             'runtime_n':       0} for f in self.FILTERS}

        # Suffix the seed when one was passed so 10-run sweeps don't overwrite.
        seed_suffix = f'_seed{self.seed:02d}' if self.seed > 0 else ''
        self._seed_suffix = seed_suffix
        ts_path = self.out_dir / f'{self.scenario}{seed_suffix}_timeseries.csv'
        self.get_logger().info(f'CSV logger writing to {ts_path}')
        self._ts_file = open(ts_path, 'w', newline='')
        cols = ['time', 'truth_x', 'truth_y', 'truth_yaw']
        for f in self.FILTERS:
            cols += [f'{f}_x', f'{f}_y', f'{f}_yaw',
                     f'{f}_err_xy', f'{f}_err_yaw',
                     f'{f}_rmse_xy', f'{f}_rmse_yaw',
                     f'{f}_nees',
                     f'{f}_cov_xx', f'{f}_cov_yy', f'{f}_cov_xy',
                     f'{f}_runtime_us']
        cols += ['pf_ess', 'n_landmarks_detected']
        self._ts_writer = csv.writer(self._ts_file)
        self._ts_writer.writerow(cols)
        self._row_count = 0

        # Subscriptions — best-effort transient (matches default publishers)
        qos = QoSProfile(depth=20,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseStamped, '/ground_truth/pose',
                                 self._on_truth, qos)
        for f in self.FILTERS:
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{f}/pose',
                lambda m, k=f: self._on_pose(k, m), qos)
            self.create_subscription(
                Float64, f'/metrics/{f}/error_xy',
                lambda m, k=f: self._set(f'{k}_err_xy', m.data), qos)
            self.create_subscription(
                Float64, f'/metrics/{f}/error_yaw',
                lambda m, k=f: self._set(f'{k}_err_yaw', m.data), qos)
            # rmse_xy / rmse_yaw: latch the latest value into BOTH
            #   * `_latest` so the timeseries column tracks the running RMSE
            #     tick-by-tick (debuggable, plotable directly), and
            #   * `_summary` so the final value lands in <scenario>_summary.csv
            # Single source of truth: whatever metrics_node publishes.
            def _set_both(filt: str, field: str, value: float):
                self._set(f'{filt}_{field}', value)
                self._set_summary(filt, field, value)
            self.create_subscription(
                Float64, f'/metrics/{f}/rmse_xy',
                lambda m, k=f: _set_both(k, 'rmse_xy', m.data), qos)
            self.create_subscription(
                Float64, f'/metrics/{f}/rmse_yaw',
                lambda m, k=f: _set_both(k, 'rmse_yaw', m.data), qos)
            self.create_subscription(
                Bool, f'/metrics/{f}/converged',
                lambda m, k=f: self._set_summary(k, 'converged', m.data), qos)
            self.create_subscription(
                Float64, f'/metrics/{f}/time_to_converge',
                lambda m, k=f: self._set_summary(k, 'ttc', m.data), qos)
            self.create_subscription(
                Float64, f'/metrics/{f}/nees',
                lambda m, k=f: self._set(f'{k}_nees', m.data), qos)
            # Per-filter runtime in microseconds, published by each filter
            # node from its tick(). Track the running mean + max for the
            # "Runtime / Performance" comparison in the paper.
            self.create_subscription(
                Float64, f'/{f}/runtime_us',
                lambda m, k=f: self._on_runtime(k, m.data), qos)
        self.create_subscription(Float64, '/pf/ess',
                                 lambda m: self._set('pf_ess', m.data), qos)
        # /landmarks/observations is event-based (only published when the
        # detector sees ≥1 landmark in this scan), and uses stride 3:
        # [id, range, bearing, id, range, bearing, ...]. So the count is
        # len(data) // 3. We latch the latest count so the periodic row
        # writer sees it; we DON'T zero between events because that would
        # under-count when logging hz > detection hz.
        self.create_subscription(
            Float32MultiArray, '/landmarks/observations',
            lambda m: self._set('n_landmarks_detected', len(m.data) // 3), qos)

        # Periodically dump a row (triggered by metrics rate, not just on truth)
        self._timer = self.create_timer(0.05, self._maybe_write_row)

    # ── topic handlers ────────────────────────────────────────────────────
    def _on_truth(self, m: PoseStamped):
        with self._lock:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self._t0 is None:
                self._t0 = t
            self._latest['time']      = t - self._t0
            self._latest['truth_x']   = m.pose.position.x
            self._latest['truth_y']   = m.pose.position.y
            self._latest['truth_yaw'] = quat_to_yaw(m.pose.orientation)

    def _on_pose(self, key: str, m: PoseWithCovarianceStamped):
        with self._lock:
            self._latest[f'{key}_x']   = m.pose.pose.position.x
            self._latest[f'{key}_y']   = m.pose.pose.position.y
            self._latest[f'{key}_yaw'] = quat_to_yaw(m.pose.pose.orientation)
            # Full 2D position covariance (row-major 6x6): [0]=Pxx, [7]=Pyy, [1]=Pxy.
            # The plotter rebuilds the 2x2 to draw a true uncertainty ellipse.
            self._latest[f'{key}_cov_xx'] = m.pose.covariance[0]
            self._latest[f'{key}_cov_yy'] = m.pose.covariance[7]
            self._latest[f'{key}_cov_xy'] = m.pose.covariance[1]

    def _set(self, key: str, value: float):
        with self._lock:
            self._latest[key] = value

    def _set_summary(self, filt: str, field: str, value):
        with self._lock:
            self._summary[filt][field] = value

    def _on_runtime(self, filt: str, value: float):
        with self._lock:
            self._latest[f'{filt}_runtime_us'] = value
            s = self._summary[filt]
            n = s['runtime_n'] + 1
            # Welford-style running mean (numerically stable, no overflow).
            s['runtime_mean_us'] += (value - s['runtime_mean_us']) / n
            s['runtime_max_us']   = max(s['runtime_max_us'], value)
            s['runtime_n']        = n

    # ── periodic row writer ───────────────────────────────────────────────
    def _maybe_write_row(self):
        with self._lock:
            if 'time' not in self._latest:
                return
            row = [self._latest.get('time', float('nan')),
                   self._latest.get('truth_x',   float('nan')),
                   self._latest.get('truth_y',   float('nan')),
                   self._latest.get('truth_yaw', float('nan'))]
            for f in self.FILTERS:
                for k in ('x', 'y', 'yaw',
                          'err_xy', 'err_yaw',
                          'rmse_xy', 'rmse_yaw',
                          'nees',
                          'cov_xx', 'cov_yy', 'cov_xy',
                          'runtime_us'):
                    row.append(self._latest.get(f'{f}_{k}', float('nan')))
            row.append(self._latest.get('pf_ess', float('nan')))
            row.append(self._latest.get('n_landmarks_detected', 0))
            self._ts_writer.writerow(row)
            self._row_count += 1
            if self._row_count % self.flush_every == 0:
                self._ts_file.flush()

    # ── shutdown: write summary CSV ───────────────────────────────────────
    def write_summary(self):
        sum_path = self.out_dir / f'{self.scenario}{self._seed_suffix}_summary.csv'
        with open(sum_path, 'w', newline='') as f:
            w = csv.writer(f)
            cols = ['scenario', 'seed', 'duration_s']
            for fn in self.FILTERS:
                cols += [f'{fn}_final_rmse_xy', f'{fn}_final_rmse_yaw',
                         f'{fn}_converged', f'{fn}_time_to_converge',
                         f'{fn}_runtime_mean_us', f'{fn}_runtime_max_us']
            w.writerow(cols)
            duration = self._latest.get('time', float('nan'))
            row = [self.scenario, self.seed, duration]
            for fn in self.FILTERS:
                s = self._summary[fn]
                row += [s['rmse_xy'], s['rmse_yaw'],
                        bool(s['converged']), s['ttc'],
                        s['runtime_mean_us'], s['runtime_max_us']]
            w.writerow(row)
        self.get_logger().info(f'wrote summary {sum_path}')

    def destroy_node(self):
        try:
            self.write_summary()
            self._ts_file.flush()
            self._ts_file.close()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'shutdown error: {e}')
        super().destroy_node()


def main():
    rclpy.init()
    node = CsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
