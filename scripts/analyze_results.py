#!/usr/bin/env python3
"""Aggregate the per-experiment CSVs from csv_logger.py into matplotlib
plots covering every metric the wrong-init study cares about.

Inputs (one of):
  <scenario>_seed01_timeseries.csv  (10-seed sweep, run via run_experiments.sh)
  <scenario>_timeseries.csv         (single deterministic run)
plus the matching *_summary.csv files.

Outputs (per scenario):
  <scenario>_error_xy.<fmt>      mean ± std xy error band, per filter
  <scenario>_error_yaw.<fmt>     mean ± std yaw error band, per filter
  <scenario>_nees.<fmt>          NEES vs χ² consistency band (3-DoF)
  <scenario>_pf_ess.<fmt>        PF ESS over time
  <scenario>_trajectory.<fmt>    truth path + each filter's estimate path (xy)

Outputs (aggregate across all scenarios):
  rmse_comparison.<fmt>          final RMSE_xy bar chart, mean ± std
  ttc_comparison.<fmt>           time-to-converge bar chart, mean ± std
  convergence_rate.<fmt>         % of seeds that converged, per filter
  runtime_comparison.<fmt>       mean per-tick runtime, log y-axis
  nees_mean_comparison.<fmt>     per-scenario mean NEES (consistency)
  all_summaries.csv              concat of every <scenario>_seed*_summary.csv

Output formats (--format):
  png   bitmap @ 200 dpi — README + PowerPoint
  pgf   LaTeX-aware, drop \\input{plot.pgf} into IEEE template
  pdf   vector PDF — plain \\includegraphics

Usage:
    python3 scripts/analyze_results.py --in ./results
    python3 scripts/analyze_results.py --in ./results --format pgf
    python3 scripts/analyze_results.py --in ./results --filters kf,ekf,pf,amcl

numpy + matplotlib only — no pandas, runs anywhere.
"""
from __future__ import annotations
import argparse
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

try:
    import numpy as np
    import matplotlib
except ImportError as e:
    print(f"Missing dependency: {e}. pip install numpy matplotlib")
    sys.exit(1)


# Default tracked filters. AMCL is the Nav2 baseline. EKF-LF is the
# advanced EKF variant with direct scan-likelihood. Order = legend order.
DEFAULT_FILTERS = ("kf", "ekf", "ekf_lf", "pf", "amcl")
COLOURS = {
    "kf":     "tab:blue",
    "ekf":    "tab:green",
    "ekf_lf": "tab:purple",
    "pf":     "tab:red",
    "amcl":   "tab:orange",
}
SEED_RE = re.compile(r"^(?P<scenario>.+?)(?:_seed(?P<seed>\d+))?_(?:timeseries|summary)\.csv$")

# Known landmarks (world/map coords), matching landmark_detector in the launch.
# Drawn as stars on the uncertainty-trajectory plots (lecture slide 12 style).


def _break_jumps(xs, ys, jump_threshold=0.5):
    """Insert NaN between consecutive samples whose step > jump_threshold.

    Used to stop matplotlib from drawing a straight line ACROSS the plot when
    the truth path teleports (kidnapped scenario) or a filter snaps to a new
    pose. At 20 Hz logging + ~0.5 m/s motion a real step is ≈ 0.025 m, so any
    gap > 0.5 m is a discontinuity, not a smooth motion segment.
    """
    import numpy as _np
    xs = _np.asarray(xs, dtype=float).copy()
    ys = _np.asarray(ys, dtype=float).copy()
    if xs.size < 2:
        return xs, ys
    dx = _np.diff(xs)
    dy = _np.diff(ys)
    big = _np.hypot(dx, dy) > jump_threshold
    if not big.any():
        return xs, ys
    # Mark the BREAK index (second point of each big-step pair) with NaN.
    idx = _np.where(big)[0] + 1
    xs[idx] = _np.nan
    ys[idx] = _np.nan
    return xs, ys


def _jump_indices(xs, ys, jump_threshold=0.5, min_gap_ticks=20):
    """Return the indices where step length > threshold (kidnap event onsets).

    For each i in the result: position[i-1] is the LAST point before the jump,
    position[i] is the FIRST point after — those are the markers we want to
    pin "kidnap" annotations to.

    Adjacent jumps within `min_gap_ticks` are collapsed into one event,
    because Gazebo's `set_pose` often manifests as 2-4 consecutive
    sub-jumps as the rigid body settles into the new pose — those are one
    logical kidnap, not several.
    """
    import numpy as _np
    xs = _np.asarray(xs, dtype=float)
    ys = _np.asarray(ys, dtype=float)
    if xs.size < 2:
        return _np.array([], dtype=int)
    big = _np.hypot(_np.diff(xs), _np.diff(ys)) > jump_threshold
    raw = (_np.where(big)[0] + 1).astype(int)
    if raw.size <= 1:
        return raw
    # Collapse: keep the first index of each cluster, where a cluster is a
    # run of indices each within min_gap_ticks of the prior one.
    keep = [raw[0]]
    for i in raw[1:]:
        if i - keep[-1] >= min_gap_ticks:
            keep.append(i)
    return _np.array(keep, dtype=int)


def _plot_path_gradient(ax, xs, ys, base_color, label, zorder, ts=None,
                        lw=1.8, ls="-"):
    """Draw a path with progressive alpha — early dim, late solid — so when
    the path is broken into multiple segments (kidnap teleports) the reader
    can see WHICH segment came first vs last. Uses matplotlib LineCollection
    so per-segment alpha is supported.
    """
    import numpy as _np
    from matplotlib.collections import LineCollection
    xs = _np.asarray(xs, dtype=float)
    ys = _np.asarray(ys, dtype=float)
    if xs.size < 2:
        return
    pts = _np.column_stack([xs, ys]).reshape(-1, 1, 2)
    segs = _np.concatenate([pts[:-1], pts[1:]], axis=1)
    # Skip segments that contain NaN (those are the inserted breaks).
    valid = ~_np.isnan(segs).any(axis=(1, 2))
    n = segs.shape[0]
    alphas = _np.linspace(0.25, 1.0, n)
    lc = LineCollection(segs[valid], colors=base_color, linewidths=lw,
                        linestyle=ls, zorder=zorder, alpha=None)
    # Per-segment alpha via the colors array (RGBA), since LineCollection's
    # `alpha` is global.
    from matplotlib.colors import to_rgba
    rgba = _np.tile(_np.array(to_rgba(base_color)), (valid.sum(), 1))
    rgba[:, 3] = alphas[valid]
    lc.set_color(rgba)
    ax.add_collection(lc)
    # One invisible handle for the legend.
    ax.plot([], [], color=base_color, lw=lw, ls=ls, label=label, zorder=zorder)
LANDMARKS = [
    (-7.45, -15.02), ( 7.49, -15.02),
    (-7.42,  -7.55), ( 7.46,  -7.52),
    (-7.51,  -0.02), ( 7.43,  -0.02),
    (-7.45,   7.54), ( 7.43,   7.48),
]


# ──────────────────────────────────────────────────────────────────────────
# Backend setup — must be called before importing pyplot
# ──────────────────────────────────────────────────────────────────────────
def configure_backend(fmt: str):
    if fmt == "pgf":
        matplotlib.use("pgf")
        matplotlib.rcParams.update({
            "pgf.texsystem":  "pdflatex",
            "font.family":    "serif",
            "text.usetex":    True,
            "pgf.rcfonts":    False,
            "axes.labelsize":  9,
            "font.size":       9,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
        })
    else:
        matplotlib.use("Agg")
        matplotlib.rcParams.update({
            "savefig.dpi":      200,
            "savefig.bbox":     "tight",
            "figure.facecolor": "white",
            "axes.facecolor":   "white",
            "font.family":      "sans-serif",
            "font.size":        11,
            "axes.titlesize":   13,
            "axes.titleweight": "bold",
            "axes.labelsize":   11,
            "axes.edgecolor":   "0.4",
            "axes.linewidth":   0.9,
            "axes.axisbelow":   True,      # grid behind data
            "axes.grid":        True,
            "grid.color":       "0.85",
            "grid.linewidth":   0.8,
            "xtick.labelsize":  10,
            "ytick.labelsize":  10,
            "lines.linewidth":  1.8,
            "legend.fontsize":  9,
            "legend.frameon":   True,
            "legend.framealpha": 0.92,
            "legend.edgecolor": "0.8",
        })


# ──────────────────────────────────────────────────────────────────────────
# CSV loading
# ──────────────────────────────────────────────────────────────────────────
def read_timeseries(path: Path) -> dict[str, np.ndarray]:
    cols: dict[str, list[float]] = {}
    with path.open() as f:
        reader = csv.DictReader(f)
        for k in reader.fieldnames or []:
            cols[k] = []
        for row in reader:
            for k, v in row.items():
                try:
                    cols[k].append(float(v))
                except (ValueError, TypeError):
                    cols[k].append(float("nan"))
    return {k: np.asarray(v) for k, v in cols.items()}


def discover_runs(in_dir: Path):
    """Walk in_dir, group *_timeseries.csv by scenario (across seeds)."""
    by_scenario: dict[str, list[Path]] = defaultdict(list)
    for p in sorted(in_dir.glob("*_timeseries.csv")):
        m = SEED_RE.match(p.name)
        if not m:
            continue
        by_scenario[m.group("scenario")].append(p)
    return by_scenario


def stack_runs(paths: list[Path], col: str) -> tuple[np.ndarray, np.ndarray]:
    """Load `col` from each CSV in `paths`, resample to a common time grid,
    and return (t, stacked) where stacked is shape (n_runs, n_steps).
    Resampling uses the union of time vectors and linear interpolation —
    keeps it dependency-free."""
    t_all = []
    series = []
    for p in paths:
        ts = read_timeseries(p)
        if "time" not in ts or col not in ts:
            continue
        t = ts["time"]
        v = ts[col]
        # drop trailing NaN block to avoid extending interpolation past data
        valid = np.isfinite(t) & np.isfinite(v)
        if not valid.any():
            continue
        t_all.append(t[valid])
        series.append((t[valid], v[valid]))
    if not series:
        return np.array([]), np.empty((0, 0))
    # Common grid: 100 ms steps from 0 to min duration
    t_max = min(t[-1] for t, _ in series)
    if t_max <= 0:
        return np.array([]), np.empty((0, 0))
    grid = np.arange(0.0, t_max, 0.1)
    stacked = np.empty((len(series), len(grid)))
    for i, (t, v) in enumerate(series):
        stacked[i, :] = np.interp(grid, t, v)
    return grid, stacked


# ──────────────────────────────────────────────────────────────────────────
# Per-scenario plots (mean + std band across seeds)
# ──────────────────────────────────────────────────────────────────────────
def plot_band(ax, t, stack, color, label):
    if stack.size == 0:
        return
    mean = np.nanmean(stack, axis=0)
    std  = np.nanstd(stack,  axis=0)
    ax.plot(t, mean, color=color, label=label, lw=1.5)
    ax.fill_between(t, mean - std, mean + std, color=color, alpha=0.2,
                    linewidth=0)


def plot_filter_explainer(scenario: str, paths: list[Path], filter_name: str,
                          out: Path, fmt: str, plt):
    """Lecture-style 3-panel figure (cf. Thrun §3.3) for a single filter on
    a single scenario:

      Panel 1: truth path + filter estimate (paths only) — "motion model"
      Panel 2: same + 2σ uncertainty ellipses from EARLY ticks where no
               landmark has been fused yet (n_landmarks_detected==0) —
               "without correction, uncertainty grows"
      Panel 3: same + 2σ ellipses from later ticks WITH landmark
               observations + landmark stars — "measurement updates
               shrink the uncertainty"

    Produces <scenario>_<filter>_explainer.<fmt>.
    """
    ts = read_timeseries(paths[0])
    if "truth_x" not in ts or f"{filter_name}_x" not in ts:
        return
    f = filter_name
    col = COLOURS.get(f, "tab:orange")

    tx = np.asarray(ts["truth_x"], dtype=float)
    ty = np.asarray(ts["truth_y"], dtype=float)
    fx = np.asarray(ts[f"{f}_x"], dtype=float)
    fy = np.asarray(ts[f"{f}_y"], dtype=float)
    n_lm = np.asarray(ts.get("n_landmarks_detected", [0]*len(tx)), dtype=float)

    # Pxx / Pyy / Pxy for proper ellipses (fall back to old cov column)
    cxx_k, cyy_k, cxy_k = f"{f}_cov_xx", f"{f}_cov_yy", f"{f}_cov_xy"
    if cxx_k not in ts:
        c_old = np.asarray(ts.get(f"{f}_cov", [0]*len(tx)), dtype=float)
        pxx = c_old; pyy = c_old; pxy = np.zeros_like(c_old)
    else:
        pxx = np.asarray(ts[cxx_k], dtype=float)
        pyy = np.asarray(ts[cyy_k], dtype=float)
        pxy = np.asarray(ts[cxy_k], dtype=float)

    fig, axs = plt.subplots(1, 3, figsize=(15, 5.2), sharey=True)
    titles = [
        f"{f.upper()}: trajectory (motion model)",
        f"{f.upper()}: predict-only uncertainty\n(ticks without landmark)",
        f"{f.upper()}: with landmark correction\n(ticks with ≥1 landmark)",
    ]

    def draw_paths(ax):
        ax.plot(tx, ty, color="k", lw=2, label="Ground Truth", zorder=8)
        ax.plot(fx, fy, color=col, ls="--", lw=1.6,
                label=f"{f.upper()} estimate", zorder=7)
        ax.scatter([tx[0]], [ty[0]], c="green", s=60, zorder=10, label="Start")
        ax.scatter([tx[-1]], [ty[-1]], c="orange", s=60, zorder=10, label="True End")
        ax.scatter([fx[-1]], [fy[-1]], c="red", s=60, zorder=10, label="Estimated End")

    def draw_ellipses(ax, mask, max_count=25):
        idxs = np.where(mask & np.isfinite(pxx) & np.isfinite(pyy) & np.isfinite(pxy)
                        & (pxx > 0) & (pyy > 0))[0]
        if idxs.size == 0:
            return 0
        step = max(1, idxs.size // max_count)
        drawn = 0
        for i in idxs[::step]:
            C = np.array([[pxx[i], pxy[i]], [pxy[i], pyy[i]]])
            vals, vecs = np.linalg.eigh(C)
            vals = np.clip(vals, 1e-9, None)
            order = vals.argsort()[::-1]
            vals, vecs = vals[order], vecs[:, order]
            width  = min(2 * 2 * np.sqrt(vals[0]), 5.0)
            height = min(2 * 2 * np.sqrt(vals[1]), 5.0)
            angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
            ax.add_patch(Ellipse((fx[i], fy[i]),
                                 width=width, height=height, angle=angle,
                                 fill=False, color=col, alpha=0.45, lw=0.9))
            drawn += 1
        return drawn

    # Panel 1: paths only
    draw_paths(axs[0])

    # Panel 2: ellipses on ticks WITHOUT a landmark observation in the recent
    # past. We mark each tick i as "predict-only" if no landmark has been
    # seen in the last `gap` ticks (≈ 0.5 s @ 20 Hz). That captures the
    # "uncertainty grows" phase between sightings.
    gap = 10
    rolling = np.zeros_like(n_lm, dtype=bool)
    seen = 0
    for i, v in enumerate(n_lm):
        if v > 0:
            seen = 0
            rolling[i] = False
        else:
            seen += 1
            rolling[i] = seen >= gap
    draw_paths(axs[1])
    draw_ellipses(axs[1], rolling)

    # Panel 3: ellipses ONLY where a landmark was just observed (corrections)
    landmark_mask = n_lm > 0
    draw_paths(axs[2])
    draw_ellipses(axs[2], landmark_mask)
    lx = [p[0] for p in LANDMARKS]; ly = [p[1] for p in LANDMARKS]
    axs[2].scatter(lx, ly, marker="*", s=180, color="tab:blue",
                   edgecolor="k", zorder=9, label="Known Landmark")

    for ax, title in zip(axs, titles):
        ax.set_xlabel("x [m]")
        ax.set_title(title, fontsize=10)
        ax.set_xlim(-2.0, 10.0)
        ax.set_ylim(-2.0, 8.0)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.3)
    axs[0].set_ylabel("y [m]")
    axs[2].legend(loc="lower right", fontsize=7)
    fig.suptitle(f"{scenario} — {f.upper()} localization: motion model → "
                 "uncertainty growth → landmark correction", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / f"{scenario}_{f}_explainer.{fmt}")
    plt.close(fig)


def plot_per_scenario(scenario: str, paths: list[Path], filters, out: Path,
                      fmt: str, plt):
    # error_xy
    fig, ax = plt.subplots(figsize=(8, 4))
    for f in filters:
        t, stk = stack_runs(paths, f"{f}_err_xy")
        plot_band(ax, t, stk, COLOURS.get(f, "k"), f.upper())
    ax.set_xlabel("time [s]"); ax.set_ylabel("xy error [m]")
    ax.set_title(f"{scenario} — xy error (mean ± 1σ over {len(paths)} seeds)")
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / f"{scenario}_error_xy.{fmt}"); plt.close(fig)

    # error_yaw
    fig, ax = plt.subplots(figsize=(8, 4))
    for f in filters:
        t, stk = stack_runs(paths, f"{f}_err_yaw")
        plot_band(ax, t, stk, COLOURS.get(f, "k"), f.upper())
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw error [rad]")
    ax.set_title(f"{scenario} — yaw error (mean ± 1σ over {len(paths)} seeds)")
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / f"{scenario}_error_yaw.{fmt}"); plt.close(fig)

    # NEES vs χ² band (3-DoF: lower 0.025 = 0.216, upper 0.975 = 9.348)
    fig, ax = plt.subplots(figsize=(8, 4))
    drew_any = False
    for f in filters:
        t, stk = stack_runs(paths, f"{f}_nees")
        if stk.size:
            plot_band(ax, t, stk, COLOURS.get(f, "k"), f.upper())
            drew_any = True
    if drew_any:
        ax.axhline(3.0,   ls="--", color="k", alpha=0.4, label="ideal (n=3)")
        ax.axhspan(0.216, 9.348, color="gray", alpha=0.1,
                   label="95% χ² band")
        ax.set_yscale("symlog", linthresh=1.0)
        ax.set_xlabel("time [s]"); ax.set_ylabel("NEES")
        ax.set_title(f"{scenario} — filter consistency (NEES, 3 DoF)")
        ax.legend(); ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / f"{scenario}_nees.{fmt}")
    plt.close(fig)

    # PF ESS
    t, stk = stack_runs(paths, "pf_ess")
    if stk.size:
        fig, ax = plt.subplots(figsize=(8, 3.5))
        plot_band(ax, t, stk, COLOURS["pf"], "PF ESS")
        ax.set_xlabel("time [s]"); ax.set_ylabel("Effective Sample Size")
        ax.set_title(f"{scenario} — PF Effective Sample Size")
        ax.grid(alpha=0.3); ax.legend()
        fig.tight_layout()
        fig.savefig(out / f"{scenario}_pf_ess.{fmt}")
        plt.close(fig)

    # Landmark detection rate — empirical evidence that the detector is
    # actually producing observations (not silently dead-reckoning).
    # Y = number of landmarks visible per logging tick; X = time.
    t, stk = stack_runs(paths, "n_landmarks_detected")
    if stk.size:
        fig, ax = plt.subplots(figsize=(8, 3.5))
        plot_band(ax, t, stk, "tab:orange", "landmarks detected")
        # Total count for the run, as a subtitle anchor.
        total = int(np.nansum(stk[0]) if stk.shape[0] == 1 else 0)
        max_seen = int(np.nanmax(stk)) if stk.size else 0
        ax.set_xlabel("time [s]")
        ax.set_ylabel("# landmarks in scan")
        ax.set_title(f"{scenario} — landmark detections per tick "
                     f"(max={max_seen}, sum over run={total})")
        ax.grid(alpha=0.3); ax.legend()
        fig.tight_layout()
        fig.savefig(out / f"{scenario}_landmark_detections.{fmt}")
        plt.close(fig)

    # Trajectory (xy paths) — overlay every seed thinly per filter; truth bold.
    fig, ax = plt.subplots(figsize=(7, 7))
    truth_drawn = False
    for p in paths:
        ts = read_timeseries(p)
        if "truth_x" in ts and not truth_drawn:
            tx, ty = _break_jumps(ts["truth_x"], ts["truth_y"])
            ax.plot(tx, ty, color="k", lw=2, label="truth", zorder=10)
            truth_drawn = True
        for f in filters:
            xk, yk = f"{f}_x", f"{f}_y"
            if xk in ts and yk in ts:
                fx, fy = _break_jumps(ts[xk], ts[yk])
                ax.plot(fx, fy, color=COLOURS.get(f, "gray"),
                        alpha=0.4, lw=0.8)
    # one full-alpha line per filter for the legend
    for f in filters:
        ax.plot([], [], color=COLOURS.get(f, "gray"), label=f.upper())
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"{scenario} — trajectories ({len(paths)} seed runs)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best"); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / f"{scenario}_trajectory.{fmt}")
    plt.close(fig)

    # State vs time — truth (black) + each filter estimate, per axis (x, y, yaw).
    # Lecture-style "true vs estimated" plot (slides 6/7). Uses the first run.
    ts = read_timeseries(paths[0])
    if "time" in ts and "truth_x" in ts:
        fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        for ax, (axis, unit) in zip(axs, [("x", "m"), ("y", "m"), ("yaw", "rad")]):
            ax.plot(ts["time"], ts[f"truth_{axis}"], color="k", lw=2,
                    label="truth", zorder=10)
            for f in filters:
                col = f"{f}_{axis}"
                if col in ts:
                    ax.plot(ts["time"], ts[col], color=COLOURS.get(f, "gray"),
                            lw=1.2, alpha=0.85, label=f.upper())
            ax.set_ylabel(f"{axis} [{unit}]"); ax.grid(alpha=0.3)
        axs[0].set_title(f"{scenario} — state estimate vs truth over time")
        axs[0].legend(ncol=len(filters) + 1, fontsize=8, loc="upper right")
        axs[-1].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(out / f"{scenario}_state_vs_time.{fmt}")
        plt.close(fig)

    # Lecture-style 3-panel explainer for KF/EKF/PF on this scenario (the
    # three "required" filters per the task spec). Each one tells the
    # standard Thrun §3.3 story: motion model → predict-only uncertainty
    # growth → measurement update shrinks it. Produces
    # <scenario>_<filter>_explainer.<fmt>.
    for f in ("kf", "ekf", "pf"):
        if f in filters:
            plot_filter_explainer(scenario, paths, f, out, fmt, plt)

    # Per-filter localization plot (lecture slide 12 style): truth path + the
    # filter estimate + growing uncertainty circles (radius = sqrt(cov)) +
    # known landmarks (stars) + start / true-end / estimated-end markers.
    ts = read_timeseries(paths[0])
    if "time" in ts and "truth_x" in ts:
        for f in filters:
            xk, yk, ck = f"{f}_x", f"{f}_y", f"{f}_cov"
            if xk not in ts or yk not in ts:
                continue
            fig, ax = plt.subplots(figsize=(7.5, 7))
            col = COLOURS.get(f, "gray")
            tx, ty = _break_jumps(ts["truth_x"], ts["truth_y"])
            fx, fy = _break_jumps(ts[xk], ts[yk])
            # Detect kidnap events from truth jumps. If any exist, switch to
            # the gradient-segment + kidnap-marker rendering so the reader
            # can tell which segment came first, where each teleport
            # happened, and where the filter started losing track.
            kidnap_idx = _jump_indices(ts["truth_x"], ts["truth_y"])
            has_kidnaps = kidnap_idx.size > 0
            if has_kidnaps:
                _plot_path_gradient(ax, tx, ty, "k", "Ground Truth",
                                    zorder=8, lw=2.0, ls="-")
                _plot_path_gradient(ax, fx, fy, col, f"{f.upper()} estimate",
                                    zorder=7, lw=1.6, ls="--")
                tx_full = np.asarray(ts["truth_x"], dtype=float)
                ty_full = np.asarray(ts["truth_y"], dtype=float)
                for k_no, i in enumerate(kidnap_idx, start=1):
                    # red X = last point before + first point after the jump
                    ax.scatter([tx_full[i - 1], tx_full[i]],
                               [ty_full[i - 1], ty_full[i]],
                               marker="x", s=80, c="red", linewidth=2,
                               zorder=11,
                               label=("kidnap event" if k_no == 1 else None))
                    ax.annotate(f"K{k_no}", (tx_full[i], ty_full[i]),
                                xytext=(6, 6), textcoords="offset points",
                                color="red", fontsize=9, fontweight="bold",
                                zorder=12)
            else:
                ax.plot(tx, ty, color="k", lw=2,
                        label="Ground Truth", zorder=8)
                ax.plot(fx, fy, color=col, ls="--", lw=1.6,
                        label=f"{f.upper()} estimate", zorder=7)
            # 2σ uncertainty ellipses from the full 2D covariance (Pxx, Pyy, Pxy),
            # subsampled along the path. 2σ ≈ 95% confidence — standard paper viz.
            # Old CSVs only logged a single Pxx column; fall back to a circle then.
            cxx_k, cyy_k, cxy_k = f"{f}_cov_xx", f"{f}_cov_yy", f"{f}_cov_xy"
            if cxx_k not in ts and ck in ts:
                ts[cxx_k] = ts[ck]
                ts[cyy_k] = ts[ck]
                ts[cxy_k] = [0.0] * len(ts[ck])
            if cxx_k in ts and cyy_k in ts and cxy_k in ts:
                n = len(ts[xk])
                step = max(1, n // 25)
                for i in range(0, n, step):
                    pxx, pyy, pxy = ts[cxx_k][i], ts[cyy_k][i], ts[cxy_k][i]
                    if not (np.isfinite(pxx) and np.isfinite(pyy) and np.isfinite(pxy)):
                        continue
                    if pxx <= 0 or pyy <= 0:
                        continue
                    C = np.array([[pxx, pxy], [pxy, pyy]])
                    vals, vecs = np.linalg.eigh(C)
                    vals = np.clip(vals, 1e-9, None)
                    order = vals.argsort()[::-1]
                    vals, vecs = vals[order], vecs[:, order]
                    # 2σ ellipse: full axis length = 2 * 2 * sqrt(λ).
                    width  = min(2 * 2 * np.sqrt(vals[0]), 5.0)
                    height = min(2 * 2 * np.sqrt(vals[1]), 5.0)
                    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
                    ax.add_patch(Ellipse((ts[xk][i], ts[yk][i]),
                                         width=width, height=height, angle=angle,
                                         fill=False, color=col, alpha=0.45, lw=0.9))
            # Known landmarks
            lx = [p[0] for p in LANDMARKS]; ly = [p[1] for p in LANDMARKS]
            ax.scatter(lx, ly, marker="*", s=180, color="tab:blue",
                       edgecolor="k", zorder=9, label="Known Landmark")
            # Start / end markers
            ax.scatter([ts["truth_x"][0]], [ts["truth_y"][0]], c="green", s=60,
                       zorder=10, label="Start")
            ax.scatter([ts["truth_x"][-1]], [ts["truth_y"][-1]], c="orange", s=60,
                       zorder=10, label="True End")
            ax.scatter([ts[xk][-1]], [ts[yk][-1]], c="red", s=60,
                       zorder=10, label="Estimated End")
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
            title = f"{scenario} — {f.upper()}: trajectory + uncertainty"
            if has_kidnaps:
                title += f"  ({kidnap_idx.size} kidnap events)"
            ax.set_title(title)
            if has_kidnaps:
                # Auto-fit to all data (truth + filter + landmarks), with a
                # small margin. The fixed window misses post-teleport segments.
                all_x = np.concatenate([
                    np.asarray(ts["truth_x"], dtype=float),
                    np.asarray(ts[xk], dtype=float),
                    np.array([p[0] for p in LANDMARKS], dtype=float),
                ])
                all_y = np.concatenate([
                    np.asarray(ts["truth_y"], dtype=float),
                    np.asarray(ts[yk], dtype=float),
                    np.array([p[1] for p in LANDMARKS], dtype=float),
                ])
                all_x = all_x[np.isfinite(all_x)]
                all_y = all_y[np.isfinite(all_y)]
                pad = 1.0
                ax.set_xlim(all_x.min() - pad, all_x.max() + pad)
                ax.set_ylim(all_y.min() - pad, all_y.max() + pad)
            else:
                ax.set_xlim(-2.0, 10.0)
                ax.set_ylim(-2.0, 8.0)
            ax.set_aspect("equal", adjustable="box")
            ax.legend(loc="best", fontsize=8); ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(out / f"{scenario}_{f}_localization.{fmt}")
            plt.close(fig)


# ──────────────────────────────────────────────────────────────────────────
# Aggregate plots across all scenarios
# ──────────────────────────────────────────────────────────────────────────
def load_all_summaries(in_dir: Path) -> tuple[list[dict], list[str]]:
    rows = []
    fields: list[str] = []
    for p in sorted(in_dir.glob("*_summary.csv")):
        with p.open() as f:
            reader = csv.DictReader(f)
            if not fields:
                fields = list(reader.fieldnames or [])
            for r in reader:
                rows.append(r)
    return rows, fields


def grouped_bar(scenarios, values_per_filter, errors_per_filter, filters,
                ylabel, title, out_path, plt, log=False, label_suffix=None):
    x = np.arange(len(scenarios))
    n = len(filters)
    w = 0.8 / max(n, 1)
    fig, ax = plt.subplots(figsize=(max(6, 0.8 * len(scenarios) * n / 3), 4.5))
    for i, f in enumerate(filters):
        means = values_per_filter.get(f, [np.nan] * len(scenarios))
        errs  = errors_per_filter.get(f,  [0.0] * len(scenarios))
        label = f.upper()
        if label_suffix and f in label_suffix:
            label = f"{label}  {label_suffix[f]}"
        ax.bar(x + (i - (n - 1) / 2) * w, means, w,
               yerr=errs, capsize=2,
               color=COLOURS.get(f, "gray"), label=label)
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=30, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if log:
        ax.set_yscale("log")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")


def aggregate_plots(in_dir: Path, out_dir: Path, fmt: str, filters, plt):
    rows, fields = load_all_summaries(in_dir)
    if not rows:
        print("no *_summary.csv found")
        return

    agg = out_dir / "all_summaries.csv"
    with agg.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)
    print(f"wrote {agg}")

    by_scenario: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        by_scenario[r["scenario"]].append(r)
    scenarios = sorted(by_scenario.keys())

    def pick(field: str, cast=float):
        means: dict[str, list[float]] = defaultdict(list)
        stds:  dict[str, list[float]] = defaultdict(list)
        for s in scenarios:
            for f in filters:
                col = f"{f}_{field}"
                if col not in fields:
                    means[f].append(float("nan"))
                    stds[f].append(0.0)
                    continue
                vals = []
                for r in by_scenario[s]:
                    try:
                        vals.append(cast(r[col]))
                    except (ValueError, TypeError):
                        pass
                if vals:
                    means[f].append(float(np.mean(vals)))
                    stds[f].append(float(np.std(vals)))
                else:
                    means[f].append(float("nan"))
                    stds[f].append(0.0)
        return means, stds

    rmse_m, rmse_s = pick("final_rmse_xy")
    grouped_bar(scenarios, rmse_m, rmse_s, filters,
                "final RMSE_xy [m]",
                f"Final RMSE_xy per scenario (mean ± 1σ over seeds)",
                out_dir / f"rmse_comparison.{fmt}", plt)

    ttc_m, ttc_s = pick("time_to_converge")
    grouped_bar(scenarios, ttc_m, ttc_s, filters,
                "time-to-converge [s]",
                "Time-to-converge per scenario (mean ± 1σ; NaN = never)",
                out_dir / f"ttc_comparison.{fmt}", plt)

    rt_m,  rt_s  = pick("runtime_mean_us")
    # Theoretical per-tick complexity, shown alongside the empirical bars so
    # the paper plot covers both Big-O scaling and measured wall-clock cost.
    # n = state dim, L = visible landmarks, M = particles, R = scan rays.
    big_o = {
        "kf":     r"$\mathcal{O}(n^3)$",
        "ekf":    r"$\mathcal{O}(n^3 + L\,n^2)$",
        "ekf_lf": r"$\mathcal{O}(n^3 + R)$",
        "pf":     r"$\mathcal{O}(M\cdot R)$",
        "amcl":   r"$\mathcal{O}(M\cdot R)$",
    }
    grouped_bar(scenarios, rt_m, rt_s, filters,
                "wall-clock per tick [µs]  (log)",
                "Per-tick runtime — measured wall-clock + Big-O complexity",
                out_dir / f"runtime_comparison.{fmt}", plt, log=True,
                label_suffix=big_o)

    # Convergence rate: fraction of seeds with converged=True per (scenario, filter)
    rate: dict[str, list[float]] = defaultdict(list)
    for s in scenarios:
        for f in filters:
            col = f"{f}_converged"
            if col not in fields:
                rate[f].append(float("nan"))
                continue
            n_total = 0; n_conv = 0
            for r in by_scenario[s]:
                v = r.get(col, "")
                if v == "":
                    continue
                n_total += 1
                if v.lower() in ("true", "1"):
                    n_conv += 1
            rate[f].append(100.0 * n_conv / n_total if n_total else float("nan"))
    grouped_bar(scenarios, rate, defaultdict(lambda: [0.0] * len(scenarios)),
                filters,
                "converged seeds [%]",
                "Convergence rate (% of seeds reaching error_xy < 0.20 m)",
                out_dir / f"convergence_rate.{fmt}", plt)


# ──────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in",  dest="in_dir",  default="./results")
    ap.add_argument("--out", dest="out_dir", default=None)
    ap.add_argument("--format", choices=["png", "pgf", "pdf"], default="png",
                    help="png for slides/README, pgf for LaTeX paper")
    ap.add_argument("--filters", default=",".join(DEFAULT_FILTERS),
                    help="comma-separated list of filters to plot "
                         "(default: kf,ekf,pf,amcl)")
    args = ap.parse_args()

    configure_backend(args.format)
    import matplotlib.pyplot as plt  # noqa: E402
    from matplotlib.patches import Ellipse  # noqa: E402
    globals()["Ellipse"] = Ellipse

    in_dir  = Path(args.in_dir).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else in_dir
    if not in_dir.is_dir():
        print(f"input dir not found: {in_dir}")
        sys.exit(1)
    out_dir.mkdir(parents=True, exist_ok=True)

    filters = [f.strip() for f in args.filters.split(",") if f.strip()]

    runs = discover_runs(in_dir)
    if not runs:
        print(f"no *_timeseries.csv in {in_dir}")
    for scenario, paths in sorted(runs.items()):
        plot_per_scenario(scenario, paths, filters, out_dir, args.format, plt)
        print(f"plotted {scenario} ({len(paths)} runs)")

    aggregate_plots(in_dir, out_dir, args.format, filters, plt)


if __name__ == "__main__":
    main()
