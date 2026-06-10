#!/usr/bin/env python3
"""Gaussian-fusion demo plots for the TurtleBot 4 sensor suite.

Two figures illustrating the core Kalman-update idea — a wide motion/odometry
prior sharpened by measurements — using the sources this system actually
fuses (IMU, wheel encoders/odometry, range/bearing landmarks, 2D lidar). No GPS.

  fusion_1d.<fmt>   Heading (yaw) fusion: wheel-odometry yaw (drifts, wide)
                    fused with the IMU yaw (tighter) → sharper posterior.
  fusion_2d.<fmt>   Position (x, y) fusion: wheel-odometry dead-reckoning
                    (elongated, drifting ellipse) corrected by the landmark
                    range/bearing fix and the lidar scan-match → smaller
                    posterior ellipse.

Each plot fuses a single physical quantity from sources that genuinely
measure it (yaw with yaw, position with position).

Fusion of independent Gaussians (information form):
    1/σ²_f = Σ 1/σ²_i        μ_f = σ²_f · Σ (μ_i / σ²_i)
2D analogue with covariance matrices:
    Σ_f = (Σ Σ_i⁻¹)⁻¹        μ_f = Σ_f · Σ (Σ_i⁻¹ μ_i)

    python3 scripts/plot_fusion_demo.py --out ./results --format png
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

try:
    import numpy as np
    import matplotlib
except ImportError as e:
    print(f"Missing dependency: {e}. pip install numpy matplotlib")
    sys.exit(1)

# Heading (yaw, rad) sources: (label, mu, sigma^2, color).
# Wheel odometry drifts → wide; the IMU yaw is tighter.
SOURCES_1D = [
    ("Wheel odometry (yaw)", 0.30, 0.05, "tab:blue"),
    ("IMU (yaw)",            0.10, 0.01, "tab:orange"),
]
# Position (x, y) sources: (label, mean_xy, cov_2x2, color).
# Odometry dead-reckoning is an elongated, drifting ellipse; the landmark
# range/bearing fix and the lidar scan-match are tight position corrections.
SOURCES_2D = [
    ("Wheel odometry",   (1.00, 0.50), [[0.60, 0.25], [0.25, 0.30]], "tab:blue"),
    ("Landmark fix",     (1.25, 0.58), [[0.10, 0.02], [0.02, 0.10]], "tab:orange"),
    ("Lidar scan-match", (1.40, 0.70), [[0.08, 0.00], [0.00, 0.12]], "tab:green"),
]


def fuse_1d(sources):
    inv = sum(1.0 / s2 for _, _, s2, _ in sources)
    var_f = 1.0 / inv
    mu_f = var_f * sum(mu / s2 for _, mu, s2, _ in sources)
    return mu_f, var_f


def fuse_2d(sources):
    info = np.zeros((2, 2))
    vec = np.zeros(2)
    for _, mean, cov, _ in sources:
        Ci = np.linalg.inv(np.asarray(cov))
        info += Ci
        vec += Ci @ np.asarray(mean)
    cov_f = np.linalg.inv(info)
    mu_f = cov_f @ vec
    return mu_f, cov_f


def gaussian(x, mu, var):
    return np.exp(-0.5 * (x - mu) ** 2 / var) / np.sqrt(2 * np.pi * var)


def ellipse_xy(mean, cov, n_std=2.0, n=100):
    vals, vecs = np.linalg.eigh(np.asarray(cov))
    t = np.linspace(0, 2 * np.pi, n)
    circle = np.stack([np.cos(t), np.sin(t)])
    pts = vecs @ (np.sqrt(vals)[:, None] * n_std * circle)
    return mean[0] + pts[0], mean[1] + pts[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="./results")
    ap.add_argument("--format", choices=["png", "pdf", "pgf"], default="png")
    args = ap.parse_args()

    matplotlib.use("Agg")
    matplotlib.rcParams.update({
        "savefig.dpi": 200, "savefig.bbox": "tight",
        "figure.facecolor": "white", "axes.facecolor": "white",
        "font.family": "sans-serif", "font.size": 11,
        "axes.titlesize": 13, "axes.titleweight": "bold", "axes.labelsize": 11,
        "axes.edgecolor": "0.4", "axes.linewidth": 0.9, "axes.axisbelow": True,
        "axes.grid": True, "grid.color": "0.85", "grid.linewidth": 0.8,
        "xtick.labelsize": 10, "ytick.labelsize": 10, "lines.linewidth": 1.8,
        "legend.fontsize": 9, "legend.frameon": True,
        "legend.framealpha": 0.92, "legend.edgecolor": "0.8",
    })
    import matplotlib.pyplot as plt

    out = Path(args.out).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)

    # ── 1D heading fusion (IMU + wheel odometry) ───────────────────────────
    mu_f, var_f = fuse_1d(SOURCES_1D)
    x = np.linspace(-0.5, 1.0, 600)
    fig, ax = plt.subplots(figsize=(8, 4))
    for label, mu, s2, color in SOURCES_1D:
        ax.plot(x, gaussian(x, mu, s2), color=color, lw=1.5,
                label=f"{label} (μ={mu:g}, σ²={s2:g})")
    ax.plot(x, gaussian(x, mu_f, var_f), color="red", lw=2.5,
            label=f"Fused (μ={mu_f:.2f}, σ²={var_f:.3f})")
    ax.set_xlabel("yaw [rad]"); ax.set_ylabel("probability density")
    ax.set_title("Heading Fusion — IMU + Wheel Odometry (1D Gaussian)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / f"fusion_1d.{args.format}")
    plt.close(fig)
    print(f"wrote {out / f'fusion_1d.{args.format}'}  (fused μ={mu_f:.2f}, σ²={var_f:.2f})")

    # ── 2D fusion (covariance ellipses) ────────────────────────────────────
    mu2, cov2 = fuse_2d(SOURCES_2D)
    fig, ax = plt.subplots(figsize=(7, 6))
    for label, mean, cov, color in SOURCES_2D:
        ex, ey = ellipse_xy(np.asarray(mean), cov)
        ax.fill(ex, ey, color=color, alpha=0.25)
        ax.plot(ex, ey, color=color, lw=1.0)
        ax.scatter(*mean, color=color, s=30, label=label)
    ex, ey = ellipse_xy(mu2, cov2)
    ax.plot(ex, ey, color="red", lw=2.0)
    ax.scatter(*mu2, color="red", s=40, label="Fused")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Position Fusion — Odometry + Landmark + Lidar (2σ ellipses)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / f"fusion_2d.{args.format}")
    plt.close(fig)
    print(f"wrote {out / f'fusion_2d.{args.format}'}  (fused mean={mu2.round(2)})")


if __name__ == "__main__":
    main()
