#!/usr/bin/env python3
"""Aggregate the per-experiment CSVs produced by csv_logger.py into:

  1) per-scenario time-series plots (xy error + yaw error per filter)
  2) per-scenario convergence + runtime tables
  3) one summary plot comparing RMSE across all (scenario × filter)

Output formats (--format):
  png   bitmap @ 200 dpi — for the README and PowerPoint slides
  pgf   matplotlib's pgf backend — drop \\input{plot.pgf} into the LaTeX
        paper to get fonts and sizing matching the surrounding text. This
        is the right format for the paper-style documentation; PDF
        \\includegraphics works too but doesn't pick up the document font.
  pdf   vector PDF — useful when including via \\includegraphics if you
        don't want the LaTeX font match.

Inputs:
  --in   directory holding `<scenario>_timeseries.csv` and `<scenario>_summary.csv`
  --out  directory to write plots + aggregated CSV (default: same as --in)

Usage:
    python3 scripts/analyze_results.py --in ./results
    python3 scripts/analyze_results.py --in ./results --format pgf

numpy + matplotlib only (no pandas) so it runs fine on the prof's machine
without extra deps.
"""
from __future__ import annotations
import argparse
import csv
import math
import os
import sys
from pathlib import Path

try:
    import numpy as np
    import matplotlib
except ImportError as e:
    print(f"Missing dependency: {e}. pip install numpy matplotlib")
    sys.exit(1)


FILTERS = ("kf", "ekf", "pf")
COLOURS = {"kf": "tab:blue", "ekf": "tab:green", "pf": "tab:red"}


def configure_backend(fmt: str):
    """Set up the matplotlib backend + rcParams for the chosen output
    format. Must be called *before* `import matplotlib.pyplot`."""
    if fmt == "pgf":
        matplotlib.use("pgf")
        matplotlib.rcParams.update({
            "pgf.texsystem":  "pdflatex",
            "font.family":    "serif",
            "text.usetex":    True,
            "pgf.rcfonts":    False,    # use LaTeX font, not matplotlib's
            "axes.labelsize":  9,
            "font.size":       9,
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
        })
    else:
        # png + pdf both fine on the default Agg backend.
        matplotlib.use("Agg")
        matplotlib.rcParams.update({"savefig.dpi": 200})


def read_timeseries(path: Path) -> dict[str, np.ndarray]:
    """Read a *_timeseries.csv into a dict of column-name → array."""
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


def plot_scenario(ts: dict[str, np.ndarray], scenario: str, out: Path,
                  fmt: str):
    """xy + yaw error, one panel each, all 3 filters overlaid."""
    if "time" not in ts:
        return
    t = ts["time"]
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for f in FILTERS:
        ek = f"{f}_err_xy"
        ey = f"{f}_err_yaw"
        if ek in ts:
            ax[0].plot(t, ts[ek], color=COLOURS[f], label=f.upper())
        if ey in ts:
            ax[1].plot(t, ts[ey], color=COLOURS[f], label=f.upper())
    ax[0].set_ylabel("xy error [m]")
    ax[1].set_ylabel("yaw error [rad]")
    ax[1].set_xlabel("time [s]")
    ax[0].set_title(f"Scenario: {scenario}")
    ax[0].legend(loc="upper right"); ax[1].legend(loc="upper right")
    ax[0].grid(True, alpha=0.3); ax[1].grid(True, alpha=0.3)
    fig.tight_layout()
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / f"{scenario}_error.{fmt}")
    plt.close(fig)


def aggregate_summaries(in_dir: Path, out_dir: Path, fmt: str):
    """Concat all *_summary.csv and emit one comparison bar chart."""
    rows: list[dict[str, str]] = []
    for p in sorted(in_dir.glob("*_summary.csv")):
        with p.open() as f:
            for row in csv.DictReader(f):
                rows.append(row)
    if not rows:
        print("no summary CSVs found")
        return
    fields = list(rows[0].keys())
    agg = out_dir / "all_summaries.csv"
    with agg.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)
    print(f"wrote {agg}")

    scenarios = [r["scenario"] for r in rows]
    x = np.arange(len(scenarios))
    w = 0.27

    def _bar(metric_col: str, ylabel: str, title: str, basename: str):
        if not any(f"{f}_{metric_col}" in fields for f in FILTERS):
            return
        fig, ax = plt.subplots(figsize=(max(6, 0.6 * len(scenarios) * 3), 4.5))
        for i, f in enumerate(FILTERS):
            col = f"{f}_{metric_col}"
            if col not in fields:
                continue
            vals = []
            for r in rows:
                try:
                    vals.append(float(r[col]))
                except (ValueError, TypeError):
                    vals.append(float("nan"))
            ax.bar(x + (i - 1) * w, vals, w, color=COLOURS[f], label=f.upper())
        ax.set_xticks(x)
        ax.set_xticklabels(scenarios, rotation=30, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend()
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        out_path = out_dir / f"{basename}.{fmt}"
        fig.savefig(out_path)
        plt.close(fig)
        print(f"wrote {out_path}")

    _bar("final_rmse_xy",   "final RMSE_xy [m]",
         "Per-scenario RMSE comparison",   "rmse_comparison")
    _bar("runtime_mean_us", "mean update runtime [µs]",
         "Per-update runtime per filter", "runtime_comparison")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in",  dest="in_dir",  default="./results")
    ap.add_argument("--out", dest="out_dir", default=None)
    ap.add_argument("--format", choices=["png", "pgf", "pdf"], default="png",
                    help="png for slides/README, pgf for LaTeX paper, "
                         "pdf for \\includegraphics without LaTeX font match")
    args = ap.parse_args()

    configure_backend(args.format)
    # pyplot must be imported AFTER backend is configured.
    global plt
    import matplotlib.pyplot as plt  # noqa: E402

    in_dir  = Path(args.in_dir).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else in_dir
    if not in_dir.is_dir():
        print(f"input dir not found: {in_dir}")
        sys.exit(1)
    out_dir.mkdir(parents=True, exist_ok=True)

    series = list(in_dir.glob("*_timeseries.csv"))
    if not series:
        print(f"no *_timeseries.csv in {in_dir}")
    for p in series:
        scenario = p.stem.replace("_timeseries", "")
        ts = read_timeseries(p)
        plot_scenario(ts, scenario, out_dir, args.format)
        print(f"plotted {scenario}.{args.format}")

    aggregate_summaries(in_dir, out_dir, args.format)


if __name__ == "__main__":
    main()
