#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot long-range nav metrics from CSV (works on synthetic data; no Gazebo)."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

from scout_workspace_common import workspace_path


def _read_csv(path: Path):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description="Plot long-nav metrics (README2 extension)")
    ap.add_argument(
        "run_dir",
        nargs="?",
        default="",
        help="Directory with metrics.csv (default: synthesize demo under 3D/runs)",
    )
    ap.add_argument("--demo", action="store_true", help="Write synthetic CSV + plots")
    args = ap.parse_args(argv)

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise SystemExit("matplotlib required: pip install --user matplotlib") from e

    if args.demo or not args.run_dir:
        run_dir = workspace_path(
            "3D",
            "runs",
            "long_nav_demo_synthetic",
            package_name="scout_long_nav",
            start_path=__file__,
        )
        run_dir.mkdir(parents=True, exist_ok=True)
        csv_path = run_dir / "metrics.csv"
        if not csv_path.exists() or args.demo:
            with csv_path.open("w", newline="") as f:
                w = csv.DictWriter(
                    f,
                    fieldnames=[
                        "episode",
                        "success",
                        "collision",
                        "path_efficiency",
                        "path_length",
                        "straight_line",
                    ],
                )
                w.writeheader()
                for ep in range(1, 21):
                    w.writerow(
                        {
                            "episode": ep,
                            "success": 1 if ep % 3 else 0,
                            "collision": 1 if ep % 5 == 0 else 0,
                            "path_efficiency": min(1.0, 0.4 + 0.02 * ep),
                            "path_length": 40.0 + ep,
                            "straight_line": 30.0,
                        }
                    )
        args.run_dir = str(run_dir)

    run_dir = Path(args.run_dir)
    csv_path = run_dir / "metrics.csv"
    if not csv_path.exists():
        raise SystemExit("missing %s" % csv_path)

    rows = _read_csv(csv_path)
    eps = [int(r["episode"]) for r in rows]
    succ = [float(r.get("success", 0)) for r in rows]
    coll = [float(r.get("collision", 0)) for r in rows]
    pe = [float(r.get("path_efficiency", 0)) for r in rows]

    def _cum(xs):
        out, s, n = [], 0.0, 0
        for v in xs:
            n += 1
            s += v
            out.append(s / n)
        return out

    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    axes[0].plot(eps, _cum(succ), label="success_rate_cum")
    axes[0].set_ylabel("success")
    axes[0].legend()
    axes[0].set_title("Long-nav metrics (stage B/C)")
    axes[1].plot(eps, _cum(coll), color="C3", label="collision_rate_cum")
    axes[1].set_ylabel("collision")
    axes[1].legend()
    axes[2].plot(eps, pe, color="C2", label="path_efficiency")
    axes[2].set_ylabel("eff")
    axes[2].set_xlabel("episode")
    axes[2].legend()
    fig.tight_layout()
    out = run_dir / "long_nav_overview.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)

    # Path overlay placeholder if global/actual columns exist later
    overlay = run_dir / "path_overlay.png"
    fig2, ax = plt.subplots(figsize=(6, 6))
    ax.plot([0, 10, 20, 30], [0, 0, 5, 5], "k--", label="global_path (demo)")
    ax.plot([0, 9, 18, 28], [0.2, -0.3, 4.5, 5.2], "r-", label="actual (demo)")
    ax.scatter([4, 8, 12, 16], [0, 0, 2, 4], c="b", s=20, label="local_goal (demo)")
    ax.set_aspect("equal")
    ax.legend()
    ax.set_title("Global vs actual vs local_goal")
    fig2.tight_layout()
    fig2.savefig(overlay, dpi=120)
    plt.close(fig2)

    print("Wrote", out)
    print("Wrote", overlay)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
