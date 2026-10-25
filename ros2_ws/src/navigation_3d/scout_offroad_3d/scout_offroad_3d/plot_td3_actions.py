#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Episode-mean |cmd_vel| (|v|, |ω|) figures from metrics.csv.

Outputs under run_*/action_viz/:
  mean_v_vs_episode.png  — ep mean |linear v| (+ side panel global weighted mean)
  mean_w_vs_episode.png  — ep mean |angular ω| (+ side panel)
  ep_v_w_3d.png          — 3D (episode, |v|, |ω|) with axis limits [0, max_v/max_w]
"""

from __future__ import print_function

import argparse
import csv
import os
import sys


def _apply_style():
    try:
        from scout_offroad_3d.plot_style import apply_paper_style
    except Exception:
        try:
            from plot_style import apply_paper_style
        except Exception:
            return False
    return bool(apply_paper_style())


def _load_max_action(run_dir, default_v=2.3, default_w=1.8):
    max_v, max_w = float(default_v), float(default_w)
    for name in ("config.yaml", "params_backup.yaml"):
        path = os.path.join(run_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            import yaml

            doc = yaml.safe_load(open(path)) or {}
        except Exception:
            continue
        # Nested train / env blobs vary by dump shape
        stack = [doc]
        while stack:
            cur = stack.pop()
            if not isinstance(cur, dict):
                continue
            if "max_v" in cur:
                try:
                    max_v = float(cur["max_v"])
                except (TypeError, ValueError):
                    pass
            if "max_w" in cur:
                try:
                    max_w = float(cur["max_w"])
                except (TypeError, ValueError):
                    pass
            for v in cur.values():
                if isinstance(v, dict):
                    stack.append(v)
        break
    return max_v, max_w


def _read_metrics(run_dir):
    path = os.path.join(run_dir, "metrics.csv")
    if not os.path.isfile(path):
        raise FileNotFoundError("no metrics.csv in %s" % run_dir)
    with open(path, "r") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError("empty metrics.csv")
    header_keys = set(rows[0].keys())
    if "mean_v" not in header_keys or "mean_w" not in header_keys:
        raise ValueError(
            "metrics.csv missing mean_v/mean_w (enable action_stats_enable on a new train)"
        )
    eps, mean_v, mean_w, lengths = [], [], [], []
    for r in rows:
        try:
            ep = int(float(r.get("episode", 0)))
            mv = float(r["mean_v"])
            mw = float(r["mean_w"])
            L = float(r.get("length") or 0)
        except (TypeError, ValueError, KeyError):
            continue
        if L <= 0:
            continue
        eps.append(ep)
        mean_v.append(mv)
        mean_w.append(mw)
        lengths.append(L)
    if not eps:
        raise ValueError("no valid mean_v/mean_w rows")
    return eps, mean_v, mean_w, lengths


def weighted_global_means(mean_v, mean_w, lengths):
    """Step-equivalent global means: sum(mean * L) / sum(L)."""
    import numpy as np

    L = np.asarray(lengths, dtype=np.float64)
    v = np.asarray(mean_v, dtype=np.float64)
    w = np.asarray(mean_w, dtype=np.float64)
    den = float(L.sum())
    if den <= 0:
        return float("nan"), float("nan")
    return float((v * L).sum() / den), float((w * L).sum() / den)


def _line_with_side_stat(eps, ys, ylabel, title, out_path, global_val, global_label):
    import matplotlib.pyplot as plt
    import numpy as np

    fig = plt.figure(figsize=(9.2, 4.2))
    gs = fig.add_gridspec(1, 2, width_ratios=[4.2, 1.15], wspace=0.28)
    ax = fig.add_subplot(gs[0, 0])
    ax_side = fig.add_subplot(gs[0, 1])
    ax_side.axis("off")

    ax.plot(eps, ys, color="#1f4e79", linewidth=1.1, alpha=0.9)
    if len(eps) > 30:
        # light rolling mean for readability
        win = min(50, max(5, len(eps) // 20))
        ker = np.ones(win, dtype=np.float64) / float(win)
        smooth = np.convolve(np.asarray(ys, dtype=np.float64), ker, mode="valid")
        ax.plot(
            eps[win - 1 :],
            smooth,
            color="#c45c26",
            linewidth=1.6,
            label="%d-ep MA" % win,
        )
        ax.legend(loc="best", frameon=False, fontsize=9)
    ax.set_xlabel("Episode")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    ax_side.text(
        0.05,
        0.62,
        global_label,
        transform=ax_side.transAxes,
        fontsize=11,
        fontweight="bold",
        va="top",
    )
    ax_side.text(
        0.05,
        0.42,
        "%.4f" % global_val,
        transform=ax_side.transAxes,
        fontsize=16,
        color="#1f4e79",
        va="top",
        fontfamily="monospace",
    )
    ax_side.text(
        0.05,
        0.22,
        "weighted by\nepisode length\n(= all-step mean)",
        transform=ax_side.transAxes,
        fontsize=8,
        color="#555555",
        va="top",
    )

    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def _plot_3d(eps, mean_v, mean_w, max_v, max_w, out_path):
    import matplotlib.pyplot as plt
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(8.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    eps_a = np.asarray(eps, dtype=np.float64)
    v_a = np.asarray(mean_v, dtype=np.float64)
    w_a = np.asarray(mean_w, dtype=np.float64)
    sc = ax.scatter(
        eps_a,
        v_a,
        w_a,
        c=eps_a,
        cmap="viridis",
        s=12,
        alpha=0.85,
        depthshade=True,
    )
    if len(eps_a) >= 2:
        ax.plot(eps_a, v_a, w_a, color="#666666", linewidth=0.4, alpha=0.35)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Mean |linear v| (m/s)")
    ax.set_zlabel("Mean |angular ω| (rad/s)")
    ax.set_ylim(0.0, abs(max_v))
    ax.set_zlim(0.0, abs(max_w))
    ax.set_title("Episode mean |cmd_vel| (|v|, |ω|) evolution")
    cb = fig.colorbar(sc, ax=ax, shrink=0.65, pad=0.08)
    cb.set_label("Episode")
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def render_action_stats_for_run(run_dir, log_fn=None):
    """Generate action_viz PNGs. Returns list of written paths (may be empty)."""

    def _log(msg, *a):
        text = msg % a if a else msg
        if log_fn:
            try:
                log_fn(text)
            except Exception:
                print(text)
        else:
            print(text)

    run_dir = os.path.abspath(run_dir)
    written = []
    try:
        _apply_style()
        eps, mean_v, mean_w, lengths = _read_metrics(run_dir)
        g_v, g_w = weighted_global_means(mean_v, mean_w, lengths)
        max_v, max_w = _load_max_action(run_dir)
        out_dir = os.path.join(run_dir, "action_viz")
        os.makedirs(out_dir, exist_ok=True)

        p_v = os.path.join(out_dir, "mean_v_vs_episode.png")
        _line_with_side_stat(
            eps,
            mean_v,
            ylabel="Mean |linear velocity| |v| (m/s)",
            title="Episode-mean |linear velocity|",
            out_path=p_v,
            global_val=g_v,
            global_label="Global mean |v|",
        )
        written.append(p_v)

        p_w = os.path.join(out_dir, "mean_w_vs_episode.png")
        _line_with_side_stat(
            eps,
            mean_w,
            ylabel="Mean |angular velocity| |ω| (rad/s)",
            title="Episode-mean |angular velocity|",
            out_path=p_w,
            global_val=g_w,
            global_label="Global mean |ω|",
        )
        written.append(p_w)

        p3 = os.path.join(out_dir, "ep_v_w_3d.png")
        _plot_3d(eps, mean_v, mean_w, max_v, max_w, p3)
        written.append(p3)

        summary = os.path.join(out_dir, "summary.txt")
        with open(summary, "w") as f:
            f.write("global_weighted_mean_v=%.6f\n" % g_v)
            f.write("global_weighted_mean_w=%.6f\n" % g_w)
            f.write("n_episodes=%d\n" % len(eps))
            f.write("total_steps=%d\n" % int(sum(lengths)))
            f.write("max_v=%.4f\n" % max_v)
            f.write("max_w=%.4f\n" % max_w)
        written.append(summary)
        _log(
            "[action_stats] global_v=%.4f global_w=%.4f → %s",
            g_v,
            g_w,
            out_dir,
        )
    except Exception as e:
        _log("[action_stats] skip: %s", e)
    return written


def main(argv=None):
    p = argparse.ArgumentParser(description="Plot episode-mean cmd_vel stats")
    p.add_argument("run_dir", help="path to run_* directory")
    args = p.parse_args(argv)
    paths = render_action_stats_for_run(args.run_dir)
    if not paths:
        return 1
    for x in paths:
        print(x)
    return 0


if __name__ == "__main__":
    sys.exit(main())
