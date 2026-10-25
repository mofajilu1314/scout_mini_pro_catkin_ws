#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Milestone trajectory overlays, outcome heatmaps, pie/CDF helpers for Stage-A TD3.

Online: imported by train_td3_node (never raises into the train loop).
Offline:
  ros2 run scout_offroad_3d plot_td3_eval_viz /path/to/run_dir
  python3 -m scout_offroad_3d.plot_td3_eval_viz --smoke /tmp/viz_smoke
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import re
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np

try:
    from scout_offroad_3d.plot_style import apply_paper_style
except Exception:
    def apply_paper_style():
        return False

# Canopy / tall foliage model names (Gazebo worlds) — treated as above clip_z occlusion.
_TREE_NAME_RE = re.compile(
    r"(pine|oak|tree|forest|leaf|willow|palm|birch|spruce|canopy)", re.I
)

OUTCOME_COLORS = {
    "success": "#2ecc71",
    "stuck": "#f1c40f",
    "timeout": "#e74c3c",
    "rollover": "#9b59b6",
    "collision": "#e67e22",
    "collision_limit": "#e67e22",
    "done": "#95a5a6",
}

OUTCOME_XY_FIELDS = (
    "x",
    "y",
    "outcome",
    "world",
    "episode",
    "sr_milestone",
    "source",
    "path_efficiency",
    "t",
    "point_kind",  # terminal | goal
)

# Bumpy heightmap is 30×30 m centered at origin → fixed heatmap frame ±15.
MAP_HEATMAP_EXTENT = (-15.0, 15.0, -15.0, 15.0)


def _ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def _world_stem(path_or_name):
    base = os.path.basename(str(path_or_name or "unknown"))
    if base.endswith(".world"):
        base = base[: -len(".world")]
    return base or "unknown"


def _smooth_xy(xs, ys, window=5):
    """Moving-average smooth; keeps endpoints."""
    n = len(xs)
    if n < 3 or window < 3:
        return list(xs), list(ys)
    w = min(window, n if n % 2 == 1 else n - 1)
    if w < 3:
        return list(xs), list(ys)
    kernel = np.ones(w, dtype=np.float64) / float(w)
    pad = w // 2
    xa = np.asarray(xs, dtype=np.float64)
    ya = np.asarray(ys, dtype=np.float64)
    xp = np.pad(xa, (pad, pad), mode="edge")
    yp = np.pad(ya, (pad, pad), mode="edge")
    xs_s = np.convolve(xp, kernel, mode="valid")
    ys_s = np.convolve(yp, kernel, mode="valid")
    xs_s[0], ys_s[0] = xa[0], ya[0]
    xs_s[-1], ys_s[-1] = xa[-1], ya[-1]
    return xs_s.tolist(), ys_s.tolist()


def _episode_outcome_from_info(info):
    if not info:
        return "done"
    if info.get("success"):
        return "success"
    if info.get("rollover"):
        return "rollover"
    if info.get("stuck"):
        return "stuck"
    if info.get("timeout"):
        return "timeout"
    if info.get("collision_limit") or info.get("collision"):
        return "collision"
    return "done"


# ---------------------------------------------------------------------------
# Benchmark pairs
# ---------------------------------------------------------------------------


def pick_benchmark_pairs(
    env,
    pairs_per_world=3,
    min_pair_dist=20.0,
    batch_size=20,
    batch_rounds=5,
    max_worlds=4,
    include_holdout=None,
    n_candidate_tries=None,
    log_fn=None,
    report_out=None,
):
    """Pre-generate fixed start–goal pairs per world (train-parity placement).

    For each world:
      1) Sample a batch of ``batch_size`` geometric pairs with dist >= min_pair_dist
         in the same spawn/goal boxes as training (typically [-10,10]^2).
      2) Validate spawn/goal like training.
      3) Repeat up to ``batch_rounds`` (default 5 → ~100 far candidates).
      4) Keep up to ``pairs_per_world`` (prefer farther + start diversity).
      5) If fewer than requested: degrade (fewer pairs) and record — still OK to train.

    ``n_candidate_tries`` is ignored (compat with old callers).
    """
    del include_holdout, n_candidate_tries

    def _log(msg, *args):
        if log_fn:
            log_fn(msg, *args)

    pairs_per_world = max(1, int(pairs_per_world))
    min_pair_dist = float(min_pair_dist)
    batch_size = max(1, int(batch_size))
    batch_rounds = max(1, int(batch_rounds))
    out = {
        "meta": {
            "pairs_per_world": pairs_per_world,
            "min_pair_dist": min_pair_dist,
            "batch_size": batch_size,
            "batch_rounds": batch_rounds,
            "n_worlds_target": 4,
            "n_traj_pngs_target": 4 * pairs_per_world,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        },
        "worlds": {},
    }
    report = {
        "created_at": out["meta"]["created_at"],
        "min_pair_dist": min_pair_dist,
        "batch_size": batch_size,
        "batch_rounds": batch_rounds,
        "pairs_per_world_requested": pairs_per_world,
        "worlds": {},
        "degraded": False,
    }
    try:
        worlds = list(env.list_worlds())[: max(1, int(max_worlds))]
    except Exception as e:
        _log("[viz_eval] list_worlds failed: %s", e)
        worlds = []
        if getattr(env, "current_world", None):
            worlds = [env.current_world]

    original = getattr(env, "current_world", None) or None
    for wpath in worlds:
        stem = _world_stem(wpath)
        switched = True
        if original is None or os.path.abspath(wpath) != os.path.abspath(original or ""):
            try:
                env.switch_world(wpath, timestep=None)
            except Exception as e:
                _log("[viz_eval] switch to %s failed (%s) — geometric fallback", stem, e)
                switched = False
        pairs, wrep = _sample_pairs_batched(
            env,
            pairs_per_world=pairs_per_world,
            min_pair_dist=min_pair_dist,
            batch_size=batch_size,
            batch_rounds=batch_rounds,
            validate=switched,
            log_fn=log_fn,
        )
        degraded = int(wrep.get("kept", 0)) < pairs_per_world
        if degraded:
            report["degraded"] = True
            _log(
                "[viz_eval] world=%s only %d/%d pairs after %dx%d — degraded",
                stem,
                int(wrep.get("kept", 0)),
                pairs_per_world,
                batch_rounds,
                batch_size,
            )
        out["worlds"][stem] = {
            "world_path": os.path.abspath(wpath) if wpath else "",
            "pairs": pairs,
            "degraded": degraded,
        }
        report["worlds"][stem] = wrep
        _log("[viz_eval] world=%s pairs=%d (requested=%d)", stem, len(pairs), pairs_per_world)

    if original:
        try:
            if os.path.abspath(env.current_world or "") != os.path.abspath(original):
                env.switch_world(original, timestep=None)
        except Exception as e:
            _log("[viz_eval] restore world failed: %s", e)

    if report_out is not None:
        report_out.clear()
        report_out.update(report)
    out["meta"]["degraded"] = bool(report["degraded"])
    return out


def _sample_far_geometry_batch(env, batch_size, min_pair_dist, floor_dist):
    """Sample up to batch_size pairs with dist >= min_pair_dist (no placement yet)."""
    batch = []
    # Generous geometric tries so a full batch of "far enough" pairs is realistic.
    max_geo = max(batch_size * 80, 500)
    for _ in range(max_geo):
        if len(batch) >= batch_size:
            break
        start = env._sample_xy(env.spawn_xy_min, env.spawn_xy_max)
        goal = env._sample_xy(env.goal_xy_min, env.goal_xy_max)
        dist = float(math.hypot(goal[0] - start[0], goal[1] - start[1]))
        if dist < float(floor_dist):
            continue
        if dist < float(min_pair_dist):
            continue
        yaw = float(np.random.uniform(-math.pi, math.pi))
        batch.append(
            {
                "start": [float(start[0]), float(start[1])],
                "goal": [float(goal[0]), float(goal[1])],
                "yaw": yaw,
                "dist": dist,
            }
        )
    return batch


def _validate_pair_placement(env, pair, validate=True):
    if not validate:
        return True
    start = pair["start"]
    goal = pair["goal"]
    yaw = float(pair["yaw"])
    try:
        if not env._ensure_spawn_ok(
            float(start[0]), float(start[1]), yaw, context="viz_pair", allow_xy_retry=False
        ):
            return False
        if env.goal_check_enable:
            g_ok, _reason = env._validate_goal(
                float(goal[0]), float(goal[1]), float(start[0]), float(start[1]), yaw
            )
            if not g_ok:
                return False
    except Exception:
        return False
    return True


def _select_diverse_pairs(cands, pairs_per_world):
    cands = sorted(cands, key=lambda p: p["dist"], reverse=True)
    selected = []
    for c in cands:
        if len(selected) >= pairs_per_world:
            break
        too_close = False
        for s in selected:
            if math.hypot(c["start"][0] - s["start"][0], c["start"][1] - s["start"][1]) < 1.0:
                too_close = True
                break
        if too_close and len(cands) > pairs_per_world * 2:
            continue
        selected.append(c)
    if len(selected) < pairs_per_world:
        for c in cands:
            if c in selected:
                continue
            selected.append(c)
            if len(selected) >= pairs_per_world:
                break
    return selected


def _sample_pairs_batched(
    env,
    pairs_per_world=3,
    min_pair_dist=20.0,
    batch_size=20,
    batch_rounds=5,
    validate=True,
    log_fn=None,
):
    """Batch far geometry → validate; degrade if still short."""
    floor_dist = float(getattr(env, "min_goal_dist", 3.0) or 3.0)
    validated = []
    geo_total = 0
    ok_total = 0
    rounds_used = 0
    for r in range(int(batch_rounds)):
        rounds_used = r + 1
        batch = _sample_far_geometry_batch(env, batch_size, min_pair_dist, floor_dist)
        geo_total += len(batch)
        if log_fn:
            log_fn(
                "[viz_eval] pair batch %d/%d: geo=%d (need dist>=%.1f)",
                r + 1,
                batch_rounds,
                len(batch),
                float(min_pair_dist),
            )
        for pair in batch:
            if _validate_pair_placement(env, pair, validate=validate):
                validated.append(pair)
                ok_total += 1
        # Dedup nearly-identical pairs
        uniq = []
        for p in validated:
            dup = False
            for u in uniq:
                if (
                    math.hypot(p["start"][0] - u["start"][0], p["start"][1] - u["start"][1]) < 0.25
                    and math.hypot(p["goal"][0] - u["goal"][0], p["goal"][1] - u["goal"][1]) < 0.25
                ):
                    dup = True
                    break
            if not dup:
                uniq.append(p)
        validated = uniq
        if len(validated) >= pairs_per_world:
            break

    selected = _select_diverse_pairs(validated, pairs_per_world)
    report = {
        "requested": int(pairs_per_world),
        "geometry_candidates": int(geo_total),
        "validated": int(ok_total),
        "unique_validated": int(len(validated)),
        "kept": int(len(selected)),
        "rounds_used": int(rounds_used),
        "degraded": int(len(selected)) < int(pairs_per_world),
        "min_pair_dist": float(min_pair_dist),
        "validate": bool(validate),
    }
    return selected, report


def _sample_max_dist_pairs(
    env,
    pairs_per_world=3,
    n_tries=120,
    validate=True,
    log_fn=None,
    min_pair_dist=None,
):
    """Backward-compatible wrapper → batched sampler. """
    del n_tries
    if min_pair_dist is None:
        min_pair_dist = max(20.0, float(getattr(env, "min_goal_dist", 3.0) or 3.0))
    selected, _rep = _sample_pairs_batched(
        env,
        pairs_per_world=pairs_per_world,
        min_pair_dist=float(min_pair_dist),
        batch_size=20,
        batch_rounds=5,
        validate=validate,
        log_fn=log_fn,
    )
    return selected


def save_benchmark_pairs(pairs_doc, path):
    import yaml

    _ensure_dir(os.path.dirname(path) or ".")
    with open(path, "w") as f:
        yaml.safe_dump(pairs_doc, f, default_flow_style=False, sort_keys=False)


def save_pair_gen_report(report, path):
    import yaml

    _ensure_dir(os.path.dirname(path) or ".")
    with open(path, "w") as f:
        yaml.safe_dump(report or {}, f, default_flow_style=False, sort_keys=False)


def load_benchmark_pairs(path):
    import yaml

    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


# ---------------------------------------------------------------------------
# CSV / heatmaps
# ---------------------------------------------------------------------------


def append_outcome_xy_csv(csv_path, row):
    _ensure_dir(os.path.dirname(csv_path) or ".")
    new_file = not os.path.isfile(csv_path)
    with open(csv_path, "a") as f:
        w = csv.DictWriter(f, fieldnames=OUTCOME_XY_FIELDS, extrasaction="ignore")
        if new_file:
            w.writeheader()
        payload = {k: row.get(k, "") for k in OUTCOME_XY_FIELDS}
        w.writerow(payload)


def read_outcome_xy_csv(csv_path):
    if not os.path.isfile(csv_path):
        return []
    with open(csv_path, "r") as f:
        return list(csv.DictReader(f))


def render_heatmap(
    rows,
    out_png,
    title=None,
    grid_res=0.5,
    sigma=2.2,
    extent=None,
):
    """Success/failure density as smooth contour bands on a fixed map (±15 m).

    Unvisited cells stay white. Diverging palette: soft amber (fail) → cream →
    sage green (success) — avoids harsh red/green clash.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap
        import numpy.ma as ma
    except Exception as e:
        raise RuntimeError("matplotlib required for heatmap: %s" % e)

    apply_paper_style()

    pts = []
    for r in rows:
        try:
            x = float(r["x"])
            y = float(r["y"])
        except (KeyError, TypeError, ValueError):
            continue
        oc = str(r.get("outcome") or "").lower()
        score = 1.0 if oc == "success" else -1.0
        pts.append((x, y, score))
    if not pts:
        return False

    xs = np.array([p[0] for p in pts], dtype=np.float64)
    ys = np.array([p[1] for p in pts], dtype=np.float64)
    sc = np.array([p[2] for p in pts], dtype=np.float64)
    if extent is None:
        extent = MAP_HEATMAP_EXTENT
    xmin, xmax, ymin, ymax = [float(v) for v in extent]
    if xmax <= xmin or ymax <= ymin:
        raise ValueError("invalid heatmap extent: %s" % (extent,))
    res = max(float(grid_res), 0.1)
    nx = max(8, int(math.ceil((xmax - xmin) / res)))
    ny = max(8, int(math.ceil((ymax - ymin) / res)))
    field = np.zeros((ny, nx), dtype=np.float64)
    weight = np.zeros((ny, nx), dtype=np.float64)
    for x, y, s in zip(xs, ys, sc):
        ix = int((x - xmin) / res)
        iy = int((y - ymin) / res)
        if 0 <= ix < nx and 0 <= iy < ny:
            field[iy, ix] += s
            weight[iy, ix] += 1.0
    try:
        from scipy.ndimage import gaussian_filter

        field = gaussian_filter(field, sigma=sigma)
        weight = gaussian_filter(weight, sigma=sigma)
    except Exception:
        k = max(1, int(round(sigma * 2)))
        kernel = np.ones((k, k), dtype=np.float64) / float(k * k)
        field = _conv2(field, kernel)
        weight = _conv2(weight, kernel)
    dens = np.where(weight > 1e-6, field / np.maximum(weight, 1e-6), 0.0)

    # Unvisited → masked (drawn as white / transparent fill)
    visit_thr = max(1e-4, 0.08 * float(np.nanmax(weight)) if np.nanmax(weight) > 0 else 1e-4)
    dens_m = ma.masked_where(weight < visit_thr, dens)

    # Soft diverging: steel-blue fail → paper cream → sage success
    # (avoids red/green clash and brown+green "rotten meat" look)
    cmap = LinearSegmentedColormap.from_list(
        "outcome_topo",
        [
            "#3d5a80",  # steel blue (strong fail)
            "#98afc7",  # mist blue
            "#f4f1ea",  # paper cream (neutral / mixed)
            "#b8d4be",  # pale sage
            "#2f6b4f",  # deep sage (strong success)
        ],
        N=256,
    )
    cmap.set_bad("#ffffff", alpha=0.0)

    fig, ax = plt.subplots(figsize=(7.8, 7.5))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # Mesh for contour (cell centers)
    xc = xmin + (np.arange(nx) + 0.5) * res
    yc = ymin + (np.arange(ny) + 0.5) * res
    XX, YY = np.meshgrid(xc, yc)

    vmax = max(0.35, float(np.nanmax(np.abs(dens_m.compressed()))) if dens_m.count() else 1.0)
    levels = np.linspace(-vmax, vmax, 21)

    cf = ax.contourf(
        XX,
        YY,
        dens_m,
        levels=levels,
        cmap=cmap,
        vmin=-vmax,
        vmax=vmax,
        extend="both",
        antialiased=True,
    )
    # Thin topographic contour lines (skip near-zero band for less clutter)
    line_levels = [lv for lv in levels[1:-1:2] if abs(lv) > 0.08 * vmax]
    if line_levels:
        ax.contour(
            XX,
            YY,
            dens_m,
            levels=line_levels,
            colors="#5c5346",
            linewidths=0.35,
            alpha=0.35,
        )

    # Sparse endpoint dots — soft, low alpha (don't fight the contours)
    in_frame = (xs >= xmin) & (xs <= xmax) & (ys >= ymin) & (ys <= ymax)
    ax.scatter(
        xs[(sc > 0) & in_frame],
        ys[(sc > 0) & in_frame],
        c="#2f5d45",
        s=5,
        alpha=0.18,
        linewidths=0,
        zorder=3,
    )
    ax.scatter(
        xs[(sc < 0) & in_frame],
        ys[(sc < 0) & in_frame],
        c="#3d5a80",
        s=5,
        alpha=0.14,
        linewidths=0,
        zorder=3,
    )

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title or "Outcome heatmap")
    cbar = fig.colorbar(cf, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("failure ← blue · cream · sage → success")
    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=170, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return True


def _conv2(arr, kernel):
    from numpy.lib.stride_tricks import as_strided

    # simple padded valid approx via edge pad + mean
    ky, kx = kernel.shape
    py, px = ky // 2, kx // 2
    padded = np.pad(arr, ((py, py), (px, px)), mode="edge")
    out = np.zeros_like(arr)
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            out[i, j] = np.sum(padded[i : i + ky, j : j + kx] * kernel)
    return out


# ---------------------------------------------------------------------------
# Trajectory overlay / pie / CDF
# ---------------------------------------------------------------------------


def render_trajectory_overlay(trajs, out_png, opacity=0.5, title=None, transparent=True):
    """Composite trajectories on one figure; transparent bg for Photoshop."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError("matplotlib required: %s" % e)

    apply_paper_style()

    fig, ax = plt.subplots(figsize=(8, 8))
    if transparent:
        fig.patch.set_alpha(0.0)
        ax.patch.set_alpha(0.0)
    else:
        fig.patch.set_facecolor("white")
        ax.set_facecolor("white")

    all_x, all_y = [], []
    for tr in trajs:
        xs = [p[0] for p in tr.get("poses") or []]
        ys = [p[1] for p in tr.get("poses") or []]
        if len(xs) < 1:
            sx, sy = tr.get("start") or [None, None]
            gx, gy = tr.get("goal") or [None, None]
            if sx is not None:
                xs, ys = [sx, gx], [sy, gy]
        if len(xs) < 2:
            continue
        xs, ys = _smooth_xy(xs, ys, window=5)
        oc = str(tr.get("outcome") or "done").lower()
        color = OUTCOME_COLORS.get(oc, "#95a5a6")
        ax.plot(xs, ys, color=color, alpha=float(opacity), linewidth=2.0, solid_capstyle="round")
        ax.scatter([xs[0]], [ys[0]], c=color, s=28, alpha=min(1.0, float(opacity) + 0.25), zorder=3)
        ax.scatter(
            [xs[-1]],
            [ys[-1]],
            c=color,
            s=55,
            marker="o",
            edgecolors="black" if not transparent else color,
            linewidths=0.6,
            alpha=min(1.0, float(opacity) + 0.35),
            zorder=4,
        )
        g = tr.get("goal")
        if g and len(g) >= 2:
            ax.scatter([g[0]], [g[1]], c=color, s=40, marker="x", alpha=0.9, zorder=5)
        all_x.extend(xs)
        all_y.extend(ys)

    if all_x:
        pad = 0.8
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)
    ax.set_aspect("equal", adjustable="box")
    # Per-pair Photoshop overlays: no axes/legend clutter (user composites themselves)
    clean = bool(transparent) and not title
    if clean:
        ax.set_axis_off()
    else:
        ax.set_xlabel("x (m)", color="#333333")
        ax.set_ylabel("y (m)", color="#333333")
        if title:
            ax.set_title(title, color="#222222")
        from matplotlib.lines import Line2D

        handles = [
            Line2D([0], [0], color=OUTCOME_COLORS[k], lw=2, label=k)
            for k in ("success", "stuck", "timeout", "rollover")
        ]
        leg = ax.legend(handles=handles, loc="upper right", framealpha=0.35)
        if transparent and leg is not None:
            leg.get_frame().set_alpha(0.25)

    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=180, transparent=bool(transparent), bbox_inches="tight", pad_inches=0.02 if clean else 0.1)
    plt.close(fig)
    return True


def render_trajectory_grid(trajs_by_world, out_png, opacity=0.5, transparent=True):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
    except Exception as e:
        raise RuntimeError("matplotlib required: %s" % e)

    worlds = list(trajs_by_world.keys())
    n = max(1, len(worlds))
    cols = 2
    rows = int(math.ceil(n / float(cols)))
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 6 * rows))
    if transparent:
        fig.patch.set_alpha(0.0)
    axes = np.atleast_1d(axes).ravel()
    for i, stem in enumerate(worlds):
        ax = axes[i]
        if transparent:
            ax.patch.set_alpha(0.0)
        trajs = trajs_by_world[stem]
        for tr in trajs:
            xs = [p[0] for p in tr.get("poses") or []]
            ys = [p[1] for p in tr.get("poses") or []]
            if len(xs) < 2:
                continue
            xs, ys = _smooth_xy(xs, ys)
            oc = str(tr.get("outcome") or "done").lower()
            color = OUTCOME_COLORS.get(oc, "#95a5a6")
            ax.plot(xs, ys, color=color, alpha=float(opacity), linewidth=1.8)
            ax.scatter([xs[-1]], [ys[-1]], c=color, s=36, zorder=3)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(stem)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
    for j in range(len(worlds), len(axes)):
        axes[j].axis("off")
    handles = [
        Line2D([0], [0], color=OUTCOME_COLORS[k], lw=2, label=k)
        for k in ("success", "stuck", "timeout", "rollover")
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, framealpha=0.3)
    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=160, transparent=bool(transparent), bbox_inches="tight")
    plt.close(fig)
    return True


def render_failure_pie(outcomes, out_png, title=None):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError("matplotlib required: %s" % e)

    counts = {}
    for oc in outcomes:
        k = str(oc or "done").lower()
        counts[k] = counts.get(k, 0) + 1
    if not counts:
        return False
    labels = list(counts.keys())
    sizes = [counts[k] for k in labels]
    colors = [OUTCOME_COLORS.get(k, "#95a5a6") for k in labels]
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.pie(sizes, labels=labels, colors=colors, autopct="%1.0f%%", startangle=90)
    ax.set_title(title or "Eval outcomes")
    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=140, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return True


def render_path_efficiency_cdf(metrics_csv, out_png):
    """Easy-win extra: CDF of path_efficiency from metrics.csv."""
    if not os.path.isfile(metrics_csv):
        return False
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    apply_paper_style()
    vals = []
    with open(metrics_csv, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                v = float(r.get("path_efficiency") or "")
            except (TypeError, ValueError):
                continue
            if v >= 0.0:
                vals.append(v)
    if len(vals) < 2:
        return False
    vals = np.sort(np.asarray(vals, dtype=np.float64))
    y = np.linspace(0.0, 1.0, len(vals), endpoint=True)
    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    ax.plot(vals, y, color="#1f77b4", linewidth=2.0)
    ax.set_xlabel("path_efficiency")
    ax.set_ylabel("CDF")
    ax.set_title("Path efficiency CDF (training episodes)")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0.0, 1.05)
    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=140, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return True


# ---------------------------------------------------------------------------
# Online eval runner
# ---------------------------------------------------------------------------


def run_benchmark_eval(policy, env, pairs_doc, max_steps=None, log_fn=None):
    """Run frozen-policy eval on all pairs. Does not touch replay/metrics.

    ``max_steps`` None or <=0 → use env.max_steps (same as training max_ep_steps).

    Returns list of trajectory dicts.
    """

    def _log(msg, *args):
        if log_fn:
            log_fn(msg, *args)

    worlds = (pairs_doc or {}).get("worlds") or {}
    trajs = []
    old_max = getattr(env, "max_steps", 600)
    if max_steps is None or int(max_steps) <= 0:
        use_max = int(old_max)
    else:
        use_max = int(max_steps)
    original = getattr(env, "current_world", None)
    try:
        env.max_steps = use_max
        for stem, block in worlds.items():
            wpath = (block or {}).get("world_path") or ""
            pairs = (block or {}).get("pairs") or []
            if wpath:
                try:
                    if os.path.abspath(env.current_world or "") != os.path.abspath(wpath):
                        env.switch_world(wpath, timestep=None)
                except Exception as e:
                    _log("[viz_eval] eval switch %s failed: %s", stem, e)
            for i, pair in enumerate(pairs):
                tr = _run_one_pair(policy, env, pair, stem, pair_idx=i, log_fn=log_fn)
                if tr is not None:
                    trajs.append(tr)
    finally:
        env.max_steps = old_max
        if original:
            try:
                if os.path.abspath(env.current_world or "") != os.path.abspath(original):
                    env.switch_world(original, timestep=None)
            except Exception as e:
                _log("[viz_eval] restore after eval failed: %s", e)
    return trajs


def _run_one_pair(policy, env, pair, world_stem, pair_idx=0, log_fn=None):
    start = pair.get("start") or [0.0, 0.0]
    goal = pair.get("goal") or [1.0, 0.0]
    yaw = float(pair.get("yaw", 0.0))
    try:
        state = env.reset(
            count_episode=False,
            start_xy=(float(start[0]), float(start[1])),
            goal_xy=(float(goal[0]), float(goal[1])),
            yaw=yaw,
            allow_world_switch=False,
        )
    except Exception as e:
        if log_fn:
            log_fn("[viz_eval] reset pair %s#%d failed: %s", world_stem, pair_idx, e)
        return None
    if state is None:
        if log_fn:
            log_fn("[viz_eval] reset returned None for %s#%d", world_stem, pair_idx)
        return None

    poses = []
    try:
        x0, y0, _ = env._wait_pose(timeout=5.0)
        poses.append([float(x0), float(y0)])
    except Exception:
        poses.append([float(start[0]), float(start[1])])

    outcome = "timeout"
    info = {}
    path_eff = -1.0
    while True:
        try:
            action = policy.select_action(np.array(state, dtype=np.float32))
        except Exception as e:
            if log_fn:
                log_fn("[viz_eval] select_action failed: %s", e)
            outcome = "done"
            break
        try:
            next_state, _reward, done, info = env.step(action)
        except Exception as e:
            if log_fn:
                log_fn("[viz_eval] step failed: %s", e)
            outcome = "done"
            break
        if info.get("episode_abort"):
            state = next_state
            continue
        x = info.get("x")
        y = info.get("y")
        if x is not None and y is not None:
            poses.append([float(x), float(y)])
        state = next_state
        if done:
            outcome = _episode_outcome_from_info(info)
            path_eff = float(info.get("path_efficiency", -1.0))
            break

    return {
        "world": world_stem,
        "pair_idx": int(pair_idx),
        "start": [float(start[0]), float(start[1])],
        "goal": [float(goal[0]), float(goal[1])],
        "yaw": yaw,
        "outcome": outcome,
        "poses": poses,
        "path_efficiency": path_eff,
        "dist": float(pair.get("dist") or math.hypot(goal[0] - start[0], goal[1] - start[1])),
    }


class VizEvalController(object):
    """Legacy SR-milestone ladder (kept for resume/compat; unused by default)."""

    def __init__(self, milestones=None, period_after_peak_ep=250):
        ms = list(milestones) if milestones else [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        self.milestones = sorted(float(m) for m in ms)
        self.period = max(0, int(period_after_peak_ep))
        self.next_idx = 0
        self.peak_hit = 0.0
        self.ladder_done = False
        self.last_period_ep = -10 ** 9
        self.last_tag = ""

    def to_dict(self):
        return {
            "milestones": self.milestones,
            "period": self.period,
            "next_idx": self.next_idx,
            "peak_hit": self.peak_hit,
            "ladder_done": self.ladder_done,
            "last_period_ep": self.last_period_ep,
            "mode": "sr_milestone",
        }

    def load_dict(self, d):
        if not d:
            return
        self.next_idx = int(d.get("next_idx", self.next_idx))
        self.peak_hit = float(d.get("peak_hit", self.peak_hit))
        self.ladder_done = bool(d.get("ladder_done", self.ladder_done))
        self.last_period_ep = int(d.get("last_period_ep", self.last_period_ep))

    def should_run(self, success_rate_win, episode_num):
        """Return (run:bool, milestone:float|None, tag:str)."""
        sr = float(success_rate_win)
        if not self.ladder_done and self.next_idx < len(self.milestones):
            m = self.milestones[self.next_idx]
            if sr + 1e-12 >= m:
                self.next_idx += 1
                self.peak_hit = max(self.peak_hit, m)
                if self.next_idx >= len(self.milestones):
                    self.ladder_done = True
                tag = "sr%02d" % int(round(m * 100))
                self.last_tag = tag
                return True, m, tag
        if self.ladder_done and self.period > 0:
            if sr + 1e-12 < self.peak_hit:
                return False, None, ""
            if int(episode_num) - int(self.last_period_ep) >= self.period:
                tag = "periodic_ep%d" % int(episode_num)
                self.last_tag = tag
                self.last_period_ep = int(episode_num)
                return True, self.peak_hit, tag
        return False, None, ""


class EpisodeIntervalEvalController(object):
    """Freeze-policy traj eval every N training episodes."""

    def __init__(self, every_n_ep=100):
        self.every_n_ep = max(1, int(every_n_ep))
        self.last_eval_ep = 0
        self.last_tag = ""
        self.milestones = None  # heatmaps no longer keyed on SR

    def to_dict(self):
        return {
            "mode": "episode_interval",
            "every_n_ep": self.every_n_ep,
            "last_eval_ep": self.last_eval_ep,
            "last_tag": self.last_tag,
        }

    def load_dict(self, d):
        if not d:
            return
        if d.get("every_n_ep") is not None:
            self.every_n_ep = max(1, int(d["every_n_ep"]))
        self.last_eval_ep = int(d.get("last_eval_ep", self.last_eval_ep))
        self.last_tag = str(d.get("last_tag") or "")

    def should_run(self, success_rate_win, episode_num):
        del success_rate_win
        ep = int(episode_num)
        if ep <= 0:
            return False, None, ""
        if ep % self.every_n_ep != 0:
            return False, None, ""
        if ep == int(self.last_eval_ep):
            return False, None, ""
        tag = "ep%04d" % ep
        self.last_eval_ep = ep
        self.last_tag = tag
        return True, None, tag


# ---------------------------------------------------------------------------
# Top-down basemap (clip canopy) + auto-composite (no Photoshop)
# ---------------------------------------------------------------------------


def extent_from_env(env, pad=1.0):
    """World-frame axis-aligned extent from spawn/goal boxes."""
    try:
        xmin = min(env.spawn_xy_min[0], env.goal_xy_min[0]) - pad
        xmax = max(env.spawn_xy_max[0], env.goal_xy_max[0]) + pad
        ymin = min(env.spawn_xy_min[1], env.goal_xy_min[1]) - pad
        ymax = max(env.spawn_xy_max[1], env.goal_xy_max[1]) + pad
        return (float(xmin), float(xmax), float(ymin), float(ymax))
    except Exception:
        return (-8.0, 8.0, -8.0, 8.0)


def extent_from_trajs(trajs, pad=1.0, fallback=None):
    xs, ys = [], []
    for tr in trajs or []:
        for p in tr.get("poses") or []:
            xs.append(float(p[0]))
            ys.append(float(p[1]))
        for key in ("start", "goal"):
            g = tr.get(key)
            if g and len(g) >= 2:
                xs.append(float(g[0]))
                ys.append(float(g[1]))
    if not xs:
        return fallback or (-8.0, 8.0, -8.0, 8.0)
    return (
        float(min(xs) - pad),
        float(max(xs) + pad),
        float(min(ys) - pad),
        float(max(ys) + pad),
    )


def _basemap_cache_path(out_root, world_stem, clip_z):
    return os.path.join(
        out_root, "basemaps", "%s_clip%.1f.png" % (world_stem, float(clip_z))
    )


def _parse_world_tall_models(world_path, clip_z):
    """Return model names that are tree-like or whose pose z >= clip_z."""
    names = []
    if not world_path or not os.path.isfile(world_path):
        return names
    try:
        tree = ET.parse(world_path)
        root = tree.getroot()
    except Exception:
        return names
    for model in root.iter("model"):
        name = model.get("name") or ""
        if not name:
            continue
        if _TREE_NAME_RE.search(name):
            names.append(name)
            continue
        pose_el = None
        for child in list(model):
            tag = child.tag.split("}")[-1] if "}" in child.tag else child.tag
            if tag == "pose":
                pose_el = child
                break
        if pose_el is not None and (pose_el.text or "").strip():
            parts = (pose_el.text or "").strip().split()
            if len(parts) >= 3:
                try:
                    z = float(parts[2])
                except ValueError:
                    z = 0.0
                if z >= float(clip_z):
                    names.append(name)
    # unique preserve order
    seen = set()
    out = []
    for n in names:
        if n not in seen:
            seen.add(n)
            out.append(n)
    return out


def hide_tall_occlusion(env, clip_z=5.5, log_fn=None):
    """Delete canopy / tall models so ground is visible in top-down views.

    Caller should restore by reloading the same world (switch_world).
    Returns deleted model names.
    """
    world_path = getattr(env, "current_world", None) or ""
    names = _parse_world_tall_models(world_path, clip_z)
    deleted = []
    gz = getattr(env, "_gazebo", None)
    if gz is None or not hasattr(gz, "_delete_model"):
        if log_fn:
            log_fn("[viz_topdown] no gazebo delete API — skip hide")
        return deleted
    for name in names:
        try:
            gz._delete_model(name)
            deleted.append(name)
        except Exception as e:
            if log_fn:
                log_fn("[viz_topdown] delete %s failed: %s", name, e)
    if log_fn:
        log_fn(
            "[viz_topdown] hid %d tall/canopy models (clip_z=%.1f) on %s",
            len(deleted),
            float(clip_z),
            _world_stem(world_path),
        )
    return deleted


def restore_world_after_hide(env, log_fn=None):
    """Reload current .world so deleted canopy models come back."""
    wpath = getattr(env, "current_world", None)
    if not wpath:
        return False
    try:
        env.switch_world(wpath, timestep=None)
        if log_fn:
            log_fn("[viz_topdown] restored world %s after canopy hide", _world_stem(wpath))
        return True
    except Exception as e:
        if log_fn:
            log_fn("[viz_topdown] restore world failed: %s", e)
        return False


def render_schematic_basemap(out_png, extent, title=None, clip_z=5.5):
    """Always-available top-down placeholder (ground visible; notes clip_z).

    Writes a clean raster (no axes chrome) so traj overlay composites cleanly.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        raise RuntimeError("matplotlib required: %s" % e)

    xmin, xmax, ymin, ymax = [float(v) for v in extent]
    # Procedural ground texture in world frame (row0 = ymax for image coords)
    h, w = 512, 512
    yy = np.linspace(ymax, ymin, h)
    xx = np.linspace(xmin, xmax, w)
    X, Y = np.meshgrid(xx, yy)
    noise = (np.sin(X * 0.7) * np.cos(Y * 0.55) + 1.0) * 0.5
    rgb = np.zeros((h, w, 3), dtype=np.float64)
    rgb[:, :, 0] = 0.28 + 0.12 * noise
    rgb[:, :, 1] = 0.42 + 0.20 * noise
    rgb[:, :, 2] = 0.18 + 0.08 * noise
    # grid lines every 2m
    for gx in np.arange(np.ceil(xmin / 2.0) * 2.0, xmax + 1e-6, 2.0):
        ix = int((gx - xmin) / max(xmax - xmin, 1e-6) * (w - 1))
        if 0 <= ix < w:
            rgb[:, ix, :] = np.minimum(rgb[:, ix, :] + 0.08, 1.0)
    for gy in np.arange(np.ceil(ymin / 2.0) * 2.0, ymax + 1e-6, 2.0):
        iy = int((ymax - gy) / max(ymax - ymin, 1e-6) * (h - 1))
        if 0 <= iy < h:
            rgb[iy, :, :] = np.minimum(rgb[iy, :, :] + 0.08, 1.0)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(rgb, extent=[xmin, xmax, ymin, ymax], origin="upper", aspect="equal")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.axis("off")
    # tiny caption burned into corner of raster via text outside? skip for clean map
    _ensure_dir(os.path.dirname(out_png) or ".")
    fig.savefig(out_png, dpi=120, bbox_inches="tight", pad_inches=0, facecolor="none")
    plt.close(fig)
    # Also dump raw array for perfect extent alignment
    try:
        import matplotlib.image as mpimg

        # Re-save pure array without any figure padding
        mpimg.imsave(out_png, np.clip(rgb, 0, 1))
    except Exception:
        pass
    meta = out_png + ".extent.json"
    with open(meta, "w") as f:
        json.dump(
            {
                "extent": list(extent),
                "clip_z": float(clip_z),
                "mode": "schematic",
                "title": title or "",
            },
            f,
        )
    return out_png


def ensure_topdown_basemap(
    out_root,
    world_stem,
    extent,
    clip_z=5.5,
    env=None,
    enable=True,
    log_fn=None,
):
    """Cache a schematic top-down PNG. Training no longer calls this.

    Gazebo hide-canopy / world-reload photography was removed.
    """
    if not enable:
        return None
    if extent is None or len(extent) != 4:
        extent = MAP_HEATMAP_EXTENT
    path = _basemap_cache_path(out_root, world_stem, clip_z)
    meta = path + ".extent.json"
    if os.path.isfile(path) and os.path.isfile(meta):
        try:
            with open(meta, "r") as f:
                old = (json.load(f) or {}).get("extent") or []
            if [float(x) for x in old] == [float(x) for x in extent]:
                return path
        except Exception:
            pass
    _ensure_dir(os.path.dirname(path))
    mode = "schematic"
    render_schematic_basemap(
        path,
        extent,
        title="%s top-down (clip_z=%.1fm, %s)" % (world_stem, float(clip_z), mode),
        clip_z=clip_z,
    )
    with open(meta, "w") as f:
        json.dump(
            {
                "extent": list(extent),
                "clip_z": float(clip_z),
                "mode": mode,
                "world": world_stem,
            },
            f,
        )
    if log_fn:
        try:
            log_fn("[viz_topdown] basemap → %s (%s)" % (path, mode))
        except TypeError:
            log_fn("[viz_topdown] basemap → %s (%s)", path, mode)
    return path


def _draw_start_goal(ax, start, goal, zorder=6):
    """Mark shared start (S) and goal (G) for a fixed benchmark pair."""
    if start and len(start) >= 2:
        ax.scatter(
            [float(start[0])],
            [float(start[1])],
            c="#111111",
            s=120,
            marker="^",
            zorder=zorder,
            edgecolors="white",
            linewidths=1.0,
            label="_nolegend_",
        )
        ax.annotate(
            "S",
            (float(start[0]), float(start[1])),
            textcoords="offset points",
            xytext=(6, 6),
            fontsize=11,
            fontweight="bold",
            color="#111111",
            zorder=zorder + 1,
        )
    if goal and len(goal) >= 2:
        ax.scatter(
            [float(goal[0])],
            [float(goal[1])],
            c="#c0392b",
            s=140,
            marker="*",
            zorder=zorder,
            edgecolors="white",
            linewidths=0.8,
            label="_nolegend_",
        )
        ax.annotate(
            "G",
            (float(goal[0]), float(goal[1])),
            textcoords="offset points",
            xytext=(6, 6),
            fontsize=11,
            fontweight="bold",
            color="#c0392b",
            zorder=zorder + 1,
        )


def render_traj_on_basemap(
    trajs,
    basemap_png,
    out_png,
    extent=None,
    opacity=0.5,
    title=None,
    color_by="outcome",
    force_map_extent=True,
    style="green",
):
    """Composite trajectories on a fixed map frame (default full ±15 m).

    ``color_by``: ``outcome`` (legacy) or ``ep`` (same start–goal, different EP).
    ``style``:
      - ``green``: faint green basemap + pale green ground (default)
      - ``green_full``: solid green fill (no basemap texture)
      - ``transparent``: transparent PNG — axes text + traj + S/G only
    Legend is always placed **outside** to the right of the axes.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.image as mpimg
        from matplotlib.lines import Line2D
        from matplotlib import cm
    except Exception as e:
        raise RuntimeError("matplotlib required: %s" % e)

    apply_paper_style()

    style = str(style or "green").lower().strip()
    if style not in ("green", "green_full", "transparent"):
        style = "green"

    if force_map_extent or extent is None:
        extent = MAP_HEATMAP_EXTENT
    elif not extent or len(extent) != 4:
        extent = MAP_HEATMAP_EXTENT

    xmin, xmax, ymin, ymax = [float(v) for v in extent]
    # Extra width so legend sits to the right of the map, not on top of trajs
    fig, ax = plt.subplots(figsize=(10.2, 8.0))
    fig.subplots_adjust(left=0.08, right=0.72, top=0.92, bottom=0.08)

    transparent = style == "transparent"
    if transparent:
        fig.patch.set_alpha(0.0)
        ax.patch.set_alpha(0.0)
        ax.set_facecolor("none")
        grid_color = "#888888"
        spine_color = "#333333"
        label_color = "#222222"
    elif style == "green_full":
        fig.patch.set_facecolor("#ffffff")
        ax.set_facecolor("#2f6b32")  # solid green ground
        grid_color = "#a8d4a0"
        spine_color = "#1a3d1c"
        label_color = "#111111"
    else:
        # green: pale green + optional faint basemap
        fig.patch.set_facecolor("#ffffff")
        ax.set_facecolor("#e8f0d8")
        grid_color = "#b0c49a"
        spine_color = "#333333"
        label_color = "#111111"
        if basemap_png and os.path.isfile(basemap_png):
            try:
                img = mpimg.imread(basemap_png)
                ax.imshow(
                    img,
                    extent=[xmin, xmax, ymin, ymax],
                    origin="upper",
                    aspect="auto",
                    zorder=0,
                    alpha=0.45,
                )
            except Exception:
                pass

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xticks(np.arange(xmin, xmax + 0.1, 5.0))
    ax.set_yticks(np.arange(ymin, ymax + 0.1, 5.0))
    ax.grid(
        True,
        which="major",
        color=grid_color,
        linewidth=0.6,
        zorder=1,
        alpha=0.85 if not transparent else 0.55,
    )
    for sp in ax.spines.values():
        sp.set_color(spine_color)
    ax.tick_params(colors=label_color)
    ax.xaxis.label.set_color(label_color)
    ax.yaxis.label.set_color(label_color)
    ax.title.set_color(label_color)

    n = max(1, len(trajs or []))
    try:
        cmap = cm.get_cmap("viridis", max(n, 2))
    except Exception:
        cmap = cm.viridis
    handles = []
    start0 = goal0 = None
    for i, tr in enumerate(trajs or []):
        xs = [p[0] for p in tr.get("poses") or []]
        ys = [p[1] for p in tr.get("poses") or []]
        if len(xs) < 2:
            continue
        xs, ys = _smooth_xy(xs, ys, window=5)
        ep_tag = str(tr.get("ep_tag") or tr.get("tag") or ("run%d" % (i + 1)))
        oc = str(tr.get("outcome") or "done").lower()
        if color_by == "ep":
            t = float(i) / float(max(n - 1, 1))
            color = cmap(t)
            label = "%s (%s)" % (ep_tag, oc)
        else:
            color = OUTCOME_COLORS.get(oc, "#95a5a6")
            label = oc
        ax.plot(xs, ys, color=color, alpha=float(opacity), linewidth=2.2, zorder=2, label=label)
        ax.scatter(
            [xs[-1]],
            [ys[-1]],
            c=[color],
            s=42,
            zorder=3,
            edgecolors="k" if not transparent else color,
            linewidths=0.4,
        )
        if start0 is None and tr.get("start"):
            start0 = tr.get("start")
        if goal0 is None and tr.get("goal"):
            goal0 = tr.get("goal")
        handles.append(Line2D([0], [0], color=color, lw=2, label=label))

    _draw_start_goal(ax, start0, goal0)
    # Deduplicate legend labels
    seen = set()
    uniq = []
    for h in handles:
        if h.get_label() in seen:
            continue
        seen.add(h.get_label())
        uniq.append(h)
    if uniq:
        # Outside the axes, flush to the right of the whole figure
        leg = ax.legend(
            handles=uniq,
            loc="upper left",
            bbox_to_anchor=(1.02, 1.0),
            borderaxespad=0.0,
            framealpha=0.9 if not transparent else 0.0,
            fontsize=8,
            title="episodes",
        )
        if transparent and leg is not None:
            leg.get_frame().set_facecolor("none")
            leg.get_frame().set_edgecolor("none")
            for txt in leg.get_texts():
                txt.set_color(label_color)
            if leg.get_title() is not None:
                leg.get_title().set_color(label_color)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    if title:
        ax.set_title(title)
    _ensure_dir(os.path.dirname(out_png) or ".")
    save_kw = dict(dpi=180, bbox_inches="tight", pad_inches=0.15)
    if transparent:
        fig.savefig(out_png, transparent=True, **save_kw)
    else:
        fig.savefig(out_png, facecolor=fig.get_facecolor(), **save_kw)
    plt.close(fig)
    return True


def _pair_folder_name(world_stem, pair_idx):
    return "%s__p%02d" % (_world_stem(world_stem), int(pair_idx) + 1)


def collect_trajs_by_pair(out_root):
    """Scan viz_eval/ep*/trajectories.json → {(world, pair_idx): [traj+ep_tag,...]}."""
    viz_root = os.path.join(out_root, "viz_eval")
    buckets = {}
    if not os.path.isdir(viz_root):
        return buckets
    for tag in sorted(os.listdir(viz_root)):
        tj = os.path.join(viz_root, tag, "trajectories.json")
        if not os.path.isfile(tj):
            continue
        try:
            with open(tj, "r") as f:
                doc = json.load(f)
        except Exception:
            continue
        for tr in doc.get("trajs") or []:
            stem = _world_stem(tr.get("world") or "unknown")
            pi = tr.get("pair_idx")
            if pi is None:
                continue
            key = (stem, int(pi))
            item = dict(tr)
            item["ep_tag"] = tag
            item["tag"] = tag
            buckets.setdefault(key, []).append(item)
    return buckets


def rebuild_viz_by_pair(out_root, opacity=0.55, clip_z=5.5, log_fn=None, basemap_enable=False):
    """Primary deliverable: one folder per fixed start–goal; overlay different EPs.

    Layout::
      viz_by_pair/<world>__p01/
        meta.yaml
        ep0100.png
        ...
        overlay_eps_transparent.png  # transparent bg (text + traj only)

    Green schematic / Gazebo top-down photography is off by default
    (``basemap_enable=False``): those PNGs were unusable and the hide+reload
    path restarted gzserver for every world.
    """
    apply_paper_style()
    buckets = collect_trajs_by_pair(out_root)
    root = os.path.join(out_root, "viz_by_pair")
    _ensure_dir(root)
    written = []
    overlay_styles = (
        (("overlay_eps.png", "green"), ("overlay_eps_green.png", "green_full"), ("overlay_eps_transparent.png", "transparent"))
        if basemap_enable
        else (("overlay_eps_transparent.png", "transparent"),)
    )
    for (stem, pi), items in sorted(buckets.items()):
        folder = os.path.join(root, _pair_folder_name(stem, pi))
        _ensure_dir(folder)
        # Prefer earliest item's S/G as canonical
        start = items[0].get("start")
        goal = items[0].get("goal")
        try:
            import yaml

            with open(os.path.join(folder, "meta.yaml"), "w") as f:
                yaml.safe_dump(
                    {
                        "world": stem,
                        "pair_idx": int(pi),
                        "pair_id": _pair_folder_name(stem, pi),
                        "start": start,
                        "goal": goal,
                        "n_eps": len(items),
                        "ep_tags": [it.get("ep_tag") for it in items],
                        "extent": list(MAP_HEATMAP_EXTENT),
                        "overlays": [name for name, _ in overlay_styles],
                        "note": "Same start–goal; lines are different training EPs",
                    },
                    f,
                    default_flow_style=False,
                )
        except Exception:
            pass

        bmap = None
        if basemap_enable:
            bmap = ensure_topdown_basemap(
                out_root,
                stem,
                MAP_HEATMAP_EXTENT,
                clip_z=clip_z,
                env=None,
                enable=True,
                log_fn=log_fn,
            )
        # Per-EP single trajectory (same S/G folder)
        for it in items:
            tag = str(it.get("ep_tag") or "ep")
            png_one = os.path.join(folder, "%s.png" % tag)
            if basemap_enable:
                render_traj_on_basemap(
                    [it],
                    bmap,
                    png_one,
                    extent=MAP_HEATMAP_EXTENT,
                    opacity=opacity,
                    title="%s | %s | %s" % (_pair_folder_name(stem, pi), tag, it.get("outcome")),
                    color_by="outcome",
                    force_map_extent=True,
                    style="green",
                )
            else:
                render_trajectory_overlay(
                    [it],
                    png_one,
                    opacity=opacity,
                    title="%s | %s | %s" % (_pair_folder_name(stem, pi), tag, it.get("outcome")),
                    transparent=True,
                )
            written.append(png_one)

        # THE figures: all EPs for this one pair, stacked — 3 background styles
        title = "%s | same S→G | %d EPs" % (_pair_folder_name(stem, pi), len(items))
        for fname, style in overlay_styles:
            overlay = os.path.join(folder, fname)
            render_traj_on_basemap(
                items,
                bmap,
                overlay,
                extent=MAP_HEATMAP_EXTENT,
                opacity=opacity,
                title=title,
                color_by="ep",
                force_map_extent=True,
                style=style,
            )
            written.append(overlay)
        if log_fn:
            log_fn(
                "[viz_by_pair] %s n_ep=%d → %s"
                % (
                    _pair_folder_name(stem, pi),
                    len(items),
                    os.path.join(folder, "overlay_eps_transparent.png"),
                )
            )
    # Index
    try:
        import yaml

        index = {
            "n_pairs": len(buckets),
            "pairs": [
                {
                    "id": _pair_folder_name(stem, pi),
                    "world": stem,
                    "pair_idx": pi,
                    "n_eps": len(items),
                }
                for (stem, pi), items in sorted(buckets.items())
            ],
        }
        with open(os.path.join(root, "index.yaml"), "w") as f:
            yaml.safe_dump(index, f, default_flow_style=False)
    except Exception:
        pass
    return written


def save_eval_artifacts(
    out_root,
    tag,
    trajs,
    opacity=0.5,
    sr_milestone=None,
    outcome_csv=None,
    episode=None,
    t=None,
    basemap_enable=False,
    clip_z=5.5,
    extent=None,
    env=None,
    log_fn=None,
):
    """Write overlays, pie, json, top-down composites; append outcome CSV (terminal only)."""
    viz_dir = os.path.join(out_root, "viz_eval", tag)
    _ensure_dir(viz_dir)
    by_world = {}
    for tr in trajs:
        by_world.setdefault(tr.get("world") or "unknown", []).append(tr)

    # meta for this snapshot
    try:
        import yaml

        meta = {
            "tag": tag,
            "episode": episode,
            "t": t,
            "n_trajs": len(trajs or []),
            "max_ep_steps": getattr(env, "max_steps", None) if env is not None else None,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        with open(os.path.join(viz_dir, "meta.yaml"), "w") as f:
            yaml.safe_dump(meta, f, default_flow_style=False)
    except Exception:
        pass

    for stem, group in by_world.items():
        for i, tr in enumerate(group):
            pair_idx = tr.get("pair_idx")
            if pair_idx is None:
                pair_idx = i
            png_one = os.path.join(viz_dir, "traj_%s_p%02d.png" % (stem, int(pair_idx) + 1))
            render_trajectory_overlay(
                [tr],
                png_one,
                opacity=opacity,
                title=None,
                transparent=True,
            )
            # Per-pair map shot for this EP only (never mix different S/G on one map)
            if basemap_enable:
                try:
                    bmap = ensure_topdown_basemap(
                        out_root,
                        stem,
                        MAP_HEATMAP_EXTENT,
                        clip_z=clip_z,
                        env=env,
                        enable=True,
                        log_fn=log_fn,
                    )
                    if bmap:
                        render_traj_on_basemap(
                            [tr],
                            bmap,
                            os.path.join(
                                viz_dir,
                                "traj_on_map_%s_p%02d.png" % (stem, int(pair_idx) + 1),
                            ),
                            extent=MAP_HEATMAP_EXTENT,
                            opacity=opacity,
                            title="%s | %s p%02d" % (tag, stem, int(pair_idx) + 1),
                            color_by="outcome",
                            force_map_extent=True,
                        )
                except Exception as e:
                    if log_fn:
                        log_fn("[viz_topdown] pair composite failed %s p%d: %s", stem, pair_idx, e)
        # DEPRECATED mixed overlay (different S/G on one canvas) — keep only as
        # traj_*_all_MIXED.png so it is obvious this is NOT the primary figure.
        png = os.path.join(viz_dir, "traj_%s_all_MIXED.png" % stem)
        render_trajectory_overlay(
            group,
            png,
            opacity=opacity,
            title="%s | %s | MIXED pairs (do not use)" % (tag, stem),
            transparent=True,
        )
    if by_world:
        render_trajectory_grid(
            by_world,
            os.path.join(viz_dir, "traj_grid.png"),
            opacity=opacity,
            transparent=True,
        )
    render_failure_pie(
        [tr.get("outcome") for tr in trajs],
        os.path.join(viz_dir, "failure_pie.png"),
        title="Eval outcomes (%s)" % tag,
    )
    with open(os.path.join(viz_dir, "trajectories.json"), "w") as f:
        json.dump({"tag": tag, "sr_milestone": sr_milestone, "trajs": trajs}, f)

    # Primary layout: folder per fixed start–goal, overlay across EPs
    try:
        rebuild_viz_by_pair(out_root, opacity=opacity, clip_z=clip_z, log_fn=log_fn)
    except Exception as e:
        if log_fn:
            log_fn("[viz_by_pair] rebuild failed: %s", e)

    if outcome_csv:
        for tr in trajs:
            poses = tr.get("poses") or []
            if poses:
                x, y = poses[-1][0], poses[-1][1]
            else:
                x, y = (tr.get("start") or [0, 0])[0], (tr.get("start") or [0, 0])[1]
            append_outcome_xy_csv(
                outcome_csv,
                {
                    "x": "%.4f" % float(x),
                    "y": "%.4f" % float(y),
                    "outcome": tr.get("outcome"),
                    "world": tr.get("world"),
                    "episode": episode if episode is not None else "",
                    "sr_milestone": "" if sr_milestone is None else "%.3f" % float(sr_milestone),
                    "source": "eval_%s" % tag,
                    "path_efficiency": tr.get("path_efficiency", ""),
                    "t": t if t is not None else "",
                    "point_kind": "terminal",
                },
            )
            g = tr.get("goal")
            if g and len(g) >= 2:
                append_outcome_xy_csv(
                    outcome_csv,
                    {
                        "x": "%.4f" % float(g[0]),
                        "y": "%.4f" % float(g[1]),
                        "outcome": tr.get("outcome"),
                        "world": tr.get("world"),
                        "episode": episode if episode is not None else "",
                        "sr_milestone": "" if sr_milestone is None else "%.3f" % float(sr_milestone),
                        "source": "eval_goal_%s" % tag,
                        "path_efficiency": "",
                        "t": t if t is not None else "",
                        "point_kind": "goal",
                    },
                )
    return viz_dir


def render_heatmaps_for_run(
    out_root,
    milestones=None,
    include_goals=True,
    extent=None,
):
    """Per-world heatmaps under heatmaps/<world>/{terminal,goals}.png.

    Frame is always the full map square (default ±15 m), not the point AABB.
    """
    del milestones
    csv_path = os.path.join(out_root, "outcome_xy.csv")
    rows = read_outcome_xy_csv(csv_path)
    if not rows:
        return []
    heat_root = os.path.join(out_root, "heatmaps")
    _ensure_dir(heat_root)
    written = []
    frame = extent if extent is not None else MAP_HEATMAP_EXTENT

    def _kind(r):
        k = str(r.get("point_kind") or "").strip().lower()
        if k in ("terminal", "goal"):
            return k
        src = str(r.get("source") or "")
        if src.startswith("eval_path_"):
            return "path"
        if src.startswith("eval_goal_") or src == "train_goal":
            return "goal"
        return "terminal"

    by_world = {}
    for r in rows:
        kind = _kind(r)
        if kind == "path":
            continue
        w = _world_stem(r.get("world") or "unknown")
        by_world.setdefault(w, {"terminal": [], "goal": []})
        if kind == "goal":
            by_world[w]["goal"].append(r)
        else:
            by_world[w]["terminal"].append(r)

    for w, buckets in sorted(by_world.items()):
        wdir = os.path.join(heat_root, w)
        _ensure_dir(wdir)
        term = buckets["terminal"]
        goals = buckets["goal"]
        png_t = os.path.join(wdir, "terminal.png")
        if term and render_heatmap(
            term, png_t, title="Terminal pose | %s" % w, extent=frame
        ):
            written.append(png_t)
        if include_goals and goals:
            png_g = os.path.join(wdir, "goals.png")
            if render_heatmap(
                goals, png_g, title="Goal location | %s" % w, extent=frame
            ):
                written.append(png_g)

    return written


def maybe_run_milestone_eval(
    *,
    controller,
    policy,
    env,
    pairs_doc,
    out_root,
    success_rate_win,
    episode_num,
    t,
    max_steps,
    opacity,
    log_fn=None,
    basemap_enable=False,
    clip_z=5.5,
    gui_pause=True,
    heatmap_goals=True,
):
    """Hook used by train_td3_node. Never raises."""
    try:
        run, milestone, tag = controller.should_run(success_rate_win, episode_num)
        if not run:
            return False
        if log_fn:
            log_fn(
                "[viz_eval] trigger tag=%s ep=%d — pausing train (frozen policy)",
                tag,
                int(episode_num),
            )
        paused = False
        trajs = []
        if gui_pause and hasattr(env, "pause_gui_for_eval"):
            try:
                paused = bool(env.pause_gui_for_eval())
            except Exception as e:
                if log_fn:
                    log_fn("[viz_eval] GUI pause failed: %s", e)
                paused = False
        try:
            trajs = run_benchmark_eval(
                policy, env, pairs_doc, max_steps=max_steps, log_fn=log_fn
            )
            outcome_csv = os.path.join(out_root, "outcome_xy.csv")
            save_eval_artifacts(
                out_root,
                tag,
                trajs,
                opacity=opacity,
                sr_milestone=milestone,
                outcome_csv=outcome_csv,
                episode=episode_num,
                t=t,
                basemap_enable=basemap_enable,
                clip_z=clip_z,
                extent=extent_from_env(env),
                env=env,
                log_fn=log_fn,
            )
            render_heatmaps_for_run(out_root, include_goals=heatmap_goals)
        finally:
            if paused and hasattr(env, "restore_gui_after_eval"):
                try:
                    env.restore_gui_after_eval(True)
                except Exception as e:
                    if log_fn:
                        log_fn("[viz_eval] GUI restore failed: %s", e)
        import yaml

        state_path = os.path.join(out_root, "viz_eval_state.yaml")
        with open(state_path, "w") as f:
            yaml.safe_dump(controller.to_dict(), f, default_flow_style=False)
        if log_fn:
            log_fn(
                "[viz_eval] done tag=%s n_traj=%d → %s/viz_eval/%s",
                tag,
                len(trajs),
                out_root,
                tag,
            )
        return True
    except Exception as e:
        if log_fn:
            log_fn("[viz_eval] FAILED (train continues): %s", e)
            log_fn(traceback.format_exc())
        return False


# ---------------------------------------------------------------------------
# Smoke + CLI
# ---------------------------------------------------------------------------


def _smoke(out_dir):
    _ensure_dir(out_dir)
    rng = np.random.RandomState(0)
    trajs = []
    for wi, world in enumerate((
        "bumpy_offroad",
        "bumpy_offroad_2",
        "bumpy_offroad_3",
        "bumpy_offroad_4",
    )):
        for i in range(3):
            n = 40 + int(rng.randint(0, 30))
            ang = rng.uniform(0, 2 * math.pi)
            sx, sy = rng.uniform(-5, 5), rng.uniform(-5, 5)
            gx = sx + 4.0 * math.cos(ang)
            gy = sy + 4.0 * math.sin(ang)
            xs = np.linspace(sx, gx, n) + rng.normal(0, 0.08, size=n)
            ys = np.linspace(sy, gy, n) + rng.normal(0, 0.08, size=n)
            oc = ["success", "stuck", "timeout", "rollover"][i % 4]
            if oc != "success":
                xs = xs[: max(5, n // 2)]
                ys = ys[: max(5, n // 2)]
            trajs.append(
                {
                    "world": world,
                    "pair_idx": i,
                    "start": [float(sx), float(sy)],
                    "goal": [float(gx), float(gy)],
                    "outcome": oc,
                    "poses": [[float(x), float(y)] for x, y in zip(xs, ys)],
                    "path_efficiency": float(rng.uniform(0.2, 0.95)),
                }
            )
    tag = "ep0010"
    save_eval_artifacts(
        out_dir,
        tag,
        trajs,
        opacity=0.5,
        sr_milestone=None,
        outcome_csv=os.path.join(out_dir, "outcome_xy.csv"),
        episode=10,
        t=1000,
        basemap_enable=False,
        clip_z=5.5,
        extent=(-8.0, 8.0, -8.0, 8.0),
        env=None,
        log_fn=lambda m, *a: print(m % a if a else m),
    )
    # also write fake goal rows for goals heatmap
    for tr in trajs:
        g = tr.get("goal") or [0, 0]
        append_outcome_xy_csv(
            os.path.join(out_dir, "outcome_xy.csv"),
            {
                "x": "%.4f" % float(g[0]),
                "y": "%.4f" % float(g[1]),
                "outcome": tr.get("outcome"),
                "world": tr.get("world"),
                "episode": 10,
                "sr_milestone": "",
                "source": "train_goal",
                "path_efficiency": "",
                "t": 1000,
                "point_kind": "goal",
            },
        )
    render_heatmaps_for_run(out_dir, include_goals=True)
    # fake metrics for CDF
    mpath = os.path.join(out_dir, "metrics.csv")
    with open(mpath, "w") as f:
        f.write("episode,path_efficiency\n")
        for i in range(80):
            f.write("%d,%.4f\n" % (i, abs(rng.normal(0.55, 0.2))))
    render_path_efficiency_cdf(mpath, os.path.join(out_dir, "path_efficiency_cdf.png"))
    print("smoke OK → %s" % out_dir)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="TD3 eval visualization (overlays / heatmaps / by-pair)")
    parser.add_argument("run_dir", nargs="?", default="", help="Training run directory")
    parser.add_argument("--smoke", nargs="?", const="/tmp/viz_smoke", default=None)
    parser.add_argument(
        "--by-pair",
        action="store_true",
        help="Rebuild viz_by_pair/ from viz_eval/ep*/trajectories.json (same S/G, stack EPs)",
    )
    parser.add_argument("--milestones", default="0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9")
    args = parser.parse_args(argv)

    if args.smoke is not None:
        return _smoke(args.smoke)

    run_dir = args.run_dir
    if not run_dir:
        parser.print_help()
        return 2
    run_dir = os.path.expanduser(run_dir)

    if args.by_pair:
        written = rebuild_viz_by_pair(run_dir, log_fn=print)
        print("viz_by_pair: %d files → %s/viz_by_pair" % (len(written), run_dir))
        return 0

    ms = [float(x) for x in args.milestones.split(",") if x.strip()]
    written = render_heatmaps_for_run(run_dir, milestones=ms)
    cdf = os.path.join(run_dir, "path_efficiency_cdf.png")
    render_path_efficiency_cdf(os.path.join(run_dir, "metrics.csv"), cdf)
    # Re-render overlays from trajectories.json if present
    viz_root = os.path.join(run_dir, "viz_eval")
    if os.path.isdir(viz_root):
        for tag in sorted(os.listdir(viz_root)):
            tj = os.path.join(viz_root, tag, "trajectories.json")
            if not os.path.isfile(tj):
                continue
            with open(tj, "r") as f:
                doc = json.load(f)
            trajs = doc.get("trajs") or []
            save_eval_artifacts(run_dir, tag, trajs, opacity=0.5, sr_milestone=doc.get("sr_milestone"))
    rebuild_viz_by_pair(run_dir, log_fn=print)
    print("plot_td3_eval_viz: heatmaps=%d run=%s" % (len(written), run_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
