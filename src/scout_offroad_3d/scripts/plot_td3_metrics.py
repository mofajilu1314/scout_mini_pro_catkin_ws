#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot Stage-A TD3 training curves from metrics.csv (research-style, white bg).

Usage:
  rosrun scout_offroad_3d plot_td3_metrics.py /path/to/run_dir
  rosrun scout_offroad_3d plot_td3_metrics.py /path/to/metrics.csv
  python3 plot_td3_metrics.py ~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS

Writes PNGs into the same run directory. New-format CSV adds outcome / window
rates and per-episode reward component sums (r_*_sum); old CSV (12 columns) is
still supported via on-the-fly recomputation.

If ``train.log`` sits next to ``metrics.csv``, episode-axis plots get dashed
vertical lines at Gazebo world-switch events. **Yellow** = successful map
change. **Gray** = failed switch / fallback restart (Record 6: every line
was fail→playpen, not a new map). Mapping: first completed episode whose
cumulative ``t >=`` the log's switch timestep (mid-episode reloads drop the
unfinished ep, so that row is the first episode after reload).
"""

from __future__ import print_function

import argparse
import csv
import os
import re
import sys

REWARD_PART_KEYS = (
    "r_episode_start",
    "r_progress",
    "r_explore",
    "r_arrive",
    "r_forward",
    "r_step",
    "r_spin",
    "r_collision",
    "r_stuck",
    "r_rollover",
    "r_timeout",
    "r_slope",
    "r_low_effort",
)

REWARD_PART_LABELS = {
    "r_episode_start": "ep_start",
    "r_progress": "progress",
    "r_explore": "explore",
    "r_arrive": "arrive",
    "r_forward": "forward",
    "r_step": "step",
    "r_spin": "spin",
    "r_collision": "collision",
    "r_stuck": "stuck",
    "r_rollover": "rollover",
    "r_timeout": "timeout",
    "r_slope": "slope",
    "r_low_effort": "low_effort",
}

# High-contrast palette + fixed linestyle per component (curves + stacked area).
REWARD_PART_COLORS = {
    "r_episode_start": "#0066CC",   # blue
    "r_progress": "#00AA00",        # green
    "r_explore": "#FF69B4",         # hot pink
    "r_arrive": "#9933CC",          # purple
    "r_forward": "#00CCCC",         # cyan
    "r_step": "#888888",            # gray
    "r_spin": "#FF8800",            # orange
    "r_collision": "#CC0000",       # red
    "r_stuck": "#FFD700",           # gold
    "r_rollover": "#8B0000",        # dark red
    "r_timeout": "#4B0082",         # indigo
    "r_slope": "#8B4513",           # brown
    "r_low_effort": "#DA70D6",      # orchid / pink-purple
}

REWARD_PART_LINESTYLES = {
    "r_episode_start": "-",
    "r_progress": "-",
    "r_explore": ":",
    "r_arrive": "-",
    "r_forward": "--",
    "r_step": "--",
    "r_spin": "--",
    "r_collision": "-",
    "r_stuck": "--",
    "r_rollover": "-.",
    "r_timeout": "-.",
    "r_slope": "--",
    "r_low_effort": "--",
}

REWARD_PART_LINEWIDTHS = {
    "r_collision": 2.4,
}

# World-switch overlay (episode-axis axvline).
WORLD_SWITCH_COLOR = "#F4D03F"       # light yellow — successful map change
WORLD_SWITCH_COLOR_ALT = "#FFB347"   # light orange when a curve is already yellow
WORLD_SWITCH_COLOR_FAIL = "#9AA0A6"  # gray — failed switch / fallback restart
WORLD_SWITCH_LS = "--"
WORLD_SWITCH_LW = 1.15
WORLD_SWITCH_ALPHA = 0.72
WORLD_SWITCH_ZORDER = 1.15
# 「换图」legend uses a CJK font only on that text item (see _legend).
WORLD_SWITCH_LABEL = "换图"
WORLD_SWITCH_FAIL_LABEL = "换图失败/回退"

# train.log lines written by train_td3_node / rospy (tolerate prefixes).
_RE_SWITCH_TO = re.compile(
    r"\[world\]\s+switch\s+to\s+(\S+?)\s+at\s+t\s*=\s*(\d+)",
    re.IGNORECASE,
)
_RE_SWITCH_COMPLETE = re.compile(
    r"\[world\]\s+switch\s+complete:?\s+(\S+?)\s+at\s+t\s*=\s*(\d+)",
    re.IGNORECASE,
)
_RE_RECOVERY_OK = re.compile(
    r"\[world\]\s+recovery\s+OK\s*(?:→|->|=>)\s*(\S+?)\s+at\s+t\s*=\s*(\d+)",
    re.IGNORECASE,
)
_RE_SWITCH_FAILED = re.compile(
    r"\[world\]\s+switch\s+FAILED\s+at\s+t\s*=\s*(\d+)",
    re.IGNORECASE,
)
_RE_RECOVERY_FAILED = re.compile(
    r"\[world\]\s+recovery\s+FAILED\s+at\s+t\s*=\s*(\d+)",
    re.IGNORECASE,
)
_RE_FALLBACK = re.compile(
    r"\[world\]\s+fallback\s+to\s+(\S+?)\s+at\s+t\s*=\s*(\d+)",
    re.IGNORECASE,
)
_RE_WORLD_FILE = re.compile(r"([A-Za-z0-9_.\-]+\.world)")


def _resolve_csv(path):
    path = os.path.expanduser(path)
    if os.path.isdir(path):
        cand = os.path.join(path, "metrics.csv")
        if not os.path.isfile(cand):
            raise SystemExit("metrics.csv not found under %s" % path)
        return cand, path
    if os.path.isfile(path):
        return path, os.path.dirname(os.path.abspath(path)) or "."
    raise SystemExit("path not found: %s" % path)


def _world_basename(name):
    if not name:
        return ""
    name = str(name).strip().strip("\"'").rstrip(".,;:")
    return os.path.basename(name)


def _world_short(name):
    base = _world_basename(name)
    if base.lower().endswith(".world"):
        base = base[: -len(".world")]
    return base


def _parse_world_switches(log_path):
    """Parse train.log world-switch events; collapse multiple lines at the same t.

    Recognized (order / ROS prefixes ignored):
      [world] switch complete: NAME at t=N   → kind=switch (true map change)
      [world] switch to NAME at t=N          → kind=switch *unless* same t already failed
      [world] switch FAILED at t=N: ...      → kind=failed (overrides switch-to)
      [world] recovery OK → NAME at t=N      → kind=recovery (old logs); if NAME ≠ attempted → failed
      [world] fallback to NAME at t=N ...    → kind=failed (reload previous; NOT a map change)
      [world] recovery FAILED at t=N: ...    → kind=failed

    Same-t merge: FAILED / fallback / recovery-to-other-world **override**
    ``switch to`` (ros_env logs that line at attempt *start*; trainer used to
    also write it on success). Record 6: every 20k line is fail→playpen, not
    a successful map change. Gray axvline ≠ yellow.
    Recovery/failed still count: gazebo is restarted and can cause metric dips.
    """
    if not log_path or not os.path.isfile(log_path):
        return []
    by_t = {}
    try:
        with open(log_path, "r") as f:
            lines = f.readlines()
    except OSError:
        return []

    def _ensure(t):
        t = int(t)
        ev = by_t.get(t)
        if ev is None:
            ev = {
                "t": t,
                "world_name": "",
                "attempted": "",
                "kind": "",
            }
            by_t[t] = ev
        return ev

    for raw in lines:
        line = raw.strip()
        if "[world]" not in line:
            continue
        m = _RE_SWITCH_COMPLETE.search(line)
        if m:
            ev = _ensure(m.group(2))
            name = _world_basename(m.group(1))
            if not ev["attempted"]:
                ev["attempted"] = name
            if ev["kind"] != "failed":
                ev["kind"] = "switch"
                ev["world_name"] = name
            continue
        m = _RE_SWITCH_TO.search(line)
        if m:
            ev = _ensure(m.group(2))
            name = _world_basename(m.group(1))
            if not ev["attempted"]:
                ev["attempted"] = name
            # ros_env logs this at attempt START. Do not promote over failed.
            if ev["kind"] != "failed":
                ev["kind"] = "switch"
                ev["world_name"] = name
            continue
        m = _RE_RECOVERY_OK.search(line)
        if m:
            ev = _ensure(m.group(2))
            name = _world_basename(m.group(1))
            attempted = ev.get("attempted") or ""
            # Old Record 6 logs: recovery OK → playpen after bumpy FAILED.
            # That is a fallback, not a successful switch.
            if attempted and _world_basename(attempted) != name:
                ev["kind"] = "failed"
                ev["world_name"] = name
            else:
                ev["kind"] = "failed" if ev["kind"] == "switch" else "recovery"
                ev["world_name"] = name
            continue
        m = _RE_FALLBACK.search(line)
        if m:
            ev = _ensure(m.group(2))
            name = _world_basename(m.group(1))
            ev["world_name"] = name
            ev["kind"] = "failed"
            continue
        m = _RE_RECOVERY_FAILED.search(line)
        if m:
            ev = _ensure(m.group(1))
            worlds = _RE_WORLD_FILE.findall(line)
            if worlds and not ev["attempted"]:
                ev["attempted"] = _world_basename(worlds[0])
            ev["kind"] = "failed"
            continue
        m = _RE_SWITCH_FAILED.search(line)
        if m:
            ev = _ensure(m.group(1))
            worlds = _RE_WORLD_FILE.findall(line)
            # "attempts (target.world)" is the intended map; later world= is current.
            if worlds and not ev["attempted"]:
                ev["attempted"] = _world_basename(worlds[0])
            ev["kind"] = "failed"
            continue

    events = [by_t[k] for k in sorted(by_t.keys())]
    for ev in events:
        if not ev["kind"]:
            ev["kind"] = "failed"
        if not ev["world_name"]:
            ev["world_name"] = ev["attempted"]
    return events


def _map_t_to_episode(ep, t_series, t_switch):
    """First completed episode whose cumulative t >= switch timestep.

    train_td3_node switches when ``t % interval == 0``, then if the episode is
    not done it drops the unfinished ep and reset()s. metrics.csv therefore has
    no row at exactly t_switch unless an episode also terminated on that step.
    The first row with t >= t_switch is the first **completed** episode after
    the reload — i.e. training on the post-switch world.
    """
    t_switch = int(t_switch)
    last_ep = None
    for e, tt in zip(ep, t_series):
        if e is None:
            continue
        last_ep = e
        if tt is None:
            continue
        if int(tt) >= t_switch:
            return int(e)
    return int(last_ep) if last_ep is not None else None


def _load_world_switches(csv_path, out_dir, ep, t_series):
    """Return list of {t, world_name, attempted, kind, episode} or []."""
    candidates = []
    if out_dir:
        candidates.append(os.path.join(out_dir, "train.log"))
    if csv_path:
        candidates.append(
            os.path.join(os.path.dirname(os.path.abspath(csv_path)), "train.log")
        )
    log_path = None
    seen = set()
    for cand in candidates:
        cand = os.path.normpath(cand)
        if cand in seen:
            continue
        seen.add(cand)
        if os.path.isfile(cand):
            log_path = cand
            break
    if log_path is None:
        return []
    events = _parse_world_switches(log_path)
    out = []
    for ev in events:
        mapped = _map_t_to_episode(ep, t_series, ev["t"])
        item = dict(ev)
        item["episode"] = mapped
        out.append(item)
    if out:
        print(
            "world switches from %s  (ep = first metrics row with t >= switch t):"
            % log_path
        )
        for ev in out:
            att = ev.get("attempted") or "-"
            w = ev.get("world_name") or "-"
            print(
                "  t=%d  ep=%s  kind=%s  world=%s  attempted=%s"
                % (ev["t"], ev.get("episode"), ev.get("kind"), w, att)
            )
    return out


def _hex_to_rgb(color):
    if not color:
        return None
    if isinstance(color, (tuple, list)) and len(color) >= 3:
        r, g, b = color[0], color[1], color[2]
        if max(r, g, b) > 1.0:
            return (r / 255.0, g / 255.0, b / 255.0)
        return (float(r), float(g), float(b))
    s = str(color).strip()
    if s.startswith("#") and len(s) >= 7:
        try:
            return (
                int(s[1:3], 16) / 255.0,
                int(s[3:5], 16) / 255.0,
                int(s[5:7], 16) / 255.0,
            )
        except ValueError:
            return None
    return None


def _is_yellowish(color):
    rgb = _hex_to_rgb(color)
    if rgb is None:
        return False
    r, g, b = rgb
    return r >= 0.78 and g >= 0.62 and b <= 0.42


def _world_switch_color(curve_colors):
    if any(_is_yellowish(c) for c in (curve_colors or ())):
        return WORLD_SWITCH_COLOR_ALT
    return WORLD_SWITCH_COLOR


def _switch_anno_text(ev):
    world = _world_short(ev.get("world_name") or "")
    attempted = _world_short(ev.get("attempted") or "")
    kind = ev.get("kind") or ""
    if kind == "switch":
        return world or WORLD_SWITCH_LABEL
    if kind == "recovery":
        if attempted and world and attempted != world:
            return "%s->%s" % (attempted, world)
        return (world or attempted or WORLD_SWITCH_LABEL) + " recov"
    if kind == "failed":
        return (attempted or world or WORLD_SWITCH_LABEL) + " fail"
    return world or attempted or WORLD_SWITCH_LABEL


def _mark_world_switches(ax, switches, curve_colors=None, annotate=False):
    """Dashed axvline behind curves.

    Yellow/orange = successful map change. Gray = failed switch / fallback
    restart (Gazebo bounced but the map usually did not change).
    """
    if not switches:
        return
    ok_color = _world_switch_color(curve_colors)
    fail_color = WORLD_SWITCH_COLOR_FAIL
    labeled_ok = False
    labeled_fail = False
    xs = []
    for ev in switches:
        ep = ev.get("episode")
        if ep is None:
            continue
        try:
            x = float(ep)
        except (TypeError, ValueError):
            continue
        xs.append((x, ev))
        is_ok = (ev.get("kind") == "switch")
        color = ok_color if is_ok else fail_color
        kw = dict(
            color=color,
            linestyle=WORLD_SWITCH_LS,
            linewidth=WORLD_SWITCH_LW,
            alpha=WORLD_SWITCH_ALPHA,
            zorder=WORLD_SWITCH_ZORDER,
        )
        if is_ok and not labeled_ok:
            kw["label"] = WORLD_SWITCH_LABEL
            labeled_ok = True
        elif (not is_ok) and not labeled_fail:
            kw["label"] = WORLD_SWITCH_FAIL_LABEL
            labeled_fail = True
        ax.axvline(x, **kw)
    if annotate and xs:
        seen_x = set()
        for x, ev in xs:
            key = int(round(x))
            if key in seen_x:
                continue
            seen_x.add(key)
            is_ok = (ev.get("kind") == "switch")
            ax.text(
                x,
                0.995,
                _switch_anno_text(ev),
                transform=ax.get_xaxis_transform(),
                fontsize=6.5,
                color=ok_color if is_ok else fail_color,
                rotation=90,
                va="top",
                ha="right",
                alpha=0.82,
                zorder=WORLD_SWITCH_ZORDER,
                clip_on=True,
            )


def _read_metrics(csv_path):
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise SystemExit("empty metrics: %s" % csv_path)
    return rows


def _col(rows, key, cast=float, fallback=None):
    out = []
    for r in rows:
        val = r.get(key, "")
        if val is None or val == "":
            if fallback is not None:
                out.append(fallback(r))
            else:
                out.append(None)
            continue
        try:
            out.append(cast(val))
        except (TypeError, ValueError):
            if fallback is not None:
                out.append(fallback(r))
            else:
                out.append(None)
    return out


def _moving_avg(xs, window):
    if window < 2:
        return list(xs)
    out = []
    buf = []
    for x in xs:
        if x is None:
            out.append(None)
            continue
        buf.append(x)
        if len(buf) > window:
            buf.pop(0)
        out.append(sum(buf) / float(len(buf)))
    return out


def _window_rate(flags, window):
    """Sliding fraction of 1s in flags (episode-level binary indicators)."""
    out = []
    buf = []
    for v in flags:
        if v is None:
            out.append(None)
            continue
        buf.append(int(v))
        if len(buf) > window:
            buf.pop(0)
        out.append(sum(buf) / float(len(buf)))
    return out


def _cum_rate(flags):
    out = []
    s = 0
    n = 0
    for v in flags:
        if v is None:
            out.append(None)
            continue
        n += 1
        s += int(v)
        out.append(s / float(n))
    return out


def _outcome_flags(rows):
    """Derive per-episode binary outcome lists; works on old and new CSV."""
    has_outcome = any(r.get("outcome") for r in rows)
    success = []
    stuck = []
    timeout = []
    rollover = []
    collision = []
    for r in rows:
        if has_outcome and r.get("outcome"):
            oc = r["outcome"].strip().lower()
            success.append(1 if oc == "success" else 0)
            stuck.append(1 if oc == "stuck" else 0)
            timeout.append(1 if oc == "timeout" else 0)
            rollover.append(1 if oc == "rollover" else 0)
            collision.append(1 if oc == "collision" else 0)
        else:
            success.append(int(float(r.get("success") or 0)))
            stuck.append(int(float(r.get("stuck") or 0)))
            timeout.append(int(float(r.get("timeout") or 0)))
            rollover.append(int(float(r.get("rollover") or 0)))
            coll_cnt = int(float(r.get("collision_count") or 0))
            coll_ev = int(float(r.get("collision_events") or 0))
            coll_flag = int(float(r.get("collision") or 0))
            collision.append(
                1 if coll_cnt > 0 or coll_ev > 0 or coll_flag > 0 else 0
            )
    return success, stuck, timeout, rollover, collision


def _collision_episode_flags(rows):
    coll = _col(rows, "collision", lambda x: int(float(x)))
    if any(v is not None for v in coll):
        return coll
    cnt = _col(rows, "collision_count", lambda x: int(float(x)))
    if any(v is not None for v in cnt):
        return [1 if (v or 0) > 0 else 0 for v in cnt]
    ev = _col(rows, "collision_events", lambda x: int(float(x)))
    return [1 if (v or 0) > 0 else 0 for v in ev]


def _collision_counts(rows):
    cnt = _col(rows, "collision_count", lambda x: int(float(x)))
    if any(v is not None for v in cnt):
        return cnt
    return _col(rows, "collision_events", lambda x: int(float(x)))


def _reward_part_series(rows, key):
    col = "%s_sum" % key
    xs = _col(rows, col)
    if any(v is not None for v in xs):
        return xs
    return None


def _has_reward_parts(rows):
    return any(
        r.get("%s_sum" % k) not in (None, "")
        for r in rows
        for k in REWARD_PART_KEYS
    )


def _binned_abs_reward_fractions(ep, part_series, bin_size):
    """Per bin: |component| / sum(|components|) for each reward part."""
    if not ep:
        return [], {}
    max_ep = max(ep)
    bins = []
    i = 0
    while i <= max_ep:
        bins.append(i)
        i += bin_size
    if bins[-1] <= max_ep:
        bins.append(max_ep + 1)

    centers = []
    fracs = {k: [] for k in REWARD_PART_KEYS}
    for b0, b1 in zip(bins[:-1], bins[1:]):
        idx = [j for j, e in enumerate(ep) if b0 <= e < b1]
        if not idx:
            continue
        totals = {}
        for k in REWARD_PART_KEYS:
            vals = [part_series[k][j] for j in idx if part_series[k][j] is not None]
            totals[k] = sum(abs(float(v)) for v in vals) if vals else 0.0
        denom = sum(totals.values())
        if denom < 1e-12:
            continue
        centers.append((b0 + b1) * 0.5)
        for k in REWARD_PART_KEYS:
            fracs[k].append(totals[k] / denom)
    return centers, fracs


def _series_or_compute(rows, key, compute_fn):
    xs = _col(rows, key)
    if any(v is not None for v in xs):
        return xs
    return compute_fn()


def _cjk_fontproperties(size=None):
    """Font that can draw 换图; do not use globally (Latin glyphs break)."""
    try:
        from matplotlib.font_manager import FontProperties
    except Exception:
        return None
    for path in (
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    ):
        if os.path.isfile(path):
            kw = {"fname": path}
            if size is not None:
                kw["size"] = size
            try:
                return FontProperties(**kw)
            except Exception:
                continue
    return None


def _legend(ax, *args, **kwargs):
    """ax.legend plus CJK font on labels that contain non-ASCII (e.g. 换图)."""
    if "frameon" not in kwargs:
        kwargs["frameon"] = False
    leg = ax.legend(*args, **kwargs)
    if leg is None:
        return None
    fp = _cjk_fontproperties()
    if fp is None:
        return leg
    try:
        for txt in leg.get_texts():
            s = txt.get_text() or ""
            if any(ord(ch) > 127 for ch in s):
                try:
                    sz = txt.get_fontsize()
                except Exception:
                    sz = None
                fp_item = _cjk_fontproperties(size=sz)
                if fp_item is not None:
                    txt.set_fontproperties(fp_item)
    except Exception:
        pass
    return leg


def _style(ax):
    ax.set_facecolor("white")
    ax.set_axisbelow(True)
    ax.grid(True, linestyle="--", alpha=0.45, color="#666666")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(colors="#222222")
    ax.xaxis.label.set_color("#222222")
    ax.yaxis.label.set_color("#222222")
    ax.title.set_color("#111111")


def _save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor="white", edgecolor="none")
    print("wrote %s" % path)


def _binned_outcome_fractions(ep, success, stuck, timeout, rollover, collision, bin_size):
    """Group episodes into bins; return bin centers and stacked fractions."""
    if not ep:
        return [], [], [], [], [], []
    max_ep = max(ep)
    bins = []
    i = 0
    while i <= max_ep:
        bins.append(i)
        i += bin_size
    if bins[-1] <= max_ep:
        bins.append(max_ep + 1)

    centers = []
    frac_s = []
    frac_k = []
    frac_t = []
    frac_r = []
    frac_c = []
    for b0, b1 in zip(bins[:-1], bins[1:]):
        idx = [j for j, e in enumerate(ep) if b0 <= e < b1]
        if not idx:
            continue
        n = float(len(idx))
        centers.append((b0 + b1) * 0.5)
        frac_s.append(sum(success[j] for j in idx) / n)
        frac_k.append(sum(stuck[j] for j in idx) / n)
        frac_t.append(sum(timeout[j] for j in idx) / n)
        frac_r.append(sum(rollover[j] for j in idx) / n)
        frac_c.append(sum(collision[j] for j in idx) / n)
    return centers, frac_s, frac_k, frac_t, frac_r, frac_c


def plot_all(csv_path, out_dir, smooth_window=20, stack_bin=50):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit(
            "matplotlib required: python3 -m pip install --user matplotlib"
        )

    rows = _read_metrics(csv_path)
    ep = _col(rows, "episode", int)
    if any(v is None for v in ep):
        ep = list(range(len(rows)))
    t_series = _col(rows, "t", int)
    switches = _load_world_switches(csv_path, out_dir, ep, t_series)

    reward = _col(rows, "reward")
    length = _col(rows, "length")
    if all(v is None for v in length):
        length = _col(rows, "ep_steps")

    success, stuck, timeout, rollover, collision_out = _outcome_flags(rows)
    collision_ep = _collision_episode_flags(rows)
    collision_cnt = _collision_counts(rows)
    path_eff = _col(rows, "path_efficiency")

    # --- reward ---
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(ep, reward, color="#1f77b4", linewidth=1.2, label="episode reward")
    _mark_world_switches(ax, switches, curve_colors=["#1f77b4"])
    ax.set_xlabel("Episode")
    ax.set_ylabel("Reward")
    ax.set_title("TD3 learning curve — episode reward")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "reward_curve.png"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(ep, reward, color="#9ecae1", linewidth=0.8, alpha=0.7, label="raw")
    sm = _moving_avg(reward, smooth_window)
    ax.plot(
        ep,
        sm,
        color="#d62728",
        linewidth=2.0,
        label="moving avg (w=%d)" % smooth_window,
    )
    _mark_world_switches(ax, switches, curve_colors=["#9ecae1", "#d62728"])
    ax.set_xlabel("Episode")
    ax.set_ylabel("Reward")
    ax.set_title("TD3 learning curve — smoothed reward")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "reward_smooth.png"))
    plt.close(fig)

    mean_reward = _series_or_compute(
        rows, "mean_reward_cum", lambda: _moving_avg(reward, smooth_window)
    )
    if any(v is not None for v in mean_reward):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(ep, mean_reward, color="#636363", linewidth=1.8, label="mean reward")
        _mark_world_switches(ax, switches, curve_colors=["#636363"])
        ax.set_xlabel("Episode")
        ax.set_ylabel("Mean reward")
        ax.set_title("TD3 — mean episode reward (cumulative / smoothed)")
        _legend(ax)
        _style(ax)
        _save(fig, os.path.join(out_dir, "mean_reward.png"))
        plt.close(fig)

    # --- success rate (primary chart: sliding window; cumulative as reference) ---
    succ_win = _series_or_compute(
        rows, "success_rate_win", lambda: _window_rate(success, smooth_window)
    )
    succ_cum = _series_or_compute(rows, "success_rate_cum", lambda: _cum_rate(success))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(
        ep,
        succ_win,
        color="#2ca02c",
        linewidth=2.0,
        label="window success rate (w=%d)" % smooth_window,
    )
    ax.plot(
        ep,
        succ_cum,
        color="#98df8a",
        linewidth=1.0,
        linestyle="--",
        alpha=0.85,
        label="cumulative (reference)",
    )
    _mark_world_switches(ax, switches, curve_colors=["#2ca02c", "#98df8a"])
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Success rate")
    ax.set_title("TD3 — success rate (sliding window, w=%d ep)" % smooth_window)
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "success_rate.png"))
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(
        ep,
        succ_win,
        color="#2ca02c",
        linewidth=1.8,
        label="window success rate (w=%d)" % smooth_window,
    )
    _mark_world_switches(ax, switches, curve_colors=["#2ca02c"])
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Success rate")
    ax.set_title("TD3 — success termination rate (sliding window)")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "outcome_success_rate.png"))
    plt.close(fig)

    # --- stuck / timeout separate curves ---
    stuck_win = _series_or_compute(
        rows, "stuck_rate_win", lambda: _window_rate(stuck, smooth_window)
    )
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(
        ep,
        stuck_win,
        color="#f5c518",
        linewidth=1.8,
        label="window stuck rate (w=%d)" % smooth_window,
    )
    _mark_world_switches(ax, switches, curve_colors=["#f5c518"])
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Stuck rate")
    ax.set_title("TD3 — stuck termination rate (sliding window)")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "outcome_stuck_rate.png"))
    plt.close(fig)

    timeout_win = _series_or_compute(
        rows, "timeout_rate_win", lambda: _window_rate(timeout, smooth_window)
    )
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(
        ep,
        timeout_win,
        color="#1f77b4",
        linewidth=1.8,
        label="window timeout rate (w=%d)" % smooth_window,
    )
    _mark_world_switches(ax, switches, curve_colors=["#1f77b4"])
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Timeout rate")
    ax.set_title("TD3 — timeout termination rate (sliding window)")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "outcome_timeout_rate.png"))
    plt.close(fig)

    # --- stacked area: outcome composition ---
    stack_outcome_colors = ["#2ca02c", "#f5c518", "#1f77b4", "#e6550d", "#fc9272"]
    centers, fs, fk, ft, fr, fc = _binned_outcome_fractions(
        ep, success, stuck, timeout, rollover, collision_out, max(1, stack_bin)
    )
    if centers:
        fig, ax = plt.subplots(figsize=(9, 4.8))
        ax.stackplot(
            centers,
            fs,
            fk,
            ft,
            fr,
            fc,
            labels=["success", "stuck", "timeout", "rollover", "collision"],
            colors=stack_outcome_colors,
            alpha=0.85,
        )
        _mark_world_switches(ax, switches, curve_colors=stack_outcome_colors)
        ax.set_ylim(0.0, 1.0)
        ax.set_xlabel("Episode (bin center)")
        ax.set_ylabel("Termination fraction")
        ax.set_title(
            "TD3 — episode termination composition (stacked, bin=%d ep)"
            % stack_bin
        )
        _legend(ax, loc="upper left")
        _style(ax)
        _save(fig, os.path.join(out_dir, "outcome_stacked_area.png"))
        plt.close(fig)

    # --- episode length ---
    if any(v is not None for v in length):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(ep, length, color="#9467bd", linewidth=1.2, label="episode length")
        sm_l = _moving_avg(length, smooth_window)
        ax.plot(
            ep,
            sm_l,
            color="#4c78a8",
            linewidth=2.0,
            label="moving avg (w=%d)" % smooth_window,
        )
        _mark_world_switches(ax, switches, curve_colors=["#9467bd", "#4c78a8"])
        ax.set_xlabel("Episode")
        ax.set_ylabel("Steps")
        ax.set_title("TD3 — episode length")
        _legend(ax)
        _style(ax)
        _save(fig, os.path.join(out_dir, "episode_length.png"))
        plt.close(fig)

    # --- collision rate (cumulative + window) ---
    coll_cum = _series_or_compute(
        rows, "collision_rate_cum", lambda: _cum_rate(collision_ep)
    )
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(
        ep,
        coll_cum,
        color="#e45756",
        linewidth=1.8,
        label="cumulative collision rate (episodes with >=1 event)",
    )
    _mark_world_switches(ax, switches, curve_colors=["#e45756"])
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Collision rate")
    ax.set_title("TD3 — cumulative collision rate")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "collision_rate.png"))
    plt.close(fig)

    coll_win = _series_or_compute(
        rows, "collision_rate_win", lambda: _window_rate(collision_ep, smooth_window)
    )
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(
        ep,
        coll_win,
        color="#e45756",
        linewidth=1.8,
        label="window collision rate (w=%d)" % smooth_window,
    )
    _mark_world_switches(ax, switches, curve_colors=["#e45756"])
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Collision rate")
    ax.set_title("TD3 — collision rate (sliding window)")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "collision_rate_window.png"))
    plt.close(fig)

    # --- collision count per episode ---
    if any(v is not None for v in collision_cnt):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.bar(ep, collision_cnt, width=max(0.6, 0.8), color="#fc9272", alpha=0.75, label="events/ep")
        sm_c = _moving_avg([float(v or 0) for v in collision_cnt], smooth_window)
        ax.plot(
            ep,
            sm_c,
            color="#a63603",
            linewidth=2.0,
            label="moving avg (w=%d)" % smooth_window,
        )
        _mark_world_switches(ax, switches, curve_colors=["#fc9272", "#a63603"])
        ax.set_xlabel("Episode")
        ax.set_ylabel("Collision events")
        ax.set_title("TD3 — collision count per episode")
        _legend(ax)
        _style(ax)
        _save(fig, os.path.join(out_dir, "collision_count.png"))
        plt.close(fig)

    # --- stuck rate cumulative (legacy name kept) ---
    stuck_cum = _series_or_compute(rows, "stuck_rate_cum", lambda: _cum_rate(stuck))
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(ep, stuck_cum, color="#f58518", linewidth=1.8, label="cumulative stuck rate")
    _mark_world_switches(ax, switches, curve_colors=["#f58518"])
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("Episode")
    ax.set_ylabel("Stuck rate")
    ax.set_title("TD3 — cumulative stuck rate")
    _legend(ax)
    _style(ax)
    _save(fig, os.path.join(out_dir, "stuck_rate.png"))
    plt.close(fig)

    # --- path efficiency ---
    if any(v is not None for v in path_eff):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(ep, path_eff, color="#17becf", linewidth=0.9, alpha=0.55, label="raw")
        sm_pe = _moving_avg(path_eff, smooth_window)
        ax.plot(
            ep,
            sm_pe,
            color="#08519c",
            linewidth=2.0,
            label="moving avg (w=%d)" % smooth_window,
        )
        _mark_world_switches(ax, switches, curve_colors=["#17becf", "#08519c"])
        ax.set_ylim(-0.02, 1.05)
        ax.set_xlabel("Episode")
        ax.set_ylabel("Path efficiency")
        ax.set_title("TD3 — path efficiency (initial_dist / path_length)")
        _legend(ax)
        _style(ax)
        _save(fig, os.path.join(out_dir, "path_efficiency.png"))
        plt.close(fig)

    # --- reward component composition (per-episode sums) ---
    if _has_reward_parts(rows):
        part_series = {}
        for k in REWARD_PART_KEYS:
            part_series[k] = _reward_part_series(rows, k)
        centers, fracs = _binned_abs_reward_fractions(
            ep, part_series, max(1, stack_bin)
        )
        if centers:
            stack_keys = [k for k in REWARD_PART_KEYS if any(fracs[k])]
            if stack_keys:
                fig, ax = plt.subplots(figsize=(10, 5.2))
                series = [fracs[k] for k in stack_keys]
                labels = [REWARD_PART_LABELS.get(k, k) for k in stack_keys]
                colors = [REWARD_PART_COLORS.get(k, "#888888") for k in stack_keys]
                ax.stackplot(centers, series, labels=labels, colors=colors, alpha=0.88)
                _mark_world_switches(ax, switches, curve_colors=colors)
                ax.set_ylim(0.0, 1.0)
                ax.set_xlabel("Episode (bin center)")
                ax.set_ylabel("|reward part| fraction")
                ax.set_title(
                    "TD3 — reward component composition (|part| / sum|parts|, bin=%d ep)"
                    % stack_bin
                )
                _legend(ax, loc="upper left", ncol=2, fontsize=8)
                _style(ax)
                _save(fig, os.path.join(out_dir, "reward_components_stacked_area.png"))
                plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 5.2))
        plotted = 0
        part_colors_used = []
        for k in REWARD_PART_KEYS:
            xs = part_series.get(k)
            if xs is None or not any(v is not None for v in xs):
                continue
            sm = _moving_avg([float(v or 0.0) for v in xs], smooth_window)
            c = REWARD_PART_COLORS.get(k, "#888888")
            part_colors_used.append(c)
            ax.plot(
                ep,
                sm,
                linewidth=REWARD_PART_LINEWIDTHS.get(k, 1.4),
                linestyle=REWARD_PART_LINESTYLES.get(k, "-"),
                color=c,
                label=REWARD_PART_LABELS.get(k, k),
            )
            plotted += 1
        if plotted:
            ax.axhline(0.0, color="#666666", linewidth=0.8, linestyle="--", alpha=0.6)
            _mark_world_switches(ax, switches, curve_colors=part_colors_used)
            ax.set_xlabel("Episode")
            ax.set_ylabel("Episode sum (smoothed)")
            ax.set_title(
                "TD3 — reward component sums (moving avg, w=%d ep)" % smooth_window
            )
            _legend(ax, loc="upper left", ncol=2, fontsize=8)
            _style(ax)
            _save(fig, os.path.join(out_dir, "reward_components_curves.png"))
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Plot TD3 metrics.csv curves")
    ap.add_argument(
        "path",
        help="run directory containing metrics.csv, or path to metrics.csv",
    )
    ap.add_argument(
        "--window",
        type=int,
        default=20,
        help="moving-average / sliding-window size (default 20)",
    )
    ap.add_argument(
        "--stack-bin",
        type=int,
        default=50,
        help="episode bin size for stacked outcome area chart (default 50)",
    )
    args = ap.parse_args()
    csv_path, out_dir = _resolve_csv(args.path)
    plot_all(
        csv_path,
        out_dir,
        smooth_window=max(1, int(args.window)),
        stack_bin=max(1, int(args.stack_bin)),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
