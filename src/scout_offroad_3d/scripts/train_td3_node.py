#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage-A online TD3 trainer for Scout Mini.

TD3 update / replay: adapted from sfujim/TD3 (MIT).
ROS episode loop: ScoutGazeboEnv (AUV/ROS DRL-style reset-step bridge).

Every world_switch_interval train timesteps (yaml; 0=off), restarts Gazebo
Classic with another worlds/*.world. Replay buffer is NOT cleared.
"""

from __future__ import print_function

import csv
import os
import subprocess
import sys
import time

import numpy as np
import rospy
import yaml

# scripts/ on path so `import td3` works when launched via rosrun/roslaunch
# (one dirname only — same as infer_td3_node.py; double dirname pointed at package root)
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from td3 import TD3, ReplayBuffer
from td3.ros_env import ScoutGazeboEnv, REWARD_PART_KEYS


def _ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def _safe_save_policy(policy, model_prefix, out_root):
    """Save checkpoint; recreate out_root if missing. Never raise."""
    _ensure_dir(out_root)
    try:
        policy.save(model_prefix)
    except Exception as e:
        rospy.logerr(
            "policy save failed (training continues): out_root=%s prefix=%s: %s",
            out_root,
            model_prefix,
            e,
        )
        return False
    rospy.loginfo("saved models → %s_{actor,critic}", model_prefix)
    return True


def _param_missing(raw):
    """True if launch/cli left an empty string instead of omitting the param."""
    if raw is None:
        return True
    if isinstance(raw, str) and not raw.strip():
        return True
    return False


def _get_param(name, default, cast=None):
    """Read private param; empty/missing → default (yaml via rosparam is authoritative)."""
    key = name if name.startswith("~") else "~%s" % name
    if not rospy.has_param(key):
        return default
    raw = rospy.get_param(key)
    if _param_missing(raw):
        return default
    if cast is None:
        return raw
    try:
        return cast(raw)
    except (TypeError, ValueError):
        rospy.logwarn("Invalid param %s=%r — using default %r", key, raw, default)
        return default


def _get_param_int(name, default):
    return int(_get_param(name, default, int))


def _get_param_float(name, default):
    return float(_get_param(name, default, float))


def _get_param_bool(name, default):
    raw = _get_param(name, default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, (int, float)):
        return bool(raw)
    if isinstance(raw, str):
        s = raw.strip().lower()
        if s in ("1", "true", "yes", "on"):
            return True
        if s in ("0", "false", "no", "off"):
            return False
    return bool(default)


def _get_param_str(name, default=""):
    raw = _get_param(name, default)
    if _param_missing(raw):
        return default
    return str(raw).strip()


def _yaml_value(v):
    if hasattr(v, "tolist"):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer)):
        return float(v) if isinstance(v, np.floating) else int(v)
    return v


def _save_run_config(out_root, env, train_cfg, workspace_root):
    """Persist effective params (yaml + launch overrides) with run metadata."""
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S")
    world_path = env.current_world or ""
    meta = {
        "started_at": started_at,
        "run_dir": out_root,
        "workspace_root": workspace_root,
        "initial_world": world_path or None,
        "world_basename": os.path.basename(world_path) if world_path else None,
        "state_dim": int(env.state_dim),
        "action_dim": int(env.action_dim),
        "max_action": env.max_action.tolist(),
        "source_yaml": "scout_offroad_3d/config/td3_train.yaml",
        "record": 4,
    }
    payload = {
        "meta": meta,
        "train": {k: _yaml_value(v) for k, v in train_cfg.items()},
        "env": env.snapshot_config(),
    }
    for fname in ("config.yaml", "params_backup.yaml"):
        path = os.path.join(out_root, fname)
        with open(path, "w") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)
    rospy.loginfo(
        "Saved effective params → %s (meta: world=%s state_dim=%d)",
        os.path.join(out_root, "config.yaml"),
        meta.get("world_basename") or "(launch default)",
        meta["state_dim"],
    )


def _resolve_workspace_root():
    """Workspace root for default 3D/runs/ output (override via ~workspace_root or CATKIN_WS)."""
    if rospy.has_param("~workspace_root"):
        root = str(rospy.get_param("~workspace_root")).strip()
        if root:
            return os.path.expanduser(os.path.expandvars(root))
    env = os.environ.get("CATKIN_WS", "").strip()
    if env:
        return os.path.expanduser(os.path.expandvars(env))
    try:
        pkg = subprocess.check_output(
            ["rospack", "find", "scout_offroad_3d"], text=True
        ).strip()
        src_dir = os.path.dirname(pkg)
        if os.path.basename(src_dir) == "src":
            return os.path.dirname(src_dir)
    except Exception:
        pass
    return os.path.expanduser("~/catkin_ws_2")


def _load_run_yaml(out_root):
    """Read an existing run's config.yaml (meta + train + env)."""
    path = os.path.join(out_root, "config.yaml")
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except Exception as e:
        rospy.logwarn("[resume] could not read %s: %s", path, e)
        return {}


def _choose_resume_horizon(max_timesteps, t, run_max=None, default_extend_to=500000):
    """Pick a max_timesteps that is strictly greater than current t.

    yaml default (200000) is often below a long run's t, which would make
    `while t < max_timesteps` exit immediately after loading the checkpoint.
    """
    max_timesteps = int(max_timesteps)
    t = int(t)
    if t < max_timesteps:
        return max_timesteps, None
    if run_max is not None:
        try:
            run_max = int(run_max)
        except (TypeError, ValueError):
            run_max = None
    if run_max is not None and run_max > t:
        return run_max, "run_config"
    chosen = max(t + 200000, int(default_extend_to), int(run_max or 0))
    return chosen, "extend"


def _restore_from_metrics(metrics_path, metrics_window):
    """Rebuild t / episode_num / _MetricsTracker from an existing metrics.csv."""
    if not os.path.isfile(metrics_path) or os.path.getsize(metrics_path) == 0:
        return None
    tracker = _MetricsTracker(window=metrics_window)
    t = 0
    episode_num = 0
    with open(metrics_path, "r") as f:
        reader = csv.reader(f)
        next(reader, None)  # header
        for row in reader:
            if not row or len(row) < 12:
                continue
            try:
                ep = int(float(str(row[0]).strip()))
                t = int(float(str(row[1]).strip()))
                reward = float(row[3])
                outcome = str(row[11]).strip()
                collision_count = int(float(str(row[7]).strip() or "0"))
            except (TypeError, ValueError):
                continue
            tracker.update(reward, outcome, collision_count)
            episode_num = ep + 1
    if t <= 0:
        return None
    return {"t": t, "episode_num": episode_num, "tracker": tracker}


def _try_load_checkpoint(policy, model_prefix):
    """Load td3_* weights if present under model_prefix."""
    actor_path = model_prefix + "_actor"
    if not os.path.isfile(actor_path):
        return False
    policy.load(model_prefix)
    return True


def _open_metrics(path):
    """Append-mode CSV; write header if new/empty."""
    new_file = (not os.path.isfile(path)) or os.path.getsize(path) == 0
    f = open(path, "a")
    writer = csv.writer(f)
    if new_file:
        header = [
            "episode",
            "t",
            "length",
            "reward",
            "success",
            "collision",
            "collision_events",
            "collision_count",
            "stuck",
            "rollover",
            "timeout",
            "outcome",
            "rho",
            "initial_goal_dist",
            "path_length",
            "path_efficiency",
            "buf",
            "succ_total",
            "success_rate_cum",
            "stuck_rate_cum",
            "rollover_rate_cum",
            "timeout_rate_cum",
            "collision_rate_cum",
            "mean_reward_cum",
            "success_rate_win",
            "stuck_rate_win",
            "rollover_rate_win",
            "timeout_rate_win",
            "collision_rate_win",
            "mean_collision_count_win",
        ]
        header.extend("%s_sum" % k for k in REWARD_PART_KEYS)
        writer.writerow(header)
        f.flush()
    return f, writer


def _episode_outcome(info):
    if info.get("success"):
        return "success"
    if info.get("rollover"):
        return "rollover"
    if info.get("stuck"):
        return "stuck"
    if info.get("timeout"):
        return "timeout"
    if info.get("collision_limit"):
        return "collision"
    return "done"


class _NoiseAdapter(object):
    """Adapt range_features / state noise from sliding-window success rate."""

    def __init__(
        self,
        enable,
        noise_min,
        noise_max,
        low_rate,
        high_rate,
        drop_thresh,
        step,
        window=50,
        range_features_param="/range_features_node/sensor_noise_std",
    ):
        self.enable = bool(enable)
        self.noise_min = float(noise_min)
        self.noise_max = float(noise_max)
        self.low_rate = float(low_rate)
        self.high_rate = float(high_rate)
        self.drop_thresh = float(drop_thresh)
        self.step = float(step)
        self.window = max(1, int(window))
        self.range_features_param = range_features_param
        self.current = self.noise_min
        self._prev_win_rate = None
        self._hist = []

    def update(self, success, env=None):
        if not self.enable:
            return self.current
        self._hist.append(1 if success else 0)
        if len(self._hist) > self.window:
            self._hist.pop(0)
        rate = float(sum(self._hist)) / float(len(self._hist))
        if rate < self.low_rate:
            self.current = min(self.noise_max, self.current + self.step)
        elif rate > self.high_rate:
            self.current = max(self.noise_min, self.current - self.step)
        elif (
            self._prev_win_rate is not None
            and self._prev_win_rate - rate > self.drop_thresh
        ):
            self.current = min(self.noise_max, self.current + self.step)
        self._prev_win_rate = rate
        self.current = float(
            max(self.noise_min, min(self.noise_max, self.current))
        )
        try:
            rospy.set_param(self.range_features_param, self.current)
        except Exception as e:
            rospy.logwarn_throttle(30.0, "noise adapt set_param failed: %s", e)
        if env is not None and hasattr(env, "set_state_noise_std"):
            env.set_state_noise_std(self.current * 0.5)
        rospy.loginfo_throttle(
            60.0,
            "[noise_adapt] success_rate_win=%.1f%% sensor_noise_std=%.4f",
            rate * 100.0,
            self.current,
        )
        return self.current


class _MetricsTracker(object):
    """Running episode stats for CSV (cumulative + sliding window)."""

    def __init__(self, window=50):
        self.window = max(1, int(window))
        self.n_episodes = 0
        self.succ_total = 0
        self.stuck_total = 0
        self.rollover_total = 0
        self.timeout_total = 0
        self.collision_ep_total = 0
        self.sum_reward = 0.0
        self._hist = []

    def update(self, reward, outcome, collision_count):
        self.n_episodes += 1
        self.sum_reward += float(reward)
        if outcome == "success":
            self.succ_total += 1
        elif outcome == "stuck":
            self.stuck_total += 1
        elif outcome == "rollover":
            self.rollover_total += 1
        elif outcome == "timeout":
            self.timeout_total += 1
        if collision_count > 0:
            self.collision_ep_total += 1
        rec = {
            "success": 1 if outcome == "success" else 0,
            "stuck": 1 if outcome == "stuck" else 0,
            "rollover": 1 if outcome == "rollover" else 0,
            "timeout": 1 if outcome == "timeout" else 0,
            "collision": 1 if collision_count > 0 else 0,
            "collision_count": int(collision_count),
            "reward": float(reward),
        }
        self._hist.append(rec)
        if len(self._hist) > self.window:
            self._hist.pop(0)
        hn = float(len(self._hist))
        n = float(self.n_episodes)
        return {
            "success_rate_cum": self.succ_total / n,
            "stuck_rate_cum": self.stuck_total / n,
            "rollover_rate_cum": self.rollover_total / n,
            "timeout_rate_cum": self.timeout_total / n,
            "collision_rate_cum": self.collision_ep_total / n,
            "mean_reward_cum": self.sum_reward / n,
            "success_rate_win": sum(r["success"] for r in self._hist) / hn,
            "stuck_rate_win": sum(r["stuck"] for r in self._hist) / hn,
            "rollover_rate_win": sum(r["rollover"] for r in self._hist) / hn,
            "timeout_rate_win": sum(r["timeout"] for r in self._hist) / hn,
            "collision_rate_win": sum(r["collision"] for r in self._hist) / hn,
            "mean_collision_count_win": sum(r["collision_count"] for r in self._hist) / hn,
        }


def _plot_metrics(out_root):
    """Call plot_td3_metrics.py; non-fatal if matplotlib missing."""
    script = os.path.join(_SCRIPTS, "plot_td3_metrics.py")
    csv_path = os.path.join(out_root, "metrics.csv")
    if not os.path.isfile(csv_path):
        rospy.logwarn("skip plots: no %s", csv_path)
        return
    cmd = [sys.executable, script, out_root]
    try:
        rospy.loginfo("Plotting metrics → %s", out_root)
        subprocess.check_call(cmd)
    except Exception as e:
        rospy.logwarn("plot_td3_metrics failed (train still OK): %s", e)


def _safe_reset(env, log_f=None):
    """reset() with spawn gate; returns state or None if this placement should skip."""
    try:
        return env.reset()
    except Exception as e:
        msg = str(e)
        rospy.logerr("[spawn] reset exception (will resample, not abort train): %s", msg)
        if log_f is not None:
            log_f.write("[spawn] reset FAILED: %s\n" % msg)
            log_f.flush()
        return None


def _reset_until_ok(env, log_f=None, t=0, label="reset"):
    """Keep resampling XY / switching maps. Never kill training for spawn."""
    rounds = 0
    while not rospy.is_shutdown():
        state = _safe_reset(env, log_f)
        if state is not None:
            return state
        rounds += 1
        rospy.logwarn(
            "[spawn] %s: pool cycle %d had no clear spawn — skip episode, "
            "retry pool, keep training (t=%d)",
            label,
            rounds,
            int(t),
        )
        if log_f is not None:
            log_f.write(
                "[spawn] skip-ep pool-cycle=%d t=%d label=%s\n"
                % (rounds, int(t), label)
            )
            log_f.flush()
        gz_ok = True
        try:
            gz_ok = env._gz_child_ok()
        except Exception:
            gz_ok = True
        if not gz_ok:
            rec = _reset_with_gazebo_recovery(env, log_f, t)
            if rec is not None:
                return rec
        time.sleep(1.0)
    return None


def _reset_with_gazebo_recovery(env, log_f=None, t=0):
    """Gazebo looks dead: switch to another world (not playpen lock), then reset."""
    previous = env.current_world or None
    world = None
    try:
        world = env.pick_next_world(exclude=previous, require_different=True)
    except Exception:
        world = None
    if not world:
        try:
            world = previous or env.pick_next_world()
        except Exception:
            world = None
    if not world:
        return None
    rospy.logwarn(
        "[spawn] Gazebo likely down — switching to %s and retrying reset",
        os.path.basename(world),
    )
    if log_f is not None:
        log_f.write(
            "[spawn] gazebo recovery switch %s at t=%d\n"
            % (os.path.basename(world), int(t))
        )
        log_f.flush()
    try:
        env.switch_world(world, timestep=int(t))
    except Exception as e:
        rospy.logerr("[spawn] gazebo recovery failed: %s — keep training", e)
        if log_f is not None:
            log_f.write("[spawn] gazebo recovery FAILED: %s\n" % e)
            log_f.flush()
        return None
    return _safe_reset(env, log_f)


def _maybe_switch_world(env, t, interval, log_f, max_timesteps=None):
    """Restart gzserver with a new world; keep replay (caller owns it).

    Spawn pose fail on the new map: reset() retries XY then pick_next_world
    to another map. On switch failure: try other worlds in pool — never abort.
    """
    if interval <= 0 or t <= 0 or (t % interval) != 0:
        return False
    if max_timesteps is not None and t >= max_timesteps:
        return False
    previous = env.current_world or None
    tried = set()
    if previous:
        tried.add(os.path.abspath(previous))
    try:
        pool_n = len(env.list_worlds())
    except Exception:
        pool_n = 4
    max_maps = max(1, min(pool_n, 4))

    for map_try in range(max_maps):
        nxt = env.pick_next_world(exclude=tried)
        if not nxt:
            try:
                nxt = env.pick_next_world()
            except Exception:
                nxt = None
        if not nxt:
            break
        tried.add(os.path.abspath(nxt))
        base = os.path.basename(nxt)
        try:
            env.switch_world(nxt, timestep=t)
            log_f.write("[world] switch to %s at t=%d\n" % (base, t))
            log_f.write("[world] switch complete: %s at t=%d\n" % (base, t))
            log_f.flush()
            return True
        except Exception as e:
            rospy.logerr(
                "[world] switch failed at t=%d map_try=%d/%d intended=%s: %s — "
                "try next world, keep training (no playpen lock)",
                t,
                map_try + 1,
                max_maps,
                base,
                e,
            )
            log_f.write(
                "[world] switch FAILED at t=%d map=%s: %s\n" % (t, base, e)
            )
            log_f.flush()
    return False


def main():
    rospy.init_node("train_td3_node")

    seed = _get_param_int("seed", 1)
    max_timesteps = _get_param_int("max_timesteps", 200000)
    start_timesteps = _get_param_int("start_timesteps", 2000)
    world_switch_interval = _get_param_int("world_switch_interval", 20000)
    expl_noise = _get_param_float("expl_noise", 0.1)
    batch_size = _get_param_int("batch_size", 256)
    discount = _get_param_float("discount", 0.99)
    tau = _get_param_float("tau", 0.005)
    policy_noise = _get_param_float("policy_noise", 0.2)
    noise_clip = _get_param_float("noise_clip", 0.5)
    policy_freq = _get_param_int("policy_freq", 2)
    save_freq = _get_param_int("save_freq", 5000)
    log_freq = _get_param_int("log_freq", 1)
    auto_plot = _get_param_bool("auto_plot", True)
    metrics_window = _get_param_int("metrics_window", 50)
    noise_adapt_enable = _get_param_bool("noise_adapt_enable", False)
    sensor_noise_min = _get_param_float("sensor_noise_min", 0.0)
    sensor_noise_max = _get_param_float("sensor_noise_max", 0.08)
    noise_adapt_window = _get_param_int("noise_adapt_window", 50)
    noise_adapt_low_rate = _get_param_float("noise_adapt_low_rate", 0.10)
    noise_adapt_high_rate = _get_param_float("noise_adapt_high_rate", 0.35)
    noise_adapt_drop_thresh = _get_param_float("noise_adapt_drop_thresh", 0.10)
    noise_adapt_step = _get_param_float("noise_adapt_step", 0.01)
    state_noise_std = _get_param_float("state_noise_std", 0.0)

    resume = _get_param_bool("resume", False)
    output_dir_raw = _get_param_str("output_dir", "")
    explicit_output_dir = bool(output_dir_raw)
    out_root = output_dir_raw
    if out_root:
        out_root = os.path.expanduser(os.path.expandvars(out_root))
    if not out_root:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        ws_root = _resolve_workspace_root()
        out_root = os.path.join(ws_root, "3D", "runs", "run_%s" % stamp)
    _ensure_dir(out_root)
    model_prefix = os.path.join(out_root, "td3")
    metrics_path = os.path.join(out_root, "metrics.csv")
    if (
        not resume
        and explicit_output_dir
        and os.path.isfile(metrics_path)
        and os.path.isfile(model_prefix + "_actor")
    ):
        resume = True
        rospy.loginfo(
            "[resume] auto-detected checkpoint in %s (metrics.csv + td3_actor)",
            out_root,
        )

    np.random.seed(seed)
    try:
        import torch

        torch.manual_seed(seed)
    except ImportError:
        rospy.logfatal(
            "PyTorch not found for this Python. Install: "
            "python3 -m pip install --user torch  (ROS Noetic uses python3.8)"
        )
        return

    env = ScoutGazeboEnv()
    state_dim = env.state_dim
    action_dim = env.action_dim
    max_action = env.max_action
    if world_switch_interval > 0:
        rospy.loginfo(
            "World switch every %d timesteps (replay kept); first world=%s",
            world_switch_interval,
            env.current_world or "(launch default)",
        )
    else:
        rospy.loginfo(
            "World switch disabled (world_switch_interval=0); locked=%s",
            env.current_world or "(launch default)",
        )

    kwargs = dict(
        state_dim=state_dim,
        action_dim=action_dim,
        max_action=max_action,
        discount=discount,
        tau=tau,
        policy_noise=policy_noise,
        noise_clip=noise_clip,
        policy_freq=policy_freq,
    )
    policy = TD3(**kwargs)
    replay = ReplayBuffer(state_dim, action_dim)

    resume_state = None
    if resume:
        if not _try_load_checkpoint(policy, model_prefix):
            rospy.logfatal(
                "[resume] resume:=true but no checkpoint at %s_* — abort",
                model_prefix,
            )
            return
        resume_state = _restore_from_metrics(metrics_path, metrics_window)
        if resume_state is None:
            rospy.logfatal(
                "[resume] resume:=true but metrics.csv missing/empty in %s — abort",
                out_root,
            )
            return
        rospy.loginfo(
            "[resume] loaded weights from %s_*; continuing t=%d ep=%d (replay buffer empty)",
            model_prefix,
            resume_state["t"],
            resume_state["episode_num"],
        )
        run_cfg = _load_run_yaml(out_root)
        train_saved = run_cfg.get("train") if isinstance(run_cfg.get("train"), dict) else {}
        run_max = train_saved.get("max_timesteps")
        new_max, why = _choose_resume_horizon(
            max_timesteps, resume_state["t"], run_max
        )
        if why:
            rospy.logwarn(
                "[resume] max_timesteps %d → %d (%s) because t=%d; "
                "pass max_timesteps:=N (N>t) to set the horizon explicitly",
                max_timesteps,
                new_max,
                why,
                resume_state["t"],
            )
        max_timesteps = new_max
        if hasattr(env, "set_episode_count"):
            env.set_episode_count(resume_state["episode_num"])

    workspace_root = _resolve_workspace_root()
    train_cfg = {
        "seed": seed,
        "max_timesteps": max_timesteps,
        "start_timesteps": start_timesteps,
        "world_switch_interval": world_switch_interval,
        "expl_noise": expl_noise,
        "batch_size": batch_size,
        "discount": discount,
        "tau": tau,
        "policy_noise": policy_noise,
        "noise_clip": noise_clip,
        "policy_freq": policy_freq,
        "save_freq": save_freq,
        "log_freq": log_freq,
        "auto_plot": auto_plot,
        "metrics_window": metrics_window,
        "noise_adapt_enable": noise_adapt_enable,
        "sensor_noise_min": sensor_noise_min,
        "sensor_noise_max": sensor_noise_max,
        "noise_adapt_window": noise_adapt_window,
        "noise_adapt_low_rate": noise_adapt_low_rate,
        "noise_adapt_high_rate": noise_adapt_high_rate,
        "noise_adapt_drop_thresh": noise_adapt_drop_thresh,
        "noise_adapt_step": noise_adapt_step,
        "state_noise_std": state_noise_std,
        "output_dir": out_root,
        "kwargs": {k: _yaml_value(v) for k, v in kwargs.items()},
    }
    if not resume:
        _save_run_config(out_root, env, train_cfg, workspace_root)
    else:
        rospy.loginfo("[resume] keeping existing config.yaml in %s", out_root)
    rospy.loginfo(
        "TD3 train: max_timesteps=%d start_timesteps=%d world_switch_interval=%d "
        "(from td3_train.yaml unless launch override)",
        max_timesteps,
        start_timesteps,
        world_switch_interval,
    )
    rospy.loginfo("TD3 train output → %s", out_root)

    log_path = os.path.join(out_root, "train.log")
    log_f = open(log_path, "a")
    metrics_f, metrics_w = _open_metrics(metrics_path)
    episode_num = resume_state["episode_num"] if resume_state else 0
    t = resume_state["t"] if resume_state else 0
    if resume and resume_state is not None:
        log_f.write(
            "[resume] t=%d ep=%d max_timesteps=%d buf=0 (replay not persisted)\n"
            % (t, episode_num, max_timesteps)
        )
        log_f.flush()
        if t >= max_timesteps:
            rospy.logfatal(
                "[resume] t=%d >= max_timesteps=%d — pass max_timesteps:=N with N>t",
                t,
                max_timesteps,
            )
            log_f.close()
            metrics_f.close()
            return

    state = _reset_until_ok(env, log_f, t, label="initial")
    if state is None:
        rospy.logerr("[spawn] ROS shutdown during initial reset — exit node")
        log_f.close()
        metrics_f.close()
        return
    episode_reward = 0.0
    episode_timesteps = 0
    episode_collision_events = 0
    metrics_tracker = (
        resume_state["tracker"]
        if resume_state
        else _MetricsTracker(window=metrics_window)
    )
    noise_adapter = _NoiseAdapter(
        enable=noise_adapt_enable,
        noise_min=sensor_noise_min,
        noise_max=sensor_noise_max,
        low_rate=noise_adapt_low_rate,
        high_rate=noise_adapt_high_rate,
        drop_thresh=noise_adapt_drop_thresh,
        step=noise_adapt_step,
        window=noise_adapt_window,
    )
    if state_noise_std > 0.0:
        env.set_state_noise_std(state_noise_std)
    if noise_adapt_enable:
        noise_adapter.update(False, env=env)

    try:
        while t < max_timesteps and not rospy.is_shutdown():
            if t < start_timesteps:
                action = np.array(
                    [
                        np.random.uniform(0.0, env.max_v),
                        np.random.uniform(-env.max_w, env.max_w),
                    ],
                    dtype=np.float32,
                )
            else:
                action = policy.select_action(np.array(state, dtype=np.float32))
                noise = np.random.normal(0, max_action * expl_noise, size=action_dim)
                action = (action + noise).astype(np.float32)
                action[0] = np.clip(action[0], -env.max_v, env.max_v)
                action[1] = np.clip(action[1], -env.max_w, env.max_w)

            try:
                next_state, reward, done, info = env.step(action)
            except RuntimeError as e:
                if rospy.is_shutdown():
                    rospy.loginfo("step aborted during shutdown: %s", e)
                    break
                rospy.logerr(
                    "[train] step RuntimeError: %s — recover via reset, keep training",
                    e,
                )
                if log_f is not None:
                    log_f.write("[train] step FAILED: %s\n" % e)
                    log_f.flush()
                state = _reset_until_ok(env, log_f, t, label="step-recover")
                if state is None:
                    rospy.logerr("[spawn] ROS shutdown during step recovery")
                    break
                continue
            except Exception as e:
                if rospy.is_shutdown():
                    break
                rospy.logerr(
                    "[train] step exception: %s — recover via reset, keep training",
                    e,
                )
                if log_f is not None:
                    log_f.write("[train] step EXCEPTION: %s\n" % e)
                    log_f.flush()
                state = _reset_until_ok(env, log_f, t, label="step-recover")
                if state is None:
                    rospy.logerr("[spawn] ROS shutdown during step recovery")
                    break
                continue

            if info.get("episode_abort"):
                state = next_state
                rospy.loginfo(
                    "[train] episode aborted (reason=%s) — no replay/metrics/ep++",
                    info.get("reason", "?"),
                )
                continue

            episode_timesteps += 1

            # Fujimoto: bootstrap on timeout; true terminal on success/stuck/collision-limit
            if info.get("timeout"):
                done_bool = 0.0
            else:
                done_bool = float(done)

            if info.get("collision_event"):
                episode_collision_events = int(info.get("collision_events", 0))

            replay.add(state, action, next_state, reward, done_bool)
            state = next_state
            episode_reward += reward
            t += 1

            if t >= start_timesteps and replay.size >= batch_size:
                policy.train(replay, batch_size)

            switched = _maybe_switch_world(
                env, t, world_switch_interval, log_f, max_timesteps
            )

            if done:
                outcome = _episode_outcome(info)
                rp = info.get("reward_parts") or {}
                reward_detail = ""
                if rp:
                    reward_detail = (
                        " r_step=%+.3f r_prog=%+.3f r_fwd=%+.3f r_spin=%+.3f"
                        " r_arrive=%+.1f r_coll=%+.1f r_stuck=%+.1f r_roll=%+.1f"
                        " r_timeout=%+.1f coll_ev=%d"
                        % (
                            rp.get("r_step", 0.0),
                            rp.get("r_progress", 0.0),
                            rp.get("r_forward", 0.0),
                            rp.get("r_spin", 0.0),
                            rp.get("r_arrive", 0.0),
                            rp.get("r_collision", 0.0),
                            rp.get("r_stuck", 0.0),
                            rp.get("r_rollover", 0.0),
                            rp.get("r_timeout", 0.0),
                            episode_collision_events,
                        )
                    )
                line = (
                    "[ep=%d] done outcome=%s reward=%.2f steps=%d rho=%.2fm "
                    "goal_tol=%.2f t=%d buf=%d succ=%d%s"
                    % (
                        episode_num,
                        outcome,
                        episode_reward,
                        episode_timesteps,
                        info.get("rho", -1),
                        env.goal_tol,
                        t,
                        replay.size,
                        metrics_tracker.succ_total,
                        reward_detail,
                    )
                )
                if episode_num % log_freq == 0:
                    rospy.loginfo(line)
                log_f.write(line + "\n")
                log_f.flush()
                rates = metrics_tracker.update(
                    episode_reward, outcome, episode_collision_events
                )
                noise_adapter.update(outcome == "success", env=env)
                had_collision = int(episode_collision_events > 0)
                rp_ep = info.get("reward_parts_episode") or info.get("reward_parts") or {}
                row = [
                    episode_num,
                    t,
                    episode_timesteps,
                    "%.6f" % episode_reward,
                    int(bool(info.get("success"))),
                    had_collision,
                    episode_collision_events,
                    episode_collision_events,
                    int(bool(info.get("stuck"))),
                    int(bool(info.get("rollover"))),
                    int(bool(info.get("timeout"))),
                    outcome,
                    "%.4f" % float(info.get("rho", -1)),
                    "%.4f" % float(info.get("initial_goal_dist", -1)),
                    "%.4f" % float(info.get("path_length", -1)),
                    "%.4f" % float(info.get("path_efficiency", -1)),
                    replay.size,
                    metrics_tracker.succ_total,
                    "%.6f" % rates["success_rate_cum"],
                    "%.6f" % rates["stuck_rate_cum"],
                    "%.6f" % rates["rollover_rate_cum"],
                    "%.6f" % rates["timeout_rate_cum"],
                    "%.6f" % rates["collision_rate_cum"],
                    "%.6f" % rates["mean_reward_cum"],
                    "%.6f" % rates["success_rate_win"],
                    "%.6f" % rates["stuck_rate_win"],
                    "%.6f" % rates["rollover_rate_win"],
                    "%.6f" % rates["timeout_rate_win"],
                    "%.6f" % rates["collision_rate_win"],
                    "%.4f" % rates["mean_collision_count_win"],
                ]
                row.extend(
                    "%.6f" % float(rp_ep.get(k, 0.0)) for k in REWARD_PART_KEYS
                )
                metrics_w.writerow(row)
                metrics_f.flush()

                state = _reset_until_ok(env, log_f, t, label="post-episode")
                if state is None:
                    rospy.logerr("[spawn] ROS shutdown during post-episode reset")
                    break
                episode_reward = 0.0
                episode_timesteps = 0
                episode_collision_events = 0
                episode_num += 1
            elif switched:
                # Mid-episode world reload: drop unfinished ep metrics, keep replay
                state = _reset_until_ok(env, log_f, t, label="post-switch")
                if state is None:
                    rospy.logerr("[spawn] ROS shutdown during post-switch reset")
                    break
                episode_reward = 0.0
                episode_timesteps = 0
                rospy.loginfo("[world] train loop resumed after switch (t=%d)", t)

            if t % save_freq == 0 and t > 0:
                _safe_save_policy(policy, model_prefix, out_root)

        if not rospy.is_shutdown():
            rospy.loginfo(
                "Training finished. Models at %s_*  log=%s", model_prefix, log_path
            )
    finally:
        if t > 0:
            if _safe_save_policy(policy, model_prefix, out_root) and rospy.is_shutdown():
                rospy.loginfo(
                    "saved models (shutdown) → %s_{actor,critic}", model_prefix
                )
        log_f.close()
        metrics_f.close()
        if auto_plot:
            _plot_metrics(out_root)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
