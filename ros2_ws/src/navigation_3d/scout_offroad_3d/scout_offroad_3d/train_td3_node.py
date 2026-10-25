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
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
import yaml

# scripts/ on path so `import td3` works when launched via rosrun/roslaunch
# (one dirname only — same as infer_td3_node.py; double dirname pointed at package root)
from scout_offroad_3d.td3 import TD3, ReplayBuffer
from scout_offroad_3d.td3.ros_env import ScoutGazeboEnv, REWARD_PART_KEYS
from scout_offroad_3d.training_run_contract import (
    create_unique_run_directory,
    fresh_output_error,
    missing_resume_artifacts,
    resolve_workspace_root,
    resume_horizon_error,
    snapshot_dimensions,
)



_NODE = None

# The current, checked training configuration is Record 9 (150-state / 1024
# hidden units).  Keep the run snapshot aligned with the configuration and
# manuals; this is provenance metadata, not a request to resume an old run.
CURRENT_TRAINING_RECORD = 9

def _log():
    return _NODE.get_logger()

def _log_text(msg, *args):
    return msg % args if args else msg


def _log_info(msg, *args):
    text = _log_text(msg, *args)
    try:
        if rclpy.ok() and _NODE is not None:
            _log().info(text)
            return
    except Exception:
        pass
    print("[INFO] " + text, flush=True)


def _log_warn(msg, *args):
    text = _log_text(msg, *args)
    try:
        if rclpy.ok() and _NODE is not None:
            _log().warning(text)
            return
    except Exception:
        pass
    print("[WARN] " + text, flush=True)


def _log_err(msg, *args):
    text = _log_text(msg, *args)
    try:
        if rclpy.ok() and _NODE is not None:
            _log().error(text)
            return
    except Exception:
        pass
    print("[ERROR] " + text, flush=True)


def _log_fatal(msg, *args):
    text = _log_text(msg, *args)
    try:
        if rclpy.ok() and _NODE is not None:
            _log().fatal(text)
            return
    except Exception:
        pass
    print("[FATAL] " + text, flush=True)

def _log_info_throttle(period, msg, *args):
    # simple: no throttle in trainer helpers
    _log_info(msg, *args)

def _log_warn_throttle(period, msg, *args):
    _log_warn(msg, *args)

def _ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def _safe_save_policy(policy, model_prefix, out_root):
    """Save checkpoint; recreate out_root if missing. Never raise."""
    _ensure_dir(out_root)
    try:
        policy.save(model_prefix)
    except Exception as e:
        _log_err(
            "policy save failed (training continues): out_root=%s prefix=%s: %s",
            out_root,
            model_prefix,
            e,
        )
        return False
    _log_info("saved models → %s_{actor,critic}", model_prefix)
    return True


def _safe_save_landscape_weights(policy, model_prefix, out_root):
    """Save actor+critic only (GIF probe weights; no optimizers). Never raise."""
    _ensure_dir(out_root)
    try:
        import torch

        torch.save(policy.critic.state_dict(), model_prefix + "_critic")
        torch.save(policy.actor.state_dict(), model_prefix + "_actor")
    except Exception as e:
        _log_warn(
            "[landscape] weight save failed (train continues): prefix=%s: %s",
            model_prefix,
            e,
        )
        return False
    return True


def _get_node_param(name, default=None):
    key = str(name).lstrip("~/")
    if _NODE.has_parameter(key):
        val = _NODE.get_parameter(key).value
        if val is None or (isinstance(val, str) and not str(val).strip() and default is not None):
            return default
        return val
    return default

def _set_node_param(name, value):
    from rclpy.parameter import Parameter
    key = str(name).lstrip("~/")
    try:
        _NODE.set_parameters([Parameter(key, Parameter.Type.DOUBLE if isinstance(value, float) else Parameter.Type.STRING, value)])
    except Exception:
        pass

def _declare_train_params(node):
    defaults = {
        "seed": 1, "max_timesteps": 200000, "start_timesteps": 2000,
        "world_switch_interval": 20000, "expl_noise": 0.1, "batch_size": 256,
        "discount": 0.99, "tau": 0.005, "policy_noise": 0.2, "noise_clip": 0.5,
        "policy_freq": 2, "save_freq": 5000, "log_freq": 1, "auto_plot": True,
        "hidden_dim": 768,
        "actor_lr": 3e-4, "critic_lr": 3e-4,
        "anneal_enable": True, "anneal_start_ep": 100, "anneal_window_ep": 50,
        "anneal_success_lo": 0.10, "anneal_success_hi": 0.90,
        "expl_noise_max": 0.1, "expl_noise_min": 0.001,
        "actor_lr_min": 1e-4, "critic_lr_min": 1e-4,
        "metrics_window": 50, "noise_adapt_enable": False, "sensor_noise_min": 0.0,
        "sensor_noise_max": 0.08, "noise_adapt_window": 50, "noise_adapt_low_rate": 0.10,
        "noise_adapt_high_rate": 0.35, "noise_adapt_drop_thresh": 0.10,
        "noise_adapt_step": 0.01, "state_noise_std": 0.0, "resume": False,
        "output_dir": "", "max_timesteps_explicit": False,
        "viz_eval_enable": True,
        "viz_eval_every_n_ep": 100,
        "viz_eval_milestones": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9],
        "viz_eval_pairs_per_world": 3,
        "viz_eval_min_pair_dist": 20.0,
        "viz_eval_pair_batch_size": 20,
        "viz_eval_pair_batch_rounds": 5,
        "viz_eval_period_after_peak_ep": 250,
        "viz_eval_max_steps": 0,
        "viz_eval_opacity": 0.5,
        "viz_eval_log_train_xy": True,
        "viz_eval_pick_at_start": True,
        "viz_eval_gui_pause": True,
        "viz_eval_heatmap_goals": True,
        "landscape_ckpt_enable": True,
        "landscape_ckpt_every_n_ep": 20,
        "action_stats_enable": True,
        "viz_topdown_enable": False,
        "viz_topdown_clip_z": 5.5,
        "viz_topdown_cam_z": 45.0,
    }
    for k, v in defaults.items():
        if not node.has_parameter(k):
            node.declare_parameter(k, v)


def _get_param_float_list(name, default):
    raw = _get_param(name, default)
    if raw is None:
        return list(default)
    if isinstance(raw, str):
        parts = [p.strip() for p in raw.replace(";", ",").split(",") if p.strip()]
        out = []
        for p in parts:
            try:
                out.append(float(p))
            except ValueError:
                pass
        return out if out else list(default)
    if isinstance(raw, (list, tuple)):
        out = []
        for v in raw:
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                pass
        return out if out else list(default)
    try:
        return [float(raw)]
    except (TypeError, ValueError):
        return list(default)

def _param_missing(raw):
    """True if launch/cli left an empty string instead of omitting the param."""
    if raw is None:
        return True
    if isinstance(raw, str) and not raw.strip():
        return True
    return False


def _get_param(name, default, cast=None):
    """Read node param; empty/missing → default (yaml + launch overrides)."""
    key = str(name).lstrip("~/")
    if not _NODE.has_parameter(key):
        return default
    raw = _NODE.get_parameter(key).value
    if _param_missing(raw):
        return default
    if cast is None:
        return raw
    try:
        return cast(raw)
    except (TypeError, ValueError):
        _log_warn("Invalid param %s=%r — using default %r", key, raw, default)
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


def _write_bootstrap_config(out_root, train_cfg, workspace_root):
    """Minimal config.yaml before ScoutGazeboEnv init (proves run dir is live)."""
    payload = {
        "meta": {
            "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "run_dir": out_root,
            "workspace_root": workspace_root,
            "status": "initializing",
            "record": CURRENT_TRAINING_RECORD,
        },
        "train": {k: _yaml_value(v) for k, v in train_cfg.items()},
        "env": {},
    }
    path = os.path.join(out_root, "config.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)


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
        "record": CURRENT_TRAINING_RECORD,
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
    _log_info(
        "Saved effective params → %s (meta: world=%s state_dim=%d)",
        os.path.join(out_root, "config.yaml"),
        meta.get("world_basename") or "(launch default)",
        meta["state_dim"],
    )


def _resolve_workspace_root():
    """Resolve the workspace root using the training-run contract order."""
    package_share_dir = None
    try:
        package_share_dir = get_package_share_directory("scout_offroad_3d")
    except Exception:
        pass
    return resolve_workspace_root(package_share_dir=package_share_dir)


def _load_run_yaml(out_root):
    """Read the immutable parameter snapshot required for continuation."""
    path = os.path.join(out_root, "params_backup.yaml")
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        if isinstance(data, dict):
            return data
    except Exception as e:
        _log_warn("[resume] could not read %s: %s", path, e)
    return {}


RESUME_WARMUP_RANDOM = 4000
RESUME_WARMUP_POLICY = 4000
_SNAPSHOT_SKIP_KEYS = frozenset(
    {
        "robot_description",
        "current_world",
        "gazebo_gui",
        "kwargs",
        "output_dir",
        "resume",
        "max_timesteps",
        "max_timesteps_explicit",
        "workspace_root",
        "world_spawn_z",
        "train_worlds",
        "viz_eval_pick_at_start",
    }
)
_SNAPSHOT_KEY_MAP = {
    "max_steps": "max_ep_steps",
    "cmd_topic": "cmd_vel_topic",
}


def _as_plain(value):
    if isinstance(value, np.generic):
        return value.item()
    return value


def _value_to_parameter(name, value):
    """Build a ROS2 Parameter; skip nested dicts / unknown types."""
    if value is None or name in _SNAPSHOT_SKIP_KEYS:
        return None
    value = _as_plain(value)
    if isinstance(value, dict):
        return None
    if _NODE is not None and _NODE.has_parameter(name):
        existing = _NODE.get_parameter(name)
        coerced = _coerce_param_value(name, existing.type_, value)
        if coerced is not None:
            return coerced
    if isinstance(value, bool):
        ptype, val = Parameter.Type.BOOL, value
    elif isinstance(value, int):
        ptype, val = Parameter.Type.INTEGER, int(value)
    elif isinstance(value, float):
        ptype, val = Parameter.Type.DOUBLE, float(value)
    elif isinstance(value, str):
        ptype, val = Parameter.Type.STRING, value
    elif isinstance(value, (list, tuple)):
        seq = [_as_plain(x) for x in value]
        if not seq:
            return None
        if all(isinstance(x, bool) for x in seq):
            ptype, val = Parameter.Type.BOOL_ARRAY, seq
        elif all(isinstance(x, int) and not isinstance(x, bool) for x in seq):
            ptype, val = Parameter.Type.INTEGER_ARRAY, [int(x) for x in seq]
        elif all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in seq):
            ptype, val = Parameter.Type.DOUBLE_ARRAY, [float(x) for x in seq]
        else:
            ptype, val = Parameter.Type.STRING_ARRAY, [str(x) for x in seq]
    else:
        return None
    try:
        return Parameter(name, ptype, val)
    except Exception:
        return None


def _coerce_param_value(name, etype, value):
    """Match an already-declared ROS param type so set_parameters does not fail."""
    try:
        if etype == Parameter.Type.BOOL:
            if isinstance(value, str):
                return Parameter(
                    name, etype, value.strip().lower() in ("1", "true", "yes", "on")
                )
            return Parameter(name, etype, bool(value))
        if etype == Parameter.Type.INTEGER:
            return Parameter(name, etype, int(value))
        if etype == Parameter.Type.DOUBLE:
            return Parameter(name, etype, float(value))
        if etype == Parameter.Type.STRING:
            return Parameter(name, etype, str(value))
        if etype == Parameter.Type.BOOL_ARRAY and isinstance(value, (list, tuple)):
            return Parameter(name, etype, [bool(x) for x in value])
        if etype == Parameter.Type.INTEGER_ARRAY and isinstance(value, (list, tuple)):
            return Parameter(name, etype, [int(x) for x in value])
        if etype == Parameter.Type.DOUBLE_ARRAY and isinstance(value, (list, tuple)):
            return Parameter(name, etype, [float(x) for x in value])
        if etype == Parameter.Type.STRING_ARRAY and isinstance(value, (list, tuple)):
            return Parameter(name, etype, [str(x) for x in value])
    except (TypeError, ValueError):
        return None
    return None


def _apply_run_snapshot_to_node(snapshot):
    """Copy saved env+train numbers onto the node. Does not change the values."""
    if not snapshot or _NODE is None:
        return 0
    applied = 0
    for section in ("env", "train"):
        block = snapshot.get(section)
        if not isinstance(block, dict):
            continue
        for raw_key, value in block.items():
            key = _SNAPSHOT_KEY_MAP.get(str(raw_key), str(raw_key))
            param = _value_to_parameter(key, value)
            if param is None:
                continue
            try:
                if not _NODE.has_parameter(key):
                    _NODE.declare_parameter(key, param.value)
                _NODE.set_parameters([param])
                applied += 1
            except Exception as e:
                _log_warn("[resume] could not set param %s: %s", key, e)
    return applied


def _nums_close(a, b, atol=1e-4):
    try:
        return abs(float(a) - float(b)) <= atol
    except (TypeError, ValueError):
        return False


def _assert_resume_snapshot(out_root):
    """Load and validate a saved snapshot before resuming its checkpoint."""
    snapshot = _load_run_yaml(out_root)
    if not snapshot:
        _log_fatal(
            "[resume] missing or unreadable params_backup.yaml in %s — abort",
            out_root,
        )
        return None
    env_saved = snapshot.get("env") if isinstance(snapshot.get("env"), dict) else {}
    try:
        dimensions = snapshot_dimensions(snapshot)
    except ValueError as exc:
        _log_fatal(
            "[resume] incompatible params_backup.yaml in %s: %s — abort",
            out_root,
            exc,
        )
        return None
    applied = _apply_run_snapshot_to_node(snapshot)
    live_arrive = _get_param_float("r_arrive", float("nan"))
    live_stuck = _get_param_float("r_stuck", float("nan"))
    live_roll = _get_param_float("r_rollover", float("nan"))
    live_hd = _get_param_int("hidden_dim", 0)
    _log_info(
        "[resume] freeze check r_arrive=%s r_stuck=%s r_rollover=%s "
        "hidden_dim=%s state_dim=%s action_dim=%s (applied %d params)",
        env_saved.get("r_arrive"),
        env_saved.get("r_stuck"),
        env_saved.get("r_rollover"),
        dimensions["hidden_dim"],
        dimensions["state_dim"],
        dimensions["action_dim"],
        applied,
    )
    mismatches = []
    for label, saved, live in (
        ("r_arrive", env_saved.get("r_arrive"), live_arrive),
        ("r_stuck", env_saved.get("r_stuck"), live_stuck),
        ("r_rollover", env_saved.get("r_rollover"), live_roll),
        ("hidden_dim", dimensions["hidden_dim"], live_hd),
    ):
        if saved is None:
            mismatches.append("%s missing in backup" % label)
        elif not _nums_close(saved, live):
            mismatches.append("%s backup=%s node=%s" % (label, saved, live))
    if mismatches:
        _log_fatal(
            "[resume] params != params_backup.yaml: %s — abort",
            "; ".join(mismatches),
        )
        return None
    return snapshot, dimensions


def _resume_pairs_path(out_root):
    viz = os.path.join(out_root, "viz_eval", "benchmark_pairs.yaml")
    root = os.path.join(out_root, "benchmark_pairs.yaml")
    if os.path.isfile(viz):
        return viz
    if os.path.isfile(root):
        return root
    return None


def _resume_warmup_step(env, replay, state, action, log_f, label):
    """One warmup transition: add to replay, no t/metrics/train. Returns state."""
    try:
        next_state, reward, done, info = env.step(action)
    except Exception as e:
        if not rclpy.ok():
            return None
        _log_warn("[resume] warmup step failed (%s): %s — reset", label, e)
        return _reset_until_ok(env, log_f, 0, label="resume-warmup-recover")
    if info.get("episode_abort"):
        return next_state
    if info.get("timeout"):
        done_bool = 0.0
    else:
        done_bool = float(done)
    replay.add(state, action, next_state, reward, done_bool)
    if done:
        nxt = _reset_until_ok(env, log_f, 0, label="resume-warmup-ep")
        return nxt
    return next_state


def _fill_resume_replay(env, policy, replay, log_f, expl_noise, max_action):
    """4000 random + 4000 policy steps into replay. Does not advance t."""
    action_dim = int(env.action_dim)
    state = _reset_until_ok(env, log_f, 0, label="resume-warmup")
    if state is None:
        return False
    v_lo = -env.max_v if getattr(env, "allow_reverse", True) else 0.0
    _log_info("[resume] warmup random %d", RESUME_WARMUP_RANDOM)
    if log_f is not None:
        log_f.write("[resume] warmup random %d\n" % RESUME_WARMUP_RANDOM)
        log_f.flush()
    for _i in range(RESUME_WARMUP_RANDOM):
        if not rclpy.ok() or state is None:
            return False
        action = np.array(
            [
                np.random.uniform(v_lo, env.max_v),
                np.random.uniform(-env.max_w, env.max_w),
            ],
            dtype=np.float32,
        )
        state = _resume_warmup_step(
            env, replay, state, action, log_f, "random"
        )
    _log_info("[resume] warmup policy %d", RESUME_WARMUP_POLICY)
    if log_f is not None:
        log_f.write("[resume] warmup policy %d\n" % RESUME_WARMUP_POLICY)
        log_f.flush()
    for _i in range(RESUME_WARMUP_POLICY):
        if not rclpy.ok() or state is None:
            return False
        action = policy.select_action(np.array(state, dtype=np.float32))
        noise = np.random.normal(0, max_action * expl_noise, size=action_dim)
        action = (action + noise).astype(np.float32)
        action[0] = np.clip(action[0], v_lo, env.max_v)
        action[1] = np.clip(action[1], -env.max_w, env.max_w)
        state = _resume_warmup_step(
            env, replay, state, action, log_f, "policy"
        )
    return True


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
    """Append-mode CSV; write header if new/empty.

    Returns (file, writer, has_action_stats_cols). New files include mean_v/mean_w
    (episode mean of |cmd_vel|) after path_efficiency. Existing files keep their
    header; action cols only if already present (resume of a stats-enabled run).
    """
    new_file = (not os.path.isfile(path)) or os.path.getsize(path) == 0
    has_action_stats = True
    if not new_file:
        try:
            with open(path, "r") as rf:
                first = rf.readline()
            has_action_stats = "mean_v" in first.split(",")
        except Exception:
            has_action_stats = False
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
            "mean_v",
            "mean_w",
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
    return f, writer, has_action_stats


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


class _SuccessCosineAnneal(object):
    """Map recent success rate s → expl_noise and Adam lr via cosine (plan F).

    u = clip((s - s_lo)/(s_hi - s_lo), 0, 1)
    x(s) = x_min + 0.5*(x_max-x_min)*(1+cos(π u))
    s≤s_lo → x_max; s≥s_hi → x_min. Enabled only when episode_num ≥ start_ep.
    """

    def __init__(
        self,
        enable,
        start_ep,
        success_lo,
        success_hi,
        noise_max,
        noise_min,
        actor_lr_max,
        actor_lr_min,
        critic_lr_max,
        critic_lr_min,
    ):
        self.enable = bool(enable)
        self.start_ep = int(start_ep)
        self.s_lo = float(success_lo)
        self.s_hi = float(success_hi)
        self.noise_max = float(noise_max)
        self.noise_min = float(noise_min)
        self.actor_lr_max = float(actor_lr_max)
        self.actor_lr_min = float(actor_lr_min)
        self.critic_lr_max = float(critic_lr_max)
        self.critic_lr_min = float(critic_lr_min)

    @staticmethod
    def _cosine_map(s, s_lo, s_hi, x_max, x_min):
        import math

        if s_hi <= s_lo:
            return x_min if s >= s_hi else x_max
        u = (float(s) - s_lo) / (s_hi - s_lo)
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        return x_min + 0.5 * (x_max - x_min) * (1.0 + math.cos(math.pi * u))

    def apply(self, episode_num, success_rate_win, policy, expl_noise_fallback):
        if not self.enable or int(episode_num) < self.start_ep:
            return float(expl_noise_fallback), self.actor_lr_max, self.critic_lr_max
        s = float(success_rate_win)
        noise = self._cosine_map(
            s, self.s_lo, self.s_hi, self.noise_max, self.noise_min
        )
        actor_lr = self._cosine_map(
            s, self.s_lo, self.s_hi, self.actor_lr_max, self.actor_lr_min
        )
        critic_lr = self._cosine_map(
            s, self.s_lo, self.s_hi, self.critic_lr_max, self.critic_lr_min
        )
        try:
            for g in policy.actor_optimizer.param_groups:
                g["lr"] = actor_lr
            for g in policy.critic_optimizer.param_groups:
                g["lr"] = critic_lr
        except Exception:
            pass
        return float(noise), float(actor_lr), float(critic_lr)


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
            _set_node_param(self.range_features_param, self.current)
        except Exception as e:
            _log_warn_throttle(30.0, "noise adapt set_param failed: %s", e)
        if env is not None and hasattr(env, "set_state_noise_std"):
            env.set_state_noise_std(self.current * 0.5)
        _log_info_throttle(
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
    """Shutdown research plots: metrics curves, heatmaps, by-pair, landscape(+GIF).

    Briefly ignores SIGINT so a second Ctrl+C after stop does not abort
    PNG generation (weights/metrics are already saved). Subprocess uses a
    new session so terminal SIGINT does not kill the plot child either.
    """
    import signal

    script = os.path.join(os.path.dirname(__file__), "plot_td3_metrics.py")
    csv_path = os.path.join(out_root, "metrics.csv")
    prev = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        _log_info(
            "Plotting metrics → %s (wait until done; 2nd Ctrl+C is ignored here)",
            out_root,
        )
        if os.path.isfile(csv_path):
            cmd = [sys.executable, script, out_root]
            try:
                subprocess.check_call(cmd, start_new_session=True)
            except Exception as e:
                _log_warn("plot_td3_metrics failed (train still OK): %s", e)
        else:
            _log_warn("skip metrics curves: no %s", csv_path)
        # Research viz extras (heatmaps / by-pair / path-efficiency CDF) — never fatal
        try:
            from scout_offroad_3d.plot_td3_eval_viz import (
                rebuild_viz_by_pair,
                render_heatmaps_for_run,
                render_path_efficiency_cdf,
            )

            render_heatmaps_for_run(out_root, include_goals=True)
            if os.path.isfile(csv_path):
                render_path_efficiency_cdf(
                    csv_path, os.path.join(out_root, "path_efficiency_cdf.png")
                )
            rebuild_viz_by_pair(out_root, log_fn=_log_info, basemap_enable=False)
        except Exception as e:
            _log_warn("plot_td3_eval_viz extras failed (train still OK): %s", e)
        # Loss / objective landscape (static; GIF if ckpts exist) — never fatal
        try:
            from scout_offroad_3d.plot_td3_loss_landscape import (
                render_landscape_for_run,
            )

            render_landscape_for_run(out_root, gif=True, log_fn=_log_info)
        except Exception as e:
            _log_warn("plot_td3_loss_landscape failed (train still OK): %s", e)
        # Episode-mean cmd_vel figures — never fatal
        try:
            from scout_offroad_3d.plot_td3_actions import render_action_stats_for_run

            render_action_stats_for_run(out_root, log_fn=_log_info)
        except Exception as e:
            _log_warn("plot_td3_actions failed (train still OK): %s", e)
        _log_info("Plotting metrics done → %s", out_root)
    finally:
        signal.signal(signal.SIGINT, prev)


def _safe_reset(env, log_f=None):
    """reset() with spawn gate; returns state or None if this placement should skip."""
    try:
        return env.reset()
    except Exception as e:
        msg = str(e)
        _log_err("[spawn] reset exception (will resample, not abort train): %s", msg)
        if log_f is not None:
            log_f.write("[spawn] reset FAILED: %s\n" % msg)
            log_f.flush()
        return None


def _reset_until_ok(env, log_f=None, t=0, label="reset"):
    """Keep resampling XY / switching maps. Never kill training for spawn."""
    rounds = 0
    while rclpy.ok():
        state = _safe_reset(env, log_f)
        if state is not None:
            return state
        rounds += 1
        _log_warn(
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
    _log_warn(
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
        _log_err("[spawn] gazebo recovery failed: %s — keep training", e)
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
    max_maps = max(1, pool_n)  # train pool size (expected 4)

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
            _log_err(
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
    resume = _get_param_bool("resume", False)
    output_dir_raw = _get_param_str("output_dir", "")
    explicit_output_dir = bool(output_dir_raw)
    workspace_root = _resolve_workspace_root()
    if not workspace_root:
        _log_fatal(
            "Cannot resolve workspace root. Set SCOUT_WS_ROOT, CATKIN_WS, or WS_ROOT."
        )
        return

    out_root = ""
    if explicit_output_dir:
        out_root = os.path.abspath(
            os.path.expanduser(os.path.expandvars(output_dir_raw))
        )
    elif resume:
        _log_fatal("[resume] resume:=true requires an explicit output_dir:=PATH — abort")
        return
    else:
        try:
            out_root = create_unique_run_directory(
                os.path.join(workspace_root, "3D", "runs")
            )
        except OSError as exc:
            _log_fatal("Cannot create a unique TD3 run directory: %s", exc)
            return

    if not resume:
        output_error = fresh_output_error(out_root)
        if output_error:
            _log_fatal(
                "[train] refusing fresh run output_dir=%s: %s. "
                "Use an empty/new directory or omit output_dir.",
                out_root,
                output_error,
            )
            return

    model_prefix = os.path.join(out_root, "td3")
    metrics_path = os.path.join(out_root, "metrics.csv")
    resume_dimensions = None
    if resume:
        missing = missing_resume_artifacts(out_root)
        if missing:
            _log_fatal(
                "[resume] incomplete checkpoint in %s; missing/empty: %s — abort",
                out_root,
                ", ".join(missing),
            )
            return
        snapshot_check = _assert_resume_snapshot(out_root)
        if snapshot_check is None:
            return
        _snapshot, resume_dimensions = snapshot_check
        pairs_found = _resume_pairs_path(out_root)
        if pairs_found is None:
            _log_fatal(
                "[resume] missing benchmark_pairs.yaml in %s "
                "(viz_eval/ or run root) — abort (will not re-pick)",
                out_root,
            )
            return
        try:
            with open(pairs_found, "r") as f:
                pairs_doc = yaml.safe_load(f) or {}
            if not isinstance(pairs_doc, dict):
                raise ValueError("not a mapping")
        except Exception as e:
            _log_fatal(
                "[resume] cannot read %s: %s — abort (will not re-pick)",
                pairs_found,
                e,
            )
            return
        _log_info("[viz_eval] loaded benchmark pairs from %s", pairs_found)

    seed = _get_param_int("seed", 1)
    max_timesteps = _get_param_int("max_timesteps", 200000)
    max_timesteps_explicit = _get_param_bool("max_timesteps_explicit", False)
    start_timesteps = _get_param_int("start_timesteps", 2000)
    world_switch_interval = _get_param_int("world_switch_interval", 20000)
    expl_noise = _get_param_float("expl_noise", 0.1)
    batch_size = _get_param_int("batch_size", 1024)
    discount = _get_param_float("discount", 0.99)
    tau = _get_param_float("tau", 0.005)
    policy_noise = _get_param_float("policy_noise", 0.2)
    noise_clip = _get_param_float("noise_clip", 0.5)
    policy_freq = _get_param_int("policy_freq", 2)
    hidden_dim = _get_param_int("hidden_dim", 1024)
    actor_lr = _get_param_float("actor_lr", 3e-4)
    critic_lr = _get_param_float("critic_lr", 3e-4)
    anneal_enable = _get_param_bool("anneal_enable", True)
    anneal_start_ep = _get_param_int("anneal_start_ep", 100)
    anneal_window_ep = _get_param_int("anneal_window_ep", 50)
    anneal_success_lo = _get_param_float("anneal_success_lo", 0.10)
    anneal_success_hi = _get_param_float("anneal_success_hi", 0.90)
    expl_noise_max = _get_param_float("expl_noise_max", 0.1)
    expl_noise_min = _get_param_float("expl_noise_min", 0.001)
    actor_lr_min = _get_param_float("actor_lr_min", 1e-4)
    critic_lr_min = _get_param_float("critic_lr_min", 1e-4)
    slope_climb_scale_max = _get_param_float("slope_climb_scale_max", 0.25)
    slope_climb_scale_min = _get_param_float("slope_climb_scale_min", 0.05)
    slope_climb_anneal_success_hi = _get_param_float(
        "slope_climb_anneal_success_hi", 0.80
    )
    slope_climb_scale = _get_param_float("slope_climb_scale", slope_climb_scale_max)
    save_freq = _get_param_int("save_freq", 5000)
    log_freq = _get_param_int("log_freq", 1)
    auto_plot = _get_param_bool("auto_plot", True)
    # Success-rate window for metrics + anneal (plan: last-50 ep)
    metrics_window = _get_param_int("metrics_window", anneal_window_ep)
    if anneal_enable and anneal_window_ep > 0:
        metrics_window = anneal_window_ep
    resume_state = None
    if resume:
        resume_state = _restore_from_metrics(metrics_path, metrics_window)
        if resume_state is None:
            _log_fatal(
                "[resume] metrics.csv has no valid completed timestep in %s — abort",
                out_root,
            )
            return
        horizon_error = resume_horizon_error(
            max_timesteps_explicit, max_timesteps, resume_state["t"]
        )
        if horizon_error:
            _log_fatal("[resume] %s — abort", horizon_error)
            return
    noise_adapt_enable = _get_param_bool("noise_adapt_enable", False)
    sensor_noise_min = _get_param_float("sensor_noise_min", 0.0)
    sensor_noise_max = _get_param_float("sensor_noise_max", 0.08)
    noise_adapt_window = _get_param_int("noise_adapt_window", 50)
    noise_adapt_low_rate = _get_param_float("noise_adapt_low_rate", 0.10)
    noise_adapt_high_rate = _get_param_float("noise_adapt_high_rate", 0.35)
    noise_adapt_drop_thresh = _get_param_float("noise_adapt_drop_thresh", 0.10)
    noise_adapt_step = _get_param_float("noise_adapt_step", 0.01)
    state_noise_std = _get_param_float("state_noise_std", 0.0)

    viz_eval_enable = _get_param_bool("viz_eval_enable", False)
    viz_eval_milestones = _get_param_float_list(
        "viz_eval_milestones",
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9],
    )
    viz_eval_pairs_per_world = _get_param_int("viz_eval_pairs_per_world", 3)
    viz_eval_every_n_ep = _get_param_int("viz_eval_every_n_ep", 100)
    viz_eval_period_after_peak_ep = _get_param_int("viz_eval_period_after_peak_ep", 250)
    viz_eval_max_steps = _get_param_int("viz_eval_max_steps", 0)
    viz_eval_opacity = _get_param_float("viz_eval_opacity", 0.5)
    viz_eval_log_train_xy = _get_param_bool("viz_eval_log_train_xy", True)
    viz_eval_pick_at_start = _get_param_bool("viz_eval_pick_at_start", True)
    viz_eval_min_pair_dist = _get_param_float("viz_eval_min_pair_dist", 20.0)
    viz_eval_pair_batch_size = _get_param_int("viz_eval_pair_batch_size", 20)
    viz_eval_pair_batch_rounds = _get_param_int("viz_eval_pair_batch_rounds", 5)
    viz_eval_gui_pause = _get_param_bool("viz_eval_gui_pause", True)
    viz_eval_heatmap_goals = _get_param_bool("viz_eval_heatmap_goals", True)
    landscape_ckpt_enable = _get_param_bool("landscape_ckpt_enable", True)
    landscape_ckpt_every_n_ep = _get_param_int("landscape_ckpt_every_n_ep", 20)
    action_stats_enable = _get_param_bool("action_stats_enable", True)
    viz_topdown_enable = _get_param_bool("viz_topdown_enable", False)
    viz_topdown_clip_z = _get_param_float("viz_topdown_clip_z", 5.5)

    _ensure_dir(out_root)
    log_path = os.path.join(out_root, "train.log")

    # Create run artifacts immediately — never block on Gazebo service waits.
    log_f = open(log_path, "a")
    log_f.write(
        "[startup] train_td3_node pid=%d out=%s max_timesteps=%d "
        "start_timesteps=%d world_switch_interval=%d\n"
        % (
            os.getpid(),
            out_root,
            max_timesteps,
            start_timesteps,
            world_switch_interval,
        )
    )
    log_f.flush()
    _log_info(
        "Effective params: max_timesteps=%d start_timesteps=%d "
        "world_switch_interval=%d (0=lock map; switches at t=N,2N,...)",
        max_timesteps,
        start_timesteps,
        world_switch_interval,
    )
    metrics_f, metrics_w, metrics_has_action_stats = _open_metrics(metrics_path)
    if resume:
        _log_info("[resume] keeping existing config.yaml in %s (no bootstrap write)", out_root)
    else:
        bootstrap_cfg = {
            "seed": seed,
            "max_timesteps": max_timesteps,
            "start_timesteps": start_timesteps,
            "world_switch_interval": world_switch_interval,
            "output_dir": out_root,
        }
        _write_bootstrap_config(out_root, bootstrap_cfg, workspace_root)

    np.random.seed(seed)
    try:
        import torch

        torch.manual_seed(seed)
    except ImportError:
        _log_fatal(
            "PyTorch not found for this Python. Install: "
            "python3 -m pip install --user torch  (ROS Noetic uses python3.8)"
        )
        log_f.close()
        metrics_f.close()
        return

    env = ScoutGazeboEnv(node=_NODE)
    state_dim = env.state_dim
    action_dim = env.action_dim
    max_action = env.max_action
    if resume:
        expected = resume_dimensions
        mismatches = []
        for label, live, saved in (
            ("state_dim", state_dim, expected["state_dim"]),
            ("action_dim", action_dim, expected["action_dim"]),
            ("hidden_dim", hidden_dim, expected["hidden_dim"]),
        ):
            try:
                if int(live) != int(saved):
                    mismatches.append("%s live=%s snapshot=%s" % (label, live, saved))
            except (TypeError, ValueError):
                mismatches.append("%s live=%s snapshot=%s" % (label, live, saved))
        if mismatches:
            _log_fatal(
                "[resume] live TD3 dimensions do not match params_backup.yaml: %s — abort",
                "; ".join(mismatches),
            )
            log_f.close()
            metrics_f.close()
            return
        _log_info(
            "[resume] env freeze r_arrive=%s r_stuck=%s r_rollover=%s "
            "state_dim=%s action_dim=%s hidden_dim=%s",
            getattr(env, "r_arrive", None),
            getattr(env, "r_stuck", None),
            getattr(env, "r_rollover", None),
            state_dim,
            action_dim,
            hidden_dim,
        )
    if world_switch_interval > 0:
        _log_info(
            "World switch every %d timesteps (replay kept); first world=%s",
            world_switch_interval,
            env.current_world or "(launch default)",
        )
    else:
        _log_info(
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
        hidden_dim=hidden_dim,
        actor_lr=actor_lr,
        critic_lr=critic_lr,
    )
    policy = TD3(**kwargs)
    _log_info(
        "TD3 MLP hidden_dim=%d actor_lr=%.2e critic_lr=%.2e",
        hidden_dim,
        actor_lr,
        critic_lr,
    )
    replay = ReplayBuffer(state_dim, action_dim)

    if resume:
        try:
            checkpoint_loaded = _try_load_checkpoint(policy, model_prefix)
        except Exception as exc:
            _log_fatal("[resume] checkpoint compatibility/load failed: %s — abort", exc)
            log_f.close()
            metrics_f.close()
            return
        if not checkpoint_loaded:
            _log_fatal(
                "[resume] resume:=true but no checkpoint at %s_* — abort",
                model_prefix,
            )
            log_f.close()
            metrics_f.close()
            return
        _log_info(
            "[resume] loaded weights from %s_*; continuing t=%d ep=%d (replay buffer empty)",
            model_prefix,
            resume_state["t"],
            resume_state["episode_num"],
        )
        if hasattr(env, "set_episode_count"):
            env.set_episode_count(resume_state["episode_num"])

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
        "hidden_dim": hidden_dim,
        "actor_lr": actor_lr,
        "critic_lr": critic_lr,
        "anneal_enable": anneal_enable,
        "anneal_start_ep": anneal_start_ep,
        "anneal_window_ep": anneal_window_ep,
        "anneal_success_lo": anneal_success_lo,
        "anneal_success_hi": anneal_success_hi,
        "expl_noise_max": expl_noise_max,
        "expl_noise_min": expl_noise_min,
        "actor_lr_min": actor_lr_min,
        "critic_lr_min": critic_lr_min,
        "slope_climb_scale": slope_climb_scale,
        "slope_climb_scale_max": slope_climb_scale_max,
        "slope_climb_scale_min": slope_climb_scale_min,
        "slope_climb_anneal_success_hi": slope_climb_anneal_success_hi,
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
        "viz_eval_enable": viz_eval_enable,
        "viz_eval_every_n_ep": viz_eval_every_n_ep,
        "viz_eval_milestones": viz_eval_milestones,
        "viz_eval_pairs_per_world": viz_eval_pairs_per_world,
        "viz_eval_min_pair_dist": viz_eval_min_pair_dist,
        "viz_eval_pair_batch_size": viz_eval_pair_batch_size,
        "viz_eval_pair_batch_rounds": viz_eval_pair_batch_rounds,
        "viz_eval_period_after_peak_ep": viz_eval_period_after_peak_ep,
        "viz_eval_max_steps": viz_eval_max_steps,
        "viz_eval_opacity": viz_eval_opacity,
        "viz_eval_gui_pause": viz_eval_gui_pause,
        "viz_eval_heatmap_goals": viz_eval_heatmap_goals,
        "landscape_ckpt_enable": landscape_ckpt_enable,
        "landscape_ckpt_every_n_ep": landscape_ckpt_every_n_ep,
        "action_stats_enable": action_stats_enable,
        "viz_topdown_enable": viz_topdown_enable,
        "viz_topdown_clip_z": viz_topdown_clip_z,
        "kwargs": {k: _yaml_value(v) for k, v in kwargs.items()},
    }
    if not resume:
        _save_run_config(out_root, env, train_cfg, workspace_root)
    else:
        _log_info("[resume] keeping existing config.yaml in %s", out_root)
    _log_info(
        "TD3 train: max_timesteps=%d start_timesteps=%d world_switch_interval=%d "
        "(from td3_train.yaml unless launch override)",
        max_timesteps,
        start_timesteps,
        world_switch_interval,
    )
    _log_info("TD3 train output → %s", out_root)
    _log_info(
        "[auto_plot] auto_plot=%s viz_eval=%s every_n_ep=%d "
        "landscape_ckpt=%s every_n_ep=%d "
        "(ckpt=weights only, not traj eval; shutdown GIF if ckpts exist)",
        auto_plot,
        viz_eval_enable,
        viz_eval_every_n_ep,
        landscape_ckpt_enable,
        landscape_ckpt_every_n_ep,
    )
    _log_info(
        "[action_stats] enable=%s metrics_cols=%s "
        "(ep-mean |v|/|ω| → metrics.csv; no per-step CSV)",
        action_stats_enable,
        metrics_has_action_stats,
    )

    episode_num = resume_state["episode_num"] if resume_state else 0
    t = resume_state["t"] if resume_state else 0
    if resume and resume_state is not None:
        log_f.write(
            "[resume] t=%d ep=%d max_timesteps=%d buf=0 (replay not persisted)\n"
            % (t, episode_num, max_timesteps)
        )
        log_f.flush()
        if t >= max_timesteps:
            _log_fatal(
                "[resume] t=%d >= max_timesteps=%d — pass max_timesteps:=N with N>t",
                t,
                max_timesteps,
            )
            log_f.close()
            metrics_f.close()
            return
        if getattr(replay, "size", 0) == 0:
            ok = _fill_resume_replay(
                env, policy, replay, log_f, expl_noise, max_action
            )
            if not ok or not rclpy.ok():
                _log_fatal("[resume] warmup failed — abort")
                log_f.close()
                metrics_f.close()
                return
            _log_info(
                "[resume] train loop resumed t=%d ep=%d buf=%d",
                t,
                episode_num,
                int(replay.size),
            )
            log_f.write(
                "[resume] train loop resumed t=%d ep=%d buf=%d\n"
                % (t, episode_num, int(replay.size))
            )
            log_f.flush()
            state = _reset_until_ok(env, log_f, t, label="resume-after-warmup")
        else:
            state = _reset_until_ok(env, log_f, t, label="initial")
    else:
        state = _reset_until_ok(env, log_f, t, label="initial")
    if state is None:
        _log_err("[spawn] ROS shutdown during initial reset — exit node")
        log_f.close()
        metrics_f.close()
        return
    episode_reward = 0.0
    episode_timesteps = 0
    episode_collision_events = 0
    episode_sum_v = 0.0
    episode_sum_w = 0.0
    episode_action_n = 0
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
    success_anneal = _SuccessCosineAnneal(
        enable=anneal_enable,
        start_ep=anneal_start_ep,
        success_lo=anneal_success_lo,
        success_hi=anneal_success_hi,
        noise_max=expl_noise_max,
        noise_min=expl_noise_min,
        actor_lr_max=actor_lr,
        actor_lr_min=actor_lr_min,
        critic_lr_max=critic_lr,
        critic_lr_min=critic_lr_min,
    )
    _log_info(
        "[anneal] enable=%s start_ep=%d σ∈[%.4f,%.3f] lr∈[%.2e,%.2e] "
        "climb_scale∈[%.3f,%.3f] climb_hi=%.2f",
        anneal_enable,
        anneal_start_ep,
        expl_noise_min,
        expl_noise_max,
        actor_lr_min,
        actor_lr,
        slope_climb_scale_min,
        slope_climb_scale_max,
        slope_climb_anneal_success_hi,
    )
    try:
        env.set_slope_climb_scale(slope_climb_scale)
    except Exception:
        env.slope_climb_scale = float(slope_climb_scale)
    if state_noise_std > 0.0:
        env.set_state_noise_std(state_noise_std)
    if noise_adapt_enable:
        noise_adapter.update(False, env=env)

    # --- Research viz eval: fixed benchmark pairs + episode-interval controller ---
    # Imports are optional; failure never aborts training.
    viz_controller = None
    viz_pairs_doc = None
    _append_outcome_xy_csv = None
    _maybe_run_milestone_eval = None
    if viz_eval_enable or viz_eval_log_train_xy:
        try:
            from scout_offroad_3d.plot_td3_eval_viz import append_outcome_xy_csv as _append_outcome_xy_csv
        except Exception as e:
            _log_warn("[viz_eval] import append_outcome_xy failed: %s", e)
            _append_outcome_xy_csv = None
    if viz_eval_enable:
        try:
            from scout_offroad_3d.plot_td3_eval_viz import (
                EpisodeIntervalEvalController,
                load_benchmark_pairs,
                maybe_run_milestone_eval,
                pick_benchmark_pairs,
                save_benchmark_pairs,
                save_pair_gen_report,
            )

            _maybe_run_milestone_eval = maybe_run_milestone_eval
            viz_controller = EpisodeIntervalEvalController(every_n_ep=viz_eval_every_n_ep)
            viz_dir = os.path.join(out_root, "viz_eval")
            _ensure_dir(viz_dir)
            pairs_path = os.path.join(viz_dir, "benchmark_pairs.yaml")
            # Also keep a copy at run root for older tooling
            pairs_path_root = os.path.join(out_root, "benchmark_pairs.yaml")
            report_path = os.path.join(viz_dir, "pair_gen_report.yaml")
            state_path = os.path.join(out_root, "viz_eval_state.yaml")
            if resume and os.path.isfile(state_path):
                try:
                    with open(state_path, "r") as f:
                        st = yaml.safe_load(f) or {}
                    if st.get("mode") == "episode_interval" or "every_n_ep" in st:
                        viz_controller.load_dict(st)
                    _log_info("[viz_eval] restored controller from %s", state_path)
                except Exception as e:
                    _log_warn("[viz_eval] state load failed: %s", e)
            load_path = pairs_path if os.path.isfile(pairs_path) else pairs_path_root
            if os.path.isfile(load_path):
                viz_pairs_doc = load_benchmark_pairs(load_path)
                _log_info("[viz_eval] loaded benchmark pairs from %s", load_path)
            elif resume:
                _log_fatal(
                    "[resume] missing benchmark_pairs.yaml under %s — abort "
                    "(will not pre-generate / re-pick)",
                    viz_dir,
                )
                log_f.close()
                metrics_f.close()
                return
            elif viz_eval_pick_at_start:
                _log_info(
                    "[viz_eval] pre-generating ≤%d pairs × 4 worlds "
                    "(min_dist=%.1fm, batch=%d×%d)…",
                    viz_eval_pairs_per_world,
                    viz_eval_min_pair_dist,
                    viz_eval_pair_batch_rounds,
                    viz_eval_pair_batch_size,
                )
                report = {}
                viz_pairs_doc = pick_benchmark_pairs(
                    env,
                    pairs_per_world=viz_eval_pairs_per_world,
                    min_pair_dist=viz_eval_min_pair_dist,
                    batch_size=viz_eval_pair_batch_size,
                    batch_rounds=viz_eval_pair_batch_rounds,
                    log_fn=_log_info,
                    report_out=report,
                )
                save_benchmark_pairs(viz_pairs_doc, pairs_path)
                save_benchmark_pairs(viz_pairs_doc, pairs_path_root)
                save_pair_gen_report(report, report_path)
                if report.get("degraded"):
                    _log_warn(
                        "[viz_eval] pair generation degraded — see %s (train continues)",
                        report_path,
                    )
                _log_info("[viz_eval] saved %s", pairs_path)
            # Top-down Gazebo photo / green schematic basemap removed:
            # hide-canopy + world-reload produced unusable blurry green PNGs
            # and wasted several extra gzserver restarts after pair picking.
            n_pairs = 0
            for _b in (viz_pairs_doc or {}).get("worlds", {}).values():
                n_pairs += len((_b or {}).get("pairs") or [])
            _log_info(
                "[viz_eval] enabled every_n_ep=%d pairs=%d max_steps=%s "
                "(0=follow train) gui_pause=%s topdown_photo=off",
                viz_eval_every_n_ep,
                n_pairs,
                viz_eval_max_steps,
                viz_eval_gui_pause,
            )
        except Exception as e:
            if resume:
                _log_fatal("[resume] viz_eval init failed: %s — abort", e)
                log_f.close()
                metrics_f.close()
                return
            _log_warn("[viz_eval] init failed (train continues): %s", e)
            viz_controller = None
            viz_pairs_doc = None
            _maybe_run_milestone_eval = None

    try:
        while t < max_timesteps and rclpy.ok():
            if t < start_timesteps:
                v_lo = -env.max_v if getattr(env, "allow_reverse", True) else 0.0
                action = np.array(
                    [
                        np.random.uniform(v_lo, env.max_v),
                        np.random.uniform(-env.max_w, env.max_w),
                    ],
                    dtype=np.float32,
                )
            else:
                action = policy.select_action(np.array(state, dtype=np.float32))
                noise = np.random.normal(0, max_action * expl_noise, size=action_dim)
                action = (action + noise).astype(np.float32)
                v_lo = -env.max_v if getattr(env, "allow_reverse", True) else 0.0
                action[0] = np.clip(action[0], v_lo, env.max_v)
                action[1] = np.clip(action[1], -env.max_w, env.max_w)

            try:
                next_state, reward, done, info = env.step(action)
            except RuntimeError as e:
                if not rclpy.ok():
                    _log_info("step aborted during shutdown: %s", e)
                    break
                _log_err(
                    "[train] step RuntimeError: %s — recover via reset, keep training",
                    e,
                )
                if log_f is not None:
                    log_f.write("[train] step FAILED: %s\n" % e)
                    log_f.flush()
                state = _reset_until_ok(env, log_f, t, label="step-recover")
                if state is None:
                    _log_err("[spawn] ROS shutdown during step recovery")
                    break
                continue
            except Exception as e:
                if not rclpy.ok():
                    break
                import traceback
                tb = traceback.format_exc()
                _log_err(
                    "[train] step exception: %s — recover via reset, keep training",
                    e,
                )
                if log_f is not None:
                    log_f.write("[train] step EXCEPTION: %s\n" % e)
                    log_f.write(tb + "\n")
                    log_f.flush()
                state = _reset_until_ok(env, log_f, t, label="step-recover")
                if state is None:
                    _log_err("[spawn] ROS shutdown during step recovery")
                    break
                continue

            if info.get("episode_abort"):
                state = next_state
                _log_info(
                    "[train] episode aborted (reason=%s) — no replay/metrics/ep++",
                    info.get("reason", "?"),
                )
                continue

            episode_timesteps += 1

            if action_stats_enable:
                # Signed cmd_vel still goes to the robot; stats use |v|/|ω|
                # so forward/back and left/right turns do not cancel.
                episode_sum_v += abs(float(action[0]))
                episode_sum_w += abs(float(action[1]))
                episode_action_n += 1

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
                    r_high = rp.get(
                        "r_high_effort", rp.get("r_forward", 0.0)
                    )
                    reward_detail = (
                        " r_step=%+.3f r_prog=%+.3f r_high=%+.3f r_spin=%+.3f"
                        " r_arrive=%+.1f r_coll=%+.1f r_stuck=%+.1f r_roll=%+.1f"
                        " r_timeout=%+.1f coll_ev=%d"
                        % (
                            rp.get("r_step", 0.0),
                            rp.get("r_progress", 0.0),
                            r_high,
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
                    _log_info(line)
                log_f.write(line + "\n")
                log_f.flush()
                rates = metrics_tracker.update(
                    episode_reward, outcome, episode_collision_events
                )
                noise_adapter.update(outcome == "success", env=env)
                expl_noise, cur_actor_lr, cur_critic_lr = success_anneal.apply(
                    episode_num,
                    rates["success_rate_win"],
                    policy,
                    expl_noise_max if anneal_enable else expl_noise,
                )
                # Climb shaping: cosine max→min vs window SR (independent hi=0.8)
                if (not anneal_enable) or int(episode_num) < int(anneal_start_ep):
                    slope_climb_scale = float(slope_climb_scale_max)
                else:
                    slope_climb_scale = _SuccessCosineAnneal._cosine_map(
                        rates["success_rate_win"],
                        anneal_success_lo,
                        slope_climb_anneal_success_hi,
                        slope_climb_scale_max,
                        slope_climb_scale_min,
                    )
                try:
                    env.set_slope_climb_scale(slope_climb_scale)
                except Exception:
                    env.slope_climb_scale = float(slope_climb_scale)
                if anneal_enable and episode_num >= anneal_start_ep and (
                    episode_num % max(1, log_freq * 10) == 0
                ):
                    _log_info(
                        "[anneal] ep=%d s_win=%.3f expl_noise=%.4f actor_lr=%.2e "
                        "critic_lr=%.2e climb_scale=%.4f",
                        episode_num,
                        rates["success_rate_win"],
                        expl_noise,
                        cur_actor_lr,
                        cur_critic_lr,
                        slope_climb_scale,
                    )
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
                ]
                if metrics_has_action_stats:
                    if episode_action_n > 0:
                        row.extend(
                            [
                                "%.6f" % (episode_sum_v / float(episode_action_n)),
                                "%.6f" % (episode_sum_w / float(episode_action_n)),
                            ]
                        )
                    else:
                        row.extend(["", ""])
                row.extend(
                    [
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
                )
                row.extend(
                    "%.6f" % float(rp_ep.get(k, 0.0)) for k in REWARD_PART_KEYS
                )
                metrics_w.writerow(row)
                metrics_f.flush()

                episode_sum_v = 0.0
                episode_sum_w = 0.0
                episode_action_n = 0

                # Log training terminal + goal for heatmaps (does not affect SR)
                if viz_eval_log_train_xy and _append_outcome_xy_csv is not None:
                    try:
                        world_name = info.get("world") or getattr(
                            env, "_world_basename", lambda: ""
                        )()
                        _append_outcome_xy_csv(
                            os.path.join(out_root, "outcome_xy.csv"),
                            {
                                "x": "%.4f" % float(info.get("x", 0.0)),
                                "y": "%.4f" % float(info.get("y", 0.0)),
                                "outcome": outcome,
                                "world": world_name,
                                "episode": episode_num,
                                "sr_milestone": "",
                                "source": "train",
                                "path_efficiency": info.get("path_efficiency", ""),
                                "t": t,
                                "point_kind": "terminal",
                            },
                        )
                        if viz_eval_heatmap_goals:
                            gx = info.get("goal_x")
                            gy = info.get("goal_y")
                            if gx is not None and gy is not None:
                                _append_outcome_xy_csv(
                                    os.path.join(out_root, "outcome_xy.csv"),
                                    {
                                        "x": "%.4f" % float(gx),
                                        "y": "%.4f" % float(gy),
                                        "outcome": outcome,
                                        "world": world_name,
                                        "episode": episode_num,
                                        "sr_milestone": "",
                                        "source": "train_goal",
                                        "path_efficiency": "",
                                        "t": t,
                                        "point_kind": "goal",
                                    },
                                )
                    except Exception as e:
                        _log_warn("[viz_eval] outcome_xy append failed: %s", e)

                # Periodic frozen-policy eval (not counted in train SR)
                if (
                    viz_eval_enable
                    and viz_controller is not None
                    and viz_pairs_doc is not None
                    and _maybe_run_milestone_eval is not None
                ):
                    try:
                        _maybe_run_milestone_eval(
                            controller=viz_controller,
                            policy=policy,
                            env=env,
                            pairs_doc=viz_pairs_doc,
                            out_root=out_root,
                            success_rate_win=rates["success_rate_win"],
                            episode_num=episode_num,
                            t=t,
                            max_steps=viz_eval_max_steps,
                            opacity=viz_eval_opacity,
                            log_fn=_log_info,
                            basemap_enable=False,
                            clip_z=viz_topdown_clip_z,
                            gui_pause=viz_eval_gui_pause,
                            heatmap_goals=viz_eval_heatmap_goals,
                        )
                    except Exception as e:
                        _log_warn("[viz_eval] milestone hook failed (train continues): %s", e)

                # Landscape GIF weights only (no Gazebo / traj eval). Cheap disk snapshot.
                if (
                    landscape_ckpt_enable
                    and landscape_ckpt_every_n_ep > 0
                    and int(episode_num) > 0
                    and int(episode_num) % int(landscape_ckpt_every_n_ep) == 0
                ):
                    tag = "ep%04d" % int(episode_num)
                    ckpt_prefix = os.path.join(
                        out_root, "loss_landscape", "ckpts", tag, "td3"
                    )
                    try:
                        os.makedirs(os.path.dirname(ckpt_prefix), exist_ok=True)
                        if _safe_save_landscape_weights(
                            policy, ckpt_prefix, out_root
                        ):
                            _log_info(
                                "[landscape] ckpt (weights) → %s_{actor,critic}",
                                ckpt_prefix,
                            )
                    except Exception as e_ckpt:
                        _log_warn("[landscape] ckpt save failed: %s", e_ckpt)

                state = _reset_until_ok(env, log_f, t, label="post-episode")
                if state is None:
                    _log_err("[spawn] ROS shutdown during post-episode reset")
                    break
                episode_reward = 0.0
                episode_timesteps = 0
                episode_collision_events = 0
                episode_sum_v = 0.0
                episode_sum_w = 0.0
                episode_action_n = 0
                episode_num += 1
            elif switched:
                # Mid-episode world reload: drop unfinished ep metrics, keep replay
                state = _reset_until_ok(env, log_f, t, label="post-switch")
                if state is None:
                    _log_err("[spawn] ROS shutdown during post-switch reset")
                    break
                episode_reward = 0.0
                episode_timesteps = 0
                episode_sum_v = 0.0
                episode_sum_w = 0.0
                episode_action_n = 0
                _log_info("[world] train loop resumed after switch (t=%d)", t)

            if t % save_freq == 0 and t > 0:
                _safe_save_policy(policy, model_prefix, out_root)

        if rclpy.ok():
            _log_info(
                "Training finished. Models at %s_*  log=%s", model_prefix, log_path
            )
    finally:
        if t > 0:
            if _safe_save_policy(policy, model_prefix, out_root) and not rclpy.ok():
                _log_info(
                    "saved models (shutdown) → %s_{actor,critic}", model_prefix
                )
        log_f.close()
        metrics_f.close()
        if auto_plot:
            _plot_metrics(out_root)
        try:
            env.close()
        except Exception:
            pass


def main_entry(args=None):
    global _NODE
    rclpy.init(args=args)
    # automatically_declare_parameters_from_overrides: launch yaml + CLI -p
    # overrides become declared params before our defaults fill gaps.
    _NODE = rclpy.create_node(
        "train_td3_node",
        automatically_declare_parameters_from_overrides=True,
    )
    _declare_train_params(_NODE)
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        _NODE.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main_entry()
