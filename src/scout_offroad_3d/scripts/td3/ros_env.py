#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS / Gazebo episode bridge for Scout Mini TD3 (stage A).

Inspired by ROS+Gazebo DRL training loops (e.g. DRL-AUV-navigation patterns):
reset pose → observe → publish cmd_vel → reward / done.
Carrier is ground skid-steer Scout, not AUV.

Reward / done (see step()):
  r_episode_start (once per ep), r_progress·Δρ, r_explore (first-visit occupancy cell),
  r_arrive, r_forward·max(0,v-|ω|), r_spin (low v + high |ω|), r_low_effort (sustained spin),
  collision_penalty_list (default -25/-50/-100), r_stuck, r_timeout, r_step.
  Episode ends when rho < goal_tol (success), rollover (IMU roll/pitch after grace),
  stuck (path displacement window), step >= max_ep_steps (timeout + r_timeout), or
  max_collision_events (default 3).
  During rollover_grace_sec, IMU flip aborts the episode (resample) when
  rollover_grace_abort_enable is true — no rollover outcome or metrics.
  Collision is usually non-terminal: front-sector proximity + approach speed +
  consecutive frames → penalty; slope vs wall via multi-elevation features;
  logs [terrain] slope suspected / [collision] wall confirmed ...

World switch (Gazebo Classic): there is no service to hot-swap a .world file.
switch_world() commits current_world + world_spawn_z BEFORE spawn validation,
kills gzserver, starts a new one with the target .world, then re-spawns
scout_mini from /robot_description. Replay buffer is owned by the trainer
and is not cleared here.

Spawn gate: reset() and switch_world() try many random (x,y) on the CURRENT
world until a pose passes (z/vz/clearance). Dense trees / heightmap holes
are expected. If this world is exhausted, reset() pick_next_world(exclude
current) and switch_world() — it must NEVER abort the whole training run.
If every world in the pool fails one cycle: skip episode, wait, retry pool.
"""

from __future__ import print_function

import atexit
import glob
import math
import os
import random
import signal
import subprocess
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    DeleteModel,
    GetModelState,
    SetModelState,
    SpawnModel,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _roll_pitch_rad_from_quat(q):
    """Body roll/pitch (rad) from quaternion; same convention as sensor_msgs/Imu."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = float(np.clip(sinp, -1.0, 1.0))
    pitch = math.asin(sinp)
    return roll, pitch


def _quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _port_11345_busy():
    try:
        out = subprocess.check_output(
            ["ss", "-lnt"], stderr=subprocess.DEVNULL
        ).decode("utf-8", "replace")
        return ":11345" in out
    except Exception:
        return False


def _as_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    return str(v).strip().lower() in ("1", "true", "yes", "on")


# Per-episode reward component keys (accumulated in step(), summed in info on done)
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


def _parse_float_list(raw, default=None):
    if raw is None:
        return list(default) if default is not None else []
    if isinstance(raw, (list, tuple)):
        return [float(x) for x in raw]
    parts = str(raw).replace(",", " ").split()
    if not parts:
        return list(default) if default is not None else []
    return [float(x) for x in parts]


def _wall_sleep(dt):
    """Wall-clock sleep — safe when /use_sim_time and gzserver is dead (rospy.sleep hangs)."""
    if dt > 0:
        time.sleep(dt)


def _rosnode_list():
    try:
        out = subprocess.check_output(
            ["rosnode", "list"], stderr=subprocess.DEVNULL, timeout=5
        ).decode("utf-8", "replace")
        return [ln.strip() for ln in out.splitlines() if ln.strip()]
    except Exception:
        return []


def _rosnode_alive(name):
    """True if name appears in rosnode list (exact or trailing match)."""
    want = name if name.startswith("/") else ("/" + name)
    for n in _rosnode_list():
        if n == want or n.endswith(want):
            return True
    return False


def _pkill_patterns(patterns):
    for pat in patterns:
        try:
            subprocess.call(
                ["pkill", "-9", "-f", pat],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass


def _gzserver_wrapper_pids():
    try:
        out = subprocess.check_output(
            ["pgrep", "-f", "gazebo_ros/gzserver"],
            stderr=subprocess.DEVNULL,
        ).decode("utf-8", "replace")
        return [int(x) for x in out.split() if x.strip()]
    except Exception:
        return []


def _kill_proc_group(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


class ScoutGazeboEnv(object):
    def __init__(self):
        self.n_sectors = int(rospy.get_param("~n_sectors", 24))
        self.max_range = float(rospy.get_param("~max_range", 5.0))
        self.goal_tol = float(rospy.get_param("~goal_tol", 0.45))
        self.collision_dist = float(rospy.get_param("~collision_dist", 0.28))
        self.collision_v_min = float(rospy.get_param("~collision_v_min", 0.08))
        self.collision_frames = int(rospy.get_param("~collision_frames", 4))
        self.collision_sector_half_fov_deg = float(
            rospy.get_param("~collision_sector_half_fov_deg", 90.0)
        )
        self.max_collision_events = int(rospy.get_param("~max_collision_events", 0))
        self.collision_penalty_double = _as_bool(
            rospy.get_param("~collision_penalty_double", True)
        )
        self.collision_penalty_list = _parse_float_list(
            rospy.get_param("~collision_penalty_list", None), None
        )
        self.stuck_time_sec = float(rospy.get_param("~stuck_time_sec", 5.0))
        self.stuck_dist_threshold = float(
            rospy.get_param("~stuck_dist_threshold", 0.25)
        )
        self.max_steps = int(rospy.get_param("~max_ep_steps", 400))
        self.control_dt = float(rospy.get_param("~control_dt", 0.1))
        self.max_v = float(rospy.get_param("~max_v", 0.6))
        self.max_w = float(rospy.get_param("~max_w", 1.0))
        self.pose_source = rospy.get_param("~pose_source", "gazebo").strip().lower()
        self.model_name = rospy.get_param("~model_name", "scout_mini")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.features_topic = rospy.get_param("~features_topic", "/offroad/range_features")
        self.td3_features_topic = rospy.get_param(
            "~td3_features_topic", "/offroad/td3_range_features"
        )
        self.elevation_features_topic = rospy.get_param(
            "~elevation_features_topic", "/offroad/elevation_features"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")

        # Slope vs wall terrain classifier (uses multi-elevation front sectors + optional IMU)
        self.terrain_classify_enable = _as_bool(
            rospy.get_param("~terrain_classify_enable", True)
        )
        self.n_elevation_bins = int(rospy.get_param("~n_elevation_bins", 4))
        self.n_elev_front_sectors = int(rospy.get_param("~n_elev_front_sectors", 12))
        self.n_elevation_bins_td3 = int(rospy.get_param("~n_elevation_bins_td3", 3))
        self.slope_max_frames = int(rospy.get_param("~slope_max_frames", 6))
        self.wall_all_layers_dist = float(rospy.get_param("~wall_all_layers_dist", 0.27))
        self.wall_lower_layers_dist = float(
            rospy.get_param("~wall_lower_layers_dist", 0.27)
        )
        self.wall_layers_required = int(rospy.get_param("~wall_layers_required", 2))
        self.wall_force_min_dist = float(rospy.get_param("~wall_force_min_dist", 0.19))
        self.slope_upper_clear_dist = float(
            rospy.get_param("~slope_upper_clear_dist", 0.8)
        )
        self.r_slope_penalty = float(rospy.get_param("~r_slope_penalty", -0.2))
        self.use_imu_pitch = _as_bool(rospy.get_param("~use_imu_pitch", True))
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.slope_pitch_delta_deg = float(
            rospy.get_param("~slope_pitch_delta_deg", 3.0)
        )
        self.wall_v_forward_max = float(rospy.get_param("~wall_v_forward_max", 0.12))

        # Random start / goal box (meters, map/world frame; tune via yaml for large maps)
        self.spawn_xy_min = [
            float(rospy.get_param("~spawn_x_min", -3.5)),
            float(rospy.get_param("~spawn_y_min", -3.5)),
        ]
        self.spawn_xy_max = [
            float(rospy.get_param("~spawn_x_max", 3.5)),
            float(rospy.get_param("~spawn_y_max", 3.5)),
        ]
        self.goal_xy_min = [
            float(rospy.get_param("~goal_x_min", self.spawn_xy_min[0])),
            float(rospy.get_param("~goal_y_min", self.spawn_xy_min[1])),
        ]
        self.goal_xy_max = [
            float(rospy.get_param("~goal_x_max", self.spawn_xy_max[0])),
            float(rospy.get_param("~goal_y_max", self.spawn_xy_max[1])),
        ]
        self.min_goal_dist = float(rospy.get_param("~min_goal_dist", 1.5))
        self.goal_check_enable = _as_bool(rospy.get_param("~goal_check_enable", True))
        self.goal_min_clearance = float(rospy.get_param("~goal_min_clearance", 0.38))
        self.goal_z_max_delta = float(rospy.get_param("~goal_z_max_delta", 0.35))
        self.goal_resample_max = int(rospy.get_param("~goal_resample_max", 25))
        self._base_spawn_z = float(rospy.get_param("~spawn_z", 0.15))
        self.spawn_z = self._base_spawn_z
        self._world_spawn_z = dict(rospy.get_param("~world_spawn_z", {}) or {})
        self.spawn_z_retry_delta = float(rospy.get_param("~spawn_z_retry_delta", 0.08))
        self.spawn_z_max_retries = int(rospy.get_param("~spawn_z_max_retries", 2))
        self.spawn_check_wait = float(rospy.get_param("~spawn_check_wait", 0.5))
        self.spawn_z_jump_threshold = float(
            rospy.get_param("~spawn_z_jump_threshold", 0.25)
        )
        self.spawn_vz_threshold = float(rospy.get_param("~spawn_vz_threshold", 0.5))
        self.spawn_collision_dist = float(
            rospy.get_param("~spawn_collision_dist", 0.35)
        )
        self.spawn_pose_stable_threshold = float(
            rospy.get_param("~spawn_pose_stable_threshold", 0.08)
        )
        self.spawn_full_reset_retries = int(
            rospy.get_param("~spawn_full_reset_retries", 40)
        )
        self.spawn_xy_max_retries = int(
            rospy.get_param("~spawn_xy_max_retries", 40)
        )
        self.spawn_xy_jitter = float(rospy.get_param("~spawn_xy_jitter", 0.8))

        self.r_progress = float(rospy.get_param("~r_progress", 1.0))
        self.r_arrive = float(rospy.get_param("~r_arrive", 20.0))
        self.r_episode_start = float(rospy.get_param("~r_episode_start", 0.0))
        self.r_collision_base = float(rospy.get_param("~r_collision_base", -2.0))
        self.r_stuck = float(rospy.get_param("~r_stuck", -20.0))
        self.r_rollover = float(rospy.get_param("~r_rollover", -25.0))
        self.r_timeout = float(rospy.get_param("~r_timeout", -20.0))
        self.r_step = float(rospy.get_param("~r_step", -0.05))
        self.r_forward_scale = float(rospy.get_param("~r_forward_scale", 0.5))
        self.r_spin_penalty = float(rospy.get_param("~r_spin_penalty", 0.5))
        self.spin_v_thresh = float(rospy.get_param("~spin_v_thresh", 0.15))
        self.spin_w_thresh = float(rospy.get_param("~spin_w_thresh", 0.8))
        self.r_low_effort_penalty = float(
            rospy.get_param("~r_low_effort_penalty", -0.08)
        )
        self.low_effort_thresh = float(rospy.get_param("~low_effort_thresh", 0.05))
        self.low_effort_time_sec = float(rospy.get_param("~low_effort_time_sec", 3.0))
        self.state_noise_std = float(rospy.get_param("~state_noise_std", 0.0))
        self.reward_log_interval = int(rospy.get_param("~reward_log_interval", 25))

        self.explore_enable = _as_bool(rospy.get_param("~explore_enable", True))
        self.explore_grid_res = float(rospy.get_param("~explore_grid_res", 0.5))
        self.explore_cell_reward = float(
            rospy.get_param("~explore_cell_reward", 0.03)
        )
        self.explore_rho_min = float(rospy.get_param("~explore_rho_min", 1.0))
        self.explore_cardiff_scale = _as_bool(
            rospy.get_param("~explore_cardiff_scale", False)
        )

        self.rollover_enable = _as_bool(rospy.get_param("~rollover_enable", True))
        self.rollover_roll_deg = float(rospy.get_param("~rollover_roll_deg", 40.0))
        self.rollover_pitch_deg = float(rospy.get_param("~rollover_pitch_deg", 40.0))
        self.rollover_sum_deg = float(rospy.get_param("~rollover_sum_deg", 55.0))
        self.rollover_grace_sec = float(rospy.get_param("~rollover_grace_sec", 2.0))
        self.rollover_grace_abort_enable = _as_bool(
            rospy.get_param("~rollover_grace_abort_enable", True)
        )
        self.rollover_model_fallback = _as_bool(
            rospy.get_param("~rollover_model_fallback", True)
        )
        self.use_imu_roll = _as_bool(rospy.get_param("~use_imu_roll", True))
        self.imu_stale_sec = float(rospy.get_param("~imu_stale_sec", 1.0))
        self.imu_wait_after_respawn_sec = float(
            rospy.get_param("~imu_wait_after_respawn_sec", 8.0)
        )
        self.imu_model_fallback_after_switch_steps = int(
            rospy.get_param("~imu_model_fallback_after_switch_steps", 50)
        )

        self.world_switch_interval = int(rospy.get_param("~world_switch_interval", 1000))
        self.world_switch_max_retries = int(
            rospy.get_param("~world_switch_max_retries", 5)
        )
        self.world_switch_kill_wait_sec = float(
            rospy.get_param("~world_switch_kill_wait_sec", 0.5)
        )
        self.world_switch_gzserver_boot_sec = float(
            rospy.get_param("~world_switch_gzserver_boot_sec", 8.0)
        )
        self.world_switch_spawn_wait_sec = float(
            rospy.get_param("~world_switch_spawn_wait_sec", 35.0)
        )
        self.world_switch_retry_sleep_sec = float(
            rospy.get_param("~world_switch_retry_sleep_sec", 1.0)
        )
        self.worlds_dir = rospy.get_param("~worlds_dir", "").strip()
        self._want_gui = _as_bool(rospy.get_param("/offroad/gazebo_gui", False))
        self.current_world = rospy.get_param("/offroad/current_world", "").strip()
        self._world_before_switch = None
        # Re-bind spawn_z now that current_world is known (heightmaps ≠ 0.20).
        self.spawn_z = self._spawn_z_for_current_world()

        self.state_dim = self.n_elevation_bins_td3 * self.n_sectors + 4
        self.action_dim = 2
        self.max_action = np.array([self.max_v, self.max_w], dtype=np.float32)

        self._features = np.full(self.n_sectors, self.max_range, dtype=np.float32)
        self._td3_feature_count = self.n_elevation_bins_td3 * self.n_sectors
        self._td3_features = np.full(self._td3_feature_count, self.max_range, dtype=np.float32)
        self._odom_pose = None
        self._goal = np.zeros(2, dtype=np.float64)
        self._prev_action = np.zeros(2, dtype=np.float32)
        self._prev_dist = None
        self._step = 0
        self._ep_count = 0
        self._ep_reward_accum = 0.0
        self._ep_reward_parts = {k: 0.0 for k in REWARD_PART_KEYS}
        self._explore_cells = set()
        self._collision_streak = 0
        self._collision_event_count = 0
        self._proximity_frame_count = 0
        self._roll = 0.0
        self._pitch = 0.0
        self._imu_received = False
        self._imu_last_time = None
        self._imu_sub = None
        self._post_switch_model_only_steps = 0
        self._pitch_at_proximity = None
        self._ep_start_time = None
        self._elevation_features = None
        self._stuck_hist = []
        self._low_effort_hist = []
        self._initial_goal_dist = 0.0
        self._path_length = 0.0
        self._prev_pose_xy = None

        self._gz_proc = None
        self._gzclient_proc = None
        self._owns_gz = False

        self._cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self._goal_pub = rospy.Publisher("/offroad/train_goal", Point, queue_size=1)
        rospy.Subscriber(self.features_topic, Float32MultiArray, self._on_features, queue_size=1)
        rospy.Subscriber(
            self.td3_features_topic, Float32MultiArray, self._on_td3_features, queue_size=1
        )
        if self.terrain_classify_enable:
            rospy.Subscriber(
                self.elevation_features_topic,
                Float32MultiArray,
                self._on_elevation_features,
                queue_size=1,
            )
        self._ensure_imu_subscriber()
        if self.pose_source == "odom":
            rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=1)

        rospy.loginfo("ScoutGazeboEnv waiting for Gazebo model services...")
        rospy.wait_for_service("/gazebo/get_model_state", timeout=120.0)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        self._bind_model_services()

        atexit.register(self._shutdown_owned_gazebo)
        rospy.on_shutdown(self._shutdown_owned_gazebo)

        rospy.loginfo(
            "ScoutGazeboEnv ready: model=%s pose=%s td3=%s horiz=%s elev=%s "
            "cmd=%s state_dim=%d terrain=%s world=%s switch_interval=%d",
            self.model_name,
            self.pose_source,
            self.td3_features_topic,
            self.features_topic,
            self.elevation_features_topic if self.terrain_classify_enable else "(off)",
            self.cmd_topic,
            self.state_dim,
            self.terrain_classify_enable,
            self.current_world or "(launch default)",
            self.world_switch_interval,
        )

    def set_episode_count(self, n):
        """Align env log [ep=N] with trainer episode index (used on resume)."""
        self._ep_count = max(0, int(n))

    def snapshot_config(self):
        """Effective private params for run backup (launch/yaml overrides applied)."""
        return {
            "n_sectors": self.n_sectors,
            "max_range": self.max_range,
            "goal_tol": self.goal_tol,
            "collision_dist": self.collision_dist,
            "collision_v_min": self.collision_v_min,
            "collision_frames": self.collision_frames,
            "collision_sector_half_fov_deg": self.collision_sector_half_fov_deg,
            "max_collision_events": self.max_collision_events,
            "collision_penalty_double": self.collision_penalty_double,
            "collision_penalty_list": list(self.collision_penalty_list or []),
            "stuck_time_sec": self.stuck_time_sec,
            "stuck_dist_threshold": self.stuck_dist_threshold,
            "max_steps": self.max_steps,
            "control_dt": self.control_dt,
            "max_v": self.max_v,
            "max_w": self.max_w,
            "pose_source": self.pose_source,
            "model_name": self.model_name,
            "cmd_topic": self.cmd_topic,
            "features_topic": self.features_topic,
            "td3_features_topic": self.td3_features_topic,
            "elevation_features_topic": self.elevation_features_topic,
            "odom_topic": self.odom_topic,
            "terrain_classify_enable": self.terrain_classify_enable,
            "n_elevation_bins": self.n_elevation_bins,
            "n_elev_front_sectors": self.n_elev_front_sectors,
            "n_elevation_bins_td3": self.n_elevation_bins_td3,
            "slope_max_frames": self.slope_max_frames,
            "wall_all_layers_dist": self.wall_all_layers_dist,
            "wall_lower_layers_dist": self.wall_lower_layers_dist,
            "wall_layers_required": self.wall_layers_required,
            "wall_force_min_dist": self.wall_force_min_dist,
            "slope_upper_clear_dist": self.slope_upper_clear_dist,
            "r_slope_penalty": self.r_slope_penalty,
            "use_imu_pitch": self.use_imu_pitch,
            "use_imu_roll": self.use_imu_roll,
            "imu_topic": self.imu_topic,
            "slope_pitch_delta_deg": self.slope_pitch_delta_deg,
            "wall_v_forward_max": self.wall_v_forward_max,
            "spawn_xy_min": list(self.spawn_xy_min),
            "spawn_xy_max": list(self.spawn_xy_max),
            "goal_xy_min": list(self.goal_xy_min),
            "goal_xy_max": list(self.goal_xy_max),
            "min_goal_dist": self.min_goal_dist,
            "goal_check_enable": self.goal_check_enable,
            "goal_min_clearance": self.goal_min_clearance,
            "goal_z_max_delta": self.goal_z_max_delta,
            "goal_resample_max": self.goal_resample_max,
            "spawn_z": self._base_spawn_z,
            "world_spawn_z": dict(self._world_spawn_z),
            "spawn_z_retry_delta": self.spawn_z_retry_delta,
            "spawn_z_max_retries": self.spawn_z_max_retries,
            "spawn_check_wait": self.spawn_check_wait,
            "spawn_z_jump_threshold": self.spawn_z_jump_threshold,
            "spawn_vz_threshold": self.spawn_vz_threshold,
            "spawn_collision_dist": self.spawn_collision_dist,
            "spawn_pose_stable_threshold": self.spawn_pose_stable_threshold,
            "spawn_full_reset_retries": self.spawn_full_reset_retries,
            "spawn_xy_max_retries": self.spawn_xy_max_retries,
            "spawn_xy_jitter": self.spawn_xy_jitter,
            "r_progress": self.r_progress,
            "r_arrive": self.r_arrive,
            "r_episode_start": self.r_episode_start,
            "r_collision_base": self.r_collision_base,
            "r_stuck": self.r_stuck,
            "r_rollover": self.r_rollover,
            "r_timeout": self.r_timeout,
            "r_step": self.r_step,
            "r_forward_scale": self.r_forward_scale,
            "r_spin_penalty": self.r_spin_penalty,
            "spin_v_thresh": self.spin_v_thresh,
            "spin_w_thresh": self.spin_w_thresh,
            "r_low_effort_penalty": self.r_low_effort_penalty,
            "low_effort_thresh": self.low_effort_thresh,
            "low_effort_time_sec": self.low_effort_time_sec,
            "explore_enable": self.explore_enable,
            "explore_grid_res": self.explore_grid_res,
            "explore_cell_reward": self.explore_cell_reward,
            "explore_rho_min": self.explore_rho_min,
            "explore_cardiff_scale": self.explore_cardiff_scale,
            "state_noise_std": self.state_noise_std,
            "reward_log_interval": self.reward_log_interval,
            "rollover_enable": self.rollover_enable,
            "rollover_roll_deg": self.rollover_roll_deg,
            "rollover_pitch_deg": self.rollover_pitch_deg,
            "rollover_sum_deg": self.rollover_sum_deg,
            "rollover_grace_sec": self.rollover_grace_sec,
            "rollover_grace_abort_enable": self.rollover_grace_abort_enable,
            "rollover_model_fallback": self.rollover_model_fallback,
            "imu_stale_sec": self.imu_stale_sec,
            "imu_wait_after_respawn_sec": self.imu_wait_after_respawn_sec,
            "imu_model_fallback_after_switch_steps": (
                self.imu_model_fallback_after_switch_steps
            ),
            "world_switch_interval": self.world_switch_interval,
            "world_switch_max_retries": self.world_switch_max_retries,
            "world_switch_kill_wait_sec": self.world_switch_kill_wait_sec,
            "world_switch_gzserver_boot_sec": self.world_switch_gzserver_boot_sec,
            "world_switch_spawn_wait_sec": self.world_switch_spawn_wait_sec,
            "world_switch_retry_sleep_sec": self.world_switch_retry_sleep_sec,
            "worlds_dir": self.worlds_dir,
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "max_action": self.max_action.tolist(),
        }

    def _bind_model_services(self):
        self._get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def _worlds_dir_resolved(self):
        if self.worlds_dir:
            return self.worlds_dir
        try:
            import rospkg

            return os.path.join(rospkg.RosPack().get_path("scout_offroad_3d"), "worlds")
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.normpath(os.path.join(here, "..", "..", "worlds"))

    def list_worlds(self):
        cands = sorted(glob.glob(os.path.join(self._worlds_dir_resolved(), "*.world")))
        if not cands:
            raise RuntimeError("no *.world under %s" % self._worlds_dir_resolved())
        return cands

    def pick_next_world(self, exclude=None, require_different=False):
        """Random world path; prefer a file not in exclude when possible.

        exclude: one path or an iterable of paths.
        require_different=True: return None if every candidate is excluded
        (used by spawn-exhaust so we do not silently reload the same map).
        """
        cands = self.list_worlds()
        excluded = set()
        if exclude:
            if isinstance(exclude, str):
                excluded = {os.path.abspath(exclude)}
            else:
                try:
                    excluded = {os.path.abspath(p) for p in exclude if p}
                except TypeError:
                    excluded = {os.path.abspath(exclude)}
        others = [c for c in cands if os.path.abspath(c) not in excluded]
        if others:
            return random.choice(others)
        if require_different:
            return None
        return random.choice(cands)

    def _on_features(self, msg):
        data = np.asarray(msg.data, dtype=np.float32)
        if data.size >= self.n_sectors:
            self._features = data[: self.n_sectors].copy()
        elif data.size > 0:
            out = np.full(self.n_sectors, self.max_range, dtype=np.float32)
            out[: data.size] = data
            self._features = out

    def _on_td3_features(self, msg):
        data = np.asarray(msg.data, dtype=np.float32)
        n = self._td3_feature_count
        if data.size >= n:
            self._td3_features = data[:n].copy()
        elif data.size > 0:
            out = np.full(n, self.max_range, dtype=np.float32)
            out[: data.size] = data
            self._td3_features = out

    def _td3_sector_distances(self):
        """TD3 state sector mins; fallback to max_range if topic not yet received."""
        return self._td3_features.copy()

    def _on_elevation_features(self, msg):
        self._elevation_features = np.asarray(msg.data, dtype=np.float32)

    def _imu_needed(self):
        return self.use_imu_pitch or self.use_imu_roll or self.rollover_enable

    def _reset_imu_state(self):
        """Drop cached IMU angles so rollover cannot use pre-respawn stale data."""
        self._roll = 0.0
        self._pitch = 0.0
        self._imu_received = False
        self._imu_last_time = None
        self._pitch_at_proximity = None

    def _ensure_imu_subscriber(self):
        """(Re)subscribe IMU — needed after gzserver restart when publisher is new."""
        if not self._imu_needed():
            return
        if self._imu_sub is not None:
            try:
                self._imu_sub.unregister()
            except Exception:
                pass
            self._imu_sub = None
        self._imu_sub = rospy.Subscriber(self.imu_topic, Imu, self._on_imu, queue_size=1)
        rospy.loginfo("[imu] (re)subscribed %s", self.imu_topic)

    def _imu_fresh(self):
        if not self._imu_received or self._imu_last_time is None:
            return False
        try:
            age = (rospy.Time.now() - self._imu_last_time).to_sec()
        except Exception:
            return False
        return age <= self.imu_stale_sec

    def _rollover_orientation_tag(self):
        """Return imu status tag for logs: ok | stale | fallback."""
        if self._post_switch_model_only_steps > 0:
            return "fallback" if self.rollover_model_fallback else "stale"
        if self._imu_fresh():
            return "ok"
        if self.rollover_model_fallback:
            return "fallback"
        return "stale"

    def _log_imu_post_switch_status(self):
        """Mandatory [imu] ok/stale/fallback log after every world switch."""
        tag = self._rollover_orientation_tag()
        roll_deg = abs(math.degrees(self._roll))
        pitch_deg = abs(math.degrees(self._pitch))
        if tag == "fallback" and self.rollover_model_fallback:
            m_roll, m_pitch = self._model_orientation_deg()
            if m_roll is not None:
                roll_deg, pitch_deg = m_roll, m_pitch
        rospy.loginfo(
            "[imu] %s after world switch topic=%s roll=%.1f° pitch=%.1f° "
            "model_only_steps=%d",
            tag,
            self.imu_topic,
            roll_deg,
            pitch_deg,
            self._post_switch_model_only_steps,
        )

    def _model_orientation_deg(self):
        """|roll|, |pitch| from Gazebo model pose; None on failure."""
        try:
            resp = self._get_state(self.model_name, "world")
            if resp.success:
                roll, pitch = _roll_pitch_rad_from_quat(resp.pose.orientation)
                return abs(math.degrees(roll)), abs(math.degrees(pitch))
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "model orientation query failed: %s", e)
        return None, None

    def _wait_imu_after_respawn(self, timeout=None):
        if timeout is None:
            timeout = self.imu_wait_after_respawn_sec
        if not self._imu_needed():
            return True
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._imu_fresh():
                rospy.loginfo(
                    "[imu] first message on %s after respawn (%.2fs)",
                    self.imu_topic,
                    time.time() - t0,
                )
                return True
            _wall_sleep(0.05)
        rospy.logwarn(
            "[imu] no data on %s after %.1fs — rollover will use model fallback",
            self.imu_topic,
            timeout,
        )
        return False

    def _on_imu(self, msg):
        self._imu_received = True
        self._imu_last_time = rospy.Time.now()
        self._roll, self._pitch = _roll_pitch_rad_from_quat(msg.orientation)

    def _on_odom(self, msg):
        self._odom_pose = msg.pose.pose

    def _get_pose_yaw(self):
        if self.pose_source == "odom":
            if self._odom_pose is None:
                return None
            p = self._odom_pose.position
            return p.x, p.y, _yaw_from_quat(self._odom_pose.orientation)

        try:
            resp = self._get_state(self.model_name, "world")
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "get_model_state failed: %s", e)
            return None
        if not resp.success:
            return None
        p = resp.pose.position
        return p.x, p.y, _yaw_from_quat(resp.pose.orientation)

    def _wait_pose(self, timeout=15.0):
        t0 = time.time()
        while not rospy.is_shutdown():
            pose = self._get_pose_yaw()
            if pose is not None:
                return pose
            if time.time() - t0 > timeout:
                raise RuntimeError(
                    "timeout waiting for robot pose (model=%s, %.1fs wall)"
                    % (self.model_name, timeout)
                )
            # wall clock: sim time freezes if gzserver died mid-switch
            _wall_sleep(0.05)
        raise RuntimeError(
            "ROS shutdown while waiting for robot pose (model=%s)"
            % self.model_name
        )

    def _wait_for_model(self, timeout=30.0):
        """Block until Gazebo reports model_name exists (spawn may lag launch)."""
        t0 = time.time()
        while not rospy.is_shutdown():
            try:
                resp = self._get_state(self.model_name, "world")
                if resp.success:
                    return True
            except rospy.ServiceException:
                pass
            if time.time() - t0 > timeout:
                return False
            _wall_sleep(0.1)
        return False

    def _world_basename(self, world_path=None):
        path = world_path if world_path else self.current_world
        if path:
            return os.path.basename(path)
        return "(default)"

    def _spawn_z_for_world(self, world_path=None):
        """Per-world spawn_z override (e.g. heightmap needs ~1.05, playpen 0.20)."""
        base = self._world_basename(world_path)
        if not base or base == "(default)":
            return self._base_spawn_z
        table = self._world_spawn_z or {}
        if base in table:
            return float(table[base])
        stem = base[:-6] if base.lower().endswith(".world") else base
        for key, val in table.items():
            k = str(key)
            if k == base or k == stem or k in base or base in k or stem in k:
                return float(val)
        return self._base_spawn_z

    def _spawn_z_for_current_world(self):
        return self._spawn_z_for_world(self.current_world)

    def _apply_world_identity(self, world_path):
        """Bind current_world + spawn_z to the world gzserver is loading.

        MUST run before _spawn_robot / _ensure_spawn_ok. Record 6 bug: identity
        was updated only after spawn OK, so bumpy terrain z≈1.06 was checked
        against playpen spawn_z=0.20 → false z_jump → abort → lock to playpen.
        """
        world_path = os.path.abspath(world_path)
        self.current_world = world_path
        self.spawn_z = self._spawn_z_for_world(world_path)
        try:
            rospy.set_param("/offroad/current_world", world_path)
        except Exception:
            pass
        rospy.loginfo(
            "[world] identity → %s spawn_z=%.3f",
            os.path.basename(world_path),
            self.spawn_z,
        )
        return self.spawn_z

    def _get_model_z_vz(self):
        try:
            resp = self._get_state(self.model_name, "world")
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "get_model_state (z): %s", e)
            return None, None
        if not resp.success:
            return None, None
        return float(resp.pose.position.z), float(resp.twist.linear.z)

    def _set_pose(self, x, y, yaw, z=None):
        z_use = self.spawn_z if z is None else float(z)
        state = ModelState()
        state.model_name = self.model_name
        state.pose = Pose()
        state.pose.position = Point(x, y, z_use)
        state.pose.orientation = _quat_from_yaw(yaw)
        state.twist = Twist()
        state.reference_frame = "world"
        for _ in range(5):
            try:
                self._set_state(state)
            except rospy.ServiceException as e:
                rospy.logwarn("set_model_state: %s", e)
            _wall_sleep(0.05)

    def _min_sector_dist(self):
        if self._features.size:
            return float(np.min(self._features))
        return self.max_range

    def _sector_bearing(self, idx):
        """Sector center bearing (rad): +x forward, +y left; matches range_features_node."""
        angle_max = math.pi
        angle_min = -math.pi
        span = angle_max - angle_min
        t = (float(idx) + 0.5) / float(self.n_sectors)
        return angle_max - t * span

    def _is_front_sector(self, idx):
        half_fov = math.radians(self.collision_sector_half_fov_deg)
        return abs(self._sector_bearing(idx)) <= half_fov + 1e-9

    def _get_body_velocity(self, yaw):
        """Body-frame (vx fwd, vy left) from Gazebo twist or last cmd_vel."""
        try:
            resp = self._get_state(self.model_name, "world")
            if resp.success:
                tw = resp.twist
                vx_w = float(tw.linear.x)
                vy_w = float(tw.linear.y)
                cy = math.cos(yaw)
                sy = math.sin(yaw)
                vx_b = vx_w * cy + vy_w * sy
                vy_b = -vx_w * sy + vy_w * cy
                return vx_b, vy_b
        except rospy.ServiceException:
            pass
        return float(self._prev_action[0]), 0.0

    def _approach_speed(self, vx, vy, bearing):
        """Speed component toward obstacle at bearing (rad, body frame)."""
        return vx * math.cos(bearing) + vy * math.sin(bearing)

    def _collision_penalty_for_event(self, k):
        """k-th collision: explicit list, or base * 2^(k-1) when collision_penalty_double."""
        if self.collision_penalty_list:
            idx = min(max(k - 1, 0), len(self.collision_penalty_list) - 1)
            return float(self.collision_penalty_list[idx])
        if self.collision_penalty_double:
            mag = abs(self.r_collision_base) * (2.0 ** (k - 1))
        else:
            mag = abs(self.r_collision_base)
        sign = -1.0 if self.r_collision_base <= 0.0 else 1.0
        return sign * mag

    def _get_angular_velocity(self):
        try:
            resp = self._get_state(self.model_name, "world")
            if resp.success:
                return float(resp.twist.angular.z)
        except rospy.ServiceException:
            pass
        return float(self._prev_action[1])

    def _forward_effort(self, vx, w_abs):
        """DRL_Navigation_Robot_ROS2_Foxy style: encourage v - |ω|."""
        return max(0.0, float(vx) - float(w_abs))

    def _compute_motion_rewards(self, vx, w):
        """Per-step forward encouragement and spin-in-place penalty."""
        w_abs = abs(float(w))
        effort = self._forward_effort(vx, w_abs)
        r_forward = self.r_forward_scale * effort
        r_spin = 0.0
        if float(vx) < self.spin_v_thresh and w_abs > self.spin_w_thresh:
            r_spin = -self.r_spin_penalty * (w_abs - self.spin_w_thresh)
        return r_forward, r_spin, effort

    def _update_low_effort_penalty(self, effort):
        """Sustained low v-|ω| → extra per-step penalty (early spin/stuck warning)."""
        now = rospy.Time.now().to_sec()
        self._low_effort_hist.append((now, float(effort)))
        cutoff = now - self.low_effort_time_sec
        self._low_effort_hist = [
            (t, e) for t, e in self._low_effort_hist if t >= cutoff
        ]
        if len(self._low_effort_hist) < 2:
            return 0.0
        t0, _ = self._low_effort_hist[0]
        if now - t0 < 0.85 * self.low_effort_time_sec:
            return 0.0
        if all(e < self.low_effort_thresh for _, e in self._low_effort_hist):
            return self.r_low_effort_penalty
        return 0.0

    def set_state_noise_std(self, std):
        """Trainer may adapt observation noise at runtime."""
        self.state_noise_std = max(0.0, float(std))

    def _front_layer_mins(self):
        """Per-elevation-bin min range over front sectors (collision / terrain)."""
        if self._elevation_features is None:
            return None
        data = self._elevation_features
        n_bins = self.n_elevation_bins
        n_front = self.n_elev_front_sectors
        expected = n_bins * n_front
        if data.size >= expected:
            mins = []
            for b in range(n_bins):
                sl = data[b * n_front : (b + 1) * n_front]
                mins.append(float(np.min(sl)) if sl.size else self.max_range)
            return mins
        if data.size >= n_bins:
            return [float(data[i]) for i in range(n_bins)]
        return None

    def _format_layer_mins(self, layer_mins):
        if not layer_mins:
            return "?"
        return "[" + ",".join("%.2f" % m for m in layer_mins) + "]"

    def _classify_obstacle(self, vx, vy, min_d, v_app):
        """
        Distinguish slope (brief / upper-clear / pitch-up) vs wall (multi-layer near,
        sustained, speed drop). Returns ('wall'|'slope'|'none', layer_mins_or_None).

        Record 4 配置: >= wall_layers_required elevation layers near -> wall (not all 4);
        min_d < wall_force_min_dist -> force wall unconditionally (bypass slope).
        """
        layer_mins = self._front_layer_mins()
        if layer_mins is None:
            if min_d < self.collision_dist and v_app > self.collision_v_min:
                if self._proximity_frame_count >= self.collision_frames:
                    return "wall", None
            return "none", None

        n = len(layer_mins)
        wall_d = self.wall_all_layers_dist
        lower_d = self.wall_lower_layers_dist
        upper_clear = self.slope_upper_clear_dist

        layer_close_count = sum(1 for m in layer_mins if m < wall_d)
        layers_wall_pattern = layer_close_count >= self.wall_layers_required
        lower_layers_low = (
            layer_mins[0] < lower_d and layer_mins[1] < lower_d
        ) if n >= 2 else False
        mid_low = layer_mins[2] < lower_d if n >= 3 else False
        upper_clear_enough = layer_mins[-1] >= upper_clear

        pitch_rise = False
        if self.use_imu_pitch and self._pitch_at_proximity is not None:
            pitch_delta = math.degrees(self._pitch - self._pitch_at_proximity)
            pitch_rise = pitch_delta >= self.slope_pitch_delta_deg

        brief_proximity = self._proximity_frame_count <= self.slope_max_frames

        # Extremely close: force wall — bypass slope / layer pattern (no frame or vx gate)
        if min_d < self.wall_force_min_dist:
            return "wall", layer_mins

        slope_pattern = (
            (lower_layers_low or mid_low)
            and upper_clear_enough
            and not layers_wall_pattern
        )
        # Slope: require terrain pattern AND (brief proximity OR pitch rise).
        # Do NOT treat "still moving forward" alone as slope — that suppressed walls.
        if slope_pattern and (brief_proximity or pitch_rise):
            return "slope", layer_mins

        if layers_wall_pattern and (
            self._proximity_frame_count >= self.collision_frames
            or vx <= self.wall_v_forward_max
        ):
            return "wall", layer_mins

        if (lower_layers_low or mid_low) and not upper_clear_enough:
            if self._proximity_frame_count >= self.collision_frames:
                return "wall", layer_mins

        if (
            not slope_pattern
            and self._proximity_frame_count >= self.collision_frames
            and min_d < self.collision_dist
        ):
            return "wall", layer_mins

        return "none", layer_mins

    def _reset_proximity_tracking(self):
        self._proximity_frame_count = 0
        self._pitch_at_proximity = None

    def _update_collision(self, vx, vy):
        """
        Front-sector proximity + approach speed + slope/wall classifier.
        Returns (event_fired, penalty, v_app, min_d).
        Slope: light r_slope_penalty, no collision event count.
        Wall: debounced collision_frames, escalating penalty.
        """
        best = None
        for i in range(self.n_sectors):
            if not self._is_front_sector(i):
                continue
            d = float(self._features[i])
            if d >= self.collision_dist:
                continue
            bearing = self._sector_bearing(i)
            v_app = self._approach_speed(vx, vy, bearing)
            if v_app <= self.collision_v_min:
                continue
            if best is None or d < best[0]:
                best = (d, v_app)

        if best is None:
            self._collision_streak = 0
            self._reset_proximity_tracking()
            return False, 0.0, 0.0, self.max_range

        min_d, v_app = best
        self._proximity_frame_count += 1
        if self._pitch_at_proximity is None:
            self._pitch_at_proximity = self._pitch

        if not self.terrain_classify_enable:
            self._collision_streak += 1
            if self._collision_streak < self.collision_frames:
                return False, 0.0, v_app, min_d
            self._collision_streak = 0
            self._reset_proximity_tracking()
            self._collision_event_count += 1
            k = self._collision_event_count
            penalty = self._collision_penalty_for_event(k)
            rospy.loginfo(
                "[collision] wall confirmed event #%d penalty=%+.1f v_app=%.3f min_d=%.3f",
                k,
                penalty,
                v_app,
                min_d,
            )
            return True, penalty, v_app, min_d

        terrain, layer_mins = self._classify_obstacle(vx, vy, min_d, v_app)
        layers_s = self._format_layer_mins(layer_mins)

        if terrain == "slope":
            self._collision_streak = 0
            penalty = self.r_slope_penalty
            rospy.loginfo_throttle(
                1.0,
                "[terrain] slope suspected min_d=%.3f layers=%s pitch=%.1f° v_fwd=%.3f frames=%d",
                min_d,
                layers_s,
                math.degrees(self._pitch),
                vx,
                self._proximity_frame_count,
            )
            return False, penalty, v_app, min_d

        if terrain != "wall":
            return False, 0.0, v_app, min_d

        self._collision_streak += 1
        if self._collision_streak < self.collision_frames:
            return False, 0.0, v_app, min_d

        self._collision_streak = 0
        self._reset_proximity_tracking()
        self._collision_event_count += 1
        k = self._collision_event_count
        penalty = self._collision_penalty_for_event(k)
        rospy.loginfo(
            "[collision] wall confirmed event #%d penalty=%+.1f v_app=%.3f min_d=%.3f layers=%s",
            k,
            penalty,
            v_app,
            min_d,
            layers_s,
        )
        return True, penalty, v_app, min_d

    def _update_stuck(self, x, y):
        """Sliding window displacement check. Returns (is_stuck, displacement_m)."""
        now = rospy.Time.now().to_sec()
        self._stuck_hist.append((now, float(x), float(y)))
        cutoff = now - self.stuck_time_sec
        self._stuck_hist = [(t, px, py) for t, px, py in self._stuck_hist if t >= cutoff]
        if len(self._stuck_hist) < 2:
            return False, 0.0
        t0, x0, y0 = self._stuck_hist[0]
        span = now - t0
        if span < 0.85 * self.stuck_time_sec:
            return False, 0.0
        disp = math.hypot(x - x0, y - y0)
        if disp >= self.stuck_dist_threshold:
            return False, disp
        rospy.loginfo(
            "[stuck] displacement=%.3fm in %.1fs (threshold=%.2fm/%.1fs) → done",
            disp,
            span,
            self.stuck_dist_threshold,
            self.stuck_time_sec,
        )
        return True, disp

    def _rollover_angles_deg(self):
        """|roll|, |pitch| in degrees; IMU primary, Gazebo model orientation fallback."""
        tag = self._rollover_orientation_tag()
        if tag == "ok":
            return abs(math.degrees(self._roll)), abs(math.degrees(self._pitch))
        if tag == "fallback":
            m_roll, m_pitch = self._model_orientation_deg()
            if m_roll is not None:
                return m_roll, m_pitch
            if self._imu_received:
                return abs(math.degrees(self._roll)), abs(math.degrees(self._pitch))
        elif self._imu_received:
            return abs(math.degrees(self._roll)), abs(math.degrees(self._pitch))
        return 0.0, 0.0

    def _is_rollover_tilt(self, roll_deg, pitch_deg):
        if roll_deg >= self.rollover_roll_deg or pitch_deg >= self.rollover_pitch_deg:
            return True
        if self.rollover_sum_deg > 0.0:
            return (roll_deg + pitch_deg) >= self.rollover_sum_deg
        return False

    def _log_rollover_debug(self, roll_deg, pitch_deg, ep_elapsed_sec):
        if roll_deg + pitch_deg < 25.0:
            return
        imu_src = self._rollover_orientation_tag()
        rospy.logdebug_throttle(
            2.0,
            "[rollover] tilt roll=%.1f° pitch=%.1f° sum=%.1f° "
            "(thresh roll=%.0f pitch=%.0f sum=%.0f) ep_t=%.1fs src=%s",
            roll_deg,
            pitch_deg,
            roll_deg + pitch_deg,
            self.rollover_roll_deg,
            self.rollover_pitch_deg,
            self.rollover_sum_deg,
            ep_elapsed_sec,
            imu_src,
        )

    def _check_spawn_grace_flip(self, ep_elapsed_sec):
        """Spawn grace window: flip → abort episode (resample), no rollover outcome."""
        if not self.rollover_enable or not self.rollover_grace_abort_enable:
            return False
        if ep_elapsed_sec >= self.rollover_grace_sec:
            return False
        roll_deg, pitch_deg = self._rollover_angles_deg()
        if self._is_rollover_tilt(roll_deg, pitch_deg):
            rospy.loginfo(
                "[rollover] spawn grace flip roll=%.1f° pitch=%.1f° sum=%.1f° "
                "ep_t=%.1fs → abort episode (resample)",
                roll_deg,
                pitch_deg,
                roll_deg + pitch_deg,
                ep_elapsed_sec,
            )
            return True
        self._log_rollover_debug(roll_deg, pitch_deg, ep_elapsed_sec)
        return False

    def _check_rollover(self, ep_elapsed_sec):
        """IMU roll/pitch flip detection after spawn grace window."""
        if not self.rollover_enable:
            return False
        if ep_elapsed_sec < self.rollover_grace_sec:
            return False
        roll_deg, pitch_deg = self._rollover_angles_deg()
        if self._is_rollover_tilt(roll_deg, pitch_deg):
            rospy.loginfo(
                "[rollover] roll=%.1f° pitch=%.1f° sum=%.1f° ep_t=%.1fs "
                "(grace=%.1fs imu=%s) → done",
                roll_deg,
                pitch_deg,
                roll_deg + pitch_deg,
                ep_elapsed_sec,
                self.rollover_grace_sec,
                self._rollover_orientation_tag(),
            )
            return True
        self._log_rollover_debug(roll_deg, pitch_deg, ep_elapsed_sec)
        return False

    def _validate_goal(self, gx, gy, sx, sy, syaw):
        """
        Temporarily place robot at goal; verify clearance / z sanity, then restore start.
        Rejects goals on boxes or inside geometry.
        """
        z_base = self._spawn_z_for_current_world()
        self._set_pose(float(gx), float(gy), float(syaw), z=self.spawn_z)
        _wall_sleep(max(0.2, self.spawn_check_wait * 0.6))
        ok, reason, z, _vz, min_d = self._check_spawn_stable(z_base)
        self._set_pose(float(sx), float(sy), float(syaw), z=self.spawn_z)
        _wall_sleep(0.1)
        if not ok:
            return False, reason
        if min_d is not None and min_d < self.goal_min_clearance:
            return False, "goal_blocked"
        if z is not None and abs(z - z_base) > self.goal_z_max_delta:
            return False, "goal_z_bad"
        return True, "ok"

    def _wait_lidar_settle(self, timeout=2.0):
        """Wait until range features are not all saturated (fresh /scan)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._min_sector_dist() < self.max_range * 0.99:
                return True
            _wall_sleep(0.1)
        return False

    def _check_spawn_stable(self, z_target):
        """
        After pose set: wall-clock settle, then validate z/vz/pose stability and clearance.
        Returns (ok, reason, z, vz, min_d).
        """
        _wall_sleep(self.spawn_check_wait)

        z1, vz1 = self._get_model_z_vz()
        if z1 is None:
            return False, "pose_unreadable", None, None, None

        _wall_sleep(0.15)
        z2, vz2 = self._get_model_z_vz()
        if z2 is None:
            return False, "pose_lost", z1, vz1, None

        if abs(z2 - z1) > self.spawn_pose_stable_threshold:
            return False, "z_unstable", z2, vz2, None

        z = z2
        vz = vz2 if vz2 is not None else 0.0
        vz_abs = abs(vz)
        dz = abs(z - z_target)

        if dz > self.spawn_z_jump_threshold:
            z_world = self._spawn_z_for_current_world()
            # Heightmap settle (bumpy z≈1.06 vs target 1.05) must not fail when
            # the world identity is correct. Also absorb a stale z_try if
            # current_world/world_spawn_z already matches the measured z.
            if abs(z - z_world) <= self.spawn_z_jump_threshold:
                rospy.logwarn(
                    "[spawn] z=%.3f vs target_z=%.3f exceeds jump thresh %.3f; "
                    "accepting world_spawn_z=%.3f (%s)",
                    z,
                    z_target,
                    self.spawn_z_jump_threshold,
                    z_world,
                    self._world_basename(),
                )
            else:
                return False, "z_jump", z, vz, None
        if vz_abs > self.spawn_vz_threshold:
            return False, "high_vz", z, vz, None

        lidar_ok = self._wait_lidar_settle(timeout=2.0)
        min_d = self._min_sector_dist()
        if lidar_ok and min_d < self.spawn_collision_dist:
            return False, "stuck_collision", z, vz, min_d
        if not lidar_ok:
            rospy.logwarn_throttle(
                5.0,
                "[spawn] lidar not settled — skipping stuck_collision check (min_d=%.2f)",
                min_d,
            )

        return True, "ok", z, vz, min_d

    def _clip_spawn_xy(self, x, y):
        x = float(np.clip(x, self.spawn_xy_min[0], self.spawn_xy_max[0]))
        y = float(np.clip(y, self.spawn_xy_min[1], self.spawn_xy_max[1]))
        return x, y

    def _place_robot_random_xy(self, context="spawn", n_tries=None):
        """Try origin then many random (x,y). Never fatal — False means keep resampling."""
        n = max(1, int(n_tries if n_tries is not None else self.spawn_xy_max_retries))
        yaw0 = 0.0
        if self._ensure_spawn_ok(0.0, 0.0, yaw0, context=context, allow_xy_retry=False):
            return True
        for i in range(n):
            start = self._sample_xy(self.spawn_xy_min, self.spawn_xy_max)
            yaw = float(np.random.uniform(-math.pi, math.pi))
            if self._ensure_spawn_ok(
                float(start[0]),
                float(start[1]),
                yaw,
                context=context,
                allow_xy_retry=False,
            ):
                return True
            if i == 0 or (i + 1) % 10 == 0:
                rospy.logwarn(
                    "[spawn] random XY %d/%d still blocked (world=%s context=%s)",
                    i + 1,
                    n,
                    self._world_basename(),
                    context,
                )
        rospy.logerr(
            "[spawn] no clear XY after origin + %d samples (world=%s context=%s) "
            "— give up this map next, do NOT abort training",
            n,
            self._world_basename(),
            context,
        )
        return False

    def _ensure_spawn_ok(self, x, y, yaw, context="reset", allow_xy_retry=True):
        """
        Gate before step(): retry this XY (z bump / local jitter) a few times.
        Returns True only when safe. False = caller must pick another XY.
        Does not abort training.
        """
        # Re-read identity every attempt (Record 6: stale playpen name + z=0.20
        # while gzserver already had bumpy heightmap z≈1.06).
        self.spawn_z = self._spawn_z_for_current_world()
        z_try = self.spawn_z
        x_try, y_try = float(x), float(y)
        max_attempts = self.spawn_z_max_retries + 1
        last_reason = "unknown"

        if not self._wait_for_model(timeout=30.0):
            rospy.logwarn(
                "[spawn] model %s not in Gazebo after 30s (context=%s)",
                self.model_name,
                context,
            )

        for attempt in range(max_attempts):
            world_name = self._world_basename()
            self.spawn_z = self._spawn_z_for_current_world()
            self._set_pose(x_try, y_try, yaw, z=z_try)
            ok, reason, z, vz, min_d = self._check_spawn_stable(z_try)
            if ok:
                self.spawn_z = z_try
                rospy.loginfo(
                    "[spawn] OK world=%s context=%s z=%.3f min_d=%.2fm "
                    "attempt=%d/%d",
                    world_name,
                    context,
                    z,
                    min_d if min_d is not None else -1.0,
                    attempt + 1,
                    max_attempts,
                )
                return True

            last_reason = reason
            vz_s = "%.3f" % vz if vz is not None else "?"
            min_d_s = "%.2f" % min_d if min_d is not None else "?"
            rospy.logwarn(
                "[spawn] FAIL world=%s context=%s attempt=%d/%d reason=%s "
                "z=%s vz=%s min_d=%s target_z=%.3f xy=(%.2f,%.2f)",
                world_name,
                context,
                attempt + 1,
                max_attempts,
                reason,
                "%.3f" % z if z is not None else "?",
                vz_s,
                min_d_s,
                z_try,
                x_try,
                y_try,
            )

            # Trees / holes: new XY immediately. Raising z helps stuck_collision only.
            if reason == "stuck_collision":
                z_try += self.spawn_z_retry_delta
            else:
                z_try = self._spawn_z_for_current_world()
            if allow_xy_retry:
                x_try, y_try = self._clip_spawn_xy(
                    x_try
                    + np.random.uniform(-self.spawn_xy_jitter, self.spawn_xy_jitter),
                    y_try
                    + np.random.uniform(-self.spawn_xy_jitter, self.spawn_xy_jitter),
                )

        rospy.logwarn(
            "[spawn] XY cluster done world=%s context=%s after %d attempts "
            "(last=%s) — resample another (x,y)",
            self._world_basename(),
            context,
            max_attempts,
            last_reason,
        )
        return False

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _sample_xy(self, lo, hi):
        return np.array(
            [
                np.random.uniform(lo[0], hi[0]),
                np.random.uniform(lo[1], hi[1]),
            ],
            dtype=np.float64,
        )

    def _goal_polar(self, x, y, yaw):
        dx = self._goal[0] - x
        dy = self._goal[1] - y
        rho = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        theta = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        return rho, theta

    def _explore_cell_key(self, x, y):
        res = max(self.explore_grid_res, 1e-6)
        return (int(math.floor(float(x) / res)), int(math.floor(float(y) / res)))

    def _compute_explore_reward(self, x, y, rho):
        """First-visit occupancy cell bonus; revisits score 0 (no farming).

        Cardiff RL navigation (arXiv:2209.14271): r_Gd = G/d with G=1 on a new
        grid cell and d = distance to goal (~0.05–0.5/step when d=2–10 m).
        Default flat explore_cell_reward (0.03) is intentionally much smaller.
        """
        if not self.explore_enable or self.explore_cell_reward <= 0.0:
            return 0.0
        cell = self._explore_cell_key(x, y)
        if cell in self._explore_cells:
            return 0.0
        self._explore_cells.add(cell)
        reward = self.explore_cell_reward
        if self.explore_cardiff_scale:
            reward /= max(float(rho), self.explore_rho_min)
        return reward

    def _build_state(self, x, y, yaw):
        rho, theta = self._goal_polar(x, y, yaw)
        sector_dists = self._td3_sector_distances()
        d_norm = np.clip(sector_dists / self.max_range, 0.0, 1.0)
        if self.state_noise_std > 0.0:
            noise = np.random.normal(0.0, self.state_noise_std, size=d_norm.shape)
            d_norm = np.clip(d_norm + noise, 0.0, 1.0)
        rho_n = min(rho / 10.0, 1.0)
        v_n = self._prev_action[0] / max(self.max_v, 1e-6)
        w_n = self._prev_action[1] / max(self.max_w, 1e-6)
        tail = np.array([rho_n, theta / math.pi, v_n, w_n], dtype=np.float32)
        if self.state_noise_std > 0.0:
            tail += np.random.normal(0.0, self.state_noise_std, size=tail.shape)
            tail = np.clip(tail, -1.0, 1.0)
        return np.concatenate([d_norm, tail]).astype(np.float32)

    def _gzserver_clean_prefix(self):
        try:
            import rospkg

            return os.path.join(
                rospkg.RosPack().get_path("scout_mini_pro_description"),
                "scripts",
                "gzserver_clean_prefix.sh",
            )
        except Exception:
            return ""

    def _shutdown_owned_gazebo(self):
        if not self._owns_gz and self._gz_proc is None and self._gzclient_proc is None:
            return
        for proc in (self._gzclient_proc, self._gz_proc):
            _kill_proc_group(proc)
        self._gz_proc = None
        self._gzclient_proc = None
        self._owns_gz = False

    def _wait_gazebo_unregistered(self, timeout=15.0):
        """
        Wait until /gazebo is gone from the ROS master.

        After kill -9, the node often stays registered as stale. Restarting too
        soon reuses the XML-RPC port: the master sends 'new node same name'
        shutdown to the *new* gzserver → spawn services never appear.
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            if not _rosnode_alive("/gazebo"):
                # Brief settle so the old XML-RPC socket is released
                _wall_sleep(0.4)
                if not _rosnode_alive("/gazebo"):
                    return True
            # Drop dead registrations (cleanup is interactive — pipe yes)
            try:
                subprocess.call(
                    ["bash", "-c", "yes | rosnode cleanup"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=12,
                )
            except Exception:
                pass
            _wall_sleep(0.4)
        still = _rosnode_alive("/gazebo")
        rospy.logwarn(
            "[world] /gazebo still in rosnode list after %.1fs (stale=%s)",
            timeout,
            still,
        )
        return not still

    def _kill_gazebo_processes(self):
        """Stop launch-owned or previously spawned gzserver/gzclient."""
        rospy.loginfo("[world] killing gzserver/gzclient before reload...")
        self._shutdown_owned_gazebo()
        for node in ("/gazebo", "/gazebo_gui"):
            try:
                subprocess.call(
                    ["rosnode", "kill", node],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5,
                )
            except Exception:
                pass
        _wall_sleep(self.world_switch_kill_wait_sec)
        _pkill_patterns(
            [
                "gazebo_ros/gzserver",
                "gazebo_ros/gzclient",
                "gzserver_clean_prefix.sh",
            ]
        )
        try:
            subprocess.call(
                ["killall", "-9", "gzserver", "gzclient"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass
        for _ in range(80):
            if not _port_11345_busy() and not _gzserver_wrapper_pids():
                break
            _wall_sleep(0.1)
        if _port_11345_busy():
            rospy.logwarn("[world] port 11345 still busy after kill; gzserver may fail")
        # Critical: do not start a new /gazebo until master dropped the old one
        ok = self._wait_gazebo_unregistered(timeout=20.0)
        if ok:
            rospy.loginfo("[world] old /gazebo unregistered; port ready")
        else:
            rospy.logwarn(
                "[world] proceeding despite stale /gazebo — watch for name-conflict shutdown"
            )

    def _gazebo_switch_log_path(self):
        return os.path.join(
            os.path.expanduser("~"), ".ros", "scout_offroad_td3_gzserver_switch.log"
        )

    def _start_gzserver(self, world_path):
        prefix = self._gzserver_clean_prefix()
        # Match roslaunch: direct gzserver script + __name:=gazebo (not rosrun).
        gzserver_bin = "/opt/ros/noetic/lib/gazebo_ros/gzserver"
        gz_args = ["-e", "ode", world_path, "__name:=gazebo"]
        if prefix and os.path.isfile(prefix):
            cmd = ["bash", prefix, gzserver_bin] + gz_args
        else:
            cmd = [gzserver_bin] + gz_args
        rospy.loginfo("[world] starting: %s", " ".join(cmd))
        log_path = self._gazebo_switch_log_path()
        try:
            log_f = open(log_path, "a")
            log_f.write(
                "\n--- %s ---\n%s\n"
                % (time.strftime("%Y-%m-%d %H:%M:%S"), " ".join(cmd))
            )
            log_f.flush()
        except Exception:
            log_f = subprocess.DEVNULL
        self._gz_proc = subprocess.Popen(
            cmd,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid if hasattr(os, "setsid") else None,
        )
        self._owns_gz = True
        # gzclient is started only after /gazebo/spawn_urdf_model is up — see
        # _start_gzclient(). Starting it in parallel loads libgazebo_ros_api_plugin
        # without __name:=gazebo_gui and steals /gazebo from gzserver.

    def _start_gzclient(self):
        if not self._want_gui:
            return
        if self._gzclient_proc is not None and self._gzclient_proc.poll() is None:
            return
        gzclient_bin = "/opt/ros/noetic/lib/gazebo_ros/gzclient"
        try:
            self._gzclient_proc = subprocess.Popen(
                [gzclient_bin, "__name:=gazebo_gui"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid if hasattr(os, "setsid") else None,
            )
            rospy.loginfo("[world] gzclient started (__name:=gazebo_gui)")
        except Exception as e:
            rospy.logwarn("[world] gzclient start failed: %s", e)

    def _gz_child_ok(self):
        """False if we own gzserver and the child has already exited."""
        if self._gz_proc is None:
            return True
        code = self._gz_proc.poll()
        if code is not None:
            return False
        return True

    def _spawn_service_ready(self):
        """True if /gazebo/spawn_urdf_model is registered (quick poll)."""
        try:
            rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=0.15)
            return True
        except (rospy.ROSException, rospy.ROSInterruptException):
            return False

    def _wait_spawn_service(self, timeout=60.0):
        """
        Wait for /gazebo/spawn_urdf_model with wall-clock timeout.
        Fail fast if owned gzserver exits (e.g. name-conflict shutdown).
        """
        svc = "/gazebo/spawn_urdf_model"
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < timeout:
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown while waiting for %s" % svc)
            if not self._gz_child_ok():
                raise RuntimeError(
                    "gzserver exited (code=%s) before %s appeared — "
                    "see ~/.ros/scout_offroad_td3_gzserver_switch.log "
                    "(often: /gazebo name conflict / stale master entry)"
                    % (self._gz_proc.returncode if self._gz_proc else "?", svc)
                )
            elapsed = time.time() - t0
            if elapsed > 8.0 and self._gz_child_ok():
                raise RuntimeError(
                    "gzserver alive but %s missing after %.0fs — likely /gazebo "
                    "name conflict (gui gzclient without __name:=gazebo_gui?) — "
                    "see %s"
                    % (svc, elapsed, self._gazebo_switch_log_path())
                )
            try:
                rospy.wait_for_service(svc, timeout=1.0)
                elapsed = time.time() - t0
                rospy.loginfo("[world] %s ready (%.1fs)", svc, elapsed)
                return
            except rospy.ROSException:
                pass
            now = time.time()
            if now - last_log >= 5.0:
                rospy.loginfo(
                    "[world] waiting for %s ... %.0fs/%.0fs (gz_alive=%s)",
                    svc,
                    now - t0,
                    timeout,
                    self._gz_child_ok(),
                )
                last_log = now
            _wall_sleep(0.2)
        raise RuntimeError(
            "timeout (%.0fs wall) waiting for %s (gz_alive=%s). "
            "Check ~/.ros/scout_offroad_td3_gzserver_switch.log"
            % (timeout, svc, self._gz_child_ok())
        )

    def _spawn_robot(self, world_path=None):
        if not rospy.has_param("robot_description"):
            raise RuntimeError("robot_description param missing; cannot respawn")
        urdf = rospy.get_param("robot_description")
        if world_path:
            self._apply_world_identity(world_path)
        self.spawn_z = self._spawn_z_for_current_world()
        self._wait_spawn_service(timeout=60.0)
        try:
            rospy.wait_for_service("/gazebo/delete_model", timeout=5.0)
            delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            delete(self.model_name)
            _wall_sleep(0.3)
        except Exception:
            pass

        pose = Pose()
        pose.position = Point(0.0, 0.0, self.spawn_z)
        pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        spawn = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        world_name = self._world_basename()
        rospy.loginfo(
            "[world] spawning model %s at z=%.3f (world=%s) ...",
            self.model_name,
            self.spawn_z,
            world_name,
        )
        resp = spawn(self.model_name, urdf, "", pose, "world")
        if not resp.success:
            raise RuntimeError("spawn_urdf_model failed: %s" % resp.status_message)
        rospy.loginfo("[world] spawn OK: %s", resp.status_message)

        try:
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=10.0)
            rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
        except Exception as e:
            rospy.logwarn("[world] unpause_physics: %s", e)

        # Origin is often inside trees/terrain. Retry many random XY on THIS world.
        # Do not raise — that used to abort the train loop / lock playpen.
        if not self._place_robot_random_xy(context="spawn"):
            rospy.logerr(
                "[spawn] world spawn: no clear XY on %s (spawn_z=%.3f) — "
                "reset() will switch map after pose exhaust, not abort",
                world_name,
                self.spawn_z,
            )

    def switch_world(self, world_path, timestep=None):
        """
        Reload a different .world by restarting gzserver (Classic cannot hot-swap
        world files). Does not clear the trainer's replay buffer.
        """
        world_path = os.path.abspath(world_path)
        if not os.path.isfile(world_path):
            raise RuntimeError("world not found: %s" % world_path)

        self._stop()
        base = os.path.basename(world_path)
        previous_world = self.current_world
        self._world_before_switch = previous_world
        # Commit identity BEFORE gzserver spawn validation (Record 6 smoking gun).
        self._apply_world_identity(world_path)
        if timestep is None:
            rospy.loginfo("[world] switch to %s", base)
        else:
            rospy.loginfo("[world] switch to %s at t=%d", base, int(timestep))
        rospy.loginfo(
            "[world] spawn_z for %s is %.3f (previous=%s)",
            base,
            self.spawn_z,
            os.path.basename(previous_world) if previous_world else "(none)",
        )

        last_err = None
        max_retries = self.world_switch_max_retries
        for attempt in range(1, max_retries + 1):
            try:
                rospy.loginfo(
                    "[world] reload attempt %d/%d for %s", attempt, max_retries, base
                )
                self._kill_gazebo_processes()
                self._start_gzserver(world_path)

                # Fail fast if child dies; exit boot loop early when spawn svc up
                t0 = time.time()
                boot_limit = self.world_switch_gzserver_boot_sec
                while time.time() - t0 < boot_limit:
                    if not self._gz_child_ok():
                        raise RuntimeError(
                            "gzserver exited early (code=%s) loading %s — "
                            "see ~/.ros/scout_offroad_td3_gzserver_switch.log"
                            % (
                                self._gz_proc.returncode if self._gz_proc else "?",
                                base,
                            )
                        )
                    if self._spawn_service_ready():
                        rospy.loginfo(
                            "[world] gzserver boot OK (%.1fs); spawn service up",
                            time.time() - t0,
                        )
                        break
                    _wall_sleep(0.15)
                else:
                    rospy.logwarn(
                        "[world] boot window %.1fs without spawn service — "
                        "continuing to _wait_spawn_service",
                        boot_limit,
                    )

                self._wait_spawn_service(timeout=self.world_switch_spawn_wait_sec)
                self._start_gzclient()
                rospy.wait_for_service("/gazebo/get_model_state", timeout=30.0)
                rospy.wait_for_service("/gazebo/set_model_state", timeout=15.0)
                self._bind_model_services()
                self._spawn_robot(world_path)
                self._reset_imu_state()
                self._ensure_imu_subscriber()
                _wall_sleep(0.6)

                # Confirm model is queryable before returning to the train loop
                self._wait_pose(timeout=20.0)
                self._wait_imu_after_respawn()
                self._post_switch_model_only_steps = (
                    self.imu_model_fallback_after_switch_steps
                )
                self._log_imu_post_switch_status()

                try:
                    rospy.set_param("/offroad/current_world", world_path)
                except Exception:
                    pass
                # Clear stale lidar cache until fresh scans arrive
                self._features = np.full(self.n_sectors, self.max_range, dtype=np.float32)
                self._td3_features = np.full(
                    self._td3_feature_count, self.max_range, dtype=np.float32
                )
                self._odom_pose = None
                rospy.loginfo("[world] switch complete: %s (scout_mini ready)", base)
                return world_path
            except Exception as e:
                last_err = e
                rospy.logerr(
                    "[world] reload attempt %d/%d failed: %s",
                    attempt,
                    max_retries,
                    e,
                )
                # Ensure dead child is cleaned before retry
                try:
                    self._kill_gazebo_processes()
                except Exception:
                    pass
                _wall_sleep(self.world_switch_retry_sleep_sec)

        # Keep intended identity (do NOT revert to playpen). Recovery/reset
        # retry poses on this world instead of locking the previous map.
        raise RuntimeError(
            "world switch failed after %d attempts (%s): %s"
            % (max_retries, base, last_err)
        )

    def _sleep_control_dt(self):
        """
        Wait ~control_dt of sim time, but never hang if /clock is frozen
        (gzserver dead after a failed world switch).
        """
        target = float(self.control_dt)
        t_wall0 = time.time()
        t_ros0 = rospy.Time.now()
        wall_limit = max(2.0, target * 50.0)
        while time.time() - t_wall0 < wall_limit:
            try:
                elapsed = (rospy.Time.now() - t_ros0).to_sec()
            except Exception:
                elapsed = 0.0
            if elapsed >= target:
                return
            _wall_sleep(0.01)
        rospy.logwarn_throttle(
            5.0,
            "sim clock not advancing during step (gzserver down?); moved on after %.1fs wall",
            wall_limit,
        )

    def _try_spawn_start_goal(self, n_xy):
        """Sample start/goal on the CURRENT world. Returns (start, goal, yaw) or None."""
        world_name = self._world_basename()
        start = goal = None
        yaw = 0.0
        for xy_try in range(n_xy):
            for _ in range(40):
                start = self._sample_xy(self.spawn_xy_min, self.spawn_xy_max)
                goal = self._sample_xy(self.goal_xy_min, self.goal_xy_max)
                if (
                    math.hypot(goal[0] - start[0], goal[1] - start[1])
                    >= self.min_goal_dist
                ):
                    break
            yaw = float(np.random.uniform(-math.pi, math.pi))
            dist = math.hypot(goal[0] - start[0], goal[1] - start[1])

            if xy_try == 0 or (xy_try + 1) % 10 == 0:
                rospy.loginfo(
                    "[ep=%d] world=%s start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f) dist=%.2fm "
                    "spawn_z=%.2f goal_tol=%.2f (xy_try=%d/%d)",
                    self._ep_count,
                    world_name,
                    start[0],
                    start[1],
                    yaw,
                    goal[0],
                    goal[1],
                    dist,
                    self._spawn_z_for_current_world(),
                    self.goal_tol,
                    xy_try + 1,
                    n_xy,
                )

            if not self._ensure_spawn_ok(
                float(start[0]),
                float(start[1]),
                yaw,
                context="reset",
                allow_xy_retry=True,
            ):
                rospy.logwarn(
                    "[spawn] reset xy_try %d/%d blocked — another random (x,y)",
                    xy_try + 1,
                    n_xy,
                )
                continue

            x, y, yaw_pose = self._wait_pose()
            goal_ok = True
            if self.goal_check_enable:
                goal_ok = False
                for g_try in range(self.goal_resample_max):
                    g_ok, g_reason = self._validate_goal(
                        goal[0], goal[1], x, y, yaw_pose
                    )
                    if g_ok:
                        goal_ok = True
                        if g_try > 0:
                            rospy.loginfo(
                                "[goal] OK after %d resamples goal=(%.2f,%.2f)",
                                g_try,
                                goal[0],
                                goal[1],
                            )
                        break
                    if g_try == 0 or (g_try + 1) % 5 == 0:
                        rospy.logwarn(
                            "[goal] reject reason=%s goal=(%.2f,%.2f) try=%d/%d",
                            g_reason,
                            goal[0],
                            goal[1],
                            g_try + 1,
                            self.goal_resample_max,
                        )
                    for _ in range(20):
                        goal = self._sample_xy(self.goal_xy_min, self.goal_xy_max)
                        if (
                            math.hypot(goal[0] - start[0], goal[1] - start[1])
                            >= self.min_goal_dist
                        ):
                            break
                if not goal_ok:
                    rospy.logwarn(
                        "[goal] no reachable goal xy_try=%d/%d — resample start/goal",
                        xy_try + 1,
                        n_xy,
                    )
                    continue

            return start, goal, yaw
        return None

    def _switch_world_after_spawn_exhaust(self, n_pose_tries, tried):
        """Leave this map after pose exhaust. Never raises / never exits training.

        Returns (abspath_or_None, loaded_ok).
        """
        nxt = self.pick_next_world(exclude=tried, require_different=True)
        if not nxt:
            return None, False
        rospy.logerr(
            "[spawn] giving up this world=%s after %d pose tries → switch to %s",
            self._world_basename(),
            int(n_pose_tries),
            os.path.basename(nxt),
        )
        try:
            self.switch_world(nxt)
            return os.path.abspath(nxt), True
        except Exception as e:
            rospy.logerr(
                "[spawn] switch to %s after spawn exhaust failed: %s "
                "— try another world, do NOT abort training",
                os.path.basename(nxt),
                e,
            )
            return os.path.abspath(nxt), False

    def reset(self, count_episode=True):
        self._stop()
        self._step = 0
        self._prev_action[:] = 0.0
        self._ep_reward_accum = 0.0
        self._ep_reward_parts = {k: 0.0 for k in REWARD_PART_KEYS}
        self._explore_cells = set()
        self._collision_event_count = 0
        self._collision_streak = 0
        self._reset_proximity_tracking()
        self._stuck_hist = []
        self._low_effort_hist = []
        self._initial_goal_dist = 0.0
        self._path_length = 0.0
        self._prev_pose_xy = None
        self._ep_start_time = time.time()
        self._reset_imu_state()

        n_xy = max(
            20, int(self.spawn_xy_max_retries), int(self.spawn_full_reset_retries)
        )
        tried = set()
        placed = None
        skip_pose = False
        while not rospy.is_shutdown():
            cur = os.path.abspath(self.current_world) if self.current_world else ""
            if cur:
                tried.add(cur)
            if not skip_pose:
                placed = self._try_spawn_start_goal(n_xy)
                if placed is not None:
                    break
            n_report = 0 if skip_pose else n_xy
            skip_pose = False
            path, loaded = self._switch_world_after_spawn_exhaust(n_report, tried)
            if path is None:
                rospy.logerr(
                    "[spawn] skip episode: all %d world(s) failed spawn this cycle "
                    "— wait, retry pool, do NOT abort training",
                    max(1, len(tried)),
                )
                _wall_sleep(2.0)
                return None
            tried.add(path)
            if not loaded:
                skip_pose = True

        if placed is None:
            return None

        _start, goal, _yaw = placed
        self._goal = goal

        if count_episode:
            self._ep_count += 1

        try:
            x, y, yaw = self._wait_pose(timeout=15.0)
        except RuntimeError as e:
            rospy.logerr(
                "[spawn] pose lost after placement (world=%s): %s — retry reset",
                self._world_basename(),
                e,
            )
            return None
        self._explore_cells.add(self._explore_cell_key(x, y))
        self._prev_dist = math.hypot(self._goal[0] - x, self._goal[1] - y)
        self._initial_goal_dist = float(self._prev_dist)
        self._path_length = 0.0
        self._prev_pose_xy = (float(x), float(y))
        g = Point(self._goal[0], self._goal[1], 0.0)
        self._goal_pub.publish(g)
        return self._build_state(x, y, yaw)

    def _log_reward(self, parts, rho, done, info):
        """Periodic / event reward log — not every step."""
        r_step = parts["r_step"]
        r_ep_start = parts.get("r_episode_start", 0.0)
        r_prog = parts["r_progress"]
        r_exp = parts.get("r_explore", 0.0)
        r_arr = parts["r_arrive"]
        r_fwd = parts.get("r_forward", 0.0)
        r_spin = parts.get("r_spin", 0.0)
        r_low = parts.get("r_low_effort", 0.0)
        r_col = parts["r_collision"]
        r_stk = parts.get("r_stuck", 0.0)
        step_r = (
            r_step
            + r_ep_start
            + r_prog
            + r_exp
            + r_arr
            + r_fwd
            + r_spin
            + r_low
            + r_col
            + r_stk
        )
        self._ep_reward_accum += step_r

        if done:
            outcome = (
                "success"
                if info.get("success")
                else (
                    "rollover"
                    if info.get("rollover")
                    else (
                        "stuck"
                        if info.get("stuck")
                        else (
                            "collision"
                            if info.get("collision_limit")
                            else "timeout" if info.get("timeout") else "done"
                        )
                    )
                )
            )
            bits = ["step %+.3f" % r_step]
            if r_ep_start:
                bits.append("ep_start %+.1f" % r_ep_start)
            if abs(r_prog) > 1e-6:
                bits.append(
                    "approach %+.3f (Δρ=%+.3f)" % (r_prog, parts.get("delta_rho", 0.0))
                )
            if abs(r_exp) > 1e-6:
                bits.append("explore %+.4f" % r_exp)
            if abs(r_fwd) > 1e-6:
                bits.append("forward %+.3f" % r_fwd)
            if abs(r_spin) > 1e-6:
                bits.append("spin %+.3f" % r_spin)
            if abs(r_low) > 1e-6:
                bits.append("low_effort %+.3f" % r_low)
            if r_arr:
                bits.append("arrive %+.1f" % r_arr)
            if r_col:
                bits.append("collision %+.1f" % r_col)
            r_roll = parts.get("r_rollover", 0.0)
            if r_roll:
                bits.append("rollover %+.1f" % r_roll)
            if r_stk:
                bits.append("stuck %+.1f" % r_stk)
            r_to = parts.get("r_timeout", 0.0)
            if r_to:
                bits.append("timeout %+.1f" % r_to)
            n_coll = int(info.get("collision_events", 0))
            if n_coll:
                bits.append("coll_events=%d" % n_coll)
            rospy.loginfo(
                "[reward] ep=%d step=%d END %s | %s | rho=%.2fm total_ep=%+.2f",
                self._ep_count - 1,
                self._step,
                outcome,
                " | ".join(bits),
                rho,
                self._ep_reward_accum,
            )
            return

        is_event = (
            abs(r_prog) > 0.3
            or abs(r_exp) > 1e-6
            or abs(r_fwd) > 0.05
            or abs(r_spin) > 0.05
            or abs(r_low) > 0.0
            or abs(r_col) > 0.0
            or abs(r_stk) > 0.0
            or abs(parts.get("r_rollover", 0.0)) > 0.0
            or abs(parts.get("r_timeout", 0.0)) > 0.0
        )
        periodic = (
            self.reward_log_interval > 0
            and self._step > 0
            and (self._step % self.reward_log_interval) == 0
        )
        if not (is_event or periodic):
            return

        msg = "[reward] ep=%d step=%d step_r=%+.3f" % (
            self._ep_count - 1,
            self._step,
            step_r,
        )
        if abs(r_prog) > 1e-6:
            msg += " approach %+.3f (Δρ=%+.3f)" % (r_prog, parts.get("delta_rho", 0.0))
        if abs(r_exp) > 1e-6:
            msg += " explore %+.4f" % r_exp
        if abs(r_fwd) > 1e-6:
            msg += " forward %+.3f" % r_fwd
        if abs(r_spin) > 1e-6:
            msg += " spin %+.3f" % r_spin
        if abs(r_low) > 1e-6:
            msg += " low_effort %+.3f" % r_low
        msg += " | total_ep=%+.2f rho=%.2fm" % (self._ep_reward_accum, rho)
        rospy.loginfo(msg)

    def step(self, action):
        if self._post_switch_model_only_steps > 0:
            self._post_switch_model_only_steps -= 1

        action = np.asarray(action, dtype=np.float32).reshape(2)
        action[0] = float(np.clip(action[0], -self.max_v, self.max_v))
        # stage A: prefer forward; allow small reverse
        action[0] = float(np.clip(action[0], -0.15, self.max_v))
        action[1] = float(np.clip(action[1], -self.max_w, self.max_w))

        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self._cmd_pub.publish(cmd)
        self._prev_action = action.copy()
        self._sleep_control_dt()

        try:
            x, y, yaw = self._wait_pose(timeout=10.0)
        except RuntimeError as e:
            rospy.logerr(
                "[step] pose lost (world=%s): %s — abort episode, trainer will reset",
                self._world_basename(),
                e,
            )
            self._stop()
            try:
                new_state = self.reset(count_episode=False)
            except Exception:
                new_state = None
            if new_state is None:
                new_state = np.zeros(self.state_dim, dtype=np.float32)
            return new_state, 0.0, False, {
                "episode_abort": True,
                "reason": "pose_lost",
            }
        if self._prev_pose_xy is not None:
            self._path_length += math.hypot(
                float(x) - self._prev_pose_xy[0], float(y) - self._prev_pose_xy[1]
            )
        self._prev_pose_xy = (float(x), float(y))
        rho, _theta = self._goal_polar(x, y, yaw)
        vx, vy = self._get_body_velocity(yaw)
        w = self._get_angular_velocity()

        # --- reward components (logged separately) ---
        r_step_val = self.r_step
        r_episode_start_val = 0.0
        if self._step == 0 and self.r_episode_start != 0.0:
            r_episode_start_val = self.r_episode_start
        delta_rho = 0.0
        r_prog_val = 0.0
        if self._prev_dist is not None:
            delta_rho = self._prev_dist - rho
            r_prog_val = self.r_progress * delta_rho
        self._prev_dist = rho

        r_forward_val, r_spin_val, effort = self._compute_motion_rewards(vx, w)
        r_low_effort_val = self._update_low_effort_penalty(effort)
        r_explore_val = self._compute_explore_reward(x, y, rho)

        reward = (
            r_step_val
            + r_episode_start_val
            + r_prog_val
            + r_explore_val
            + r_forward_val
            + r_spin_val
            + r_low_effort_val
        )
        r_arrive_val = 0.0
        r_collision_val = 0.0
        r_slope_val = 0.0
        r_stuck_val = 0.0
        r_rollover_val = 0.0
        r_timeout_val = 0.0

        ep_elapsed = (
            time.time() - self._ep_start_time
            if self._ep_start_time is not None
            else self._step * self.control_dt
        )

        if self._check_spawn_grace_flip(ep_elapsed):
            self._stop()
            new_state = self.reset(count_episode=False)
            return new_state, 0.0, False, {
                "episode_abort": True,
                "reason": "spawn_grace_flip",
            }

        # Episode termination:
        # 1) rho < goal_tol → success
        # 2) rollover (IMU) → terminal (after grace)
        # 3) stuck window → path stuck (terminal)
        # 4) step count >= max_ep_steps → timeout
        done = False
        info = {
            "success": False,
            "collision": False,
            "collision_event": False,
            "collision_events": self._collision_event_count,
            "stuck": False,
            "rollover": False,
            "timeout": False,
            "rho": rho,
        }

        if rho < self.goal_tol:
            r_arrive_val = self.r_arrive
            reward += r_arrive_val
            done = True
            info["success"] = True
        elif self._check_rollover(ep_elapsed):
            r_rollover_val = self.r_rollover
            reward += r_rollover_val
            done = True
            info["rollover"] = True
        else:
            stuck, _disp = self._update_stuck(x, y)
            if stuck:
                r_stuck_val = self.r_stuck
                reward += r_stuck_val
                done = True
                info["stuck"] = True

        coll_event, coll_penalty, _v_app, _min_d = self._update_collision(vx, vy)
        if coll_event:
            r_collision_val = coll_penalty
            reward += r_collision_val
            info["collision"] = True
            info["collision_event"] = True
            info["collision_events"] = self._collision_event_count
            if (
                self.max_collision_events > 0
                and self._collision_event_count >= self.max_collision_events
            ):
                done = True
                info["collision_limit"] = True
        elif coll_penalty != 0.0 and not coll_event:
            r_slope_val = coll_penalty
            reward += r_slope_val
            info["slope_suspected"] = True

        self._step += 1
        if self._step >= self.max_steps:
            done = True
            info["timeout"] = True
            if not (
                info.get("success")
                or info.get("rollover")
                or info.get("stuck")
                or info.get("collision_limit")
            ):
                r_timeout_val = self.r_timeout
                reward += r_timeout_val

        parts = {
            "r_step": r_step_val,
            "r_episode_start": r_episode_start_val,
            "r_progress": r_prog_val,
            "r_explore": r_explore_val,
            "r_forward": r_forward_val,
            "r_spin": r_spin_val,
            "r_low_effort": r_low_effort_val,
            "r_arrive": r_arrive_val,
            "r_collision": r_collision_val,
            "r_slope": r_slope_val,
            "r_stuck": r_stuck_val,
            "r_rollover": r_rollover_val,
            "r_timeout": r_timeout_val,
            "delta_rho": delta_rho,
            "forward_effort": effort,
        }
        for k in REWARD_PART_KEYS:
            self._ep_reward_parts[k] += float(parts.get(k, 0.0))
        info["reward_parts"] = parts
        if done:
            info["reward_parts_episode"] = dict(self._ep_reward_parts)
        info["initial_goal_dist"] = self._initial_goal_dist
        info["path_length"] = self._path_length
        if self._path_length > 1e-6:
            info["path_efficiency"] = min(
                1.0, float(self._initial_goal_dist) / float(self._path_length)
            )
        else:
            info["path_efficiency"] = 0.0
        self._log_reward(parts, rho, done, info)

        if done:
            self._stop()

        next_state = self._build_state(x, y, yaw)
        return next_state, float(reward), bool(done), info
