#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 Gazebo episode bridge for Scout Mini TD3."""
from __future__ import print_function
import atexit, glob, math, os, random, time
from typing import Optional
import numpy as np
import rclpy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory
from scout_offroad_3d.td3.gazebo_backend_ros2 import GazeboLifecycle, ThrottleLogger

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
    "r_forward",  # legacy alias: high-effort β(|v|+α|ω|) written here for metrics
    "r_high_effort",
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


def _parse_world_spawn_z(raw_entries=None):
    """Build {world_stem_or_name: z} from "stem:z" entries (record 56: single source)."""
    out = {}
    entries = raw_entries
    if entries is None:
        entries = []
    if isinstance(entries, str):
        entries = [entries]
    for item in entries or []:
        s = str(item).strip()
        if not s or ":" not in s:
            continue
        k, v = s.rsplit(":", 1)
        try:
            out[k.strip()] = float(v)
        except (TypeError, ValueError):
            pass
    return out


def _wall_sleep(dt):
    """Wall-clock sleep — safe when /use_sim_time and gzserver is dead (rospy.sleep hangs)."""
    if dt > 0:
        time.sleep(dt)


def _time_to_sec(stamp):
    """rclpy Time → seconds (ROS2 has no Time.to_sec())."""
    return float(stamp.nanoseconds) * 1e-9


def _duration_to_sec(delta):
    """rclpy Duration → seconds."""
    return float(delta.nanoseconds) * 1e-9







class ScoutGazeboEnv(object):
    def __init__(self, node: Optional[Node] = None):
        if node is None:
            if not rclpy.ok(): rclpy.init()
            self._node = rclpy.create_node("scout_gazebo_env")
            self._owns_node = True
        else:
            self._node = node
            self._owns_node = False
        self._log = ThrottleLogger(self._node.get_logger())
        self._declare_all_parameters()
        self._imu_sub = None

        self.n_sectors = int(self._param("n_sectors", 36))
        self.max_range = float(self._param("max_range", 5.0))
        self.goal_tol = float(self._param("goal_tol", 0.45))
        self.collision_dist = float(self._param("collision_dist", 0.28))
        self.collision_v_min = float(self._param("collision_v_min", 0.08))
        self.collision_frames = int(self._param("collision_frames", 4))
        self.collision_sector_half_fov_deg = float(
            self._param("collision_sector_half_fov_deg", 90.0)
        )
        self.max_collision_events = int(self._param("max_collision_events", 0))
        self.collision_penalty_double = _as_bool(
            self._param("collision_penalty_double", True)
        )
        self.collision_penalty_list = _parse_float_list(
            self._param("collision_penalty_list", None), None
        )
        self.stuck_time_sec = float(self._param("stuck_time_sec", 5.0))
        self.stuck_dist_threshold = float(
            self._param("stuck_dist_threshold", 0.25)
        )
        # Near goal: skip stuck so reverse/micro-adjust can finish; timeout if steps exhaust.
        self.stuck_near_goal_rho = float(
            self._param("stuck_near_goal_rho", 0.7)
        )
        self.max_steps = int(self._param("max_ep_steps", 400))
        self.control_dt = float(self._param("control_dt", 0.1))
        self.control_dt_wall_cap_min = float(
            self._param("control_dt_wall_cap_min", 1.0)
        )
        self.control_dt_wall_cap_mult = float(
            self._param("control_dt_wall_cap_mult", 10.0)
        )
        self.max_v = float(self._param("max_v", 0.6))
        self.max_w = float(self._param("max_w", 1.0))
        self.pose_source = self._param("pose_source", "gazebo").strip().lower()
        self.model_name = self._param("model_name", "scout_mini")
        self.cmd_topic = self._param("cmd_vel_topic", "/cmd_vel")
        self.features_topic = self._param("features_topic", "/offroad/range_features")
        self.td3_features_topic = self._param_global(
            "~td3_features_topic", "/offroad/td3_range_features"
        )
        self.elevation_features_topic = self._param_global(
            "~elevation_features_topic", "/offroad/elevation_features"
        )
        self.odom_topic = self._param("odom_topic", "/odom")

        # Slope vs wall terrain classifier (uses multi-elevation front sectors + optional IMU)
        self.terrain_classify_enable = _as_bool(
            self._param("terrain_classify_enable", True)
        )
        self.n_elevation_bins = int(self._param("n_elevation_bins", 4))
        self.n_elev_front_sectors = int(self._param("n_elev_front_sectors", 12))
        self.n_elevation_bins_td3 = int(self._param("n_elevation_bins_td3", 4))
        self.slope_max_frames = int(self._param("slope_max_frames", 6))
        self.wall_all_layers_dist = float(self._param("wall_all_layers_dist", 0.27))
        self.wall_lower_layers_dist = float(
            self._param("wall_lower_layers_dist", 0.27)
        )
        self.wall_layers_required = int(self._param("wall_layers_required", 2))
        self.wall_force_min_dist = float(self._param("wall_force_min_dist", 0.19))
        self.slope_upper_clear_dist = float(
            self._param("slope_upper_clear_dist", 0.8)
        )
        self.r_slope_penalty = float(self._param("r_slope_penalty", -0.2))
        self.use_imu_pitch = _as_bool(self._param("use_imu_pitch", True))
        self.imu_topic = self._param("imu_topic", "/imu/data")
        self.slope_pitch_delta_deg = float(
            self._param("slope_pitch_delta_deg", 3.0)
        )
        self.wall_v_forward_max = float(self._param("wall_v_forward_max", 0.12))

        # Random start / goal box (meters, map/world frame; tune via yaml for large maps)
        self.spawn_xy_min = [
            float(self._param("spawn_x_min", -3.5)),
            float(self._param("spawn_y_min", -3.5)),
        ]
        self.spawn_xy_max = [
            float(self._param("spawn_x_max", 3.5)),
            float(self._param("spawn_y_max", 3.5)),
        ]
        self.goal_xy_min = [
            float(self._param("goal_x_min", self.spawn_xy_min[0])),
            float(self._param("goal_y_min", self.spawn_xy_min[1])),
        ]
        self.goal_xy_max = [
            float(self._param("goal_x_max", self.spawn_xy_max[0])),
            float(self._param("goal_y_max", self.spawn_xy_max[1])),
        ]
        self.min_goal_dist = float(self._param("min_goal_dist", 1.5))
        self.goal_check_enable = _as_bool(self._param("goal_check_enable", True))
        self.goal_min_clearance = float(self._param("goal_min_clearance", 0.38))
        self.goal_z_max_delta = float(self._param("goal_z_max_delta", 5.0))
        self.goal_resample_max = int(self._param("goal_resample_max", 25))
        self._base_spawn_z = float(self._param("spawn_z", 0.15))
        self.spawn_z = self._base_spawn_z
        self._world_spawn_z = _parse_world_spawn_z(
            self._param(
                "world_spawn_z_entries",
                [
                    "bumpy_offroad:0.23",
                    "bumpy_offroad_2:0.23",
                    "bumpy_offroad_3:0.23",
                    "bumpy_offroad_4:0.23",
                ],
            )
        )
        # Exactly 4 train / switch / viz worlds.
        from scout_offroad_3d.world_pool import (
            TRAIN_WORLD_STEMS,
            world_stem as _world_stem,
        )

        raw_train = self._param("train_worlds", list(TRAIN_WORLD_STEMS))
        if isinstance(raw_train, str):
            raw_train = [s.strip() for s in raw_train.split(",") if s.strip()]
        self._train_world_stems = [
            _world_stem(s) for s in (raw_train or list(TRAIN_WORLD_STEMS)) if s
        ] or list(TRAIN_WORLD_STEMS)
        if self._world_spawn_z:
            self._log.info(
                "world_spawn_z overrides: %s",
                ", ".join("%s=%.2f" % (k, v) for k, v in sorted(self._world_spawn_z.items())),
            )
        self._log.info(
            "world pool: train=%s",
            ",".join(self._train_world_stems),
        )
        self.spawn_z_retry_delta = float(self._param("spawn_z_retry_delta", 0.5))
        self.spawn_z_max_retries = int(self._param("spawn_z_max_retries", 7))
        # Heightmap Z ladder: fixed XY, try z_min..z_max by step (see _spawn_z_ladder).
        self.spawn_z_scan_enable = _as_bool(self._param("spawn_z_scan_enable", True))
        self.spawn_z_scan_min = float(self._param("spawn_z_scan_min", 0.5))
        self.spawn_z_scan_max = float(self._param("spawn_z_scan_max", 3.5))
        self.spawn_z_scan_step = float(self._param("spawn_z_scan_step", 0.5))
        # Accept landing this far below z_try (dropped onto uneven ground); reject if higher.
        self.spawn_z_max_fall = float(self._param("spawn_z_max_fall", 2.8))
        self.spawn_check_wait = float(self._param("spawn_check_wait", 0.5))
        self.spawn_z_jump_threshold = float(
            self._param("spawn_z_jump_threshold", 0.35)
        )
        self.spawn_vz_threshold = float(self._param("spawn_vz_threshold", 0.5))
        self.spawn_collision_dist = float(
            self._param("spawn_collision_dist", 0.35)
        )
        self.spawn_pose_stable_threshold = float(
            self._param("spawn_pose_stable_threshold", 0.08)
        )
        self.spawn_full_reset_retries = int(
            self._param("spawn_full_reset_retries", 50)
        )
        self.spawn_xy_max_retries = int(
            self._param("spawn_xy_max_retries", 50)
        )
        self.spawn_xy_jitter = float(self._param("spawn_xy_jitter", 0.8))

        self.r_progress = float(self._param("r_progress", 1.0))
        self.r_arrive = float(self._param("r_arrive", 20.0))
        self.r_episode_start = float(self._param("r_episode_start", 0.0))
        self.r_collision_base = float(self._param("r_collision_base", -2.0))
        self.r_stuck = float(self._param("r_stuck", -20.0))
        self.r_rollover = float(self._param("r_rollover", -25.0))
        self.r_timeout = float(self._param("r_timeout", -20.0))
        self.r_step = float(self._param("r_step", -0.05))
        # Legacy; high-effort uses effort_reward_scale (β) instead of v-|ω|
        self.r_forward_scale = float(self._param("r_forward_scale", 0.0))
        self.effort_omega_weight = float(self._param("effort_omega_weight", 1.0))
        self.effort_reward_scale = float(self._param("effort_reward_scale", 1.0 / 3.0))
        self.high_effort_thresh = float(self._param("high_effort_thresh", 0.8))
        self.r_spin_penalty = float(self._param("r_spin_penalty", 0.08))
        self.spin_v_thresh = float(self._param("spin_v_thresh", 0.08))
        self.spin_w_thresh = float(self._param("spin_w_thresh", 1.35))
        self.r_low_effort_penalty = float(
            self._param("r_low_effort_penalty", -0.72)
        )
        self.low_effort_thresh = float(self._param("low_effort_thresh", 0.25))
        self.low_effort_time_sec = float(self._param("low_effort_time_sec", 1.5))
        self.r_slope_climb = float(self._param("r_slope_climb", 0.16))
        self.r_slope_descend = float(self._param("r_slope_descend", 0.05))
        self.slope_climb_pitch_min_deg = float(
            self._param("slope_climb_pitch_min_deg", 2.0)
        )
        self.slope_climb_pitch_max_deg = float(
            self._param("slope_climb_pitch_max_deg", 35.0)
        )
        # Record 9: A_climb = slope_climb_scale * r_progress * Δρ (trainer anneals scale)
        self.slope_climb_scale = float(self._param("slope_climb_scale", 0.25))
        self.use_imu_in_state = _as_bool(self._param("use_imu_in_state", True))
        self.allow_reverse = _as_bool(self._param("allow_reverse", True))
        self.state_noise_std = float(self._param("state_noise_std", 0.0))
        self.reward_log_interval = int(self._param("reward_log_interval", 25))

        self.explore_enable = _as_bool(self._param("explore_enable", True))
        self.explore_grid_res = float(self._param("explore_grid_res", 0.5))
        self.explore_cell_reward = float(
            self._param("explore_cell_reward", 0.03)
        )
        self.explore_rho_min = float(self._param("explore_rho_min", 1.0))
        self.explore_cardiff_scale = _as_bool(
            self._param("explore_cardiff_scale", False)
        )

        self.rollover_enable = _as_bool(self._param("rollover_enable", True))
        self.rollover_roll_deg = float(self._param("rollover_roll_deg", 60.0))
        self.rollover_pitch_deg = float(self._param("rollover_pitch_deg", 60.0))
        self.rollover_sum_deg = float(self._param("rollover_sum_deg", 80.0))
        self.rollover_grace_sec = float(self._param("rollover_grace_sec", 2.0))
        self.rollover_grace_abort_enable = _as_bool(
            self._param("rollover_grace_abort_enable", True)
        )
        self.rollover_model_fallback = _as_bool(
            self._param("rollover_model_fallback", True)
        )
        self.use_imu_roll = _as_bool(self._param("use_imu_roll", True))
        self.imu_stale_sec = float(self._param("imu_stale_sec", 1.0))
        self.imu_wait_after_respawn_sec = float(
            self._param("imu_wait_after_respawn_sec", 8.0)
        )
        self.imu_model_fallback_after_switch_steps = int(
            self._param("imu_model_fallback_after_switch_steps", 50)
        )

        self.world_switch_interval = int(self._param("world_switch_interval", 20000))
        self.world_switch_max_retries = int(
            self._param("world_switch_max_retries", 5)
        )
        self.world_switch_kill_wait_sec = float(
            self._param("world_switch_kill_wait_sec", 0.5)
        )
        self.world_switch_gzserver_boot_sec = float(
            self._param("world_switch_gzserver_boot_sec", 8.0)
        )
        self.world_switch_spawn_wait_sec = float(
            self._param("world_switch_spawn_wait_sec", 35.0)
        )
        self.world_switch_retry_sleep_sec = float(
            self._param("world_switch_retry_sleep_sec", 1.0)
        )
        self.worlds_dir = self._param("worlds_dir", "").strip()
        self._want_gui = _as_bool(self._param("gazebo_gui", False))
        self.current_world = str(self._param("current_world", "") or "").strip()
        # Private, launch-generated nonce used only to prove ownership of the
        # first gzserver sibling during the initial world-switch handoff.
        self.gazebo_owner_token = str(
            self._param("gazebo_owner_token", "") or ""
        ).strip()
        self._world_before_switch = None
        # Re-bind spawn_z now that current_world is known (heightmaps ≠ 0.20).
        self.spawn_z = self._spawn_z_for_current_world()

        self.state_dim = (
            self.n_elevation_bins_td3 * self.n_sectors
            + 4
            + (2 if self.use_imu_in_state else 0)
        )
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

        qos_cmd = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cmd_pub = self._node.create_publisher(Twist, self.cmd_topic, qos_cmd)
        self._goal_pub = self._node.create_publisher(Point, "/offroad/train_goal", qos_cmd)
        self._node.create_subscription(Float32MultiArray, self.features_topic, self._on_features, qos_sensor)
        self._node.create_subscription(Float32MultiArray, self.td3_features_topic, self._on_td3_features, qos_sensor)
        if self.terrain_classify_enable:
            self._node.create_subscription(Float32MultiArray, self.elevation_features_topic, self._on_elevation_features, qos_sensor)
        self._ensure_imu_subscriber()
        if self.pose_source == "odom":
            self._node.create_subscription(Odometry, self.odom_topic, self._on_odom, qos_sensor)

        self._gazebo = GazeboLifecycle(self._node, self)
        self._log.info("ScoutGazeboEnv waiting for Gazebo entity-state services...")
        self._gazebo.wait_model_services(timeout=60.0)
        # If launch already started gzclient (gui:=true), do nothing. Only start
        # one after world-switch when the launch-owned client was killed.
        self._gazebo.ensure_gzclient()
        atexit.register(self._gazebo.shutdown_owned)
        self._log.info(
            "ScoutGazeboEnv ready: model=%s pose=%s td3=%s horiz=%s elev=%s "
            "cmd=%s state_dim=%d terrain=%s world=%s switch_interval=%d gui=%s",
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
            self._want_gui,
        )


    def _declare_all_parameters(self):
        """Declare defaults only when launch yaml did not already load env params."""
        if self._node.has_parameter("max_ep_steps"):
            return
        defaults = {"n_sectors":36,"max_range":5.0,"goal_tol":0.45,"collision_dist":0.28,"collision_v_min":0.08,
            "collision_frames":4,"collision_sector_half_fov_deg":90.0,"max_collision_events":0,
            "collision_penalty_double":True,"collision_penalty_list":[],"stuck_time_sec":5.0,
            "stuck_dist_threshold":0.25,"stuck_near_goal_rho":0.7,"max_ep_steps":400,"control_dt":0.1,"max_v":0.6,"max_w":1.0,
            "pose_source":"gazebo","model_name":"scout_mini","cmd_vel_topic":"/cmd_vel",
            "features_topic":"/offroad/range_features","td3_features_topic":"/offroad/td3_range_features",
            "elevation_features_topic":"/offroad/elevation_features","odom_topic":"/odom",
            "terrain_classify_enable":True,"n_elevation_bins":4,"n_elev_front_sectors":12,
            "n_elevation_bins_td3":4,"use_imu_in_state":True,"slope_climb_scale":0.25,
            "slope_max_frames":6,"wall_all_layers_dist":0.27,
            "wall_lower_layers_dist":0.27,"wall_layers_required":2,"wall_force_min_dist":0.19,
            "slope_upper_clear_dist":0.8,"r_slope_penalty":-0.2,"use_imu_pitch":True,"use_imu_roll":True,
            "imu_topic":"/imu/data","slope_pitch_delta_deg":3.0,"wall_v_forward_max":0.12,
            "spawn_x_min":-3.5,"spawn_x_max":3.5,"spawn_y_min":-3.5,"spawn_y_max":3.5,
            "goal_x_min":-3.5,"goal_x_max":3.5,"goal_y_min":-3.5,"goal_y_max":3.5,"min_goal_dist":1.5,
            "goal_check_enable":True,"goal_min_clearance":0.38,"goal_z_max_delta":5.0,"goal_resample_max":25,
            "control_dt_wall_cap_min":1.0,"control_dt_wall_cap_mult":10.0,
            "spawn_z":0.15,"world_spawn_z_entries":[
                "bumpy_offroad:0.23","bumpy_offroad_2:0.23","bumpy_offroad_3:0.23",
                "bumpy_offroad_4:0.23"],
            "train_worlds":[
                "bumpy_offroad","bumpy_offroad_2","bumpy_offroad_3","bumpy_offroad_4"],
            "spawn_z_retry_delta":0.5,"spawn_z_max_retries":7,
            "spawn_z_scan_enable":True,"spawn_z_scan_min":0.5,"spawn_z_scan_max":3.5,
            "spawn_z_scan_step":0.5,"spawn_z_max_fall":2.8,
            "spawn_check_wait":0.5,"spawn_z_jump_threshold":0.35,"spawn_vz_threshold":0.5,
            "spawn_collision_dist":0.35,"spawn_pose_stable_threshold":0.08,"spawn_full_reset_retries":50,
            "spawn_xy_max_retries":50,"spawn_xy_jitter":0.8,"r_progress":1.0,"r_arrive":20.0,
            "r_episode_start":0.0,"r_collision_base":-2.0,"r_stuck":-20.0,"r_rollover":-25.0,
            "r_timeout":-20.0,"r_step":-0.05,"r_forward_scale":0.5,"r_spin_penalty":0.5,
            "spin_v_thresh":0.15,"spin_w_thresh":0.8,"r_low_effort_penalty":-0.08,
            "low_effort_thresh":0.05,"low_effort_time_sec":3.0,"state_noise_std":0.0,"reward_log_interval":25,
            "explore_enable":True,"explore_grid_res":0.5,"explore_cell_reward":0.03,"explore_rho_min":1.0,
            "explore_cardiff_scale":False,"rollover_enable":True,"rollover_roll_deg":40.0,
            "rollover_pitch_deg":40.0,"rollover_sum_deg":55.0,"rollover_grace_sec":2.0,
            "rollover_grace_abort_enable":True,"rollover_model_fallback":True,"imu_stale_sec":1.0,
            "imu_wait_after_respawn_sec":8.0,"imu_model_fallback_after_switch_steps":50,
            "world_switch_interval":20000,"world_switch_max_retries":5,"world_switch_kill_wait_sec":0.5,
            "world_switch_gzserver_boot_sec":8.0,"world_switch_spawn_wait_sec":35.0,
            "world_switch_retry_sleep_sec":1.0,"worlds_dir":"","gazebo_gui":False,"current_world":"","gazebo_owner_token":"",
            "robot_description":""}
        for name, val in defaults.items():
            if not self._node.has_parameter(name):
                try:
                    self._node.declare_parameter(name, val)
                except Exception:
                    pass

    def _param(self, name, default=None):
        key = str(name).lstrip("~")
        if self._node.has_parameter(key):
            val = self._node.get_parameter(key).value
            if val is None or (isinstance(val, str) and not str(val).strip() and default is not None):
                return default
            return val
        return default

    def _param_global(self, name, default=""):
        key = str(name).lstrip("/")
        if self._node.has_parameter(key):
            return self._node.get_parameter(key).value
        return default

    def _set_global_param(self, name, value):
        try:
            self._node.set_parameters([Parameter(str(name).lstrip("/"), Parameter.Type.STRING, str(value))])
        except Exception:
            pass

    def switch_world(self, world_path, timestep=None):
        return self._gazebo.switch_world(world_path, timestep=timestep)

    def pause_gui_for_eval(self):
        """Stop gzclient during viz eval if GUI was on. Returns restore flag."""
        try:
            return bool(self._gazebo.pause_gzclient_for_eval())
        except Exception:
            return False

    def restore_gui_after_eval(self, was_paused):
        try:
            self._gazebo.restore_gzclient_after_eval(bool(was_paused))
        except Exception:
            pass

    def _gz_child_ok(self):
        return self._gazebo.gz_child_ok()

    def close(self):
        self._gazebo.shutdown_owned()
        if getattr(self, "_owns_node", False):
            try: self._node.destroy_node()
            except Exception: pass


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
            "stuck_near_goal_rho": self.stuck_near_goal_rho,
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
            "use_imu_in_state": self.use_imu_in_state,
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
            "spawn_z_scan_enable": self.spawn_z_scan_enable,
            "spawn_z_scan_min": self.spawn_z_scan_min,
            "spawn_z_scan_max": self.spawn_z_scan_max,
            "spawn_z_scan_step": self.spawn_z_scan_step,
            "spawn_z_max_fall": self.spawn_z_max_fall,
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
            "effort_omega_weight": self.effort_omega_weight,
            "effort_reward_scale": self.effort_reward_scale,
            "high_effort_thresh": self.high_effort_thresh,
            "r_spin_penalty": self.r_spin_penalty,
            "spin_v_thresh": self.spin_v_thresh,
            "spin_w_thresh": self.spin_w_thresh,
            "r_low_effort_penalty": self.r_low_effort_penalty,
            "low_effort_thresh": self.low_effort_thresh,
            "low_effort_time_sec": self.low_effort_time_sec,
            "r_slope_climb": self.r_slope_climb,
            "r_slope_descend": self.r_slope_descend,
            "slope_climb_pitch_min_deg": self.slope_climb_pitch_min_deg,
            "slope_climb_pitch_max_deg": self.slope_climb_pitch_max_deg,
            "slope_climb_scale": self.slope_climb_scale,
            "use_imu_in_state": self.use_imu_in_state,
            "allow_reverse": self.allow_reverse,
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
            "train_worlds": list(self._train_world_stems),
            "state_dim": self.state_dim,
            "action_dim": self.action_dim,
            "max_action": self.max_action.tolist(),
        }

    def _worlds_dir_resolved(self):
        if self.worlds_dir: return self.worlds_dir
        try: return os.path.join(get_package_share_directory("scout_offroad_3d"), "worlds")
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.normpath(os.path.join(here, "..", "..", "worlds"))

    def list_worlds(self, include_holdout=False):
        """Training / switch / viz pool (exactly 4). include_holdout ignored (compat)."""
        del include_holdout  # unused; kept for call-site compat
        from scout_offroad_3d.world_pool import filter_world_paths

        worlds_dir = self._worlds_dir_resolved()
        cands = sorted(glob.glob(os.path.join(worlds_dir, "*.world")))
        if not cands:
            raise RuntimeError("no *.world under %s" % worlds_dir)
        pool = filter_world_paths(cands, self._train_world_stems)
        if not pool:
            from scout_offroad_3d.world_pool import TRAIN_WORLD_STEMS

            pool = filter_world_paths(cands, TRAIN_WORLD_STEMS) or cands
        return pool

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
            try: self._node.destroy_subscription(self._imu_sub)
            except Exception: pass
            self._imu_sub = None
        qos_imu = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._imu_sub = self._node.create_subscription(Imu, self.imu_topic, self._on_imu, qos_imu)
        self._log.info("[imu] (re)subscribed %s", self.imu_topic)

    def _imu_fresh(self):
        if not self._imu_received or self._imu_last_time is None:
            return False
        try:
            age = _duration_to_sec(self._node.get_clock().now() - self._imu_last_time)
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
        self._log.info(
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
            resp = self._gazebo.get_model_state(self.model_name, "world")
            if resp.success:
                roll, pitch = _roll_pitch_rad_from_quat(resp.pose.orientation)
                return abs(math.degrees(roll)), abs(math.degrees(pitch))
        except Exception as e:
            self._log.warn_throttle(5.0, "model orientation query failed: %s", e)
        return None, None

    def _wait_imu_after_respawn(self, timeout=None):
        if timeout is None:
            timeout = self.imu_wait_after_respawn_sec
        if not self._imu_needed():
            return True
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.01)
            if self._imu_fresh():
                self._log.info(
                    "[imu] first message on %s after respawn (%.2fs)",
                    self.imu_topic,
                    time.time() - t0,
                )
                return True
            _wall_sleep(0.05)
        self._log.warn(
            "[imu] no data on %s after %.1fs — rollover will use model fallback",
            self.imu_topic,
            timeout,
        )
        return False

    def _on_imu(self, msg):
        self._imu_received = True
        self._imu_last_time = self._node.get_clock().now()
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
            resp = self._gazebo.get_model_state(self.model_name, "world")
        except Exception as e:
            self._log.warn_throttle(2.0, "get_model_state failed: %s", e)
            return None
        if not resp.success:
            return None
        p = resp.pose.position
        return p.x, p.y, _yaw_from_quat(resp.pose.orientation)

    def _wait_pose(self, timeout=15.0):
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.01)
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
        while rclpy.ok():
            try:
                resp = self._gazebo.get_model_state(self.model_name, "world")
                if resp.success:
                    return True
            except Exception:
                pass
            if time.time() - t0 > timeout:
                return False
            _wall_sleep(0.1)
        return False

    def _world_basename(self, world_path=None):
        """World id for logs / metrics / heatmaps — stem without ``.world``."""
        path = world_path if world_path else self.current_world
        if not path:
            return "(default)"
        base = os.path.basename(path)
        if base.lower().endswith(".world"):
            base = base[: -len(".world")]
        return base or "(default)"

    def _spawn_z_for_world(self, world_path=None):
        """Per-world spawn_z override (e.g. heightmap bumpy ~0.35, playpen 0.20)."""
        base = self._world_basename(world_path)
        if not base or base == "(default)":
            return self._base_spawn_z
        table = self._world_spawn_z or {}
        if base in table:
            return float(table[base])
        stem = base[:-6] if base.lower().endswith(".world") else base
        if stem in table:
            return float(table[stem])
        # 子串兜底放最后：否则 bumpy_offroad 会抢先匹配 bumpy_offroad_2…4
        for key, val in table.items():
            k = str(key)
            if k in base or base in k or stem in k:
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
            self._set_global_param("offroad.current_world", world_path)
        except Exception:
            pass
        self._log.info(
            "[world] identity → %s spawn_z=%.3f",
            os.path.basename(world_path),
            self.spawn_z,
        )
        return self.spawn_z

    def _get_model_z_vz(self):
        try:
            resp = self._gazebo.get_model_state(self.model_name, "world")
        except Exception as e:
            self._log.warn_throttle(2.0, "get_model_state (z): %s", e)
            return None, None
        if not resp.success:
            return None, None
        return float(resp.pose.position.z), float(resp.twist.linear.z)

    def _set_pose(self, x, y, yaw, z=None):
        z_use = self.spawn_z if z is None else float(z)
        state = ModelState()
        state.model_name = self.model_name
        state.pose = Pose()
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z_use)
        state.pose.orientation = _quat_from_yaw(yaw)
        state.twist = Twist()
        state.reference_frame = "world"
        # Two sets are enough; five × sleep was reset thrash under dense worlds.
        for _ in range(2):
            try:
                self._gazebo.set_model_state(state)
            except Exception as e:
                self._log.warn("set_model_state: %s", e)
            _wall_sleep(0.02)

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
            resp = self._gazebo.get_model_state(self.model_name, "world")
            if resp.success:
                tw = resp.twist
                vx_w = float(tw.linear.x)
                vy_w = float(tw.linear.y)
                cy = math.cos(yaw)
                sy = math.sin(yaw)
                vx_b = vx_w * cy + vy_w * sy
                vy_b = -vx_w * sy + vy_w * cy
                return vx_b, vy_b
        except Exception:
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
            resp = self._gazebo.get_model_state(self.model_name, "world")
            if resp.success:
                return float(resp.twist.angular.z)
        except Exception:
            pass
        return float(self._prev_action[1])

    def _maneuver_effort(self, vx, w):
        """E = |v| + α|ω| (encourages speed and turning; α=effort_omega_weight)."""
        return abs(float(vx)) + self.effort_omega_weight * abs(float(w))

    def _compute_motion_rewards(self, vx, w):
        """High-effort β·E when E≥e_hi; light spin only for near-zero-v + very high |ω|."""
        w_abs = abs(float(w))
        effort = self._maneuver_effort(vx, w)
        r_high = 0.0
        if effort >= self.high_effort_thresh:
            r_high = self.effort_reward_scale * effort
        # Optional legacy forward term (default 0)
        if self.r_forward_scale != 0.0:
            r_high += self.r_forward_scale * max(0.0, float(vx) - w_abs)
        r_spin = 0.0
        if abs(float(vx)) < self.spin_v_thresh and w_abs > self.spin_w_thresh:
            r_spin = -self.r_spin_penalty * (w_abs - self.spin_w_thresh)
        return r_high, r_spin, effort

    def _update_low_effort_penalty(self, effort):
        """Sustained low E → per-step penalty (discourage grinding / waiting for stuck)."""
        now = _time_to_sec(self._node.get_clock().now())
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

    def _compute_slope_traverse_reward(self, delta_rho):
        """Climb/descend: scale * r_progress * Δρ (Record 9; scale from trainer)."""
        if delta_rho <= 0.0:
            return 0.0
        pitch_deg = math.degrees(float(self._pitch))
        lo = self.slope_climb_pitch_min_deg
        hi = self.slope_climb_pitch_max_deg
        scale = float(self.slope_climb_scale)
        base = scale * self.r_progress * float(delta_rho)
        if lo <= pitch_deg <= hi:
            return base
        if -hi <= pitch_deg <= -lo:
            return base / 3.0
        return 0.0

    def set_slope_climb_scale(self, scale):
        """Trainer updates climb shaping scale from window success rate."""
        self.slope_climb_scale = max(0.0, float(scale))

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
            self._log.info(
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
            self._log.info_throttle(
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
        self._log.info(
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
        now = _time_to_sec(self._node.get_clock().now())
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
        self._log.info(
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
        self._log.debug_throttle(
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
            self._log.info(
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
            self._log.info(
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

    def _spawn_z_ladder(self):
        """Z heights to try at a fixed XY.

        * gui:=true / scan off: light path — world_spawn_z then a few +delta bumps.
        * gui:=false + spawn_z_scan_enable: world_spawn_z FIRST, then scan up to
          z_max (for heightmap holes / hills). Never skip world_z: starting at
          0.5 while world_z≈0.23 freefalls into high_vz / z_unstable false fails.
        """
        z0 = float(self._spawn_z_for_current_world())
        light = (not self.spawn_z_scan_enable) or bool(
            getattr(self, "_want_gui", False)
        )
        if light:
            step = max(0.25, float(self.spawn_z_retry_delta))
            n = max(1, int(self.spawn_z_max_retries))
            if getattr(self, "_want_gui", False):
                n = min(n, 3)  # at most 4 tries with GUI
            return [round(z0 + i * step, 3) for i in range(n + 1)]

        z_min = float(self.spawn_z_scan_min)
        z_max = float(self.spawn_z_scan_max)
        step = max(1e-3, float(self.spawn_z_scan_step))
        ladder = []
        z = z_min
        while z <= z_max + 1e-9:
            ladder.append(round(z, 3))
            z += step
        if not ladder:
            ladder = [z_min]
        # Always lead with world spawn_z (≈0.23). Old code dropped z0 when
        # z0 < z_min, so headless first try was always 0.5 → drop → high_vz.
        rest = [h for h in ladder if abs(h - z0) > 1e-6]
        return [round(z0, 3)] + rest

    def _goal_z_ladder(self):
        """Goals never use the headless 0.5→3.5 storm — only a few world_z bumps."""
        z0 = float(self._spawn_z_for_current_world())
        step = max(0.25, float(self.spawn_z_retry_delta))
        n = min(3, max(1, int(self.spawn_z_max_retries)))
        return [round(z0 + i * step, 3) for i in range(n)]

    def _validate_goal(self, gx, gy, sx, sy, syaw):
        """Place robot at goal; light Z only. Blocked XY must not climb height ladder."""
        last_reason = "goal_untested"
        for z_try in self._goal_z_ladder():
            self._set_pose(float(gx), float(gy), float(syaw), z=float(z_try))
            ok, reason, z, _vz, min_d = self._check_spawn_stable(
                z_try, lidar_timeout=1.0, clearance_dist=self.goal_min_clearance
            )
            if ok:
                if min_d is not None and min_d < self.goal_min_clearance:
                    # Tree/wall at this XY — higher Z will not clear horizontal range.
                    self._set_pose(float(sx), float(sy), float(syaw), z=self.spawn_z)
                    _wall_sleep(0.05)
                    return False, "goal_blocked"
                # 记录57③：终点落点与起点落点（self.spawn_z）高差超限则拒绝——
                # 避免目标落在陡坡顶/深坑底等机器人根本到不了的高度。
                if z is not None and abs(float(z) - float(self.spawn_z)) > self.goal_z_max_delta:
                    self._set_pose(float(sx), float(sy), float(syaw), z=self.spawn_z)
                    _wall_sleep(0.05)
                    return False, "goal_z_delta"
                self._set_pose(float(sx), float(sy), float(syaw), z=self.spawn_z)
                _wall_sleep(0.05)
                return True, "ok"
            if reason == "stuck_collision":
                self._set_pose(float(sx), float(sy), float(syaw), z=self.spawn_z)
                _wall_sleep(0.05)
                return False, "goal_blocked"
            last_reason = reason or "goal_unstable"
        self._set_pose(float(sx), float(sy), float(syaw), z=self.spawn_z)
        _wall_sleep(0.05)
        return False, last_reason

    def _wait_lidar_settle(self, timeout=2.0):
        """Wait until range features are not all saturated (fresh /scan)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.01)
            if self._min_sector_dist() < self.max_range * 0.99:
                return True
            _wall_sleep(0.05)
        return False

    def _check_spawn_stable(self, z_target, lidar_timeout=2.0, clearance_dist=None):
        """
        After pose set: wall-clock settle, then validate z/vz/pose stability and clearance.
        Returns (ok, reason, z, vz, min_d).

        On heightmaps, landing *below* z_target (dropped onto terrain) is OK as long as
        fall ≤ spawn_z_max_fall and vz has settled. Spawning *inside* ground (settled
        well above z_target) still fails as z_jump.
        """
        clear_d = (
            float(self.spawn_collision_dist)
            if clearance_dist is None
            else float(clearance_dist)
        )
        _wall_sleep(self.spawn_check_wait)

        z1, vz1 = self._get_model_z_vz()
        if z1 is None:
            return False, "pose_unreadable", None, None, None

        _wall_sleep(0.1)
        z2, vz2 = self._get_model_z_vz()
        if z2 is None:
            return False, "pose_lost", z1, vz1, None

        if abs(z2 - z1) > self.spawn_pose_stable_threshold:
            return False, "z_unstable", z2, vz2, None

        z = z2
        vz = vz2 if vz2 is not None else 0.0
        vz_abs = abs(vz)
        z_target = float(z_target)

        # Settled higher than intended → likely embedded / bounced out of mesh.
        if z > z_target + self.spawn_z_jump_threshold:
            z_world = self._spawn_z_for_current_world()
            if abs(z - z_world) <= self.spawn_z_jump_threshold:
                self._log.warn(
                    "[spawn] z=%.3f > target_z=%.3f but matches world_spawn_z=%.3f — accept",
                    z,
                    z_target,
                    z_world,
                )
            else:
                return False, "z_jump", z, vz, None
        # Settled much lower than intended → fell too far (bad XY / hole).
        elif z_target - z > self.spawn_z_max_fall:
            return False, "z_fall", z, vz, None

        if vz_abs > self.spawn_vz_threshold:
            return False, "high_vz", z, vz, None

        lidar_ok = self._wait_lidar_settle(timeout=float(lidar_timeout))
        min_d = self._min_sector_dist()
        if lidar_ok and min_d < clear_d:
            return False, "stuck_collision", z, vz, min_d
        if not lidar_ok:
            self._log.warn_throttle(
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
                self._log.warn(
                    "[spawn] random XY %d/%d still blocked (world=%s context=%s)",
                    i + 1,
                    n,
                    self._world_basename(),
                    context,
                )
        self._log.err(
            "[spawn] no clear XY after origin + %d samples (world=%s context=%s) "
            "— give up this map next, do NOT abort training",
            n,
            self._world_basename(),
            context,
        )
        return False

    def _ensure_spawn_ok(self, x, y, yaw, context="reset", allow_xy_retry=True):
        """
        Fixed XY: try a short Z list (light under GUI; full ladder only headless).
        All Z fail → False so caller picks another XY.
        """
        del allow_xy_retry
        self.spawn_z = self._spawn_z_for_current_world()
        x_try, y_try = float(x), float(y)
        ladder = self._spawn_z_ladder()
        last_reason = "unknown"

        if not self._wait_for_model(timeout=30.0):
            self._log.warn(
                "[spawn] model %s not in Gazebo after 30s (context=%s)",
                self.model_name,
                context,
            )

        for attempt, z_try in enumerate(ladder):
            world_name = self._world_basename()
            z_try = float(z_try)
            self._set_pose(x_try, y_try, yaw, z=z_try)
            ok, reason, z, vz, min_d = self._check_spawn_stable(z_try)
            if ok:
                self.spawn_z = float(z) if z is not None else z_try
                self._log.info(
                    "[spawn] OK world=%s context=%s z=%.3f (tried=%.3f) min_d=%.2fm "
                    "z_try=%d/%d xy=(%.2f,%.2f)",
                    world_name,
                    context,
                    z if z is not None else z_try,
                    z_try,
                    min_d if min_d is not None else -1.0,
                    attempt + 1,
                    len(ladder),
                    x_try,
                    y_try,
                )
                return True

            last_reason = reason
            vz_s = "%.3f" % vz if vz is not None else "?"
            min_d_s = "%.2f" % min_d if min_d is not None else "?"
            self._log.warn(
                "[spawn] FAIL world=%s context=%s z_try=%d/%d reason=%s "
                "z=%s vz=%s min_d=%s target_z=%.3f xy=(%.2f,%.2f)",
                world_name,
                context,
                attempt + 1,
                len(ladder),
                reason,
                "%.3f" % z if z is not None else "?",
                vz_s,
                min_d_s,
                z_try,
                x_try,
                y_try,
            )

        self._log.warn(
            "[spawn] Z ladder exhausted world=%s context=%s after %d heights "
            "(last=%s) xy=(%.2f,%.2f) — resample another (x,y)",
            self._world_basename(),
            context,
            len(ladder),
            last_reason,
            x_try,
            y_try,
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
        tail_vals = [rho_n, theta / math.pi, v_n, w_n]
        if self.use_imu_in_state:
            # Body attitude from /imu/data (or model_state fallback)
            tail_vals.extend(
                [
                    float(self._roll) / math.pi,
                    float(self._pitch) / math.pi,
                ]
            )
        tail = np.array(tail_vals, dtype=np.float32)
        if self.state_noise_std > 0.0:
            tail += np.random.normal(0.0, self.state_noise_std, size=tail.shape)
            tail = np.clip(tail, -1.0, 1.0)
        return np.concatenate([d_norm, tail]).astype(np.float32)

    def _sleep_control_dt(self):
        """
        Wait ~control_dt of sim time. Safety wall cap only if /clock freezes
        (gzserver dead) — not a substitute for fixing RTF (tree mesh / ray load).
        """
        target = float(self.control_dt)
        t_wall0 = time.time()
        t_ros0 = self._node.get_clock().now()
        # 记录58：默认 ~10× realtime；RTF 健康时几乎不触发。低 RTF 先修 RTF，
        # 不要把 wall cap 放宽当创可贴（可调：control_dt_wall_cap_min/_mult）。
        wall_limit = max(
            float(self.control_dt_wall_cap_min),
            target * float(self.control_dt_wall_cap_mult),
        )
        while time.time() - t_wall0 < wall_limit:
            rclpy.spin_once(self._node, timeout_sec=0.01)
            try:
                elapsed = _duration_to_sec(self._node.get_clock().now() - t_ros0)
            except Exception:
                elapsed = 0.0
            if elapsed >= target:
                return
            _wall_sleep(0.01)
        self._log.warn_throttle(
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
                self._log.info(
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
                self._log.warn(
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
                            self._log.info(
                                "[goal] OK after %d resamples goal=(%.2f,%.2f)",
                                g_try,
                                goal[0],
                                goal[1],
                            )
                        break
                    if g_try == 0 or (g_try + 1) % 5 == 0:
                        self._log.warn(
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
                    self._log.warn(
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
        self._log.err(
            "[spawn] giving up this world=%s after %d pose tries → switch to %s",
            self._world_basename(),
            int(n_pose_tries),
            os.path.basename(nxt),
        )
        try:
            self.switch_world(nxt)
            return os.path.abspath(nxt), True
        except Exception as e:
            self._log.err(
                "[spawn] switch to %s after spawn exhaust failed: %s "
                "— try another world, do NOT abort training",
                os.path.basename(nxt),
                e,
            )
            return os.path.abspath(nxt), False

    def _try_fixed_start_goal(self, start_xy, goal_xy, yaw=None):
        """Place a fixed start/goal on the current world. Returns (start, goal, yaw) or None."""
        start = np.array([float(start_xy[0]), float(start_xy[1])], dtype=np.float64)
        goal = np.array([float(goal_xy[0]), float(goal_xy[1])], dtype=np.float64)
        if yaw is None:
            yaw = float(np.random.uniform(-math.pi, math.pi))
        else:
            yaw = float(yaw)
        if not self._ensure_spawn_ok(
            float(start[0]),
            float(start[1]),
            yaw,
            context="viz_fixed",
            allow_xy_retry=True,
        ):
            return None
        if self.goal_check_enable:
            try:
                x, y, yaw_pose = self._wait_pose()
            except RuntimeError:
                return None
            g_ok, g_reason = self._validate_goal(
                float(goal[0]), float(goal[1]), x, y, yaw_pose
            )
            if not g_ok:
                self._log.warn(
                    "[viz_fixed] goal reject reason=%s goal=(%.2f,%.2f)",
                    g_reason,
                    goal[0],
                    goal[1],
                )
                return None
        return start, goal, yaw

    def reset(
        self,
        count_episode=True,
        start_xy=None,
        goal_xy=None,
        yaw=None,
        allow_world_switch=True,
    ):
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

        placed = None
        if start_xy is not None and goal_xy is not None:
            placed = self._try_fixed_start_goal(start_xy, goal_xy, yaw=yaw)
            if placed is None and not allow_world_switch:
                return None
        if placed is None:
            n_xy = max(
                20, int(self.spawn_xy_max_retries), int(self.spawn_full_reset_retries)
            )
            tried = set()
            skip_pose = False
            while rclpy.ok():
                cur = os.path.abspath(self.current_world) if self.current_world else ""
                if cur:
                    tried.add(cur)
                if not skip_pose:
                    placed = self._try_spawn_start_goal(n_xy)
                    if placed is not None:
                        break
                if not allow_world_switch:
                    return None
                n_report = 0 if skip_pose else n_xy
                skip_pose = False
                path, loaded = self._switch_world_after_spawn_exhaust(n_report, tried)
                if path is None:
                    self._log.err(
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
            self._log.err(
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
        g = Point()
        g.x = float(self._goal[0])
        g.y = float(self._goal[1])
        g.z = 0.0
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
                bits.append("high %+.3f" % r_fwd)
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
            self._log.info(
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
            msg += " high %+.3f" % r_fwd
        if abs(r_spin) > 1e-6:
            msg += " spin %+.3f" % r_spin
        if abs(r_low) > 1e-6:
            msg += " low_effort %+.3f" % r_low
        msg += " | total_ep=%+.2f rho=%.2fm" % (self._ep_reward_accum, rho)
        self._log.info(msg)

    def step(self, action):
        if self._post_switch_model_only_steps > 0:
            self._post_switch_model_only_steps -= 1

        action = np.asarray(action, dtype=np.float32).reshape(2)
        # Symmetric reverse: v ∈ [-max_v, max_v] when allow_reverse (default true)
        v_lo = -self.max_v if self.allow_reverse else 0.0
        action[0] = float(np.clip(action[0], v_lo, self.max_v))
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
            self._log.err(
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

        r_high_val, r_spin_val, effort = self._compute_motion_rewards(vx, w)
        r_forward_val = r_high_val  # metrics alias
        r_low_effort_val = self._update_low_effort_penalty(effort)
        r_explore_val = self._compute_explore_reward(x, y, rho)
        r_slope_traverse = self._compute_slope_traverse_reward(delta_rho)

        reward = (
            r_step_val
            + r_episode_start_val
            + r_prog_val
            + r_explore_val
            + r_high_val
            + r_spin_val
            + r_low_effort_val
            + r_slope_traverse
        )
        r_arrive_val = 0.0
        r_collision_val = 0.0
        r_slope_val = r_slope_traverse
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
        # 3) stuck window → path stuck (terminal); skipped when rho < stuck_near_goal_rho
        # 4) step count >= max_ep_steps → timeout (also near-goal if never arrived)
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
        elif rho < self.stuck_near_goal_rho:
            # Reverse/parallel-park zone: clear hist so exiting 0.7m does not instant-stuck.
            self._stuck_hist = []
            if self._step % 50 == 0:
                self._log.info(
                    "[stuck] skip near_goal rho=%.3f < %.2f (timeout if steps exhaust)",
                    rho,
                    self.stuck_near_goal_rho,
                )
        else:
            stuck, _disp = self._update_stuck(x, y)
            if stuck:
                self._log.info("[stuck] rho=%.3f → done", rho)
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
            r_slope_val = r_slope_traverse + coll_penalty
            reward += coll_penalty
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
            "r_high_effort": r_high_val,
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
        info["x"] = float(x)
        info["y"] = float(y)
        info["yaw"] = float(yaw)
        info["goal_x"] = float(self._goal[0])
        info["goal_y"] = float(self._goal[1])
        info["world"] = self._world_basename()
        self._log_reward(parts, rho, done, info)

        if done:
            self._stop()

        next_state = self._build_state(x, y, yaw)
        return next_state, float(reward), bool(done), info
