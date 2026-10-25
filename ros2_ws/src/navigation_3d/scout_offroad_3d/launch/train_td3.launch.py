"""train_td3.launch.py — Gazebo + range_features + online TD3 training (ROS2 Humble).

Usage:
  ros2 launch scout_offroad_3d train_td3.launch.py gui:=false
  ros2 launch scout_offroad_3d train_td3.launch.py gui:=false max_timesteps:=500000

Default first world: random pick from the 4-map train pool (bumpy_offroad … _4).
Omit max_timesteps/start_timesteps/world_switch_interval to use td3_train.yaml.

Real yaml (edit this): ros2_ws/src/navigation_3d/scout_offroad_3d/config/td3_train.yaml

gui:=true: gazebo_room starts gzserver+gzclient together (watchable). Train
restarts gzclient only after a world switch kills it.
"""

import os
import random
import secrets

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from scout_offroad_3d.training_run_contract import fresh_output_error


def _resolve_world(world_arg: str, pkg: str) -> str:
    worlds_dir = os.path.join(pkg, "worlds")
    # Prefer package helper (4 train stems); fall back if import unavailable at launch.
    try:
        from scout_offroad_3d.world_pool import (
            list_train_world_paths,
            resolve_world_path,
        )

        if not world_arg or world_arg == "random":
            cands = list_train_world_paths(worlds_dir)
            if not cands:
                raise RuntimeError("no train *.world under %s" % worlds_dir)
            return random.choice(cands)
        path = resolve_world_path(worlds_dir, world_arg)
        if path:
            return path
        if os.path.isabs(world_arg) and os.path.isfile(world_arg):
            return world_arg
        if os.path.isfile(world_arg):
            return os.path.abspath(world_arg)
        raise RuntimeError("world not found: %s" % world_arg)
    except ImportError:
        pass
    if not world_arg or world_arg == "random":
        # Fallback stems if world_pool not importable yet
        prefer = (
            "bumpy_offroad.world",
            "bumpy_offroad_2.world",
            "bumpy_offroad_3.world",
            "bumpy_offroad_4.world",
        )
        cands = [
            os.path.join(worlds_dir, f)
            for f in prefer
            if os.path.isfile(os.path.join(worlds_dir, f))
        ]
        if not cands:
            raise RuntimeError("no train *.world under %s" % worlds_dir)
        return random.choice(cands)
    if os.path.isabs(world_arg) and os.path.isfile(world_arg):
        return world_arg
    name = world_arg if world_arg.endswith(".world") else world_arg + ".world"
    path = os.path.join(worlds_dir, name)
    if os.path.isfile(path):
        return path
    if os.path.isfile(world_arg):
        return os.path.abspath(world_arg)
    raise RuntimeError("world not found: %s" % world_arg)


def _maybe_int(raw, default):
    if raw is None or (isinstance(raw, str) and not raw.strip()):
        return default
    return int(raw)


def _as_bool(raw):
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def _read_sim_cfg(config_file):
    """Read sim_* + lidar_* from td3_train.yaml (single control point)."""
    try:
        import yaml

        with open(config_file) as f:
            doc = yaml.safe_load(f)
        params = (doc.get("/**") or {}).get("ros__parameters") or {}
        keys = (
            "sim_max_step_size",
            "sim_real_time_update_rate",
            "sim_solver_iters",
            "sim_collision_heightmap_res",
            "lidar_update_rate",
            "lidar_h_samples",
            "lidar_v_samples",
        )
        return {k: params[k] for k in keys if k in params}
    except Exception:
        return {}


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("scout_offroad_3d")
    mini_pkg = get_package_share_directory("scout_mini_pro_description")
    # The initial gzserver is created by the shared launch as a sibling of the
    # train node.  A one-launch nonce explicitly hands just that listener to
    # the node for the first world switch; it is never logged or user supplied.
    gazebo_owner_token = secrets.token_urlsafe(24)
    # Prefer source yaml when developing with --symlink-install (share → build → src).
    config_file = os.path.join(pkg, "config", "td3_train.yaml")

    gui = LaunchConfiguration("gui").perform(context)
    world_arg = LaunchConfiguration("world").perform(context)
    world_path = _resolve_world(world_arg, pkg)

    output_dir = LaunchConfiguration("output_dir").perform(context).strip()
    resume = LaunchConfiguration("resume").perform(context)
    resume_requested = _as_bool(resume)
    max_ts = LaunchConfiguration("max_timesteps").perform(context)
    start_ts = LaunchConfiguration("start_timesteps").perform(context)
    switch_iv = LaunchConfiguration("world_switch_interval").perform(context)
    if resume_requested and not output_dir:
        raise RuntimeError("resume:=true requires an explicit output_dir:=PATH")
    if not resume_requested and output_dir:
        expanded_output = os.path.abspath(
            os.path.expanduser(os.path.expandvars(output_dir))
        )
        output_error = fresh_output_error(expanded_output)
        if output_error:
            raise RuntimeError(
                "refusing fresh output_dir=%s: %s; use an empty/new directory "
                "or omit output_dir" % (expanded_output, output_error)
            )

    # Apply yaml sim params (physics / collision grid) to a cached world copy.
    # current_world keeps the SOURCE path (identity / world_spawn_z lookup by stem);
    # only gzserver loads the patched copy.
    sim_cfg = _read_sim_cfg(config_file)
    lidar_rate = str(sim_cfg.get("lidar_update_rate", 10))
    lidar_h = str(sim_cfg.get("lidar_h_samples", 96))
    lidar_v = str(sim_cfg.get("lidar_v_samples", 8))
    world_path_sim = world_path
    try:
        from scout_offroad_3d.world_sim import prepare_world

        patched = prepare_world(world_path, sim_cfg)
        if patched != world_path:
            world_path_sim = patched
            print(f"[train_td3] sim params applied → {os.path.basename(patched)}")
    except Exception as e:
        print(f"[train_td3] sim patch skipped: {e}")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([FindPackageShare("scout_description"), "urdf", "mini.xacro"]),
                " robot_namespace:=/ enable_ros_control:=false",
                f" lidar_update_rate:={lidar_rate}",
                f" lidar_h_samples:={lidar_h}",
                f" lidar_v_samples:={lidar_v}",
            ]
        ),
        value_type=str,
    )

    want_gui = gui.lower() in ("1", "true", "yes", "on")
    train_params = {
        "robot_description": robot_description,
        "gazebo_gui": want_gui,
        "current_world": world_path,
        "gazebo_owner_token": gazebo_owner_token,
        "resume": resume_requested,
        # The node uses this only to ensure a resume horizon came from this
        # invocation rather than silently inheriting the yaml default.
        "max_timesteps_explicit": bool(max_ts.strip()),
    }
    if output_dir.strip():
        train_params["output_dir"] = output_dir.strip()
    # Empty CLI string = omit → node reads td3_train.yaml. Non-empty overrides yaml.
    if max_ts.strip():
        train_params["max_timesteps"] = _maybe_int(max_ts, 300000)
    if start_ts.strip():
        train_params["start_timesteps"] = _maybe_int(start_ts, 3000)
    if switch_iv.strip():
        train_params["world_switch_interval"] = _maybe_int(switch_iv, 20000)
    viz_en = LaunchConfiguration("viz_eval_enable").perform(context).strip()
    if viz_en:
        train_params["viz_eval_enable"] = viz_en.lower() in ("1", "true", "yes", "on")
    viz_pick = LaunchConfiguration("viz_eval_pick_at_start").perform(context).strip()
    if viz_pick:
        train_params["viz_eval_pick_at_start"] = viz_pick.lower() in (
            "1", "true", "yes", "on"
        )

    # Always start gzserver via gazebo_room. When gui:=true, also start gzclient
    # there (Noetic-style: server+client together after world load). Train only
    # restarts gzclient after world-switch kills it — do NOT defer first GUI to
    # a late Popen that races spawn / Z-retries and freezes Ogre.
    nvidia_gl = LaunchConfiguration("nvidia_gl").perform(context).strip() or "auto"
    # Ensure train_td3_node world-switch Popen sees the same mode as gazebo_scout.
    nvidia_mode_env = SetEnvironmentVariable(name="SCOUT_NVIDIA_GL", value=nvidia_gl.lower())
    try:
        from scout_offroad_3d.gazebo_gl_env import nvidia_gl_environ

        nvidia_prime_envs = [
            SetEnvironmentVariable(name=k, value=v)
            for k, v in nvidia_gl_environ(nvidia_gl).items()
        ]
    except ImportError:
        nvidia_prime_envs = []

    gazebo_room = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mini_pkg, "launch", "gazebo_room.launch.py")
        ),
        launch_arguments={
            "gui": "true" if want_gui else "false",
            "spawn_delay": LaunchConfiguration("spawn_delay").perform(context),
            "x": LaunchConfiguration("x").perform(context),
            "y": LaunchConfiguration("y").perform(context),
            "z": LaunchConfiguration("z").perform(context),
            "yaw": LaunchConfiguration("yaw").perform(context),
            "world": world_path_sim,
            "server_required": "false",
            "lidar_update_rate": lidar_rate,
            "lidar_h_samples": lidar_h,
            "lidar_v_samples": lidar_v,
            "nvidia_gl": nvidia_gl,
            "gazebo_owner_token": gazebo_owner_token,
        }.items(),
    )

    range_features = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "range_features.launch.py")
        ),
        launch_arguments={
            "source": "cloud",
            "cloud_topic": "/lidar/points",
            "cloud_msg_type": "pointcloud",
            "frame_mode": "base",
            "n_sectors": "36",
            "full_circle": "true",
            "n_elevation_bins_td3": "4",
            "td3_elevation_bin_edges_deg": "[-15.0, -7.5, 0.0, 7.5, 15.0]",
            "elevation_bin_edges_deg": "[-15.0, -7.5, 0.0, 7.5, 15.0]",
        }.items(),
    )

    # Shut down Gazebo + feature nodes when training finishes (smoke/CI must not hang).
    # First Ctrl+C starts shutdown plots; default 5s SIGINT→SIGTERM then SIGKILL
    # used to abort PNG generation (exit -9). Give the plot path time to finish.
    train_node = Node(
        package="scout_offroad_3d",
        executable="train_td3_node",
        name="train_td3_node",
        output="screen",
        parameters=[config_file, train_params],
        on_exit=Shutdown(),
        sigterm_timeout="120.0",
        sigkill_timeout="30.0",
    )

    switch_note = (
        "CLI world_switch_interval=%s" % switch_iv.strip()
        if switch_iv.strip()
        else "world_switch_interval from yaml %s" % config_file
    )
    gui_note = (
        "gui=true → gazebo_room gzclient; train delayed 15s so GUI can finish load"
        if want_gui
        else "gui=false"
    )

    # With GUI: let Ogre finish loading before lidar/train hammer the process.
    # Headless: short delay is enough.
    if want_gui:
        return [
            LogInfo(msg=["[train_td3] config=", config_file, " | ", switch_note, " | ", gui_note]),
            nvidia_mode_env,
            *nvidia_prime_envs,
            gazebo_room,
            TimerAction(period=8.0, actions=[range_features]),
            TimerAction(period=15.0, actions=[train_node]),
        ]
    return [
        LogInfo(msg=["[train_td3] config=", config_file, " | ", switch_note, " | ", gui_note]),
        nvidia_mode_env,
        *nvidia_prime_envs,
        gazebo_room,
        range_features,
        TimerAction(period=3.0, actions=[train_node]),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("scout_offroad_3d")
    pkg_models = os.path.join(pkg, "models")
    env_actions = []
    if os.path.isdir(pkg_models):
        # uneven_ground_td3_1..5 live under the package share (not a user model cache); _6 retired
        env_actions.append(
            AppendEnvironmentVariable("GAZEBO_MODEL_PATH", pkg_models)
        )

    return LaunchDescription(
        env_actions
        + [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument(
                "nvidia_gl",
                default_value="auto",
                description="Force Gazebo OpenGL onto NVIDIA: auto|true|false",
            ),
            DeclareLaunchArgument("spawn_delay", default_value="1.0"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            # Slightly above ground so first spawn is visible before train reset.
            DeclareLaunchArgument("z", default_value="0.20"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "output_dir",
                default_value="",
                description=(
                    "Fresh run: empty/new directory only. Resume: required and "
                    "must contain a complete compatible checkpoint."
                ),
            ),
            DeclareLaunchArgument(
                "resume",
                default_value="false",
                description=(
                    "Set true only to continue output_dir; requires explicit "
                    "max_timesteps:=N greater than recorded progress."
                ),
            ),
            DeclareLaunchArgument(
                "max_timesteps",
                default_value="",
                description=(
                    "Fresh run: empty = use td3_train.yaml. Resume: required "
                    "explicit target N, strictly greater than completed t."
                ),
            ),
            DeclareLaunchArgument(
                "start_timesteps",
                default_value="",
                description="Empty = use td3_train.yaml",
            ),
            DeclareLaunchArgument(
                "world_switch_interval",
                default_value="",
                description="Empty = use td3_train.yaml (ros2_ws/src/.../td3_train.yaml)",
            ),
            DeclareLaunchArgument(
                "viz_eval_enable",
                default_value="",
                description="Empty = use yaml; false disables research viz eval (smoke)",
            ),
            DeclareLaunchArgument(
                "viz_eval_pick_at_start",
                default_value="",
                description="Empty = use yaml; false skips pre-train pair gen (smoke)",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="random",
                description="random | basename | full path to .world",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
