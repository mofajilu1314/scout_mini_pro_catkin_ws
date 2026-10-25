"""Gazebo server/client with sane GAZEBO_* paths (avoids gazebo_ros empty merge -> ":")."""

import os
from os import environ, pathsep

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _merge_path(pkg_paths: str, env_key: str) -> str:
    extra = environ.get(env_key, "").strip()
    if extra in ("", ":"):
        extra = ""
    if pkg_paths and extra:
        return pkg_paths + pathsep + extra
    return pkg_paths or extra


def _model_path_with_system_default() -> str:
    """Preserve explicit model roots while retaining Gazebo's system models."""
    _base, default_models, _plugins = _default_gazebo_paths()
    configured = _merge_path("", "GAZEBO_MODEL_PATH")
    if not configured:
        return default_models
    parts = [part for part in configured.split(pathsep) if part]
    if default_models not in parts:
        parts.append(default_models)
    return pathsep.join(parts)


def _default_gazebo_paths():
    prefix = environ.get("GAZEBO_INSTALL_PREFIX", "/usr")
    base = os.path.join(prefix, "share", "gazebo-11")
    plugins = os.path.join(prefix, "lib", "x86_64-linux-gnu", "gazebo-11", "plugins")
    models = os.path.join(base, "models")
    # External assets remain opt-in through GAZEBO_MODEL_PATH.  Automatically
    # scanning a user-local model cache makes launches non-reproducible.
    return base, models, plugins


def _nvidia_gl_environ(mode: str = "auto") -> dict:
    """Return generic NVIDIA GL variables without importing a 3D package."""
    m = (mode or "auto").strip().lower()
    if m in ("false", "0", "no", "off"):
        return {}
    inject = m in ("true", "1", "yes", "on")
    if not inject and not os.path.exists("/dev/nvidia0"):
        try:
            import subprocess

            result = subprocess.run(
                ["nvidia-smi"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
                check=False,
            )
            inject = result.returncode == 0
        except Exception:
            inject = False
    if not inject:
        return {}
    return {
        "__NV_PRIME_RENDER_OFFLOAD": "1",
        "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        "__VK_LAYER_NV_optimus": "NVIDIA_only",
    }


def _launch_setup(context, *args, **kwargs):
    base, _default_models, default_plugins = _default_gazebo_paths()

    model = _model_path_with_system_default()
    plugin = _merge_path("", "GAZEBO_PLUGIN_PATH") or default_plugins
    media = _merge_path("", "GAZEBO_RESOURCE_PATH") or base

    nvidia_mode = LaunchConfiguration("nvidia_gl").perform(context).strip() or "auto"
    gazebo_owner_token = LaunchConfiguration("gazebo_owner_token").perform(context).strip()
    env = {
        "GAZEBO_MODEL_PATH": model,
        "GAZEBO_PLUGIN_PATH": plugin,
        "GAZEBO_RESOURCE_PATH": media,
        "SCOUT_NVIDIA_GL": nvidia_mode.lower(),
    }
    # A TD3 train launch may provide a one-shot handoff token.  It is added to
    # the gzserver child only (not the ambient launch environment) so later
    # code can prove this exact initial listener is safe to stop on map switch.
    if gazebo_owner_token:
        env["SCOUT_GAZEBO_OWNER_TOKEN"] = gazebo_owner_token
    env.update(_nvidia_gl_environ(nvidia_mode))

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    server_required = LaunchConfiguration("server_required")

    cmd = [
        "gzserver",
        world,
        "-slibgazebo_ros_init.so",
        "-slibgazebo_ros_factory.so",
        "-slibgazebo_ros_force_system.so",
    ]

    gzserver_required = ExecuteProcess(
        cmd=cmd,
        output="both",
        additional_env=env,
        on_exit=Shutdown(),
        condition=IfCondition(server_required),
    )
    gzserver = ExecuteProcess(
        cmd=cmd,
        output="both",
        additional_env=env,
        condition=UnlessCondition(server_required),
    )

    # SetEnvironmentVariable so gazebo_ros gzclient.launch inherits NVIDIA PRIME.
    env_actions = [
        SetEnvironmentVariable(name=k, value=v)
        for k, v in env.items()
        if k != "SCOUT_GAZEBO_OWNER_TOKEN"
    ]

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
        ),
        condition=IfCondition(gui),
    )

    return env_actions + [gzserver_required, gzserver, gzclient]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=""),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("server_required", default_value="false"),
            DeclareLaunchArgument(
                "nvidia_gl",
                default_value="auto",
                description="Force Gazebo OpenGL onto NVIDIA: auto|true|false",
            ),
            DeclareLaunchArgument(
                "gazebo_owner_token",
                default_value="",
                description="Internal TD3 initial-gzserver handoff nonce.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
