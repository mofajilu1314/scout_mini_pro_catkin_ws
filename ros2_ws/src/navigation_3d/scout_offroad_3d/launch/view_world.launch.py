"""Gazebo GUI + .world only (no robot spawn). For inspecting/editing training maps."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def _merge_model_path(default: str) -> str:
    """Keep explicit external model directories without probing user caches."""
    configured = os.environ.get("GAZEBO_MODEL_PATH", "").strip()
    if configured in ("", ":"):
        return default
    default_parts = [part for part in default.split(os.pathsep) if part]
    extra_parts = [part for part in configured.split(os.pathsep) if part]
    return os.pathsep.join(default_parts + [part for part in extra_parts if part not in default_parts])


def _gazebo_env(pkg_share: str, nvidia_mode: str = "auto") -> dict:
    prefix = os.environ.get("GAZEBO_INSTALL_PREFIX", "/usr")
    base = os.path.join(prefix, "share", "gazebo-11")
    plugins = os.path.join(prefix, "lib", "x86_64-linux-gnu", "gazebo-11", "plugins")
    models = os.path.join(base, "models")
    pkg_models = os.path.join(pkg_share, "models")
    if os.path.isdir(pkg_models):
        models = pkg_models + os.pathsep + models
    # An arena can intentionally depend on an externally installed model.
    # Keep a caller's explicit GAZEBO_MODEL_PATH, but never silently discover
    # a user-local cache.
    models = _merge_model_path(models)
    env = {
        "GAZEBO_MODEL_PATH": models,
        "GAZEBO_PLUGIN_PATH": plugins,
        "GAZEBO_RESOURCE_PATH": base,
        "SCOUT_NVIDIA_GL": (nvidia_mode or "auto").strip().lower() or "auto",
    }
    try:
        from scout_offroad_3d.gazebo_gl_env import nvidia_gl_environ

        env.update(nvidia_gl_environ(nvidia_mode))
    except ImportError:
        pass
    return env


def _port_11345_status() -> tuple[bool, str]:
    """Inspect Gazebo's master port without taking ownership of its listener."""
    details = []
    try:
        result = subprocess.run(
            ["ss", "-lntp"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=3,
        )
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 4 and fields[3].endswith(":11345"):
                details.append(line.strip())
    except (OSError, subprocess.SubprocessError) as exc:
        # A failed preflight must not silently start another Gazebo session.
        return True, "unable to inspect port 11345: %s" % exc

    if details:
        return True, "\n".join(details)
    return False, ""


def _launch_gazebo(context, *args, **kwargs):
    pkg = get_package_share_directory("scout_offroad_3d")
    world_arg = LaunchConfiguration("world").perform(context)
    nvidia_mode = LaunchConfiguration("nvidia_gl").perform(context).strip() or "auto"
    # The installed share can resolve through a build link to the active
    # source world.  Gazebo Save As must overwrite this resolved file.
    # NOTE: view_world is for EDITING maps — never patch to a cache copy here,
    # or Save As would write to the cache instead of the source world.
    world_real = os.path.realpath(world_arg)
    gz_env = _gazebo_env(pkg, nvidia_mode)

    port_busy, port_details = _port_11345_status()
    if port_busy:
        message = (
            "[view_world] ERROR: Gazebo port 11345 is occupied by another "
            "session. No process was stopped. Close the owning launch in its "
            "original terminal, then retry.\n"
            "[view_world] Listener details:\n%s" % (port_details or "unavailable")
        )
        return [
            LogInfo(msg=message),
            EmitEvent(event=Shutdown(reason="Gazebo port 11345 is already in use")),
        ]

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            world_real,
            "-slibgazebo_ros_init.so",
            "-slibgazebo_ros_factory.so",
            "-slibgazebo_ros_force_system.so",
        ],
        output="screen",
        additional_env=gz_env,
    )
    gzclient = ExecuteProcess(
        cmd=["gzclient"], output="screen", additional_env=gz_env
    )

    env_actions = [
        SetEnvironmentVariable(name=k, value=v) for k, v in gz_env.items()
    ]

    return [
        LogInfo(
            msg=(
                f"[view_world] world arg: {world_arg}\n"
                f"[view_world] REALPATH (File→Save As must overwrite THIS file):\n"
                f"             {world_real}\n"
                "[view_world] Keep edited worlds in the active ROS2 package; "
                "legacy checkout copies are not launch inputs."
            )
        ),
        *env_actions,
        gzserver,
        gzclient,
    ]


def generate_launch_description():
    pkg = get_package_share_directory("scout_offroad_3d")
    default_world = os.path.realpath(
        os.path.join(pkg, "worlds", "bumpy_offroad.world")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "nvidia_gl",
                default_value="auto",
                description="Force Gazebo OpenGL onto NVIDIA: auto|true|false",
            ),
            OpaqueFunction(function=_launch_gazebo),
        ]
    )
