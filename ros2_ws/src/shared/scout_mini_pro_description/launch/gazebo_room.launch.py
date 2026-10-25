"""Gazebo + official Scout Mini spawn (no nav stack).

ROS1 equivalent: gazebo_room_official.launch

Usage (22.04 + Humble):
  ros2 launch scout_mini_pro_description gazebo_room.launch.py
  ros2 launch scout_mini_pro_description gazebo_room.launch.py world:=/path/to.world gui:=true
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _gazebo_port_preflight() -> tuple[bool, str]:
    """Run the non-destructive Gazebo port preflight before creating actions."""
    try:
        pkg_share = get_package_share_directory("scout_mini_pro_description")
        script = os.path.join(pkg_share, "scripts", "gzserver_clean_prefix.sh")
        if not os.path.isfile(script):
            return False, "Gazebo preflight script is unavailable: %s" % script
        result = subprocess.run(["bash", script, "/bin/true"], check=False)
        if result.returncode == 0:
            return True, ""
        return False, "Gazebo port preflight exited with status %s" % result.returncode
    except Exception as exc:
        return False, "Gazebo port preflight could not run: %s" % exc


def _expand_mini_urdf(
    lidar_update_rate: str = "10",
    lidar_h_samples: str = "96",
    lidar_v_samples: str = "8",
) -> tuple:
    """Expand mini.xacro once → (urdf_xml, urdf_file_path).

    Humble joint_state_publisher does NOT read the robot_description *parameter*;
    it only accepts a URDF file argv or the /robot_description topic. Without a
    file it sits on 'Waiting for robot_description...' and, with use_sim_time,
    hangs on shutdown after gzserver dies → SIGKILL → Ubuntu Crash report.
    """
    scout_share = get_package_share_directory("scout_description")
    xacro_path = os.path.join(scout_share, "urdf", "mini.xacro")
    cmd = [
        "xacro",
        xacro_path,
        "robot_namespace:=/",
        "enable_ros_control:=false",
        f"lidar_update_rate:={lidar_update_rate}",
        f"lidar_h_samples:={lidar_h_samples}",
        f"lidar_v_samples:={lidar_v_samples}",
    ]
    urdf_xml = subprocess.check_output(cmd, text=True)
    # Persist for JSP argv; cleaned up with process (tmp under /tmp is fine).
    fd, urdf_path = tempfile.mkstemp(prefix="scout_mini_", suffix=".urdf")
    with os.fdopen(fd, "w") as f:
        f.write(urdf_xml)
    return urdf_xml, urdf_path


def _launch_setup(context, *args, **kwargs):
    preflight_ok, preflight_error = _gazebo_port_preflight()
    if not preflight_ok:
        return [
            LogInfo(
                msg=(
                    "[gazebo_room] ERROR: %s. No Gazebo process was started; "
                    "stop the session that owns port 11345 and retry."
                    % preflight_error
                )
            ),
            Shutdown(reason=preflight_error),
        ]

    pkg_share = FindPackageShare("scout_mini_pro_description")

    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    server_required = LaunchConfiguration("server_required")
    nvidia_gl = LaunchConfiguration("nvidia_gl")
    gazebo_owner_token = LaunchConfiguration("gazebo_owner_token")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    spawn_delay = LaunchConfiguration("spawn_delay")
    lidar_update_rate = LaunchConfiguration("lidar_update_rate").perform(context)
    lidar_h_samples = LaunchConfiguration("lidar_h_samples").perform(context)
    lidar_v_samples = LaunchConfiguration("lidar_v_samples").perform(context)

    urdf_xml, urdf_path = _expand_mini_urdf(
        lidar_update_rate, lidar_h_samples, lidar_v_samples
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "gazebo_scout.launch.py"])
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "server_required": server_required,
            "nvidia_gl": nvidia_gl,
            # Empty for normal 2D/shared use.  TD3 supplies a per-launch
            # nonce so it can safely take over its initial server at switch.
            "gazebo_owner_token": gazebo_owner_token,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": urdf_xml,
                "use_sim_time": True,
                # Wall-stamped /joint_states otherwise never match RViz sim time → wheel TF errors
                "ignore_timestamp": True,
            }
        ],
    )

    # File argv → configure immediately (no topic wait).
    # use_sim_time=True so wheel TF stamps match /odom and RViz (use_sim_time).
    # After world-switch, /clock returns with new gzserver; brief gap is OK.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        arguments=[urdf_path],
        parameters=[{"use_sim_time": True, "rate": 20}],
    )

    spawn_scout = Node(
        package="scout_mini_pro_description",
        executable="spawn_scout_clean_prefix",
        name="spawn_scout_mini_official",
        output="screen",
        parameters=[
            {
                "robot_description": urdf_xml,
                "model_name": "scout_mini",
                # TimerAction already waited spawn_delay; do not sleep again inside the node.
                "spawn_delay": 0.0,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
            }
        ],
    )

    odom_to_tf = Node(
        package="scout_mini_pro_description",
        executable="odom_to_tf_official",
        name="odom_to_tf_official",
        output="screen",
        parameters=[{"odom_topic": "/odom", "use_sim_time": True}],
    )

    return [
        SetParameter(name="use_sim_time", value=True),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        odom_to_tf,
        TimerAction(period=spawn_delay, actions=[spawn_scout]),
    ]


def generate_launch_description():
    pkg_share = FindPackageShare("scout_mini_pro_description")

    declared = [
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.20"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("spawn_delay", default_value="2.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("lidar_update_rate", default_value="10"),
        DeclareLaunchArgument("lidar_h_samples", default_value="96"),
        DeclareLaunchArgument("lidar_v_samples", default_value="8"),
        DeclareLaunchArgument(
            "nvidia_gl",
            default_value="auto",
            description="Force Gazebo OpenGL onto NVIDIA: auto|true|false "
            "(passed to gazebo_scout; world-switch reads SCOUT_NVIDIA_GL).",
        ),
        DeclareLaunchArgument(
            "server_required",
            default_value="true",
            description="If true, gzserver exit shuts down launch (standalone room). "
            "Set false when train_td3 manages world switch.",
        ),
        DeclareLaunchArgument(
            "gazebo_owner_token",
            default_value="",
            description=(
                "Internal TD3 handoff nonce; leave empty for standalone/shared launches."
            ),
        ),
        DeclareLaunchArgument(
            "world",
            default_value=[
                PathJoinSubstitution([pkg_share, "world", "room_1.world"])
            ],
        ),
    ]

    # 3D launch files add their own package-local terrain models before
    # including this shared vehicle/world launcher.  Do not probe a 3D package
    # here: the 2D/shared launch path must be independently usable.  Optional
    # external Gazebo assets are supplied by the caller's GAZEBO_MODEL_PATH;
    # this shared launcher never guesses a user-local cache.

    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])
