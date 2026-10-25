# DO NOT RUN while TD3 training Gazebo is up.
# Idle-window only. Prefer: DRY_RUN smoke script first.
#
# Stage B: fake Path + Arena look-ahead + goal bridge (no Gazebo, no TD3).
# To attach TD3 later: separately launch infer with start_sim:=false on DOMAIN 41
# AFTER training is stopped — never from this file while gzserver is busy.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("scout_long_nav")
    cfg = os.path.join(pkg, "config", "long_nav.yaml")
    return [
        LogInfo(
            msg=(
                "[scout_long_nav] stage_b_fake_path: NO Gazebo. "
                "Do not run alongside TD3 training simulation."
            )
        ),
        Node(
            package="scout_long_nav",
            executable="fake_path_node",
            name="fake_path_node",
            output="screen",
            parameters=[cfg, {"waypoints_xy": LaunchConfiguration("waypoints_xy")}],
        ),
        Node(
            package="scout_long_nav",
            executable="arena_local_goal_node",
            name="arena_local_goal_node",
            output="screen",
            parameters=[cfg],
        ),
        Node(
            package="scout_long_nav",
            executable="goal_bridge_node",
            name="goal_bridge_node",
            output="screen",
            parameters=[cfg],
        ),
        Node(
            package="scout_long_nav",
            executable="pose_stub_node",
            name="pose_stub_node",
            output="screen",
            parameters=[
                {
                    "input_odom_topic": LaunchConfiguration("input_odom"),
                    "output_odom_topic": "/long_nav/odom",
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_xy",
                default_value="0,0;10,0;20,5;30,5",
            ),
            DeclareLaunchArgument(
                "input_odom",
                default_value="/odom",
                description="Existing odom to remap; leave unused if no odom publisher",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
