# DO NOT RUN while TD3 training Gazebo is up.
# Planning stack only: costmap + Hybrid A* + Arena + bridge (no Gazebo).

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
        LogInfo(msg="[scout_long_nav] stage_c_planning: NO Gazebo in this launch."),
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
        Node(
            package="scout_long_nav",
            executable="cloud_to_costmap_node",
            name="cloud_to_costmap_node",
            output="screen",
            parameters=[cfg, {"cloud_topic": LaunchConfiguration("cloud_topic")}],
        ),
        Node(
            package="scout_long_nav",
            executable="hybrid_astar_node",
            name="hybrid_astar_node",
            output="screen",
            parameters=[cfg],
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
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_odom", default_value="/odom"),
            DeclareLaunchArgument("cloud_topic", default_value="/lidar/points2"),
            OpaqueFunction(function=_setup),
        ]
    )
