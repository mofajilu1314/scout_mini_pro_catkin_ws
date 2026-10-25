"""Stage-A TD3 inference (sim or real robot).

Sim:
  ros2 launch scout_offroad_3d infer_td3.launch.py \\
    weights:=/path/to/run/td3 gui:=false

Real robot topics (Humble side / bridge; 车上最终用 ROS1 移植节点):
  ros2 launch scout_offroad_3d infer_td3.launch.py \\
    start_sim:=false pose_source:=odom auto_goal:=false \\
    cloud_topic:=/rslidar_points cloud_msg_type:=pointcloud2 \\
    max_v:=0.3 weights:=/path/to/run/td3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg = get_package_share_directory("scout_offroad_3d")
    mini_pkg = get_package_share_directory("scout_mini_pro_description")
    config_file = os.path.join(pkg, "config", "td3_train.yaml")

    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    start_sim = LaunchConfiguration("start_sim")
    weights = LaunchConfiguration("weights")

    gazebo_room = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mini_pkg, "launch", "gazebo_room.launch.py")
        ),
        condition=IfCondition(start_sim),
        launch_arguments={
            "gui": gui,
            "world": world,
            "x": LaunchConfiguration("spawn_x"),
            "y": LaunchConfiguration("spawn_y"),
            "z": LaunchConfiguration("spawn_z"),
            "yaw": LaunchConfiguration("spawn_yaw"),
            "spawn_delay": LaunchConfiguration("spawn_delay"),
            "server_required": "true",
            "nvidia_gl": LaunchConfiguration("nvidia_gl"),
        }.items(),
    )

    # Sim: Gazebo PointCloud → bridge. Real: PointCloud2 on /rslidar_points.
    range_features_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "range_features.launch.py")
        ),
        condition=IfCondition(start_sim),
        launch_arguments={
            "source": "cloud",
            "cloud_topic": "/lidar/points",
            "cloud_msg_type": "pointcloud",
            "bridge_cloud2": "true",
            "frame_mode": "base",
            "n_sectors": "36",
            "full_circle": "true",
            "n_elevation_bins_td3": "4",
            "td3_elevation_bin_edges_deg": "[-15.0, -7.5, 0.0, 7.5, 15.0]",
            "elevation_bin_edges_deg": "[-15.0, -7.5, 0.0, 7.5, 15.0]",
        }.items(),
    )

    range_features_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "range_features.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_real_lidar")),
        launch_arguments={
            "source": "cloud",
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "cloud_msg_type": LaunchConfiguration("cloud_msg_type"),
            "bridge_cloud2": "false",
            "frame_mode": "base",
            "n_sectors": "36",
            "full_circle": "true",
            "n_elevation_bins_td3": "4",
            "td3_elevation_bin_edges_deg": "[-15.0, -7.5, 0.0, 7.5, 15.0]",
            "elevation_bin_edges_deg": "[-15.0, -7.5, 0.0, 7.5, 15.0]",
        }.items(),
    )

    infer_node = Node(
        package="scout_offroad_3d",
        executable="infer_td3_node",
        name="infer_td3_node",
        output="screen",
        parameters=[
            config_file,
            {
                "weights_prefix": weights,
                "pose_source": LaunchConfiguration("pose_source"),
                "auto_goal": LaunchConfiguration("auto_goal"),
                "auto_goal_timeout_sec": LaunchConfiguration("auto_goal_timeout_sec"),
                "auto_goal_x": LaunchConfiguration("auto_goal_x"),
                "auto_goal_y": LaunchConfiguration("auto_goal_y"),
                "use_wall_rate": LaunchConfiguration("use_wall_rate"),
                "allow_reverse": LaunchConfiguration("allow_reverse"),
                "min_forward_v": LaunchConfiguration("min_forward_v"),
                "min_forward_rho": LaunchConfiguration("min_forward_rho"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "td3_features_topic": LaunchConfiguration("td3_features_topic"),
                "n_sectors": LaunchConfiguration("n_sectors"),
                "n_elevation_bins_td3": LaunchConfiguration("n_elevation_bins_td3"),
                "hidden_dim": LaunchConfiguration("hidden_dim"),
                "use_imu_in_state": LaunchConfiguration("use_imu_in_state"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "max_v": LaunchConfiguration("max_v"),
                "max_w": LaunchConfiguration("max_w"),
                "goal_topic": LaunchConfiguration("goal_topic"),
            },
        ],
    )

    env_actions = []
    pkg_models = os.path.join(pkg, "models")
    if os.path.isdir(pkg_models):
        # The shared vehicle launcher stays 2D-neutral; 3D owns the terrain
        # models required by its bumpy worlds.
        env_actions.append(AppendEnvironmentVariable("GAZEBO_MODEL_PATH", pkg_models))

    use_sim_time_parameter = SetParameter(
        name="use_sim_time", value=LaunchConfiguration("use_sim_time")
    )

    return LaunchDescription(
        env_actions
        + [
            DeclareLaunchArgument("weights", description="Checkpoint prefix (td3, not td3_actor)"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "nvidia_gl",
                default_value="auto",
                description="Force Gazebo OpenGL onto NVIDIA: auto|true|false",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(pkg, "worlds", "bumpy_offroad.world"),
            ),
            DeclareLaunchArgument("spawn_z", default_value="0.15"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("spawn_delay", default_value="1.0"),
            DeclareLaunchArgument("start_sim", default_value="true"),
            # Real robot: start_sim:=false use_real_lidar:=true
            DeclareLaunchArgument("use_real_lidar", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("pose_source", default_value="gazebo"),
            DeclareLaunchArgument("auto_goal", default_value="true"),
            DeclareLaunchArgument("auto_goal_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("auto_goal_x", default_value="3.0"),
            DeclareLaunchArgument("auto_goal_y", default_value="2.0"),
            DeclareLaunchArgument("use_wall_rate", default_value="true"),
            DeclareLaunchArgument("allow_reverse", default_value="true"),
            DeclareLaunchArgument("min_forward_v", default_value="0.0"),
            DeclareLaunchArgument("min_forward_rho", default_value="0.6"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("td3_features_topic", default_value="/offroad/td3_range_features"),
            DeclareLaunchArgument("n_sectors", default_value="36"),
            DeclareLaunchArgument("n_elevation_bins_td3", default_value="4"),
            DeclareLaunchArgument("hidden_dim", default_value="1024"),
            DeclareLaunchArgument(
                "use_imu_in_state",
                default_value="true",
                description="If false, state_dim = elev*sectors+4 (R8=112)",
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "goal_topic",
                default_value="/offroad/train_goal",
                description="geometry_msgs/Point goal for TD3 (stage B may remap)",
            ),
            # Scout Mini Pro：实车用 cloud_topic:=/rslidar_points cloud_msg_type:=pointcloud2 max_v:=0.3
            DeclareLaunchArgument("cloud_topic", default_value="/rslidar_points"),
            DeclareLaunchArgument("cloud_msg_type", default_value="pointcloud2"),
            DeclareLaunchArgument("max_v", default_value="2.3"),
            DeclareLaunchArgument("max_w", default_value="1.8"),
            # Declarations above must execute before this parameter consumes
            # LaunchConfiguration("use_sim_time").
            use_sim_time_parameter,
            gazebo_room,
            range_features_sim,
            range_features_real,
            TimerAction(period=4.0, actions=[infer_node]),
        ]
    )
