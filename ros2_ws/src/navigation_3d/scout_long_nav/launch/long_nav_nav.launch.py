# 阶段2 导航 — 空窗专用（禁止与 train_td3 共用 gzserver）
#
# Gazebo+车 + 静态代价图 + Hybrid A* + Arena + TD3(infer 只读 Record 9 仿真权重)
# RViz: 「2D Goal Pose」点终点 → 看绿路径与车跟 TD3 走
#
# Current operating notes live in 3D/README4.md.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from scout_workspace_common import workspace_path
from scout_long_nav.isolation import DEFAULT_WEIGHTS_PREFIX


def _gate(context, *args, **kwargs):
    from scout_long_nav.isolation import (
        DEFAULT_WEIGHTS_PREFIX,
        ISOLATION_BANNER,
        NAV_PHASE_BANNER,
        check_training_idle,
        map_paths,
    )

    print(ISOLATION_BANNER)
    print(NAV_PHASE_BANNER)
    ok, msg = check_training_idle(allow_gzserver=False)
    print(msg)
    if not ok:
        raise RuntimeError(msg)

    map_dir = LaunchConfiguration("map_dir").perform(context)
    stem = LaunchConfiguration("map_stem").perform(context)
    yaml_path, _pgm = map_paths(map_dir, stem)
    if not os.path.isfile(yaml_path):
        raise RuntimeError(
            "【缺图】找不到静态代价图：%s\n"
            "请先跑阶段1建图（有车、能开）：\n"
            "  ros2 launch scout_long_nav long_nav_map.launch.py\n"
            "开一会儿后 Ctrl+C 存盘；或临时烘焙（无窗口）：\n"
            "  python3 -m scout_long_nav.bake_heightmap_trav --out-dir %s --stem %s"
            % (yaml_path, map_dir, stem)
        )
    weights = LaunchConfiguration("weights").perform(context) or DEFAULT_WEIGHTS_PREFIX
    actor = weights + "_actor"
    if not os.path.exists(actor) and not os.path.isfile(actor + ".pt"):
        print("警告：未找到权重文件 %s_actor，TD3 可能起不来" % weights)

    return [
        LogInfo(
            msg=(
                "[long_nav_nav] 图=%s | 权重=%s | gui=开 | 请用 RViz「2D Goal Pose」点终点"
                % (yaml_path, weights)
            )
        ),
        LogInfo(msg=NAV_PHASE_BANNER.replace("\n", " | ")),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("scout_long_nav")
    mini_pkg = get_package_share_directory("scout_mini_pro_description")
    offroad_pkg = get_package_share_directory("scout_offroad_3d")
    cfg = os.path.join(pkg, "config", "long_nav.yaml")

    default_world = os.path.join(offroad_pkg, "arena", "bumpy_offroad_full.world")

    map_dir = str(workspace_path("3D", "maps_long", package_name="scout_long_nav"))
    default_weights = DEFAULT_WEIGHTS_PREFIX
    rviz_cfg = os.path.join(pkg, "rviz", "long_nav_nav.rviz")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mini_pkg, "launch", "gazebo_room.launch.py")
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "world": LaunchConfiguration("world"),
            "x": LaunchConfiguration("spawn_x"),
            "y": LaunchConfiguration("spawn_y"),
            "z": LaunchConfiguration("spawn_z"),
            "yaw": LaunchConfiguration("spawn_yaw"),
            "nvidia_gl": LaunchConfiguration("nvidia_gl"),
        }.items(),
    )

    range_features = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(offroad_pkg, "launch", "range_features.launch.py")
        ),
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
            "publish_td3_range_features": "true",
        }.items(),
    )

    # Record 9: state_dim=150 → elev=4, hidden=1024, IMU roll/pitch enabled.
    # cmd_vel → /cmd_vel_td3；nav_cmd_gate 到点锁轮后再发 /cmd_vel
    infer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(offroad_pkg, "launch", "infer_td3.launch.py")
        ),
        launch_arguments={
            "start_sim": "false",
            "use_real_lidar": "false",
            "auto_goal": "false",
            "pose_source": "odom",
            "odom_topic": "/odom",
            "goal_topic": "/offroad/train_goal",
            "cmd_vel_topic": "/cmd_vel_td3",
            "weights": LaunchConfiguration("weights"),
            "use_sim_time": "true",
            "gui": "false",
            "n_elevation_bins_td3": "4",
            "hidden_dim": "1024",
            "use_imu_in_state": "true",
            "use_wall_rate": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("nvidia_gl", default_value="auto"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.15"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("map_dir", default_value=map_dir),
            DeclareLaunchArgument("map_stem", default_value="arena_trav"),
            DeclareLaunchArgument("weights", default_value=default_weights),
            OpaqueFunction(function=_gate),
            gazebo,
            TimerAction(period=3.0, actions=[range_features]),
            Node(
                package="scout_long_nav",
                executable="pose_stub_node",
                name="pose_stub_node",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": "/odom",
                        "output_odom_topic": "/long_nav/odom",
                    }
                ],
            ),
            Node(
                package="scout_long_nav",
                executable="static_costmap_node",
                name="static_costmap_node",
                output="screen",
                parameters=[
                    {
                        "map_dir": LaunchConfiguration("map_dir"),
                        "map_stem": LaunchConfiguration("map_stem"),
                        "costmap_topic": "/long_nav/costmap",
                        "frame_id": "odom",
                        "rate_hz": 1.0,
                    }
                ],
            ),
            Node(
                package="scout_long_nav",
                executable="costmap_color_cloud_node",
                name="costmap_color_cloud_node",
                output="screen",
                parameters=[
                    {
                        "costmap_topic": "/long_nav/costmap",
                        "cloud_topic": "/long_nav/costmap_rgb",
                        "z": 0.05,
                        "stride": 1,
                    }
                ],
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
            # /goal_pose 由 hybrid_astar_node 快检红线后才发到 /long_nav/goal
            Node(
                package="scout_long_nav",
                executable="nav_cmd_gate_node",
                name="nav_cmd_gate_node",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_in_topic": "/cmd_vel_td3",
                        "cmd_vel_out_topic": "/cmd_vel",
                        "odom_topic": "/long_nav/odom",
                        "goal_topic": "/long_nav/goal",
                        "hold_topic": "/long_nav/cmd_hold",
                        "arrived_topic": "/long_nav/goal_arrived",
                        "goal_tol_m": 2.0,
                        "gate_hz": 50.0,
                    }
                ],
            ),
            Node(
                package="scout_long_nav",
                executable="rollover_guard_node",
                name="rollover_guard_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "model_name": "scout_mini",
                        "z0": 0.23,
                        "z_step": 0.5,
                        "z_max": 3.23,
                        "clear_radius_m": 12.0,
                        "rollover_roll_deg": 55.0,
                        "rollover_pitch_deg": 55.0,
                        "rollover_sum_deg": 75.0,
                        "rollover_grace_sec": 2.5,
                        "odom_topic": "/long_nav/odom",
                        "odom_fallback_topic": "/odom",
                        "imu_topic": "/imu/data",
                        "tick_hz": 10.0,
                        "poll_gz_hz": 5.0,
                        "cmd_hold_topic": "/long_nav/cmd_hold",
                        "publish_cmd_hold": True,
                        "recover_timeout_sec": 20.0,
                        "hold_false_burst": 8,
                    }
                ],
            ),
            # Wait for spawn + lidar before TD3
            TimerAction(period=8.0, actions=[infer]),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg] if os.path.isfile(rviz_cfg) else [],
            ),
            LogInfo(
                msg=(
                    "[long_nav_nav] RViz Fixed Frame=odom →「2D Goal Pose」点终点 → "
                    "红 Path=/long_nav/global_path；Arena 球=/long_nav/arena_waypoints；"
                    "到点≤2m 锁轮，下次 Goal Pose 再开；翻车 Z 阶梯重生"
                )
            ),
        ]
    )
