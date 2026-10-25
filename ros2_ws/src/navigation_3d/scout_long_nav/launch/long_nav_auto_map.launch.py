# 自动覆盖建图 — 空窗专用（禁止与 train_td3 共用 gzserver）
#
# Gazebo + 累积代价图 + Hybrid A* + Arena + Record 9 TD3（只读）+ coverage_supervisor
# 无遥控、无训练；ROI ±35 m（接近 80×80 大图内侧），覆盖至每格 hit≥2（200%）后存图
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
        AUTO_MAP_PHASE_BANNER,
        DEFAULT_WEIGHTS_PREFIX,
        ISOLATION_BANNER,
        check_training_idle,
    )

    print(ISOLATION_BANNER)
    print(AUTO_MAP_PHASE_BANNER)
    ok, msg = check_training_idle(allow_gzserver=False)
    print(msg)
    if not ok:
        raise RuntimeError(msg)

    weights = LaunchConfiguration("weights").perform(context) or DEFAULT_WEIGHTS_PREFIX
    actor = weights + "_actor"
    if not os.path.exists(actor) and not os.path.isfile(actor + ".pt"):
        print("警告：未找到权重文件 %s_actor，TD3 可能起不来" % weights)

    return [
        LogInfo(
            msg=(
                "[long_nav_auto_map] 权重=%s | 无 Flysky | coverage → /long_nav/coverage"
                % weights
            )
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("scout_long_nav")
    mini_pkg = get_package_share_directory("scout_mini_pro_description")
    offroad_pkg = get_package_share_directory("scout_offroad_3d")
    cfg = os.path.join(pkg, "config", "long_nav.yaml")

    default_world = os.path.join(offroad_pkg, "arena", "bumpy_offroad_full.world")

    map_dir = str(workspace_path("3D", "maps_long", package_name="scout_long_nav"))
    default_weights = DEFAULT_WEIGHTS_PREFIX
    rviz_cfg = os.path.join(pkg, "rviz", "long_nav_map.rviz")

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
            "lidar_h_samples": LaunchConfiguration("lidar_h_samples"),
            "lidar_v_samples": LaunchConfiguration("lidar_v_samples"),
            "lidar_update_rate": LaunchConfiguration("lidar_update_rate"),
        }.items(),
    )

    range_features = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(offroad_pkg, "launch", "range_features.launch.py")
        ),
        launch_arguments={
            "source": "cloud",
            "cloud_topic": "/lidar/points2",
            "cloud_msg_type": "pointcloud2",
            "bridge_cloud2": "false",  # pointcloud_bridge already started below
            "frame_mode": "base",
            "n_sectors": "36",
            "full_circle": "true",
            "n_elevation_bins_td3": "4",
            "td3_elevation_bin_edges_deg": "[-15.0, -7.5, 0.0, 7.5, 15.0]",
            "publish_td3_range_features": "true",
        }.items(),
    )

    # Record 9 architecture is fixed by its archived configuration snapshot.
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
            # 400×400 @0.2m：整张 80×80 大图；避免 OccupancyGrid 过大撑爆订阅端
            DeclareLaunchArgument("size_m", default_value="80.0"),
            DeclareLaunchArgument("resolution", default_value="0.2"),
            DeclareLaunchArgument("costmap_rate_hz", default_value="4.0"),
            DeclareLaunchArgument("lidar_h_samples", default_value="720"),
            DeclareLaunchArgument("lidar_v_samples", default_value="16"),
            DeclareLaunchArgument("lidar_update_rate", default_value="20"),
            DeclareLaunchArgument(
                "slope_max",
                default_value="1.0",
                description="坡度超此→lethal；大图80×80×2m+树，需比训练小图更宽",
            ),
            DeclareLaunchArgument(
                "beta_rough",
                default_value="12.0",
                description="粗糙度权重；降一点减轻雷达扫到树干假高代价",
            ),
            DeclareLaunchArgument(
                "alpha_slope",
                default_value="50.0",
                description="坡度项权重",
            ),
            DeclareLaunchArgument("min_points_per_cell", default_value="4"),
            DeclareLaunchArgument("accum_merge", default_value="max"),
            DeclareLaunchArgument("cloud_topic", default_value="/lidar/points2"),
            # 围墙约 ±40.5；起终点采样留 5 m 边距 → ±35
            DeclareLaunchArgument("xy_min", default_value="-35.0"),
            DeclareLaunchArgument("xy_max", default_value="35.0"),
            DeclareLaunchArgument("cell_m", default_value="1.0"),
            DeclareLaunchArgument("target_hits", default_value="2"),
            DeclareLaunchArgument("timeout_sec", default_value="90.0"),
            DeclareLaunchArgument("min_sep_m", default_value="12.0"),
            OpaqueFunction(function=_gate),
            gazebo,
            Node(
                package="scout_offroad_3d",
                executable="pointcloud_bridge_node",
                name="pointcloud_bridge_node",
                output="screen",
                parameters=[
                    {
                        "in_topic": "/lidar/points",
                        "out_topic": "/lidar/points2",
                    }
                ],
            ),
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
                executable="cloud_to_costmap_node",
                name="cloud_to_costmap_node",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": LaunchConfiguration("cloud_topic"),
                        "odom_topic": "/long_nav/odom",
                        "costmap_topic": "/long_nav/costmap",
                        "size_m": LaunchConfiguration("size_m"),
                        "resolution": LaunchConfiguration("resolution"),
                        "slope_max": LaunchConfiguration("slope_max"),
                        "alpha_slope": LaunchConfiguration("alpha_slope"),
                        "beta_rough": LaunchConfiguration("beta_rough"),
                        "min_points_per_cell": LaunchConfiguration(
                            "min_points_per_cell"
                        ),
                        "accum_merge": LaunchConfiguration("accum_merge"),
                        "fixed_center": True,
                        "center_x": 0.0,
                        "center_y": 0.0,
                        "accumulate": True,
                        "map_frame": "odom",
                        "frame_id": "odom",
                        "rate_hz": LaunchConfiguration("costmap_rate_hz"),
                    }
                ],
            ),
            Node(
                package="scout_long_nav",
                executable="save_costmap_node",
                name="save_costmap_node",
                output="screen",
                parameters=[
                    {
                        "costmap_topic": "/long_nav/costmap",
                        "map_dir": LaunchConfiguration("map_dir"),
                        "map_stem": LaunchConfiguration("map_stem"),
                        "save_on_shutdown": True,
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
                        "stride": 2,
                        "min_cost_viz": 1,
                        "free_stride": 4,
                    }
                ],
            ),
            TimerAction(period=3.0, actions=[range_features]),
            # A* 可选；TD3 已由 supervisor 直发 /offroad/train_goal，避免大图订阅崩溃拖死整链
            Node(
                package="scout_long_nav",
                executable="hybrid_astar_node",
                name="hybrid_astar_node",
                output="screen",
                parameters=[
                    cfg,
                    {
                        "xy_resolution": 1.0,
                        "max_iterations": 8000,
                        "replan_hz": 0.25,
                    },
                ],
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
                executable="goal_pose_relay_node",
                name="goal_pose_relay_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/goal_pose",
                        "output_topic": "/long_nav/goal",
                        "force_frame_id": "odom",
                    }
                ],
            ),
            TimerAction(period=8.0, actions=[infer]),
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="scout_long_nav",
                        executable="coverage_supervisor_node",
                        name="coverage_supervisor_node",
                        output="screen",
                        parameters=[
                            {
                                "xy_min": LaunchConfiguration("xy_min"),
                                "xy_max": LaunchConfiguration("xy_max"),
                                "cell_m": LaunchConfiguration("cell_m"),
                                "target_hits": LaunchConfiguration("target_hits"),
                                "timeout_sec": LaunchConfiguration("timeout_sec"),
                                "min_sep_m": LaunchConfiguration("min_sep_m"),
                                "model_name": "scout_mini",
                                "spawn_z": LaunchConfiguration("spawn_z"),
                                "odom_topic": "/long_nav/odom",
                                "imu_topic": "/imu/data",
                                "costmap_topic": "/long_nav/costmap",
                                "goal_topic": "/goal_pose",
                                "cmd_vel_topic": "/cmd_vel",
                                "coverage_topic": "/long_nav/coverage",
                                "save_service": "/save_costmap_node/save",
                                "start_delay_sec": 2.0,
                                "shutdown_when_done": False,
                            }
                        ],
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg] if os.path.isfile(rviz_cfg) else [],
            ),
            LogInfo(
                msg=(
                    "[long_nav_auto_map] 自动扫图中 | echo /long_nav/coverage | "
                    "Ctrl+C 也会存图 → "
                    + map_dir
                    + "/arena_trav.yaml"
                )
            ),
        ]
    )
