# 阶段1 建图 — 空窗专用（禁止与 train_td3 共用 gzserver）
#
# 会弹出：Gazebo(大图+车) + RViz(代价图) + 默认 Flysky USB 遥控
# Ctrl+C → 自动保存 3D/maps_long/arena_trav.{yaml,pgm}
#
# Current operating notes live in 3D/README4.md.

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from scout_workspace_common import workspace_path


def _apply_lidar_profile(context, *args, **kwargs):
    """mapping=密扫少漏；real_rs16≈真车 RS-16（16线·10Hz·密方位）。"""
    profile = LaunchConfiguration("lidar_profile").perform(context).strip().lower()
    if profile in ("real_rs16", "rs16", "real"):
        return [
            SetLaunchConfiguration("lidar_h_samples", "1800"),
            SetLaunchConfiguration("lidar_v_samples", "16"),
            SetLaunchConfiguration("lidar_update_rate", "10"),
            LogInfo(msg="[long_nav_map] lidar_profile=real_rs16 → 1800×16 @10Hz（贴近真车）"),
        ]
    # mapping / dense / default：保持 Declare 默认 720×16@20
    return [
        LogInfo(
            msg="[long_nav_map] lidar_profile=mapping → 用 lidar_h/v/rate 参数（默认 720×16 @20Hz）"
        ),
    ]


def _gate(context, *args, **kwargs):
    from scout_long_nav.isolation import (
        ISOLATION_BANNER,
        MAP_PHASE_BANNER,
        check_training_idle,
    )

    print(ISOLATION_BANNER)
    print(MAP_PHASE_BANNER)
    ok, msg = check_training_idle(allow_gzserver=False)
    print(msg)
    if not ok:
        raise RuntimeError(msg)
    v = LaunchConfiguration("teleop_max_v").perform(context)
    w = LaunchConfiguration("teleop_max_w").perform(context)
    res = LaunchConfiguration("resolution").perform(context)
    h = LaunchConfiguration("lidar_h_samples").perform(context)
    vv = LaunchConfiguration("lidar_v_samples").perform(context)
    hz = LaunchConfiguration("lidar_update_rate").perform(context)
    return [
        LogInfo(msg="[long_nav_map] 隔离通过 — 建图模式（无 A* / 无 TD3）"),
        LogInfo(msg=MAP_PHASE_BANNER.replace("\n", " | ")),
        LogInfo(
            msg=(
                "[long_nav_map] 安全：控仿真请关真车电源；"
                "Flysky USB 仅 → 仿真 /cmd_vel（不碰 CAN）"
            )
        ),
        LogInfo(
            msg=(
                f"[long_nav_map] 建图限速 teleop_max_v={v} m/s "
                f"teleop_max_w={w} rad/s | 栅格 resolution={res} m | "
                f"雷达 gpu_ray {h}x{vv} @{hz}Hz"
            )
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("scout_long_nav")
    mini_pkg = get_package_share_directory("scout_mini_pro_description")
    offroad_pkg = get_package_share_directory("scout_offroad_3d")

    default_world = os.path.join(offroad_pkg, "arena", "bumpy_offroad_full.world")

    map_dir = str(workspace_path("3D", "maps_long", package_name="scout_long_nav"))
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

    # Keyboard teleop needs its own terminal focus (optional backup)
    teleop_prefix = "xterm -T long_nav_teleop -e" if shutil.which("xterm") else ""

    teleop_kb = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix=teleop_prefix,
        parameters=[{"use_sim_time": True}],
        remappings=[("cmd_vel", "/cmd_vel")],
        condition=IfCondition(LaunchConfiguration("use_keyboard")),
    )

    # 直接读 /dev/input/js*；限幅仅建图（不改遥控器 / 不改 TD3）
    joy_nodes = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_joy")),
        actions=[
            Node(
                package="scout_long_nav",
                executable="flysky_js_teleop",
                name="flysky_js_teleop",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "axis_linear": 2,
                        "axis_angular": 0,
                        "scale_linear": LaunchConfiguration("teleop_max_v"),
                        "scale_angular": LaunchConfiguration("teleop_max_w"),
                        "deadzone": 0.08,
                        "enable_button": -1,
                        "rate_hz": 20.0,
                    }
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "nvidia_gl",
                default_value="auto",
                description="建图雷达为 gpu_ray，建议 auto/true 走 NVIDIA",
            ),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.15"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("map_dir", default_value=map_dir),
            DeclareLaunchArgument("map_stem", default_value="arena_trav"),
            DeclareLaunchArgument(
                "seed_map_yaml",
                default_value="",
                description="续建：已存 arena_trav.yaml 路径；空=从零建图",
            ),
            DeclareLaunchArgument("size_m", default_value="80.0"),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.08",
                description="代价图栅格边长(m)；建图默认 0.08",
            ),
            DeclareLaunchArgument(
                "costmap_rate_hz",
                default_value="5.0",
                description="代价图刷新 Hz",
            ),
            DeclareLaunchArgument(
                "teleop_max_v",
                default_value="2.5",
                description="建图遥控线速度上限 m/s（仅本 launch）",
            ),
            DeclareLaunchArgument(
                "teleop_max_w",
                default_value="-1.8",
                description="建图遥控角速度上限 rad/s（负号=转向纠正；仅本 launch）",
            ),
            DeclareLaunchArgument(
                "lidar_profile",
                default_value="mapping",
                description="mapping=720×16@20 少漏扫；real_rs16=1800×16@10 贴近真车 RS-16",
            ),
            DeclareLaunchArgument(
                "lidar_h_samples",
                default_value="720",
                description="建图水平雷达采样（gpu_ray）；可被 lidar_profile 覆盖",
            ),
            DeclareLaunchArgument(
                "lidar_v_samples",
                default_value="16",
                description="建图竖直雷达采样（gpu_ray）",
            ),
            DeclareLaunchArgument(
                "lidar_update_rate",
                default_value="20",
                description="建图雷达 Hz（gpu_ray）",
            ),
            DeclareLaunchArgument(
                "slope_max",
                default_value="0.55",
                description="坡度超此→lethal；略放宽减轻颠簸误杀",
            ),
            DeclareLaunchArgument(
                "beta_rough",
                default_value="25.0",
                description="粗糙度权重；略降减轻震颤假橙色",
            ),
            DeclareLaunchArgument(
                "min_points_per_cell",
                default_value="4",
                description="每格最少点数；抑制零星射线噪点",
            ),
            DeclareLaunchArgument(
                "accum_merge",
                default_value="max",
                description="累积合并：max=障碍更稳；last=本帧覆盖",
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/lidar/points2"),
            DeclareLaunchArgument(
                "use_joy",
                default_value="true",
                description="Flysky USB → /cmd_vel（默认开；真车须断电）",
            ),
            DeclareLaunchArgument(
                "use_keyboard",
                default_value="false",
                description="键盘 teleop xterm（默认关，避免与摇杆抢 /cmd_vel）",
            ),
            OpaqueFunction(function=_apply_lidar_profile),
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
                        "seed_map_yaml": LaunchConfiguration("seed_map_yaml"),
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
                        "stride": 1,
                        "min_cost_viz": 1,
                        "free_stride": 3,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg] if os.path.isfile(rviz_cfg) else [],
            ),
            TimerAction(period=4.0, actions=[joy_nodes, teleop_kb]),
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
                    }
                ],
            ),
            LogInfo(
                msg=(
                    "[long_nav_map] 有车。Ctrl+C 存图 → "
                    + map_dir
                    + "/arena_trav.yaml"
                    + " | 建图限速 2.5/-1.8 | 栅格 0.08 | 雷达 gpu_ray 720x16@20"
                    + " | 翻车监护：停图/静音遥控 → 清半径 → Z阶梯重生 → 再开雷达"
                    + " | 键盘备用: use_keyboard:=true"
                    + " | 关摇杆: use_joy:=false"
                    + " | 续建: seed_map_yaml:=…/arena_trav.yaml"
                )
            ),
        ]
    )
