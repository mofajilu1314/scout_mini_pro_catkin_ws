"""range_features.launch.py — sector features from lidar point cloud / scan."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="cloud"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("cloud_topic", default_value="/lidar/points"),
        DeclareLaunchArgument("cloud_msg_type", default_value="pointcloud"),
        # Gazebo block_laser needs PointCloud→PointCloud2 bridge; real RSLidar is already Cloud2
        DeclareLaunchArgument("bridge_cloud2", default_value="true"),
        DeclareLaunchArgument("n_sectors", default_value="36"),
        DeclareLaunchArgument("max_range", default_value="5.0"),
        DeclareLaunchArgument("min_range", default_value="0.18"),
        DeclareLaunchArgument("full_circle", default_value="true"),
        DeclareLaunchArgument("frame_mode", default_value="base"),
        DeclareLaunchArgument("publish_elevation_features", default_value="true"),
        DeclareLaunchArgument("n_elevation_bins", default_value="4"),
        DeclareLaunchArgument("n_front_sectors", default_value="12"),
        DeclareLaunchArgument("publish_td3_range_features", default_value="true"),
        DeclareLaunchArgument("n_elevation_bins_td3", default_value="4"),
        DeclareLaunchArgument(
            "td3_elevation_bin_edges_deg",
            default_value="[-15.0, -7.5, 0.0, 7.5, 15.0]",
        ),
        DeclareLaunchArgument(
            "elevation_bin_edges_deg",
            default_value="[-15.0, -7.5, 0.0, 7.5, 15.0]",
        ),
        DeclareLaunchArgument("filter_body_box", default_value="true"),

        Node(
            package="scout_offroad_3d",
            executable="pointcloud_bridge_node",
            name="pointcloud_bridge_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("bridge_cloud2")),
            parameters=[{
                "in_topic": LaunchConfiguration("cloud_topic"),
                "out_topic": "/lidar/points2",
            }],
        ),

        Node(
            package="scout_offroad_3d",
            executable="range_features_node",
            name="range_features_node",
            output="screen",
            parameters=[{
                "source": LaunchConfiguration("source"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "cloud_msg_type": LaunchConfiguration("cloud_msg_type"),
                "n_sectors": LaunchConfiguration("n_sectors"),
                "max_range": LaunchConfiguration("max_range"),
                "min_range": LaunchConfiguration("min_range"),
                "full_circle": LaunchConfiguration("full_circle"),
                "frame_mode": LaunchConfiguration("frame_mode"),
                "publish_elevation_features": LaunchConfiguration(
                    "publish_elevation_features"
                ),
                "n_elevation_bins": LaunchConfiguration("n_elevation_bins"),
                "n_front_sectors": LaunchConfiguration("n_front_sectors"),
                "publish_td3_range_features": LaunchConfiguration(
                    "publish_td3_range_features"
                ),
                "n_elevation_bins_td3": LaunchConfiguration("n_elevation_bins_td3"),
                "td3_elevation_bin_edges_deg": LaunchConfiguration(
                    "td3_elevation_bin_edges_deg"
                ),
                "elevation_bin_edges_deg": LaunchConfiguration(
                    "elevation_bin_edges_deg"
                ),
                "filter_body_box": LaunchConfiguration("filter_body_box"),
            }],
        ),
    ])
