#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo block_laser publishes sensor_msgs/PointCloud; republish as PointCloud2."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs_py import point_cloud2


def _cloud_to_cloud2(cloud: PointCloud) -> PointCloud2:
    points = []
    for p in cloud.points:
        if abs(p.x) < 1e-6 and abs(p.y) < 1e-6 and abs(p.z) < 1e-6:
            continue
        points.append((p.x, p.y, p.z))
    return point_cloud2.create_cloud_xyz32(cloud.header, points)


class PointCloudBridge(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_bridge_node")
        self.declare_parameter("in_topic", "/lidar/points")
        self.declare_parameter("out_topic", "/lidar/points2")
        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        self._pub = self.create_publisher(PointCloud2, out_topic, 1)
        self._sub = self.create_subscription(
            PointCloud, in_topic, self._on_cloud, 1
        )
        self.get_logger().info(
            f"pointcloud_bridge: {in_topic} (PointCloud) -> {out_topic} (PointCloud2)"
        )

    def _on_cloud(self, msg: PointCloud) -> None:
        self._pub.publish(_cloud_to_cloud2(msg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
