#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish a synthetic polyline Path (stage-B stand-in for Hybrid A*)."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class FakePathNode(Node):
    def __init__(self):
        super().__init__("fake_path_node")
        self.declare_parameter("path_topic", "/long_nav/global_path")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("waypoints_xy", "0,0;10,0;20,5;30,5")
        self.declare_parameter("rate_hz", 1.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        raw = str(self.get_parameter("waypoints_xy").value)
        self._pts = []
        for tok in raw.split(";"):
            tok = tok.strip()
            if not tok:
                continue
            x_s, y_s = tok.split(",")
            self._pts.append((float(x_s), float(y_s)))
        if len(self._pts) < 2:
            self._pts = [(0.0, 0.0), (20.0, 0.0)]

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), qos
        )
        period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.1)
        self.create_timer(period, self._tick)
        self.get_logger().info("fake_path with %d waypoints" % len(self._pts))

    def _tick(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in self._pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakePathNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
