#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline / idle-window pose stub: republish /odom → /long_nav/odom.

Does NOT talk to Gazebo. Safe to use only when you intentionally bridge an
existing odom source on an isolated ROS_DOMAIN_ID.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PoseStubNode(Node):
    def __init__(self):
        super().__init__("pose_stub_node")
        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter("output_odom_topic", "/long_nav/odom")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_odom_topic").value), qos
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_odom_topic").value),
            self._on_odom,
            qos,
        )
        self.get_logger().info("pose_stub: remap odom → /long_nav/odom (no Gazebo)")

    def _on_odom(self, msg: Odometry):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseStubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
