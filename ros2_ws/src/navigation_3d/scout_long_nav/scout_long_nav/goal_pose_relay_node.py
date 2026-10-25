#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Relay RViz 2D Goal Pose (/goal_pose) → /long_nav/goal (PoseStamped)."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalPoseRelayNode(Node):
    def __init__(self):
        super().__init__("goal_pose_relay_node")
        self.declare_parameter("input_topic", "/goal_pose")
        self.declare_parameter("output_topic", "/long_nav/goal")
        self.declare_parameter("force_frame_id", "odom")
        inn = str(self.get_parameter("input_topic").value)
        out = str(self.get_parameter("output_topic").value)
        self._force = str(self.get_parameter("force_frame_id").value)
        self._pub = self.create_publisher(PoseStamped, out, 10)
        self.create_subscription(PoseStamped, inn, self._on_goal, 10)
        self.get_logger().info("goal_pose_relay: %s → %s" % (inn, out))

    def _on_goal(self, msg: PoseStamped):
        out = PoseStamped()
        out.header = msg.header
        if self._force:
            out.header.frame_id = self._force
        out.pose = msg.pose
        self._pub.publish(out)
        self.get_logger().info(
            "goal relayed (%.2f, %.2f)" % (out.pose.position.x, out.pose.position.y)
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
