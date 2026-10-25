#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PoseStamped local_goal → geometry_msgs/Point for TD3 infer_td3_node."""

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class GoalBridgeNode(Node):
    def __init__(self):
        super().__init__("goal_bridge_node")
        self.declare_parameter("local_goal_topic", "/offroad/local_goal")
        self.declare_parameter("train_goal_topic", "/offroad/train_goal")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            Point, str(self.get_parameter("train_goal_topic").value), qos
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("local_goal_topic").value),
            self._on_goal,
            qos,
        )
        self.get_logger().info("goal_bridge PoseStamped → Point for TD3")

    def _on_goal(self, msg: PoseStamped):
        pt = Point()
        pt.x = float(msg.pose.position.x)
        pt.y = float(msg.pose.position.y)
        pt.z = 0.0
        self._pub.publish(pt)


def main(args=None):
    rclpy.init(args=args)
    node = GoalBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
