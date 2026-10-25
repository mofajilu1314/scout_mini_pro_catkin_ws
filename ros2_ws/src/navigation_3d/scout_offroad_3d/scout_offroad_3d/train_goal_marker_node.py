#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Republish /offroad/train_goal as an RViz Marker (frame=odom)."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker


class TrainGoalMarker(Node):
    def __init__(self):
        super().__init__("train_goal_marker")
        self.declare_parameter("goal_topic", "/offroad/train_goal")
        self.declare_parameter("marker_topic", "/offroad/train_goal_marker")
        self.declare_parameter("frame_id", "odom")
        goal_topic = str(self.get_parameter("goal_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self._frame = str(self.get_parameter("frame_id").value)
        self._pub = self.create_publisher(Marker, marker_topic, 1)
        self.create_subscription(Point, goal_topic, self._on_goal, 1)
        self.get_logger().info(
            "goal %s → marker %s (frame=%s)" % (goal_topic, marker_topic, self._frame)
        )

    def _on_goal(self, msg: Point):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self._frame
        m.ns = "train_goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(msg.x)
        m.pose.position.y = float(msg.y)
        m.pose.position.z = float(msg.z) if abs(float(msg.z)) > 1e-6 else 0.35
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.6
        m.color.r = 0.1
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 0.95
        m.lifetime.sec = 0
        self._pub.publish(m)

        # Thin vertical stick so it stays visible in top-down view
        stick = Marker()
        stick.header = m.header
        stick.ns = "train_goal"
        stick.id = 1
        stick.type = Marker.CYLINDER
        stick.action = Marker.ADD
        stick.pose.position.x = m.pose.position.x
        stick.pose.position.y = m.pose.position.y
        stick.pose.position.z = 0.6
        stick.pose.orientation.w = 1.0
        stick.scale.x = stick.scale.y = 0.12
        stick.scale.z = 1.2
        stick.color.r = 0.05
        stick.color.g = 0.75
        stick.color.b = 0.15
        stick.color.a = 0.8
        self._pub.publish(stick)


def main():
    rclpy.init()
    node = TrainGoalMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
