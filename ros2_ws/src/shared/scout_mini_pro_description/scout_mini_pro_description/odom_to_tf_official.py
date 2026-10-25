#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish odom → base_link TF from /odom, skipping non-increasing stamps.

Official Scout Mini skid_steer uses publish_odom_tf=false; this node is the
single TF authority for odom→base_link (avoids TF_REPEATED_DATA).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class OdomToTfOfficial(Node):
    def __init__(self):
        super().__init__("odom_to_tf_official")
        self.declare_parameter("odom_topic", "/odom")
        odom_topic = str(self.get_parameter("odom_topic").value)
        # Compare nanoseconds only: Time() defaults to SYSTEM_TIME while
        # Time.from_msg is ROS_TIME — mixing them raises TypeError under sim time.
        self._last_stamp_ns = 0
        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._cb, 1)
        self.get_logger().info(f"odom→base_link TF from {odom_topic}")

    def _cb(self, msg: Odometry):
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        # World switch restarts gzserver and resets /clock; allow stamp to jump back.
        if stamp_ns < self._last_stamp_ns:
            self._last_stamp_ns = 0
        if stamp_ns <= self._last_stamp_ns:
            return
        self._last_stamp_ns = stamp_ns

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTfOfficial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
