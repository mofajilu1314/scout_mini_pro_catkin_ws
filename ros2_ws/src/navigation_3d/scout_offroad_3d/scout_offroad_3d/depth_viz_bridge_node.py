#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert Gazebo 32FC1 depth (meters, with +inf) to 16UC1 mm for RViz Image.

RViz Image display often shows a black panel for 32FC1 + inf; 16UC1 works.
Does not change the original /camera/depth/image_raw used by the sim.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthVizBridge(Node):
    def __init__(self):
        super().__init__("depth_viz_bridge")
        self.declare_parameter("in_topic", "/camera/depth/image_raw")
        self.declare_parameter("out_topic", "/camera/depth/image_viz")
        self.declare_parameter("max_range_m", 6.0)
        in_topic = str(self.get_parameter("in_topic").value)
        out_topic = str(self.get_parameter("out_topic").value)
        self._max_m = float(self.get_parameter("max_range_m").value)
        self._pub = self.create_publisher(Image, out_topic, 1)
        self.create_subscription(Image, in_topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(
            "depth viz %s → %s (16UC1 mm, clip %.1fm)" % (in_topic, out_topic, self._max_m)
        )

    def _cb(self, msg: Image):
        if msg.encoding not in ("32FC1", "32FC"):
            self.get_logger().warn("unexpected encoding %s" % msg.encoding, throttle_duration_sec=5.0)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
        # +inf / nan (no hit) → 0 so RViz does not wash out normalize
        bad = ~np.isfinite(depth)
        depth[bad] = 0.0
        depth = np.clip(depth, 0.0, self._max_m)
        mm = (depth * 1000.0).astype(np.uint16)
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = msg.width * 2
        out.data = mm.tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = DepthVizBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
