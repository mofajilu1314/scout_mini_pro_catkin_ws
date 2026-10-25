#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish a static traversability cost OccupancyGrid from maps_long yaml+pgm."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from scout_long_nav.cost_codes import grid_to_occ_int8
from scout_long_nav.isolation import DEFAULT_MAP_DIR, DEFAULT_MAP_STEM, map_paths
from scout_long_nav.map_io import load_trav_map


class StaticCostmapNode(Node):
    def __init__(self):
        super().__init__("static_costmap_node")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_dir", DEFAULT_MAP_DIR)
        self.declare_parameter("map_stem", DEFAULT_MAP_STEM)
        self.declare_parameter("costmap_topic", "/long_nav/costmap")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("frame_id", "")

        yaml_path = str(self.get_parameter("map_yaml").value).strip()
        if not yaml_path:
            yaml_path, _ = map_paths(
                str(self.get_parameter("map_dir").value),
                str(self.get_parameter("map_stem").value),
            )
        loaded = load_trav_map(yaml_path)
        self._loaded = loaded
        frame = str(self.get_parameter("frame_id").value).strip() or loaded["frame_id"]

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topic = str(self.get_parameter("costmap_topic").value)
        self._pub = self.create_publisher(OccupancyGrid, topic, qos)
        self._frame = frame
        period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.1)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "static_costmap: %s (%dx%d res=%.3f) → %s"
            % (
                yaml_path,
                loaded["width"],
                loaded["height"],
                loaded["resolution"],
                topic,
            )
        )
        self._tick()

    def _tick(self):
        L = self._loaded
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._frame
        grid.info.resolution = float(L["resolution"])
        grid.info.width = int(L["width"])
        grid.info.height = int(L["height"])
        grid.info.origin.position.x = float(L["origin_x"])
        grid.info.origin.position.y = float(L["origin_y"])
        grid.info.origin.orientation.w = 1.0
        flat = L["grid"].reshape(-1)
        grid.data = grid_to_occ_int8(L["grid"])
        self._pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = StaticCostmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
