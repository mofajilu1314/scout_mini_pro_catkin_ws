#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Subscribe /long_nav/costmap; save latest grid on shutdown (Ctrl+C) or service."""

import os
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_srvs.srv import Trigger

from scout_long_nav.isolation import DEFAULT_MAP_DIR, DEFAULT_MAP_STEM, map_paths
from scout_long_nav.map_io import occupancy_to_arrays, save_trav_map


class SaveCostmapNode(Node):
    def __init__(self):
        super().__init__("save_costmap_node")
        self.declare_parameter("costmap_topic", "/long_nav/costmap")
        self.declare_parameter("map_dir", DEFAULT_MAP_DIR)
        self.declare_parameter("map_stem", DEFAULT_MAP_STEM)
        self.declare_parameter("save_on_shutdown", True)

        self._last = None  # type: Optional[OccupancyGrid]
        topic = str(self.get_parameter("costmap_topic").value)
        self.create_subscription(OccupancyGrid, topic, self._on_map, 1)
        self.create_service(Trigger, "~/save", self._on_save_srv)
        self.get_logger().info(
            "save_costmap: listening %s; Ctrl+C or `ros2 service call %s/save` to write"
            % (topic, self.get_fully_qualified_name())
        )

    def _on_map(self, msg: OccupancyGrid):
        self._last = msg

    def _write(self) -> Tuple[bool, str]:
        if self._last is None:
            return False, "no OccupancyGrid received yet"
        msg = self._last
        grid = occupancy_to_arrays(msg.info.width, msg.info.height, msg.data)
        yaml_path, _ = map_paths(
            str(self.get_parameter("map_dir").value),
            str(self.get_parameter("map_stem").value),
        )
        y, p = save_trav_map(
            yaml_path,
            grid,
            float(msg.info.resolution),
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            frame_id=msg.header.frame_id or "odom",
        )
        return True, "saved %s and %s (grid %dx%d)" % (y, p, msg.info.width, msg.info.height)

    def _on_save_srv(self, _req, resp):
        ok, msg = self._write()
        resp.success = ok
        resp.message = msg
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().warn(msg)
        return resp

    def save_if_needed(self):
        if not bool(self.get_parameter("save_on_shutdown").value):
            return
        ok, msg = self._write()
        if ok:
            self.get_logger().info("[shutdown] " + msg)
        else:
            self.get_logger().warn("[shutdown] " + msg)


def main(args=None):
    rclpy.init(args=args)
    node = SaveCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_if_needed()
        except Exception as e:
            node.get_logger().error("save on shutdown failed: %s" % e)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
