# -*- coding: utf-8 -*-
"""OccupancyGrid → colored PointCloud2 for a fresher RViz heatmap (viz only).

Does not change /long_nav/costmap values used by planners.
Palette:
  0..100 terrain → cyan/mint → soft sunset orange (100 = orange, mapping max)
  security (≥101 / 127 / 1000) → pure bright red
Unknown (-1) skipped in the heat cloud (transparent); click via Map/click-plane.
"""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from scout_long_nav.cost_codes import OCC_SECURITY, SECURITY_COST, TERRAIN_MAX


def _costs_to_rgb_float(costs: np.ndarray) -> np.ndarray:
    """costs: terrain 0..100 + security carriers → float32 RGB packed."""
    c = costs.astype(np.float64)
    security = (c >= 101.0) | (c >= float(SECURITY_COST)) | (c == float(OCC_SECURITY))
    terrain = np.clip(c, 0.0, float(TERRAIN_MAX))
    t = terrain / float(TERRAIN_MAX)
    r = np.empty_like(t)
    g = np.empty_like(t)
    b = np.empty_like(t)
    lo = t < 0.5
    hi = ~lo
    u = np.zeros_like(t)
    u[lo] = t[lo] / 0.5
    u[hi] = (t[hi] - 0.5) / 0.5
    # cyan → mint (low cost)
    r[lo] = 120 + (90 - 120) * u[lo]
    g[lo] = 200 + (190 - 200) * u[lo]
    b[lo] = 230 + (140 - 230) * u[lo]
    # mint → soft sunset orange (terrain high / 100)
    r[hi] = 90 + (220 - 90) * u[hi]
    g[hi] = 190 + (130 - 190) * u[hi]
    b[hi] = 140 + (55 - 140) * u[hi]
    # security fence only → pure red
    r[security] = 255.0
    g[security] = 28.0
    b[security] = 28.0
    rgb_u = (
        (r.astype(np.uint32) << 16)
        | (g.astype(np.uint32) << 8)
        | b.astype(np.uint32)
    )
    return np.ascontiguousarray(rgb_u).view(np.float32)


class CostmapColorCloud(Node):
    def __init__(self) -> None:
        super().__init__("costmap_color_cloud_node")
        self.declare_parameter("costmap_topic", "/long_nav/costmap")
        self.declare_parameter("cloud_topic", "/long_nav/costmap_rgb")
        self.declare_parameter("z", 0.05)
        self.declare_parameter("stride", 1)
        # Skip near-free solid sheet: show free sparsely, always show cost>=min_cost
        self.declare_parameter("min_cost_viz", 1)
        self.declare_parameter("free_stride", 4)  # cost==0 keep every Nth cell

        in_topic = str(self.get_parameter("costmap_topic").value)
        out_topic = str(self.get_parameter("cloud_topic").value)
        self._z = float(self.get_parameter("z").value)
        self._stride = max(1, int(self.get_parameter("stride").value))
        self._min_cost = int(self.get_parameter("min_cost_viz").value)
        self._free_stride = max(1, int(self.get_parameter("free_stride").value))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.create_subscription(OccupancyGrid, in_topic, self._on_map, qos)
        self.get_logger().info(
            f"costmap_color_cloud: {in_topic} → {out_topic} (viz only)"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._publish_cloud(msg)
        except Exception as exc:
            self.get_logger().warn("costmap_color_cloud skip frame: %s" % exc)

    def _publish_cloud(self, msg: OccupancyGrid) -> None:
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        stride = self._stride
        grid = data[::stride, ::stride]
        # Interesting costs always; free (0) only every free_stride to avoid solid sheet
        interesting = grid >= max(self._min_cost, 1)
        free = grid == 0
        yy, xx = np.indices(grid.shape)
        free_keep = free & ((yy + xx) % self._free_stride == 0)
        iy, ix = np.nonzero(interesting | free_keep)
        if iy.size == 0:
            cloud = PointCloud2()
            cloud.header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id or "odom")
            cloud.height = 1
            cloud.width = 0
            cloud.is_dense = True
            cloud.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            cloud.point_step = 16
            cloud.row_step = 0
            cloud.data = b""
            self._pub.publish(cloud)
            return

        raw = grid[iy, ix].astype(np.int16)
        # keep security carrier 127; do not clip to 100
        costs = raw.copy()
        costs[(raw >= 0) & (raw <= 100)] = raw[(raw >= 0) & (raw <= 100)]
        xs = ox + (ix.astype(np.float64) * stride + 0.5) * res
        ys = oy + (iy.astype(np.float64) * stride + 0.5) * res
        zs = np.full_like(xs, self._z)
        rgbs = _costs_to_rgb_float(costs)

        n = xs.size
        # Cap size so RViz stays responsive
        max_n = 80000
        if n > max_n:
            sel = np.linspace(0, n - 1, max_n, dtype=np.int64)
            xs, ys, zs, rgbs = xs[sel], ys[sel], zs[sel], rgbs[sel]
            n = max_n

        arr = np.empty((n, 4), dtype=np.float32)
        arr[:, 0] = xs
        arr[:, 1] = ys
        arr[:, 2] = zs
        arr[:, 3] = rgbs

        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = msg.header.frame_id or "odom"
        cloud.height = 1
        cloud.width = n
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 16
        cloud.row_step = 16 * n
        cloud.data = arr.tobytes()
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = CostmapColorCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
