#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PointCloud2 (+ odom) → /long_nav/costmap OccupancyGrid.

Default: latest-scan only (online local window).
Mapping: accumulate:=true + fixed_center:=true → persistent odom-frame map
that grows as you drive (old cells keep last known cost).
"""

import struct

from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from scout_long_nav.costmap_geom import CostmapParams, points_to_costmap
from scout_long_nav.map_io import load_trav_map


def _read_xyz_from_cloud(msg: PointCloud2) -> np.ndarray:
    """Minimal XYZ extractor (float32 x,y,z fields)."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float64)
    fx, fy, fz = field_map["x"], field_map["y"], field_map["z"]
    pts = []
    point_step = msg.point_step
    data = bytes(msg.data)
    n = msg.width * msg.height
    for i in range(n):
        base = i * point_step
        x = struct.unpack_from("<f", data, base + fx.offset)[0]
        y = struct.unpack_from("<f", data, base + fy.offset)[0]
        z = struct.unpack_from("<f", data, base + fz.offset)[0]
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            pts.append((x, y, z))
    if not pts:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


def _quat_rotate_translate(pts: np.ndarray, qx, qy, qz, qw, tx, ty, tz) -> np.ndarray:
    """Apply quaternion rotation + translation to Nx3 points."""
    if pts.size == 0:
        return pts
    x, y, z, w = qx, qy, qz, qw
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    out = (R @ pts.T).T
    out[:, 0] += tx
    out[:, 1] += ty
    out[:, 2] += tz
    return out


class CloudToCostmapNode(Node):
    def __init__(self):
        super().__init__("cloud_to_costmap_node")
        self.declare_parameter("cloud_topic", "/lidar/points2")
        self.declare_parameter("odom_topic", "/long_nav/odom")
        self.declare_parameter("costmap_topic", "/long_nav/costmap")
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("size_m", 40.0)
        self.declare_parameter("slope_max", 0.45)
        self.declare_parameter("alpha_slope", 80.0)
        self.declare_parameter("beta_rough", 40.0)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("rate_hz", 2.0)
        # Mapping: fix window at (0,0) covering ~[-size/2, size/2]
        self.declare_parameter("fixed_center", False)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        # Persist cells across scans (建图要开；在线导航窗可关)
        self.declare_parameter("accumulate", False)
        # accumulate 合并：max=障碍更易留下；last=本帧覆盖（易抖）
        self.declare_parameter("accum_merge", "max")
        # 续建：启动时从已存 yaml+pgm 灌入累积图（继续扫会 merge）
        self.declare_parameter("seed_map_yaml", "")

        self._cloud_pts = None
        self._cloud_frame = ""
        self._center = (0.0, 0.0)
        self._map_frame = str(self.get_parameter("map_frame").value) or "odom"
        self._fixed_center = bool(self.get_parameter("fixed_center").value)
        self._accumulate = bool(self.get_parameter("accumulate").value)
        self._accum_merge = str(self.get_parameter("accum_merge").value).lower()
        if self._fixed_center:
            self._center = (
                float(self.get_parameter("center_x").value),
                float(self.get_parameter("center_y").value),
            )
        self.params = CostmapParams(
            resolution=float(self.get_parameter("resolution").value),
            size_m=float(self.get_parameter("size_m").value),
            slope_max=float(self.get_parameter("slope_max").value),
            alpha_slope=float(self.get_parameter("alpha_slope").value),
            beta_rough=float(self.get_parameter("beta_rough").value),
            min_points_per_cell=int(self.get_parameter("min_points_per_cell").value),
        )
        # Persistent grid in map_frame (row-major flat int16, -1 unknown)
        self._acc = None  # type: Optional[np.ndarray]
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._width = 0
        self._height = 0
        self._last_tf_warn = 0.0
        self._paused = False  # flip recovery: ignore lidar into costmap
        self._resolution = float(self.get_parameter("resolution").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("costmap_topic").value), qos
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("cloud_topic").value), self._on_cloud, qos_sensor
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, qos_sensor
        )
        self.create_subscription(Bool, "/long_nav/mapping_pause", self._on_pause, 10)
        self.create_subscription(
            Float32MultiArray, "/long_nav/clear_costmap_radius", self._on_clear_radius_msg, 10
        )
        self.create_service(Trigger, "~/clear_all", self._on_clear_all)
        period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.2)
        self.create_timer(period, self._tick)

        seed = str(self.get_parameter("seed_map_yaml").value).strip()
        if seed:
            self._load_seed(seed)

        self.get_logger().info(
            "cloud_to_costmap ready (accum=%s fixed_center=%s map_frame=%s) "
            "| pause=/long_nav/mapping_pause clear=/long_nav/clear_costmap_radius"
            % (self._accumulate, self._fixed_center, self._map_frame)
        )

    def _load_seed(self, path_yaml: str) -> None:
        """Load saved arena_trav into accum so remapping continues from disk."""
        try:
            loaded = load_trav_map(path_yaml)
        except Exception as exc:  # noqa: BLE001 — surface path/IO errors in log
            self.get_logger().error("seed_map load FAILED %s: %s" % (path_yaml, exc))
            return
        grid = np.asarray(loaded["grid"], dtype=np.int16)
        h, w = int(grid.shape[0]), int(grid.shape[1])
        # Lock to launch geometry so odom frame / steering frame stay aligned
        res = float(self.get_parameter("resolution").value)
        size_m = float(self.get_parameter("size_m").value)
        expect_n = int(round(size_m / res))
        if w != expect_n or h != expect_n:
            self.get_logger().error(
                "seed_map size mismatch: file %dx%d vs launch ~%dx%d — NOT loaded"
                % (w, h, expect_n, expect_n)
            )
            return
        half = 0.5 * size_m
        ox = float(self.get_parameter("center_x").value) - half
        oy = float(self.get_parameter("center_y").value) - half
        self._resolution = res
        self.params.resolution = res
        self.params.size_m = size_m
        self._width = w
        self._height = h
        self._origin_x = ox
        self._origin_y = oy
        self._acc = grid.reshape(-1).copy()
        known = int(np.sum(self._acc >= 0))
        self.get_logger().info(
            "seed_map loaded %s → %dx%d res=%.4f origin=(%.2f,%.2f) known=%d"
            % (path_yaml, w, h, res, ox, oy, known)
        )

    def _on_pause(self, msg: Bool):
        self._paused = bool(msg.data)
        self.get_logger().info("mapping_pause=%s (lidar→costmap %s)" % (
            self._paused, "OFF" if self._paused else "ON"
        ))
        if self._paused:
            self._cloud_pts = np.zeros((0, 3), dtype=np.float64)

    def _on_clear_radius_msg(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn("clear_costmap_radius needs [x,y,r]")
            return
        n = self._clear_radius(float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
        self.get_logger().info(
            "cleared %d cells around (%.1f,%.1f) r=%.1f"
            % (n, msg.data[0], msg.data[1], msg.data[2])
        )

    def _on_clear_all(self, _req, resp):
        if self._acc is not None:
            self._acc.fill(-1)
            resp.success = True
            resp.message = "accum cleared to unknown"
        else:
            resp.success = False
            resp.message = "no accum yet"
        return resp

    def _clear_radius(self, x: float, y: float, radius: float) -> int:
        if self._acc is None or self._width <= 0 or radius <= 0.0:
            return 0
        res = self._resolution if self._resolution > 1e-6 else 0.2
        r2 = radius * radius
        ix0 = max(0, int((x - radius - self._origin_x) / res) - 1)
        ix1 = min(self._width - 1, int((x + radius - self._origin_x) / res) + 1)
        iy0 = max(0, int((y - radius - self._origin_y) / res) - 1)
        iy1 = min(self._height - 1, int((y + radius - self._origin_y) / res) + 1)
        n = 0
        for iy in range(iy0, iy1 + 1):
            for ix in range(ix0, ix1 + 1):
                wx = self._origin_x + (ix + 0.5) * res
                wy = self._origin_y + (iy + 0.5) * res
                if (wx - x) * (wx - x) + (wy - y) * (wy - y) <= r2:
                    idx = iy * self._width + ix
                    if self._acc[idx] >= 0:
                        self._acc[idx] = -1
                        n += 1
        return n

    def _on_cloud(self, msg: PointCloud2):
        if self._paused:
            return
        self._cloud_pts = _read_xyz_from_cloud(msg)
        self._cloud_frame = msg.header.frame_id or ""

    def _on_odom(self, msg: Odometry):
        if self._fixed_center:
            return
        self._center = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _points_in_map_frame(self, pts: np.ndarray) -> np.ndarray:
        """Transform sensor-frame points into map_frame (odom)."""
        if pts.size == 0:
            return pts
        src = self._cloud_frame or self._map_frame
        if src == self._map_frame or src in ("", self._map_frame):
            return pts
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                src,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as e:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_tf_warn > 2.0:
                self.get_logger().warn(
                    "TF %s→%s failed: %s (skip frame)" % (src, self._map_frame, e)
                )
                self._last_tf_warn = now
            return np.zeros((0, 3), dtype=np.float64)
        t = tf.transform.translation
        q = tf.transform.rotation
        return _quat_rotate_translate(pts, q.x, q.y, q.z, q.w, t.x, t.y, t.z)

    def _ensure_acc(self, width: int, height: int, origin_x: float, origin_y: float):
        if self._acc is not None and self._width == width and self._height == height:
            # Keep seed/continue map; ignore tiny float origin drift
            if abs(self._origin_x - origin_x) > 0.05 or abs(self._origin_y - origin_y) > 0.05:
                self.get_logger().warn(
                    "costmap origin drift (%.3f,%.3f)->(%.3f,%.3f); keeping accum"
                    % (self._origin_x, self._origin_y, origin_x, origin_y)
                )
            return
        self._width = width
        self._height = height
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._acc = np.full(width * height, -1, dtype=np.int16)

    def _merge_into_acc(self, new_flat: np.ndarray):
        """Merge this-scan costs into persistent map.

        max: keep higher cost so trees/walls don't vanish on a later thin scan;
            with higher min_points_per_cell this resists single-ray jitter.
        last: overwrite with latest observation.
        """
        assert self._acc is not None
        known = new_flat >= 0
        if self._accum_merge == "last":
            self._acc[known] = new_flat[known].astype(np.int16)
            return
        # default max
        both = known & (self._acc >= 0)
        only_new = known & (self._acc < 0)
        self._acc[only_new] = new_flat[only_new].astype(np.int16)
        if both.any():
            self._acc[both] = np.maximum(
                self._acc[both], new_flat[both].astype(np.int16)
            )

    def _tick(self):
        # While paused (rollover recovery): keep publishing last accum, no lidar merge
        if self._paused:
            if self._acc is not None and self._width > 0:
                self._publish_grid(self._acc, self._origin_x, self._origin_y, self._width, self._height, self._resolution)
            return

        pts = self._cloud_pts
        if pts is None:
            pts = np.zeros((0, 3), dtype=np.float64)
        else:
            pts = self._points_in_map_frame(pts)

        res = points_to_costmap(pts, self._center[0], self._center[1], self.params)
        data = np.asarray(res.data, dtype=np.int16)
        self._resolution = float(res.resolution)

        if self._accumulate and self._fixed_center:
            self._ensure_acc(res.width, res.height, res.origin_x, res.origin_y)
            self._merge_into_acc(data)
            out = self._acc
            origin_x, origin_y = self._origin_x, self._origin_y
            w, h = self._width, self._height
            resolution = res.resolution
        else:
            out = data
            origin_x, origin_y = res.origin_x, res.origin_y
            w, h = res.width, res.height
            resolution = res.resolution

        self._publish_grid(out, origin_x, origin_y, w, h, resolution)

    def _publish_grid(self, out, origin_x, origin_y, w, h, resolution):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._map_frame
        grid.info.resolution = float(resolution)
        grid.info.width = int(w)
        grid.info.height = int(h)
        grid.info.origin.position.x = float(origin_x)
        grid.info.origin.position.y = float(origin_y)
        grid.info.origin.orientation.w = 1.0
        flat = np.asarray(out, dtype=np.int16).reshape(-1)
        grid.data = flat.astype(np.int8).tolist()
        self._pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToCostmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
