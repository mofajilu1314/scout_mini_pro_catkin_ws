#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Range sector features for off-road / TD3 local control (ROS2 rclpy).

Sources:
  source:=scan   — LaserScan (/scan)
  source:=cloud  — PointCloud2 or Gazebo block_laser PointCloud (16-line lidar)

Outputs:
  /offroad/td3_range_features  — n_elevation_bins_td3 × N sectors (default 4×36=144)
  /offroad/range_features      — N horizontal sectors (default 36)
  /offroad/elevation_features  — n_elevation_bins × n_front_sectors (default 4×12=48)
"""

from __future__ import annotations

import math
import random
import struct
from typing import Iterable, Iterator, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def _parse_float_list(raw, default: List[float]) -> List[float]:
    if raw is None:
        return list(default)
    if isinstance(raw, (list, tuple)):
        return [float(x) for x in raw]
    text = str(raw).replace(",", " ").replace("[", " ").replace("]", " ")
    parts = text.split()
    if not parts:
        return list(default)
    return [float(x) for x in parts]


def _bin_index(elev_deg: float, edges: List[float]) -> Optional[int]:
    """Map elevation into bins defined by `edges`.

    Interior bins are half-open [lo, hi); the last bin is closed [lo, hi]
    so the FOV endpoint (e.g. +15°) is not dropped.
    """
    n = len(edges) - 1
    if n < 1:
        return None
    last = n - 1
    for i in range(n):
        lo = edges[i]
        hi = edges[i + 1]
        if i == last:
            if lo <= elev_deg <= hi:
                return i
        elif lo <= elev_deg < hi:
            return i
    return None


def _field_offsets(cloud: PointCloud2) -> Tuple[int, int, int]:
    offsets = {}
    for f in cloud.fields:
        if f.name in ("x", "y", "z"):
            offsets[f.name] = f.offset
    if len(offsets) < 3:
        return 0, 4, 8
    return offsets["x"], offsets["y"], offsets["z"]


def _iter_xyz_cloud2(cloud: PointCloud2) -> Iterator[Tuple[float, float, float]]:
    ox, oy, oz = _field_offsets(cloud)
    data = cloud.data
    step = cloud.point_step
    n = cloud.width * cloud.height
    for i in range(n):
        base = i * step
        try:
            x = struct.unpack_from("<f", data, base + ox)[0]
            y = struct.unpack_from("<f", data, base + oy)[0]
            z = struct.unpack_from("<f", data, base + oz)[0]
        except struct.error:
            continue
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            continue
        if math.isinf(x) or math.isinf(y) or math.isinf(z):
            continue
        yield x, y, z


def _iter_xyz_pointcloud(cloud: PointCloud) -> Iterator[Tuple[float, float, float]]:
    for p in cloud.points:
        x, y, z = p.x, p.y, p.z
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            continue
        if math.isinf(x) or math.isinf(y) or math.isinf(z):
            continue
        if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            continue
        yield x, y, z


class RangeFeaturesNode(Node):
    def __init__(self) -> None:
        super().__init__("range_features_node")

        self.declare_parameter("n_sectors", 36)
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("min_range", 0.18)
        self.declare_parameter("elevation_min_deg", -10.0)
        self.declare_parameter("elevation_max_deg", 10.0)
        self.declare_parameter("publish_elevation_features", True)
        self.declare_parameter("n_elevation_bins", 4)
        self.declare_parameter("n_front_sectors", 12)
        self.declare_parameter("front_half_fov_deg", 60.0)
        default_edges = [-15.0, -7.5, 0.0, 7.5, 15.0]
        self.declare_parameter("elevation_bin_edges_deg", default_edges)
        self.declare_parameter("publish_td3_range_features", True)
        self.declare_parameter("n_elevation_bins_td3", 4)
        default_td3_edges = [-15.0, -7.5, 0.0, 7.5, 15.0]
        self.declare_parameter("td3_elevation_bin_edges_deg", default_td3_edges)
        self.declare_parameter("y_abs_max", 0.6)
        self.declare_parameter("frame_mode", "base")
        self.declare_parameter("source", "cloud")
        self.declare_parameter("cloud_msg_type", "pointcloud")
        self.declare_parameter("full_circle", True)
        self.declare_parameter("use_angle_window", False)
        self.declare_parameter("filter_body_box", True)
        self.declare_parameter("body_x_min", -0.36)
        self.declare_parameter("body_x_max", 0.36)
        self.declare_parameter("body_y_min", -0.32)
        self.declare_parameter("body_y_max", 0.32)
        self.declare_parameter("body_z_min", -0.42)
        self.declare_parameter("body_z_max", 0.06)
        self.declare_parameter("body_filter_max_range", 1.2)
        self.declare_parameter("sensor_noise_std", 0.0)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("fov_deg", 87.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cloud_topic", "/lidar/points")
        self.declare_parameter("features_topic", "/offroad/range_features")
        self.declare_parameter("elevation_features_topic", "/offroad/elevation_features")
        self.declare_parameter("td3_features_topic", "/offroad/td3_range_features")

        self.n_sectors = int(self.get_parameter("n_sectors").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.elevation_min_deg = float(self.get_parameter("elevation_min_deg").value)
        self.elevation_max_deg = float(self.get_parameter("elevation_max_deg").value)
        self.publish_elevation_features = bool(
            self.get_parameter("publish_elevation_features").value
        )
        self.n_elevation_bins = int(self.get_parameter("n_elevation_bins").value)
        self.n_front_sectors = int(self.get_parameter("n_front_sectors").value)
        self.front_half_fov_deg = float(self.get_parameter("front_half_fov_deg").value)
        self._elevation_bin_edges_deg = _parse_float_list(
            self.get_parameter("elevation_bin_edges_deg").value, default_edges
        )
        if len(self._elevation_bin_edges_deg) != self.n_elevation_bins + 1:
            self.get_logger().warn(
                f"elevation_bin_edges_deg length {len(self._elevation_bin_edges_deg)} "
                f"!= n_elevation_bins+1 ({self.n_elevation_bins + 1}); rebuilding"
            )
            self._elevation_bin_edges_deg = default_edges[: self.n_elevation_bins + 1]
        self.publish_td3_range_features = bool(
            self.get_parameter("publish_td3_range_features").value
        )
        self.n_elevation_bins_td3 = int(self.get_parameter("n_elevation_bins_td3").value)
        self._td3_elevation_bin_edges_deg = _parse_float_list(
            self.get_parameter("td3_elevation_bin_edges_deg").value, default_td3_edges
        )
        if len(self._td3_elevation_bin_edges_deg) != self.n_elevation_bins_td3 + 1:
            self.get_logger().warn(
                f"td3_elevation_bin_edges_deg length {len(self._td3_elevation_bin_edges_deg)} "
                f"!= n_elevation_bins_td3+1 ({self.n_elevation_bins_td3 + 1}); rebuilding"
            )
            self._td3_elevation_bin_edges_deg = default_td3_edges[
                : self.n_elevation_bins_td3 + 1
            ]
        self.y_abs_max = float(self.get_parameter("y_abs_max").value)
        self.frame_mode = str(self.get_parameter("frame_mode").value).strip().lower()
        self.source = str(self.get_parameter("source").value).strip().lower()
        self.cloud_msg_type = str(self.get_parameter("cloud_msg_type").value).strip().lower()
        self.full_circle = bool(self.get_parameter("full_circle").value)
        use_angle_window = bool(self.get_parameter("use_angle_window").value)
        self.filter_body_box = bool(self.get_parameter("filter_body_box").value)
        self.body_x_min = float(self.get_parameter("body_x_min").value)
        self.body_x_max = float(self.get_parameter("body_x_max").value)
        self.body_y_min = float(self.get_parameter("body_y_min").value)
        self.body_y_max = float(self.get_parameter("body_y_max").value)
        self.body_z_min = float(self.get_parameter("body_z_min").value)
        self.body_z_max = float(self.get_parameter("body_z_max").value)
        self.body_filter_max_range = float(
            self.get_parameter("body_filter_max_range").value
        )
        self.sensor_noise_std = float(self.get_parameter("sensor_noise_std").value)

        if self.full_circle:
            self.angle_min = float(self.get_parameter("angle_min").value)
            self.angle_max = float(self.get_parameter("angle_max").value)
        elif use_angle_window:
            self.angle_min = float(self.get_parameter("angle_min").value)
            self.angle_max = float(self.get_parameter("angle_max").value)
        else:
            fov_deg = float(self.get_parameter("fov_deg").value)
            half = math.radians(fov_deg) * 0.5
            self.angle_min = -half
            self.angle_max = half

        if self.angle_max <= self.angle_min:
            raise ValueError("angle_max must be > angle_min")

        self._span = self.angle_max - self.angle_min
        self._elevation_min = math.radians(self.elevation_min_deg)
        self._elevation_max = math.radians(self.elevation_max_deg)
        self._front_half_fov = math.radians(self.front_half_fov_deg)
        self._front_span = 2.0 * self._front_half_fov

        features_topic = self.get_parameter("features_topic").value
        elev_topic = self.get_parameter("elevation_features_topic").value
        td3_topic = self.get_parameter("td3_features_topic").value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Float32MultiArray, features_topic, qos)
        self._pub_elev = None
        if self.publish_elevation_features:
            self._pub_elev = self.create_publisher(Float32MultiArray, elev_topic, qos)
        self._pub_td3 = None
        if self.publish_td3_range_features:
            self._pub_td3 = self.create_publisher(Float32MultiArray, td3_topic, qos)

        if self.source == "scan":
            scan_topic = self.get_parameter("scan_topic").value
            self._sub = self.create_subscription(
                LaserScan, scan_topic, self._on_scan, qos
            )
            self.get_logger().info(
                f"range_features: source=scan sub={scan_topic} pub={features_topic} "
                f"N={self.n_sectors} max={self.max_range:.1f}"
            )
        elif self.source == "cloud":
            cloud_topic = self.get_parameter("cloud_topic").value
            if self.cloud_msg_type == "pointcloud2":
                self._sub = self.create_subscription(
                    PointCloud2, cloud_topic, self._on_cloud2, qos
                )
            else:
                self._sub = self.create_subscription(
                    PointCloud, cloud_topic, self._on_pointcloud, qos
                )
            self.get_logger().info(
                f"range_features: source=cloud sub={cloud_topic} ({self.cloud_msg_type}) "
                f"pub={features_topic} N={self.n_sectors}"
            )
        else:
            raise ValueError("source must be 'scan' or 'cloud', got %r" % self.source)

    def _sector_index(self, ang: float) -> Optional[int]:
        if ang < self.angle_min or ang > self.angle_max:
            return None
        t = (self.angle_max - ang) / self._span
        idx = int(t * self.n_sectors)
        if idx < 0:
            idx = 0
        elif idx >= self.n_sectors:
            idx = self.n_sectors - 1
        return idx

    def _front_sector_index(self, ang: float) -> Optional[int]:
        if abs(ang) > self._front_half_fov + 1e-9:
            return None
        t = (self._front_half_fov - ang) / self._front_span
        idx = int(t * self.n_front_sectors)
        if idx < 0:
            idx = 0
        elif idx >= self.n_front_sectors:
            idx = self.n_front_sectors - 1
        return idx

    def _elevation_bin_index(self, elev_deg: float) -> Optional[int]:
        return _bin_index(elev_deg, self._elevation_bin_edges_deg)

    def _td3_elevation_bin_index(self, elev_deg: float) -> Optional[int]:
        return _bin_index(elev_deg, self._td3_elevation_bin_edges_deg)

    def _apply_sensor_noise(self, ranges: List[float]) -> List[float]:
        if self.sensor_noise_std <= 0.0:
            return ranges
        out = []
        for r in ranges:
            noisy = float(r) + random.gauss(0.0, self.sensor_noise_std)
            noisy = max(self.min_range, min(self.max_range, noisy))
            out.append(noisy)
        return out

    def _publish_ranges(self, ranges: List[float]) -> None:
        ranges = self._apply_sensor_noise(ranges)
        out = Float32MultiArray()
        out.layout.dim = [
            MultiArrayDimension(label="sectors", size=self.n_sectors, stride=self.n_sectors)
        ]
        out.layout.data_offset = 0
        out.data = ranges
        self._pub.publish(out)

    def _publish_elevation(self, elev_ranges: List[float]) -> None:
        if self._pub_elev is None:
            return
        elev_ranges = self._apply_sensor_noise(elev_ranges)
        n_bins = self.n_elevation_bins
        n_front = self.n_front_sectors
        out = Float32MultiArray()
        out.layout.dim = [
            MultiArrayDimension(
                label="elevation_bin", size=n_bins, stride=n_bins * n_front
            ),
            MultiArrayDimension(label="front_sector", size=n_front, stride=n_front),
        ]
        out.layout.data_offset = 0
        out.data = elev_ranges
        self._pub_elev.publish(out)

    def _publish_td3_ranges(self, td3_ranges: List[float]) -> None:
        if self._pub_td3 is None:
            return
        td3_ranges = self._apply_sensor_noise(td3_ranges)
        n_bins = self.n_elevation_bins_td3
        n_sec = self.n_sectors
        out = Float32MultiArray()
        out.layout.dim = [
            MultiArrayDimension(
                label="elevation_bin_td3", size=n_bins, stride=n_bins * n_sec
            ),
            MultiArrayDimension(label="sector", size=n_sec, stride=n_sec),
        ]
        out.layout.data_offset = 0
        out.data = td3_ranges
        self._pub_td3.publish(out)

    def _on_scan(self, msg: LaserScan) -> None:
        n = self.n_sectors
        ranges = [self.max_range] * n
        ang = msg.angle_min
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                ang += msg.angle_increment
                continue
            if r < self.min_range or r > self.max_range:
                ang += msg.angle_increment
                continue
            if r > msg.range_max or r < msg.range_min:
                ang += msg.angle_increment
                continue
            idx = self._sector_index(ang)
            if idx is not None and r < ranges[idx]:
                ranges[idx] = r
            ang += msg.angle_increment
        self._publish_ranges(ranges)
        if self._pub_elev is not None:
            n_bins = self.n_elevation_bins
            n_front = self.n_front_sectors
            elev = [self.max_range] * (n_bins * n_front)
            mid_bin = n_bins // 2
            for i in range(n):
                ang_i = self.angle_max - (float(i) + 0.5) / float(n) * self._span
                fidx = self._front_sector_index(ang_i)
                if fidx is None:
                    continue
                flat = mid_bin * n_front + fidx
                if ranges[i] < elev[flat]:
                    elev[flat] = ranges[i]
            self._publish_elevation(elev)
        if self._pub_td3 is not None:
            n_td3_bins = self.n_elevation_bins_td3
            td3 = [self.max_range] * (n_td3_bins * n)
            mid_bin = n_td3_bins // 2
            for i in range(n):
                flat = mid_bin * n + i
                td3[flat] = ranges[i]
            self._publish_td3_ranges(td3)

    def _to_fwd_lat(self, x: float, y: float, z: float) -> Tuple[float, float]:
        if self.frame_mode == "base":
            return x, y
        return z, -x

    def _passes_elevation(self, x: float, y: float, z: float) -> bool:
        if self.frame_mode == "optical":
            return abs(y) <= self.y_abs_max
        horiz = math.hypot(x, y)
        if horiz < 1e-4:
            return False
        elev = math.atan2(z, horiz)
        return self._elevation_min <= elev <= self._elevation_max

    def _is_body_self_hit(self, x: float, y: float, z: float) -> bool:
        if not self.filter_body_box or self.frame_mode == "optical":
            return False
        horiz = math.hypot(x, y)
        if horiz > self.body_filter_max_range:
            return False
        return (
            self.body_x_min <= x <= self.body_x_max
            and self.body_y_min <= y <= self.body_y_max
            and self.body_z_min <= z <= self.body_z_max
        )

    def _accumulate_cloud_point(
        self,
        ranges: List[float],
        elev_ranges: Optional[List[float]],
        td3_ranges: Optional[List[float]],
        x: float,
        y: float,
        z: float,
    ) -> None:
        if self._is_body_self_hit(x, y, z):
            return
        fwd, lat = self._to_fwd_lat(x, y, z)
        r = math.hypot(fwd, lat)
        if r < self.min_range or r > self.max_range:
            return
        ang = math.atan2(lat, fwd)
        if self._passes_elevation(x, y, z):
            idx = self._sector_index(ang)
            if idx is not None and r < ranges[idx]:
                ranges[idx] = r
        if self.frame_mode != "base":
            return
        horiz = math.hypot(x, y)
        if horiz < 1e-4:
            return
        elev_deg = math.degrees(math.atan2(z, horiz))
        if elev_ranges is not None:
            bidx = self._elevation_bin_index(elev_deg)
            fidx = self._front_sector_index(ang)
            if bidx is not None and fidx is not None:
                flat = bidx * self.n_front_sectors + fidx
                if r < elev_ranges[flat]:
                    elev_ranges[flat] = r
        if td3_ranges is not None:
            tbidx = self._td3_elevation_bin_index(elev_deg)
            sidx = self._sector_index(ang)
            if tbidx is not None and sidx is not None:
                flat = tbidx * self.n_sectors + sidx
                if r < td3_ranges[flat]:
                    td3_ranges[flat] = r

    def _project_cloud(self, points: Iterable[Tuple[float, float, float]]) -> None:
        n = self.n_sectors
        ranges = [self.max_range] * n
        elev = None
        if self._pub_elev is not None:
            elev = [self.max_range] * (self.n_elevation_bins * self.n_front_sectors)
        td3 = None
        if self._pub_td3 is not None:
            td3 = [self.max_range] * (self.n_elevation_bins_td3 * n)
        for x, y, z in points:
            self._accumulate_cloud_point(ranges, elev, td3, x, y, z)
        self._publish_ranges(ranges)
        if elev is not None:
            self._publish_elevation(elev)
        if td3 is not None:
            self._publish_td3_ranges(td3)

    def _on_cloud2(self, msg: PointCloud2) -> None:
        self._project_cloud(_iter_xyz_cloud2(msg))

    def _on_pointcloud(self, msg: PointCloud) -> None:
        self._project_cloud(_iter_xyz_pointcloud(msg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RangeFeaturesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
