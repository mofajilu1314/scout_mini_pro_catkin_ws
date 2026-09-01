#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Range sector features for off-road / TD3 local control.

Sources:
  source:=scan   — LaserScan (/scan), near-360° when 2D lidar FOV allows
  source:=cloud  — PointCloud2 or Gazebo block_laser PointCloud (16-line lidar)

Outputs (scheme A partial — TD3 gets 3 elevation layers; collision unchanged):
  /offroad/td3_range_features  — n_elevation_bins_td3 × N sectors (default 3×24=72) → TD3 state
  /offroad/range_features      — N horizontal sectors (default 24); legacy / collision horizontal
  /offroad/elevation_features  — n_elevation_bins × n_front_sectors (default 4×12=48); slope/wall

Sector indexing: left (angle_max) → idx 0, right (angle_min) → idx N-1.

3D lidar (laser_link, +x fwd +y left +z up): elevation bins partition ±15° FOV;
body-box self-filter applied before binning.
"""

from __future__ import print_function

import math
import random
import struct

import rospy
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def _parse_float_list(raw, default):
    if raw is None:
        return list(default)
    if isinstance(raw, (list, tuple)):
        return [float(x) for x in raw]
    parts = str(raw).replace(",", " ").split()
    if not parts:
        return list(default)
    return [float(x) for x in parts]


def _field_offsets(cloud):
    offsets = {}
    for f in cloud.fields:
        if f.name in ("x", "y", "z"):
            offsets[f.name] = f.offset
    if len(offsets) < 3:
        return 0, 4, 8
    return offsets["x"], offsets["y"], offsets["z"]


def _iter_xyz_cloud2(cloud):
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


def _iter_xyz_pointcloud(cloud):
    for p in cloud.points:
        x, y, z = p.x, p.y, p.z
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            continue
        if math.isinf(x) or math.isinf(y) or math.isinf(z):
            continue
        if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            continue
        yield x, y, z


class RangeFeaturesNode(object):
    def __init__(self):
        self.n_sectors = int(rospy.get_param("~n_sectors", 24))
        self.max_range = float(rospy.get_param("~max_range", 5.0))
        self.min_range = float(rospy.get_param("~min_range", 0.18))
        # Near-horizontal band for TD3 /offroad/range_features (|z|/r band)
        self.elevation_min_deg = float(rospy.get_param("~elevation_min_deg", -10.0))
        self.elevation_max_deg = float(rospy.get_param("~elevation_max_deg", 10.0))
        # Multi-elevation bins for /offroad/elevation_features (collision / terrain)
        self.publish_elevation_features = bool(
            rospy.get_param("~publish_elevation_features", True)
        )
        self.n_elevation_bins = int(rospy.get_param("~n_elevation_bins", 4))
        self.n_front_sectors = int(rospy.get_param("~n_front_sectors", 12))
        self.front_half_fov_deg = float(rospy.get_param("~front_half_fov_deg", 60.0))
        # Contiguous edges (deg) partition lidar vertical FOV into bins labelled ≈ -10/0/+10/+20
        default_edges = [-15.0, -5.0, 5.0, 12.5, 15.01]
        self._elevation_bin_edges_deg = _parse_float_list(
            rospy.get_param("~elevation_bin_edges_deg", default_edges),
            default_edges,
        )
        if len(self._elevation_bin_edges_deg) != self.n_elevation_bins + 1:
            rospy.logwarn(
                "elevation_bin_edges_deg length %d != n_elevation_bins+1 (%d); rebuilding",
                len(self._elevation_bin_edges_deg),
                self.n_elevation_bins + 1,
            )
            self._elevation_bin_edges_deg = default_edges[: self.n_elevation_bins + 1]
        # TD3 state: 3 elevation bins × full-circle sectors (excludes top +20° layer)
        self.publish_td3_range_features = bool(
            rospy.get_param("~publish_td3_range_features", True)
        )
        self.n_elevation_bins_td3 = int(rospy.get_param("~n_elevation_bins_td3", 3))
        default_td3_edges = [-15.0, -5.0, 5.0, 12.5]
        self._td3_elevation_bin_edges_deg = _parse_float_list(
            rospy.get_param("~td3_elevation_bin_edges_deg", default_td3_edges),
            default_td3_edges,
        )
        if len(self._td3_elevation_bin_edges_deg) != self.n_elevation_bins_td3 + 1:
            rospy.logwarn(
                "td3_elevation_bin_edges_deg length %d != n_elevation_bins_td3+1 (%d); rebuilding",
                len(self._td3_elevation_bin_edges_deg),
                self.n_elevation_bins_td3 + 1,
            )
            self._td3_elevation_bin_edges_deg = default_td3_edges[
                : self.n_elevation_bins_td3 + 1
            ]
        # Legacy alias for depth camera optical frame (|y| band)
        self.y_abs_max = float(rospy.get_param("~y_abs_max", 0.6))
        # optical: Kinect (+z fwd, +x right, +y down); base/lidar: +x fwd, +y left, +z up
        self.frame_mode = rospy.get_param("~frame_mode", "base").strip().lower()
        self.source = rospy.get_param("~source", "cloud").strip().lower()
        self.cloud_msg_type = rospy.get_param("~cloud_msg_type", "pointcloud").strip().lower()
        self.full_circle = bool(rospy.get_param("~full_circle", True))
        use_angle_window = bool(rospy.get_param("~use_angle_window", False))

        # Body self-hit filter (laser_link / base-aligned frame)
        self.filter_body_box = bool(rospy.get_param("~filter_body_box", True))
        self.body_x_min = float(rospy.get_param("~body_x_min", -0.36))
        self.body_x_max = float(rospy.get_param("~body_x_max", 0.36))
        self.body_y_min = float(rospy.get_param("~body_y_min", -0.32))
        self.body_y_max = float(rospy.get_param("~body_y_max", 0.32))
        self.body_z_min = float(rospy.get_param("~body_z_min", -0.42))
        self.body_z_max = float(rospy.get_param("~body_z_max", 0.06))
        self.body_filter_max_range = float(
            rospy.get_param("~body_filter_max_range", 1.2)
        )
        self.sensor_noise_std = float(rospy.get_param("~sensor_noise_std", 0.0))

        if self.full_circle:
            self.angle_min = float(rospy.get_param("~angle_min", -math.pi))
            self.angle_max = float(rospy.get_param("~angle_max", math.pi))
        elif use_angle_window:
            self.angle_min = float(rospy.get_param("~angle_min", -math.pi / 2.0))
            self.angle_max = float(rospy.get_param("~angle_max", math.pi / 2.0))
        else:
            fov_deg = float(rospy.get_param("~fov_deg", 87.0))
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

        self._pub = rospy.Publisher(
            "/offroad/range_features", Float32MultiArray, queue_size=1
        )
        self._pub_elev = None
        if self.publish_elevation_features:
            self._pub_elev = rospy.Publisher(
                "/offroad/elevation_features", Float32MultiArray, queue_size=1
            )
        self._pub_td3 = None
        if self.publish_td3_range_features:
            self._pub_td3 = rospy.Publisher(
                "/offroad/td3_range_features", Float32MultiArray, queue_size=1
            )

        if self.source == "scan":
            scan_topic = rospy.get_param("~scan_topic", "/scan")
            self._sub = rospy.Subscriber(
                scan_topic, LaserScan, self._on_scan, queue_size=1
            )
            rospy.loginfo(
                "range_features: source=scan sub=%s pub=/offroad/range_features "
                "N=%d max=%.1f ang=[%.1f°,%.1f°] full_circle=%s elev_pub=%s",
                scan_topic,
                self.n_sectors,
                self.max_range,
                math.degrees(self.angle_min),
                math.degrees(self.angle_max),
                self.full_circle,
                self.publish_elevation_features,
            )
        elif self.source == "cloud":
            cloud_topic = rospy.get_param("~cloud_topic", "/lidar/points")
            if self.cloud_msg_type == "pointcloud2":
                self._sub = rospy.Subscriber(
                    cloud_topic,
                    PointCloud2,
                    self._on_cloud2,
                    queue_size=1,
                    buff_size=2**24,
                )
            else:
                self._sub = rospy.Subscriber(
                    cloud_topic,
                    PointCloud,
                    self._on_pointcloud,
                    queue_size=1,
                )
            rospy.loginfo(
                "range_features: source=cloud sub=%s (%s) pub=/offroad/range_features "
                "N=%d max=%.1f mode=%s elev=[%.1f°,%.1f°] body_filter=%s "
                "ang=[%.1f°,%.1f°] elev_bins=%d front_sectors=%d edges=%s "
                "td3_pub=%s td3_bins=%d td3_edges=%s",
                cloud_topic,
                self.cloud_msg_type,
                self.n_sectors,
                self.max_range,
                self.frame_mode,
                self.elevation_min_deg,
                self.elevation_max_deg,
                self.filter_body_box,
                math.degrees(self.angle_min),
                math.degrees(self.angle_max),
                self.n_elevation_bins,
                self.n_front_sectors,
                self._elevation_bin_edges_deg,
                self.publish_td3_range_features,
                self.n_elevation_bins_td3,
                self._td3_elevation_bin_edges_deg,
            )
        else:
            raise ValueError("source must be 'scan' or 'cloud', got %r" % self.source)

    def _sector_index(self, ang):
        """Map yaw ang (rad, +left) into [0, N-1]; left→0, right→N-1."""
        if ang < self.angle_min or ang > self.angle_max:
            return None
        t = (self.angle_max - ang) / self._span
        idx = int(t * self.n_sectors)
        if idx < 0:
            idx = 0
        elif idx >= self.n_sectors:
            idx = self.n_sectors - 1
        return idx

    def _front_sector_index(self, ang):
        """Front-only sector index for elevation_features (0=left, n-1=right)."""
        if abs(ang) > self._front_half_fov + 1e-9:
            return None
        t = (self._front_half_fov - ang) / self._front_span
        idx = int(t * self.n_front_sectors)
        if idx < 0:
            idx = 0
        elif idx >= self.n_front_sectors:
            idx = self.n_front_sectors - 1
        return idx

    def _elevation_bin_index(self, elev_deg):
        edges = self._elevation_bin_edges_deg
        for i in range(len(edges) - 1):
            if edges[i] <= elev_deg < edges[i + 1]:
                return i
        return None

    def _td3_elevation_bin_index(self, elev_deg):
        edges = self._td3_elevation_bin_edges_deg
        for i in range(len(edges) - 1):
            if edges[i] <= elev_deg < edges[i + 1]:
                return i
        return None

    def _apply_sensor_noise(self, ranges):
        """Optional Gaussian noise on published ranges (m); trainer may adapt at runtime."""
        if self.sensor_noise_std <= 0.0:
            return ranges
        out = []
        for r in ranges:
            noisy = float(r) + random.gauss(0.0, self.sensor_noise_std)
            noisy = max(self.min_range, min(self.max_range, noisy))
            out.append(noisy)
        return out

    def _publish_ranges(self, ranges):
        ranges = self._apply_sensor_noise(ranges)
        out = Float32MultiArray()
        out.layout.dim = [
            MultiArrayDimension(label="sectors", size=self.n_sectors, stride=self.n_sectors)
        ]
        out.layout.data_offset = 0
        out.data = ranges
        self._pub.publish(out)

    def _publish_elevation(self, elev_ranges):
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

    def _publish_td3_ranges(self, td3_ranges):
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

    def _on_scan(self, msg):
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
        # 2D scan has no elevation — duplicate horizontal front mins into mid bin only
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
        # 2D scan has no elevation — duplicate horizontal mins into mid TD3 bin only
        if self._pub_td3 is not None:
            n_td3_bins = self.n_elevation_bins_td3
            td3 = [self.max_range] * (n_td3_bins * n)
            mid_bin = n_td3_bins // 2
            for i in range(n):
                flat = mid_bin * n + i
                td3[flat] = ranges[i]
            self._publish_td3_ranges(td3)

    def _to_fwd_lat(self, x, y, z):
        if self.frame_mode == "base":
            return x, y  # +x fwd, +y left
        # optical default (depth camera)
        return z, -x  # +z fwd, +x right → lat = -x so +lat = left

    def _passes_elevation(self, x, y, z):
        if self.frame_mode == "optical":
            return abs(y) <= self.y_abs_max
        horiz = math.hypot(x, y)
        if horiz < 1e-4:
            return False
        elev = math.atan2(z, horiz)
        return self._elevation_min <= elev <= self._elevation_max

    def _is_body_self_hit(self, x, y, z):
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

    def _accumulate_cloud_point(self, ranges, elev_ranges, td3_ranges, x, y, z):
        if self._is_body_self_hit(x, y, z):
            return

        fwd, lat = self._to_fwd_lat(x, y, z)
        r = math.hypot(fwd, lat)
        if r < self.min_range or r > self.max_range:
            return
        ang = math.atan2(lat, fwd)

        # Legacy horizontal projection (near-horizontal band; collision / compat)
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

        # Multi-elevation front sectors (slope/wall classifier — unchanged)
        if elev_ranges is not None:
            bidx = self._elevation_bin_index(elev_deg)
            fidx = self._front_sector_index(ang)
            if bidx is not None and fidx is not None:
                flat = bidx * self.n_front_sectors + fidx
                if r < elev_ranges[flat]:
                    elev_ranges[flat] = r

        # TD3: 3 elevation bins × full-circle sectors (excludes +20° top layer)
        if td3_ranges is not None:
            tbidx = self._td3_elevation_bin_index(elev_deg)
            sidx = self._sector_index(ang)
            if tbidx is not None and sidx is not None:
                flat = tbidx * self.n_sectors + sidx
                if r < td3_ranges[flat]:
                    td3_ranges[flat] = r

    def _project_cloud(self, points):
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

    def _on_cloud2(self, msg):
        self._project_cloud(_iter_xyz_cloud2(msg))

    def _on_pointcloud(self, msg):
        self._project_cloud(_iter_xyz_pointcloud(msg))


def main():
    rospy.init_node("range_features_node")
    RangeFeaturesNode()
    rospy.spin()


if __name__ == "__main__":
    main()
