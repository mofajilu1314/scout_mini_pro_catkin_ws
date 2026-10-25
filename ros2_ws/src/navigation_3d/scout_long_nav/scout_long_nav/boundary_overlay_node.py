# -*- coding: utf-8 -*-
"""Live boundary overlay: base trav map + fence bake → costmap + red markers."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from scout_workspace_common import debug_log_path, workspace_path
from scout_long_nav.cost_codes import SECURITY_COST, grid_to_occ_int8
from scout_long_nav.fence_bake import bake_fence_into_grid, segments_from_points
from scout_long_nav.isolation import DEFAULT_MAP_DIR, DEFAULT_MAP_STEM, map_paths
from scout_long_nav.map_io import load_trav_map, save_trav_map

DEBUG_LOG = debug_log_path(
    "long_nav_boundary_overlay.jsonl",
    package_name="scout_long_nav",
    start_path=__file__,
)
DEFAULT_STATE_FILE = workspace_path(
    "3D",
    "debug_logs",
    "scout_nav_boundary_live.json",
    package_name="scout_long_nav",
    start_path=__file__,
)
DEFAULT_SECURITY_DIR = workspace_path(
    "3D", "maps_security", package_name="scout_long_nav", start_path=__file__
)


def _agent_log(hid: str, location: str, message: str, data: Optional[dict] = None) -> None:
    # #region agent log
    try:
        with open(DEBUG_LOG, "a") as f:
            f.write(
                json.dumps(
                    {
                        "sessionId": "8873ab",
                        "hypothesisId": hid,
                        "location": location,
                        "message": message,
                        "data": data or {},
                        "timestamp": int(time.time() * 1000),
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
    except Exception:
        pass
    # #endregion


class BoundaryOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__("boundary_overlay_node")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_dir", DEFAULT_MAP_DIR)
        self.declare_parameter("map_stem", DEFAULT_MAP_STEM)
        self.declare_parameter("costmap_topic", "/long_nav/costmap")
        self.declare_parameter("marker_topic", "/scout_nav/boundary_markers")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("fence_half_width_m", 0.28)
        self.declare_parameter("security_dir", "")
        self.declare_parameter(
            "state_file",
            str(DEFAULT_STATE_FILE),
        )

        yaml_path = str(self.get_parameter("map_yaml").value).strip()
        if not yaml_path:
            yaml_path, _ = map_paths(
                str(self.get_parameter("map_dir").value),
                str(self.get_parameter("map_stem").value),
            )
        loaded = load_trav_map(yaml_path)
        self._base = loaded
        self._yaml_path = yaml_path
        self._frame = str(self.get_parameter("frame_id").value).strip() or loaded["frame_id"]
        self._half_w = float(self.get_parameter("fence_half_width_m").value)
        sec = str(self.get_parameter("security_dir").value).strip()
        self._security_dir = Path(sec) if sec else DEFAULT_SECURITY_DIR
        self._state_file = Path(str(self.get_parameter("state_file").value))
        self._state_mtime = -1.0

        self._points: List[Tuple[float, float]] = []
        self._closed = False
        self._grid = np.array(loaded["grid"], dtype=np.int16, copy=True)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        topic = str(self.get_parameter("costmap_topic").value)
        self._pub = self.create_publisher(OccupancyGrid, topic, qos)
        mtopic = str(self.get_parameter("marker_topic").value)
        self._mpub = self.create_publisher(MarkerArray, mtopic, 10)
        self.create_subscription(String, "/scout_nav/boundary_json", self._on_json, 10)
        self.create_subscription(String, "/scout_nav/save_security", self._on_save, 10)

        period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.1)
        self.create_timer(period, self._tick)
        self.create_timer(0.2, self._poll_state_file)
        self.get_logger().info(
            "boundary_overlay: %s (%dx%d) → %s + markers (state=%s)"
            % (yaml_path, loaded["width"], loaded["height"], topic, self._state_file)
        )
        _agent_log(
            "K",
            "boundary_overlay.__init__",
            "ready",
            {"yaml": yaml_path, "topic": topic, "frame": self._frame, "state": str(self._state_file)},
        )
        self._tick()

    def _apply_points(self, pts: List[Tuple[float, float]], closed: bool) -> None:
        self._points = pts
        self._closed = closed
        segs = segments_from_points(pts, closed=closed)
        self._grid = bake_fence_into_grid(
            self._base["grid"],
            resolution=float(self._base["resolution"]),
            origin_x=float(self._base["origin_x"]),
            origin_y=float(self._base["origin_y"]),
            segments=segs,
            half_width_m=self._half_w,
            lethal=SECURITY_COST,
        )
        n_sec = int(np.sum(self._grid == SECURITY_COST) - np.sum(self._base["grid"] == SECURITY_COST))
        self.get_logger().info(
            "boundary update: %d pts closed=%s segs=%d new_security(1000)≈%d"
            % (len(pts), closed, len(segs), max(0, n_sec))
        )
        _agent_log(
            "L",
            "boundary_overlay._apply_points",
            "baked",
            {
                "n": len(pts),
                "closed": closed,
                "segs": len(segs),
                "new_security_1000": max(0, n_sec),
                "security_cost": SECURITY_COST,
            },
        )
        self._tick()

    def _poll_state_file(self) -> None:
        try:
            if not self._state_file.exists():
                return
            mtime = self._state_file.stat().st_mtime
            if mtime <= self._state_mtime:
                return
            self._state_mtime = mtime
            payload = json.loads(self._state_file.read_text(encoding="utf-8"))
            if payload.get("cmd") == "save":
                out = str(payload.get("security_dir") or self._security_dir)
                self._do_save(Path(out))
                # clear cmd so we don't re-save
                payload["cmd"] = ""
                self._state_file.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
                self._state_mtime = self._state_file.stat().st_mtime
                return
            pts = [(float(p[0]), float(p[1])) for p in payload.get("points", [])]
            closed = bool(payload.get("closed", False))
            self._apply_points(pts, closed)
        except Exception as exc:
            self.get_logger().warn("state_file poll fail: %s" % exc)

    def _on_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            pts = [(float(p[0]), float(p[1])) for p in payload.get("points", [])]
            closed = bool(payload.get("closed", False))
        except Exception as exc:
            self.get_logger().warn("bad boundary_json: %s" % exc)
            _agent_log("L", "boundary_overlay._on_json", "parse_fail", {"err": str(exc)})
            return
        self._apply_points(pts, closed)

    def _do_save(self, out_dir: Path) -> None:
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = str(self.get_parameter("map_stem").value) or "arena_trav"
        out_yaml = out_dir / f"{stem}.yaml"
        try:
            save_trav_map(
                str(out_yaml),
                self._grid,
                float(self._base["resolution"]),
                float(self._base["origin_x"]),
                float(self._base["origin_y"]),
                frame_id=self._frame,
            )
            side = out_dir / f"{stem}_boundary.json"
            side.write_text(
                json.dumps(
                    {"points": self._points, "closed": self._closed},
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            self.get_logger().info("saved security costmap → %s" % out_yaml)
            _agent_log(
                "M",
                "boundary_overlay._do_save",
                "saved",
                {"yaml": str(out_yaml), "n": len(self._points), "closed": self._closed},
            )
        except Exception as exc:
            self.get_logger().error("save security failed: %s" % exc)
            _agent_log("M", "boundary_overlay._do_save", "fail", {"err": str(exc)})

    def _on_save(self, msg: String) -> None:
        out_dir = Path(msg.data.strip()) if msg.data.strip() else self._security_dir
        self._do_save(out_dir)
    def _tick(self) -> None:
        L = self._base
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._frame
        grid.info.resolution = float(L["resolution"])
        grid.info.width = int(L["width"])
        grid.info.height = int(L["height"])
        grid.info.origin.position.x = float(L["origin_x"])
        grid.info.origin.position.y = float(L["origin_y"])
        grid.info.origin.orientation.w = 1.0
        # OccupancyGrid is int8: security 1000 → 127 on the wire
        grid.data = grid_to_occ_int8(self._grid)
        self._pub.publish(grid)
        self._mpub.publish(self._make_markers())

    def _make_markers(self) -> MarkerArray:
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        delete = Marker()
        delete.header.frame_id = self._frame
        delete.header.stamp = now
        delete.ns = "scout_nav_boundary"
        delete.id = 0
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        # Full-map click plane so Publish Point works on unscanned (transparent) cells
        L = self._base
        res = float(L["resolution"])
        w = float(L["width"]) * res
        h = float(L["height"]) * res
        ox = float(L["origin_x"])
        oy = float(L["origin_y"])
        plane = Marker()
        plane.header.frame_id = self._frame
        plane.header.stamp = now
        plane.ns = "scout_nav_boundary"
        plane.id = 2
        plane.type = Marker.CUBE
        plane.action = Marker.ADD
        plane.pose.position.x = ox + w * 0.5
        plane.pose.position.y = oy + h * 0.5
        plane.pose.position.z = -0.02
        plane.pose.orientation.w = 1.0
        plane.scale.x = w
        plane.scale.y = h
        plane.scale.z = 0.04
        plane.color.r = 0.05
        plane.color.g = 0.05
        plane.color.b = 0.05
        plane.color.a = 0.01  # nearly invisible, but ray-pickable
        arr.markers.append(plane)

        if len(self._points) < 1:
            return arr

        line = Marker()
        line.header.frame_id = self._frame
        line.header.stamp = now
        line.ns = "scout_nav_boundary"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.45  # thick corridor outline
        line.color.r = 1.0
        line.color.g = 0.05
        line.color.b = 0.05
        line.color.a = 1.0
        line.pose.orientation.w = 1.0
        pts = list(self._points)
        if self._closed and len(pts) >= 3:
            pts = pts + [pts[0]]
        for x, y in pts:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.12
            line.points.append(p)
        arr.markers.append(line)

        for i, (x, y) in enumerate(self._points):
            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = now
            m.ns = "scout_nav_boundary"
            m.id = 10 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.55
            m.color.r = 1.0
            m.color.g = 0.15
            m.color.b = 0.1
            m.color.a = 1.0
            arr.markers.append(m)
        return arr


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoundaryOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
