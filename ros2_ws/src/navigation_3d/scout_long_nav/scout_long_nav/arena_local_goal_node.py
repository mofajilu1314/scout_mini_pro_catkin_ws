#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Arena-style look-ahead: Path + odom → local_goal + colored waypoint spheres."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from scout_long_nav.lookahead import lookahead_point, lookahead_waypoints


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _lerp_rgb(t: float):
    """Near robot → cyan; far (lookahead) → hot pink."""
    t = max(0.0, min(1.0, float(t)))
    r0, g0, b0 = 0.15, 0.95, 1.0
    r1, g1, b1 = 1.0, 0.25, 0.85
    return (r0 + (r1 - r0) * t, g0 + (g1 - g0) * t, b0 + (b1 - b0) * t)


class ArenaLocalGoalNode(Node):
    def __init__(self):
        super().__init__("arena_local_goal_node")
        self.declare_parameter("path_topic", "/long_nav/global_path")
        self.declare_parameter("odom_topic", "/long_nav/odom")
        self.declare_parameter("local_goal_topic", "/offroad/local_goal")
        self.declare_parameter("waypoints_marker_topic", "/long_nav/arena_waypoints")
        self.declare_parameter("lookahead_m", 4.0)
        self.declare_parameter("waypoint_spacing_m", 0.8)
        self.declare_parameter("sphere_diameter_m", 0.55)
        self.declare_parameter("sphere_z", 0.45)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("frame_id", "odom")

        self.lookahead_m = float(self.get_parameter("lookahead_m").value)
        self.spacing_m = float(self.get_parameter("waypoint_spacing_m").value)
        self.sphere_d = float(self.get_parameter("sphere_diameter_m").value)
        self.sphere_z = float(self.get_parameter("sphere_z").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self._path_pts = []
        self._pose = None  # (x, y, yaw)
        self._marker_topic = str(self.get_parameter("waypoints_marker_topic").value)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("local_goal_topic").value), qos
        )
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, qos)
        self.create_subscription(
            Path, str(self.get_parameter("path_topic").value), self._on_path, qos
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, qos_sensor
        )
        period = 1.0 / max(float(self.get_parameter("rate_hz").value), 1.0)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "arena_local_goal lookahead=%.2fm | spheres → %s"
            % (self.lookahead_m, self._marker_topic)
        )

    def _on_path(self, msg: Path):
        self._path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, _yaw_from_quat(msg.pose.pose.orientation))

    def _publish_spheres(self, wps, stamp):
        arr = MarkerArray()
        n = max(len(wps), 1)
        for i, (wx, wy) in enumerate(wps):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.frame_id
            m.ns = "arena_waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = float(self.sphere_z)
            m.pose.orientation.w = 1.0
            m.scale.x = self.sphere_d
            m.scale.y = self.sphere_d
            m.scale.z = self.sphere_d
            r, g, b = _lerp_rgb(i / max(n - 1, 1))
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = 0.95
            m.lifetime.sec = 0
            m.lifetime.nanosec = 250000000  # 0.25s — refreshed each tick
            arr.markers.append(m)
        # delete leftover ids if path shortened
        for j in range(len(wps), len(wps) + 32):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.frame_id
            m.ns = "arena_waypoints"
            m.id = j
            m.action = Marker.DELETE
            arr.markers.append(m)
        self._marker_pub.publish(arr)

    def _tick(self):
        if not self._path_pts or self._pose is None:
            return
        x, y, yaw = self._pose
        goal = lookahead_point(self._path_pts, x, y, self.lookahead_m)
        if goal is None:
            return
        stamp = self.get_clock().now().to_msg()
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.frame_id
        out.pose.position.x = float(goal[0])
        out.pose.position.y = float(goal[1])
        out.pose.position.z = 0.0
        bearing = math.atan2(goal[1] - y, goal[0] - x)
        out.pose.orientation.z = math.sin(bearing * 0.5)
        out.pose.orientation.w = math.cos(bearing * 0.5)
        self._pub.publish(out)

        wps = lookahead_waypoints(
            self._path_pts, x, y, self.lookahead_m, self.spacing_m
        )
        self._publish_spheres(wps, stamp)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaLocalGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
