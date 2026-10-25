#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hybrid / grid global planner → /long_nav/global_path.

- Accepts RViz /goal_pose only after a fast security-fence connectivity check.
- On arrival (≤ goal_tol): mission SUCCESS — clear path, stop replanning.
- Rejected goals that would cross the red fence keep the current mission unchanged.
"""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from scout_workspace_common import debug_log_path
from scout_long_nav.cost_codes import OCC_SECURITY
from scout_long_nav.hybrid_astar import (
    HybridAstarParams,
    plan_global_path,
    security_blocks_connection,
)

DEBUG_LOG = debug_log_path(
    "long_nav_hybrid_astar.jsonl",
    package_name="scout_long_nav",
    start_path=__file__,
)


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _agent_log(hid: str, location: str, message: str, data=None) -> None:
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


class HybridAstarNode(Node):
    def __init__(self):
        super().__init__("hybrid_astar_node")
        self.declare_parameter("costmap_topic", "/long_nav/costmap")
        self.declare_parameter("odom_topic", "/long_nav/odom")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("goal_topic", "/long_nav/goal")
        self.declare_parameter("path_topic", "/long_nav/global_path")
        self.declare_parameter("xy_resolution", 1.0)
        self.declare_parameter("max_iterations", 35000)
        self.declare_parameter("replan_hz", 0.5)
        self.declare_parameter("step_length", 1.0)
        self.declare_parameter("goal_xy_tol", 2.0)
        self.declare_parameter("footprint_radius_m", 0.25)
        self.declare_parameter("arrive_tol_m", 2.0)

        self._grid_data = None
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._resolution = 0.2
        self._width = 0
        self._height = 0
        self._pose = None
        self._goal = None
        self.frame_id = "odom"
        self._last_no_path_log = 0.0
        self._no_path_since = None
        self._mission_active = False
        self._last_map_log = 0.0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), qos
        )
        self._abort_pub = self.create_publisher(String, "/long_nav/nav_abort", 10)
        self._success_pub = self.create_publisher(String, "/long_nav/nav_success", 10)
        self._reject_pub = self.create_publisher(String, "/long_nav/goal_rejected", 10)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("costmap_topic").value), self._on_map, qos
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, qos_sensor
        )
        # RViz 2D Goal Pose → 先快检红线，通过才发 /long_nav/goal
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_pose_topic").value),
            self._on_goal_pose,
            qos,
        )
        self.create_subscription(Bool, "/long_nav/cancel_nav", self._on_cancel, 10)
        period = 1.0 / max(float(self.get_parameter("replan_hz").value), 0.1)
        self.create_timer(period, self._replan)
        self.get_logger().info(
            "hybrid_astar_node ready (goal gate + arrive SUCCESS + security check)"
        )

    def _publish_empty_path(self):
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        self._pub.publish(out)

    def _on_cancel(self, msg: Bool):
        if bool(msg.data):
            self._goal = None
            self._mission_active = False
            self._no_path_since = None
            self._publish_empty_path()
            self.get_logger().warn("cancel_nav — 本次导航已废除（非穿红线）")
            _agent_log("S", "hybrid_astar_node._on_cancel", "cancel", {})

    def _abort_security(self, reason: str):
        self._goal = None
        self._mission_active = False
        self._no_path_since = None
        self._publish_empty_path()
        msg = String()
        msg.data = reason
        self._abort_pub.publish(msg)
        self.get_logger().error("NAV ABORT(security): %s" % reason)
        _agent_log("S", "hybrid_astar_node._abort_security", "abort", {"reason": reason})

    def _complete_success(self, dist: float):
        """到点：结束任务、清路径，发 SUCCESS（不再蹭点）。"""
        gx = self._goal[0] if self._goal else 0.0
        gy = self._goal[1] if self._goal else 0.0
        self._goal = None
        self._mission_active = False
        self._no_path_since = None
        self._publish_empty_path()
        text = (
            "本次导航任务完成 SUCCESS "
            "(到点 d=%.2fm ≤ %.2fm，终点≈(%.1f,%.1f)。请重新 2D Goal Pose 开始下一次)"
            % (dist, float(self.get_parameter("arrive_tol_m").value), gx, gy)
        )
        msg = String()
        msg.data = text
        self._success_pub.publish(msg)
        self.get_logger().info(">>> %s" % text)
        _agent_log(
            "G",
            "hybrid_astar_node._complete_success",
            "success",
            {"dist": round(dist, 2), "goal": [round(gx, 2), round(gy, 2)]},
        )

    def _on_map(self, msg: OccupancyGrid):
        try:
            self._origin_x = float(msg.info.origin.position.x)
            self._origin_y = float(msg.info.origin.position.y)
            self._resolution = float(msg.info.resolution)
            self._width = int(msg.info.width)
            self._height = int(msg.info.height)
            self._grid_data = np.asarray(msg.data, dtype=np.int16).reshape(-1).copy()
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id
            now = time.time()
            if now - self._last_map_log > 5.0:
                self._last_map_log = now
                n_sec = int(np.sum(self._grid_data >= OCC_SECURITY))
                _agent_log(
                    "R",
                    "hybrid_astar_node._on_map",
                    "costmap_snapshot",
                    {
                        "w": self._width,
                        "h": self._height,
                        "res": self._resolution,
                        "security_cells": n_sec,
                        "max_val": int(self._grid_data.max()) if self._grid_data.size else None,
                    },
                )
        except Exception as exc:
            self.get_logger().warn("costmap snapshot failed: %s" % exc)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _on_goal_pose(self, msg: PoseStamped):
        """RViz 新终点：快检是否必穿红线；通过才切换，否则保持现状。"""
        p = msg.pose
        cand = (p.position.x, p.position.y, _yaw_from_quat(p.orientation))
        if self._pose is None:
            self.get_logger().warn("goal ignored — no odom yet")
            return
        if self._grid_data is None or self._grid_data.size != self._width * self._height:
            self.get_logger().warn("goal ignored — no costmap yet")
            return

        must_cross = security_blocks_connection(
            self._origin_x,
            self._origin_y,
            self._resolution,
            self._width,
            self._height,
            self._grid_data,
            (self._pose[0], self._pose[1]),
            (cand[0], cand[1]),
        )
        _agent_log(
            "G",
            "hybrid_astar_node._on_goal_pose",
            "candidate",
            {
                "must_cross": must_cross,
                "mission": self._mission_active,
                "pose": [round(self._pose[0], 2), round(self._pose[1], 2)],
                "cand": [round(cand[0], 2), round(cand[1], 2)],
            },
        )
        if must_cross:
            reason = (
                "新终点会被安全红线隔开（从当前位姿无法不过红线到达）。"
                "已拒绝切换；"
                + (
                    "继续执行当前导航。"
                    if self._mission_active
                    else "当前无导航任务，保持待机。"
                )
            )
            self.get_logger().warn("GOAL REJECTED(security): %s" % reason)
            rj = String()
            rj.data = reason
            self._reject_pub.publish(rj)
            return

        # 接受：切换终点，并广播 /long_nav/goal（gate 解锁并跟新任务）
        self._goal = cand
        self._mission_active = True
        self._no_path_since = None
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id or "odom"
        out.pose = msg.pose
        self._goal_pub.publish(out)
        self.get_logger().info(
            "new goal ACCEPTED (%.2f,%.2f) — mission active / switched"
            % (cand[0], cand[1])
        )

    def _replan(self):
        if self._grid_data is None or self._pose is None or self._goal is None:
            return
        if self._grid_data.size != self._width * self._height:
            return

        # 到点成功：停规划、清红线路径
        arrive_tol = float(self.get_parameter("arrive_tol_m").value)
        dist = math.hypot(self._pose[0] - self._goal[0], self._pose[1] - self._goal[1])
        if self._mission_active and dist <= arrive_tol:
            self._complete_success(dist)
            return

        params = HybridAstarParams(
            xy_resolution=float(self.get_parameter("xy_resolution").value),
            max_iterations=int(self.get_parameter("max_iterations").value),
            step_length=float(self.get_parameter("step_length").value),
            goal_xy_tol=float(self.get_parameter("goal_xy_tol").value),
            footprint_radius_m=float(self.get_parameter("footprint_radius_m").value),
        )
        try:
            path, planner = plan_global_path(
                self._pose,
                self._goal,
                self._origin_x,
                self._origin_y,
                self._resolution,
                self._width,
                self._height,
                self._grid_data,
                params,
            )
        except Exception as exc:
            self.get_logger().warn("global plan exception: %s" % exc)
            return

        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        if path is None or not path.xs:
            now = time.time()
            if self._no_path_since is None:
                self._no_path_since = now
            must_cross = security_blocks_connection(
                self._origin_x,
                self._origin_y,
                self._resolution,
                self._width,
                self._height,
                self._grid_data,
                (self._pose[0], self._pose[1]),
                (self._goal[0], self._goal[1]),
            )
            _agent_log(
                "A",
                "hybrid_astar_node._replan",
                "no_path",
                {
                    "must_cross": must_cross,
                    "mission": self._mission_active,
                    "dt": round(now - self._no_path_since, 2),
                    "planner": planner,
                    "pose": [round(self._pose[0], 2), round(self._pose[1], 2)],
                    "goal": [round(self._goal[0], 2), round(self._goal[1], 2)],
                },
            )
            if now - self._last_no_path_log > 3.0:
                self.get_logger().warn(
                    "global plan failed (security_blocks=%s planner=%s)"
                    % (must_cross, planner)
                )
                self._last_no_path_log = now
            if (
                self._mission_active
                and must_cross
                and self._no_path_since is not None
                and (now - self._no_path_since) >= 1.5
            ):
                self._abort_security(
                    "小车规划路径会穿过安全红线（代价无限大）。"
                    "本次导航已停止，红线已清除。"
                    "请重新设置导航终点，或用 2D Pose Estimate / 重生改起点后再导航。"
                )
                return
            self._pub.publish(out)
            return

        self._no_path_since = None
        for x, y, yaw in zip(path.xs, path.ys, path.yaws):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = math.sin(yaw * 0.5)
            ps.pose.orientation.w = math.cos(yaw * 0.5)
            out.poses.append(ps)
        self._pub.publish(out)
        _agent_log(
            "N",
            "hybrid_astar_node._replan",
            "path_ok",
            {
                "n": len(path.xs),
                "cost": float(path.cost),
                "planner": planner,
                "pose": [round(self._pose[0], 2), round(self._pose[1], 2)],
                "goal": [round(self._goal[0], 2), round(self._goal[1], 2)],
            },
        )


def main(args=None):
    rclpy.init(args=args)
    node = HybridAstarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
