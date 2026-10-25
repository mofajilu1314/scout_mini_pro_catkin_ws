#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav cmd_vel gate: pass TD3 until goal reached, then lock wheels until next accepted Goal.

Only reacts to accepted goals on /long_nav/goal (hybrid_astar validates /goal_pose first).
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class NavCmdGateNode(Node):
    def __init__(self):
        super().__init__("nav_cmd_gate_node")
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel_td3")
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/long_nav/odom")
        self.declare_parameter("goal_topic", "/long_nav/goal")
        self.declare_parameter("hold_topic", "/long_nav/cmd_hold")
        self.declare_parameter("arrived_topic", "/long_nav/goal_arrived")
        self.declare_parameter("goal_tol_m", 2.0)
        self.declare_parameter("ext_hold_stale_sec", 0.4)
        self.declare_parameter("gate_hz", 50.0)

        self._goal: Optional[Tuple[float, float]] = None
        self._pose: Optional[Tuple[float, float]] = None
        self._last_raw = Twist()
        self._have_raw = False
        self._arrival_hold = False
        self._mission_abort_hold = False
        self._ext_hold_wanted = False
        self._ext_hold_t = 0.0
        self._last_hold_log = 0.0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_out_topic").value), 10
        )
        self._arrived_pub = self.create_publisher(
            Bool, str(self.get_parameter("arrived_topic").value), 10
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_in_topic").value),
            self._on_raw,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            qos_sensor,
        )
        # 仅接受 hybrid 校验通过后的 /long_nav/goal（不再直接吃 /goal_pose）
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self._on_accepted_goal,
            qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("hold_topic").value),
            self._on_ext_hold,
            10,
        )
        self.create_subscription(Bool, "/long_nav/cancel_nav", self._on_cancel_nav, 10)
        self.create_subscription(String, "/long_nav/nav_abort", self._on_nav_abort, 10)
        self.create_subscription(String, "/long_nav/nav_success", self._on_nav_success, 10)

        period = 1.0 / max(float(self.get_parameter("gate_hz").value), 10.0)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "nav_cmd_gate: %s → %s | stop @ goal≤%.2fm | accepted goals only"
            % (
                self.get_parameter("cmd_vel_in_topic").value,
                self.get_parameter("cmd_vel_out_topic").value,
                float(self.get_parameter("goal_tol_m").value),
            )
        )

    def _on_raw(self, msg: Twist):
        self._last_raw = msg
        self._have_raw = True

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (float(p.x), float(p.y))

    def _on_accepted_goal(self, msg: PoseStamped):
        self._goal = (float(msg.pose.position.x), float(msg.pose.position.y))
        was = self._arrival_hold or self._ext_hold_wanted or self._mission_abort_hold
        self._arrival_hold = False
        self._mission_abort_hold = False
        self._ext_hold_wanted = False
        self._ext_hold_t = 0.0
        self._arrived_pub.publish(Bool(data=False))
        self.get_logger().info(
            "accepted Goal (%.1f,%.1f) — mission %s"
            % (
                self._goal[0],
                self._goal[1],
                "restarted (holds released)" if was else "started",
            )
        )

    def _on_nav_success(self, msg: String):
        # hybrid 已到点：锁轮，清目标，避免继续蹭
        self._goal = None
        self._arrival_hold = True
        self._mission_abort_hold = False
        self._pub.publish(Twist())
        self._arrived_pub.publish(Bool(data=True))
        self.get_logger().info(
            ">>> 本次导航任务完成 SUCCESS — %s" % (msg.data or "到点停航")
        )

    def _on_cancel_nav(self, msg: Bool):
        if not bool(msg.data):
            return
        self._goal = None
        self._mission_abort_hold = True
        self._arrival_hold = False
        self._pub.publish(Twist())
        self.get_logger().error("cancel_nav — 本次导航任务废除，锁轮直到新 Goal Pose")

    def _on_nav_abort(self, msg: String):
        self._goal = None
        self._mission_abort_hold = True
        self._arrival_hold = False
        self._pub.publish(Twist())
        self.get_logger().error("nav_abort: %s" % (msg.data or ""))

    def _on_ext_hold(self, msg: Bool):
        if bool(msg.data):
            self._ext_hold_wanted = True
            self._ext_hold_t = time.time()
        else:
            if self._ext_hold_wanted:
                self.get_logger().info("ext cmd_hold OFF (rollover recover done)")
            self._ext_hold_wanted = False
            self._ext_hold_t = 0.0

    def _ext_holding(self) -> bool:
        if not self._ext_hold_wanted:
            return False
        stale = float(self.get_parameter("ext_hold_stale_sec").value)
        if (time.time() - self._ext_hold_t) > stale:
            self.get_logger().warn(
                "ext cmd_hold stale >%.2fs — auto unlock (was stuck after respawn?)"
                % stale
            )
            self._ext_hold_wanted = False
            return False
        return True

    @property
    def _holding(self) -> bool:
        return self._arrival_hold or self._ext_holding() or self._mission_abort_hold

    def _tick(self):
        tol = float(self.get_parameter("goal_tol_m").value)
        if (
            not self._arrival_hold
            and not self._mission_abort_hold
            and self._goal is not None
            and self._pose is not None
        ):
            d = math.hypot(self._pose[0] - self._goal[0], self._pose[1] - self._goal[1])
            if d <= tol:
                self._arrival_hold = True
                self._goal = None
                self._arrived_pub.publish(Bool(data=True))
                self._pub.publish(Twist())
                self.get_logger().info(
                    ">>> 本次导航任务完成 SUCCESS (d=%.2fm ≤ %.2fm) — 锁轮，请重新 Set Goal"
                    % (d, tol)
                )

        if self._holding:
            now = time.time()
            if now - self._last_hold_log > 2.0:
                self._last_hold_log = now
                self.get_logger().info(
                    "holding cmd_vel=0 (arrival=%s ext=%s abort=%s)"
                    % (
                        self._arrival_hold,
                        self._ext_holding(),
                        self._mission_abort_hold,
                    )
                )
            self._pub.publish(Twist())
            return

        if self._have_raw:
            self._pub.publish(self._last_raw)


def main(args=None):
    rclpy.init(args=args)
    node = NavCmdGateNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
