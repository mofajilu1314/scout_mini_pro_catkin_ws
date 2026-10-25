#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mapping rollover guard (manual long_nav_map).

Primary tilt source: Gazebo /get_entity_state (not only /imu/data).
On flip (same thresholds as TD3 train): mute teleop, pause lidar→costmap,
clear disk, Z-ladder respawn at flip XY (z=0.23 +0.5…), else origin.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray, String


class Phase(Enum):
    MONITOR = 0
    RECOVER = 1


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _quat_to_rpy(q) -> Tuple[float, float, float]:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class RolloverGuardNode(Node):
    def __init__(self):
        super().__init__("rollover_guard_node")
        self._cg = ReentrantCallbackGroup()
        self._srv_cg = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("rollover_roll_deg", 55.0)
        self.declare_parameter("rollover_pitch_deg", 55.0)
        self.declare_parameter("rollover_sum_deg", 75.0)
        self.declare_parameter("rollover_grace_sec", 2.5)
        self.declare_parameter("stable_roll_deg", 35.0)
        self.declare_parameter("stable_pitch_deg", 35.0)
        self.declare_parameter("z0", 0.23)
        self.declare_parameter("z_step", 0.5)
        self.declare_parameter("z_max", 3.23)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("clear_radius_m", 12.0)
        self.declare_parameter("model_name", "scout_mini")
        self.declare_parameter("fallback_xy", [0.0, 0.0])
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/long_nav/odom")
        self.declare_parameter("odom_fallback_topic", "/odom")
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("poll_gz_hz", 5.0)
        self.declare_parameter("cmd_hold_topic", "/long_nav/cmd_hold")
        self.declare_parameter("publish_cmd_hold", True)
        self.declare_parameter("recover_timeout_sec", 20.0)
        self.declare_parameter("hold_false_burst", 8)
        self.declare_parameter("enabled", True)
        self.declare_parameter("nearby_offsets_m", [2.0, 3.5, 5.0])
        self.declare_parameter("goal_topic", "/long_nav/goal")

        self._phase = Phase.MONITOR
        self._grace_t0 = time.time()
        self._pose: Optional[Tuple[float, float, float]] = None
        self._goal_xy: Optional[Tuple[float, float]] = None
        self._recover_mode = "here"  # here | nearby | origin | goal
        self._roll_deg = 0.0
        self._pitch_deg = 0.0
        self._tilt_src = "none"
        self._last_tilt_log = 0.0
        self._gz_poll_pending = False
        self._z_tries: List[float] = []
        self._z_i = 0
        self._target_xy = (0.0, 0.0)
        self._target_yaw = 0.0
        self._using_origin = False
        self._nearby_candidates: List[Tuple[float, float]] = []
        self._nearby_i = 0
        self._settle_until = 0.0
        self._set_pending = False
        self._started = time.time()
        self._recover_t0 = 0.0
        self._stable_ticks = 0

        imu_t = str(self.get_parameter("imu_topic").value)
        odom_t = str(self.get_parameter("odom_topic").value)
        odom_fb = str(self.get_parameter("odom_fallback_topic").value)
        self.create_subscription(Imu, imu_t, self._on_imu, 20, callback_group=self._cg)
        self.create_subscription(Odometry, odom_t, self._on_odom, 20, callback_group=self._cg)
        self.create_subscription(Odometry, odom_fb, self._on_odom, 20, callback_group=self._cg)
        self.create_subscription(
            Bool, "/long_nav/force_recover", self._on_force_recover, 10, callback_group=self._cg
        )
        self.create_subscription(
            String,
            "/long_nav/force_recover_mode",
            self._on_recover_mode,
            10,
            callback_group=self._cg,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self._on_goal,
            10,
            callback_group=self._cg,
        )
        # RViz「2D Pose Estimate」→ 在点击处 Z 阶梯重生，并废除当前导航
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._on_initialpose,
            10,
            callback_group=self._cg,
        )
        self._cancel_nav_pub = self.create_publisher(Bool, "/long_nav/cancel_nav", 10)

        self._mute_pub = self.create_publisher(Bool, "/long_nav/teleop_mute", 10)
        self._pause_pub = self.create_publisher(Bool, "/long_nav/mapping_pause", 10)
        self._clear_pub = self.create_publisher(
            Float32MultiArray, "/long_nav/clear_costmap_radius", 10
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._hold_pub = self.create_publisher(
            Bool, str(self.get_parameter("cmd_hold_topic").value), 10
        )

        self._set_cli = self.create_client(
            SetEntityState, "/set_entity_state", callback_group=self._srv_cg
        )
        self._get_cli = self.create_client(
            GetEntityState, "/get_entity_state", callback_group=self._srv_cg
        )

        tick = float(self.get_parameter("tick_hz").value)
        poll = float(self.get_parameter("poll_gz_hz").value)
        self.create_timer(1.0 / max(tick, 1.0), self._tick, callback_group=self._cg)
        self.create_timer(1.0 / max(poll, 1.0), self._poll_gz, callback_group=self._cg)
        self._set_gates(mute=False, pause=False)

        self.get_logger().info(
            "rollover_guard READY | tilt from Gazebo get_entity_state (+IMU) | "
            "thresh roll/pitch/sum=%.0f/%.0f/%.0f | clear r=%.1fm | Z %.2f+%.2f..%.2f"
            % (
                float(self.get_parameter("rollover_roll_deg").value),
                float(self.get_parameter("rollover_pitch_deg").value),
                float(self.get_parameter("rollover_sum_deg").value),
                float(self.get_parameter("clear_radius_m").value),
                float(self.get_parameter("z0").value),
                float(self.get_parameter("z_step").value),
                float(self.get_parameter("z_max").value),
            )
        )

    def _on_imu(self, msg: Imu):
        # IMU secondary; Gazebo model pose is primary
        r, p, _ = _quat_to_rpy(msg.orientation)
        if self._tilt_src != "gazebo":
            self._roll_deg = abs(math.degrees(r))
            self._pitch_deg = abs(math.degrees(p))
            self._tilt_src = "imu"

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        _, _, yaw = _quat_to_rpy(msg.pose.pose.orientation)
        self._pose = (float(p.x), float(p.y), yaw)

    def _poll_gz(self):
        if self._gz_poll_pending:
            return
        if not self._get_cli.service_is_ready():
            return
        req = GetEntityState.Request()
        req.name = str(self.get_parameter("model_name").value)
        req.reference_frame = "world"
        self._gz_poll_pending = True
        fut = self._get_cli.call_async(req)

        def _done(f):
            self._gz_poll_pending = False
            try:
                resp = f.result()
            except Exception:
                return
            if not resp or not resp.success:
                return
            pose = resp.state.pose
            r, p, yaw = _quat_to_rpy(pose.orientation)
            self._roll_deg = abs(math.degrees(r))
            self._pitch_deg = abs(math.degrees(p))
            self._tilt_src = "gazebo"
            self._pose = (
                float(pose.position.x),
                float(pose.position.y),
                yaw,
            )

        fut.add_done_callback(_done)

    def _set_gates(self, mute: bool, pause: bool):
        self._mute_pub.publish(Bool(data=mute))
        self._pause_pub.publish(Bool(data=pause))
        if bool(self.get_parameter("publish_cmd_hold").value):
            self._hold_pub.publish(Bool(data=mute))

    def _release_cmd_hold(self):
        """Burst hold=false so nav_cmd_gate never stays locked after respawn."""
        if not bool(self.get_parameter("publish_cmd_hold").value):
            self._mute_pub.publish(Bool(data=False))
            self._pause_pub.publish(Bool(data=False))
            return
        n = max(8, int(self.get_parameter("hold_false_burst").value))
        for _ in range(n):
            self._hold_pub.publish(Bool(data=False))
            self._mute_pub.publish(Bool(data=False))
            self._pause_pub.publish(Bool(data=False))
        self.get_logger().info(
            "teleop_mute/cmd_hold OFF burst x%d — 遥控应可立即使用" % n
        )

    def _on_goal(self, msg: PoseStamped):
        self._goal_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _cancel_nav_mission(self):
        self._cancel_nav_pub.publish(Bool(data=True))
        self.get_logger().error("已发 /long_nav/cancel_nav — 本次导航任务废除")

    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        """RViz 2D Pose Estimate：箭头位置 Z 阶梯重生（0.23 起）。"""
        if not bool(self.get_parameter("enabled").value):
            self.get_logger().warn("initialpose ignored — rollover_guard enabled=false")
            return
        p = msg.pose.pose
        x = float(p.position.x)
        y = float(p.position.y)
        _r, _p, yaw = _quat_to_rpy(p.orientation)
        self.get_logger().error(
            ">>> RViz 2D Pose Estimate → Z-ladder spawn at (%.2f,%.2f) yaw=%.2f"
            % (x, y, yaw)
        )
        self._cancel_nav_mission()
        if self._phase == Phase.RECOVER:
            self._target_xy = (x, y)
            self._target_yaw = yaw
            self._using_origin = False
            self._nearby_candidates = []
            self._z_tries = self._z_ladder()
            self._z_i = 0
            self._clear_around(x, y)
            self._place_next_z()
            return
        self._begin_recover_at(x, y, yaw, label="rviz_initialpose")

    def _on_recover_mode(self, msg: String):
        mode = (msg.data or "here").strip().lower()
        if mode not in ("here", "nearby", "origin", "goal"):
            self.get_logger().warn("unknown recover mode %r — use here" % msg.data)
            mode = "here"
        self._recover_mode = mode
        self.get_logger().info("force_recover_mode ← %s" % mode)

    def _on_force_recover(self, msg: Bool):
        """ScoutNav 脱离卡死：按 mode 选重生点（here/nearby/origin/goal）。"""
        if not bool(msg.data):
            return
        if not bool(self.get_parameter("enabled").value):
            self.get_logger().warn("force_recover ignored — rollover_guard enabled=false")
            return
        mode = self._recover_mode
        self.get_logger().error(
            ">>> FORCE RECOVER mode=%s — pause, clear, Z-ladder, unmute" % mode
        )
        self._cancel_nav_mission()
        if self._phase == Phase.RECOVER:
            self._z_tries = self._z_ladder()
            self._z_i = 0
            self._place_next_z()
            return
        self._begin_recover(mode=mode)

    def _resolve_recover_xy(self, mode: str) -> Tuple[float, float, float, str]:
        """Return (x, y, yaw, label)."""
        x, y, yaw = self._pose if self._pose else (0.0, 0.0, 0.0)
        if mode == "origin":
            fb = self.get_parameter("fallback_xy").value
            ox = float(fb[0]) if len(fb) > 0 else 0.0
            oy = float(fb[1]) if len(fb) > 1 else 0.0
            return ox, oy, 0.0, "origin"
        if mode == "goal" and self._goal_xy is not None:
            return self._goal_xy[0], self._goal_xy[1], yaw, "goal"
        if mode == "nearby":
            offsets = list(self.get_parameter("nearby_offsets_m").value) or [
                2.5,
                4.0,
                6.0,
                8.0,
            ]
            dirs = [
                (1, 0),
                (0, 1),
                (-1, 0),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ]
            cands: List[Tuple[float, float]] = []
            for r in offsets:
                for dx, dy in dirs:
                    scale = 0.7071 if (dx != 0 and dy != 0) else 1.0
                    cands.append((x + float(r) * scale * float(dx), y + float(r) * scale * float(dy)))
            self._nearby_candidates = cands
            self._nearby_i = 0
            nx, ny = cands[0]
            return nx, ny, yaw, "nearby[0]"
        if mode == "goal":
            self.get_logger().warn("goal unknown — fallback to nearby")
            return self._resolve_recover_xy("nearby")
        return x, y, yaw, "here"

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _is_rollover(self) -> bool:
        if not bool(self.get_parameter("enabled").value):
            return False
        # boot grace: wait services + first gz pose
        if (time.time() - self._started) < 3.0:
            return False
        if self._tilt_src == "none":
            return False
        rd, pd = abs(self._roll_deg), abs(self._pitch_deg)
        severe = rd >= 90.0 or pd >= 90.0 or (rd + pd) >= 150.0
        # 普通 grace：刚重生后的小晃动不重复触发；严重倒置必须立刻再救
        if (not severe) and (time.time() - self._grace_t0) < float(
            self.get_parameter("rollover_grace_sec").value
        ):
            return False
        if rd >= float(self.get_parameter("rollover_roll_deg").value):
            return True
        if pd >= float(self.get_parameter("rollover_pitch_deg").value):
            return True
        if (rd + pd) >= float(self.get_parameter("rollover_sum_deg").value):
            return True
        return False

    def _is_stable(self) -> bool:
        if self._tilt_src == "none":
            return False
        # 用绝对值：倒置时 roll≈180 不能被当成「稳定」
        return abs(self._roll_deg) < float(
            self.get_parameter("stable_roll_deg").value
        ) and abs(self._pitch_deg) < float(self.get_parameter("stable_pitch_deg").value)

    def _z_ladder(self) -> List[float]:
        z0 = float(self.get_parameter("z0").value)
        step = float(self.get_parameter("z_step").value)
        zmax = float(self.get_parameter("z_max").value)
        out = []
        z = z0
        while z <= zmax + 1e-9:
            out.append(round(z, 3))
            z += step
        return out or [z0]

    def _clear_around(self, x: float, y: float):
        msg = Float32MultiArray()
        msg.data = [
            float(x),
            float(y),
            float(self.get_parameter("clear_radius_m").value),
        ]
        self._clear_pub.publish(msg)

    def _teleport(self, x: float, y: float, yaw: float, z: float):
        if not self._set_cli.service_is_ready():
            self.get_logger().warn("set_entity_state not ready")
            return
        state = EntityState()
        state.name = str(self.get_parameter("model_name").value)
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation = _yaw_to_quat(yaw)
        state.twist = Twist()
        state.reference_frame = "world"
        req = SetEntityState.Request()
        req.state = state
        self._set_pending = True

        def _done(fut):
            self._set_pending = False
            try:
                r = fut.result()
                if not r or not r.success:
                    self.get_logger().warn("set_entity_state failed z=%.2f" % z)
                else:
                    self.get_logger().info(
                        "set_entity_state ok → (%.2f,%.2f,z=%.2f)" % (x, y, z)
                    )
            except Exception as exc:
                self.get_logger().warn("set_entity_state: %s" % exc)

        fut = self._set_cli.call_async(req)
        fut.add_done_callback(_done)

    def _begin_recover_at(
        self, x: float, y: float, yaw: float, *, label: str = "custom"
    ) -> None:
        self.get_logger().error(
            ">>> RECOVER at %s (%.2f,%.2f) yaw=%.2f — Z-ladder from z0"
            % (label, x, y, yaw)
        )
        self._stop()
        self._set_gates(mute=True, pause=True)
        self._clear_around(x, y)
        self._target_xy = (float(x), float(y))
        self._target_yaw = float(yaw)
        self._using_origin = label == "origin"
        self._nearby_candidates = []
        self._nearby_i = 0
        self._z_tries = self._z_ladder()
        self._z_i = 0
        self._recover_t0 = time.time()
        self._stable_ticks = 0
        self._phase = Phase.RECOVER
        self._place_next_z()

    def _begin_recover(self, mode: str = "here"):
        x, y, yaw, label = self._resolve_recover_xy(mode)
        self.get_logger().error(
            ">>> RECOVER start mode=%s → %s (%.2f,%.2f) roll=%.1f° pitch=%.1f° src=%s"
            % (
                mode,
                label,
                x,
                y,
                self._roll_deg,
                self._pitch_deg,
                self._tilt_src,
            )
        )
        self._stop()
        self._set_gates(mute=True, pause=True)
        self._clear_around(x, y)
        self._target_xy = (x, y)
        self._target_yaw = yaw
        self._using_origin = mode == "origin"
        if mode != "nearby":
            self._nearby_candidates = []
            self._nearby_i = 0
        self._z_tries = self._z_ladder()
        self._z_i = 0
        self._recover_t0 = time.time()
        self._stable_ticks = 0
        self._phase = Phase.RECOVER
        self._place_next_z()

    def _place_next_z(self):
        if self._z_i >= len(self._z_tries):
            # nearby: try next offset before falling back to origin
            if self._nearby_candidates and self._nearby_i + 1 < len(self._nearby_candidates):
                self._nearby_i += 1
                nx, ny = self._nearby_candidates[self._nearby_i]
                self.get_logger().warn(
                    "Z-ladder failed → next nearby[%d] (%.2f,%.2f)"
                    % (self._nearby_i, nx, ny)
                )
                self._target_xy = (nx, ny)
                self._clear_around(nx, ny)
                self._z_tries = self._z_ladder()
                self._z_i = 0
            elif not self._using_origin:
                fb = self.get_parameter("fallback_xy").value
                ox = float(fb[0]) if len(fb) > 0 else 0.0
                oy = float(fb[1]) if len(fb) > 1 else 0.0
                self.get_logger().warn(
                    "Z-ladder failed at flip XY → origin (%.1f,%.1f)" % (ox, oy)
                )
                self._target_xy = (ox, oy)
                self._target_yaw = 0.0
                self._using_origin = True
                self._nearby_candidates = []
                self._z_tries = self._z_ladder()
                self._z_i = 0
            else:
                z0 = float(self.get_parameter("z0").value)
                self.get_logger().error(
                    "Z-ladder failed at origin — force upright z=%.2f" % z0
                )
                self._teleport(self._target_xy[0], self._target_xy[1], 0.0, z0)
                self._finish_recover()
                return
        z = self._z_tries[self._z_i]
        self._z_i += 1
        self.get_logger().info(
            "respawn try #%d xy=(%.2f,%.2f) z=%.2f"
            % (self._z_i, self._target_xy[0], self._target_xy[1], z)
        )
        self._teleport(self._target_xy[0], self._target_xy[1], self._target_yaw, z)
        self._settle_until = time.time() + float(self.get_parameter("settle_sec").value)

    def _finish_recover(self):
        if self._pose:
            self._clear_around(self._pose[0], self._pose[1])
        self._stop()
        self._release_cmd_hold()
        self._grace_t0 = time.time()
        self._phase = Phase.MONITOR
        self.get_logger().info(
            "<<< rollover recovery DONE — cmd_hold OFF, nav unlocked (grace %.1fs)"
            % float(self.get_parameter("rollover_grace_sec").value)
        )

    def _tick(self):
        now = time.time()
        # heartbeat: show we see tilt
        if self._tilt_src != "none" and (self._roll_deg + self._pitch_deg) > 20.0:
            if now - self._last_tilt_log > 2.0:
                self._last_tilt_log = now
                self.get_logger().warn(
                    "tilt watch roll=%.1f° pitch=%.1f° sum=%.1f° src=%s phase=%s"
                    % (
                        self._roll_deg,
                        self._pitch_deg,
                        self._roll_deg + self._pitch_deg,
                        self._tilt_src,
                        self._phase.name,
                    )
                )

        if self._phase == Phase.MONITOR:
            if self._is_rollover():
                self._begin_recover(mode="here")
            return

        self._stop()
        self._set_gates(mute=True, pause=True)
        # Hard timeout: never leave nav locked forever after respawn
        timeout = float(self.get_parameter("recover_timeout_sec").value)
        if self._recover_t0 > 0 and (now - self._recover_t0) > timeout:
            self.get_logger().error(
                "recover timeout %.1fs — force upright + unlock nav" % timeout
            )
            z0 = float(self.get_parameter("z0").value)
            xy = self._target_xy
            self._teleport(xy[0], xy[1], self._target_yaw, z0)
            self._finish_recover()
            return
        if self._set_pending:
            return
        if now < self._settle_until:
            return
        if self._is_stable():
            self._stable_ticks += 1
            if self._stable_ticks == 1:
                self.get_logger().info(
                    "stable candidate roll=%.1f° pitch=%.1f° (need 8 ticks)"
                    % (self._roll_deg, self._pitch_deg)
                )
            if self._stable_ticks >= 8:
                self.get_logger().info(
                    "stable after respawn roll=%.1f° pitch=%.1f°"
                    % (self._roll_deg, self._pitch_deg)
                )
                self._finish_recover()
            return
        self._stable_ticks = 0
        self._place_next_z()


def main(args=None):
    rclpy.init(args=args)
    node = RolloverGuardNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._release_cmd_hold()
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
