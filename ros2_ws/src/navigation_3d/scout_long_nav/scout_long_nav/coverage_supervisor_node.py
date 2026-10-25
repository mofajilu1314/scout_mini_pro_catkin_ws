#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Auto coverage mapping supervisor (frozen TD3 — no training).

State machine: IDLE → PLACE → DRIVE → END → PLACE …
- ROI hit grid over [xy_min, xy_max]^2 (default ±35 m inside 80×80 arena, ~1 m cells)
- Biased start/goal toward lowest-hit cells; force argmin every N episodes
- End episode on stuck / rollover / timeout / success; teleport next pair
- Publish /long_nav/coverage (JSON String); save costmap when min_hit >= target
"""

from __future__ import annotations

import json
import math
import random
import time
from enum import Enum
from typing import Optional, Tuple

import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Phase(Enum):
    IDLE = 0
    PLACE = 1
    SETTLE = 2
    DRIVE = 3
    END = 4
    DONE = 5


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _quat_to_rpy(q: Quaternion) -> Tuple[float, float, float]:
    # roll (x), pitch (y), yaw (z)
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class CoverageSupervisorNode(Node):
    def __init__(self):
        super().__init__("coverage_supervisor_node")
        self._cg = ReentrantCallbackGroup()

        # ROI / grid
        self.declare_parameter("xy_min", -35.0)
        self.declare_parameter("xy_max", 35.0)
        self.declare_parameter("cell_m", 1.0)
        self.declare_parameter("target_hits", 2)
        self.declare_parameter("min_sep_m", 12.0)
        self.declare_parameter("force_gap_every_n", 5)

        # Episode termination (align R8 training)
        self.declare_parameter("stuck_time_sec", 5.6)
        self.declare_parameter("stuck_dist_m", 0.25)
        self.declare_parameter("stuck_near_goal_rho", 0.7)
        self.declare_parameter("goal_tol_m", 0.8)
        self.declare_parameter("timeout_sec", 75.0)
        self.declare_parameter("rollover_roll_deg", 60.0)
        self.declare_parameter("rollover_pitch_deg", 60.0)
        self.declare_parameter("rollover_sum_deg", 80.0)
        self.declare_parameter("rollover_grace_sec", 2.0)

        # Placement
        self.declare_parameter("model_name", "scout_mini")
        self.declare_parameter("spawn_z", 0.15)
        self.declare_parameter("settle_sec", 1.5)
        self.declare_parameter("start_delay_sec", 10.0)

        # Topics / services
        self.declare_parameter("odom_topic", "/long_nav/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("costmap_topic", "/long_nav/costmap")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("train_goal_topic", "/offroad/train_goal")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("coverage_topic", "/long_nav/coverage")
        self.declare_parameter("save_service", "/save_costmap_node/save")
        self.declare_parameter("goal_repub_hz", 0.5)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("log_every_sec", 5.0)
        self.declare_parameter("save_every_episodes", 10)
        self.declare_parameter("shutdown_when_done", True)

        xmin = float(self.get_parameter("xy_min").value)
        xmax = float(self.get_parameter("xy_max").value)
        cell = float(self.get_parameter("cell_m").value)
        if xmax <= xmin or cell <= 0.0:
            raise ValueError("invalid ROI / cell_m")
        self._xmin = xmin
        self._xmax = xmax
        self._cell = cell
        self._n = int(math.ceil((xmax - xmin) / cell))
        self._hits = np.zeros((self._n, self._n), dtype=np.int32)
        self._known_prev: Optional[np.ndarray] = None

        self._target_hits = int(self.get_parameter("target_hits").value)
        self._min_sep = float(self.get_parameter("min_sep_m").value)
        self._force_n = max(1, int(self.get_parameter("force_gap_every_n").value))

        self._phase = Phase.IDLE
        self._episodes = 0
        self._oks = 0
        self._fails = 0
        self._pass = 1
        self._t0_wall = time.time()
        self._phase_t0 = 0.0
        self._drive_t0 = 0.0
        self._grace_t0 = 0.0
        self._last_log = 0.0
        self._last_goal_pub = 0.0

        self._pose: Optional[Tuple[float, float, float]] = None  # x,y,yaw
        self._imu_rpy: Optional[Tuple[float, float, float]] = None
        self._goal: Optional[Tuple[float, float]] = None
        self._start: Optional[Tuple[float, float]] = None
        self._stuck_anchor: Optional[Tuple[float, float]] = None
        self._stuck_t0 = 0.0
        self._teleport_pending = False
        self._end_reason = ""

        odom_t = str(self.get_parameter("odom_topic").value)
        imu_t = str(self.get_parameter("imu_topic").value)
        cmap_t = str(self.get_parameter("costmap_topic").value)
        goal_t = str(self.get_parameter("goal_topic").value)
        train_t = str(self.get_parameter("train_goal_topic").value)
        cmd_t = str(self.get_parameter("cmd_vel_topic").value)
        cov_t = str(self.get_parameter("coverage_topic").value)

        self._goal_pub = self.create_publisher(PoseStamped, goal_t, 10)
        self._train_goal_pub = self.create_publisher(Point, train_t, 10)
        self._cmd_pub = self.create_publisher(Twist, cmd_t, 10)
        self._cov_pub = self.create_publisher(String, cov_t, 10)

        self.create_subscription(Odometry, odom_t, self._on_odom, 20, callback_group=self._cg)
        self.create_subscription(Imu, imu_t, self._on_imu, 20, callback_group=self._cg)
        self.create_subscription(
            OccupancyGrid, cmap_t, self._on_costmap, 1, callback_group=self._cg
        )

        self._set_cli = self.create_client(
            SetEntityState, "/set_entity_state", callback_group=self._cg
        )
        save_srv = str(self.get_parameter("save_service").value)
        self._save_cli = self.create_client(Trigger, save_srv, callback_group=self._cg)

        tick = float(self.get_parameter("tick_hz").value)
        self.create_timer(1.0 / max(1.0, tick), self._tick, callback_group=self._cg)

        self.get_logger().info(
            "coverage_supervisor: ROI [%.1f,%.1f]^2 cell=%.2fm n=%d target_hits=%d "
            "(no training; frozen TD3 via /goal_pose)"
            % (xmin, xmax, cell, self._n, self._target_hits)
        )

    # ---- sensors ----
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        r, pitch, yaw = _quat_to_rpy(msg.pose.pose.orientation)
        self._pose = (float(p.x), float(p.y), yaw)
        # Prefer IMU for rollover; fall back to odom attitude
        if self._imu_rpy is None:
            self._imu_rpy = (r, pitch, yaw)
        self._mark_robot_hit(float(p.x), float(p.y))

    def _on_imu(self, msg: Imu):
        self._imu_rpy = _quat_to_rpy(msg.orientation)

    def _on_costmap(self, msg: OccupancyGrid):
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        known = data >= 0
        if self._known_prev is None or self._known_prev.shape != known.shape:
            newly = known
        else:
            newly = known & (~self._known_prev)
            # Also count re-observed known cells that stay known: sample every callback
            # via robot path; here only promote unknown→known and bump if already known
            # when robot is nearby (handled in _mark_robot_hit). Extra: any newly known.
        self._known_prev = known
        ys, xs = np.where(newly)
        for iy, ix in zip(ys.tolist(), xs.tolist()):
            wx = ox + (ix + 0.5) * res
            wy = oy + (iy + 0.5) * res
            self._bump_world(wx, wy, 1)

    def _ij(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if x < self._xmin or x >= self._xmax or y < self._xmin or y >= self._xmax:
            return None
        i = int((x - self._xmin) / self._cell)
        j = int((y - self._xmin) / self._cell)
        if i < 0 or j < 0 or i >= self._n or j >= self._n:
            return None
        return i, j

    def _bump_world(self, x: float, y: float, amt: int = 1):
        ij = self._ij(x, y)
        if ij is None:
            return
        i, j = ij
        self._hits[j, i] = int(self._hits[j, i]) + amt

    def _mark_robot_hit(self, x: float, y: float):
        # Increment at most once per cell entry per episode via sticky last cell
        ij = self._ij(x, y)
        if ij is None:
            return
        i, j = ij
        key = (i, j)
        if getattr(self, "_last_cell", None) != key:
            self._last_cell = key
            self._hits[j, i] = int(self._hits[j, i]) + 1

    # ---- sampling ----
    def _cell_center(self, i: int, j: int) -> Tuple[float, float]:
        x = self._xmin + (i + 0.5) * self._cell
        y = self._xmin + (j + 0.5) * self._cell
        return x, y

    def _sample_biased(self, force_min: bool) -> Tuple[float, float]:
        h = self._hits
        vmin = int(h.min())
        if force_min:
            js, is_ = np.where(h == vmin)
            k = int(random.randrange(len(js)))
            return self._cell_center(int(is_[k]), int(js[k]))
        # Softmax-like: weight = (vmax - h + 1)^2
        vmax = int(h.max())
        w = (vmax - h.astype(np.float64) + 1.0) ** 2
        flat = w.reshape(-1)
        flat = flat / flat.sum()
        idx = int(np.random.choice(flat.size, p=flat))
        j = idx // self._n
        i = idx % self._n
        # jitter inside cell
        x, y = self._cell_center(i, j)
        half = 0.35 * self._cell
        x += random.uniform(-half, half)
        y += random.uniform(-half, half)
        x = max(self._xmin + 0.3, min(self._xmax - 0.3, x))
        y = max(self._xmin + 0.3, min(self._xmax - 0.3, y))
        return x, y

    def _sample_pair(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        force = (self._episodes % self._force_n) == 0
        for _ in range(40):
            sx, sy = self._sample_biased(force_min=force)
            gx, gy = self._sample_biased(force_min=True if force else False)
            if math.hypot(gx - sx, gy - sy) >= self._min_sep:
                return (sx, sy), (gx, gy)
        # fallback: opposite corners of low region
        sx, sy = self._sample_biased(True)
        gx = max(self._xmin + 0.5, min(self._xmax - 0.5, -sx))
        gy = max(self._xmin + 0.5, min(self._xmax - 0.5, -sy))
        if math.hypot(gx - sx, gy - sy) < self._min_sep:
            gx = min(self._xmax - 0.5, sx + self._min_sep)
            gy = sy
        return (sx, sy), (gx, gy)

    # ---- actions ----
    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_goal(self):
        if self._goal is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(self._goal[0])
        msg.pose.position.y = float(self._goal[1])
        msg.pose.position.z = 0.0
        if self._pose is not None:
            yaw = math.atan2(self._goal[1] - self._pose[1], self._goal[0] - self._pose[0])
        else:
            yaw = 0.0
        msg.pose.orientation = _yaw_to_quat(yaw)
        self._goal_pub.publish(msg)
        # Direct TD3 goal — do not wait for Hybrid A* (empty map / slow plan)
        pt = Point()
        pt.x = float(self._goal[0])
        pt.y = float(self._goal[1])
        pt.z = 0.0
        self._train_goal_pub.publish(pt)
        self._last_goal_pub = time.time()

    def _teleport(self, x: float, y: float, yaw: float):
        if not self._set_cli.service_is_ready():
            self.get_logger().warn("set_entity_state not ready")
            return
        state = EntityState()
        state.name = str(self.get_parameter("model_name").value)
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(self.get_parameter("spawn_z").value)
        state.pose.orientation = _yaw_to_quat(yaw)
        state.twist = Twist()
        state.reference_frame = "world"
        req = SetEntityState.Request()
        req.state = state
        self._teleport_pending = True

        def _done(fut):
            self._teleport_pending = False
            try:
                resp = fut.result()
                if not resp or not resp.success:
                    self.get_logger().warn("set_entity_state failed")
            except Exception as exc:
                self.get_logger().warn("set_entity_state error: %s" % exc)

        fut = self._set_cli.call_async(req)
        fut.add_done_callback(_done)

    def _save_map(self):
        if not self._save_cli.service_is_ready():
            self.get_logger().warn("save service not ready")
            return
        fut = self._save_cli.call_async(Trigger.Request())

        def _done(f):
            try:
                r = f.result()
                if r and r.success:
                    self.get_logger().info("saved: %s" % r.message)
                else:
                    self.get_logger().warn("save failed: %s" % (r.message if r else "?"))
            except Exception as exc:
                self.get_logger().warn("save error: %s" % exc)

        fut.add_done_callback(_done)

    def _coverage_stats(self):
        h = self._hits
        need = self._pass  # pass1 need≥1, pass2 need≥2 … but target_hits is final
        frac1 = float(np.mean(h >= 1))
        frac2 = float(np.mean(h >= 2))
        return {
            "pass": int(self._pass),
            "target_hits": int(self._target_hits),
            "min_hit": int(h.min()),
            "mean_hit": float(h.mean()),
            "frac_ge_1": frac1,
            "frac_ge_2": frac2,
            "frac_ge_pass": float(np.mean(h >= need)),
            "episodes": int(self._episodes),
            "oks": int(self._oks),
            "fails": int(self._fails),
            "phase": self._phase.name,
            "reason": self._end_reason,
            "start": list(self._start) if self._start else None,
            "goal": list(self._goal) if self._goal else None,
            "elapsed_sec": round(time.time() - self._t0_wall, 1),
        }

    def _publish_coverage(self, force_log: bool = False):
        st = self._coverage_stats()
        msg = String()
        msg.data = json.dumps(st, ensure_ascii=False)
        self._cov_pub.publish(msg)
        now = time.time()
        every = float(self.get_parameter("log_every_sec").value)
        if force_log or (now - self._last_log) >= every:
            self._last_log = now
            self.get_logger().info(
                "coverage pass=%d min=%d mean=%.2f ge1=%.1f%% ge2=%.1f%% ep=%d ok=%d fail=%d [%s]"
                % (
                    st["pass"],
                    st["min_hit"],
                    st["mean_hit"],
                    100.0 * st["frac_ge_1"],
                    100.0 * st["frac_ge_2"],
                    st["episodes"],
                    st["oks"],
                    st["fails"],
                    st["phase"],
                )
            )

    def _update_pass(self):
        mh = int(self._hits.min())
        if self._pass == 1 and mh >= 1:
            self._pass = 2
            self.get_logger().info("=== Pass1 complete (min_hit≥1) → Pass2 toward 200%% ===")
        if mh >= self._target_hits:
            return True
        return False

    # ---- termination checks ----
    def _check_success(self) -> bool:
        if self._pose is None or self._goal is None:
            return False
        rho = math.hypot(self._pose[0] - self._goal[0], self._pose[1] - self._goal[1])
        return rho < float(self.get_parameter("goal_tol_m").value)

    def _check_stuck(self) -> bool:
        if self._pose is None or self._goal is None:
            return False
        rho = math.hypot(self._pose[0] - self._goal[0], self._pose[1] - self._goal[1])
        if rho < float(self.get_parameter("stuck_near_goal_rho").value):
            self._stuck_anchor = (self._pose[0], self._pose[1])
            self._stuck_t0 = time.time()
            return False
        if self._stuck_anchor is None:
            self._stuck_anchor = (self._pose[0], self._pose[1])
            self._stuck_t0 = time.time()
            return False
        dist = math.hypot(
            self._pose[0] - self._stuck_anchor[0], self._pose[1] - self._stuck_anchor[1]
        )
        thr = float(self.get_parameter("stuck_dist_m").value)
        win = float(self.get_parameter("stuck_time_sec").value)
        if dist >= thr:
            self._stuck_anchor = (self._pose[0], self._pose[1])
            self._stuck_t0 = time.time()
            return False
        return (time.time() - self._stuck_t0) >= win

    def _check_rollover(self) -> bool:
        if self._imu_rpy is None:
            return False
        roll, pitch, _ = self._imu_rpy
        rd = abs(math.degrees(roll))
        pd = abs(math.degrees(pitch))
        if (
            rd < float(self.get_parameter("rollover_roll_deg").value)
            and pd < float(self.get_parameter("rollover_pitch_deg").value)
            and (rd + pd) < float(self.get_parameter("rollover_sum_deg").value)
        ):
            return False
        grace = float(self.get_parameter("rollover_grace_sec").value)
        # grace from episode start
        if (time.time() - self._grace_t0) < grace:
            return False
        return True

    def _check_timeout(self) -> bool:
        return (time.time() - self._drive_t0) >= float(
            self.get_parameter("timeout_sec").value
        )

    # ---- state machine ----
    def _tick(self):
        if self._phase == Phase.DONE:
            return

        delay = float(self.get_parameter("start_delay_sec").value)
        if self._phase == Phase.IDLE:
            if (time.time() - self._t0_wall) < delay:
                return
            if not self._set_cli.service_is_ready():
                return
            self._phase = Phase.PLACE
            self.get_logger().info("coverage_supervisor: starting auto coverage loop")

        if self._phase == Phase.PLACE:
            self._stop()
            self._start, self._goal = self._sample_pair()
            yaw = math.atan2(
                self._goal[1] - self._start[1], self._goal[0] - self._start[0]
            )
            self._last_cell = None
            self._stuck_anchor = None
            self._end_reason = ""
            self._teleport(self._start[0], self._start[1], yaw)
            self._phase = Phase.SETTLE
            self._phase_t0 = time.time()
            self._grace_t0 = time.time()
            self.get_logger().info(
                "ep#%d PLACE start=(%.1f,%.1f) goal=(%.1f,%.1f)"
                % (
                    self._episodes + 1,
                    self._start[0],
                    self._start[1],
                    self._goal[0],
                    self._goal[1],
                )
            )
            return

        if self._phase == Phase.SETTLE:
            self._stop()
            if (time.time() - self._phase_t0) < float(
                self.get_parameter("settle_sec").value
            ):
                return
            if self._teleport_pending:
                return
            self._publish_goal()
            self._phase = Phase.DRIVE
            self._drive_t0 = time.time()
            self._grace_t0 = time.time()
            self._stuck_anchor = None
            return

        if self._phase == Phase.DRIVE:
            # re-publish goal periodically
            repub = 1.0 / max(0.1, float(self.get_parameter("goal_repub_hz").value))
            if (time.time() - self._last_goal_pub) >= repub:
                self._publish_goal()

            reason = None
            if self._check_success():
                reason = "success"
            elif self._check_rollover():
                reason = "rollover"
            elif self._check_stuck():
                reason = "stuck"
            elif self._check_timeout():
                reason = "timeout"

            self._publish_coverage()

            if reason is None:
                return

            self._end_reason = reason
            self._stop()
            self._episodes += 1
            if reason == "success":
                self._oks += 1
            else:
                self._fails += 1
            self.get_logger().info(
                "ep#%d END reason=%s" % (self._episodes, reason)
            )
            every = int(self.get_parameter("save_every_episodes").value)
            if every > 0 and (self._episodes % every) == 0:
                self._save_map()

            if self._update_pass():
                self._phase = Phase.DONE
                self._publish_coverage(force_log=True)
                self._save_map()
                self.get_logger().info(
                    "=== Coverage COMPLETE (min_hit≥%d, 200%%) — map saved ==="
                    % self._target_hits
                )
                self._stop()
                if bool(self.get_parameter("shutdown_when_done").value):
                    self.create_timer(0.5, self._request_shutdown)
                return

            self._phase = Phase.PLACE
            return

        if self._phase == Phase.END:
            self._phase = Phase.PLACE

    def _request_shutdown(self):
        self.get_logger().info("coverage done — shutting down supervisor node")
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageSupervisorNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
