#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Load TD3 actor and publish /cmd_vel (stage-A inference, ROS2 Humble)."""

from __future__ import print_function

import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty

from scout_offroad_3d.td3 import TD3


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _roll_pitch_rad_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def _as_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    return str(v).strip().lower() in ("1", "true", "yes", "on")


class InferNode(Node):
    def __init__(self):
        super().__init__("infer_td3_node")
        self.declare_parameter("weights_prefix", "")
        self.declare_parameter("n_sectors", 36)
        self.declare_parameter("n_elevation_bins_td3", 4)
        self.declare_parameter("use_imu_in_state", True)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("hidden_dim", 1024)
        self.declare_parameter("max_range", 5.0)
        self.declare_parameter("max_v", 2.3)
        self.declare_parameter("max_w", 1.8)
        self.declare_parameter("pose_source", "gazebo")
        self.declare_parameter("model_name", "scout_mini")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("use_wall_rate", True)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("min_forward_v", 0.0)
        self.declare_parameter("min_forward_rho", 0.6)
        self.declare_parameter("auto_goal", True)
        self.declare_parameter("auto_goal_timeout_sec", 2.0)
        self.declare_parameter("auto_goal_x", 3.0)
        self.declare_parameter("auto_goal_y", 2.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("td3_features_topic", "/offroad/td3_range_features")
        self.declare_parameter("odom_topic", "/odom")
        # Stage B/C: Arena may publish /offroad/local_goal via bridge → still Point on this topic.
        self.declare_parameter("goal_topic", "/offroad/train_goal")

        weights = str(self.get_parameter("weights_prefix").value).strip()
        if not weights:
            raise ValueError("weights_prefix required (path prefix td3, not td3_actor)")
        if weights.endswith("_actor"):
            self.get_logger().warn(
                "weights_prefix ends with _actor; stripping suffix (use .../td3 not .../td3_actor)"
            )
            weights = weights[: -len("_actor")]

        self.n_sectors = int(self.get_parameter("n_sectors").value)
        self.n_elevation_bins_td3 = int(self.get_parameter("n_elevation_bins_td3").value)
        self.use_imu_in_state = _as_bool(self.get_parameter("use_imu_in_state").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.max_v = float(self.get_parameter("max_v").value)
        self.max_w = float(self.get_parameter("max_w").value)
        self.pose_source = str(self.get_parameter("pose_source").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.rate_hz = float(self.get_parameter("rate").value)
        self.use_wall_rate = _as_bool(self.get_parameter("use_wall_rate").value)
        self.allow_reverse = _as_bool(self.get_parameter("allow_reverse").value)
        self.min_forward_v = float(self.get_parameter("min_forward_v").value)
        self.min_forward_rho = float(self.get_parameter("min_forward_rho").value)
        self.auto_goal = _as_bool(self.get_parameter("auto_goal").value)
        self.auto_goal_timeout_sec = float(self.get_parameter("auto_goal_timeout_sec").value)
        self.auto_goal_x = float(self.get_parameter("auto_goal_x").value)
        self.auto_goal_y = float(self.get_parameter("auto_goal_y").value)

        self._t0_wall = time.time()
        self._auto_goal_applied = False
        self._roll = 0.0
        self._pitch = 0.0

        state_dim = (
            self.n_elevation_bins_td3 * self.n_sectors
            + 4
            + (2 if self.use_imu_in_state else 0)
        )
        hidden_dim = int(self.get_parameter("hidden_dim").value)
        max_action = np.array([self.max_v, self.max_w], dtype=np.float32)
        self.policy = TD3(state_dim, 2, max_action, hidden_dim=hidden_dim)
        self.policy.load(weights)
        self.get_logger().info(
            "TD3 load state_dim=%d hidden_dim=%d imu_in_state=%s weights=%s"
            % (state_dim, hidden_dim, self.use_imu_in_state, weights)
        )
        self._td3_feature_count = self.n_elevation_bins_td3 * self.n_sectors
        self._features = np.full(self._td3_feature_count, self.max_range, dtype=np.float32)
        self._goal = None
        self._odom = None
        self._entity_pose = None
        self._prev = np.zeros(2, dtype=np.float32)
        self._warned_no_goal = False
        self._warned_no_pose = False
        self._cmd_count = 0

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        feat_topic = str(self.get_parameter("td3_features_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value).strip() or "/offroad/train_goal"
        qos_cmd = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Twist, cmd_topic, qos_cmd)
        self._goal_pub = self.create_publisher(Point, self.goal_topic, qos_cmd)
        self.create_subscription(Float32MultiArray, feat_topic, self._on_feat, qos_sensor)
        self.create_subscription(Point, self.goal_topic, self._on_goal, qos_cmd)
        if self.use_imu_in_state:
            imu_topic = str(self.get_parameter("imu_topic").value).strip() or "/imu/data"
            self.create_subscription(Imu, imu_topic, self._on_imu, qos_sensor)

        if self.pose_source == "odom":
            odom_topic = str(self.get_parameter("odom_topic").value)
            self.create_subscription(Odometry, odom_topic, self._on_odom, qos_sensor)
        else:
            self._get_state = self.create_client(GetEntityState, "/get_entity_state")
            self._try_unpause()

        self.get_logger().info(
            "infer_td3 loaded %s_* (auto_goal=%s timeout=%.1fs wall_rate=%s)"
            % (weights, self.auto_goal, self.auto_goal_timeout_sec, self.use_wall_rate)
        )
        if self.auto_goal:
            self.get_logger().warn(
                "If no /offroad/train_goal in %.1fs, will auto-set (%.1f, %.1f)"
                % (self.auto_goal_timeout_sec, self.auto_goal_x, self.auto_goal_y)
            )

    def _try_unpause(self):
        client = self.create_client(Empty, "/unpause_physics")
        if client.wait_for_service(timeout_sec=5.0):
            req = Empty.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            self.get_logger().info("Called /unpause_physics")

    def _on_feat(self, msg):
        data = np.asarray(msg.data, dtype=np.float32)
        n = self._td3_feature_count
        if data.size >= n:
            self._features = data[:n].copy()
        elif data.size > 0:
            out = np.full(n, self.max_range, dtype=np.float32)
            out[: data.size] = data
            self._features = out

    def _on_goal(self, msg):
        self._goal = np.array([msg.x, msg.y], dtype=np.float64)
        self._warned_no_goal = False
        self.get_logger().info("Goal set to (%.2f, %.2f)" % (msg.x, msg.y))

    def _on_odom(self, msg):
        self._odom = msg.pose.pose

    def _on_imu(self, msg):
        self._roll, self._pitch = _roll_pitch_rad_from_quat(msg.orientation)

    def _fetch_entity_pose(self):
        if not self._get_state.wait_for_service(timeout_sec=0.0):
            return None
        req = GetEntityState.Request()
        req.name = self.model_name
        req.reference_frame = "world"
        future = self._get_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return None
        resp = future.result()
        if not resp.success:
            return None
        return resp.state.pose

    def _maybe_auto_goal(self):
        if self._goal is not None or not self.auto_goal or self._auto_goal_applied:
            return
        if time.time() - self._t0_wall < self.auto_goal_timeout_sec:
            return
        self._goal = np.array([self.auto_goal_x, self.auto_goal_y], dtype=np.float64)
        self._auto_goal_applied = True
        self._warned_no_goal = False
        pt = Point(x=self.auto_goal_x, y=self.auto_goal_y, z=0.0)
        self._goal_pub.publish(pt)
        self.get_logger().warn(
            "No goal received; auto-set %s to (%.1f, %.1f)"
            % (self.goal_topic, self.auto_goal_x, self.auto_goal_y)
        )

    def _pose(self):
        if self.pose_source == "odom":
            if self._odom is None:
                return None
            return (
                self._odom.position.x,
                self._odom.position.y,
                _yaw_from_quat(self._odom.orientation),
            )
        pose = self._fetch_entity_pose()
        if pose is None:
            return None
        return pose.position.x, pose.position.y, _yaw_from_quat(pose.orientation)

    def _sleep(self):
        # Always pump DDS callbacks (goal/odom/features). use_wall_rate only
        # chooses wall-clock pacing vs single spin_once wait.
        period = max(1.0 / max(self.rate_hz, 1.0), 0.01)
        if self.use_wall_rate:
            t0 = time.time()
            while time.time() - t0 < period:
                rclpy.spin_once(self, timeout_sec=0.01)
        else:
            rclpy.spin_once(self, timeout_sec=period)

    def spin(self):
        while rclpy.ok():
            self._maybe_auto_goal()
            if self._goal is None:
                if not self._warned_no_goal:
                    self.get_logger().warn(
                        "Waiting for %s (geometry_msgs/Point x,y); "
                        "no cmd_vel until a goal is received" % self.goal_topic
                    )
                    self._warned_no_goal = True
                self._sleep()
                continue
            pose = self._pose()
            if pose is None:
                if not self._warned_no_pose:
                    self.get_logger().warn(
                        "Waiting for robot pose (model=%s via %s)"
                        % (self.model_name, self.pose_source)
                    )
                    self._warned_no_pose = True
                self._sleep()
                continue
            self._warned_no_pose = False
            x, y, yaw = pose
            dx = self._goal[0] - x
            dy = self._goal[1] - y
            rho = math.hypot(dx, dy)
            heading = math.atan2(dy, dx)
            theta = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
            d_norm = np.clip(self._features / self.max_range, 0.0, 1.0)
            tail = [
                min(rho / 10.0, 1.0),
                theta / math.pi,
                self._prev[0] / self.max_v,
                self._prev[1] / self.max_w,
            ]
            if self.use_imu_in_state:
                tail.extend([self._roll / math.pi, self._pitch / math.pi])
            state = np.concatenate(
                [d_norm, np.array(tail, dtype=np.float32)]
            ).astype(np.float32)
            action = self.policy.select_action(state)
            cmd = Twist()
            v_lo = -self.max_v if self.allow_reverse else 0.0
            v = float(np.clip(action[0], v_lo, self.max_v))
            if (not self.allow_reverse) and rho > self.min_forward_rho and v < self.min_forward_v:
                v = self.min_forward_v
            cmd.linear.x = v
            cmd.angular.z = float(np.clip(action[1], -self.max_w, self.max_w))
            self._prev = np.array([cmd.linear.x, cmd.angular.z], dtype=np.float32)
            self._pub.publish(cmd)
            self._cmd_count += 1
            if self._cmd_count == 1 or self._cmd_count % 20 == 0:
                self.get_logger().info(
                    "cmd_vel #%d v=%.2f w=%.2f pose=(%.2f,%.2f) rho=%.2f"
                    % (self._cmd_count, cmd.linear.x, cmd.angular.z, x, y, rho)
                )
            self._sleep()


def main():
    rclpy.init()
    node = InferNode()
    try:
        node.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
