#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Load TD3 actor and publish /cmd_vel (stage-A inference)."""

from __future__ import print_function

import math
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.srv import GetModelState, GetPhysicsProperties, SetPhysicsProperties
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from td3 import TD3


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _as_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    return str(v).strip().lower() in ("1", "true", "yes", "on")


class InferNode(object):
    def __init__(self):
        weights = rospy.get_param("~weights_prefix", "").strip()
        if not weights:
            raise ValueError("~weights_prefix required (path prefix td3, not td3_actor)")
        if weights.endswith("_actor"):
            rospy.logwarn(
                "weights_prefix ends with _actor; stripping suffix (use .../td3 not .../td3_actor)"
            )
            weights = weights[: -len("_actor")]

        self.n_sectors = int(rospy.get_param("~n_sectors", 24))
        self.n_elevation_bins_td3 = int(rospy.get_param("~n_elevation_bins_td3", 3))
        self.max_range = float(rospy.get_param("~max_range", 5.0))
        self.max_v = float(rospy.get_param("~max_v", 0.6))
        self.max_w = float(rospy.get_param("~max_w", 1.0))
        self.pose_source = rospy.get_param("~pose_source", "gazebo")
        self.model_name = rospy.get_param("~model_name", "scout_mini")
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        # Wall-clock control loop: rospy.Rate blocks forever when /clock stalls (heavy heightmap).
        self.use_wall_rate = _as_bool(rospy.get_param("~use_wall_rate", True))
        # Infer demo: reverse+spin looks stuck on bumpy; prefer forward progress.
        self.allow_reverse = _as_bool(rospy.get_param("~allow_reverse", False))
        self.min_forward_v = float(rospy.get_param("~min_forward_v", 0.18))
        self.min_forward_rho = float(rospy.get_param("~min_forward_rho", 0.6))

        use_sim = _as_bool(rospy.get_param("/use_sim_time", False))
        self.auto_goal = _as_bool(rospy.get_param("~auto_goal", use_sim))
        self.auto_goal_timeout_sec = float(rospy.get_param("~auto_goal_timeout_sec", 2.0))
        self.auto_goal_x = float(rospy.get_param("~auto_goal_x", 3.0))
        self.auto_goal_y = float(rospy.get_param("~auto_goal_y", 2.0))
        self._t0_wall = time.time()
        self._auto_goal_applied = False

        state_dim = self.n_elevation_bins_td3 * self.n_sectors + 4
        max_action = np.array([self.max_v, self.max_w], dtype=np.float32)
        self.policy = TD3(state_dim, 2, max_action)
        self.policy.load(weights)
        self._td3_feature_count = self.n_elevation_bins_td3 * self.n_sectors
        self._features = np.full(self._td3_feature_count, self.max_range, dtype=np.float32)
        self._goal = None
        self._odom = None
        self._model_pose = None
        self._prev = np.zeros(2, dtype=np.float32)
        self._warned_no_goal = False
        self._warned_no_pose = False
        self._cmd_count = 0

        self._pub = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1
        )
        self._goal_pub = rospy.Publisher("/offroad/train_goal", Point, queue_size=1, latch=True)
        rospy.Subscriber(
            rospy.get_param("~td3_features_topic", "/offroad/td3_range_features"),
            Float32MultiArray,
            self._on_feat,
            queue_size=1,
        )
        rospy.Subscriber("/offroad/train_goal", Point, self._on_goal, queue_size=1)
        if self.pose_source == "odom":
            rospy.Subscriber(
                rospy.get_param("~odom_topic", "/odom"), Odometry, self._on_odom, queue_size=1
            )
        else:
            from gazebo_msgs.msg import ModelStates
            self._model_pose = None
            self._get = None
            rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1)
            try:
                rospy.wait_for_service("/gazebo/get_model_state", timeout=30.0)
                self._get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            except Exception as e:
                rospy.logwarn("get_model_state unavailable yet: %s", e)
            self._try_unpause_and_speed_physics()

        rospy.loginfo(
            "infer_td3 loaded %s_* (auto_goal=%s timeout=%.1fs wall_rate=%s)",
            weights,
            self.auto_goal,
            self.auto_goal_timeout_sec,
            self.use_wall_rate,
        )
        if self.auto_goal:
            rospy.logwarn(
                "If no /offroad/train_goal in %.1fs, will auto-set (%.1f, %.1f)",
                self.auto_goal_timeout_sec,
                self.auto_goal_x,
                self.auto_goal_y,
            )

    def _try_unpause_and_speed_physics(self):
        try:
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=5.0)
            rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
            rospy.loginfo("Called /gazebo/unpause_physics")
        except Exception as e:
            rospy.logwarn("unpause_physics failed: %s", e)
        # If heightmap makes RTF near 0, drop update rate so wall time advances sim.
        try:
            rospy.wait_for_service("/gazebo/get_physics_properties", timeout=5.0)
            rospy.wait_for_service("/gazebo/set_physics_properties", timeout=5.0)
            get_phys = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
            set_phys = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
            props = get_phys()
            if not props.success:
                return
            max_rate = float(rospy.get_param("~physics_max_update_rate", 200.0))  # 0 = as fast as possible
            time_step = float(rospy.get_param("~physics_time_step", 0.01))
            if props.max_update_rate > max_rate + 1.0 or props.time_step < time_step - 1e-6:
                ode = props.ode_config
                ok = set_phys(
                    time_step,
                    max_rate,
                    props.gravity,
                    ode,
                )
                rospy.logwarn(
                    "Tuned Gazebo physics for infer: time_step=%.4f max_update_rate=%.1f (was %.4f / %.1f) ok=%s",
                    time_step,
                    max_rate,
                    props.time_step,
                    props.max_update_rate,
                    getattr(ok, "success", ok),
                )
        except Exception as e:
            rospy.logwarn("physics tune skipped: %s", e)

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
        rospy.loginfo_throttle(2.0, "Goal set to (%.2f, %.2f)", msg.x, msg.y)

    def _on_odom(self, msg):
        self._odom = msg.pose.pose

    def _on_model_states(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        self._model_pose = msg.pose[idx]

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
        rospy.logwarn(
            "No goal received; auto-set /offroad/train_goal to (%.1f, %.1f)",
            self.auto_goal_x,
            self.auto_goal_y,
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
        if self._model_pose is not None:
            p = self._model_pose.position
            return p.x, p.y, _yaw_from_quat(self._model_pose.orientation)
        # Do not call get_model_state in the control loop (blocks / spams before spawn).
        return None

    def _sleep(self):
        if self.use_wall_rate:
            time.sleep(max(1.0 / max(self.rate_hz, 1.0), 0.01))
        else:
            rospy.Rate(self.rate_hz).sleep()

    def spin(self):
        while not rospy.is_shutdown():
            self._maybe_auto_goal()
            if self._goal is None:
                if not self._warned_no_goal:
                    rospy.logwarn(
                        "Waiting for /offroad/train_goal (geometry_msgs/Point x,y); "
                        "no cmd_vel until a goal is received"
                    )
                    self._warned_no_goal = True
                self._sleep()
                continue
            pose = self._pose()
            if pose is None:
                if not self._warned_no_pose:
                    rospy.logwarn(
                        "Waiting for robot pose (model=%s via %s)",
                        self.model_name,
                        self.pose_source,
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
            state = np.concatenate(
                [
                    d_norm,
                    np.array(
                        [
                            min(rho / 10.0, 1.0),
                            theta / math.pi,
                            self._prev[0] / self.max_v,
                            self._prev[1] / self.max_w,
                        ],
                        dtype=np.float32,
                    ),
                ]
            ).astype(np.float32)
            action = self.policy.select_action(state)
            cmd = Twist()
            v_lo = -0.15 if self.allow_reverse else 0.0
            v = float(np.clip(action[0], v_lo, self.max_v))
            if (not self.allow_reverse) and rho > self.min_forward_rho and v < self.min_forward_v:
                v = self.min_forward_v
            cmd.linear.x = v
            cmd.angular.z = float(np.clip(action[1], -self.max_w, self.max_w))
            self._prev = np.array([cmd.linear.x, cmd.angular.z], dtype=np.float32)
            self._pub.publish(cmd)
            self._cmd_count += 1
            if self._cmd_count == 1 or self._cmd_count % 20 == 0:
                rospy.loginfo(
                    "cmd_vel #%d v=%.2f w=%.2f pose=(%.2f,%.2f) rho=%.2f",
                    self._cmd_count,
                    cmd.linear.x,
                    cmd.angular.z,
                    x,
                    y,
                    rho,
                )
            self._sleep()


def main():
    rospy.init_node("infer_td3_node")
    InferNode().spin()


if __name__ == "__main__":
    main()
