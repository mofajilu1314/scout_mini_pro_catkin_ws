#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure Pursuit path following node for ROS1 Noetic.
Subscribes: /path (nav_msgs/Path), /amcl_pose (pose in map frame).
Publishes: /scout_mini_pro/cmd_vel (Twist).
Ported from Pure-tracking-slam-automatic-navigation-system (ROS2).
"""

from __future__ import print_function

import math
import numpy as np
from scipy.spatial import KDTree

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitController(object):
    def __init__(self, lookahead_distance=0.5):
        self.lookahead_distance = lookahead_distance

    def calculate_steering_angle(self, vehicle_pose, path_points):
        path_arr = np.asarray(path_points)
        if path_arr.ndim == 1 or path_arr.shape[0] < 2:
            return 0.0, vehicle_pose[:2]
        try:
            closest_point_idx = int(KDTree(path_arr[:, :2]).query(vehicle_pose[:2])[1])
        except Exception:
            closest_point_idx = 0
        closest_point_idx = max(0, min(closest_point_idx, len(path_arr) - 1))

        # 在「到车距离」在 [lookahead_distance, max_lookahead] 的范围内，取沿路径最靠前的点（看得更远，弯道前能看到转弯段）
        # 避免取「第一个>=L 的点」落在路径直线段导致一直直行
        max_lookahead = self.lookahead_distance * 2.5  # 最大约 1.25m @ L=0.5，防止看到终点
        target_point = path_arr[closest_point_idx]
        for i in range(len(path_arr)):
            lookahead_idx = min(closest_point_idx + i, len(path_arr) - 1)
            pt = path_arr[lookahead_idx]
            dx = pt[0] - vehicle_pose[0]
            dy = pt[1] - vehicle_pose[1]
            distance_to_target = math.sqrt(dx*dx + dy*dy)
            if self.lookahead_distance <= distance_to_target <= max_lookahead:
                target_point = pt
        # 若没有点落在范围内，退化为「第一个 >= lookahead_distance」的点
        if (target_point[0] - vehicle_pose[0])**2 + (target_point[1] - vehicle_pose[1])**2 < self.lookahead_distance**2:
            for i in range(len(path_arr)):
                lookahead_idx = min(closest_point_idx + i, len(path_arr) - 1)
                pt = path_arr[lookahead_idx]
                d2 = (pt[0] - vehicle_pose[0])**2 + (pt[1] - vehicle_pose[1])**2
                target_point = pt
                if d2 >= self.lookahead_distance**2:
                    break

        dx = target_point[0] - vehicle_pose[0]
        dy = target_point[1] - vehicle_pose[1]
        target_angle = math.atan2(dy, dx)
        steering_angle = target_angle - vehicle_pose[2]
        while steering_angle > math.pi:
            steering_angle -= 2 * math.pi
        while steering_angle < -math.pi:
            steering_angle += 2 * math.pi
        return steering_angle, target_point


class PurePursuitNavNode(object):
    def __init__(self):
        rospy.init_node('pure_pursuit_nav_node', anonymous=False)
        self.base_lookahead = rospy.get_param('~lookahead_distance', 0.7)
        self.pure_pursuit = PurePursuitController(lookahead_distance=self.base_lookahead)
        # 角速度上限设很高，相当于不限制，保证能转得过弯
        self.omega_max = rospy.get_param('~omega_max', 10.0)
        # 线速度上限，避免冲太快
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)
        self.path_points = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.path_received = False

        rospy.Subscriber('/path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/scout_mini_pro/cmd_vel', Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.control_cb)  # 20 Hz control loop; callbacks run via spin()

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return
        self.path_points_list = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.path_points = np.array(self.path_points_list)
        if self.path_points.ndim == 2 and self.path_points.shape[0] > 1:
            self.path_points = self.interpolate_path(self.path_points)
        self.path_received = True
        rospy.loginfo('Pure pursuit: path received, %d points', len(self.path_points) if self.path_points is not None else 0)

    def interpolate_path(self, points, segment_length=0.1):
        interpolated = []
        for i in range(len(points) - 1):
            start = points[i]
            end = points[i + 1]
            distance = np.linalg.norm(end - start)
            num_points = max(2, int(distance / segment_length) + 1)
            t_values = np.linspace(0, 1, num_points)
            segment = start + (end - start) * t_values[:, np.newaxis]
            interpolated.append(segment)
        if not interpolated:
            return points
        return np.vstack(interpolated)

    def amcl_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.pose_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def control_cb(self, _event):
        if not self.path_received or self.path_points is None or len(self.path_points) == 0:
            return
        pose = [self.pose_x, self.pose_y, self.pose_yaw]
        steering_angle, _ = self.pure_pursuit.calculate_steering_angle(pose, self.path_points)
        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        rospy.loginfo_throttle(1.0, 'Pure pursuit steering_angle=%.2f rad', steering_angle)

        cmd = Twist()
        if distance_to_end < 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.path_received = False
        else:
            speed = max(0.3, 1.5 - 1.5 * math.sin(0.6 * abs(steering_angle)))
            if abs(steering_angle) > math.pi / 3:
                speed = min(speed, 0.4)
            cmd.linear.x = min(speed, self.max_linear_speed)
            # 标准 Pure Pursuit：角速度 omega = 2*v*sin(alpha)/L，不是把 steering_angle 直接当 omega
            # 否则差速底盘收到的角速度与几何不一致，转弯不足甚至只直行
            L = max(0.2, self.base_lookahead)
            omega = 2.0 * speed * math.sin(steering_angle) / L
            cmd.angular.z = max(-self.omega_max, min(self.omega_max, omega))
        self.cmd_vel_pub.publish(cmd)


def main():
    node = PurePursuitNavNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
