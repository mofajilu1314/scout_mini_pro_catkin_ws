#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 Gazebo 中的 scout_mini_pro 模型复位到指定位姿（用于小车卡住后恢复）。
仅调用 /gazebo/set_model_state，不参与导航逻辑。
用法: rosrun scout_mini_pro_description reset_robot_pose.py [x] [y] [yaw]
  默认 x=0 y=0 yaw=0；yaw 单位弧度。
"""

import sys
import math
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion


def quat_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main():
    rospy.init_node('reset_robot_pose', anonymous=True)
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
    except rospy.ROSException:
        print('Service /gazebo/set_model_state not available (is Gazebo running?).')
        sys.exit(1)
    srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    state = ModelState()
    state.model_name = 'scout_mini_pro'
    state.pose.position = Point(x, y, 0.5)
    state.pose.orientation = quat_from_yaw(yaw)
    state.twist.linear.x = 0.0
    state.twist.linear.y = 0.0
    state.twist.linear.z = 0.0
    state.twist.angular.x = 0.0
    state.twist.angular.y = 0.0
    state.twist.angular.z = 0.0
    srv(state)
    print('Set scout_mini_pro to x=%.2f y=%.2f yaw=%.2f' % (x, y, yaw))


if __name__ == '__main__':
    main()
