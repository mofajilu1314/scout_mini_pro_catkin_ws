#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A* path planning node for ROS1 Noetic.
Subscribes: /map (map_server), /amcl_pose (position in map), /move_base_simple/goal (RViz SetGoal).
Publishes: /path (nav_msgs/Path, frame_id=map).
Ported from Pure-tracking-slam-automatic-navigation-system (ROS2).
"""

from __future__ import print_function

import math
import heapq
import numpy as np
from scipy.interpolate import BSpline
from scipy.ndimage import distance_transform_edt

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# 默认硬膨胀 4 格（0.20 m @ 0.05 m/格）；room 图门口约宽 0.45 m，5 格会堵死通道
# 可通过 ~expansion_size 调节（3～5 推荐；过大则自动回退）
DEFAULT_EXPANSION_SIZE = 4
# 安全域：硬膨胀之外再加软代价，宽走廊时偏好走中间；0 关闭
DEFAULT_SAFETY_ZONE_RADIUS = 10
DEFAULT_SAFETY_ZONE_WEIGHT = 1.0
# 无路径时硬膨胀自动回退的下限（格）
MIN_EXPANSION_FALLBACK = 2


def costmap(data, width, height, expansion_size, safety_zone_radius=0, safety_zone_weight=1.0):
    """
    代价地图：
    - 100: 真实障碍，或距障碍 <= expansion_size 的硬膨胀区（不可通行）
    - expansion_size < d <= safety_zone_radius: 软代价（越靠墙越高），偏好走廊中线
    - 更远处: 代价 1
    """
    occ = np.array(data, dtype=np.int16).reshape(height, width)
    # 将未知区域视为障碍，避免走出已知地图
    occ[occ < 0] = 100

    cost = np.ones_like(occ, dtype=np.float64)
    wall = occ == 100
    cost[wall] = 100.0

    # 到最近障碍的欧式距离（单位：格）
    dist_free = distance_transform_edt(~wall)
    free_mask = ~wall

    # 硬膨胀：expansion_size 内一律不可通行，路径至少离墙 expansion_size 格
    if expansion_size > 0:
        inflated = free_mask & (dist_free <= float(expansion_size))
        cost[inflated] = 100.0

    # 安全域软代价：硬膨胀之外、安全半径之内，离墙越近代价越高
    passable = cost < 100.0
    if safety_zone_radius > expansion_size and safety_zone_weight > 0:
        d = dist_free[passable]
        in_zone = (d > float(expansion_size)) & (d <= float(safety_zone_radius))
        gap = float(safety_zone_radius - expansion_size)
        extra_soft = np.zeros_like(d)
        extra_soft[in_zone] = safety_zone_weight * (safety_zone_radius - d[in_zone]) / gap
        cost_vals = np.ones_like(d)
        cost_vals[in_zone] = 1.0 + extra_soft[in_zone]
        cost[passable] = cost_vals

    return cost, wall


def clear_near_pose(grid, wall, row, col, height, width, clear_r):
    """起点/终点附近取消硬膨胀（不打开真实墙），避免被堵死无法起步/到达。"""
    clear_r = max(1, int(clear_r))
    r2 = clear_r * clear_r
    for dr in range(-clear_r, clear_r + 1):
        for dc in range(-clear_r, clear_r + 1):
            if dr * dr + dc * dc > r2:
                continue
            rn, cn = row + dr, col + dc
            if 0 <= rn < height and 0 <= cn < width and not wall[rn, cn]:
                grid[rn, cn] = 1.0


def bezier_smoothing(array, num_points):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        dx = np.diff(x, prepend=x[0])
        dy = np.diff(y, prepend=y[0])
        chord_lengths = np.sqrt(dx**2 + dy**2)
        t = np.concatenate(([0], np.cumsum(chord_lengths)))
        t /= t[-1]
        k = min(num_points - 1, len(x) - 1)
        if k < 1:
            return array
        t_knots = np.concatenate(([0] * k, t, [1] * k))
        x_padded = np.pad(x, (k, k), 'edge')
        y_padded = np.pad(y, (k, k), 'edge')
        spline_x = BSpline(t_knots, x_padded, k, extrapolate=False)
        spline_y = BSpline(t_knots, y_padded, k, extrapolate=False)
        t_new = np.linspace(0, 1, num_points)
        x_smoothed = spline_x(t_new)
        y_smoothed = spline_y(t_new)
        path = np.column_stack((x_smoothed, y_smoothed))
    except Exception:
        path = array
    return path


def astar(start, goal, grid):
    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    closed_set = set()
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        if current in closed_set:
            continue
        closed_set.add(current)
        for d in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 100:
                step = 1.414 if d[0] != 0 and d[1] != 0 else 1.0
                new_cost = cost_so_far[current] + step * float(grid[neighbor])
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current
    return []


class AStarNavNode(object):
    def __init__(self):
        rospy.init_node('astar_nav_node', anonymous=False)
        self.map_msg = None
        self.x = 0.0
        self.y = 0.0
        self.goal = None
        self.path = None
        self.path_smoothed = None
        self.expansion_size = int(rospy.get_param('~expansion_size', DEFAULT_EXPANSION_SIZE))
        self.safety_zone_radius = int(rospy.get_param('~safety_zone_radius', DEFAULT_SAFETY_ZONE_RADIUS))
        self.safety_zone_weight = float(rospy.get_param('~safety_zone_weight', DEFAULT_SAFETY_ZONE_WEIGHT))

        # 目标话题使用可 remap 名，launch 中 remap 到 /move_base_simple/goal，与 RViz SetGoal 一致
        goal_topic = rospy.get_param('~goal_topic', 'goal')
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)
        rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback, queue_size=1)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        rospy.loginfo('A* node started. Goal topic (remapped): %s, expansion_size: %d (hard), safety_zone_radius: %d',
                      rospy.resolve_name(goal_topic), self.expansion_size, self.safety_zone_radius)

    def goal_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo('A* received goal: (%.2f, %.2f)', self.goal[0], self.goal[1])

    def amcl_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def map_callback(self, msg):
        self.map_msg = msg

    def timer_cb(self, _event):
        if self.map_msg is None or self.goal is None:
            self.publish_path_cb()
            return
        distance = math.hypot(self.x - self.goal[0], self.y - self.goal[1])
        if distance <= 0.2:
            self.path = None
            self.publish_path_cb()
            return
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        col = int((self.x - origin_x) / resolution)
        row = int((self.y - origin_y) / resolution)
        col_goal = int((self.goal[0] - origin_x) / resolution)
        row_goal = int((self.goal[1] - origin_y) / resolution)
        if not (0 <= row < height and 0 <= col < width and 0 <= row_goal < height and 0 <= col_goal < width):
            rospy.logwarn_throttle(2.0, 'A*: start (%.1f,%.1f) or goal (%.1f,%.1f) out of map bounds', self.x, self.y, self.goal[0], self.goal[1])
            self.publish_path_cb()
            return

        start = (row, col)
        goal = (row_goal, col_goal)
        path = []
        # 硬膨胀过大时门口会被堵死；从配置值向下回退直到找到路径
        for exp in range(self.expansion_size, MIN_EXPANSION_FALLBACK - 1, -1):
            grid, wall = costmap(self.map_msg.data, width, height, exp,
                                 self.safety_zone_radius, self.safety_zone_weight)
            # 起点/终点附近取消硬膨胀（不打开真实墙），保证可起步、可到达
            clear_r = max(2, exp)
            clear_near_pose(grid, wall, row, col, height, width, clear_r)
            clear_near_pose(grid, wall, row_goal, col_goal, height, width, clear_r)
            grid[row, col] = 1.0
            grid[row_goal, col_goal] = 1.0
            path = astar(start, goal, grid)
            if path:
                if exp < self.expansion_size:
                    rospy.logwarn_throttle(
                        5.0,
                        'A*: path found with reduced expansion_size=%d (configured=%d); narrow passage',
                        exp, self.expansion_size)
                break

        if path:
            paths_world = [(p[1] * resolution + origin_x, p[0] * resolution + origin_y) for p in path]
            num_smooth = max(30, len(paths_world))
            path_smoothed = bezier_smoothing(paths_world, num_smooth)
            self.path = list(path_smoothed) if len(path_smoothed) > 0 else paths_world
        else:
            rospy.logwarn_throttle(
                2.0,
                'A*: no path found from (%.2f,%.2f) to (%.2f,%.2f) '
                '(tried expansion_size %d..%d; check goal / map connectivity)',
                self.x, self.y, self.goal[0], self.goal[1],
                self.expansion_size, MIN_EXPANSION_FALLBACK)
            self.path = None
        self.publish_path_cb()

    def publish_path_cb(self, _event=None):
        if self.path is None or len(self.path) == 0:
            return
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'
        for (x, y) in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        rospy.loginfo_throttle(1.0, 'A*: publishing path with %d points', len(path_msg.poses))


def main():
    node = AStarNavNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
