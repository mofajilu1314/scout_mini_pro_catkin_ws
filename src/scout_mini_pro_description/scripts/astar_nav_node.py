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

# 默认膨胀 3 格（0.15m@0.05）：房间门/窄通道不易被堵死；可通过 rosparam ~expansion_size 调节（2=更易出路径，4=更贴近车身）
DEFAULT_EXPANSION_SIZE = 3
# 安全域半径默认 0 表示关闭；设为 6～10 格可让路径在宽走廊时更偏好走中间（非必要不贴墙）
DEFAULT_SAFETY_ZONE_RADIUS = 0
DEFAULT_SAFETY_ZONE_WEIGHT = 1.0


def costmap(data, width, height, expansion_size, safety_zone_radius=0, safety_zone_weight=1.0):
    """
    基于安全距离的代价地图：
    - 100: 障碍
    - d <= expansion_size: 代价 1 + (expansion_size - d)，越靠近墙越高
    - expansion_size < d <= safety_zone_radius 且 safety_zone_radius > 0: 额外软代价，使路径偏好远离墙（非必要不贴墙）
    - d > safety_zone_radius（或 safety_zone_radius=0）: 代价 1
    """
    occ = np.array(data, dtype=np.int16).reshape(height, width)
    # 将未知区域视为障碍，避免走出已知地图
    occ[occ < 0] = 100

    # 初始代价：1 表示 free，100 表示障碍
    cost = np.ones_like(occ, dtype=np.float64)
    wall = occ == 100
    cost[wall] = 100.0

    # 计算到最近障碍的欧式距离（单位：格）
    dist_free = distance_transform_edt(~wall)
    free_mask = ~wall

    if expansion_size > 0:
        # 在 expansion_size 范围内根据距离反比增加代价，越靠近障碍越贵
        d_clipped = np.clip(dist_free, 0.0, float(expansion_size))
        extra_cost = (float(expansion_size) - d_clipped)
        cost[free_mask] = 1.0 + extra_cost[free_mask]

    # 安全域：在 expansion_size < d <= safety_zone_radius 内增加软代价，路径更偏好走廊中线
    if safety_zone_radius > expansion_size and safety_zone_weight > 0:
        d = dist_free[free_mask]
        in_zone = (d > expansion_size) & (d <= safety_zone_radius)
        # extra_soft = weight * (safety_zone_radius - d) / (safety_zone_radius - expansion_size)，越靠近墙越大
        gap = float(safety_zone_radius - expansion_size)
        extra_soft = np.zeros_like(d)
        extra_soft[in_zone] = safety_zone_weight * (safety_zone_radius - d[in_zone]) / gap
        cost[free_mask] = cost[free_mask] + extra_soft

    return cost


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
                new_cost = cost_so_far[current] + grid[neighbor]
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
        rospy.loginfo('A* node started. Goal topic (remapped): %s, expansion_size: %d, safety_zone_radius: %d',
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
        # 基于安全距离的代价地图：靠墙格代价高，走廊中线代价低；安全域半径内加软代价使路径非必要不贴墙
        grid = costmap(self.map_msg.data, width, height, self.expansion_size,
                       self.safety_zone_radius, self.safety_zone_weight)

        # 起点/目标强制可通行且代价最低，避免刚好落在高代价环上
        grid[row, col] = 1.0
        grid[row_goal, col_goal] = 1.0

        # 目标周围 3x3 置为可通行，避免膨胀/代价导致无法到达
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                rn, cn = row_goal + dr, col_goal + dc
                if 0 <= rn < height and 0 <= cn < width:
                    grid[rn, cn] = 1.0
        start = (row, col)
        goal = (row_goal, col_goal)
        path = astar(start, goal, grid)
        if path:
            paths_world = [(p[1] * resolution + origin_x, p[0] * resolution + origin_y) for p in path]
            num_smooth = max(30, len(paths_world))
            path_smoothed = bezier_smoothing(paths_world, num_smooth)
            self.path = list(path_smoothed) if len(path_smoothed) > 0 else paths_world
        else:
            rospy.logwarn_throttle(2.0, 'A*: no path found from (%.2f,%.2f) to (%.2f,%.2f)', self.x, self.y, self.goal[0], self.goal[1])
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
