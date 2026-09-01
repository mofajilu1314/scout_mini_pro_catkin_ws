#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure Pursuit path following for ROS1 Noetic (differential drive).

Subscribes: /path, /amcl_pose, /scan
Publishes: cmd_vel（参数 ~cmd_vel_topic，默认 /scout_mini_pro/cmd_vel；官方车 launch 可改为 /cmd_vel）

Design notes (post-mortem of "won't turn" bug):
- omega = gain * 2 * v_eff * sin(α) / L with v_eff floor so slowing v cannot kill turn rate.
- Laser may slow/stop linear motion but must NOT publish ω=0 when the path still needs a turn;
  both-blocked → spin in place toward the carrot (or steered micro-escape).
- Angle reverse (optional, allow_angle_reverse): enter when |α| > reverse_enter_angle
  (default 130°), exit with hysteresis; laser must NOT flip REVERSE → forward on rear≈0.10 body echoes,
  and must NOT treat those echoes as rear obstacles that crush |v| (keep reverse_linear_speed).
- Stuck recovery (~5 s) flips drive direction with real linear.x (must reverse/forward),
  and laser must NOT flip that sign during RECOVER_* (body/rear false-positives).
- Reverse reorient: after reverse (angle or recover) force reverse for ~5 s (no reorient
  check), then require open LaserScan sector min-range + real displacement → REORIENT
  (forward); x_b not required.
- REORIENT lock (~10 s): forbid angle-reverse enter; only stuck+near-obstacles may
  recover-reverse; after lock ends, keep suppressing angle-reverse until heading is
  sane or the robot has moved forward a bit.
- REORIENT steer: blend green-path side with left/right LaserScan clearance; if one side
  is clearly open and the other near a wall, turn toward the open side even if the path
  leans toward the wall.
- Avoid-steer: temporary small ω bias away from LF/RF (or LR/RR when reversing)
  near obstacles; does not change the path; vanishes when clear.
"""

from __future__ import print_function

import math
import numpy as np
from collections import deque
from scipy.spatial import KDTree

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


MODE_FORWARD = 'FORWARD'
MODE_REVERSE = 'REVERSE'
MODE_RECOVER_FORWARD = 'RECOVER_FORWARD'
MODE_RECOVER_REVERSE = 'RECOVER_REVERSE'
MODE_REORIENT = 'REORIENT'  # 倒车回正后强制前进；锁定期内禁止夹角倒车


class PurePursuitController(object):
    def __init__(self, lookahead_distance=0.9):
        self.lookahead_distance = lookahead_distance

    def calculate_steering_angle(self, vehicle_pose, path_points):
        """弧长前视点：从最近点起累计弧长 >= L 的第一个点。"""
        path_arr = np.asarray(path_points)
        if path_arr.ndim == 1 or path_arr.shape[0] < 2:
            return 0.0, vehicle_pose[:2], 0.0
        try:
            closest_point_idx = int(KDTree(path_arr[:, :2]).query(vehicle_pose[:2])[1])
        except Exception:
            closest_point_idx = 0
        closest_point_idx = max(0, min(closest_point_idx, len(path_arr) - 1))

        target_point = path_arr[closest_point_idx]
        accum = 0.0
        found = False
        for i in range(closest_point_idx, len(path_arr) - 1):
            seg = math.hypot(
                path_arr[i + 1][0] - path_arr[i][0],
                path_arr[i + 1][1] - path_arr[i][1],
            )
            accum += seg
            if accum >= self.lookahead_distance:
                target_point = path_arr[i + 1]
                found = True
                break
        if not found:
            target_point = path_arr[-1]

        dx = target_point[0] - vehicle_pose[0]
        dy = target_point[1] - vehicle_pose[1]
        # 车体坐标系下前视点：x>0 在车头前方
        cy = math.cos(vehicle_pose[2])
        sy = math.sin(vehicle_pose[2])
        x_b = dx * cy + dy * sy
        y_b = -dx * sy + dy * cy
        steering_angle = normalize_angle(math.atan2(dy, dx) - vehicle_pose[2])
        return steering_angle, target_point, x_b


class PurePursuitNavNode(object):
    def __init__(self):
        rospy.init_node('pure_pursuit_nav_node', anonymous=False)
        self.base_lookahead = rospy.get_param('~lookahead_distance', 0.9)
        self.pure_pursuit = PurePursuitController(lookahead_distance=self.base_lookahead)

        # ---- 四要素相关（更激进，保证弯道转得过）----
        self.omega_max = rospy.get_param('~omega_max', 4.0)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.75)
        self.min_linear_speed = rospy.get_param('~min_linear_speed', 0.20)
        # 倒车线速度 ≈ reverse_speed_ratio * max_linear_speed（默认 2/3）
        self.reverse_speed_ratio = float(
            rospy.get_param('~reverse_speed_ratio', 2.0 / 3.0)
        )
        self.reverse_linear_speed = rospy.get_param(
            '~reverse_linear_speed',
            self.max_linear_speed * self.reverse_speed_ratio,
        )
        self.omega_v_floor = rospy.get_param('~omega_v_floor', 0.55)
        self.omega_gain = rospy.get_param('~omega_gain', 1.35)

        self.curvature_enabled = rospy.get_param('~curvature_enabled', True)
        self.curvature_v_min_scale = rospy.get_param('~curvature_v_min_scale', 0.50)
        self.curvature_kappa_soft = rospy.get_param('~curvature_kappa_soft', 1.0)
        self.curvature_kappa_hard = rospy.get_param('~curvature_kappa_hard', 2.5)

        # 双向跟踪：夹角倒车由 allow_angle_reverse 控制；脱困倒车始终可用（见 stuck FSM）
        self.allow_reversing = rospy.get_param('~allow_reversing', True)
        # True=非脱困时允许按路径夹角倒车；False=仅受困/脱困才倒车，平时只前进+转向
        self.allow_angle_reverse = rospy.get_param('~allow_angle_reverse', True)
        self.reverse_enter_angle = rospy.get_param(
            '~reverse_enter_angle', math.radians(130.0)
        )
        self.reverse_exit_angle = rospy.get_param(
            '~reverse_exit_angle', math.radians(80.0)
        )
        self.reversing = False

        # 卡住恢复
        self.stuck_enabled = rospy.get_param('~stuck_enabled', True)
        self.stuck_window_sec = rospy.get_param('~stuck_window_sec', 5.0)
        self.stuck_dist_threshold = rospy.get_param('~stuck_dist_threshold', 0.08)
        self.stuck_recover_duration = rospy.get_param('~stuck_recover_duration', 5.0)
        self.stuck_cooldown_sec = rospy.get_param('~stuck_cooldown_sec', 1.5)
        self.stuck_steer_omega = rospy.get_param('~stuck_steer_omega', 1.8)
        if rospy.has_param('~stuck_reverse_duration') and not rospy.has_param(
            '~stuck_recover_duration'
        ):
            self.stuck_recover_duration = rospy.get_param('~stuck_reverse_duration')
        self._pose_hist = deque()
        self._drive_mode = MODE_FORWARD
        self._recover_until = None
        self._stuck_cooldown_until = None
        self._intent_moving = False
        self._laser_holding = False
        self._path_goal_xy = None

        # 倒车回正：先强制倒满 min_reverse_sec，再查前后扇区开阔 + 位移 → 切前进
        self.reorient_enabled = rospy.get_param('~reorient_enabled', True)
        self.reorient_min_reverse_sec = rospy.get_param(
            '~reorient_min_reverse_sec', 5.0
        )
        self.reorient_clear_front = rospy.get_param('~reorient_clear_front', 0.80)
        self.reorient_clear_rear = rospy.get_param('~reorient_clear_rear', 0.70)
        # 低于此距离视为车体回波，不参与「开阔」判定（现场 rear≈0.10）
        self.reorient_ignore_below = rospy.get_param('~reorient_ignore_below', 0.15)
        self.reorient_min_displacement = rospy.get_param(
            '~reorient_min_displacement', 0.20
        )
        # 回正锁定期：REORIENT 强制前进时长（禁止夹角倒车）；默认 ~10s
        self.reorient_hold_sec = rospy.get_param('~reorient_hold_sec', 10.0)
        # 锁定期结束后，须「正常前进」才恢复夹角倒车（位移门槛）
        self.reorient_unlock_disp = rospy.get_param('~reorient_unlock_disp', 0.30)
        # REORIENT 期间脱困：除位移卡住外，还要求近障（避免空地慢掉头误脱困）
        self.reorient_stuck_near = rospy.get_param('~reorient_stuck_near', 0.55)
        # REORIENT 掉头方向：绿线侧 + 左右雷达开阔；一侧明显空、另一侧贴墙 → 优先开阔侧
        self.reorient_steer_lidar_enabled = rospy.get_param(
            '~reorient_steer_lidar_enabled', True
        )
        self.reorient_side_half_angle = rospy.get_param(
            '~reorient_side_half_angle', math.radians(40.0)
        )
        # |left-right| ≥ 此值才算「一侧明显更空」
        self.reorient_side_clear_margin = rospy.get_param(
            '~reorient_side_clear_margin', 0.40
        )
        # 较近一侧 < 此值视为「贴墙」，才允许雷达覆盖绿线偏好
        self.reorient_side_near_wall = rospy.get_param(
            '~reorient_side_near_wall', 0.55
        )
        self._reverse_seg_start = None
        self._reverse_seg_xy = None
        self._reorient_until = None
        # 回正锁 / 锁后抑制：True 时 allow_angle_reverse 对「进入」暂时无效
        self._angle_rev_suppress = False
        self._reorient_unlock_xy = None

        # 瞬时避障转向力：不改路径，仅在 cmd_vel.ω 上叠小幅远离偏置
        self.avoid_steer_enabled = rospy.get_param('~avoid_steer_enabled', True)
        self.avoid_steer_near = rospy.get_param('~avoid_steer_near', 0.50)
        self.avoid_steer_far = rospy.get_param('~avoid_steer_far', 0.90)
        self.avoid_steer_omega_max = rospy.get_param('~avoid_steer_omega_max', 0.40)
        self.avoid_steer_half_angle = rospy.get_param(
            '~avoid_steer_half_angle', math.radians(30.0)
        )

        # 激光：只在真正近障时干预；开阔弯道不误刹
        self.laser_enabled = rospy.get_param('~laser_enabled', True)
        self.laser_topic = rospy.get_param('~laser_topic', '/scan')
        self.laser_front_stop = rospy.get_param('~laser_front_stop', 0.22)
        self.laser_front_slow = rospy.get_param('~laser_front_slow', 0.45)
        self.laser_rear_stop = rospy.get_param('~laser_rear_stop', 0.20)
        self.laser_rear_slow = rospy.get_param('~laser_rear_slow', 0.40)
        self.laser_sector_half_angle = rospy.get_param(
            '~laser_sector_half_angle', math.radians(35.0)
        )
        self.laser_escape_speed = rospy.get_param('~laser_escape_speed', 0.32)
        self.laser_escape_duration = rospy.get_param('~laser_escape_duration', 1.4)
        self.laser_escape_omega = rospy.get_param('~laser_escape_omega', 1.6)
        self.laser_min_valid = rospy.get_param('~laser_min_valid', 0.08)
        self._front_min = float('inf')
        self._rear_min = float('inf')
        self._left_min = float('inf')
        self._right_min = float('inf')
        self._lf_min = float('inf')  # 左前
        self._rf_min = float('inf')  # 右前
        self._lr_min = float('inf')  # 左后
        self._rr_min = float('inf')  # 右后
        self._laser_escape_until = None
        self._laser_escape_sign = 0
        self._laser_escape_omega_cmd = 0.0
        self._last_avoid_bias = 0.0

        self.path_points = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.path_received = False

        # 方块车默认 /scout_mini_pro/cmd_vel；官方车用 nav_bringup_official 设为 /cmd_vel
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/scout_mini_pro/cmd_vel')

        rospy.Subscriber('/path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)
        if self.laser_enabled:
            rospy.Subscriber(self.laser_topic, LaserScan, self.scan_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.control_cb)
        rospy.loginfo(
            'Pure pursuit started: cmd_vel=%s L=%.2f omega_max=%.2f vmax=%.2f '
            'vrev=%.2f(ratio=%.2f) omega_v_floor=%.2f gain=%.2f '
            'reverse_enter=%.0fdeg allow_angle_reverse=%s reorient=%s '
            'reorient_min=%.1fs hold=%.1fs unlock_disp=%.2f '
            'clear_f/r=%.2f/%.2f disp=%.2f '
            'steer_lidar=%s side_margin=%.2f near_wall=%.2f '
            'avoid_steer=%s near/far=%.2f/%.2f ωmax=%.2f laser=%s',
            self.cmd_vel_topic,
            self.base_lookahead,
            self.omega_max,
            self.max_linear_speed,
            self.reverse_linear_speed,
            self.reverse_speed_ratio,
            self.omega_v_floor,
            self.omega_gain,
            math.degrees(self.reverse_enter_angle),
            self.allow_angle_reverse,
            self.reorient_enabled,
            self.reorient_min_reverse_sec,
            self.reorient_hold_sec,
            self.reorient_unlock_disp,
            self.reorient_clear_front,
            self.reorient_clear_rear,
            self.reorient_min_displacement,
            self.reorient_steer_lidar_enabled,
            self.reorient_side_clear_margin,
            self.reorient_side_near_wall,
            self.avoid_steer_enabled,
            self.avoid_steer_near,
            self.avoid_steer_far,
            self.avoid_steer_omega_max,
            self.laser_enabled,
        )

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            return
        self.path_points = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        )
        if self.path_points.ndim == 2 and self.path_points.shape[0] > 1:
            self.path_points = self.interpolate_path(self.path_points)
        self.path_received = True

        goal = (
            float(msg.poses[-1].pose.position.x),
            float(msg.poses[-1].pose.position.y),
        )
        new_goal = (
            self._path_goal_xy is None
            or math.hypot(goal[0] - self._path_goal_xy[0], goal[1] - self._path_goal_xy[1])
            > 0.35
        )
        if new_goal:
            self._path_goal_xy = goal
            self.reversing = False
            self._drive_mode = MODE_FORWARD
            self._recover_until = None
            self._laser_escape_until = None
            self._stuck_cooldown_until = None
            self._pose_hist.clear()
            self._laser_holding = False
            self._reverse_seg_start = None
            self._reverse_seg_xy = None
            self._reorient_until = None
            self._angle_rev_suppress = False
            self._reorient_unlock_xy = None
            rospy.loginfo(
                'Pure pursuit: new goal path, %d points',
                len(self.path_points),
            )
        else:
            # 绝不在路径刷新时清空位姿历史，否则卡住检测永远攒不满窗口
            rospy.loginfo_throttle(
                5.0,
                'Pure pursuit: path refresh, %d points (state kept)',
                len(self.path_points),
            )

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

    def scan_callback(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        angle = msg.angle_min
        front_min = float('inf')
        rear_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')
        lf_min = float('inf')
        rf_min = float('inf')
        lr_min = float('inf')
        rr_min = float('inf')
        half = self.laser_sector_half_angle
        side_half = self.reorient_side_half_angle
        diag_half = self.avoid_steer_half_angle
        # 对角扇区中心：左前 +45°、右前 -45°、左后 +135°、右后 -135°
        lf_c = 0.25 * math.pi
        rf_c = -0.25 * math.pi
        lr_c = 0.75 * math.pi
        rr_c = -0.75 * math.pi
        # 与回正共用：≤ ignore_below 视为车体回波，不参与扇区最近距离
        # （否则 rear≈0.10 会让 Laser keep-dir 把倒车 |v| 砍到 ~0.32）
        body_echo_max = max(self.laser_min_valid, self.reorient_ignore_below)
        for i in range(n):
            r = msg.ranges[i]
            a = normalize_angle(angle)
            angle += msg.angle_increment
            if math.isnan(r) or math.isinf(r):
                continue
            if r < max(msg.range_min, self.laser_min_valid) or r > msg.range_max:
                continue
            if abs(a) <= half:
                if r < front_min:
                    front_min = r
            elif abs(a) >= (math.pi - half):
                # 仅后向忽略车体回波；前向仍保留近距，避免漏刹
                if r <= body_echo_max:
                    continue
                if r < rear_min:
                    rear_min = r
            # 左右侧扇区（±90° 附近）：REORIENT 掉头开阔度；忽略车体回波
            if r > body_echo_max:
                if abs(normalize_angle(a - 0.5 * math.pi)) <= side_half:
                    if r < left_min:
                        left_min = r
                elif abs(normalize_angle(a + 0.5 * math.pi)) <= side_half:
                    if r < right_min:
                        right_min = r
                # 对角扇区：瞬时避障转向力
                if abs(normalize_angle(a - lf_c)) <= diag_half:
                    if r < lf_min:
                        lf_min = r
                if abs(normalize_angle(a - rf_c)) <= diag_half:
                    if r < rf_min:
                        rf_min = r
                if abs(normalize_angle(a - lr_c)) <= diag_half:
                    if r < lr_min:
                        lr_min = r
                if abs(normalize_angle(a - rr_c)) <= diag_half:
                    if r < rr_min:
                        rr_min = r
        self._front_min = front_min
        self._rear_min = rear_min
        self._left_min = left_min
        self._right_min = right_min
        self._lf_min = lf_min
        self._rf_min = rf_min
        self._lr_min = lr_min
        self._rr_min = rr_min

    def _laser_obstacle_dist(self, dist, ignore_body_echo=False):
        """
        供激光安全使用的障碍距离。
        ignore_body_echo=True（倒车后向）：无效或 ≤ ignore_below 视为无障碍，
        避免 rear≈0.10 把倒车目标速度误砍成 ~0.32。
        """
        if not math.isfinite(dist):
            return float('inf')
        if ignore_body_echo and dist <= self.reorient_ignore_below:
            return float('inf')
        return dist

    def _path_steer_sign(self, steering_angle, want_reverse):
        alpha_eff = (
            normalize_angle(steering_angle + math.pi) if want_reverse else steering_angle
        )
        if abs(alpha_eff) < 1e-3:
            return 0.0
        return 1.0 if alpha_eff > 0.0 else -1.0

    def _update_reverse_mode(self, steering_angle, x_b):
        """
        夹角倒车（Logic A）：绿线/前视点相对车头夹角 |α| 超过进入阈值 → REVERSE。
        由 allow_angle_reverse 控制；关掉后非脱困只前进+转向。
        回正锁定期 / 锁后抑制期间：禁止因夹角进入倒车（开关暂时无效）。
        不再强制 x_b<0（前视点略靠前时路径已在车后仍可倒车）。
        """
        if not self.allow_reversing or not self.allow_angle_reverse:
            # 脱困态不在这里清 reversing；非脱困则禁止夹角倒车
            if self._drive_mode not in (
                MODE_RECOVER_FORWARD,
                MODE_RECOVER_REVERSE,
                MODE_REORIENT,
            ):
                self.reversing = False
            return
        if self._drive_mode in (
            MODE_RECOVER_FORWARD,
            MODE_RECOVER_REVERSE,
            MODE_REORIENT,
        ):
            return
        # 回正锁后抑制：禁止夹角进入倒车，但仍允许已在倒车时滞回退出
        if self._angle_rev_suppress and not self.reversing:
            self._try_clear_angle_rev_suppress(steering_angle)
            if self._angle_rev_suppress:
                return
        abs_alpha = abs(steering_angle)
        if self.reversing:
            # 滞回退出：夹角回到退出角以下，或前视已明显在车头前方且夹角不大
            if abs_alpha < self.reverse_exit_angle:
                self.reversing = False
            elif x_b > 0.35 and abs_alpha < math.radians(90.0):
                self.reversing = False
        else:
            if abs_alpha > self.reverse_enter_angle:
                self.reversing = True
                rospy.loginfo_throttle(
                    2.0,
                    'Angle reverse ENTER |α|=%.0fdeg > %.0fdeg x_b=%.2f',
                    math.degrees(abs_alpha),
                    math.degrees(self.reverse_enter_angle),
                    x_b,
                )

    def _try_clear_angle_rev_suppress(self, steering_angle):
        """锁定期结束后：夹角已回正，或已正常前进一段位移 → 恢复夹角倒车检测。"""
        if not self._angle_rev_suppress:
            return
        if self._drive_mode == MODE_REORIENT:
            return
        abs_alpha = abs(steering_angle)
        if abs_alpha < self.reverse_exit_angle:
            self._clear_angle_rev_suppress('alpha_ok')
            return
        if self._reorient_unlock_xy is not None:
            x0, y0 = self._reorient_unlock_xy
            disp = math.hypot(self.pose_x - x0, self.pose_y - y0)
            if disp >= self.reorient_unlock_disp:
                self._clear_angle_rev_suppress('disp_ok')

    def _clear_angle_rev_suppress(self, reason):
        if not self._angle_rev_suppress:
            return
        self._angle_rev_suppress = False
        self._reorient_unlock_xy = None
        rospy.loginfo(
            'REORIENT unlock: angle-reverse restored (%s)',
            reason,
        )

    def _reorient_has_near_obstacles(self):
        """回正期间脱困门槛：近障较多（前/侧至少两侧近，或前向很近）。"""
        near = self.reorient_stuck_near
        hits = 0
        for dist in (
            self._front_min,
            self._left_min,
            self._right_min,
            self._lf_min,
            self._rf_min,
        ):
            if math.isfinite(dist) and dist < near:
                hits += 1
        front_close = (
            math.isfinite(self._front_min) and self._front_min < near * 0.85
        )
        return hits >= 2 or front_close

    def _is_stuck(self, now):
        if not self.stuck_enabled:
            return False
        self._pose_hist.append((now, self.pose_x, self.pose_y))
        cutoff = now - rospy.Duration.from_sec(self.stuck_window_sec)
        while self._pose_hist and self._pose_hist[0][0] < cutoff:
            self._pose_hist.popleft()

        if self._stuck_cooldown_until is not None and now < self._stuck_cooldown_until:
            return False
        if not (self._intent_moving or self._laser_holding):
            return False
        if len(self._pose_hist) < 2:
            return False
        t0, x0, y0 = self._pose_hist[0]
        span = (now - t0).to_sec()
        if span < 0.85 * self.stuck_window_sec:
            return False
        disp = math.hypot(self.pose_x - x0, self.pose_y - y0)
        return disp < self.stuck_dist_threshold

    def _update_stuck_fsm(self, now, want_reverse_tracking, steering_angle):
        if self._recover_until is not None and now >= self._recover_until:
            self._recover_until = None
            self._stuck_cooldown_until = now + rospy.Duration.from_sec(
                self.stuck_cooldown_sec
            )
            self._pose_hist.clear()
            self._drive_mode = MODE_REVERSE if want_reverse_tracking else MODE_FORWARD
            self.reversing = want_reverse_tracking
            self._laser_escape_until = None
            # 恢复结束仍倒车时继续累计回正段；否则清零
            if not want_reverse_tracking:
                self._reverse_seg_start = None
                self._reverse_seg_xy = None
            rospy.loginfo('Pure pursuit recover done → %s', self._drive_mode)

        if self._drive_mode in (MODE_RECOVER_FORWARD, MODE_RECOVER_REVERSE):
            return

        # REORIENT：保持前进、抑制夹角倒车；仅卡住且近障多 → 脱困倒车
        if self._drive_mode == MODE_REORIENT:
            if self._reorient_until is not None and now >= self._reorient_until:
                self._reorient_until = None
                self._drive_mode = MODE_FORWARD
                self.reversing = False
                # 锁定期结束：仍抑制夹角倒车，直到正常前进（α 回正或位移够）
                self._angle_rev_suppress = True
                self._reorient_unlock_xy = (self.pose_x, self.pose_y)
                rospy.loginfo(
                    'REORIENT done → FORWARD (angle-reverse still locked '
                    'until α<%.0fdeg or disp≥%.2fm)',
                    math.degrees(self.reverse_exit_angle),
                    self.reorient_unlock_disp,
                )
            else:
                stuck = self._is_stuck(now)
                if stuck and self._reorient_has_near_obstacles():
                    rospy.logwarn(
                        'REORIENT stuck+near → allow recover reverse '
                        '(front=%.2f L=%.2f R=%.2f)',
                        self._front_min if math.isfinite(self._front_min) else -1.0,
                        self._left_min if math.isfinite(self._left_min) else -1.0,
                        self._right_min if math.isfinite(self._right_min) else -1.0,
                    )
                    self._trigger_stuck_recover(
                        now, want_reverse_tracking, steering_angle
                    )
                elif stuck:
                    rospy.loginfo_throttle(
                        2.0,
                        'REORIENT stuck but not near-dense → keep forward '
                        '(no angle-reverse; wait unlock or near obstacles)',
                    )
            return

        self._drive_mode = MODE_REVERSE if want_reverse_tracking else MODE_FORWARD
        self.reversing = want_reverse_tracking

        if not self._is_stuck(now):
            return

        self._trigger_stuck_recover(now, want_reverse_tracking, steering_angle)

    def _trigger_stuck_recover(self, now, want_reverse_tracking, steering_angle):
        """前进卡住→倒车脱困；倒车卡住→前进脱困。REORIENT 视为前进。"""
        steer_s = self._path_steer_sign(steering_angle, want_reverse_tracking)
        # REORIENT 是前进回正；仅真正倒车态才切 RECOVER_FORWARD
        if self._drive_mode == MODE_REVERSE or (
            self.reversing and self._drive_mode != MODE_REORIENT
        ):
            # 倒车卡住 → 前进绕回路径
            self._drive_mode = MODE_RECOVER_FORWARD
            self.reversing = False
            self._reverse_seg_start = None
            self._reverse_seg_xy = None
            self._reorient_until = None
            rospy.logwarn(
                'STUCK → RECOVER_FORWARD %.1fs (steer=%+.0f front=%.2f rear=%.2f)',
                self.stuck_recover_duration,
                steer_s,
                self._front_min if math.isfinite(self._front_min) else -1.0,
                self._rear_min if math.isfinite(self._rear_min) else -1.0,
            )
        else:
            # 前进 / REORIENT 卡住 → 强制倒车（与 allow_angle_reverse 无关；仅 allow_reversing 总闸）
            if not self.allow_reversing:
                self._drive_mode = MODE_RECOVER_FORWARD
                rospy.logwarn(
                    'STUCK → RECOVER_FORWARD(spin) %.1fs (allow_reversing=false)',
                    self.stuck_recover_duration,
                )
            else:
                self._drive_mode = MODE_RECOVER_REVERSE
                self.reversing = True
                self._reverse_seg_start = None
                self._reverse_seg_xy = None
                rospy.logwarn(
                    'STUCK → RECOVER_REVERSE %.1fs (steer=%+.0f front=%.2f rear=%.2f)',
                    self.stuck_recover_duration,
                    steer_s,
                    self._front_min if math.isfinite(self._front_min) else -1.0,
                    self._rear_min if math.isfinite(self._rear_min) else -1.0,
                )
        self._recover_until = now + rospy.Duration.from_sec(self.stuck_recover_duration)
        self._pose_hist.clear()
        self._laser_holding = False
        self._laser_escape_until = None
        self._reorient_until = None
        # 进入脱困倒车时清掉回正抑制，让回正链路重新武装
        if self._drive_mode == MODE_RECOVER_REVERSE:
            self._angle_rev_suppress = False
            self._reorient_unlock_xy = None

    def _is_sector_open_for_reorient(self, dist, clear_m):
        """
        开阔判定（LaserScan 扇区最近距离，非点云数量）：
        无效读数视为开阔；≤ ignore_below 视为车体回波忽略；否则需 ≥ clear_m。
        """
        if not math.isfinite(dist):
            return True
        if dist <= self.reorient_ignore_below:
            return True
        return dist >= clear_m

    def _reorient_sector_fmt(self, dist):
        return dist if math.isfinite(dist) else -1.0

    def _side_clear_for_compare(self, dist):
        """无有效回波视为开阔（大距离），便于左右差比较。"""
        if not math.isfinite(dist) or dist <= self.reorient_ignore_below:
            return 5.0
        return dist

    def _pick_reorient_steer_sign(self, steering_angle):
        """
        REORIENT 掉头方向：绿线侧给路径偏好；左右雷达最近距离给安全偏好。
        一侧明显更空，且（对侧贴墙 或 绿线偏向更窄侧）→ 优先开阔侧；两侧差不多 → 跟绿线。
        返回 (pick_sign, path_label, left_m, right_m)，sign: +1=L / -1=R / 0=无偏好。
        """
        path_s = self._path_steer_sign(steering_angle, want_reverse=False)
        if path_s > 0.0:
            path_label = 'L'
        elif path_s < 0.0:
            path_label = 'R'
        else:
            path_label = '-'

        left_m = self._reorient_sector_fmt(self._left_min)
        right_m = self._reorient_sector_fmt(self._right_min)
        left_c = self._side_clear_for_compare(self._left_min)
        right_c = self._side_clear_for_compare(self._right_min)
        delta = left_c - right_c
        margin = self.reorient_side_clear_margin
        near_wall = self.reorient_side_near_wall

        pick_s = path_s
        if self.reorient_steer_lidar_enabled and abs(delta) >= margin:
            clearer = 1.0 if delta > 0.0 else -1.0
            tighter = right_c if clearer > 0.0 else left_c
            # 一侧明显更空：对侧贴墙，或绿线偏向更窄一侧 → 覆盖绿线（贴墙路径防拧进墙）
            path_leans_tight = path_s != 0.0 and path_s != clearer
            if tighter < near_wall or path_leans_tight:
                pick_s = clearer

        if pick_s == 0.0 and abs(delta) >= margin:
            pick_s = 1.0 if delta > 0.0 else -1.0

        if pick_s > 0.0:
            pick_label = 'L'
        elif pick_s < 0.0:
            pick_label = 'R'
        else:
            pick_label = '-'

        rospy.loginfo_throttle(
            0.8,
            'REORIENT steer: path=%s lidar L=%.2f R=%.2f → pick %s',
            path_label,
            left_m,
            right_m,
            pick_label,
        )
        return pick_s, path_label, left_m, right_m

    def _update_reorient(self, now):
        """
        倒车回正：进入夹角/脱困倒车后先强制倒满 reorient_min_reverse_sec（不做回正检测），
        之后若前后扇区最小 range 够开阔且坐标位移够大，才切 REORIENT（前进）；x_b 不参与。
        """
        if not self.reorient_enabled:
            return

        in_reverse_drive = self._drive_mode in (MODE_REVERSE, MODE_RECOVER_REVERSE)
        if not in_reverse_drive:
            if self._drive_mode != MODE_REORIENT:
                self._reverse_seg_start = None
                self._reverse_seg_xy = None
            return

        if self._reverse_seg_start is None:
            self._reverse_seg_start = now
            self._reverse_seg_xy = (self.pose_x, self.pose_y)
            rospy.loginfo(
                'REORIENT arm: reverse started, force reverse %.1fs before check',
                self.reorient_min_reverse_sec,
            )
            return

        elapsed = (now - self._reverse_seg_start).to_sec()
        x0, y0 = self._reverse_seg_xy
        disp = math.hypot(self.pose_x - x0, self.pose_y - y0)
        front_m = self._reorient_sector_fmt(self._front_min)
        rear_m = self._reorient_sector_fmt(self._rear_min)

        # 强制倒车窗口内：绝不因位移/开阔提前回正
        if elapsed < self.reorient_min_reverse_sec:
            rospy.loginfo_throttle(
                1.0,
                'REORIENT check: t=%.1fs front=%.2f rear=%.2f disp=%.2f '
                'pass=False (wait min=%.1fs; no reorient yet)',
                elapsed,
                front_m,
                rear_m,
                disp,
                self.reorient_min_reverse_sec,
            )
            return

        front_ok = self._is_sector_open_for_reorient(
            self._front_min, self.reorient_clear_front
        )
        rear_ok = self._is_sector_open_for_reorient(
            self._rear_min, self.reorient_clear_rear
        )
        disp_ok = disp >= self.reorient_min_displacement
        passed = front_ok and rear_ok and disp_ok

        rospy.loginfo_throttle(
            1.0,
            'REORIENT check: t=%.1fs front=%.2f(ok=%s thr=%.2f) '
            'rear=%.2f(ok=%s thr=%.2f) disp=%.2f(ok=%s thr=%.2f) pass=%s',
            elapsed,
            front_m,
            front_ok,
            self.reorient_clear_front,
            rear_m,
            rear_ok,
            self.reorient_clear_rear,
            disp,
            disp_ok,
            self.reorient_min_displacement,
            passed,
        )
        if not passed:
            return

        # 触发回正：退出倒车 / 提前结束脱困倒车；进入锁定期（禁止夹角倒车）
        self.reversing = False
        self._recover_until = None
        self._drive_mode = MODE_REORIENT
        self._reorient_until = now + rospy.Duration.from_sec(self.reorient_hold_sec)
        self._angle_rev_suppress = True
        self._reorient_unlock_xy = None
        self._reverse_seg_start = None
        self._reverse_seg_xy = None
        self._pose_hist.clear()
        self._laser_escape_until = None
        self._laser_holding = False
        rospy.loginfo(
            'REORIENT: reverse reorient → FORWARD '
            '(t=%.1fs front=%.2f rear=%.2f disp=%.2f hold=%.1fs lock)',
            elapsed,
            front_m,
            rear_m,
            disp,
            self.reorient_hold_sec,
        )

    def _curvature_speeds(self, steering_angle, want_reverse):
        """直线更快；弯道主要降 v，ω 上限保持 omega_max。倒车降速更温和。"""
        L = max(0.35, self.base_lookahead)
        alpha_eff = (
            normalize_angle(steering_angle + math.pi) if want_reverse else steering_angle
        )
        abs_alpha = abs(alpha_eff)
        kappa = abs(2.0 * math.sin(alpha_eff) / L)

        vmax = self.reverse_linear_speed if want_reverse else self.max_linear_speed
        omega_cap = self.omega_max
        # 倒车：保留 reverse_linear_speed 意图，曲率/夹角最多轻降（不低于 85%）
        min_scale = (
            max(0.85, self.curvature_v_min_scale) if want_reverse else self.curvature_v_min_scale
        )

        if self.curvature_enabled:
            soft = self.curvature_kappa_soft
            hard = max(soft + 1e-3, self.curvature_kappa_hard)
            if kappa <= soft:
                v_scale = 1.0
            elif kappa >= hard:
                v_scale = min_scale
            else:
                t = (kappa - soft) / (hard - soft)
                v_scale = 1.0 - t * (1.0 - min_scale)
            # 前进仍可按夹角再压；倒车跳过激进 angle_scale，避免 |v| 远低于设定
            if (not want_reverse) and abs_alpha > 0.50:
                angle_scale = 1.0 - 0.30 * min(1.0, abs_alpha / (math.pi / 2.0))
                v_scale = min(v_scale, max(min_scale, angle_scale))
            speed = max(self.min_linear_speed, min(vmax, vmax * v_scale))
        else:
            speed = vmax
            if (not want_reverse) and abs_alpha > 0.35:
                speed = max(
                    self.min_linear_speed,
                    vmax * (1.0 - 0.35 * min(1.0, abs_alpha / (math.pi / 2.0))),
                )

        return speed, omega_cap, kappa, alpha_eff

    def _compute_omega(self, steering_angle, signed_v, omega_cap, alpha_eff):
        """
        曲率大 → |ω| 更大（在上限内）；v 被降也不拖垮转向。
        差速：omega = gain * 2 * v_eff * sin(α) / L，α 用车头指向前视点的角。
        """
        L = max(0.35, self.base_lookahead)
        v_mag = max(abs(signed_v), self.omega_v_floor)
        v_eff = math.copysign(v_mag, signed_v if abs(signed_v) > 1e-9 else 1.0)
        omega = self.omega_gain * 2.0 * v_eff * math.sin(steering_angle) / L

        abs_alpha = abs(alpha_eff)
        # 中大转角保证最小角速度权威（弯道必须转得过）
        if abs_alpha > 0.20:
            # α↑ → min_w↑，最高到接近 omega_cap
            min_w = min(
                omega_cap,
                0.70 + 1.6 * min(1.0, abs_alpha / (math.pi / 2.0)),
            )
            if abs(omega) < min_w:
                if abs(omega) > 1e-6:
                    sign = 1.0 if omega > 0 else -1.0
                elif abs(alpha_eff) > 1e-6:
                    sign = 1.0 if alpha_eff > 0 else -1.0
                else:
                    sign = 1.0 if steering_angle >= 0 else -1.0
                omega = sign * min_w
        return max(-omega_cap, min(omega_cap, omega))

    def _spin_toward_path(self, want_reverse, steering_angle):
        """双侧近障时绝不发全零：原地朝绿线拧。"""
        cmd = Twist()
        steer_s = self._path_steer_sign(steering_angle, want_reverse)
        if steer_s == 0.0:
            steer_s = 1.0 if steering_angle >= 0 else -1.0
        cmd.angular.z = steer_s * min(self.omega_max, max(1.2, self.stuck_steer_omega))
        return cmd

    def _recover_dir_sign(self):
        """RECOVER_REVERSE → -1（必须倒车）；RECOVER_FORWARD → +1；其它 → 0。"""
        if self._drive_mode == MODE_RECOVER_REVERSE:
            return -1.0
        if self._drive_mode == MODE_RECOVER_FORWARD:
            return 1.0
        return 0.0

    def _apply_laser_safety(self, cmd, now, want_reverse, steering_angle):
        """
        近障：减速 / 换向逃离（带打角）/ 原地拧。
        禁止在仍需转向时发布 v=0 且 ω=0。
        后向扇区仅在倒车意图时启用，避免前进转弯被车后墙壁误 STOP。
        卡住恢复（RECOVER_*）与夹角倒车（REVERSE / want_reverse）期间：
        禁止激光把线速度符号翻反（现场：rear≈0.10 车体回波误触发
        Laser escape v_sign=+1，导致倒车立刻变前进顶墙）。
        """
        self._laser_holding = False
        if not self.laser_enabled:
            return cmd

        in_recover = self._drive_mode in (MODE_RECOVER_FORWARD, MODE_RECOVER_REVERSE)
        recover_sign = self._recover_dir_sign()
        # 夹角倒车与脱困倒车：保住负线速度；回正 REORIENT：保住前进
        keep_reverse = (
            want_reverse
            or self._drive_mode == MODE_REVERSE
            or recover_sign < 0.0
        )
        keep_forward = recover_sign > 0.0 or self._drive_mode == MODE_REORIENT
        keep_sign = -1.0 if keep_reverse else (1.0 if keep_forward else 0.0)

        # ---- 保住意图方向：激光只减速，不翻符号 ----
        if keep_sign != 0.0 and (
            in_recover
            or keep_reverse
            or self._drive_mode == MODE_REVERSE
            or self._drive_mode == MODE_REORIENT
        ):
            if self._laser_escape_until is not None:
                if float(self._laser_escape_sign) * keep_sign <= 0.0:
                    self._laser_escape_until = None
                elif now < self._laser_escape_until:
                    esc = Twist()
                    esc.linear.x = abs(self.laser_escape_speed) * keep_sign
                    esc.angular.z = (
                        self._laser_escape_omega_cmd
                        if abs(self._laser_escape_omega_cmd) > 1e-6
                        else cmd.angular.z
                    )
                    return esc
                else:
                    self._laser_escape_until = None

            if keep_sign < 0.0:
                dist = self._laser_obstacle_dist(self._rear_min, ignore_body_echo=True)
                stop_d = self.laser_rear_stop
                slow_d = self.laser_rear_slow
            else:
                dist = self._laser_obstacle_dist(self._front_min, ignore_body_echo=False)
                stop_d = self.laser_front_stop
                slow_d = self.laser_front_slow

            speed = abs(cmd.linear.x)
            if speed < 1e-3:
                speed = max(self.laser_escape_speed, self.min_linear_speed)
            # 意图方向真实近障：最多降速，绝不翻成对向；车体回波已在扇区距离中忽略
            if math.isfinite(dist) and dist < slow_d:
                if dist < stop_d:
                    scale = 0.55 if in_recover else 0.45
                else:
                    scale = max(
                        0.40, (dist - stop_d) / max(1e-3, slow_d - stop_d)
                    )
                floor_v = (
                    max(self.min_linear_speed, 0.22)
                    if in_recover
                    else self.min_linear_speed * 0.85
                )
                speed = max(speed * scale, floor_v)
                rospy.loginfo_throttle(
                    1.0,
                    'Laser keep-dir: mode=%s v_sign=%+.0f |v|=%.2f dist=%.2f',
                    self._drive_mode,
                    keep_sign,
                    speed,
                    dist,
                )
            cmd.linear.x = keep_sign * speed
            return cmd

        if self._laser_escape_until is not None and now < self._laser_escape_until:
            esc = Twist()
            esc.linear.x = self.laser_escape_speed * float(self._laser_escape_sign)
            esc.angular.z = self._laser_escape_omega_cmd
            return esc
        self._laser_escape_until = None

        going_reverse = want_reverse or (cmd.linear.x < -1e-6)
        # 前进时忽略后向停车，避免开阔弯道/走廊被身后墙误刹
        if going_reverse:
            stop_d = self.laser_rear_stop
            slow_d = self.laser_rear_slow
            dist = self._laser_obstacle_dist(self._rear_min, ignore_body_echo=True)
            other_dist = self._laser_obstacle_dist(self._front_min, ignore_body_echo=False)
            other_stop = self.laser_front_stop
        else:
            stop_d = self.laser_front_stop
            slow_d = self.laser_front_slow
            dist = self._laser_obstacle_dist(self._front_min, ignore_body_echo=False)
            other_dist = self._laser_obstacle_dist(self._rear_min, ignore_body_echo=True)
            other_stop = self.laser_rear_stop

        if not math.isfinite(dist):
            return cmd

        if dist < stop_d:
            # 对侧更空 → 换向逃离 + 打角（非意图倒车/脱困时才允许翻符号）
            other_ok = (not math.isfinite(other_dist)) or (
                other_dist > max(other_stop * 0.85, dist + 0.05)
            )
            if other_ok and self.allow_reversing:
                self._laser_escape_sign = 1 if going_reverse else -1
                escape_reverse = self._laser_escape_sign < 0
                esc_steer = self._path_steer_sign(steering_angle, escape_reverse)
                if esc_steer == 0.0:
                    esc_steer = 1.0 if steering_angle >= 0 else -1.0
                self._laser_escape_omega_cmd = esc_steer * self.laser_escape_omega
                self._laser_escape_until = now + rospy.Duration.from_sec(
                    self.laser_escape_duration
                )
                rospy.logwarn_throttle(
                    1.0,
                    'Laser escape: dist=%.2f → v_sign=%d ω=%+.2f',
                    dist,
                    self._laser_escape_sign,
                    self._laser_escape_omega_cmd,
                )
                esc = Twist()
                esc.linear.x = self.laser_escape_speed * float(self._laser_escape_sign)
                esc.angular.z = self._laser_escape_omega_cmd
                return esc

            # 双侧都近：绝不全零 —— 原地朝绿线拧；对侧略空则带一点线速度
            spin = self._spin_toward_path(want_reverse, steering_angle)
            if math.isfinite(other_dist) and other_dist > dist + 0.02:
                if going_reverse:
                    spin.linear.x = self.laser_escape_speed * 0.7
                else:
                    spin.linear.x = -self.laser_escape_speed * 0.7
            rospy.logwarn_throttle(
                1.0,
                'Laser blocked → spin/creep v=%.2f ω=%.2f (dist=%.2f recover=%d)',
                spin.linear.x,
                spin.angular.z,
                dist,
                1 if in_recover else 0,
            )
            self._laser_holding = abs(spin.linear.x) < 1e-3
            return spin

        if dist < slow_d:
            scale = max(0.25, (dist - stop_d) / max(1e-3, slow_d - stop_d))
            cmd.linear.x *= scale
            # 关键：减速时不削弱角速度（弯道贴障更需要转）
            rospy.loginfo_throttle(
                1.5,
                'Laser slow v_scale=%.2f dist=%.2f (%s) ω kept=%.2f',
                scale,
                dist,
                'rev' if going_reverse else 'fwd',
                cmd.angular.z,
            )
        return cmd

    def _avoid_weight(self, dist):
        """距离 → [0,1]：≤near 满偏置，≥far 为 0，中间线性。"""
        if not math.isfinite(dist):
            return 0.0
        near = self.avoid_steer_near
        far = max(near + 1e-3, self.avoid_steer_far)
        if dist >= far:
            return 0.0
        if dist <= near:
            return 1.0
        return (far - dist) / (far - near)

    def _compute_avoid_steer_bias(self, want_reverse):
        """
        瞬时避障转向力：左前/右前（倒车时左后/右后）近障 → 小幅 ω 偏置远离障碍。
        不改路径；障碍消失后偏置归零，回到纯追踪。力故意小，不抢主控。
        """
        if not self.avoid_steer_enabled or not self.laser_enabled:
            return 0.0
        if want_reverse or self._drive_mode in (
            MODE_REVERSE,
            MODE_RECOVER_REVERSE,
        ):
            left_d = self._lr_min
            right_d = self._rr_min
            side = 'rear'
        else:
            left_d = self._lf_min
            right_d = self._rf_min
            side = 'front'
        w_l = self._avoid_weight(left_d)
        w_r = self._avoid_weight(right_d)
        # 左侧近 → 负 ω（右转远离）；右侧近 → 正 ω（左转远离）
        bias = self.avoid_steer_omega_max * (w_r - w_l)
        # 硬帽：不超过设定小幅上限（再由外层 clamp 到 omega_max）
        bias = max(-self.avoid_steer_omega_max, min(self.avoid_steer_omega_max, bias))
        if abs(bias) > 0.02:
            rospy.loginfo_throttle(
                0.8,
                'AvoidSteer %s L=%.2f R=%.2f → bias=%+.2f (cap=%.2f)',
                side,
                left_d if math.isfinite(left_d) else -1.0,
                right_d if math.isfinite(right_d) else -1.0,
                bias,
                self.avoid_steer_omega_max,
            )
        return bias

    def _apply_avoid_steer(self, cmd, want_reverse):
        bias = self._compute_avoid_steer_bias(want_reverse)
        self._last_avoid_bias = bias
        if abs(bias) < 1e-4:
            return cmd
        cmd.angular.z = max(
            -self.omega_max, min(self.omega_max, cmd.angular.z + bias)
        )
        return cmd

    def control_cb(self, _event):
        if not self.path_received or self.path_points is None or len(self.path_points) == 0:
            return
        pose = [self.pose_x, self.pose_y, self.pose_yaw]
        steering_angle, _, x_b = self.pure_pursuit.calculate_steering_angle(
            pose, self.path_points
        )
        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        now = rospy.Time.now()

        cmd = Twist()
        if distance_to_end < 0.2:
            self.path_received = False
            self.reversing = False
            self._drive_mode = MODE_FORWARD
            self._recover_until = None
            self._laser_escape_until = None
            self._intent_moving = False
            self._laser_holding = False
            self._pose_hist.clear()
            self._path_goal_xy = None
            self._reverse_seg_start = None
            self._reverse_seg_xy = None
            self._reorient_until = None
            self._angle_rev_suppress = False
            self._reorient_unlock_xy = None
        else:
            self._update_reverse_mode(steering_angle, x_b)
            tracking_reverse = bool(
                self.allow_reversing and self.allow_angle_reverse and self.reversing
            )
            self._update_stuck_fsm(now, tracking_reverse, steering_angle)
            self._update_reorient(now)
            # 锁定期已结束但仍在抑制时，持续尝试解锁
            if self._angle_rev_suppress and self._drive_mode == MODE_FORWARD:
                self._try_clear_angle_rev_suppress(steering_angle)

            if self._drive_mode == MODE_RECOVER_FORWARD:
                want_reverse = False
            elif self._drive_mode == MODE_RECOVER_REVERSE:
                want_reverse = True
            elif self._drive_mode == MODE_REORIENT:
                want_reverse = False
            else:
                want_reverse = tracking_reverse

            speed, omega_cap, kappa, alpha_eff = self._curvature_speeds(
                steering_angle, want_reverse
            )
            if self._drive_mode in (MODE_RECOVER_FORWARD, MODE_RECOVER_REVERSE):
                speed = max(speed, min(0.40, self.max_linear_speed * 0.60))
            elif self._drive_mode == MODE_REORIENT:
                # 回正段略抬前进速度，便于掉头沿绿线正向走
                speed = max(speed, min(self.max_linear_speed * 0.55, 0.45))

            # REORIENT：绿线侧 + 左右雷达开阔 → 最终打方向（可贴墙侧覆盖绿线）
            steer_angle_cmd = steering_angle
            reorient_pick_s = 0.0
            if self._drive_mode == MODE_REORIENT:
                reorient_pick_s, _, _, _ = self._pick_reorient_steer_sign(
                    steering_angle
                )
                if reorient_pick_s != 0.0:
                    # 强制 α 符号与最终侧一致，避免 PP ω 与雷达偏好打架
                    mag = max(abs(steering_angle), math.radians(55.0))
                    steer_angle_cmd = math.copysign(mag, reorient_pick_s)
                    alpha_eff = steer_angle_cmd

            signed_v = -speed if want_reverse else speed
            cmd.linear.x = signed_v
            cmd.angular.z = self._compute_omega(
                steer_angle_cmd, signed_v, omega_cap, alpha_eff
            )

            if self._drive_mode in (
                MODE_RECOVER_FORWARD,
                MODE_RECOVER_REVERSE,
                MODE_REORIENT,
            ):
                if self._drive_mode == MODE_REORIENT and reorient_pick_s != 0.0:
                    steer_s = reorient_pick_s
                else:
                    steer_s = self._path_steer_sign(steering_angle, want_reverse)
                if steer_s != 0.0:
                    boost = steer_s * self.stuck_steer_omega
                    if cmd.angular.z * boost >= 0.0:
                        cmd.angular.z = math.copysign(
                            max(abs(cmd.angular.z), abs(boost)), boost
                        )
                    else:
                        cmd.angular.z = boost
                    cmd.angular.z = max(
                        -self.omega_max, min(self.omega_max, cmd.angular.z)
                    )

            # 意图在激光之前记录，卡住检测才能在 STOP/holding 时触发
            self._intent_moving = abs(cmd.linear.x) > 1e-3 or abs(cmd.angular.z) > 1e-3
            cmd = self._apply_laser_safety(cmd, now, want_reverse, steer_angle_cmd)

            # 瞬时避障转向力：叠在最终 ω 上（路径不变；力小不抢主控）
            cmd = self._apply_avoid_steer(cmd, want_reverse)

            # 最终兜底：有路径误差时禁止全零
            if (
                abs(steer_angle_cmd) > 0.15
                and abs(cmd.linear.x) < 1e-4
                and abs(cmd.angular.z) < 1e-4
            ):
                cmd = self._spin_toward_path(want_reverse, steer_angle_cmd)
                self._laser_holding = True

            rospy.loginfo_throttle(
                1.0,
                'PP α=%.2f(%.0f°) x_b=%.2f κ=%.2f mode=%s v=%.2f ω=%.2f '
                'front=%.2f rear=%.2f hold=%d avoid=%+.2f lock=%d',
                steering_angle,
                math.degrees(steering_angle),
                x_b,
                kappa,
                self._drive_mode,
                cmd.linear.x,
                cmd.angular.z,
                self._front_min if math.isfinite(self._front_min) else -1.0,
                self._rear_min if math.isfinite(self._rear_min) else -1.0,
                1 if self._laser_holding else 0,
                self._last_avoid_bias,
                1 if (self._drive_mode == MODE_REORIENT or self._angle_rev_suppress) else 0,
            )

        self.cmd_vel_pub.publish(cmd)


def main():
    node = PurePursuitNavNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
