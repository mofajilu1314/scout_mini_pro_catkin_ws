"""安全监测：翻车/卡死/碰撞（GUI 侧逻辑；翻车事件主要靠 EventMonitor 读 launch 日志）。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class AlarmLevel(Enum):
    INFO = "info"
    WARN = "warn"
    CRITICAL = "critical"


@dataclass
class Alarm:
    level: AlarmLevel
    message: str
    timestamp: float


class SafetyMonitor:
    def __init__(
        self,
        stuck_time_sec: float = 8.0,
        stuck_dist_m: float = 0.25,
        collision_warn_dist: float = 0.5,
        rollover_roll_deg: float = 55.0,
        rollover_pitch_deg: float = 55.0,
    ) -> None:
        self.stuck_time_sec = stuck_time_sec
        self.stuck_dist_m = stuck_dist_m
        self.collision_warn_dist = collision_warn_dist
        self.rollover_roll_deg = rollover_roll_deg
        self.rollover_pitch_deg = rollover_pitch_deg
        self._alarms: list[Alarm] = []
        self._anchor: Optional[Tuple[float, float, float]] = None  # x,y,t0
        self._was_stuck = False

    def check_rollover(self, roll_deg: float, pitch_deg: float) -> Optional[Alarm]:
        if abs(roll_deg) >= self.rollover_roll_deg or abs(pitch_deg) >= self.rollover_pitch_deg:
            a = Alarm(
                AlarmLevel.CRITICAL,
                f"翻车风险 roll={roll_deg:.1f}° pitch={pitch_deg:.1f}°",
                time.time(),
            )
            self._alarms.append(a)
            return a
        return None

    def update_odom(self, x: float, y: float, commanding: bool) -> Optional[Alarm]:
        """位移窗口卡死检测。commanding=True 表示正在发非零速度。"""
        now = time.time()
        if not commanding:
            self._anchor = (x, y, now)
            if self._was_stuck:
                self._was_stuck = False
                return Alarm(AlarmLevel.INFO, "卡死解除", now)
            return None
        if self._anchor is None:
            self._anchor = (x, y, now)
            return None
        ax, ay, t0 = self._anchor
        dist = math.hypot(x - ax, y - ay)
        if dist >= self.stuck_dist_m:
            self._anchor = (x, y, now)
            if self._was_stuck:
                self._was_stuck = False
                return Alarm(AlarmLevel.INFO, "已脱离卡死", now)
            return None
        if (now - t0) >= self.stuck_time_sec:
            if not self._was_stuck:
                self._was_stuck = True
                a = Alarm(
                    AlarmLevel.CRITICAL,
                    f"卡死：{self.stuck_time_sec:.1f}s 内位移仅 {dist:.2f}m",
                    now,
                )
                self._alarms.append(a)
                return a
        return None

    def check_collision_risk(self, front_min_dist: float) -> Optional[Alarm]:
        if front_min_dist < self.collision_warn_dist:
            a = Alarm(
                AlarmLevel.WARN,
                f"碰撞风险：前方 {front_min_dist:.2f}m（仅报警）",
                time.time(),
            )
            self._alarms.append(a)
            return a
        return None

    def check_heartbeat(self, last_heartbeat: float) -> Optional[Alarm]:
        if time.time() - last_heartbeat > 3.0:
            a = Alarm(AlarmLevel.CRITICAL, "心跳超时", time.time())
            self._alarms.append(a)
            return a
        return None

    def alarms(self) -> list[Alarm]:
        return self._alarms
