"""ROS1/ROS2 桥接：根据模式选择。"""

from __future__ import annotations

from enum import Enum
from typing import Any


class ROSMode(Enum):
    SIM = "sim"  # ROS2 Humble
    REAL = "real"  # ROS1 Noetic via SSH


class ROSBridge:
    def __init__(self, mode: ROSMode) -> None:
        self.mode = mode
        self._node: Any = None

    def start(self) -> None:
        if self.mode == ROSMode.SIM:
            self._start_ros2()
        else:
            self._start_ros1_ssh()

    def _start_ros2(self) -> None:
        # TODO: rclpy init + 订阅/发布
        pass

    def _start_ros1_ssh(self) -> None:
        # TODO: SSH 到车上，启动 launch，本机 ROS_MASTER_URI 连车
        pass

    def publish_cmd_vel(self, v: float, w: float) -> None:
        # TODO: 发 Twist
        pass

    def estop(self) -> None:
        self.publish_cmd_vel(0.0, 0.0)
