# -*- coding: utf-8 -*-
"""Flysky FS-AGX USB HID → /cmd_vel（仅仿真，不碰 CAN）。

直接读 /dev/input/js*，不依赖 teleop_twist_joy 的使能键。
屏幕上的 Signal lost = 找不到真车接收机（关真车时正常），与 USB 控仿真无关。
"""

from __future__ import annotations

import glob
import os
import struct
import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def _find_flysky_js(prefer_name: str) -> Optional[str]:
    by_id = sorted(glob.glob("/dev/input/by-id/*Flysky*-joystick"))
    by_id = [p for p in by_id if "event" not in os.path.basename(p)]
    if by_id:
        return os.path.realpath(by_id[0])
    # fallback js0..js3
    for i in range(4):
        p = f"/dev/input/js{i}"
        if os.path.exists(p):
            return p
    return None


class FlyskyJsTeleop(Node):
    def __init__(self) -> None:
        super().__init__("flysky_js_teleop")
        self.declare_parameter("device_name_substr", "Flysky")
        self.declare_parameter("axis_linear", 2)  # Mode2 throttle CH3
        self.declare_parameter("axis_angular", 0)  # Mode2 aileron CH1
        self.declare_parameter("scale_linear", 1.0)  # 仿真满杆约 1 m/s（仅电脑端）
        self.declare_parameter("scale_angular", -1.2)  # 负号纠正转向；仅仿真
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("rate_hz", 20.0)
        # 默认不需要使能键；若 >0 则该按钮必须为 1
        self.declare_parameter("enable_button", -1)

        self._axis_lin = int(self.get_parameter("axis_linear").value)
        self._axis_ang = int(self.get_parameter("axis_angular").value)
        self._scale_lin = float(self.get_parameter("scale_linear").value)
        self._scale_ang = float(self.get_parameter("scale_angular").value)
        self._dead = float(self.get_parameter("deadzone").value)
        self._enable_btn = int(self.get_parameter("enable_button").value)

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._mute = False
        self.create_subscription(Bool, "/long_nav/teleop_mute", self._on_mute, 10)
        self._axes: List[float] = [0.0] * 8
        self._btns: List[int] = [0] * 16
        self._lock = threading.Lock()
        self._last_motion_t = 0.0
        self._opened = False
        self._dev_path = ""
        self._ever_moved = False

        self.get_logger().info(
            "Flysky USB→仿真 /cmd_vel | 忽略屏幕 Signal lost | "
            f"油门轴={self._axis_lin} 转向轴={self._axis_ang} | "
            "松杆回中持续发零速（锁轮，不对拨杆轴当油门）"
        )

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        period = 1.0 / max(float(self.get_parameter("rate_hz").value), 1.0)
        self.create_timer(period, self._tick)
        self.create_timer(3.0, self._heartbeat)

    def _reader_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            path = _find_flysky_js(str(self.get_parameter("device_name_substr").value))
            if not path:
                self.get_logger().warn("未找到 Flysky js 设备，2s 后重试…")
                time.sleep(2.0)
                continue
            try:
                fd = os.open(path, os.O_RDONLY)
            except OSError as e:
                self.get_logger().warn(f"打开 {path} 失败: {e}")
                time.sleep(2.0)
                continue
            self._dev_path = path
            self._opened = True
            self.get_logger().info(f"已打开 {path}")
            try:
                while not self._stop.is_set() and rclpy.ok():
                    data = os.read(fd, 8)
                    if len(data) < 8:
                        break
                    _t, val, typ, num = struct.unpack("IhBB", data)
                    et = typ & ~JS_EVENT_INIT
                    with self._lock:
                        if et == JS_EVENT_AXIS and num < len(self._axes):
                            v = max(-1.0, min(1.0, float(val) / 32767.0))
                            if abs(v - self._axes[num]) > 0.02:
                                if not (typ & JS_EVENT_INIT):
                                    first = not self._ever_moved
                                    self._ever_moved = True
                                    self._last_motion_t = time.time()
                                    if first:
                                        self.get_logger().info(
                                            f"收到 USB 推杆数据：轴{num}={v:.2f}"
                                        )
                            self._axes[num] = v
                        elif et == JS_EVENT_BUTTON and num < len(self._btns):
                            b = 1 if val else 0
                            if b != self._btns[num] and not (typ & JS_EVENT_INIT):
                                first = not self._ever_moved
                                self._ever_moved = True
                                self._last_motion_t = time.time()
                                if first:
                                    self.get_logger().info(f"收到 USB 开关：按钮{num}={b}")
                            self._btns[num] = b
            except OSError as e:
                self.get_logger().warn(f"读 {path} 中断: {e}")
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass
                self._opened = False
                time.sleep(1.0)

    def _on_mute(self, msg: Bool) -> None:
        was = self._mute
        self._mute = bool(msg.data)
        if self._mute and not was:
            self.get_logger().warn("teleop_mute ON（翻车恢复中，遥控无效）")
            self._pub.publish(Twist())
        elif (not self._mute) and was:
            self.get_logger().info("teleop_mute OFF（可继续遥控）")

    def _apply_dead(self, v: float) -> float:
        return 0.0 if abs(v) < self._dead else v

    def _tick(self) -> None:
        if self._mute:
            self._pub.publish(Twist())
            return
        with self._lock:
            axes = list(self._axes)
            btns = list(self._btns)
        if self._enable_btn >= 0:
            if self._enable_btn >= len(btns) or btns[self._enable_btn] == 0:
                # 仍发零速，避免差速控制器悬空
                self._pub.publish(Twist())
                return
        # 只用配置的油门/转向轴；杆回中 → 死区归零 → 持续发零速（对齐真车锁轮）
        # 禁止把拨杆轴(如 axis5 静止约 0.9)自动当油门，否则松杆仍会往前冲
        lin = ang = 0.0
        if self._axis_lin < len(axes):
            lin = self._apply_dead(axes[self._axis_lin]) * self._scale_lin
        if self._axis_ang < len(axes):
            ang = self._apply_dead(axes[self._axis_ang]) * self._scale_ang
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self._pub.publish(msg)

    def _heartbeat(self) -> None:
        with self._lock:
            axes = [round(a, 2) for a in self._axes[:6]]
            moved = self._ever_moved
            last = self._last_motion_t
        age = (time.time() - last) if last else -1.0
        if not self._opened:
            self.get_logger().warn("手柄未打开。检查 USB：ls /dev/input/by-id/*Flysky*")
            return
        if not moved:
            self.get_logger().warn(
                "已连上 USB，但摇杆还没有任何变化。"
                "请：①确认遥控已开机（电池）②换一根能传数据的 USB 线③拔插 USB；"
                "屏幕 Signal lost 可忽略（真车关着就会显示）。"
                f" 当前轴={axes}"
            )
        elif age > 5.0:
            self.get_logger().info(f"待机中（上次推杆 {age:.0f}s 前）轴={axes}")

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = FlyskyJsTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
