"""运行时安全事件监视：翻车/卡死/碰撞/遥控器 → 终端告警。"""

from __future__ import annotations

import os
import re
import time
from typing import Callable, Dict, Optional, Tuple

from PySide6.QtCore import QObject, QTimer, Signal

from .dep_check import _find_joy_device
from .workspace_helper import append_debug_event, launch_log_paths


LogFn = Callable[[str], None]


class EventMonitor(QObject):
    """轮询 launch 日志 + 本机摇杆；不阻塞 GUI。"""

    event = Signal(str)  # 告警文本

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._enabled = {"rollover": True, "stuck": True, "collision": True, "joy": True}
        # New launches write under 3D/debug_logs.  App-local .last_* files are
        # read only as a migration fallback so an existing session is not lost.
        self._log_paths = launch_log_paths()
        self._offsets: Dict[str, int] = {}
        self._joy_ok: Optional[bool] = None
        self._last_odom: Optional[Tuple[float, float, float]] = None  # x,y,t
        self._stuck_since: Optional[float] = None
        self._last_collision_warn = 0.0
        self._last_unstuck = 0.0
        self._last_success_emit = 0.0
        self._active = False
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._tick)

    def set_enabled(self, key: str, enabled: bool) -> None:
        if key in self._enabled:
            self._enabled[key] = enabled

    def start(self) -> None:
        self._active = True
        # 从文件末尾开始，避免刷历史
        for p in self._log_paths:
            try:
                self._offsets[str(p)] = p.stat().st_size if p.exists() else 0
            except OSError:
                self._offsets[str(p)] = 0
        if not self._timer.isActive():
            self._timer.start()
        self._agent("start", {"paths": [str(p) for p in self._log_paths]})

    def stop(self) -> None:
        self._active = False
        self._timer.stop()

    def _agent(self, message: str, data: dict) -> None:
        append_debug_event("event_monitor", message, data)

    def _emit(self, text: str) -> None:
        self.event.emit(text)
        self._agent("event", {"text": text[:200]})

    def _tick(self) -> None:
        if not self._active:
            return
        self._scan_logs()
        if self._enabled.get("joy", True):
            self._check_joy()

    def _scan_logs(self) -> None:
        for p in self._log_paths:
            key = str(p)
            if not p.exists():
                continue
            try:
                size = p.stat().st_size
                off = self._offsets.get(key, 0)
                if size < off:
                    off = 0
                if size == off:
                    continue
                with p.open("r", errors="replace") as f:
                    f.seek(off)
                    chunk = f.read()
                    self._offsets[key] = f.tell()
            except OSError:
                continue
            self._parse_chunk(chunk)

    def _parse_chunk(self, chunk: str) -> None:
        for line in chunk.splitlines():
            if self._enabled.get("rollover", True):
                if "ROLLOVER DETECTED" in line or "FORCE RECOVER" in line:
                    if "FORCE RECOVER" in line:
                        self._emit("⚠⚠ 脱离卡死：正在原地重生并解锁遥控…")
                    else:
                        self._emit("⚠⚠ 小车翻车！正在自动调整姿态（Z 阶梯重生）…")
                elif "rollover recovery DONE" in line or "teleop_mute/cmd_hold OFF burst" in line:
                    self._emit("✓ 姿态已调整，遥控已解锁（可继续推杆）")
                elif "teleop_mute OFF" in line:
                    self._emit("✓ 遥控器已解除静音")
                elif "tilt watch" in line and "RECOVER" in line:
                    self._emit("⚠ 翻车/脱困恢复中…")
                elif "NAV ABORT(security)" in line or "穿过安全红线" in line:
                    self._emit("⚠⚠ 安全边界：规划路径会穿红线，本次导航已停止")
                elif "GOAL REJECTED(security)" in line or "新终点会被安全红线隔开" in line:
                    self._emit("⚠ 新终点会穿红线：已拒绝切换，保持当前导航/待机")
                elif "本次导航任务完成 SUCCESS" in line or "GOAL REACHED" in line:
                    now = time.time()
                    if now - self._last_success_emit > 3.0:
                        self._last_success_emit = now
                        self._emit("✓ 本次导航任务完成 SUCCESS（到点停航，可重新 Set Goal）")
                elif "cancel_nav" in line and "非穿红线" in line:
                    self._emit("· 导航任务已取消（重生/手动），请重新 Set Goal")
            if self._enabled.get("collision", True):
                # 预留：若 launch 打出 collision 关键词
                if re.search(r"\bCOLLISION\b|collision risk|碰撞", line, re.I):
                    now = time.time()
                    if now - self._last_collision_warn > 3.0:
                        self._last_collision_warn = now
                        self._emit("⚠ 碰撞风险：请注意（仅报警，不停车）")

    def _check_joy(self) -> None:
        joy = _find_joy_device()
        ok = bool(joy)
        if self._joy_ok is None:
            self._joy_ok = ok
            if ok:
                self._emit(f"✓ 遥控器已连接: {joy}")
            else:
                self._emit("⚠ 未检测到遥控器 USB（Flysky/js*）。建图请插上遥控并开机")
            return
        if ok and not self._joy_ok:
            self._joy_ok = True
            self._emit(f"✓ 遥控器已连接: {joy}")
        elif (not ok) and self._joy_ok:
            self._joy_ok = False
            self._emit("⚠ 遥控器已断开！请检查 USB / 遥控电源")

    def notify_stuck(self, stuck: bool) -> None:
        """由外部 odom 监视调用。"""
        if not self._enabled.get("stuck", True):
            return
        if stuck:
            self._emit("⚠ 疑似卡死：位移过小。可点「脱离卡死」尝试倒车脱困")
        else:
            self._emit("✓ 已脱离卡死（位移恢复）")

    def notify_collision_risk(self) -> None:
        if not self._enabled.get("collision", True):
            return
        now = time.time()
        if now - self._last_collision_warn < 3.0:
            return
        self._last_collision_warn = now
        self._emit("⚠ 碰撞风险：前方过近，请注意（仅报警，不停车）")
