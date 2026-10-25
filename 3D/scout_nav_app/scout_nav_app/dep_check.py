"""启动时检测本机仿真依赖（ROS2 / Gazebo / RViz / Python 包）。"""

from __future__ import annotations

import importlib.util
import os
import shutil
import sys
from dataclasses import dataclass
from typing import List, Tuple

from .workspace_helper import WorkspaceRootError, append_debug_event, workspace_file


@dataclass
class CheckItem:
    name: str
    ok: bool
    detail: str


def _which(name: str) -> str:
    return shutil.which(name) or ""


def _has_module(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def run_dep_checks() -> List[CheckItem]:
    items: List[CheckItem] = []
    # Python / GUI
    items.append(CheckItem("Python3", True, f"{sys.version.split()[0]}"))
    items.append(
        CheckItem(
            "PySide6",
            _has_module("PySide6"),
            "已安装" if _has_module("PySide6") else "缺失：pip install PySide6",
        )
    )
    # ROS2 / tools
    ros2 = _which("ros2")
    items.append(CheckItem("ros2 CLI", bool(ros2), ros2 or "未在 PATH（需 source /opt/ros/humble）"))
    gz = _which("gzserver") or _which("gazebo")
    items.append(CheckItem("Gazebo", bool(gz), gz or "未找到 gzserver/gazebo"))
    rviz = _which("rviz2")
    items.append(CheckItem("RViz2", bool(rviz), rviz or "未找到 rviz2"))
    # scout_long_nav share — derive it from the active workspace, never a home path.
    share = None
    share_error = ""
    try:
        share = workspace_file(
            "ros2_ws", "install", "scout_long_nav", "share", "scout_long_nav"
        )
    except WorkspaceRootError as exc:
        share_error = f"无法定位工作区：{exc}"
    items.append(
        CheckItem(
            "scout_long_nav",
            bool(share and share.is_dir()),
            (
                str(share)
                if share and share.is_dir()
                else share_error or "未找到 install/scout_long_nav（请在 ros2_ws 中 colcon build）"
            ),
        )
    )
    # torch optional for model load
    items.append(
        CheckItem(
            "torch(可选)",
            _has_module("torch"),
            "已安装" if _has_module("torch") else "未装：仅影响本机直接读权重校验",
        )
    )
    # Flysky device (info, not hard fail)
    joy = _find_joy_device()
    items.append(
        CheckItem(
            "遥控器 USB",
            bool(joy),
            joy or "未检测到 Flysky/js*（建图遥控前请插上并开机）",
        )
    )
    return items


def _find_joy_device() -> str:
    import glob

    by_id = sorted(glob.glob("/dev/input/by-id/*Flysky*-joystick"))
    by_id = [p for p in by_id if "event" not in os.path.basename(p)]
    if by_id:
        return os.path.realpath(by_id[0])
    for i in range(4):
        p = f"/dev/input/js{i}"
        if os.path.exists(p):
            return p
    return ""


def format_report(items: List[CheckItem]) -> Tuple[bool, str]:
    lines = ["======== 依赖自检 ========"]
    hard = {"Python3", "PySide6", "ros2 CLI", "Gazebo", "RViz2", "scout_long_nav"}
    all_hard_ok = True
    for it in items:
        mark = "✓" if it.ok else ("✗" if it.name in hard else "⚠")
        if it.name in hard and not it.ok:
            all_hard_ok = False
        lines.append(f"{mark} {it.name}: {it.detail}")
    lines.append("========================")
    return all_hard_ok, "\n".join(lines)


def agent_log_deps(items: List[CheckItem]) -> None:
    append_debug_event(
        "dep_check.run_dep_checks",
        "startup_deps",
        {it.name: {"ok": it.ok, "detail": it.detail} for it in items},
    )
