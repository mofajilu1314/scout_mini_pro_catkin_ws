# -*- coding: utf-8 -*-
"""Hard isolation helpers: long-nav must not share Gazebo with TD3 training."""

from __future__ import annotations

import os
import subprocess
from typing import List, Optional, Tuple

from scout_workspace_common import resolve_workspace_root


WORKSPACE_ROOT = resolve_workspace_root(package_name="scout_long_nav")
DEFAULT_WEIGHTS_PREFIX = str(
    WORKSPACE_ROOT
    / "archive"
    / "experiments"
    / "td3"
    / "runs"
    / "run_20260907_120841"
    / "td3"
)
DEFAULT_MAP_DIR = str(WORKSPACE_ROOT / "3D" / "maps_long")
DEFAULT_MAP_STEM = "arena_trav"
ARENA_WORLD = str(
    WORKSPACE_ROOT
    / "ros2_ws"
    / "src"
    / "navigation_3d"
    / "scout_offroad_3d"
    / "arena"
    / "bumpy_offroad_full.world"
)

ISOLATION_BANNER = """
============================================================
[scout_long_nav] 与 TD3 训练硬隔离
  - 禁止与 train_td3.launch 同时开
  - 长导航只用 arena 大图，不用 worlds/bumpy_offroad_{2,3,4}
  - 禁止写入 3D/runs/ 训练目录
  - 导航只读 Record 9 仿真权重（不可直接用于实车）
  - 整机只能有一台 gzserver（端口 11345）
============================================================
"""

MAP_PHASE_BANNER = """
************************************************************
【阶段1 建图】你会看到：
  1) Gazebo 窗口 = arena 大图 + Scout 小车
  2) RViz 窗口 = CostmapHeat + CameraRGB + 车模型
  3) 默认 = Flysky USB 遥控仿真车（无键盘 xterm）
  4) 翻车监护：停图/静音 → 清半径 → Z 阶梯重生

安全（必读）：
  - 控仿真前请关真车电源；USB 只进电脑，拦不住遥控对真车的无线电
  - 本链路只发仿真 /cmd_vel，不碰 CAN / 实车 bringup
  - 屏幕 Signal lost = 找不到真车接收机（关车时正常），可忽略

操作：
  - Flysky：遥控开机 + USB 插稳 → 直接推油门/转向（不需使能键）
  - 可选键盘：use_keyboard:=true；续建：seed_map_yaml:=…/arena_trav.yaml
  - 开够了 → 本终端 Ctrl+C → 自动存
      $SCOUT_WS_ROOT/3D/maps_long/arena_trav.yaml
  - 建议再拷一份 maps_long/map_时间戳/（见 README4 §0）

本阶段不起 A* / Arena / TD3，不会自动导航。
************************************************************
"""

NAV_PHASE_BANNER = """
************************************************************
【阶段2 导航】你会看到：
  1) Gazebo = 同一大图 + 小车
  2) RViz = 代价热力 + 鲜红 GlobalPath + Arena 彩色球

操作：
  - Fixed Frame = odom → 工具「2D Goal Pose」点终点
  - 红路径 = Hybrid A*；青→粉球 = Arena 局部点
  - 到终点≤2m 自动锁轮；再点一次 Goal Pose 才继续
  - 翻车：Z 阶梯重生后应解锁（日志 cmd_hold OFF）

不要用 U 盘 open_world / view_world（那些没有导航栈、没有车控）。
************************************************************
"""

AUTO_MAP_PHASE_BANNER = """
************************************************************
【自动覆盖建图】已弃用 — 请改用 long_nav_map.launch.py（Flysky 自驾）
  无人 TD3 扫图易乱撞、易把代价图标脏。除非你明确要试，否则不要开本 launch。
************************************************************
"""


def _pgrep(pattern: str) -> List[str]:
    try:
        out = subprocess.check_output(
            ["pgrep", "-a", "-f", pattern], text=True, stderr=subprocess.DEVNULL
        )
        lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
        return [ln for ln in lines if "pgrep" not in ln]
    except subprocess.CalledProcessError:
        return []


def check_training_idle(allow_gzserver: bool = False) -> Tuple[bool, str]:
    """
    Return (ok, message).
    Fails hard if train_td3 is running.
    Fails if gzserver is up unless allow_gzserver / LONG_NAV_ALLOW_GZ=1.
    """
    msgs: List[str] = []
    train_hits = _pgrep("train_td3")
    if train_hits:
        detail = "\n  ".join(train_hits[:8])
        return False, (
            "【拦住了】检测到 TD3 训练进程，请先停训再开长导航。\n"
            "  请在启动训练的终端正常停止它；不要按端口或进程名强杀未知进程。\n"
            "  进程：\n  " + detail
        )

    force = os.environ.get("LONG_NAV_FORCE", "").strip().lower() in ("1", "yes", "true")
    allow_gz = allow_gzserver or os.environ.get("LONG_NAV_ALLOW_GZ", "").strip().lower() in (
        "1",
        "yes",
        "true",
    )
    gz_hits = _pgrep("gzserver")
    if gz_hits and not allow_gz and not force:
        detail = "\n  ".join(gz_hits[:8])
        return False, (
            "【拦住了】已有 gzserver 在跑（可能是训练残留）。\n"
            "请确认归属，并在启动它的终端正常停止；不要按端口或进程名强杀未知进程。\n"
            "确认是本次长导航实例后才可使用 LONG_NAV_ALLOW_GZ=1。\n"
            "  进程：\n  " + detail
        )
    if gz_hits:
        msgs.append("警告：gzserver 仍在（已允许覆盖）。" + "; ".join(gz_hits[:3]))
    else:
        msgs.append("检查通过：无 train_td3，无 gzserver")
    return True, "\n".join(msgs)


def map_paths(map_dir: Optional[str] = None, stem: Optional[str] = None) -> Tuple[str, str]:
    d = map_dir or DEFAULT_MAP_DIR
    s = stem or DEFAULT_MAP_STEM
    return os.path.join(d, s + ".yaml"), os.path.join(d, s + ".pgm")
