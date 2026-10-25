"""仿真资源冲突检测：Gazebo / RViz / 训练 / long_nav。"""

from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass
from typing import List, Optional

from .workspace_helper import append_debug_event


@dataclass
class SimConflict:
    kind: str
    line: str  # "pid cmd..."

    @property
    def pid(self) -> str:
        return self.line.split(" ", 1)[0] if self.line else ""


# (pgrep -f pattern, 显示名)
_CONFLICT_PATTERNS = (
    (r"(^|/)gzserver( |$)", "Gazebo Server (gzserver)"),
    (r"(^|/)gzclient( |$)", "Gazebo GUI (gzclient)"),
    (r"ruby.*/usr/bin/gz", "Gazebo (gz)"),
    (r"(^|/)rviz2( |$)", "RViz2"),
    (r"train_td3", "TD3 训练 (train_td3)"),
    (r"ros2 launch scout_long_nav", "long_nav launch"),
    (r"scout_long_nav/", "long_nav 节点"),
    (r"flysky_js_teleop", "Flysky 遥控节点"),
    (r"infer_td3", "TD3 推理"),
    (r"spawn_scout_clean_prefix", "Scout 生成"),
)


def _proc_state(pid: int) -> Optional[str]:
    """Return single-letter state from /proc/<pid>/stat, or None if gone."""
    try:
        with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as f:
            raw = f.read()
        # comm may contain spaces/parens: "pid (comm) STATE ..."
        rparen = raw.rfind(")")
        if rparen < 0:
            return None
        return raw[rparen + 2 : rparen + 3]
    except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
        return None


def is_zombie_pid(pid: int) -> bool:
    return _proc_state(pid) == "Z"


def reap_zombie_children() -> int:
    """Reap exited children of this process (zombies cannot be killed)."""
    n = 0
    while True:
        try:
            pid, _status = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            break
        if pid <= 0:
            break
        n += 1
    return n


def _agent_log(hid: str, location: str, message: str, data=None) -> None:
    payload = dict(data or {})
    payload["event_id"] = hid
    append_debug_event(location, message, payload)


def _pgrep_af(pattern: str) -> List[str]:
    try:
        out = subprocess.check_output(
            ["pgrep", "-a", "-f", pattern],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        return []
    lines = []
    for ln in out.splitlines():
        ln = ln.strip()
        if not ln or "pgrep" in ln:
            continue
        # 别把 ScoutNav 自己算进去
        if "scout_nav_app" in ln and "scout_long_nav" not in ln:
            continue
        lines.append(ln)
    return lines


def list_sim_conflicts() -> List[SimConflict]:
    """列出当前占用仿真槽位的进程（去重 by pid；忽略僵尸）。"""
    # 先回收本进程的僵尸子进程，避免误报
    reaped = reap_zombie_children()
    seen = set()
    out: List[SimConflict] = []
    skipped_z = []
    for pat, kind in _CONFLICT_PATTERNS:
        for ln in _pgrep_af(pat):
            pid_s = ln.split(" ", 1)[0]
            if pid_s in seen:
                continue
            try:
                pid_i = int(pid_s)
            except ValueError:
                continue
            if is_zombie_pid(pid_i):
                skipped_z.append({"pid": pid_i, "kind": kind})
                continue
            seen.add(pid_s)
            out.append(SimConflict(kind=kind, line=ln))
    _agent_log(
        "Z",
        "sim_conflict.list_sim_conflicts",
        "scan",
        {"n": len(out), "reaped": reaped, "skipped_zombies": skipped_z[:10]},
    )
    return out


def format_conflicts(conflicts: List[SimConflict], limit: int = 24) -> str:
    if not conflicts:
        return "(无)"
    rows = []
    for c in conflicts[:limit]:
        cmd = c.line.split(" ", 1)[1] if " " in c.line else c.line
        if len(cmd) > 100:
            cmd = cmd[:97] + "..."
        rows.append(f"• [{c.kind}] pid={c.pid}\n  {cmd}")
    if len(conflicts) > limit:
        rows.append(f"… 另有 {len(conflicts) - limit} 个进程未列出")
    return "\n".join(rows)
