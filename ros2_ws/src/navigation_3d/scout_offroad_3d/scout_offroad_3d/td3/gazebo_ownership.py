"""Pure procfs checks for the TD3 initial-Gazebo ownership handoff.

Kept separate from the ROS backend so the safety invariant can be tested on a
plain filesystem without importing ROS messages or starting Gazebo.
"""

from __future__ import annotations

import os
import re
from typing import List, Optional, Tuple


GAZEBO_OWNER_TOKEN_ENV = "SCOUT_GAZEBO_OWNER_TOKEN"


def listener_pids_from_ss(details: str) -> List[int]:
    """Extract unique process IDs from ``ss -lntp`` listener diagnostics."""
    return sorted({int(pid) for pid in re.findall(r"pid=(\d+)", details or "")})


def read_proc_bytes(pid: int, name: str, proc_root: str = "/proc") -> Optional[bytes]:
    """Read a small procfs identity field, returning None on any race/error."""
    try:
        with open(os.path.join(proc_root, str(int(pid)), name), "rb") as f:
            return f.read()
    except (OSError, ValueError):
        return None


def proc_start_ticks(pid: int, proc_root: str = "/proc") -> Optional[str]:
    """Return Linux /proc start-time ticks (field 22), robust to spaces in comm."""
    raw = read_proc_bytes(pid, "stat", proc_root)
    if not raw:
        return None
    try:
        # stat's comm is parenthesized and may contain spaces/parentheses; the
        # fields after its final ')' begin at field 3, so starttime is index 19.
        fields = raw.decode("utf-8", "replace").rsplit(")", 1)[1].split()
        return fields[19]
    except (IndexError, ValueError):
        return None


def handoff_gzserver_identity(
    pid: int, owner_token: str, proc_root: str = "/proc"
) -> Optional[Tuple[int, str]]:
    """Return a stable identity only for the explicitly handed-off gzserver.

    PID and command name are both checked before the token.  The start time is
    retained so a PID reuse between TERM and KILL cannot target a new process.
    ``proc_root`` exists solely to make this safety check unit-testable without
    a real Gazebo listener.
    """
    if not owner_token:
        return None
    cmdline = read_proc_bytes(pid, "cmdline", proc_root)
    environ = read_proc_bytes(pid, "environ", proc_root)
    start_ticks = proc_start_ticks(pid, proc_root)
    if not cmdline or environ is None or not start_ticks:
        return None
    argv = [
        item.decode("utf-8", "replace")
        for item in cmdline.split(b"\0")
        if item
    ]
    if not argv or not any(os.path.basename(item) == "gzserver" for item in argv):
        return None
    expected = (GAZEBO_OWNER_TOKEN_ENV + "=" + owner_token).encode("utf-8")
    if expected not in environ.split(b"\0"):
        return None
    return int(pid), start_ticks
