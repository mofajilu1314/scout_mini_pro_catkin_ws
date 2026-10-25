"""Workspace-aware paths for ScoutNav.

ScoutNav is usually launched directly from ``3D/scout_nav_app`` rather than
through a ROS console entry point.  This tiny bridge first imports the shared
``scout_workspace_common`` package when it is already available, then falls
back to locating that *source package* from this file or an explicitly set
workspace root.  All actual root discovery remains in the shared helper.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Optional


def _source_common_root() -> Optional[Path]:
    """Locate the shared helper source without a user-specific path."""
    for variable in ("SCOUT_WS_ROOT", "CATKIN_WS", "WS_ROOT"):
        value = os.environ.get(variable, "").strip()
        if not value:
            continue
        candidate = (
            Path(value).expanduser().resolve()
            / "ros2_ws"
            / "src"
            / "shared"
            / "scout_workspace_common"
        )
        if (candidate / "scout_workspace_common" / "workspace_paths.py").is_file():
            return candidate

    for parent in Path(__file__).resolve().parents:
        candidate = (
            parent / "ros2_ws" / "src" / "shared" / "scout_workspace_common"
        )
        if (candidate / "scout_workspace_common" / "workspace_paths.py").is_file():
            return candidate
    return None


def _import_shared_paths() -> tuple[Any, Any, Any, Any]:
    try:
        from scout_workspace_common import (
            WorkspaceRootError,
            debug_log_path,
            resolve_workspace_root,
            workspace_path,
        )

        return WorkspaceRootError, debug_log_path, resolve_workspace_root, workspace_path
    except ImportError as initial_error:
        source_root = _source_common_root()
        if source_root is None:
            raise RuntimeError(
                "ScoutNav cannot import scout_workspace_common. Source the ROS 2 "
                "workspace or set SCOUT_WS_ROOT to the Scout workspace root."
            ) from initial_error
        source_root_text = str(source_root)
        if source_root_text not in sys.path:
            sys.path.insert(0, source_root_text)
        from scout_workspace_common import (
            WorkspaceRootError,
            debug_log_path,
            resolve_workspace_root,
            workspace_path,
        )

        return WorkspaceRootError, debug_log_path, resolve_workspace_root, workspace_path


WorkspaceRootError, _debug_log_path, _resolve_workspace_root, _workspace_path = (
    _import_shared_paths()
)


def workspace_root() -> Path:
    """Return this Scout checkout's root via ``scout_workspace_common``."""
    return _resolve_workspace_root(start_path=Path(__file__))


def workspace_file(*parts: str) -> Path:
    """Return a path beneath the active Scout workspace."""
    return _workspace_path(*parts, start_path=Path(__file__))


def runtime_debug_log_path(filename: str = "scout_nav_app.jsonl") -> Path:
    """Return a ScoutNav debug log under ``3D/debug_logs``."""
    return _debug_log_path(filename, start_path=Path(__file__))


def launch_log_paths(*, include_legacy: bool = True) -> list[Path]:
    """Return current launch logs, plus old app-local logs for read-only migration."""
    current = [
        workspace_file("3D", "debug_logs", "scout_nav_app_map_launch.log"),
        workspace_file("3D", "debug_logs", "scout_nav_app_nav_launch.log"),
    ]
    if not include_legacy:
        return current
    legacy = [
        workspace_file("3D", "scout_nav_app", ".last_map_launch.log"),
        workspace_file("3D", "scout_nav_app", ".last_nav_launch.log"),
    ]
    return current + legacy


def append_debug_event(
    location: str,
    message: str,
    data: Optional[dict[str, Any]] = None,
    *,
    filename: str = "scout_nav_app.jsonl",
) -> None:
    """Append a best-effort structured event without interrupting the GUI."""
    record = {
        "component": "scout_nav_app",
        "location": location,
        "message": message,
        "data": data or {},
        "timestamp": int(time.time() * 1000),
    }
    try:
        with runtime_debug_log_path(filename).open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
    except (OSError, RuntimeError):
        # Diagnostics must never stop the control UI.
        pass
