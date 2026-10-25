"""Shared utilities used by the Scout ROS 2 workspace."""

from .workspace_paths import (
    WorkspaceRootError,
    debug_log_path,
    resolve_workspace_root,
    workspace_path,
)

__all__ = [
    "WorkspaceRootError",
    "debug_log_path",
    "resolve_workspace_root",
    "workspace_path",
]
