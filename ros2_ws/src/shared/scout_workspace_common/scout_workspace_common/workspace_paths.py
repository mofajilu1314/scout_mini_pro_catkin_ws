"""Portable paths for a checked-out Scout workspace.

``SCOUT_WS_ROOT`` is the canonical override.  The two legacy variable names
remain supported so existing shells keep working while scripts migrate.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, Optional, Union


PathLike = Union[str, os.PathLike[str]]
_ROOT_ENV_NAMES = ("SCOUT_WS_ROOT", "CATKIN_WS", "WS_ROOT")


class WorkspaceRootError(RuntimeError):
    """Raised when the current process cannot identify a Scout workspace."""


def _normalise(path: PathLike) -> Path:
    return Path(path).expanduser().resolve()


def _looks_like_workspace(path: Path) -> bool:
    """Return whether *path* has the stable top-level workspace markers."""
    return (path / "ros2_ws" / "src").is_dir() and (path / "3D").is_dir()


def _ancestor_workspace(path: PathLike) -> Optional[Path]:
    candidate = _normalise(path)
    if candidate.is_file():
        candidate = candidate.parent
    for parent in (candidate, *candidate.parents):
        if _looks_like_workspace(parent):
            return parent
    return None


def _package_share_workspace(package_name: str) -> Optional[Path]:
    """Find a workspace by walking upward from an installed package share."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return _ancestor_workspace(get_package_share_directory(package_name))
    except (ImportError, LookupError, OSError, RuntimeError):
        return None


def resolve_workspace_root(
    *, package_name: Optional[str] = None, start_path: Optional[PathLike] = None
) -> Path:
    """Return the workspace root without relying on a user-specific path.

    Resolution order is deliberately deterministic: ``SCOUT_WS_ROOT``, then
    legacy ``CATKIN_WS``/``WS_ROOT``, then package/source/current-directory
    discovery.  An explicitly supplied environment path must exist; silently
    falling through to a different checkout would make logs and artifacts land
    in the wrong place.
    """
    for name in _ROOT_ENV_NAMES:
        value = os.environ.get(name, "").strip()
        if not value:
            continue
        root = _normalise(value)
        if not root.is_dir():
            raise WorkspaceRootError(
                "%s points to a directory that does not exist: %s" % (name, root)
            )
        return root

    if package_name:
        root = _package_share_workspace(package_name)
        if root:
            return root

    starts: Iterable[PathLike] = tuple(
        value
        for value in (start_path, Path(__file__), Path.cwd())
        if value is not None
    )
    for start in starts:
        root = _ancestor_workspace(start)
        if root:
            return root

    raise WorkspaceRootError(
        "Cannot locate the Scout workspace. Set SCOUT_WS_ROOT to its root "
        "(CATKIN_WS and WS_ROOT are accepted for compatibility)."
    )


def workspace_path(
    *parts: PathLike, package_name: Optional[str] = None, start_path: Optional[PathLike] = None
) -> Path:
    """Return a path underneath the resolved workspace root."""
    return resolve_workspace_root(package_name=package_name, start_path=start_path).joinpath(
        *map(str, parts)
    )


def debug_log_path(
    filename: str = "runtime.jsonl", *, package_name: Optional[str] = None, start_path: Optional[PathLike] = None
) -> Path:
    """Return a writable workspace-owned debug log location.

    Debug output belongs in ``3D/debug_logs`` rather than editor metadata or a
    training-result directory.  Callers may safely open the returned path.
    """
    directory = workspace_path(
        "3D", "debug_logs", package_name=package_name, start_path=start_path
    )
    directory.mkdir(parents=True, exist_ok=True)
    return directory / Path(filename).name
