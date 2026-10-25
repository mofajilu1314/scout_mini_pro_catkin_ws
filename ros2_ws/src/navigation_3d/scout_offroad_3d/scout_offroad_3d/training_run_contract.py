"""Filesystem-only guardrails for TD3 run creation and continuation.

This module deliberately has no ROS, Gazebo, NumPy, or PyTorch imports so the
policy around run directories can be tested before a simulator is started.
"""

from __future__ import print_function

import os
import time


# TD3.load() restores all four files.  Treat an incomplete checkpoint as an
# invalid continuation rather than discovering the problem after writing to it.
REQUIRED_RESUME_ARTIFACTS = (
    "td3_actor",
    "td3_critic",
    "td3_actor_optimizer",
    "td3_critic_optimizer",
    "metrics.csv",
    "params_backup.yaml",
)


def _expanded_absolute_path(value):
    """Expand a user/environment supplied path into an absolute path."""
    return os.path.abspath(os.path.expanduser(os.path.expandvars(str(value))))


def workspace_root_from_package_share(package_share_dir):
    """Derive the project root from an installed or source package directory.

    A source package normally lives below ``<root>/ros2_ws/src/...`` and an
    installed package below ``<root>/ros2_ws/install/...``.  During source and
    symlink installs the exact number of intermediate directories varies, so
    walk upward instead of relying on a fixed offset.  ``3D/`` is the most
    specific project-root marker; ``ros2_ws`` is the portable fallback marker.

    Return ``None`` rather than inventing a machine-specific path when the
    package location is not part of a recognizable Scout workspace.
    """
    if not package_share_dir:
        return None
    current = _expanded_absolute_path(package_share_dir)
    for _ in range(16):
        if os.path.isdir(os.path.join(current, "3D")) and os.path.isdir(
            os.path.join(current, "ros2_ws")
        ):
            return current
        if os.path.basename(current) == "ros2_ws":
            return os.path.dirname(current)
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return None


def resolve_workspace_root(environ=None, package_share_dir=None):
    """Resolve the Scout workspace without a user-specific fallback.

    The order is deliberately fixed for every caller:
    ``SCOUT_WS_ROOT``, ``CATKIN_WS``, ``WS_ROOT``, then a location derived from
    the ROS package share directory.  A missing value is represented by
    ``None`` so the caller can fail before creating output in an unknown place.
    """
    environ = os.environ if environ is None else environ
    for key in ("SCOUT_WS_ROOT", "CATKIN_WS", "WS_ROOT"):
        value = str(environ.get(key, "")).strip()
        if value:
            return _expanded_absolute_path(value)
    return workspace_root_from_package_share(package_share_dir)


def _nonempty_regular_file(path):
    """Return true only for a regular, non-empty file."""
    return os.path.isfile(path) and os.path.getsize(path) > 0


def fresh_output_error(output_dir):
    """Describe why ``output_dir`` cannot be used for a fresh run, or None.

    An absent directory and an empty directory are both safe.  A non-directory
    target or any existing child entry is not: a new run must never append to,
    overwrite, or masquerade as a previous run.
    """
    if not os.path.exists(output_dir):
        return None
    if not os.path.isdir(output_dir):
        return "output path exists but is not a directory"
    try:
        with os.scandir(output_dir) as entries:
            if next(entries, None) is not None:
                return "output directory is not empty"
    except OSError as exc:
        return "cannot inspect output directory: %s" % exc
    return None


def create_unique_run_directory(runs_dir, timestamp=None):
    """Atomically claim a new ``run_<timestamp>`` directory.

    The timestamp is normally generated here.  A numeric suffix avoids a
    collision if two training launches begin in the same second.
    """
    os.makedirs(runs_dir, exist_ok=True)
    stamp = timestamp or time.strftime("%Y%m%d_%H%M%S")
    for index in range(10000):
        suffix = "" if index == 0 else "_%02d" % index
        candidate = os.path.join(runs_dir, "run_%s%s" % (stamp, suffix))
        try:
            os.mkdir(candidate)
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("could not allocate a unique run directory under %s" % runs_dir)


def missing_resume_artifacts(output_dir):
    """Return the required continuation artifacts that are absent or empty."""
    return tuple(
        name
        for name in REQUIRED_RESUME_ARTIFACTS
        if not _nonempty_regular_file(os.path.join(output_dir, name))
    )


def resume_horizon_error(horizon_was_explicit, max_timesteps, completed_timesteps):
    """Return a continuation-horizon error, or None.

    ``horizon_was_explicit`` is supplied by the launch file.  It prevents a
    yaml default from silently extending a run that a user merely asked to
    resume.  A continuation target must also be strictly beyond the last
    timestep recorded in metrics.csv.
    """
    if not horizon_was_explicit:
        return "resume requires an explicit max_timesteps:=N launch argument"
    try:
        horizon = int(max_timesteps)
        completed = int(completed_timesteps)
    except (TypeError, ValueError):
        return "max_timesteps and completed timesteps must be integers"
    if horizon <= completed:
        return "max_timesteps=%d must be greater than completed t=%d" % (
            horizon,
            completed,
        )
    return None


def snapshot_dimension_errors(snapshot):
    """Validate the shape metadata required to reconstruct a TD3 checkpoint.

    The saved metadata, train kwargs, and network width must agree before
    PyTorch is asked to load any weight tensors.  This intentionally accepts
    future valid state dimensions instead of hard-coding the current 150-D
    representation.
    """
    if not isinstance(snapshot, dict):
        return ("snapshot is not a mapping",)
    meta = snapshot.get("meta")
    train = snapshot.get("train")
    if not isinstance(meta, dict):
        return ("snapshot meta section is missing",)
    if not isinstance(train, dict):
        return ("snapshot train section is missing",)

    errors = []

    def positive_int(section, key):
        try:
            value = int(section.get(key))
        except (TypeError, ValueError):
            errors.append("%s is missing or invalid" % key)
            return None
        if value <= 0:
            errors.append("%s must be positive" % key)
            return None
        return value

    state_dim = positive_int(meta, "state_dim")
    action_dim = positive_int(meta, "action_dim")
    hidden_dim = positive_int(train, "hidden_dim")
    kwargs = train.get("kwargs")
    if not isinstance(kwargs, dict):
        errors.append("train.kwargs is missing")
        return tuple(errors)

    for key, expected in (
        ("state_dim", state_dim),
        ("action_dim", action_dim),
        ("hidden_dim", hidden_dim),
    ):
        try:
            actual = int(kwargs.get(key))
        except (TypeError, ValueError):
            errors.append("train.kwargs.%s is missing or invalid" % key)
            continue
        if expected is not None and actual != expected:
            errors.append(
                "train.kwargs.%s=%d does not match snapshot %d"
                % (key, actual, expected)
            )
    return tuple(errors)


def snapshot_dimensions(snapshot):
    """Return validated checkpoint dimensions or raise ``ValueError``."""
    errors = snapshot_dimension_errors(snapshot)
    if errors:
        raise ValueError("; ".join(errors))
    return {
        "state_dim": int(snapshot["meta"]["state_dim"]),
        "action_dim": int(snapshot["meta"]["action_dim"]),
        "hidden_dim": int(snapshot["train"]["hidden_dim"]),
    }
