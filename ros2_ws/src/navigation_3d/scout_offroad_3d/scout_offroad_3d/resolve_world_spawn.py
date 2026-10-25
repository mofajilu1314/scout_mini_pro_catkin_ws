#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Resolve per-world spawn_z from td3_train.yaml (for infer / launch).

Usage:
  ros2 run scout_offroad_3d resolve_world_spawn /path/to/bumpy_offroad.world
  ros2 run scout_offroad_3d resolve_world_spawn bumpy_offroad

Prints one float to stdout.
"""

from __future__ import annotations

import os
import sys

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None  # type: ignore


def _config_path() -> str:
    if get_package_share_directory is not None:
        try:
            return os.path.join(
                get_package_share_directory("scout_offroad_3d"),
                "config",
                "td3_train.yaml",
            )
        except Exception:
            pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(
        os.path.join(here, "..", "..", "config", "td3_train.yaml")
    )


def _unwrap_ros2_params(cfg: dict) -> dict:
    """Extract ros__parameters from ROS2 param file if present."""
    if not isinstance(cfg, dict):
        return {}
    if "/**" in cfg and isinstance(cfg["/**"], dict):
        inner = cfg["/**"].get("ros__parameters", {})
        if isinstance(inner, dict):
            return inner
    return cfg


def resolve_spawn_z(world_path: str, config_path: str | None = None) -> float:
    config_path = config_path or _config_path()
    base = os.path.basename(os.path.abspath(world_path))
    default_z = 0.15
    overrides: dict = {}
    if yaml is not None and os.path.isfile(config_path):
        with open(config_path) as fh:
            raw = yaml.safe_load(fh) or {}
        cfg = _unwrap_ros2_params(raw)
        default_z = float(cfg.get("spawn_z", default_z))
        # 记录56：单一来源 world_spawn_z_entries（"stem:z" 列表）
        for item in cfg.get("world_spawn_z_entries") or []:
            s = str(item).strip()
            if not s or ":" not in s:
                continue
            k, v = s.rsplit(":", 1)
            try:
                overrides[k.strip()] = float(v)
            except (TypeError, ValueError):
                pass
    stem = base[:-6] if base.lower().endswith(".world") else base
    # 先精确匹配 stem/base，再子串兜底——否则 bumpy_offroad 抢先匹配 _2…_4
    for key, val in overrides.items():
        if key == base or key == stem:
            return float(val)
    for key, val in overrides.items():
        if key in base or base in key:
            return float(val)
    return default_z


def main(argv=None) -> None:
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        raise SystemExit("usage: resolve_world_spawn <world.path|basename>")
    print("%.3f" % resolve_spawn_z(argv[0]))


if __name__ == "__main__":
    main()
