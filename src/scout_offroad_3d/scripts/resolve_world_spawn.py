#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Resolve per-world spawn_z from td3_train.yaml (for infer / launch).

Usage:
  rosrun scout_offroad_3d resolve_world_spawn.py /path/to/bumpy_offroad.world
  rosrun scout_offroad_3d resolve_world_spawn.py bumpy_offroad

Prints one float to stdout.
"""

from __future__ import print_function

import os
import sys

try:
    import yaml
except ImportError:
    yaml = None


def _config_path():
    try:
        import rospkg

        return os.path.join(
            rospkg.RosPack().get_path("scout_offroad_3d"), "config", "td3_train.yaml"
        )
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, "..", "config", "td3_train.yaml"))


def resolve_spawn_z(world_path, config_path=None):
    config_path = config_path or _config_path()
    base = os.path.basename(os.path.abspath(world_path))
    default_z = 0.15
    overrides = {}
    if yaml is not None and os.path.isfile(config_path):
        with open(config_path) as fh:
            cfg = yaml.safe_load(fh) or {}
        default_z = float(cfg.get("spawn_z", default_z))
        overrides = dict(cfg.get("world_spawn_z") or {})
    for key, val in overrides.items():
        if key == base or key in base or base in key:
            return float(val)
    return default_z


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: resolve_world_spawn.py <world.path|basename>")
    print("%.3f" % resolve_spawn_z(sys.argv[1]))


if __name__ == "__main__":
    main()
