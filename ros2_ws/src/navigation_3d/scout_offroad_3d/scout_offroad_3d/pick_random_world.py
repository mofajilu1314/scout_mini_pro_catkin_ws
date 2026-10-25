#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Resolve / randomly pick a .world from the TD3 train pool (exactly 4 maps).

Usage:
  ros2 run scout_offroad_3d pick_random_world           # random
  ros2 run scout_offroad_3d pick_random_world random
  ros2 run scout_offroad_3d pick_random_world bumpy_offroad_3
  ros2 run scout_offroad_3d pick_random_world /abs/path/x.world

Prints one absolute path to stdout (for shell substitution into launch).
"""

from __future__ import annotations

import os
import random
import sys

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None  # type: ignore

from scout_offroad_3d.world_pool import list_train_world_paths, resolve_world_path


def _worlds_dir() -> str:
    if get_package_share_directory is not None:
        try:
            return os.path.join(
                get_package_share_directory("scout_offroad_3d"), "worlds"
            )
        except Exception:
            pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "worlds"))


def list_worlds() -> list[str]:
    cands = list_train_world_paths(_worlds_dir())
    if not cands:
        raise SystemExit("no train *.world under %s" % _worlds_dir())
    return cands


def resolve(arg: str) -> str:
    wd = _worlds_dir()
    if not arg or arg == "random":
        return random.choice(list_worlds())
    if os.path.isabs(arg) and os.path.isfile(arg):
        return arg
    path = resolve_world_path(wd, arg)
    if path:
        return path
    if os.path.isfile(arg):
        return os.path.abspath(arg)
    raise SystemExit("world not found: %s (looked in %s)" % (arg, wd))


def main(argv=None) -> None:
    argv = argv if argv is not None else sys.argv[1:]
    arg = argv[0] if argv else "random"
    print(resolve(arg))


if __name__ == "__main__":
    main()
