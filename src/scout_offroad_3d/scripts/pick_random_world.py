#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Resolve / randomly pick a .world under scout_offroad_3d/worlds/.

Usage:
  rosrun scout_offroad_3d pick_random_world.py           # random
  rosrun scout_offroad_3d pick_random_world.py random
  rosrun scout_offroad_3d pick_random_world.py rough_obstacles
  rosrun scout_offroad_3d pick_random_world.py /abs/path/x.world
  rosrun scout_offroad_3d pick_random_world.py --print-launch-eval

Prints one absolute path to stdout (for shell substitution into launch).

train_td3.launch default world uses roslaunch $(eval)+anon (Noetic forbids
__import__/subprocess in eval). Keep that basename list in sync with
worlds/*.world; --print-launch-eval prints the expression to paste.
"""

from __future__ import print_function

import glob
import os
import random
import sys


def _worlds_dir():
    try:
        import rospkg

        return os.path.join(rospkg.RosPack().get_path("scout_offroad_3d"), "worlds")
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, "..", "worlds"))


def list_worlds():
    cands = sorted(glob.glob(os.path.join(_worlds_dir(), "*.world")))
    if not cands:
        raise SystemExit("no *.world under %s" % _worlds_dir())
    return cands


def launch_eval_expr():
    """Python expression for roslaunch $(eval ...) default (Noetic-safe)."""
    names = [os.path.basename(p) for p in list_worlds()]
    n = len(names)
    quoted = ",".join("'%s'" % name for name in names)
    return (
        "find('scout_offroad_3d')+'/worlds/'+[%s][int(anon('rw').split('_')[-1])%%%d]"
        % (quoted, n)
    )


def resolve(arg):
    wd = _worlds_dir()
    if not arg or arg == "random":
        return random.choice(list_worlds())
    if os.path.isabs(arg) and os.path.isfile(arg):
        return arg
    name = arg if arg.endswith(".world") else arg + ".world"
    path = os.path.join(wd, name)
    if os.path.isfile(path):
        return path
    if os.path.isfile(arg):
        return os.path.abspath(arg)
    raise SystemExit("world not found: %s (looked in %s)" % (arg, wd))


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "random"
    if arg in ("--print-launch-eval", "-e"):
        print(launch_eval_expr())
        return
    print(resolve(arg))


if __name__ == "__main__":
    main()
