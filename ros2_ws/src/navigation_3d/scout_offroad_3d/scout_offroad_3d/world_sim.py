# -*- coding: utf-8 -*-
"""Launch-time .world patcher: physics + collision-grid from yaml.

Gazebo Classic reads physics/solver/collision from the static .world SDF at
gzserver start. To control them from one place (td3_train.yaml), we patch a
cached copy of the world right before gzserver loads it — source worlds under
ros2_ws/src are NEVER modified by this.

Does NOT patch heightmap friction (Record 8: bare <ode/>). Wheel μ lives on
scout_mini.gazebo (factory 1.0/0.9).

Params (yaml ros__parameters):
  sim_max_step_size          physics step (s)        default 0.01
  sim_real_time_update_rate  physics rate cap (Hz)   default 100
  sim_solver_iters           ODE quickstep iters     default 20
  sim_collision_heightmap_res  65 = fast (train) / 129 = full fidelity (eval)
"""

from __future__ import annotations

import hashlib
import os
import re

from scout_workspace_common import workspace_path

DEFAULTS = {
    "sim_max_step_size": 0.01,
    "sim_real_time_update_rate": 100,
    "sim_solver_iters": 20,
    "sim_collision_heightmap_res": 65,
}


def _fmt(x) -> str:
    f = float(x)
    return ("%g" % f)


def patch_world_text(text: str, cfg: dict) -> str:
    step = _fmt(cfg.get("sim_max_step_size", DEFAULTS["sim_max_step_size"]))
    rate = _fmt(cfg.get("sim_real_time_update_rate", DEFAULTS["sim_real_time_update_rate"]))
    iters = str(int(cfg.get("sim_solver_iters", DEFAULTS["sim_solver_iters"])))
    res = int(cfg.get("sim_collision_heightmap_res", DEFAULTS["sim_collision_heightmap_res"]))

    text = re.sub(r"<max_step_size>[^<]+</max_step_size>",
                  f"<max_step_size>{step}</max_step_size>", text, count=1)
    text = re.sub(r"<real_time_update_rate>[^<]+</real_time_update_rate>",
                  f"<real_time_update_rate>{rate}</real_time_update_rate>", text, count=1)
    text = re.sub(r"(<solver>[\s\S]*?<iters>)[^<]+(</iters>)",
                  r"\g<1>" + iters + r"\g<2>", text, count=1)

    # Collision heightmap resolution: only inside <collision> blocks.
    def _col(m):
        block = m.group(0)
        def _uri(mm):
            uri = mm.group(1)
            stem = re.sub(r"(_65)?\.png$", "", uri)
            return f"<uri>{stem}_65.png</uri>" if res == 65 else f"<uri>{stem}.png</uri>"
        return re.sub(r"<uri>(model://[^<]*terrain_td3_\d+(?:_65)?\.png)</uri>", _uri, block)

    text = re.sub(r"<collision name='collision'>[\s\S]*?</collision>", _col, text)
    return text


def prepare_world(world_path: str, cfg: dict, cache_dir: str | None = None) -> str:
    """Return a world path with sim params applied (cached). Source untouched."""
    try:
        with open(world_path, "r") as f:
            text = f.read()
    except Exception:
        return world_path
    patched = patch_world_text(text, cfg)
    if patched == text:
        return world_path

    if cache_dir is None:
        cache_dir = str(
            workspace_path(
                "3D",
                "debug_logs",
                "world_cache",
                package_name="scout_offroad_3d",
                start_path=__file__,
            )
        )
    os.makedirs(cache_dir, exist_ok=True)
    key = hashlib.md5(
        (os.path.abspath(world_path) + "|" + patched[:4096]).encode()
    ).hexdigest()[:10]
    stem = os.path.basename(world_path).replace(".world", "")
    out = os.path.join(cache_dir, f"{stem}__sim{key}.world")
    if not os.path.isfile(out):
        with open(out, "w") as f:
            f.write(patched)
    return out
