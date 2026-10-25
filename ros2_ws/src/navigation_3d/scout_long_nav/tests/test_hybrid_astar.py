# -*- coding: utf-8 -*-
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scout_long_nav.hybrid_astar import HybridAstarParams, plan_hybrid_astar
from scout_workspace_common import workspace_path


def _empty_grid(size_m=20.0, res=0.5):
    w = h = int(size_m / res)
    data = np.zeros(w * h, dtype=np.int8)
    origin = -0.5 * size_m
    return origin, origin, res, w, h, data


def test_hybrid_astar_finds_path_open_space():
    ox, oy, res, w, h, data = _empty_grid()
    start = (-5.0, -5.0, 0.0)
    goal = (5.0, 5.0, math.radians(45.0))
    path = plan_hybrid_astar(
        start,
        goal,
        ox,
        oy,
        res,
        w,
        h,
        data,
        HybridAstarParams(xy_resolution=0.5, max_iterations=8000, goal_xy_tol=1.0),
    )
    assert path is not None
    assert len(path.xs) >= 2
    assert math.hypot(path.xs[-1] - goal[0], path.ys[-1] - goal[1]) < 1.5


def test_hybrid_astar_blocked_wall():
    """Security wall blocks; terrain-100 strip alone must still allow a path."""
    from scout_long_nav.cost_codes import OCC_SECURITY

    ox, oy, res, w, h, data = _empty_grid(size_m=20.0, res=0.5)
    for iy in range(h):
        for ix in range(w):
            x = ox + (ix + 0.5) * res
            if abs(x) < 0.6 and abs(oy + (iy + 0.5) * res) < 8.0:
                data[iy * w + ix] = 100
    path_soft = plan_hybrid_astar(
        (-5.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        ox,
        oy,
        res,
        w,
        h,
        data,
        HybridAstarParams(xy_resolution=0.5, max_iterations=20000, goal_xy_tol=1.2),
    )
    assert path_soft is not None and len(path_soft.xs) >= 2

    data2 = data.copy()
    for iy in range(h):
        for ix in range(w):
            x = ox + (ix + 0.5) * res
            if abs(x) < 0.6:
                data2[iy * w + ix] = OCC_SECURITY
    path_hard = plan_hybrid_astar(
        (-5.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        ox,
        oy,
        res,
        w,
        h,
        data2,
        HybridAstarParams(xy_resolution=0.5, max_iterations=12000, goal_xy_tol=1.2),
    )
    assert path_hard is None


def test_security_fence_infinite_not_crossed():
    """Thin security wall must not produce a path that snaps through the fence."""
    from scout_long_nav.cost_codes import OCC_SECURITY
    from scout_long_nav.hybrid_astar import _segment_traversal_cost, security_blocks_connection

    ox, oy, res, w, h, data = _empty_grid(size_m=20.0, res=0.2)
    for iy in range(h):
        for ix in range(w):
            x = ox + (ix + 0.5) * res
            if abs(x) < res * 0.6:
                data[iy * w + ix] = OCC_SECURITY
    start = (-4.0, 0.0, 0.0)
    goal = (4.0, 0.0, 0.0)
    path = plan_hybrid_astar(
        start,
        goal,
        ox,
        oy,
        res,
        w,
        h,
        data,
        HybridAstarParams(
            xy_resolution=0.4,
            step_length=0.5,
            max_iterations=25000,
            goal_xy_tol=1.0,
            footprint_radius_m=0.25,
        ),
    )
    assert path is None, "must not return path that jumps across security fence"
    assert (
        _segment_traversal_cost(-1, 0, 1, 0, ox, oy, res, w, h, data, 90, 0.2) is None
    )
    assert security_blocks_connection(ox, oy, res, w, h, data, (-4.0, 0.0), (4.0, 0.0))


def test_terrain_100_is_not_hard_block():
    """Orange terrain cost=100 strip is expensive but passable; only security is infinite."""
    ox, oy, res, w, h, data = _empty_grid(size_m=20.0, res=0.5)
    for iy in range(h):
        for ix in range(w):
            x = ox + (ix + 0.5) * res
            if abs(x) < 1.0:
                data[iy * w + ix] = 100
    path = plan_hybrid_astar(
        (-5.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        ox,
        oy,
        res,
        w,
        h,
        data,
        HybridAstarParams(max_iterations=20000, goal_xy_tol=1.2, footprint_radius_m=0.2),
    )
    assert path is not None and len(path.xs) >= 2


def test_plan_global_path_long_goal_on_maps_long():
    """Regression: ~35m goals on arena_trav must yield a path (grid fallback)."""
    from scout_long_nav.cost_codes import grid_to_occ_int8
    from scout_long_nav.hybrid_astar import plan_global_path
    from scout_long_nav.map_io import load_trav_map

    map_yaml = workspace_path(
        "3D",
        "maps_long",
        "arena_trav.yaml",
        package_name="scout_long_nav",
        start_path=__file__,
    )
    m = load_trav_map(str(map_yaml))
    data = grid_to_occ_int8(m["grid"])
    path, planner = plan_global_path(
        (-27.83, 22.28, 2.72),
        (6.21, 30.98, 1.37),
        m["origin_x"],
        m["origin_y"],
        m["resolution"],
        m["grid"].shape[1],
        m["grid"].shape[0],
        data,
        HybridAstarParams(),
    )
    assert path is not None and len(path.xs) >= 2
    assert planner in ("grid", "hybrid")
