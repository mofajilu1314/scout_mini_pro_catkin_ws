# -*- coding: utf-8 -*-
"""
Simplified Hybrid A* for OccupancyGrid costmaps.

Security fence cells (OCC_SECURITY=127 / internal 1000) are **infinite cost**:
any motion that touches them is rejected. Terrain 0..100 is always passable
(finite cost). Steps are collision-checked along the whole segment so a thin
fence cannot be skipped by step_length > cell size.
"""


from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from scout_long_nav.cost_codes import OCC_SECURITY, SECURITY_COST


@dataclass
class HybridAstarParams:
    # Coarser defaults: fine 0.5m/15k iters fails on ~35m arena goals
    xy_resolution: float = 1.0
    yaw_resolution_deg: float = 45.0
    wheelbase: float = 0.45
    step_length: float = 1.0
    max_steer_rad: float = 0.65
    n_steer: int = 3
    lethal_threshold: int = 90
    max_iterations: int = 35000
    goal_xy_tol: float = 2.0
    goal_yaw_tol_deg: float = 45.0
    reverse_cost: float = 2.0
    steer_cost: float = 0.1
    # Robot body radius for security inflation along the path (meters)
    footprint_radius_m: float = 0.25


@dataclass
class HybridPath:
    xs: List[float]
    ys: List[float]
    yaws: List[float]
    cost: float


def _yaw_res(p: HybridAstarParams) -> float:
    return math.radians(p.yaw_resolution_deg)


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class _Node:
    __slots__ = ("ix", "iy", "iyaw", "x", "y", "yaw", "g", "parent", "dir_fwd")

    def __init__(self, ix, iy, iyaw, x, y, yaw, g, parent, dir_fwd):
        self.ix = ix
        self.iy = iy
        self.iyaw = iyaw
        self.x = x
        self.y = y
        self.yaw = yaw
        self.g = g
        self.parent = parent
        self.dir_fwd = dir_fwd


def _index(x: float, y: float, yaw: float, origin_x: float, origin_y: float, p: HybridAstarParams):
    ix = int(round((x - origin_x) / p.xy_resolution))
    iy = int(round((y - origin_y) / p.xy_resolution))
    iyaw = int(round(_wrap(yaw) / _yaw_res(p)))
    return ix, iy, iyaw


def _is_security_cell(v: int) -> bool:
    """Security fence = infinite cost (wire 127 or internal 1000)."""
    return int(v) >= OCC_SECURITY or int(v) >= SECURITY_COST or int(v) == OCC_SECURITY


def _cell_at(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    res: float,
    width: int,
    height: int,
    data: np.ndarray,
) -> Optional[int]:
    ix = int(math.floor((x - origin_x) / res))
    iy = int(math.floor((y - origin_y) / res))
    if ix < 0 or iy < 0 or ix >= width or iy >= height:
        return None
    return int(data[iy * width + ix])


def _point_blocked(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    res: float,
    width: int,
    height: int,
    data: np.ndarray,
    lethal: int,
    footprint_r: float,
) -> bool:
    """True only for out-of-map or **security** (infinite). Terrain 0..100 is passable."""
    del lethal  # terrain max 100 is high finite cost, not a hard block
    samples = [(x, y)]
    if footprint_r > 1e-6:
        for ang_deg in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
            a = math.radians(ang_deg)
            for ring in (footprint_r * 0.5, footprint_r):
                samples.append((x + ring * math.cos(a), y + ring * math.sin(a)))
    for px, py in samples:
        v = _cell_at(px, py, origin_x, origin_y, res, width, height, data)
        if v is None:
            return True
        if _is_security_cell(v):
            return True
    return False


def _segment_traversal_cost(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    origin_x: float,
    origin_y: float,
    res: float,
    width: int,
    height: int,
    data: np.ndarray,
    lethal: int,
    footprint_r: float,
) -> Optional[float]:
    """
    Cost along segment. Security → infinite (None).
    Samples densely so thin fences cannot be skipped by long steps.
    """
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    # sample every ~half cell
    step = max(res * 0.4, 0.02)
    n = max(2, int(math.ceil(length / step)) + 1)
    acc = 0.0
    for i in range(n):
        t = i / (n - 1)
        x = x0 + t * dx
        y = y0 + t * dy
        if _point_blocked(
            x, y, origin_x, origin_y, res, width, height, data, lethal, footprint_r
        ):
            return None
        v = _cell_at(x, y, origin_x, origin_y, res, width, height, data)
        if v is None:
            return None
        if v < 0:
            acc += 1.0
        else:
            acc += 1.0 + 0.05 * float(min(v, 100))
    return acc / float(n)


def security_blocks_connection(
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    data: Sequence[int],
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
) -> bool:
    """
    True if start cannot reach goal when **only** security cells are impassable
    (terrain 0..100 and unknown are walkable). Used to decide red-line abort.
    """
    from collections import deque

    arr = np.asarray(data, dtype=np.int16).reshape(-1)
    res = float(resolution)
    if res <= 1e-9:
        return False

    def idx(x: float, y: float) -> Optional[Tuple[int, int]]:
        ix = int(math.floor((x - origin_x) / res))
        iy = int(math.floor((y - origin_y) / res))
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            return None
        return ix, iy

    s = idx(start_xy[0], start_xy[1])
    g = idx(goal_xy[0], goal_xy[1])
    if s is None or g is None:
        return True
    sx, sy = s
    gx, gy = g
    if _is_security_cell(int(arr[sy * width + sx])) or _is_security_cell(
        int(arr[gy * width + gx])
    ):
        return True

    q = deque([(sx, sy)])
    seen = {(sx, sy)}
    # subsample stride for speed on 1000x1000
    while q:
        x, y = q.popleft()
        if abs(x - gx) + abs(y - gy) <= 2:
            return False
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if (nx, ny) in seen:
                continue
            v = int(arr[ny * width + nx])
            if _is_security_cell(v):
                continue
            seen.add((nx, ny))
            q.append((nx, ny))
    return True


def plan_hybrid_astar(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    data: Sequence[int],
    params: Optional[HybridAstarParams] = None,
) -> Optional[HybridPath]:
    """
    start/goal: (x, y, yaw_rad) in the same frame as the OccupancyGrid.
    data: row-major occupancy (-1 unknown, 0..100 terrain, 127=security infinite).
    """
    p = params or HybridAstarParams()
    p = HybridAstarParams(
        xy_resolution=max(p.xy_resolution, resolution),
        yaw_resolution_deg=p.yaw_resolution_deg,
        wheelbase=p.wheelbase,
        step_length=p.step_length,
        max_steer_rad=p.max_steer_rad,
        n_steer=p.n_steer,
        lethal_threshold=p.lethal_threshold,
        max_iterations=p.max_iterations,
        goal_xy_tol=p.goal_xy_tol,
        goal_yaw_tol_deg=p.goal_yaw_tol_deg,
        reverse_cost=p.reverse_cost,
        steer_cost=p.steer_cost,
        footprint_radius_m=p.footprint_radius_m,
    )
    arr = np.asarray(data, dtype=np.int16).reshape(-1)
    if arr.size != width * height:
        raise ValueError("data size mismatch")

    steers = np.linspace(-p.max_steer_rad, p.max_steer_rad, p.n_steer)
    yaw_tol = math.radians(p.goal_yaw_tol_deg)

    sx, sy, syaw = start
    gx, gy, gyaw = goal
    six, siy, siyaw = _index(sx, sy, syaw, origin_x, origin_y, p)
    start_node = _Node(six, siy, siyaw, sx, sy, syaw, 0.0, None, True)

    open_heap: List[Tuple[float, int, _Node]] = []
    counter = 0
    heapq.heappush(open_heap, (0.0, counter, start_node))
    closed = set()
    best: Optional[_Node] = None
    best_h = float("inf")

    def heuristic(x, y, yaw):
        return math.hypot(gx - x, gy - y) + 0.1 * abs(_wrap(gyaw - yaw))

    iters = 0
    while open_heap and iters < p.max_iterations:
        iters += 1
        _, _, cur = heapq.heappop(open_heap)
        key = (cur.ix, cur.iy, cur.iyaw)
        if key in closed:
            continue
        closed.add(key)

        h = heuristic(cur.x, cur.y, cur.yaw)
        if h < best_h:
            best_h = h
            best = cur

        if math.hypot(gx - cur.x, gy - cur.y) <= p.goal_xy_tol:
            best = cur
            break

        for steer in steers:
            for direction in (1.0, -1.0):
                yaw = cur.yaw
                x, y = cur.x, cur.y
                ds = p.step_length * direction
                yaw_new = _wrap(yaw + ds / p.wheelbase * math.tan(steer))
                x_new = x + ds * math.cos(yaw)
                y_new = y + ds * math.sin(yaw)
                gc = _segment_traversal_cost(
                    x,
                    y,
                    x_new,
                    y_new,
                    origin_x,
                    origin_y,
                    resolution,
                    width,
                    height,
                    arr,
                    p.lethal_threshold,
                    p.footprint_radius_m,
                )
                if gc is None:
                    continue
                step_cost = abs(ds) * gc
                if direction < 0:
                    step_cost *= p.reverse_cost
                step_cost += p.steer_cost * abs(steer)
                g = cur.g + step_cost
                ix, iy, iyaw = _index(x_new, y_new, yaw_new, origin_x, origin_y, p)
                nkey = (ix, iy, iyaw)
                if nkey in closed:
                    continue
                node = _Node(ix, iy, iyaw, x_new, y_new, yaw_new, g, cur, direction > 0)
                f = g + heuristic(x_new, y_new, yaw_new)
                counter += 1
                heapq.heappush(open_heap, (f, counter, node))

    if best is None:
        return None

    # Accept path if XY reached; yaw is soft (RViz goals often have awkward yaw)
    reached = math.hypot(gx - best.x, gy - best.y) <= p.goal_xy_tol
    if not reached:
        return None

    xs, ys, yaws = [], [], []
    n = best
    while n is not None:
        xs.append(n.x)
        ys.append(n.y)
        yaws.append(n.yaw)
        n = n.parent
    xs.reverse()
    ys.reverse()
    yaws.reverse()

    # Optional short snap to exact goal — only if the segment is free of security/lethal
    if math.hypot(xs[-1] - gx, ys[-1] - gy) > 0.05:
        snap = _segment_traversal_cost(
            xs[-1],
            ys[-1],
            gx,
            gy,
            origin_x,
            origin_y,
            resolution,
            width,
            height,
            arr,
            p.lethal_threshold,
            p.footprint_radius_m,
        )
        if snap is None:
            return None
        xs.append(gx)
        ys.append(gy)
        yaws.append(gyaw)

    # Final integrity: reject any illegal segment (defense in depth)
    for i in range(len(xs) - 1):
        if (
            _segment_traversal_cost(
                xs[i],
                ys[i],
                xs[i + 1],
                ys[i + 1],
                origin_x,
                origin_y,
                resolution,
                width,
                height,
                arr,
                p.lethal_threshold,
                p.footprint_radius_m,
            )
            is None
        ):
            return None

    return HybridPath(xs, ys, yaws, best.g)


def plan_grid_astar(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    data: Sequence[int],
    footprint_r: float = 0.25,
    stride_cells: int = 4,
) -> Optional[HybridPath]:
    """
    Fast 2D A* for long-range goals. Only security cells are hard obstacles.
    Used when Hybrid A* exhausts iterations on large arenas.
    """
    arr = np.asarray(data, dtype=np.int16).reshape(-1)
    if arr.size != width * height:
        raise ValueError("data size mismatch")
    res = float(resolution)
    stride = max(1, int(stride_cells))
    sx, sy, syaw = start
    gx, gy, gyaw = goal

    def to_idx(x: float, y: float) -> Optional[Tuple[int, int]]:
        ix = int(math.floor((x - origin_x) / res))
        iy = int(math.floor((y - origin_y) / res))
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            return None
        return ix, iy

    def to_xy(ix: int, iy: int) -> Tuple[float, float]:
        return origin_x + (ix + 0.5) * res, origin_y + (iy + 0.5) * res

    s = to_idx(sx, sy)
    g = to_idx(gx, gy)
    if s is None or g is None:
        return None
    if _is_security_cell(int(arr[s[1] * width + s[0]])) or _is_security_cell(
        int(arr[g[1] * width + g[0]])
    ):
        return None

    open_heap: List[Tuple[float, int, int, int]] = []
    counter = 0
    gscore = {s: 0.0}
    came: dict = {}
    heapq.heappush(open_heap, (math.hypot(g[0] - s[0], g[1] - s[1]), counter, s[0], s[1]))
    closed = set()
    neigh = (
        (stride, 0),
        (-stride, 0),
        (0, stride),
        (0, -stride),
        (stride, stride),
        (-stride, stride),
        (stride, -stride),
        (-stride, -stride),
    )

    found: Optional[Tuple[int, int]] = None
    while open_heap:
        _f, _, x, y = heapq.heappop(open_heap)
        if (x, y) in closed:
            continue
        closed.add((x, y))
        if abs(x - g[0]) <= stride and abs(y - g[1]) <= stride:
            found = (x, y)
            break
        for dx, dy in neigh:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if (nx, ny) in closed:
                continue
            v = int(arr[ny * width + nx])
            if _is_security_cell(v):
                continue
            # Reject moves that cross security along the segment
            x0, y0 = to_xy(x, y)
            x1, y1 = to_xy(nx, ny)
            if (
                _segment_traversal_cost(
                    x0, y0, x1, y1, origin_x, origin_y, res, width, height, arr, 90, footprint_r
                )
                is None
            ):
                continue
            step = math.hypot(dx, dy) * (1.0 + 0.02 * float(max(0, min(v if v >= 0 else 0, 100))))
            ng = gscore[(x, y)] + step
            key = (nx, ny)
            if key in gscore and ng >= gscore[key]:
                continue
            gscore[key] = ng
            came[key] = (x, y)
            counter += 1
            h = math.hypot(g[0] - nx, g[1] - ny)
            heapq.heappush(open_heap, (ng + h, counter, nx, ny))

    if found is None:
        return None

    cells = [found]
    cur = found
    while cur in came:
        cur = came[cur]
        cells.append(cur)
    cells.reverse()

    xs = [sx]
    ys = [sy]
    yaws = [syaw]
    for ix, iy in cells[1:]:
        x, y = to_xy(ix, iy)
        yaw = math.atan2(y - ys[-1], x - xs[-1])
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
    # snap to goal if free
    if math.hypot(xs[-1] - gx, ys[-1] - gy) > 0.05:
        if (
            _segment_traversal_cost(
                xs[-1], ys[-1], gx, gy, origin_x, origin_y, res, width, height, arr, 90, footprint_r
            )
            is None
        ):
            return None
        xs.append(gx)
        ys.append(gy)
        yaws.append(gyaw)
    return HybridPath(xs, ys, yaws, float(gscore.get(found, 0.0)))


def plan_global_path(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    data: Sequence[int],
    params: Optional[HybridAstarParams] = None,
) -> Tuple[Optional[HybridPath], str]:
    """
    Global path for arena nav.
    Long goals: fast 2D A* first (Hybrid alone often exhausts iters on 30m+).
    Short goals: Hybrid first for smoother heading, then grid fallback.
    Security cells remain infinite in both.
    """
    p = params or HybridAstarParams()
    dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
    stride = max(3, int(round(0.8 / max(resolution, 1e-6))))

    def _grid() -> Optional[HybridPath]:
        return plan_grid_astar(
            start,
            goal,
            origin_x,
            origin_y,
            resolution,
            width,
            height,
            data,
            footprint_r=p.footprint_radius_m,
            stride_cells=stride,
        )

    if dist >= 12.0:
        path2 = _grid()
        if path2 is not None and path2.xs:
            return path2, "grid"
        path = plan_hybrid_astar(
            start, goal, origin_x, origin_y, resolution, width, height, data, p
        )
        if path is not None and path.xs:
            return path, "hybrid"
        return None, "none"

    path = plan_hybrid_astar(
        start, goal, origin_x, origin_y, resolution, width, height, data, p
    )
    if path is not None and path.xs:
        return path, "hybrid"
    path2 = _grid()
    if path2 is not None and path2.xs:
        return path2, "grid"
    return None, "none"
