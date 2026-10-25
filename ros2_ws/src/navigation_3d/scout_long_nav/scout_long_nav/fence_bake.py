# -*- coding: utf-8 -*-
"""Rasterize safety-boundary polylines into security-cost (1000) cells."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from scout_long_nav.cost_codes import SECURITY_COST

Point = Tuple[float, float]
Segment = Tuple[Point, Point]


def segments_from_points(
    points: Sequence[Point], *, closed: bool
) -> List[Segment]:
    pts = list(points)
    if len(pts) < 2:
        return []
    segs = [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    if closed and len(pts) >= 3:
        segs.append((pts[-1], pts[0]))
    return segs


def bake_fence_into_grid(
    grid: np.ndarray,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    segments: Iterable[Segment],
    half_width_m: float = 0.25,
    lethal: int = SECURITY_COST,
) -> np.ndarray:
    """Return a copy of grid with thick fence corridors set to SECURITY_COST (1000)."""
    out = np.array(grid, dtype=np.int16, copy=True)
    h, w = out.shape
    res = float(resolution)
    if res <= 1e-9:
        return out
    hw_cells = max(1, int(math.ceil(float(half_width_m) / res)))
    r2 = hw_cells * hw_cells + 1

    for (x0, y0), (x1, y1) in segments:
        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        n = max(2, int(length / (res * 0.4)) + 1)
        for i in range(n):
            t = i / (n - 1)
            x = x0 + t * dx
            y = y0 + t * dy
            cx = int(math.floor((x - origin_x) / res))
            cy = int(math.floor((y - origin_y) / res))
            for oy in range(-hw_cells, hw_cells + 1):
                yy = cy + oy
                if yy < 0 or yy >= h:
                    continue
                for ox in range(-hw_cells, hw_cells + 1):
                    xx = cx + ox
                    if xx < 0 or xx >= w:
                        continue
                    if ox * ox + oy * oy <= r2:
                        out[yy, xx] = lethal
    return out
