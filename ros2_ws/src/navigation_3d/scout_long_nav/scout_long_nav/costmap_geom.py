# -*- coding: utf-8 -*-
"""Geometric point-cloud → 2D cost / traversability grid (offline-testable)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass
class CostmapParams:
    resolution: float = 0.2
    size_m: float = 40.0  # square window centered at robot
    slope_max: float = 0.45  # ~24 deg rise/run; above → lethal
    alpha_slope: float = 80.0
    beta_rough: float = 40.0
    lethal: int = 100
    free: int = 0
    unknown: int = -1
    min_points_per_cell: int = 2


@dataclass
class CostmapResult:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: np.ndarray  # int8 flat row-major, OccupancyGrid convention


def _cell_indices(
    xy: np.ndarray, origin_x: float, origin_y: float, res: float, w: int, h: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    ix = np.floor((xy[:, 0] - origin_x) / res).astype(np.int32)
    iy = np.floor((xy[:, 1] - origin_y) / res).astype(np.int32)
    mask = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    return ix[mask], iy[mask], mask


def points_to_costmap(
    points_xyz: np.ndarray,
    center_x: float,
    center_y: float,
    params: Optional[CostmapParams] = None,
) -> CostmapResult:
    """
    Build a local OccupancyGrid-like cost layer from Nx3 points in a fixed frame.

    Cost model (README4):
      c = alpha * (s / s_max) + beta * roughness, clipped to [0, 99];
      lethal if slope > s_max or height jump too large.
    """
    p = params or CostmapParams()
    half = 0.5 * p.size_m
    origin_x = center_x - half
    origin_y = center_y - half
    w = h = int(round(p.size_m / p.resolution))
    if w < 2:
        raise ValueError("costmap too small")

    data = np.full((h, w), p.unknown, dtype=np.int8)
    if points_xyz is None or len(points_xyz) == 0:
        return CostmapResult(w, h, p.resolution, origin_x, origin_y, data.reshape(-1))

    pts = np.asarray(points_xyz, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError("points_xyz must be Nx3")

    ix, iy, mask = _cell_indices(pts[:, :2], origin_x, origin_y, p.resolution, w, h)
    z = pts[mask, 2]

    # Accumulate sum/sumsq/count per cell
    count = np.zeros((h, w), dtype=np.int32)
    z_sum = np.zeros((h, w), dtype=np.float64)
    z_sumsq = np.zeros((h, w), dtype=np.float64)
    np.add.at(count, (iy, ix), 1)
    np.add.at(z_sum, (iy, ix), z)
    np.add.at(z_sumsq, (iy, ix), z * z)

    mean = np.zeros((h, w), dtype=np.float64)
    rough = np.zeros((h, w), dtype=np.float64)
    valid = count >= p.min_points_per_cell
    mean[valid] = z_sum[valid] / count[valid]
    # population variance
    rough[valid] = np.maximum(
        z_sumsq[valid] / count[valid] - mean[valid] ** 2, 0.0
    )
    rough = np.sqrt(rough)

    # Slope from finite differences of mean height
    slope = np.zeros((h, w), dtype=np.float64)
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if not valid[y, x]:
                continue
            neighbors = []
            for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                if valid[y + dy, x + dx]:
                    neighbors.append(
                        abs(mean[y + dy, x + dx] - mean[y, x]) / p.resolution
                    )
            if neighbors:
                slope[y, x] = max(neighbors)

    out = np.full((h, w), p.unknown, dtype=np.int8)
    for y in range(h):
        for x in range(w):
            if not valid[y, x]:
                continue
            s = slope[y, x]
            if s > p.slope_max:
                out[y, x] = p.lethal
                continue
            c = p.alpha_slope * (s / max(p.slope_max, 1e-6)) + p.beta_rough * rough[y, x]
            out[y, x] = int(max(p.free, min(99, round(c))))

    return CostmapResult(w, h, p.resolution, origin_x, origin_y, out.reshape(-1))


def synthetic_plane(n: int = 400, z: float = 0.0) -> np.ndarray:
    xs = np.linspace(-5, 5, int(np.sqrt(n)))
    ys = np.linspace(-5, 5, int(np.sqrt(n)))
    xx, yy = np.meshgrid(xs, ys)
    zz = np.full_like(xx, z)
    return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)


def synthetic_ramp(n: int = 400, slope: float = 0.3) -> np.ndarray:
    pts = synthetic_plane(n, 0.0)
    pts[:, 2] = slope * pts[:, 0]
    return pts


def synthetic_cliff(n: int = 400, drop: float = 2.0) -> np.ndarray:
    pts = synthetic_plane(n, 0.0)
    pts[pts[:, 0] > 0, 2] = -drop
    return pts
