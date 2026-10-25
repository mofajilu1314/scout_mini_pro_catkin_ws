# -*- coding: utf-8 -*-
"""Save / load traversability cost OccupancyGrid as yaml + pgm."""

from __future__ import annotations

import os
from typing import Any, Dict, Tuple

import numpy as np
import yaml


def occupancy_to_arrays(
    width: int,
    height: int,
    data,
) -> np.ndarray:
    """OccupancyGrid.data → int16 HxW (-1 unknown, 0..100)."""
    arr = np.asarray(list(data), dtype=np.int16).reshape((height, width))
    return arr


def encode_pgm(grid: np.ndarray) -> np.ndarray:
    """
    Encode cost grid to uint8 PGM pixels.
    unknown (-1) → 255; security (1000) → 254; costs 0..100 → same value.
    """
    from scout_long_nav.cost_codes import PGM_SECURITY, SECURITY_COST, TERRAIN_MAX

    out = np.empty(grid.shape, dtype=np.uint8)
    g = np.asarray(grid)
    unknown = g < 0
    security = (~unknown) & (g >= SECURITY_COST)
    known = ~(unknown | security)
    out[unknown] = 255
    out[security] = PGM_SECURITY
    out[known] = np.clip(g[known], 0, TERRAIN_MAX).astype(np.uint8)
    return out


def decode_pgm(pixels: np.ndarray) -> np.ndarray:
    """PGM uint8 → int16 cost grid (254 → security 1000)."""
    from scout_long_nav.cost_codes import PGM_SECURITY, SECURITY_COST

    g = pixels.astype(np.int16)
    g[g == 255] = -1
    g[g == PGM_SECURITY] = SECURITY_COST
    return g


def save_trav_map(
    path_yaml: str,
    grid: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    frame_id: str = "odom",
) -> Tuple[str, str]:
    """Write arena_trav.yaml + .pgm next to it. Returns (yaml, pgm)."""
    path_yaml = os.path.abspath(path_yaml)
    out_dir = os.path.dirname(path_yaml)
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(path_yaml))[0]
    path_pgm = os.path.join(out_dir, stem + ".pgm")
    h, w = grid.shape
    pixels = encode_pgm(grid)

    # P5 binary pgm
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    with open(path_pgm, "wb") as f:
        f.write(header)
        # pgm rows top-to-bottom; OccupancyGrid is row-major bottom-up in ROS?
        # ROS OccupancyGrid: row 0 is bottom (y=origin). Standard map_server flips.
        # We store row 0 = grid[0] (bottom) as last image row for map_server-like viewers.
        flipped = np.flipud(pixels)
        f.write(flipped.tobytes())

    meta: Dict[str, Any] = {
        "image": os.path.basename(path_pgm),
        "mode": "traversability_cost",
        "resolution": float(resolution),
        "origin": [float(origin_x), float(origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "width": int(w),
        "height": int(h),
        "frame_id": str(frame_id),
        "note": "255=unknown(-1); 0-100=terrain cost; 254=security(1000). Row0 of OccupancyGrid = image bottom.",
    }
    with open(path_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(meta, f, default_flow_style=False, sort_keys=False)
    return path_yaml, path_pgm


def load_trav_map(path_yaml: str) -> Dict[str, Any]:
    """Load yaml+pgm → dict with keys grid, resolution, origin_x/y, frame_id, width, height."""
    path_yaml = os.path.abspath(path_yaml)
    with open(path_yaml, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    img_name = meta.get("image")
    if not img_name:
        raise ValueError("yaml missing image:")
    path_pgm = img_name if os.path.isabs(img_name) else os.path.join(
        os.path.dirname(path_yaml), img_name
    )
    with open(path_pgm, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError("only binary P5 pgm supported")
        # skip comments
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        wh = line.split()
        while len(wh) < 2:
            wh += f.readline().split()
        w, h = int(wh[0]), int(wh[1])
        maxval = int(f.readline().split()[0])
        if maxval != 255:
            raise ValueError("expected maxval 255")
        raw = f.read(w * h)
    pixels = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
    pixels = np.flipud(pixels)  # image top → OccupancyGrid row0 bottom
    grid = decode_pgm(pixels)
    origin = meta.get("origin") or [0.0, 0.0, 0.0]
    return {
        "grid": grid,
        "resolution": float(meta["resolution"]),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "frame_id": str(meta.get("frame_id") or "odom"),
        "width": int(w),
        "height": int(h),
        "meta": meta,
    }


def grid_from_height_dem(
    height: np.ndarray,
    size_m: float,
    origin_x: float,
    origin_y: float,
    slope_max: float = 0.45,
    alpha_slope: float = 80.0,
    beta_rough: float = 40.0,
) -> Tuple[np.ndarray, float]:
    """
    height: HxW meters. Returns (cost int16 HxW, resolution).
    Roughness uses local 3x3 height std; slope from neighbor finite differences.
    """
    h, w = height.shape
    resolution = size_m / float(w)
    slope = np.zeros((h, w), dtype=np.float64)
    rough = np.zeros((h, w), dtype=np.float64)
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            patch = height[y - 1 : y + 2, x - 1 : x + 2]
            rough[y, x] = float(np.std(patch))
            neighbors = [
                abs(height[y, x + 1] - height[y, x]) / resolution,
                abs(height[y, x - 1] - height[y, x]) / resolution,
                abs(height[y + 1, x] - height[y, x]) / resolution,
                abs(height[y - 1, x] - height[y, x]) / resolution,
            ]
            slope[y, x] = max(neighbors)
    out = np.zeros((h, w), dtype=np.int16)
    for y in range(h):
        for x in range(w):
            s = slope[y, x]
            if s > slope_max:
                out[y, x] = 100
            else:
                c = alpha_slope * (s / max(slope_max, 1e-6)) + beta_rough * rough[y, x]
                out[y, x] = int(max(0, min(99, round(c))))
    return out, resolution
