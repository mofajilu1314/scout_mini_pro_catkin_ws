#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bake arena heightmap PNG → static traversability cost map (no Gazebo needed).

Example:
  ros2 run scout_long_nav bake_heightmap_trav \\
    --terrain "$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/models/uneven_ground_td3_1/materials/textures/terrain_td3_1.png" \\
    --out-dir "$SCOUT_WS_ROOT/3D/maps_long" --stem arena_trav
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

from scout_long_nav.isolation import DEFAULT_MAP_DIR, DEFAULT_MAP_STEM
from scout_long_nav.map_io import grid_from_height_dem, save_trav_map
from scout_workspace_common import workspace_path


_PACKAGE_TERRAIN_RELATIVE = (
    "models",
    "uneven_ground_td3_1",
    "materials",
    "textures",
    "terrain_td3_1.png",
)


def default_terrain_path() -> str:
    """Return the reproducible packaged heightmap for offline map baking.

    The full long-nav arena may use a separately installed Gazebo model.  That
    asset is intentionally *not* guessed from a user cache: callers that need
    its exact texture must pass it with ``--terrain``.  The package-local TD3
    terrain keeps the no-Gazebo fallback portable and exportable.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = Path(get_package_share_directory("scout_offroad_3d")).joinpath(
            *_PACKAGE_TERRAIN_RELATIVE
        )
        if candidate.is_file():
            return str(candidate)
    except (ImportError, LookupError, OSError, RuntimeError):
        pass

    return str(
        workspace_path(
            "ros2_ws",
            "src",
            "navigation_3d",
            "scout_offroad_3d",
            *_PACKAGE_TERRAIN_RELATIVE,
            package_name="scout_long_nav",
            start_path=__file__,
        )
    )


def _load_height_png(path: str, size_m: float, height_amp: float) -> np.ndarray:
    try:
        from PIL import Image
    except ImportError:
        # Fallback: matplotlib / imageio not required; use raw via numpy if png via cv2
        try:
            import cv2

            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise RuntimeError("cv2 failed to read %s" % path)
            gray = img.astype(np.float64) / 255.0
        except Exception as e:
            raise SystemExit(
                "Need Pillow or OpenCV to read PNG (%s). pip/apt install python3-pil" % e
            )
    else:
        gray = np.asarray(Image.open(path).convert("L"), dtype=np.float64) / 255.0

    # Gazebo heightmap: typically white = high. Center about mid-grey.
    height = (gray - 0.5) * 2.0 * height_amp  # roughly [-amp, +amp] then scale
    # Prefer [0, amp] style matching size Z=2 in world: z = gray * height_amp
    height = gray * height_amp
    return height


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--terrain",
        default=default_terrain_path(),
        help=(
            "heightmap PNG; defaults to scout_offroad_3d's packaged terrain. "
            "Pass an explicit external arena texture when an exact arena map is needed."
        ),
    )
    p.add_argument("--size-m", type=float, default=80.0)
    p.add_argument("--height-amp", type=float, default=2.0, help="world heightmap Z size")
    p.add_argument("--out-dir", default=DEFAULT_MAP_DIR)
    p.add_argument("--stem", default=DEFAULT_MAP_STEM)
    p.add_argument("--slope-max", type=float, default=0.45)
    p.add_argument("--frame-id", default="odom")
    # Optional resample to fixed resolution
    p.add_argument("--resolution", type=float, default=0.2)
    args = p.parse_args(argv)
    args.terrain = os.path.abspath(os.path.expandvars(os.path.expanduser(args.terrain)))

    if not os.path.isfile(args.terrain):
        print(
            "terrain PNG missing: %s (pass --terrain with an explicit accessible PNG)"
            % args.terrain,
            file=sys.stderr,
        )
        return 2

    height = _load_height_png(args.terrain, args.size_m, args.height_amp)
    # Resample to resolution grid
    n = int(round(args.size_m / args.resolution))
    if height.shape[0] != n or height.shape[1] != n:
        try:
            from PIL import Image

            im = Image.fromarray((height / max(args.height_amp, 1e-6) * 255.0).astype(np.uint8))
            im = im.resize((n, n), Image.BILINEAR)
            height = np.asarray(im, dtype=np.float64) / 255.0 * args.height_amp
        except ImportError:
            ys = np.linspace(0, height.shape[0] - 1, n)
            xs = np.linspace(0, height.shape[1] - 1, n)
            # nearest
            yi = np.round(ys).astype(int)
            xi = np.round(xs).astype(int)
            height = height[np.ix_(yi, xi)]

    cost, res = grid_from_height_dem(
        height,
        size_m=args.size_m,
        origin_x=-0.5 * args.size_m,
        origin_y=-0.5 * args.size_m,
        slope_max=args.slope_max,
    )
    # grid_from_height_dem derives res from size/w; force our resolution
    res = args.resolution
    out_yaml = os.path.join(os.path.expanduser(args.out_dir), args.stem + ".yaml")
    y, pgm = save_trav_map(
        out_yaml,
        cost,
        res,
        origin_x=-0.5 * args.size_m,
        origin_y=-0.5 * args.size_m,
        frame_id=args.frame_id,
    )
    print("baked traversability cost map:")
    print("  ", y)
    print("  ", pgm)
    print("  shape=%s resolution=%.3f origin=(-%.1f,-%.1f)" % (cost.shape, res, 0.5 * args.size_m, 0.5 * args.size_m))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
