"""安全边界内 costmap 覆盖率计算。"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml


def _point_in_poly(x: float, y: float, poly: List[Tuple[float, float]]) -> bool:
    # ray casting
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-18) + xi
        ):
            inside = not inside
        j = i
    return inside


def load_costmap_array(map_dir: Path, stem: str = "arena_trav") -> Tuple[np.ndarray, dict]:
    yaml_path = map_dir / f"{stem}.yaml"
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    img = map_dir / meta.get("image", f"{stem}.pgm")
    if not img.is_absolute():
        img = map_dir / img
    raw = img.read_bytes()
    j = 0
    lines = []
    while len(lines) < 3:
        nl = raw.find(b"\n", j)
        line = raw[j:nl]
        j = nl + 1
        if line.startswith(b"#"):
            continue
        lines.append(line)
    w, h = map(int, lines[1].split())
    arr = np.frombuffer(raw[j : j + w * h], dtype=np.uint8).reshape((h, w))
    return arr, meta


def coverage_inside_boundary(
    map_dir: Path,
    polygon: List[Tuple[float, float]],
    stem: str = "arena_trav",
    stride: int = 2,
) -> Tuple[float, dict]:
    """
    覆盖率 = 多边形内「已知栅格」/ 多边形内全部采样栅格。
    已知：像素 != 255（unknown）。lethal(100) 与 free/cost 都算已覆盖勘察。
    """
    if len(polygon) < 3:
        return 0.0, {"error": "polygon_too_small"}
    arr, meta = load_costmap_array(map_dir, stem)
    res = float(meta.get("resolution", 0.08))
    origin = meta.get("origin", [-40.0, -40.0, 0.0])
    ox, oy = float(origin[0]), float(origin[1])
    h, w = arr.shape
    # ROS OccupancyGrid row0 = image bottom
    total = 0
    known = 0
    for r in range(0, h, stride):
        for c in range(0, w, stride):
            wx = ox + (c + 0.5) * res
            wy = oy + (h - r - 0.5) * res
            if not _point_in_poly(wx, wy, polygon):
                continue
            total += 1
            if int(arr[r, c]) != 255:
                known += 1
    ratio = (known / total) if total else 0.0
    return ratio, {"total": total, "known": known, "stride": stride, "resolution": res}
