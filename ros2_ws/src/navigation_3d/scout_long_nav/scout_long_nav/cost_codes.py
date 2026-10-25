# -*- coding: utf-8 -*-
"""Cost value conventions for trav + security fence.

Terrain (mapping): -1 unknown, 0..100 (100 = max mappable = sunset orange).
Security fence: 1000 (independent of mapping; bright red). OccupancyGrid is
int8 so on the wire we carry security as OCC_SECURITY=127 (infinite / hard block).
Terrain 0..100 is always passable (high finite cost); only security is infinite.
"""


from __future__ import annotations

import numpy as np

SECURITY_COST = 1000
TERRAIN_MAX = 100
OCC_SECURITY = 127  # int8 OccupancyGrid carrier for SECURITY_COST
PGM_SECURITY = 254  # PGM pixel for SECURITY_COST (255=unknown)


def to_occ_int8_cell(v: int) -> int:
    if v < 0:
        return -1
    if int(v) >= SECURITY_COST or int(v) == OCC_SECURITY:
        return OCC_SECURITY
    if int(v) > TERRAIN_MAX:
        return TERRAIN_MAX
    return int(v)


def grid_to_occ_int8(grid: np.ndarray) -> list:
    """HxW int16 → flat OccupancyGrid.data list (int8-safe)."""
    flat = np.asarray(grid, dtype=np.int32).reshape(-1)
    out = np.empty(flat.shape, dtype=np.int8)
    unknown = flat < 0
    security = (~unknown) & ((flat >= SECURITY_COST) | (flat == OCC_SECURITY))
    terrain = ~(unknown | security)
    out[unknown] = -1
    out[security] = OCC_SECURITY
    out[terrain] = np.clip(flat[terrain], 0, TERRAIN_MAX).astype(np.int8)
    return out.tolist()


def is_security_cost(v) -> bool:
    x = float(v)
    return x >= 101.0 or x >= SECURITY_COST
