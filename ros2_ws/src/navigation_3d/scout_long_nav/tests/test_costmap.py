# -*- coding: utf-8 -*-
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scout_long_nav.costmap_geom import (
    CostmapParams,
    points_to_costmap,
    synthetic_cliff,
    synthetic_plane,
    synthetic_ramp,
)


def test_plane_mostly_free():
    pts = synthetic_plane(1600, 0.0)
    res = points_to_costmap(
        pts, 0.0, 0.0, CostmapParams(size_m=20.0, resolution=0.5, min_points_per_cell=1)
    )
    data = res.data.reshape(res.height, res.width)
    known = data >= 0
    assert known.any()
    assert (data[known] < 50).mean() > 0.8


def test_cliff_has_lethal():
    pts = synthetic_cliff(1600, drop=2.0)
    res = points_to_costmap(
        pts,
        0.0,
        0.0,
        CostmapParams(size_m=20.0, resolution=0.4, slope_max=0.5, min_points_per_cell=1),
    )
    assert (res.data == 100).any()


def test_ramp_elevated_cost():
    pts = synthetic_ramp(1600, slope=0.35)
    res = points_to_costmap(
        pts,
        0.0,
        0.0,
        CostmapParams(size_m=20.0, resolution=0.4, slope_max=0.5, min_points_per_cell=1),
    )
    data = res.data
    known = data >= 0
    assert known.any()
    # Should not be all free zeros on a sustained ramp
    assert data[known].mean() > 5.0
