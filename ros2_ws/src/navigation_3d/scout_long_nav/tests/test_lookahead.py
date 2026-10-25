# -*- coding: utf-8 -*-
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scout_long_nav.lookahead import (
    closest_point_on_polyline,
    lookahead_point,
    lookahead_waypoints,
    path_length,
)


def test_path_length_straight():
    pts = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
    assert abs(path_length(pts) - 7.0) < 1e-9


def test_closest_on_segment():
    pts = [(0.0, 0.0), (10.0, 0.0)]
    i, t, (cx, cy) = closest_point_on_polyline(pts, 3.0, 1.0)
    assert i == 0
    assert abs(cx - 3.0) < 1e-6
    assert abs(cy) < 1e-6


def test_lookahead_along_line():
    pts = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]
    g = lookahead_point(pts, 0.0, 0.0, 4.0)
    assert g is not None
    assert abs(g[0] - 4.0) < 1e-6
    assert abs(g[1]) < 1e-6


def test_lookahead_past_end():
    pts = [(0.0, 0.0), (2.0, 0.0)]
    g = lookahead_point(pts, 0.0, 0.0, 10.0)
    assert g == (2.0, 0.0)


def test_lookahead_from_mid_path():
    pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    g = lookahead_point(pts, 8.0, 0.1, 5.0)
    assert g is not None
    # ~2m left on first seg + 3m up second
    assert abs(g[0] - 10.0) < 0.2
    assert abs(g[1] - 3.0) < 0.3


def test_lookahead_waypoints_spacing():
    pts = [(0.0, 0.0), (10.0, 0.0)]
    wps = lookahead_waypoints(pts, 0.0, 0.0, 4.0, spacing_m=1.0)
    assert len(wps) >= 4
    assert abs(wps[0][0]) < 0.2
    assert abs(wps[-1][0] - 4.0) < 0.2
    # roughly 1m spacing
    for a, b in zip(wps[:-1], wps[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        assert 0.5 < d < 1.6
