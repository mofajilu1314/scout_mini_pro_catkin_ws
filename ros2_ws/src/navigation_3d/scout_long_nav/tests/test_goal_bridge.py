# -*- coding: utf-8 -*-
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scout_long_nav.lookahead import pose_to_point_xy


def test_pose_to_point():
    x, y = pose_to_point_xy(1.5, -2.25)
    assert x == 1.5
    assert y == -2.25
