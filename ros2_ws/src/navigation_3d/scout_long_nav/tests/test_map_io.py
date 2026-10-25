# -*- coding: utf-8 -*-
import os
import tempfile

import numpy as np

from scout_long_nav.map_io import (
    decode_pgm,
    encode_pgm,
    load_trav_map,
    save_trav_map,
)


def test_roundtrip_trav_map():
    grid = np.full((20, 20), -1, dtype=np.int16)
    grid[5:15, 5:15] = 10
    grid[10, 10] = 100
    with tempfile.TemporaryDirectory() as td:
        ypath = os.path.join(td, "arena_trav.yaml")
        save_trav_map(ypath, grid, 0.5, -5.0, -5.0, frame_id="odom")
        loaded = load_trav_map(ypath)
        assert loaded["width"] == 20
        assert loaded["height"] == 20
        assert abs(loaded["resolution"] - 0.5) < 1e-9
        assert abs(loaded["origin_x"] + 5.0) < 1e-9
        np.testing.assert_array_equal(loaded["grid"], grid)


def test_encode_unknown():
    g = np.array([[-1, 0, 50, 100]], dtype=np.int16)
    pix = encode_pgm(g)
    assert pix[0, 0] == 255
    assert list(decode_pgm(pix)[0]) == [-1, 0, 50, 100]


def test_encode_security_1000():
    g = np.array([[-1, 0, 100, 1000]], dtype=np.int16)
    pix = encode_pgm(g)
    assert list(pix[0]) == [255, 0, 100, 254]
    assert list(decode_pgm(pix)[0]) == [-1, 0, 100, 1000]
