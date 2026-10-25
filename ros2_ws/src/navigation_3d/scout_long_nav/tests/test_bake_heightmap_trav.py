# -*- coding: utf-8 -*-
from pathlib import Path

from scout_long_nav.bake_heightmap_trav import default_terrain_path


def test_default_terrain_is_packaged_and_not_user_cache():
    terrain = Path(default_terrain_path())

    assert terrain.is_file()
    assert terrain.name == "terrain_td3_1.png"
    assert "models" in terrain.parts
    assert ".gazebo" not in str(terrain)
