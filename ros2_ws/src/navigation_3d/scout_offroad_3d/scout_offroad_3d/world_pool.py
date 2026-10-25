# -*- coding: utf-8 -*-
"""TD3 bumpy world pool — single source of stem names.

Active layout: exactly 4 train / switch / viz worlds
  bumpy_offroad … _4  (uneven_ground_td3_1…4)

Heightmaps stay unique: terrain_td3_N.png (do not reuse generic terrain.png).
"""

from __future__ import annotations

import glob
import os

TRAIN_WORLD_STEMS = (
    "bumpy_offroad",
    "bumpy_offroad_2",
    "bumpy_offroad_3",
    "bumpy_offroad_4",
)


def world_stem(path_or_name: str) -> str:
    base = os.path.basename(str(path_or_name or "").strip())
    if base.endswith(".world"):
        base = base[: -len(".world")]
    return base


def filter_world_paths(paths, stems) -> list:
    """Keep paths whose basename stem is in stems (order follows stems)."""
    by_stem = {}
    for p in paths or []:
        by_stem[world_stem(p)] = p
    out = []
    for s in stems:
        p = by_stem.get(world_stem(s))
        if p:
            out.append(p)
    return out


def list_train_world_paths(worlds_dir: str) -> list:
    cands = sorted(glob.glob(os.path.join(worlds_dir, "*.world")))
    return filter_world_paths(cands, TRAIN_WORLD_STEMS)


def resolve_world_path(worlds_dir: str, stem_or_path: str) -> str | None:
    if not stem_or_path:
        return None
    if os.path.isabs(stem_or_path) and os.path.isfile(stem_or_path):
        return stem_or_path
    stem = world_stem(stem_or_path)
    path = os.path.join(worlds_dir, stem + ".world")
    if os.path.isfile(path):
        return path
    return None
