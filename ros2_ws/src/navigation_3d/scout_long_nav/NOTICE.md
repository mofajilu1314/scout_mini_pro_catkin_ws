# scout_long_nav — third-party attributions

## Hybrid A* (simplified port inspired by PythonRobotics)

- Upstream: https://github.com/AtsushiSakai/PythonRobotics
  PathPlanning/HybridAStar (MIT License)
- Authors: Atsushi Sakai / Zheng Zh and contributors
- This package implements a reduced kinematic Hybrid A* suitable for OccupancyGrid
  costmaps (no Reeds-Shepp / scipy KDTree dependency). Algorithm ideas follow
  Dolgov et al. Hybrid A* and the PythonRobotics teaching implementation.

## Arena-style look-ahead

- Concept from Arena-Rosnav (Kästner et al., arXiv:2104.03616)
- Upstream repo (not vendored): https://github.com/ignc-research/arena-rosnav
- This package only reimplements look-ahead local sub-goal selection.

## FAST-LIO2

- Upstream: https://github.com/hku-mars/FAST_LIO
- Vendored separately under ros2_ws/src/third_party/ when present; not part of
  this Python package's runtime during the code-first phase.
