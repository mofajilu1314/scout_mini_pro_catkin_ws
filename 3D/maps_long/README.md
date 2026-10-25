# Long-nav static traversability cost maps (OccupancyGrid yaml+pgm).
# Produced by: long_nav_map.launch (Ctrl+C) or bake_heightmap_trav.
# Consumed by: long_nav_nav.launch → static_costmap_node.
# Do not confuse with TD3 training runs under 3D/runs/.
# The bake command defaults to a package-owned terrain texture.  To reproduce
# a separately installed arena exactly, pass its texture explicitly with
# `--terrain`; no user-local Gazebo cache is assumed.
