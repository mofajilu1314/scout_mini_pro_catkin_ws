cmake_minimum_required(VERSION 3.0.2)
project(scout_offroad_3d)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/range_features_node.py
  scripts/pointcloud_bridge_node.py
  scripts/train_td3_node.py
  scripts/infer_td3_node.py
  scripts/pick_random_world.py
  scripts/resolve_world_spawn.py
  scripts/plot_td3_metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# TD3 python package (imported by train/infer nodes via scripts/ on PYTHONPATH)
install(DIRECTORY scripts/td3
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN "*.pyc" EXCLUDE
)
