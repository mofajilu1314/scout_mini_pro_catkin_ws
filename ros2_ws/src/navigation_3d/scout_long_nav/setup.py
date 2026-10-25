from setuptools import find_packages, setup
import os
from glob import glob

package_name = "scout_long_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "NOTICE.md", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="scout_offroad",
    maintainer_email="todo@todo.todo",
    description="Long-range nav: costmap + Hybrid A* + Arena look-ahead → TD3",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arena_local_goal_node = scout_long_nav.arena_local_goal_node:main",
            "goal_bridge_node = scout_long_nav.goal_bridge_node:main",
            "fake_path_node = scout_long_nav.fake_path_node:main",
            "cloud_to_costmap_node = scout_long_nav.cloud_to_costmap_node:main",
            "hybrid_astar_node = scout_long_nav.hybrid_astar_node:main",
            "pose_stub_node = scout_long_nav.pose_stub_node:main",
            "plot_long_nav_metrics = scout_long_nav.plot_long_nav_metrics:main",
            "save_costmap_node = scout_long_nav.save_costmap_node:main",
            "static_costmap_node = scout_long_nav.static_costmap_node:main",
            "goal_pose_relay_node = scout_long_nav.goal_pose_relay_node:main",
            "bake_heightmap_trav = scout_long_nav.bake_heightmap_trav:main",
            "flysky_js_teleop = scout_long_nav.flysky_js_teleop:main",
            "costmap_color_cloud_node = scout_long_nav.costmap_color_cloud_node:main",
            "boundary_overlay_node = scout_long_nav.boundary_overlay_node:main",
            "coverage_supervisor_node = scout_long_nav.coverage_supervisor_node:main",
            "rollover_guard_node = scout_long_nav.rollover_guard_node:main",
            "nav_cmd_gate_node = scout_long_nav.nav_cmd_gate_node:main",
        ],
    },
)
