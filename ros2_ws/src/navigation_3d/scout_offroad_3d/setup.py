import os
from glob import glob

from setuptools import find_packages, setup

package_name = "scout_offroad_3d"


def _active_assets(pattern):
    """Return installable assets, excluding recoverable backup files.

    Backup meshes/worlds belong in ``archive/asset_backups``.  Filtering them
    here prevents a stale setuptools manifest from making a new build depend
    on a file that was deliberately archived.
    """
    return [path for path in glob(pattern) if ".bak" not in os.path.basename(path)]


def _walk_data(directory):
    """Install all files under directory into share/<pkg>/<directory>/..."""
    entries = []
    for root, _dirs, files in os.walk(directory):
        if not files:
            continue
        dest = os.path.join("share", package_name, root)
        active_files = [
            os.path.join(root, filename)
            for filename in files
            if ".bak" not in filename
        ]
        if active_files:
            entries.append((dest, active_files))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), _active_assets("launch/*.py")),
        (os.path.join("share", package_name, "config"), _active_assets("config/*")),
        (os.path.join("share", package_name, "worlds"), _active_assets("worlds/*")),
        *(_walk_data("models")),
        *(_walk_data("arena")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="scout_offroad",
    maintainer_email="todo@todo.todo",
    description="Off-road TD3 training for Scout Mini (ROS2 Humble)",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pointcloud_bridge_node = scout_offroad_3d.pointcloud_bridge_node:main",
            "range_features_node = scout_offroad_3d.range_features_node:main",
            "pick_random_world = scout_offroad_3d.pick_random_world:main",
            "resolve_world_spawn = scout_offroad_3d.resolve_world_spawn:main",
            "train_td3_node = scout_offroad_3d.train_td3_node:main_entry",
            "infer_td3_node = scout_offroad_3d.infer_td3_node:main",
            "plot_td3_metrics = scout_offroad_3d.plot_td3_metrics:main",
            "plot_td3_eval_viz = scout_offroad_3d.plot_td3_eval_viz:main",
            "plot_td3_loss_landscape = scout_offroad_3d.plot_td3_loss_landscape:main",
            "plot_td3_actions = scout_offroad_3d.plot_td3_actions:main",
            "record_camera_stub = scout_offroad_3d.record_camera_stub:main",
            "train_goal_marker = scout_offroad_3d.train_goal_marker_node:main",
            "depth_viz_bridge = scout_offroad_3d.depth_viz_bridge_node:main",
        ],
    },
)
