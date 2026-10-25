import os
from glob import glob

from setuptools import find_packages, setup

package_name = "scout_mini_pro_description"


def _walk_data_files(source_dir):
    entries = []
    for root, _dirs, files in os.walk(source_dir):
        if not files:
            continue
        rel = os.path.relpath(root, source_dir)
        dest = (
            os.path.join("share", package_name, source_dir, rel)
            if rel != "."
            else os.path.join("share", package_name, source_dir)
        )
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ]
    + _walk_data_files("xacro")
    + _walk_data_files("world"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="scout_offroad",
    maintainer_email="todo@todo.todo",
    description="Scout Mini Pro Gazebo simulation description for ROS2",
    license="BSD",
    entry_points={
        "console_scripts": [
            "spawn_scout_clean_prefix = "
            "scout_mini_pro_description.spawn_scout_clean_prefix:main",
            "odom_to_tf_official = "
            "scout_mini_pro_description.odom_to_tf_official:main",
        ],
    },
)
