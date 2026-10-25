# LONG-NAV — planning-only include (legacy).
# Prefer the two-phase idle-window playbook:
#   ros2 launch scout_long_nav long_nav_map.launch.py
#   ros2 launch scout_long_nav long_nav_nav.launch.py
# See 3D/README4.md §7ops. NEVER with train_td3 / shared gzserver.
#
# This file does NOT include Gazebo or TD3 infer.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("scout_long_nav")
    stage_c = os.path.join(pkg, "launch", "stage_c_planning.launch.py")
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "[scout_long_nav] long_nav_full_sim: includes planning only. "
                    "Gazebo/TD3 must be started by YOU in an idle window."
                )
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(stage_c)),
        ]
    )
