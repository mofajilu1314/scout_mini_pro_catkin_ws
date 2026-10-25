"""Public, gated long-navigation topic-contract inspection launch.

This compatibility-named launch is deliberately *not* a real-robot deployment
entry point.  It never starts a robot driver, TD3 inference, ``nav_cmd_gate``,
or a publisher to ``/cmd_vel``.  By default it starts no nodes at all.  An
operator may explicitly set ``start_planning:=true`` to inspect planning-only
topics after an independent, reviewed deployment has supplied sensor data.

The public default config is generic and contains no connection details.  A
private deployment can explicitly pass a locally reviewed absolute YAML path
through ``topic_contract``; private vehicle-specific configuration files are
neither a public-export dependency nor a deployment instruction.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("scout_long_nav")
    enabled = LaunchConfiguration("start_planning").perform(context).strip().lower()
    if enabled != "true":
        return [
            LogInfo(
                msg=(
                    "[scout_long_nav] public topic-contract launch is safely idle. "
                    "No planning nodes, driver, TD3 inference, nav_cmd_gate, or "
                    "/cmd_vel publisher was started. To inspect planning-only "
                    "topics after separate approval, pass start_planning:=true."
                )
            )
        ]

    cfg = LaunchConfiguration("topic_contract").perform(context).strip()
    if not cfg or not os.path.isfile(cfg):
        raise RuntimeError(
            "topic_contract must be a readable YAML file. The public default is "
            "scout_long_nav/config/public_topic_contract.yaml; a vehicle-specific "
            "file must be supplied explicitly from an approved private deployment."
        )
    long_cfg = os.path.join(pkg, "config", "long_nav.yaml")
    return [
        LogInfo(
            msg=(
                "[scout_long_nav] planning-only topic inspection: no driver, TD3 "
                "inference, nav_cmd_gate, or /cmd_vel publisher. TD3 goal output "
                "is isolated under /long_nav/inspection/."
            )
        ),
        Node(
            package="scout_long_nav",
            executable="pose_stub_node",
            name="pose_stub_node",
            output="screen",
            parameters=[cfg],
        ),
        Node(
            package="scout_long_nav",
            executable="cloud_to_costmap_node",
            name="cloud_to_costmap_node",
            output="screen",
            parameters=[long_cfg, cfg],
        ),
        Node(
            package="scout_long_nav",
            executable="hybrid_astar_node",
            name="hybrid_astar_node",
            output="screen",
            parameters=[long_cfg],
        ),
        Node(
            package="scout_long_nav",
            executable="arena_local_goal_node",
            name="arena_local_goal_node",
            output="screen",
            parameters=[
                long_cfg,
                {"local_goal_topic": "/long_nav/inspection/local_goal"},
            ],
        ),
        Node(
            package="scout_long_nav",
            executable="goal_bridge_node",
            name="goal_bridge_node",
            output="screen",
            parameters=[
                long_cfg,
                {
                    "local_goal_topic": "/long_nav/inspection/local_goal",
                    "train_goal_topic": "/long_nav/inspection/train_goal",
                },
            ],
        ),
    ]


def generate_launch_description():
    pkg = get_package_share_directory("scout_long_nav")
    public_contract = os.path.join(pkg, "config", "public_topic_contract.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_planning",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Safety gate. false starts no nodes; true starts only the "
                    "read-only planning inspection nodes."
                ),
            ),
            DeclareLaunchArgument(
                "topic_contract",
                default_value=public_contract,
                description=(
                    "Public generic YAML by default. A reviewed private deployment "
                    "may explicitly provide its own readable YAML path."
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
