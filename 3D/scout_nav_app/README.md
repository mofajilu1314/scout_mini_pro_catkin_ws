# ScoutNav 3D navigation console

ScoutNav is the local GUI for the Scout 3D simulation/navigation workflow. It
uses the active ROS 2 workspace, RViz, Gazebo Classic, `scout_offroad_3d`, and
`scout_long_nav`; it is not a standalone robot-control deployment package.

## Start from a workspace

Use the workspace root rather than a fixed home-directory path:

```bash
export SCOUT_WS_ROOT=/absolute/path/to/scout-workspace
source /opt/ros/humble/setup.bash

cd "$SCOUT_WS_ROOT/ros2_ws"
colcon build --symlink-install
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"

python3 -m pip install --user -r "$SCOUT_WS_ROOT/3D/scripts/requirements-ros2.txt"
python3 -m pip install --user -r "$SCOUT_WS_ROOT/3D/scout_nav_app/requirements.txt"
bash "$SCOUT_WS_ROOT/3D/scout_nav_app/run.sh"
```

`SCOUT_WS_ROOT` is the canonical override. `CATKIN_WS` and `WS_ROOT` remain
accepted by the shared workspace helper for older shells.

## Default model

`config/models.yaml` registers Record 9 at
`archive/experiments/td3/runs/run_20260907_120841/`. It is the current
150-dimensional / 1024-hidden-unit **simulation-only** baseline. The GUI must
not treat it as a real-robot controller or use it to authorize physical motion.

The model loader validates the checkpoint directory and its parameter snapshot.
For an inference run, retain the complete checkpoint prefix required by the TD3
loader (`td3_actor`, `td3_actor_optimizer`, `td3_critic`, and
`td3_critic_optimizer`) together with the compatible configuration snapshot.

## Paths and logs

All local defaults are resolved through `scout_workspace_common`, so the GUI
works from any checkout location. Runtime diagnostics belong in
`3D/debug_logs/`; the application source directory is not a runtime log or
state location.

## Directory layout

- `scout_nav_app/` — GUI source
- `config/` — model and topic registry
- `resources/` — icon and RViz resources
- `tests/` — offline unit tests

Before starting a simulation, confirm no other module owns Gazebo or the
navigation slot. Do not use this GUI to start real-robot control without the
separate, reviewed real-robot workflow.

The historical ROS1 package in `archive/` is not an application dependency.
If a reviewed real-robot workflow must synchronize `scout_long_nav_ros1`, the
operator has to provide an explicit maintained local package directory through
the private `SCOUT_ROBOT_LONG_NAV_SOURCE` setting. Missing or invalid settings
are rejected rather than silently reading archived sources.
