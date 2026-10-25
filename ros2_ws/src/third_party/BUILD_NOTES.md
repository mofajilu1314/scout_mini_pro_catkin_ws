# FAST_LIO vendor status (2026-09-05)

## Clone

- Direct `git clone https://github.com/hku-mars/FAST_LIO.git` from this workstation once timed out / produced an incomplete checkout.
- Only use when network allows and the training/Gazebo session is idle:

```bash
: "${SCOUT_WS_ROOT:?Set SCOUT_WS_ROOT to the workspace root first}"
bash "$SCOUT_WS_ROOT/ros2_ws/src/third_party/clone_fast_lio.sh"
```

- The helper refuses to overwrite an existing `third_party/FAST_LIO` path. Inspect or move a partial checkout manually before retrying; it never deletes it automatically.
- Upstream: https://github.com/hku-mars/FAST_LIO
- Public topic shape reference: `scout_long_nav/config/public_topic_contract.yaml`. Vehicle-specific topics, calibration and credentials belong to a separately reviewed private deployment and are not part of this workspace export.

## Build (idle window only)

Do **not** apt-install heavy deps while TD3 training owns the machine.

Expected (after successful clone; package name may be `fast_lio`):

```bash
: "${SCOUT_WS_ROOT:?Set SCOUT_WS_ROOT to the workspace root first}"
cd "$SCOUT_WS_ROOT/ros2_ws"
colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Likely missing system deps (install later)

- `libeigen3-dev`, PCL, `ros-humble-pcl-conversions`, `ros-humble-tf2-*`
- Livox driver **not** required if using standard `sensor_msgs/PointCloud2` on `/rslidar_points` (may need small adapter vs Livox CustomMsg)

## Code-phase decision

- **Not running** FAST_LIO nodes.
- Downstream long-nav already consumes `/long_nav/odom` via `pose_stub_node` so the stack is ready when LIO appears.
