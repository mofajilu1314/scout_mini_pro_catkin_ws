# third_party — 可选外部依赖（勿在 TD3 训练期间操作）

## FAST_LIO

- Upstream: https://github.com/hku-mars/FAST_LIO
- 角色：未来现场 LiDAR-IMU 里程计/建图的可选外部依赖，不是当前仿真或 3D 冒烟的前提。
- 当前状态：工作区**不随附** FAST_LIO 源码，也没有可由 `colcon` 发现的 `fast_lio` 包；不要在未克隆成功前执行 build 命令。
- 只有在训练空闲、网络可用且用户明确需要该可选依赖时，先运行受控克隆脚本：

```bash
: "${SCOUT_WS_ROOT:?Set SCOUT_WS_ROOT to the workspace root first}"
bash "$SCOUT_WS_ROOT/ros2_ws/src/third_party/clone_fast_lio.sh"
```

- 成功克隆并检查其中实际存在 `package.xml` 后，才可在空闲窗口构建：

```bash
# Install missing deps only when training is idle — do not apt during long runs.
: "${SCOUT_WS_ROOT:?Set SCOUT_WS_ROOT to the workspace root first}"
cd "$SCOUT_WS_ROOT/ros2_ws"
colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

上游包名和 ROS 发行版可能需要 Humble 兼容分支。若 `colcon` 失败，应在
[`BUILD_NOTES.md`](BUILD_NOTES.md) 记录缺失依赖后停止；不要在长训练期间强行安装依赖。

### Scout topic remap (planned)

| Scout | FAST_LIO typical |
|-------|------------------|
| `/rslidar_points` | cloud input (may need PointCloud2 adapter) |
| IMU on robot | `/imu/data` or manufacturer topic |
| Output odom | remap → `/long_nav/odom` |

外参和车辆话题属于独立的私有部署审查内容；当前公开 3D 包不提供实车部署配置。
