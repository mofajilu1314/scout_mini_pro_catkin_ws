# scout_long_nav

ROS2 Humble 长途导航包：可通行代价图 → Hybrid A* → 前瞻子目标 → TD3 接口。

## 当前入口

从工作区根目录设置 `SCOUT_WS_ROOT`，然后按 [`3D/README4.md`](../../../../3D/README4.md) 的边界执行离线规划冒烟。默认脚本只运行单元测试：

```bash
bash "$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav/scripts/smoke_long_nav_planning.sh"
```

它不得与 TD3 训练同时占用仿真。需要启动 Gazebo 的 launch 只可在训练空闲、外部占用已确认的情况下使用。

## 包内边界

- `config/sim_stub.yaml` 和 `launch/stage_*`：仿真与阶段化接口。
- `config/public_topic_contract.yaml` 与 `launch/long_nav_real.launch.py`：公开的、默认无节点的规划话题检查入口。显式 `start_planning:=true` 后也只启动隔离的规划观察节点；它不启动驱动、TD3 推理、`nav_cmd_gate`，也不发布 `/cmd_vel`。
- `config/real_robot_long.yaml`：本地开发期占位，公开导出时会被排除。公开 launch 不依赖它；任何现场专用话题配置必须由独立审查的私有部署显式传入，不能把该入口当作部署命令。
- `tests/`：不依赖 Gazebo 的单元测试。

地图与调试输出必须写入工作区规定的位置，不能回写旧顶层 `src/`、编辑器目录或训练 run。
