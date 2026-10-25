# Scout 导航工作区

这是整理后的 ROS2 Humble + Gazebo Classic 工作区。活动代码只在
`ros2_ws/src/`；本机只保留 3D、共享层和历史资料。2D 已独立外置到 GitHub，
不再作为本机活动模块。先读
[`AGENTS.md`](AGENTS.md) 和 [`MODULE_MAP.md`](MODULE_MAP.md)，再修改或运行。

## 建立工作环境

```bash
cd /path/to/scout-workspace
export SCOUT_WS_ROOT="$PWD"
source /opt/ros/humble/setup.bash
cd "$SCOUT_WS_ROOT/ros2_ws"
colcon build --symlink-install
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"
```

`SCOUT_WS_ROOT` 是首选工作区根；旧环境变量 `CATKIN_WS`、`WS_ROOT` 仅用于兼容。不要把个人目录或旧顶层 `src/` 写进代码、配置或脚本。

## 模块入口

- 2D 室内导航：已外置到 [scout-navigation-2d](https://github.com/mofajilu1314/scout-navigation-2d)，
  不再从本机工作区启动或构建。
- 3D TD3 与长途导航：[`3D/README.md`](3D/README.md)。先执行离线规划检查：
  `bash "$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav/scripts/smoke_long_nav_planning.sh"`；再按需执行
  `bash "$SCOUT_WS_ROOT/3D/scripts/humble_train_smoke.sh" --no-build`。
- 训练默认始终新建输出目录；只有显式 `resume:=true` 才能续训，详细规则见 [`3D/README1.md`](3D/README1.md)。

运行期间的日志暂存于模块的 `debug_logs/`；验证结束后必须连同命令和结果移入 `archive/validation/YYYYMMDD/`。未知 Gazebo 或 ROS 会话只报告、不强制结束。

## 独立导出与 GitHub 准备

工作区不保留长期复制包。需要带走模块时，从根目录按需导出：

```bash
bash "$SCOUT_WS_ROOT/tools/export_navigation_module.sh" 3d /目标目录 --with-baseline
```

3D 导出只包含共享层、`scout_offroad_3d`、`scout_long_nav` 和经过清理的 3D 资源；默认不含 2D、归档、私有实车资料和历史训练。具体边界见 [`3D/DELIVERY_MANIFEST.md`](3D/DELIVERY_MANIFEST.md)。

## 目录边界

```text
3D/                         3D 文档、脚本、地图、当前验证产物与日志
ros2_ws/src/shared/          共享车辆模型和路径工具
ros2_ws/src/navigation_3d/   scout_offroad_3d、scout_long_nav
archive/                     历史文档、ROS1、旧实验与资产备份（非运行入口）
article/                     独立研究资料（非运行依赖）
```

2D 的源码、说明和运行入口由独立仓库维护；不要把它的源码、构建目录或日志重新混入本机 3D 工作区。

当前活动目录、代码索引和可执行入口以 [`MODULE_MAP.md`](MODULE_MAP.md) 与 [`3D/README.md`](3D/README.md) 为准；历史审计材料仅保存在归档区，不作为日常操作入口。
