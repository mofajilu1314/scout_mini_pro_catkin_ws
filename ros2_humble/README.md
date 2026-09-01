# ROS2 Humble 工作区占位（Phase 1）

本目录是 **ROS2 迁移的文档锚点**，不是 colcon 工作区本身。

## 为什么单独建这个目录？

- 现有 `src/` 下 5 个包均为 **ROS1 catkin**（`package.xml` format 2）。
- 若在 `src/` 内提前放入 ROS2（format 3）包，可能干扰 `catkin_make` 或造成混 build。
- Phase 1 **只**做规划与空工作区准备，**不**移植 `ros_env.py` 等业务代码。

## 完整路线图

请阅读：**[`3D/ROS2_HUMBLE_MIGRATION.md`](../3D/ROS2_HUMBLE_MIGRATION.md)**

## 推荐的 Git 分支

| 分支 | 用途 |
|------|------|
| `main` | ROS1 基线 |
| **`ros2-humble`** | 全部 Humble 迁移（建议新建，勿在未 stash 时强切） |

## 下一步：创建 colcon 工作区（在 22.04 上手动执行）

```bash
# 与 catkin 同仓、独立 build（推荐）
mkdir -p /home/test/catkin_ws_2/ros2_ws/src
cd /home/test/catkin_ws_2/ros2_ws

# 确保已 source Humble
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

**Phase 2** 再将以下包**复制**到 `ros2_ws/src/`（非 symlink）：

- `scout_description`
- `scout_mini_pro_description`（仅仿真文件）
- `scout_gazebo_sim`
- `scout_offroad_3d`

## 目录规划（Phase 2 完成后预期）

```text
catkin_ws_2/
├── src/                 # ROS1 — 保持不变
├── ros2_ws/             # ROS2 colcon — Phase 1 手动 mkdir
│   ├── src/
│   ├── build/
│   └── install/
├── ros2_humble/         # 本 README（文档锚点）
└── 3D/
    └── ROS2_HUMBLE_MIGRATION.md
```

## .gitignore 建议（Phase 1 后可加入仓库根 .gitignore）

```
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
```

---

*Phase 1 脚手架 · 不包含任何 ROS2 包源码*
