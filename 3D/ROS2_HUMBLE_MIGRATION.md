# ROS2 Humble 迁移路线图（scout_offroad_3d 阶段 A）

> **约束（已定稿）**：服务器**禁止 Docker**；必须在 **Ubuntu 22.04 + ROS2 Humble 原生**环境完成训练；ROS1 catkin 工作区保持可运行直至 Phase 8 验收通过。
>
> **现状审计（2026-09）**：5 个 ROS1 catkin 包；核心 `scout_offroad_3d`；`ros_env.py` ≈2424 行；Gazebo Classic 强绑定；**0 个 ROS2 包**。

---

## 目录

1. [总体策略](#1-总体策略)
2. [Git 分支与工作区布局（Phase 1）](#2-git-分支与工作区布局phase-1)
3. [Phase 0–10 分阶段计划](#3-phase-010-分阶段计划)
4. [Phase 7 专题：Gazebo Classic on Humble（无 Docker）](#4-phase-7-专题gazebo-classic-on-humble无-docker)
5. [Phase 8：分阶段冒烟测试清单](#5-phase-8分阶段冒烟测试清单)
6. [Phase 9：服务器原生部署](#6-phase-9服务器原生部署)
7. [Phase 10：真车（scout_ros2）— 与训练分离](#7-phase-10真车scout_ros2--与训练分离)
8. [ROS1 → ROS2 包对照表](#8-ros1--ros2-包对照表)

---

## 1. 总体策略

| 原则 | 说明 |
|------|------|
| **并行树，不混 build** | ROS1 继续 `catkin_ws_2/src/` + `catkin_make`；ROS2 使用独立 `ros2_ws/`，`colcon build`。绝不在同一 `src/` 下混放 format 2/3 的 `package.xml`。 |
| **最小可训练路径** | 阶段 A 只需：描述包（仿真）→ 扇区链 → `ros_env` → `train_td3` → Gazebo。**不做** nav/AMCL/A\*。 |
| **Gazebo Classic 优先** | 现有 `.world`（heightmap）、`libgazebo_ros_block_laser.so`、`switch_world()` 均面向 Classic 11；**不**优先迁 Ignition/Gazebo Sim（Fortress/Harmonic），除非 Classic 阻塞。 |
| **增量验收** | 每 Phase 有独立冒烟测试；通过后合并到 `ros2-humble` 分支。 |

```text
catkin_ws_2/                    # ROS1 基线（保持）
├── src/                        # 现有 5 包，勿改 package format
├── 3D/                         # 文档、训练产物
└── ros2_humble/                # Phase 1 占位 + 指向 ros2_ws

~/ros2_ws/  或  catkin_ws_2/ros2_ws/   # ROS2 colcon（Phase 1 手动创建）
└── src/
    ├── scout_description/          # Phase 2
    ├── scout_mini_pro_description/ # Phase 2（仅 sim 子集）
    ├── scout_gazebo_sim/           # Phase 2（skid_steer 插件）
    └── scout_offroad_3d/           # Phase 3–6
```

---

## 2. Git 分支与工作区布局（Phase 1）

### 2.1 推荐 Git 分支名

| 分支 | 用途 |
|------|------|
| **`main`** | ROS1 Noetic/Melodic 可跑基线；不在此分支直接改 ROS2 包。 |
| **`ros2-humble`** | 全部 Humble 迁移提交；与 `main` 长期并存直至 cutover。 |

**操作（在 22.04 开发机上手动执行，当前仓库有未提交变更，勿在未备份时强切）：**

```bash
cd /home/test/catkin_ws_2
git stash push -u -m "WIP before ros2-humble"   # 可选
git checkout -b ros2-humble
# 或：git checkout main && git pull && git checkout -b ros2-humble
```

> **Phase 1 交付物**：本文件 + `ros2_humble/README.md`。**不在 `src/` 内创建 ROS2 包**，避免 catkin 误索引或 `package.xml` format 冲突。

### 2.2 ROS2 工作区创建（Phase 1 手动第一步）

在 **Ubuntu 22.04** 上：

```bash
# 方案 A（推荐）：与 catkin 同仓、独立 build 目录
mkdir -p /home/test/catkin_ws_2/ros2_ws/src
cd /home/test/catkin_ws_2/ros2_ws
rosdep update   # 首次

# 方案 B：用户 home 下独立 ws
# mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
```

**符号链接 vs 复制（Phase 2 起）：**

| 方式 | 优点 | 缺点 | 建议 |
|------|------|------|------|
| **复制** `scout_description` 等到 `ros2_ws/src/` | ROS2 改 URDF/launch 不影响 ROS1 | 需同步 bugfix | **Phase 2 首选** |
| **git subtree / submodule** | 版本可追溯 |  setup 复杂 | Phase 4 后考虑 |
| **symlink 整个包** | 快 | 改 `package.xml` 会破坏 ROS1 | **禁止** |

### 2.3 Phase 1 验收标准

- [ ] 22.04 上 `ros2 --version` 为 Humble
- [ ] `ros2_ws/src/` 目录存在且为空（或仅有 `.gitkeep`）
- [ ] `colcon build` 空工作区无报错
- [ ] ROS1 `catkin_ws_2` 仍可 `catkin_make`（未动 `src/`）

---

## 3. Phase 0–10 分阶段计划

### Phase 0：开发机环境（Ubuntu 22.04 + Humble desktop）

| 项 | 内容 |
|----|------|
| **目标** | 原生 Humble 开发/训练环境就绪；**无 Docker**。 |
| **依赖** | 无 |
| **预估** | **0.5–1 天** |
| **涉及文件/操作** | 系统级 apt；`~/.bashrc`；本仓 `3D/ROS2_HUMBLE_MIGRATION.md` |

**步骤：**

```bash
# 1. ROS2 Humble（desktop 或 desktop-full）
sudo apt update
sudo apt install -y ros-humble-desktop-full \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  build-essential git

# 2. 环境
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null; rosdep update

# 3. Gazebo Classic 11 + ROS2 桥（详见 Phase 7）
sudo apt install -y gazebo libgazebo-dev \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros \
  ros-humble-gazebo-msgs ros-humble-gazebo-plugins \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui

# 4. Python 训练依赖（与 ROS1 相同栈）
pip3 install torch numpy --user   # 版本按现有 train 环境对齐
```

**验收标准：**

- [ ] `ros2 doctor --report` 无致命错误
- [ ] `gazebo --version` 显示 11.x（Jammy 仓库版 ≈11.10.2）
- [ ] `ros2 pkg list | grep gazebo_ros` 有 `gazebo_ros`、`gazebo_msgs`
- [ ] **未**安装与 focal 混用的 `~focal` 后缀 Gazebo 包（见 Phase 7 风险）

---

### Phase 1：ROS2 工作区脚手架

| 项 | 内容 |
|----|------|
| **目标** | 目录与文档就位；**不**移植业务代码。 |
| **依赖** | Phase 0 |
| **预估** | **0.5 天** |
| **涉及文件** | `3D/ROS2_HUMBLE_MIGRATION.md`（本文件）、`ros2_humble/README.md`；手动创建 `ros2_ws/` |

**验收标准：** 见 [§2.3](#23-phase-1-验收标准)。

---

### Phase 2：移植描述包（仅仿真，无 nav）

| 项 | 内容 |
|----|------|
| **目标** | Scout Mini URDF/xacro + Gazebo 插件在 Humble 下 spawn；**不含** AMCL/nav launch。 |
| **依赖** | Phase 0、1 |
| **预估** | **3–5 天** |
| **涉及文件** | |

**ROS1 源 → ROS2 目标：**

| ROS1 包 | 复制到 `ros2_ws/src/` | 改动要点 |
|---------|----------------------|----------|
| `scout_description` | 同名 | `package.xml` format 3；`CMakeLists.txt` → ament；`$(find pkg)` → `$(find-pkg-share pkg)`；Gazebo 插件 XML 见 Phase 7 |
| `scout_mini_pro_description` | 同名 | **只保留** `xacro/`、`meshes/`、`launch/gazebo_room_official` 仿真子集；**跳过** `nav_*`、`amcl`、`astar_nav_node` |
| `scout_gazebo_sim` | 同名 | `scout_skid_steer` 插件：ROS1 `gazebo_ros` API → ROS2 `gazebo_ros` 或 `gazebo_plugins`；`cmd_vel` → `geometry_msgs/msg/Twist` |

**关键 URDF 文件：**

- `src/scout_description/urdf/scout_mini.gazebo` — 16 线 `block_laser` → `libgazebo_ros_ray_sensor.so`（Phase 7）
- `src/scout_description/urdf/mini.xacro`
- `src/scout_mini_pro_description/launch/gazebo_room_official.launch` → `gazebo_room_official.launch.py`

**验收标准：**

- [ ] `ros2 launch scout_mini_pro_description gazebo_room_official.launch.py` 启动 gzserver + spawn Scout
- [ ] `ros2 topic echo /lidar/points --once` 有点云（PointCloud 或 PointCloud2）
- [ ] `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...` 车有运动（skid_steer）
- [ ] **未**启动 map_server / amcl

---

### Phase 3：移植 range_features 链（最小训练依赖）

| 项 | 内容 |
|----|------|
| **目标** | `/offroad/td3_range_features` 等话题在 ROS2 下与 ROS1 维度和语义一致。 |
| **依赖** | Phase 2 |
| **预估** | **2–3 天** |
| **涉及文件** | |

| 文件 | 改动 |
|------|------|
| `scout_offroad_3d/scripts/range_features_node.py` | `rospy` → `rclpy`；参数 `declare_parameter`；`Float32MultiArray` 发布 |
| `scout_offroad_3d/scripts/pointcloud_bridge_node.py` | 可选：若 Phase 7 直接发 PointCloud2 可简化或删除 |
| `scout_offroad_3d/launch/range_features.launch` | → `range_features.launch.py` |
| `scout_offroad_3d/package.xml` | ament_python；依赖 `sensor_msgs`, `std_msgs`, `rclpy` |
| `scout_offroad_3d/setup.py` | entry_points 注册节点 |

**验收标准：**

- [ ] Gazebo + lidar 运行时，`ros2 topic hz /offroad/td3_range_features` ≈10 Hz
- [ ] 输出维度 = `n_elevation_bins_td3 × n_sectors`（默认 72）
- [ ] `source:=cloud` 与 `source:=scan` 至少 cloud 路径通过
- [ ] 与 ROS1 同 world 下扇区值粗测一致（同一 seed/位姿）

---

### Phase 4：移植 `ros_env.py` 后端（最难）

| 项 | 内容 |
|----|------|
| **目标** | `ScoutGazeboEnv` → `Ros2GazeboBackend`（或同名类）；reset/step/switch_world 行为对齐。 |
| **依赖** | Phase 2、3 |
| **预估** | **7–12 天** |
| **涉及文件** | |

| 模块 | 行数/复杂度 | 要点 |
|------|------------|------|
| `scripts/td3/ros_env.py` | ≈2424 行 | **拆分**：`gazebo_lifecycle.py`（启停 gzserver）、`spawn_gate.py`、`reward.py`、`Ros2GazeboBackend` |
| 服务调用 | 高 | `/gazebo/get_model_state` 等 → `gazebo_msgs/srv/*` async 客户端 |
| `switch_world()` | **最高风险** | 见 [§4.3](#43-switch_world-移植风险) |
| 参数 | 中 | `rospy.get_param` → `node.declare_parameter` + yaml |

**建议拆分结构：**

```text
scout_offroad_3d/scout_offroad_3d/
├── gazebo_backend.py      # Ros2GazeboBackend：reset, step, observe
├── gazebo_lifecycle.py    # _kill_gazebo, _start_gzserver, _wait_unregistered
├── spawn_utils.py         # spawn gate, world_spawn_z
├── reward.py              # REWARD_PART_KEYS, step 奖励逻辑
└── td3/                   # TD3.py, replay_buffer.py（纯 Python，改动小）
```

**验收标准：**

- [ ] 单测式脚本：`ros2 run scout_offroad_3d env_smoke_test` 完成 reset → 10 step → done 不崩溃
- [ ] spawn gate 在 `bumpy_offroad.world` heightmap 上可通过
- [ ] `switch_world()` 连续切换 3 个 world 不导致 `/gazebo` 名冲突
- [ ] `/cmd_vel` 发布、`/offroad/td3_range_features` 订阅正常
- [ ] rollover / collision / stuck 与 ROS1 同参下行为一致（允许 ±1 step 浮点差）

---

### Phase 5：移植 `train_td3_node` + launch + yaml

| 项 | 内容 |
|----|------|
| **目标** | `ros2 launch scout_offroad_3d train_td3.launch.py` 可开训并写 metrics。 |
| **依赖** | Phase 4 |
| **预估** | **3–4 天** |
| **涉及文件** | |

| 文件 | 改动 |
|------|------|
| `scripts/train_td3_node.py` | `rclpy`；`world_switch_interval`；metrics 路径逻辑保留 |
| `config/td3_train.yaml` | ROS2 参数文件格式（`/**:` ros__parameters） |
| `launch/train_td3.launch` | → `train_td3.launch.py`；`pick_random_world.py` → ament 脚本 |
| `scripts/pick_random_world.py` | `ament_index` 查 share 路径 |
| `scripts/plot_td3_metrics.py` | 可选 Phase 5 末或 Phase 8 |

**验收标准：**

- [ ] `gui:=false` 下跑 1000 step 生成 `3D/runs/run_*/metrics.csv`
- [ ] `world_switch_interval: 500` 时 metrics 有 world 切换记录
- [ ] `resume:=true` 可加载权重续训
- [ ] Ctrl+C 干净退出，无僵尸 gzserver

---

### Phase 6：移植 `infer_td3_node`

| 项 | 内容 |
|----|------|
| **目标** | 加载 actor 权重在仿真中推理；支持 `pose_source:=odom` 占位（真车 Phase 10）。 |
| **依赖** | Phase 5 |
| **预估** | **2 天** |
| **涉及文件** | `scripts/infer_td3_node.py`、`launch/infer_td3.launch.py` |

**验收标准：**

- [ ] `ros2 launch scout_offroad_3d infer_td3.launch.py checkpoint:=...` 车能跟策略走
- [ ] `start_sim:=false` 仅连已有 topics 不启 Gazebo（接口预留）
- [ ] heavy heightmap 下 wall-clock loop 不卡死（与 ROS1 注释一致）

---

### Phase 7：Gazebo Classic on Humble 策略（研究结论）

| 项 | 内容 |
|----|------|
| **目标** | 文档化 apt 包、插件映射、world 兼容性；**不迁 Gazebo Sim**。 |
| **依赖** | Phase 0 |
| **预估** | **1–2 天**（与 Phase 2 并行） |
| **详见** | [§4 专题](#4-phase-7-专题gazebo-classic-on-humble无-docker) |

---

### Phase 8：分阶段冒烟测试

| 项 | 内容 |
|----|------|
| **目标** | 可重复的 CI/手工检查表。 |
| **依赖** | 各 Phase |
| **预估** | **持续** |
| **详见** | [§5](#5-phase-8分阶段冒烟测试清单) |

---

### Phase 9：服务器原生部署（无 Docker）

| 项 | 内容 |
|----|------|
| **目标** | 训练服务器 Ubuntu 22.04 上 `colcon build` + systemd/screen 长跑 TD3。 |
| **依赖** | Phase 5–8 |
| **预估** | **1–2 天** |
| **详见** | [§6](#6-phase-9服务器原生部署) |

---

### Phase 10：真车（scout_ros2 Humble）— 与训练分离

| 项 | 内容 |
|----|------|
| **目标** | 实车 AgileX Scout ROS2 驱动；**不**阻塞仿真训练迁移。 |
| **依赖** | 无（与 Phase 5+ 并行规划即可） |
| **预估** | **5+ 天**（独立项目） |
| **详见** | [§7](#7-phase-10真车scout_ros2--与训练分离) |

---

## 4. Phase 7 专题：Gazebo Classic on Humble（无 Docker）

### 4.1 推荐方案：**Gazebo Classic 11 + ros-humble-gazebo-ros-pkgs**

| 组件 | Ubuntu 22.04 / Humble 包名 | 说明 |
|------|---------------------------|------|
| 仿真器本体 | `gazebo`, `libgazebo-dev` | Jammy 官方 **11.10.2+dfsg**；命令仍为 `gazebo` / `gzserver` |
| ROS2 元包 | `ros-humble-gazebo-ros-pkgs` | 拉取 ros 侧插件与 launch |
| 核心 | `ros-humble-gazebo-ros`, `ros-humble-gazebo-msgs`, `ros-humble-gazebo-plugins` | spawn、set/get model state、block_laser 替代 |
| 工具 | `ros-humble-gazebo-dev` | 编译自定义插件时需要 |
| 机器人描述 | `ros-humble-xacro`, `ros-humble-robot-state-publisher` | URDF 链 |

**不推荐作为本项目的首选：**

| 方案 | 原因 |
|------|------|
| **Gazebo Fortress / Harmonic + ros_gz** | Humble 官方对接的是 **Gazebo Sim**，与现有 Classic `.world`、SDF 1.6 heightmap、插件 `.so` 名不兼容；迁移量 ≈ 重写仿真层。 |
| **tue_rosc_gazebo** | 公开仓库中**无**面向 Humble 的成熟 fork；TU/e 系列多面向课程/旧版，**不建议**作为依赖。 |
| **Docker 镜像** | 用户约束禁止。 |

**从 20.04/Noetic 升级到 22.04 的常见坑：**

- 若系统残留 `libgazebo11` **~focal** 后缀包，会与 Jammy 的 `gazebo` **冲突**（`gz-tools2 Conflicts: gazebo`）。
- 处理：卸载所有 `~focal` 的 ignition/sdformat/gazebo 包后再 `apt install gazebo libgazebo-dev ros-humble-gazebo-ros-pkgs`。
- 参考：[gazebo-classic#3277](https://github.com/gazebosim/gazebo-classic/issues/3277)

**ARM64 注意：** `ros-humble-gazebo-ros-pkgs` 无官方 arm64 deb，需源码 colcon 构建 `gazebo_ros_pkgs`（仍不需要 Docker）。

### 4.2 插件与传感器映射

| ROS1（Noetic） | ROS2 Humble 替代 | 本项目位置 |
|----------------|------------------|------------|
| `libgazebo_ros_block_laser.so` | `libgazebo_ros_ray_sensor.so` + `<output_type>sensor_msgs/PointCloud</output_type>` | `scout_mini.gazebo` 16 线 lidar |
| `libgazebo_ros_skid_steer_drive.so` 或自研 skid_steer | `gazebo_ros_diff_drive` / 保留 `scout_gazebo_sim` 自研插件 port | `scout_gazebo_sim` |
| `gazebo_ros` spawn/delete/set_state | 同名 ROS2 包，服务名仍为 `/spawn_entity` 等（**需核对 Humble 文档**） | `ros_env.py` |
| `empty_world.launch` | `gazebo_ros/launch/gazebo.launch.py` | `view_world.launch.py` |

**block_laser → ray_sensor 示例（写入 Phase 2 URDF）：**

```xml
<plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
  <ros>
    <remapping>~/out:=/lidar/points</remapping>
  </ros>
  <output_type>sensor_msgs/PointCloud</output_type>
  <frame_name>laser_link</frame_name>
</plugin>
```

**heightmap worlds：**

- 现有 `worlds/bumpy_offroad.world`、`offroad_map.world` 使用 Classic `<heightmap>` + ODE，Jammy Gazebo11 **可加载**。
- 风险：大 heightmap 导致 **RTF ≪ 1**；与 ROS1 相同，需 `infer`/训练侧 wall-clock loop（已在 `infer_td3_node` / yaml 中考虑）。
- `world_spawn_z`  per-world 覆盖（如 bumpy ≈1.05）必须在 ROS2 backend **原样保留**。

### 4.3 `switch_world()` 移植风险（诚实评估）

| 风险项 | 级别 | 说明 |
|--------|------|------|
| 热切换 world | **高** | Classic **无**官方 hot-swap 服务；ROS1 实现为 `kill gzserver → 新 gzserver → respawn URDF`，ROS2 同样必须保留。 |
| `/gazebo` 节点名冲突 | **高** | gzclient 与 gzserver 抢 `__node:=gazebo` 会导致 spawn 服务缺失；ROS2 launch 需显式 `gzserver`/`gzclient` 命名（与 ROS1 `_start_gzclient` 教训相同）。 |
| Master 残留 | **中** | ROS2 无 rosmaster，但 **节点同名**仍会通过 DDS/graph 冲突；需 `ros2 node list` 轮询代替 `_wait_gazebo_unregistered`。 |
| 端口 11345 | **中** | 仍监听 `ss -lnt`；快速重启易 `Address already in use`。 |
| 训练不中断 | **中** | 切换失败时须 skip episode 而非 crash（ROS1 已做）；ROS2 需同等 `try/except` + 世界池 retry。 |
| launch 路径硬编码 | **低** | ROS1 写死 `/opt/ros/noetic/lib/gazebo_ros/gzserver`；ROS2 改为 `which gzserver` 或 `get_package_share_directory('gazebo_ros')`。 |

**结论：** `switch_world()` **可移植**，但 Phase 4 应单独开 **3–5 天** 做 world 轮换 soak test；不要与 reward 逻辑同 PR。

### 4.4 备选：仅当 Classic 完全不可用时

若 apt 无法解决依赖（极老机器、混装 focal）：

1. 源码编译 Gazebo Classic 11 + `gazebo_ros_pkgs`（humble 分支）。
2. **仍不要**上 Fortress，除非接受重写全部 `.world` 与 heightmap 工作流。

---

## 5. Phase 8：分阶段冒烟测试清单

### Phase 0

- [ ] `ros2 run demo_nodes_cpp talker` / `listener` 互通
- [ ] `gazebo worlds/empty.world` 空 world 可开

### Phase 2

- [ ] `ros2 topic list | grep -E 'joint_states|robot_description'`
- [ ] TF: `laser_link` 存在
- [ ] 点云非空

### Phase 3

- [ ] `ros2 topic echo /offroad/range_features --once` 长度 24
- [ ] `ros2 topic echo /offroad/td3_range_features --once` 长度 72

### Phase 4

- [ ] `env_smoke_test`: 10 episodes × reset，无 unhandled exception
- [ ] `switch_world` ×3：`ros2 node list` 始终有 spawn 服务
- [ ] 日志无 `[world] /gazebo name conflict`

### Phase 5

- [ ] 1000 step 训练 <30 min（headless, rough_obstacles）
- [ ] `metrics.csv` 含 `r_collision`, `ep_success` 列
- [ ] `resume` 后 `t` 单调增

### Phase 6

- [ ] infer 5 min 无 NaN cmd_vel
- [ ] checkpoint 与 ROS1 格式互通（纯 torch state_dict）

### 回归（Phase 8 末）

- [ ] 全 world 池各跑 1 episode spawn 成功
- [ ] 与 ROS1 同 seed 下前 100 step 奖励曲线相关系数 >0.9（允许噪声）

---

## 6. Phase 9：服务器原生部署

**约束：** 无 Docker；Ubuntu 22.04；建议 headless (`gui:=false`)。

```bash
# 1. 系统依赖（同 Phase 0）
# 2. 克隆/同步 ros2-humble 分支
cd /home/test/catkin_ws_2/ros2_ws
git pull
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 3. 长跑示例（screen/tmux）
screen -S td3_train
source /opt/ros/humble/setup.bash
source /home/test/catkin_ws_2/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1   # 无 GPU 时可选
ros2 launch scout_offroad_3d train_td3.launch.py gui:=false \
  output_dir:=/home/test/catkin_ws_2/3D/runs/run_$(date +%Y%m%d_%H%M%S)
```

** systemd（可选）：** `After=network-online.target`；`Environment=ROS_DOMAIN_ID=0`；日志进 `3D/runs/`。

**验收：**

- [ ]  reboot 后手动 source 可复现
- [ ] 7×24h 无 gzserver 泄漏（`pgrep gzserver | wc -l` ≤1）
- [ ] 磁盘：runs 目录轮转（已有 `.gitignore`）

---

## 7. Phase 10：真车（scout_ros2）— 与训练分离

| 项 | 说明 |
|----|------|
| **仓库** | [AgilexRobotics/scout_ros2](https://github.com/agilexrobotics/scout_ros2)（Humble 分支） |
| **与训练关系** | 阶段 A **仅** `pose_source:=odom` + 已有 `/offroad/*` topics；**不**在训练迁移 PR 中带真车 launch |
| **依赖** | 真车驱动、velodyne/雷达驱动、TF tree |
| **infer 入口** | `infer_td3.launch.py start_sim:=false pose_source:=odom` |
| **预估** | 驱动+标定+安全 5–10 天，独立 milestone |

---

## 8. ROS1 → ROS2 包对照表

| ROS1 包 | 阶段 A 是否必需 | ROS2 策略 |
|---------|----------------|-----------|
| `scout_description` | ✅ | 复制 + ament + ray_sensor 插件 |
| `scout_mini_pro_description` | ✅（sim 子集） | 复制；删 nav |
| `scout_gazebo_sim` | ✅ | 插件 port 或 diff_drive 替代 |
| `scout_offroad_3d` | ✅ | ament_python 主包 |
| `scout_control` | ❌ 阶段 A | 阶段 B/C 或真车 |
| Gazebo Classic | ✅ | apt `gazebo` + `ros-humble-gazebo-ros-pkgs` |

---

## 附录 A：预估总工期

| Phase | 天数 |
|-------|------|
| 0 | 0.5–1 |
| 1 | 0.5 |
| 2 | 3–5 |
| 3 | 2–3 |
| 4 | 7–12 |
| 5 | 3–4 |
| 6 | 2 |
| 7 | 1–2（与 2 并行） |
| 8 | 持续 |
| 9 | 1–2 |
| **合计（训练闭环）** | **≈20–32 人天** |

---

## 附录 B：Phase 1 完成后「你应在 22.04 上手动做的第一件事」

1. 安装 Phase 0 apt（Humble + Gazebo Classic + colcon）。
2. `git checkout -b ros2-humble`（可选，见 §2.1）。
3. `mkdir -p /home/test/catkin_ws_2/ros2_ws/src`。
4. `cd ros2_ws && colcon build && source install/setup.bash`。
5. 阅读本文 Phase 7，确认 `gazebo --version` 与 `ros2 pkg prefix gazebo_ros` 正常。
6. **不要**复制 `src/` 到 ros2_ws，直到开始 Phase 2。

---

*文档版本：Phase 1 脚手架 · 2026-09-01*
