# 松灵官方 Scout Mini 仿真模型接入与自动驾驶流程

本文档记录将松灵官方开源 Scout Mini Gazebo 仿真包引入本工作空间的过程、话题对照，以及**基于官方真车模型**的室内自动驾驶启动命令。

| 文档 | 车模型 | 用途 |
|---|---|---|
| [README_3.md](README_3.md) | 自制**方块小车**（`scout_mini_pro_description`） | 方块车建图/导航手册（**保持原样，勿混用**） |
| **本文 README_4** | **官方真车**（`scout_description` + `mini.xacro`） | 官方车打开、话题映射、仿 README_3 的导航流程 |

**重要**：本项目是 **Scout Mini**，不是 Limo。客服混发的 Limo / Isaac 链接取舍见 **第九节**；EDU 套件 PDF 在 `../article/EDU套件使用手册.pdf`。

---

## 一、来源与落盘

| 项 | 内容 |
|---|---|
| 仓库 | https://github.com/agilexrobotics/ugv_gazebo_sim.git |
| 分支 | `master`（ROS1 / Noetic） |
| Commit | `27633a956c845903ee630538afeb17fe70afdd84`（2026-06-03，信息：修复 ranger_air urdf 模型） |
| 下载方式 | GitHub 直连与部分镜像较慢/失败；最终用 **ghfast.top** 镜像做 `--depth 1 --filter=blob:none` 稀疏克隆，只检出 `scout/` |
| 临时目录 | `/tmp/ugv_sparse`（仍保留溯源用 `.git`） |
| 工作空间落盘 | 无 `.git` 拷贝到 `catkin_ws_2/src/`：`scout_description`、`scout_gazebo_sim`、`scout_control` |

未拷贝 `scout/pro`（Scout Pro 另一套模型）；当前目标为 Scout Mini。

---

## 二、官方包结构

```
src/
├── scout_description/     # URDF/xacro、mesh、rviz、stock launch
├── scout_gazebo_sim/      # Gazebo world、spawn launch、控制 yaml、可选 C++ skid 节点
└── scout_control/         # 控制相关配置（与 spawn 中 yaml 配合）
```

### 2.1 模型构造（Scout Mini）

- **入口 xacro**：`scout_description/launch/scout_mini_stock.launch` 加载 **`urdf/mini.xacro`**（不是 `scout_mini.xacro`）。
- **车身**：`base_link` + `box_link`（官方 mesh），四轮通过 `scout_mini_wheel_*.xacro` 挂到 `base_link`。
- **传感器 link（官方已定义）**：`mini.xacro` 内含 **`laser_link`**、**`camera_link`** 及固定关节。
- **Gazebo 扩展**：`urdf/scout_mini.gazebo`（由 `mini.xacro` / `scout_mini.xacro` include）：
  - 四轮 `transmission` + `libgazebo_ros_control.so`
  - `libgazebo_ros_skid_steer_drive.so`（主运动控制）
  - `libgazebo_ros_laser.so`（2D 激光）
  - `libgazebo_ros_openni_kinect.so`（深度相机，兼 RGB）

### 2.2 控制方式

官方包里写了两套控制，**本仓库官方导航默认只启用 skid_steer**：

1. **实际驱动**：Gazebo 插件 `libgazebo_ros_skid_steer_drive.so`，订阅 **`/cmd_vel`**，发布 **`/odom`**。
2. **ros_control**（可选）：`spawn_scout_mini.launch` 传 `enable_ros_control:=true` 并 `spawner` 控制器。  
   **注意**：`gazebo_ros_control` 与 skid_steer **不要同时空转**——未 spawn 的控制器会把轮速钳在 0，表现为 `/cmd_vel` 无效。`gazebo_room_official` 默认 `enable_ros_control:=false`。

轮子 TF：skid_steer 不发 `/joint_states`，`gazebo_room_official` 已加 `joint_state_publisher`（零位），避免 RViz `*_wheel_link` No transform。

激光：`laser_link` 在 `mini.xacro` 中置于约 **z=0.37**（车顶雷达圆柱顶），`min_range=0.12`，避免 `/scan` 大面积打在白箱上。

### 2.3 哪个是「官方完备小车」？

| 候选 | 入口 | 推荐 |
|---|---|---|
| **Scout Mini 库存（推荐）** | `mini.xacro` + `scout_mini_playpen.launch` / `gazebo_room_official.launch` | **优先**：官方 mesh + 2D 激光 + 深度/彩色相机 + `/cmd_vel` |
| Scout V2 | `scout_v2.xacro` + `scout_v2_playpen.launch` | 另一车型，本项目不用 |
| 方块车 | `scout_mini_pro.xacro` + `gazebo_room.launch` | README_3 专用，对比用 |

---

## 三、编译与依赖

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

可选：`sudo apt install ros-noetic-amcl ros-noetic-map-server`（导航）；`ros-noetic-velocity-controllers` 仅官方 `spawn_scout_mini` 需要。

---

## 四、立即可用：在 Gazebo 打开官方真车

每个新终端先执行：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 4.1 推荐：空世界官方演示（最快看清真车外观）

```bash
roslaunch scout_gazebo_sim scout_mini_playpen.launch
```

预期：Gazebo + 官方 Scout Mini mesh。键盘遥控（另开终端，同样先 source）：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# 官方车订阅根话题 /cmd_vel（teleop 默认即为此话题）
```

### 4.2 推荐：同一 room 场景 + 官方车（与 README_3 场景对齐）

```bash
roslaunch scout_mini_pro_description gazebo_room_official.launch
```

预期：room_1 + cafe_table，约 8 s 后 spawn **官方** `scout_mini`。

自检：

```bash
rostopic list | grep -E 'cmd_vel|odom|scan|camera'
# 应有 /cmd_vel /odom /scan /camera/color/... /camera/depth/...
rostopic echo /scan --noarr -n 1
rosrun tf tf_echo odom base_link
```

### 4.3 备选：官方 empty world + spawn

```bash
roslaunch gazebo_ros empty_world.launch \
  world_name:=$(rospack find scout_gazebo_sim)/worlds/weston_robot_empty.world \
  use_sim_time:=true
# 另开终端：
roslaunch scout_gazebo_sim spawn_scout_mini.launch
```

说明：`spawn_scout_mini` 可能因缺 `velocity_controllers` 报错，但 skid_steer 仍可用。日常更推荐 **4.1 / 4.2**。

---

## 五、集成结论清单（传感器 / 话题）

| 能力 | 官方是否自带 | 话题 / 消息类型 | 说明 |
|---|---|---|---|
| 运动控制 | **有** | `/cmd_vel`（`geometry_msgs/Twist`） | skid_steer |
| 里程计 | **有** | `/odom`（`nav_msgs/Odometry`） | `child_frame_id=base_link` |
| TF `odom`→`base_link` | **有（桥接）** | TF | 插件 `broadcastTF=false`；由 `odom_to_tf_official` 发 TF（避免 skid_steer 旧 tf API 的 `TF_REPEATED_DATA`） |
| 2D 激光 | **有** | `/scan`（`sensor_msgs/LaserScan`） | `frame_id=laser_link`；约 ±120° |
| 深度相机 | **有** | `/camera/depth/image_raw`、`/camera/depth/points`、`/camera/depth/camera_info` | Kinect 插件 |
| 彩色相机 | **有** | `/camera/color/image_raw`、`/camera/color/camera_info` | 同一插件 |
| IMU | **无** | — | README_3 导航不用 IMU |
| M360 / 多线雷达 | **无（仿真）** | — | 官方 Gazebo **只有 2D `/scan`**；方块车 Helios 仿真也是 2D `/scan`。真机 EDU 为 RS-Helios-16P |
| Ackermann | **无** | — | 官方为 skid_steer + Twist，与方块车差速一致 |

---

## 六、与方块车话题对照（能否直接映射）

| 用途 | 方块车（README_3） | 官方真车（本文） | 是否可直接映射 | 本仓库做法 |
|---|---|---|---|---|
| 速度指令 | `/scout_mini_pro/cmd_vel` | `/cmd_vel` | 名不同 | `nav_bringup_official` 设 `cmd_vel_topic:=/cmd_vel` |
| 里程计 | `/scout_mini_pro/odom` | `/odom` | 名不同 | AMCL `odom:=/odom` |
| 激光 | `/scan` | `/scan` | **可直接用** | 同话题 |
| RGB | `/scout_mini_pro/image_raw` 等 | `/camera/color/image_raw` | 名不同 | 仅可视化需改 RViz；导航不依赖 |
| 深度/点云 | `/scout_mini_pro/depth/points` | `/camera/depth/points` | 名不同 | 同上 |
| IMU | 可选（多注释） | 无 | 导航不依赖 | 暂不移植 |
| M360 | 无独立话题 | 无 | N/A | 仿真两侧都用 2D `/scan` |
| 基座 frame | `base_footprint` | `base_link` | 不同 | `amcl_params_official.yaml`：`base_frame_id: base_link` |
| TF 桥 | static `odom`↔命名空间 | `/odom`→TF（`odom_to_tf_official`） | 官方无方块车 static bridge；**勿**再开插件 `broadcastTF` | `gazebo_room_official` 启 `odom_to_tf_official` |

**结论**：控制 + 激光 + 里程计 **可映射**，不需要桥接节点；主要改 launch remap / AMCL `base_frame_id` / PP 的 `cmd_vel_topic`。**不要**在同一 Gazebo 里同时 spawn 两套车。

---

## 七、已落地改造（README_4 专用，不改 README_3 流程）

| 文件 | 作用 |
|---|---|
| `scout_description/urdf/mini.xacro` | `laser_joint` 抬到 z=0.37；修正 laser/camera collision 偏移；清理 `box_link.STL` 异常顶点 |
| `scout_description/urdf/scout_mini.gazebo` | 默认 `enable_ros_control:=false`；激光 `min_range=0.12`；`broadcastTF=false` |
| `scout_description/launch/scout_mini_stock.launch` | 透传 `enable_ros_control` |
| `scripts/odom_to_tf_official.py` | 唯一 `odom`→`base_link` TF 源（跳过重复时间戳） |
| `launch/gazebo_room_official.launch` | room + 官方 `mini.xacro` + `joint_state_publisher` + `odom_to_tf_official` |
| `launch/nav_bringup_official.launch` | 同 A*/PP，发 `/cmd_vel` |
| `launch/nav_all_in_one_official.launch` | 一键官方车导航 |
| `config/nav/amcl_params_official.yaml` | `base_frame_id: base_link` |
| `scripts/pure_pursuit_nav_node.py` | `~cmd_vel_topic`（**默认仍为** `/scout_mini_pro/cmd_vel`，README_3 不受影响） |

**未改**：`README_3.md` 正文、`gazebo_room.launch`、`nav_all_in_one.launch`、`nav_bringup.launch` 的方块车默认行为。

---

## 八、官方车自动驾驶流程（仿 README_3）

**前提**：已有地图 `scout_mini_pro_description/maps/room.yaml` + `room.pgm`（见 README_2 / README_3）。算法相同：**map_server + AMCL + A* + Pure Pursuit**。

### 8.1 一键启动（推荐）

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scout_mini_pro_description nav_all_in_one_official.launch
```

拉起：Gazebo room + 官方车 → map_server + A* + PP（`/cmd_vel`）→ AMCL（`/odom`，`base_link`）→ RViz。

RViz：先 **SetInitialPose**，再 **SetGoal**（与 README_3 相同）。

### 8.2 分步启动（调试用）

| 顺序 | 命令 |
|---|---|
| 1 | `roslaunch scout_mini_pro_description gazebo_room_official.launch` |
| 2 | `rosparam set use_sim_time true` |
| 3 | `roslaunch scout_mini_pro_description nav_bringup_official.launch`（须先于 amcl） |
| 4 | 见下方 amcl |
| 5 | `rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room_slam.rviz` |

```bash
rosparam load $(rospack find scout_mini_pro_description)/config/nav/amcl_params_official.yaml /amcl
rosrun amcl amcl odom:=/odom scan:=/scan
```

### 8.3 导航话题（官方车）

| 用途 | 话题 |
|---|---|
| 激光 | `/scan` |
| 里程计 | `/odom` |
| 控制 | `/cmd_vel` |
| 地图 / 初始位姿 / 目标 / 路径 | `/map`、`/initialpose`、`/move_base_simple/goal`、`/path` |
| TF | `map` → `odom` → `base_link` |

### 8.4 自检

```bash
rostopic list | grep -E 'clock|odom|cmd_vel|scan|joint_states'
# 期望：/clock /odom /cmd_vel /scan /joint_states（不要期望 /scout_mini_pro/*）
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link front_left_wheel_link   # 轮子 TF 应存在
rosrun tf tf_echo base_link laser_link              # z 约 0.37
# SetGoal 之后：
rostopic hz /path
rostopic echo /cmd_vel -n 3
```

RViz：RobotModel 不应再报四个轮子 No transform；车顶不应再出现浮空圆柱；`/scan` 红点不应大面积贴在白箱上。

若曾见大量 `TF_REPEATED_DATA ... frame base_link (parent odom) ... unknown_publisher`：那是 skid_steer 插件 `broadcastTF=true` 用旧 tf API 重复打时间戳；当前已改为插件只发 `/odom`，由 `odom_to_tf_official` 独家发 TF。验证：`rostopic info /tf` 中应有 `/odom_to_tf_official`，且终端不再刷该 WARN。

键盘遥控：`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`（默认 `/cmd_vel`）。车应能移动；若完全不动，确认未误开 `enable_ros_control:=true` 却未 spawn 控制器。

卡住复位：Gazebo **Model → Reset Pose**；模型名是 **`scout_mini`**（不是 `scout_mini_pro`）。

### 8.5 与 README_3 命令对照（防混用）

| 步骤 | 方块车（README_3） | 官方车（本文） |
|---|---|---|
| 仅开仿真 | `gazebo_room.launch` | `gazebo_room_official.launch` |
| 一键导航 | `nav_all_in_one.launch` | `nav_all_in_one_official.launch` |
| bringup | `nav_bringup.launch` | `nav_bringup_official.launch` |
| AMCL | `amcl_params.yaml` + `odom:=/scout_mini_pro/odom` | `amcl_params_official.yaml` + `odom:=/odom` |
| teleop | `cmd_vel:=/scout_mini_pro/cmd_vel` | 默认 `/cmd_vel` |

---

## 九、客服链接核实与取舍（勿混 Limo / Scout）

| 链接 | 结论 |
|---|---|
| limo_pro_doc / Limo-Isaac-Sim | **忽略**（Limo，非本项目） |
| EDU 套件手册 PDF | **保留**作真机 Helios/D435/CH110 参考；几乎不讲 Gazebo |
| scout_ros | 真机 CAN 再用；**不要覆盖**本机仿真 `scout_description` |
| 本机 `scout_*`（ugv_gazebo_sim） | **仿真主线，保留** |

---

## 十、相关文档

- EDU PDF：[`../article/EDU套件使用手册.pdf`](../article/EDU套件使用手册.pdf)
- 调试记录：[debug_records.md](debug_records.md)
- 方块车：[README_1.md](README_1.md)、[README_2.md](README_2.md)、[README_3.md](README_3.md)（**勿与本文 official launch 混用同一仿真实例**）
- 第二阶段 3D：[../3D/README.md](../3D/README.md)
