# 工作空间与包文件架构说明

> **用途**：想改某功能时，用本文快速锚定到具体文件。  
> **调参核心结论**：**线速度 / 角速度 / 倒车 / 回正 / 夹角倒车 / A\* 膨胀与安全域，默认值几乎都写在 `launch/nav_bringup.launch`**，不要先翻算法脚本找默认值。  
> **操作手册**：启动步骤、日志解读、故障排查见 [README_3.md](README_3.md)。本文只做「文件地图」，不重复长操作流程。

**文档位置**：`2D/FILE_STRUCTURE.md`（工作区根下的文档区，**不参与 catkin 编译**）  
**统计基准**：按当前磁盘真实文件编写（无 `meshes/` 于本包；官方 mesh 在 `scout_description`）。

---

## 一、工作空间顶层（`~/catkin_ws_2`）

```
catkin_ws_2/
├── README.md              # 工作区总索引（2D / 3D / article / src）
├── .catkin_workspace      # catkin 工作空间标记
├── .gitignore
├── article/               # 论文 / 文献 PDF / 调研笔记（保持独立，不参与编译）
├── 2D/                    # ★ 第一阶段文档（本目录：室内 2D 建图导航手册）
├── 3D/                    # ★ 第二阶段文档（越野 / 深度相机 / 强化学习方向）
├── build/                 # catkin_make 编译中间产物（可删，可重建）
├── devel/                 # 编译产物、setup.bash、可执行入口（运行前需 source）
└── src/                   # 全部 ROS 包源码（编译与打包只关心这里）
    ├── CMakeLists.txt     # 指向 /opt/ros/noetic/.../toplevel.cmake
    ├── scout_mini_pro_description/   # ★ 自制方块车 + 2D 导航主包
    ├── scout_description/            # 官方 Scout Mini URDF/mesh
    ├── scout_gazebo_sim/             # 官方 Gazebo spawn / world
    └── scout_control/                # 官方控制 yaml
```

| 路径 | 作用 | 要不要手改 |
|------|------|------------|
| `src/` | 源码与配置；`roslaunch` / `rosrun` 的包都在这里 | **改代码在这里** |
| `2D/` | 第一阶段 md 文档（本系列） | 改 2D 手册在这里 |
| `3D/` | 第二阶段 md 文档 | 改 3D/RL 方向文档在这里 |
| `article/` | 文献清单、PDF、申报相关；子目录含导航/建图等 PDF | 写论文/查文献；**不影响仿真运行** |
| `build/` | CMake 缓存、各包编译中间文件 | 一般不改；坏了可删后重编 |
| `devel/` | `setup.bash`、`lib/`、`share/` 等运行环境 | 不手改；每次新终端 `source devel/setup.bash` |
| `.catkin_workspace` | 标记本目录为 catkin 工作空间 | 勿删 |
| `.git` / `.gitignore` | 版本管理 | 按需 |

**为何这样分**：文档不放进 `src/<pkg>/`，避免以后打 ROS 包 / 备份 3D 实验时和运行时代码搅在一起；`article/` 继续单独放 PDF。

**编译后常用**：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

---

## 二、`src/` 下四个包的关系

| 包名 | 角色 | 与导航的关系 |
|------|------|----------------|
| **`scout_mini_pro_description`** | 自制简易 Scout Mini Pro（方块车身 + 传感器 xacro）+ **当前导航栈（A\* + 纯追踪）** | **日常开发 / 调参主包** |
| `scout_description` | 松灵官方 URDF、`.dae`/`.STL` mesh、展示用 launch | 真实外观车模；**当前一键导航未默认用它** |
| `scout_gazebo_sim` | 官方空世界 / playpen、spawn launch、可选 skid-steer C++ | 官方仿真入口；与自制导航独立 |
| `scout_control` | 官方 `scout_mini_control.yaml`、`twist_mux.yaml` 等 | 配合官方 spawn；自制差速在 xacro 插件里 |

官方三包来源与对接说明见 [README_4.md](README_4.md)。  
**若只做室内导航调参，几乎只需动 `scout_mini_pro_description`。**

调试流水账见同目录 [debug_records.md](debug_records.md)（「踩坑 → 原因 → 修法」）。  
第二阶段建议新建独立包 `src/scout_offroad_3d/`（尚未创建），文档见 [`../3D/`](../3D/README.md)。

---

## 三、主包 `scout_mini_pro_description` 一级目录

```
scout_mini_pro_description/
├── CMakeLists.txt / package.xml
├── README.md                  # 短指针 → 指向工作区 2D/、3D/、article/
├── launch/                    # 所有启动入口
├── scripts/                   # A*、纯追踪、辅助脚本
├── config/nav/                # AMCL 等 yaml（部分为旧 move_base 残留）
├── xacro/                     # 机器人模型（main / sensor / macro / actuator）
├── maps/                      # 静态导航地图 room.pgm + room.yaml
├── rviz/                      # RViz 预设布局
├── world/                     # Gazebo 房间世界与模型
└── rqt/                       # TF 树导出图（dot/svg/png）
```

> **说明**：原包根下的 `README_1`～`4`、`3D_README*`、`FILE_STRUCTURE.md` 已迁到工作区 `2D/` 与 `3D/`。

**本包没有 `meshes/`**：方块车用几何体（box/cylinder）描述；精细 mesh 在官方包 `scout_description/meshes/`。

| 目录 | 干什么 | 典型何时打开 |
|------|--------|----------------|
| `launch/` | 一键/分步启动仿真、建图、导航 | **改默认调参 → `nav_bringup.launch`** |
| `scripts/` | 规划与控制节点实现、调试工具 | 改算法逻辑时；**调默认参数优先改 launch** |
| `config/nav/` | AMCL 参数；旧 costmap/planner yaml | 改定位 → `amcl_params.yaml` |
| `xacro/` | 车身、轮、传感器、Gazebo 插件 | 改摩擦、外观尺寸、传感器安装 |
| `maps/` | 导航用静态栅格图 | 换图 / 重新建图保存 |
| `rviz/` | 可视化布局（含 SetGoal / Map） | 导航 RViz 异常时核对配置 |
| `world/` | room 场景 `.world` 与墙体模型 | 改房间布局 |
| `rqt/` | 已导出的 TF 示意图 | 查坐标链时参考 |

---

## 四、关键文件逐个说明

### 4.1 `launch/`（启动入口）

| 文件 | 作用 | 备注 |
|------|------|------|
| **`nav_bringup.launch`** | **导航核心**：`map_server` + `astar_nav_node` + `pure_pursuit_nav_node` | **★ 调速度/倒车/膨胀主要改这里**；可被 `nav_all_in_one` include，也可单独开 |
| **`nav_all_in_one.launch`** | **一键导航**：include `gazebo_room` + `nav_bringup` + 延迟启动 amcl + RViz | 日常演示首选；**参数仍来自 nav_bringup** |
| `gazebo_room.launch` | Gazebo 加载 `world/room_1.world` + 生成自制车 + TF 桥 | 分步导航时先起它 |
| `gazebo_room_rviz.launch` | room 仿真 + RViz 传感器可视化（无导航） | 看激光/相机 |
| `gazebo_room_slam.launch` | room + slam_toolbox + SLAM 用 RViz | **建图用**；导航时勿与静态 map 同时用 |
| `gazebo_sim.launch` | 空世界 + 生成车 | 无房间 |
| `gazebo_rviz.launch` | 空世界仿真 + RViz | 键盘遥控对照 |
| `display_rviz.launch` | 仅模型 + joint_state_publisher GUI + RViz | 不启 Gazebo，看 URDF/TF |

**启动方式对照**：

| 需求 | 命令 |
|------|------|
| 一键导航 | `roslaunch scout_mini_pro_description nav_all_in_one.launch` |
| 分步：只仿真 | `roslaunch scout_mini_pro_description gazebo_room.launch` |
| 分步：导航栈（须已有 Gazebo，且 **先于 amcl**） | `roslaunch scout_mini_pro_description nav_bringup.launch` |
| 建图 | `roslaunch scout_mini_pro_description gazebo_room_slam.launch` |

#### `nav_bringup.launch` 里两类参数（当前默认，以文件为准）

**A\*（`astar_nav_node`）**

| 参数 | 当前默认 | 含义 |
|------|----------|------|
| `expansion_size` | `4.5` | 硬膨胀（格）；过大门口可能 `no path` |
| `safety_zone_radius` | `10` | 安全域半径（格，软代价） |
| `safety_zone_weight` | `1.0` | 安全域权重；`0`≈关软代价 |

**纯追踪（`pure_pursuit_nav_node`）——速度 / 倒车 / 回正等**

| 类别 | 主要参数（见 launch 内完整列表） |
|------|----------------------------------|
| 线速度 | `max_linear_speed`、`min_linear_speed` |
| 角速度 | `omega_max`、`omega_v_floor`、`omega_gain`、`lookahead_distance` |
| 曲率降速 | `curvature_enabled`、`curvature_v_min_scale`、`curvature_kappa_*` |
| 倒车速度 | `reverse_speed_ratio`、`reverse_linear_speed` |
| 夹角倒车 | `allow_reversing`、`allow_angle_reverse`、`reverse_enter_angle`、`reverse_exit_angle` |
| 倒车回正 | `reorient_enabled`、`reorient_min_reverse_sec`、`reorient_clear_*` 等 |
| 卡住脱困 | `stuck_*` |
| 激光防撞 | `laser_*` |

> 节点只在**启动时**读 `~` 参数；改完必须重启 `nav_bringup` 或整棵 `nav_all_in_one`。仅 `rosparam set` 通常**不会**改变已运行节点行为。

---

### 4.2 `scripts/`（节点与工具）

| 文件 | 作用 |
|------|------|
| **`astar_nav_node.py`** | A\* 全局规划：订 `/map`、`/amcl_pose`、目标（remap 为 `/move_base_simple/goal`），发 `/path`（RViz 绿线）。硬膨胀 + 安全域软代价逻辑在此；**默认格数请改 launch** |
| **`pure_pursuit_nav_node.py`** | 纯追踪跟踪 `/path`，发 `/scout_mini_pro/cmd_vel`；含曲率调速、夹角倒车、卡住恢复、倒车回正、激光减速。**默认速度等请改 launch** |
| `check_cmd_vel_drops.py` | 调试：统计 `cmd_vel` 接收间隔，查是否丢包/卡顿 |
| `reset_robot_pose.py` | 调试：调用 `/gazebo/set_model_state` 把车挪回指定位姿（卡住后手动复位） |

`scripts/__pycache__/` 为 Python 缓存，可忽略。

---

### 4.3 `config/nav/`

| 文件 | 当前是否被导航使用 | 说明 |
|------|-------------------|------|
| **`amcl_params.yaml`** | **是** | AMCL 定位参数；`nav_all_in_one` 加载到 `amcl` 命名空间 |
| `costmap_common_params.yaml` | **否** | 旧 move_base 代价地图公共参数，可保留作参考 |
| `global_costmap_params.yaml` | **否** | 旧全局 costmap |
| `local_costmap_params.yaml` | **否** | 旧局部 costmap |
| `base_local_planner_params.yaml` | **否** | 旧局部规划器 |

本栈**没有** move_base；改「离墙远近」请改 `nav_bringup.launch` 的 A\* 参数，不要改上述 costmap yaml 指望生效。

---

### 4.4 `xacro/`（模型结构）

```
xacro/
├── main/scout_mini_pro.xacro     # 主入口：全局尺寸参数 + include 各子模块 + 车身
├── actuator/wheel.xacro          # 四轮实例（前轮低摩擦 / 后轮默认摩擦）
├── sensor/
│   ├── laser_helios.xacro        # 激光
│   ├── camera_d435.xacro         # 深度相机
│   └── imu_ch110.xacro           # IMU
└── macro/
    ├── common_macros.xacro       # wheel / sensor_fixed 等宏（含 mu1/mu2）
    ├── gazebo_diff_drive.xacro   # 差速驱动插件（cmd_vel、odom）
    └── gazebo_sensor_plugin.xacro# 激光/相机/IMU 的 Gazebo 插件
```

| 改什么 | 优先文件 |
|--------|----------|
| 车身长宽高、轮径、轮距 | `xacro/main/scout_mini_pro.xacro` 顶部 `xacro:property` |
| **前轮摩擦（转向涩/滑）** | `xacro/actuator/wheel.xacro` 的前轮 `mu1`/`mu2`（宏定义在 `common_macros.xacro`） |
| 后轮摩擦 | `wheel.xacro` 后轮（未显式写则用宏默认 `mu1=1.0 mu2=0.8`） |
| 差速话题/轮基 | `macro/gazebo_diff_drive.xacro` |
| 传感器安装位姿/话题 | 对应 `sensor/*.xacro` + `gazebo_sensor_plugin.xacro` |

改 xacro 后须**完全重启 Gazebo**（仅重启 nav 不够）。

---

### 4.5 `maps/`

| 文件 | 作用 |
|------|------|
| `room.yaml` | map_server 元数据（分辨率 0.05、origin、阈值等） |
| `room.pgm` | 占用栅格图像 |
| `.gitkeep` | 保证空 maps 目录可被 git 跟踪 |

`nav_bringup` / `nav_all_in_one` 默认 `map_file` 指向本目录的 `room.yaml`。换图：放入新 `.pgm`+`.yaml`，或改 launch 的 `map_file` 参数。

---

### 4.6 `rviz/`

| 文件 | 用途 |
|------|------|
| `display_room_slam.rviz` | **导航 / SLAM 推荐**：含 Map、LaserScan、Path、SetGoal 等 |
| `display_room.rviz` | room 场景传感器可视化 |
| `display.rviz` | 仅模型展示（配合 `display_rviz.launch`） |

`nav_all_in_one` 默认加载 `display_room_slam.rviz`。

---

### 4.7 `world/`

| 路径 | 作用 |
|------|------|
| `room_1.world` | **当前 `gazebo_room.launch` 使用的世界**（小房间 + cafe_table） |
| `room_3.world` | 另一房间世界（备用） |
| `room/room_1/` | room_1 墙体模型（`model.sdf` + `model.config`） |
| `room/room_2/room_2/` | room_2 模型文件 |

改房间几何 → 改对应 `model.sdf` / `.world`，再重启 Gazebo。

---

### 4.8 `rqt/`

| 文件 | 作用 |
|------|------|
| `frames.dot` / `frames.svg` / `frames.png` | 某次导出的 TF 树图，便于理解 `map→odom→base_footprint…` |

非运行时依赖。

---

## 五、「我想改 X → 去改 Y」速查表

> 改完重启规则：  
> - 改 `nav_bringup.launch` / 脚本节点逻辑 → 重启 **nav_bringup** 或 **nav_all_in_one**  
> - 改 xacro 摩擦/外观 → **重启 Gazebo**（或整棵 all_in_one）  
> - 改 `maps/room.*` → 重启 map_server（随 nav 一起重启）

| 我想… | 去改 | 说明 |
|-------|------|------|
| **线速度更快/更慢** | `launch/nav_bringup.launch` → `max_linear_speed` / `min_linear_speed` | ★ 主调参文件 |
| **角速度更敢转/更柔** | 同上 → `omega_max`、`omega_v_floor`、`omega_gain` | 勿把 `omega_max` 压得过低 |
| **倒车速度** | 同上 → `reverse_linear_speed` / `reverse_speed_ratio` | 建议约 `vmax×2/3` |
| **倒车回正更早/更晚/开关** | 同上 → `reorient_*` / `reorient_enabled` | 见 README_3 第十二节细表 |
| **开/关夹角倒车** | 同上 → `allow_angle_reverse` | `false` 时仅脱困才倒车 |
| **更早/更晚进入夹角倒车** | 同上 → `reverse_enter_angle` | 调小更早（如 `1.745`≈100°） |
| **A\* 膨胀（绿线离墙）** | 同上 → `expansion_size`（astar 节点段） | 过大门口易无路径 |
| **A\* 安全域** | 同上 → `safety_zone_radius` / `safety_zone_weight` | 软代价，不堵死通道 |
| **一键启动导航** | 用 `nav_all_in_one.launch`（参数仍在 nav_bringup） | 不必复制参数到 all_in_one |
| **分步启动** | `gazebo_room` → `nav_bringup` → amcl → RViz | **必须先 nav_bringup（含 map）再 amcl** |
| **建图** | `gazebo_room_slam.launch`；保存到 `maps/` | 见 README_2 |
| **车模前轮摩擦 / 转向涩** | `xacro/actuator/wheel.xacro` 前轮 `mu1`/`mu2` | 须重启 Gazebo |
| **车身尺寸 / 外观颜色** | `xacro/main/scout_mini_pro.xacro`（及 sensor 材质） | 方块几何，非官方 mesh |
| **换 / 更新地图** | `maps/room.yaml` + `room.pgm`（或改 `map_file`） | 重启 map_server |
| **AMCL 定位行为** | `config/nav/amcl_params.yaml` | 重启 amcl / all_in_one |
| **官方真实外观 Scout** | `scout_description` + README_4 | 与自制导航栈分离；勿与日常调参混淆 |
| **改规划/跟踪算法本身** | `scripts/astar_nav_node.py` 或 `pure_pursuit_nav_node.py` | 默认数值仍建议经 launch 注入 |

更细的参数表与日志字段见 [README_3.md 第十二节](README_3.md#十二手调参数一页指南想调-x--改-y)。

---

## 六、文档各自定位

### 6.1 第一阶段 · 2D（本目录 `2D/`）

| 文档 | 路径 | 定位 |
|------|------|------|
| **README.md** | `2D/README.md` | 本目录索引 |
| **FILE_STRUCTURE.md** | 本文件 | **文件架构 +「改 X 找 Y」** |
| **README_1.md** | `2D/` | 自制模型说明：xacro、空世界/RViz、传感器话题 |
| **README_2.md** | `2D/` | room 场景、传感器可视化、**slam_toolbox 建图与存图** |
| **README_3.md** | `2D/` | **导航操作手册**：一键启动、调参、故障排查 |
| **README_4.md** | `2D/` | **官方 scout_\* 包**接入与 official 导航 |
| **debug_records.md** | `2D/` | 现场调试流水账 |

### 6.2 第二阶段 · 3D（工作区 `3D/`）

> 总方向：图像/语义后置；先深度几何 + TD3 + 可通行性规划。详见 [`../3D/`](../3D/README.md)。

| 文档 | 路径 | 定位 |
|------|------|------|
| **README.md** | `3D/README.md` | 第二阶段索引 |
| **3D_README.md** | `3D/` | 总纲与拍板方向 |
| **3D_README_1.md** | `3D/` | 开源资料取舍 |
| **3D_README_2.md** | `3D/` | 架构与话题 |
| **3D_README_3.md** | `3D/` | 里程碑与验收 |

### 6.3 其它

| 路径 | 定位 |
|------|------|
| **`article/`** | 文献清单 `article.md`、过程笔记 `record.md`、EDU PDF、分类 PDF 子目录；**不参与编译** |
| **`src/scout_mini_pro_description/README.md`** | 包内短指针，指向 `2D/` / `3D/` / `article/` |
| **工作区根 `README.md`** | 四分区总览 |

---

## 七、官方 `scout_*` 包简表（与自制包对照）

| 官方路径 | 主要内容 | 何时用 |
|----------|----------|--------|
| `scout_description/urdf/`、`meshes/` | 真实 Scout Mini mesh + `mini.xacro` 等 | 要真实外观仿真时 |
| `scout_description/launch/` | `scout_mini_stock.launch`、`display_*` 等 | 官方模型展示/生成 |
| `scout_gazebo_sim/launch/` | `spawn_scout_mini.launch`、`scout_mini_playpen.launch` 等 | 官方世界里跑官方车 |
| `scout_gazebo_sim/worlds/` | 空世界、playpen | 官方场景 |
| `scout_control/config/` | 控制与 twist_mux yaml | 配合官方控制栈 |

**关系一句话**：日常室内导航演示与调参走 **`scout_mini_pro_description`**；官方三包是调研/换真车模资源，细节见 README_4，**不要指望改官方 yaml 就能改当前 A\*+纯追踪速度**。

---

## 八、推荐阅读顺序（迷路时）

**第一阶段（室内 2D）——本目录**

1. 本文「我想改 X → 去改 Y」→ 打开 `src/...` 对应文件  
2. 动手开仿真/导航 → [README_3.md](README_3.md)  
3. 建图 → [README_2.md](README_2.md)  
4. 改车模 → [README_1.md](README_1.md) + `src/.../xacro/`  
5. 官方真车模 → [README_4.md](README_4.md)  
6. 旧错误 → [debug_records.md](debug_records.md)  

**第二阶段（3D 越野）——[`../3D/`](../3D/README.md)**

1. [3D_README.md](../3D/3D_README.md)  
2. [3D_README_1.md](../3D/3D_README_1.md)  
3. [3D_README_2.md](../3D/3D_README_2.md)  
4. [3D_README_3.md](../3D/3D_README_3.md)
)
