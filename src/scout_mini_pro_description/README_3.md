# 导航与建图操作指南（ROS Noetic）

本文档说明如何在 **Ubuntu 20.04、ROS Noetic、Gazebo 11.15** 下，基于已有 Scout Mini Pro 仿真与 room 场景，完成**建图**和**基于地图的自主导航**。导航栈采用 **A 路径规划 + 纯追踪（Pure Pursuit）**，不使用 move_base。

---

## 一、概述与版本对照

### 文档用途

在已有 **Scout Mini Pro + Gazebo room** 与**建图流程**（见 [README_2.md 第六节](README_2.md#六使用-slam_toolbox-构建导航地图与保存)）基础上，完成**基于已保存地图的自主导航**：加载地图、定位（AMCL）、A 规划路径、纯追踪控制，以及在 RViz 中设目标、初始化位姿。

### ROS2 教材 → ROS Noetic 对照表


| 教材（ROS2）                      | ROS Noetic 对应                                                   |
| ----------------------------- | --------------------------------------------------------------- |
| Nav2 Goal（RViz 按钮）            | **SetGoal**，发布到 `/move_base_simple/goal`                        |
| 2D Pose Estimate（初始化位姿）       | **SetInitialPose**，发布 `/initialpose`（PoseWithCovarianceStamped） |
| amcl（/initialpose、/map、/scan） | amcl 包，话题与消息类型一致                                                |
| map_server 加载地图               | map_server 节点 + room.yaml                                       |
| 目标点 / 路径跟踪                    | 本包：A 节点发布 `/path`，纯追踪节点订阅并发布 `/scout_mini_pro/cmd_vel`          |


### Noetic 导航架构简述

- **map_server**：根据已保存的 `room.yaml`/`room.pgm` 发布 `/map`（OccupancyGrid）。
- **amcl**：订阅 `/map`、`/scan`、`/initialpose`，发布 `map` → `odom` 的 TF 及 `/amcl_pose`。
- **astar_nav_node**：订阅 `/map`、`/amcl_pose`、`/move_base_simple/goal`，规划路径并发布 `/path`（Path，frame_id=map）。
- **pure_pursuit_nav_node**：订阅 `/path`、`/amcl_pose`，发布 `/scout_mini_pro/cmd_vel` 控制小车。
- **TF 链**：`map` → `odom`（amcl）→ `scout_mini_pro/odom`（static）→ `scout_mini_pro/base_footprint`（Gazebo）→ `base_footprint`（static）→ 其余 link。

### 整体导航逻辑与可检索名称（查资料用）

本套导航**没有**像「A*」那样一个词就概括全栈的名字，而是**三块经典算法的组合**：

| 模块 | 算法/名称 | 英文检索关键词（便于网上查资料） |
|------|-----------|----------------------------------|
| 定位 | AMCL（自适应蒙特卡洛定位） | **AMCL**, "particle filter localization" |
| 全局路径规划 | **A\***（A 星） | **A\* path planning**, "A star grid", "costmap" |
| 路径跟踪 / 控制 | **Pure Pursuit**（纯追踪） | **Pure Pursuit**, "car-like path tracking", "lookahead tracking" |

- **整体可称为**：「**A\* 全局路径规划 + Pure Pursuit 路径跟踪**」（英文：A* global path planning with Pure Pursuit path following）。
- 在 ROS 里，和本方案最接近的经典栈是 **move_base**（全局规划 + 局部规划）；本包是简化版：只有全局 A* + 纯追踪，没有局部避障层（如 DWA、TEB）。
- 若要查「不贴墙」「安全距离」相关，可再加：**costmap inflation**、**clearance cost**、**distance transform cost**。

---

## 二、建图部分（简要）

建图流程见 [README_2.md 第六节](README_2.md#六使用-slam_toolbox-构建导航地图与保存)：使用 **slam_toolbox** 在线建图，用 **map_saver** 保存为 `room.pgm` 与 `room.yaml`。

**导航前提**：已得到 `maps/room.pgm` 与 `maps/room.yaml`。若尚未建图，请先按 README_2 完成建图并保存到 `scout_mini_pro_description/maps/`。

---

## 三、导航前置条件（话题与 TF）


| 用途   | 话题/帧                      | 说明                                        |
| ---- | ------------------------- | ----------------------------------------- |
| 激光   | `/scan`                   | 激光已发布到全局 `/scan`                          |
| 里程计  | `/scout_mini_pro/odom`    | amcl 需 remap：`odom:=/scout_mini_pro/odom` |
| 控制   | `/scout_mini_pro/cmd_vel` | 纯追踪节点直接发布到该话题                             |
| 地图   | `/map`                    | map_server 发布                             |
| 初始位姿 | `/initialpose`            | amcl 订阅                                   |
| 导航目标 | `/move_base_simple/goal`  | RViz SetGoal 发布，A* 节点订阅                   |
| 路径   | `/path`                   | A* 节点发布，纯追踪节点订阅                           |


**帧名称**：`map`、`odom`、`base_footprint`。

---

## 四、安装依赖

```bash
sudo apt update
sudo apt install ros-noetic-amcl ros-noetic-map-server
```

- **ros-noetic-amcl**：定位，订阅 /map、/scan、/initialpose，发布 map→odom TF 与 /amcl_pose。
- **ros-noetic-map_server**：map_server（加载地图）和 map_saver（保存地图）。建图时若已安装可跳过。

本包导航不依赖 move_base，无需安装 `ros-noetic-navigation`。A 与纯追踪节点依赖 **numpy**、**scipy**（贝塞尔平滑与 KDTree）。若未安装可执行：`sudo apt install python3-scipy python3-numpy`。

---

## 4.1 编译工作空间

导航前需先编译工作空间并 source，否则 `roslaunch`、`rospack find` 可能报 “package not found”。

- **若 `src` 下只有 catkin 包**（如仅 `scout_mini_pro_description` 等）：
  ```bash
  cd ~/catkin_ws_2
  source /opt/ros/noetic/setup.bash
  catkin_make
  source devel/setup.bash
  ```
- **若 `src` 下还有 ROS2（ament）包**（例如同学的 ROS2 仓库）：需使用 `catkin_make_isolated`，或在编译前将 ROS2 包移出 `src` 后再执行 `catkin_make`。使用 isolated 时编译后执行 `source devel_isolated/setup.bash`（或按实际生成的 devel 路径 source）。

之后每次新开终端做导航相关操作前，都要先执行 `cd ~/catkin_ws_2` 和 `source devel/setup.bash`（见第六节）。

---

## 五、导航参数与配置

- **AMCL**：使用 `config/nav/amcl_params.yaml`。启动 amcl 时需 remap：`odom:=/scout_mini_pro/odom`。
- **use_sim_time**：仿真时必须先执行 `rosparam set use_sim_time true`（见第六节步骤 2）。
- **A* / 纯追踪**：由 `nav_bringup.launch` 启动，无需额外 yaml；话题与帧已在节点内写死。

`config/nav/` 下其余 costmap、base_local_planner 等为历史 move_base 配置，当前导航栈**不再使用**，可保留作参考或删除。

---

## 六、启动导航的步骤（具体操作）

**前提**：已建图，`scout_mini_pro_description/maps/` 下存在 `room.yaml` 和 `room.pgm`。导航时**不要**使用 `gazebo_room_slam.launch`（建图用，与静态地图冲突）。

### 6.0 操作总览（按顺序复制执行）

**每个新终端必须先执行下面两行**（否则 `rospack find` 会报 “package not found”，后续命令中的路径会为空）：

```bash
cd ~/catkin_ws_2
source devel/setup.bash
```

然后按顺序开 4 个终端，分别执行（以下均默认“该终端已执行过上面两行”）：

1. **终端 1**：`roslaunch scout_mini_pro_description gazebo_room.launch` → 保持运行。
2. **终端 1 或 2**：`rosparam set use_sim_time true` → 执行一次即可。
3. **终端 2**：`roslaunch scout_mini_pro_description nav_bringup.launch` → 保持运行（**先启动 map_server 发布 /map**，否则 amcl 会报 “Request for map failed” 并崩溃）。
4. **终端 3**：先执行 `rosparam load $(rospack find scout_mini_pro_description)/config/nav/amcl_params.yaml /amcl`，再执行 `rosrun amcl amcl odom:=/scout_mini_pro/odom` → 保持运行。
5. **终端 4**：`rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room_slam.rviz` → 在 RViz 中先 **SetInitialPose**，再 **SetGoal**（见下方步骤 6）。

**重要**：必须先执行第 3 步（nav_bringup，内含 map_server），再执行第 4 步（amcl）。amcl 启动时会请求 `/map`，若 map_server 未先运行，会持续报 “Request for map failed; trying again...” 并可能段错误退出。

若任一步中 `rospack find scout_mini_pro_description` 报错，说明该终端未执行上述两行，请先执行再重试。

### 启动顺序总览表


| 顺序  | 终端       | 做什么                         | 命令                                                                               |
| --- | -------- | --------------------------- | -------------------------------------------------------------------------------- |
| 1   | 终端 1     | Gazebo + 房间 + 小车            | `roslaunch scout_mini_pro_description gazebo_room.launch`                        |
| 2   | 终端 1 或 2 | 仿真时间                        | `rosparam set use_sim_time true`                                                 |
| 3   | 终端 2     | 启动导航（map_server + A* + 纯追踪） | `roslaunch scout_mini_pro_description nav_bringup.launch`（**必须先于 amcl**）         |
| 4   | 终端 3     | 加载 amcl 参数并启动 amcl          | 见下方「步骤 4」                                                                        |
| 5   | 终端 4     | 启动 RViz                     | `rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room_slam.rviz` |
| 6   | RViz     | 设初始位姿再设目标                   | 先 **SetInitialPose**，再 **SetGoal**（见下方步骤 6）                                      |


### 步骤 1：启动 Gazebo（终端 1）

```bash
roslaunch scout_mini_pro_description gazebo_room.launch
```

**预期**：Gazebo 窗口打开，出现 room 场景和 Scout Mini Pro 小车；终端可能有 “Finished loading” 等提示。**本终端不要关闭，保持运行**。

### 步骤 2：设置仿真时间（执行一次）

```bash
rosparam set use_sim_time true
```

**预期**：无输出即表示成功。在**任意已 source 的终端**执行一次即可（例如在终端 1 或新开终端 2 执行）；amcl 与导航 launch 启动前需已执行。

### 步骤 3：启动导航 launch（终端 2）

```bash
roslaunch scout_mini_pro_description nav_bringup.launch
```

**预期**：map_server 输出类似 “Read a map from ... resolution 0.05 ... width ... height ...”；astar_nav_node、pure_pursuit_nav_node 无报错并持续运行。**本终端保持运行**。

**重要**：必须先执行本步骤再启动 amcl。amcl 启动时会请求 `/map`，若 map_server 未先运行，amcl 会持续报 “Request for map failed; trying again...” 并可能段错误退出。

**自检（可选）**：`rostopic list | grep -E 'path|map'` 应能看到 `/path`、`/map`；`rostopic echo /map -n 1` 能收到一帧说明地图已发布。

### 步骤 4：启动 AMCL（终端 3）

先确保步骤 1、2、3 已完成（**尤其步骤 3 的 map_server 已在运行**）；若刚启动 Gazebo，建议等约 10 秒再启动 nav_bringup。

**命令 4.1**（加载参数到 `/amcl`）：

```bash
rosparam load $(rospack find scout_mini_pro_description)/config/nav/amcl_params.yaml /amcl
```

**命令 4.2**（启动 amcl，remap odom）：

```bash
rosrun amcl amcl odom:=/scout_mini_pro/odom
```

**预期**：amcl 持续运行，终端无 “Request for map failed”；收到地图后可能出现 “Received a ... X ... Y map” 之类输出。**本终端保持运行**。

**自检（可选）**：在**已设 SetInitialPose 之后**，另开终端执行 `rostopic echo /amcl_pose -n 1`，若有位姿输出说明 amcl 正常。`rosrun tf tf_echo map base_footprint` 能打出坐标也说明 map 帧已存在。

### 步骤 5：启动 RViz（终端 4）

```bash
rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room_slam.rviz
```

**预期**：RViz 窗口打开，左侧 Displays 中有 Map、RobotModel、LaserScan 等；Global Options 中 **Fixed Frame** 为 `map`；地图、机器人、激光可见。若看不到地图，检查 Displays 里 Map 的 **Topic** 是否为 `/map`。本包该 rviz 配置已把 SetGoal 的 Topic 设为 `/move_base_simple/goal`，无需改配置。**步骤 4（amcl）需已启动**，否则 A* 与纯追踪收不到 map 下位姿（/amcl_pose）。

### 步骤 6：在 RViz 中操作（具体怎么点）

**6.1 设初始位姿（SetInitialPose）**

- 在 RViz **顶部工具栏**或左侧 **Tool Properties** 面板的工具列表中，选择 **SetInitialPose**（或 2D Pose Estimate，绿色直线箭头图标）。
- 在地图窗口中**先对照 Gazebo 里小车实际位置**，在对应位置**单击一次**，然后**按住左键拖拽**拉出朝向箭头，箭头方向与 Gazebo 中小车车头一致后**松开**。
- 说明：该操作只向 `/initialpose` 发一条消息，由 AMCL 接收并更新估计位姿；**不会**在 Gazebo 里移动小车。若点偏了，RViz 与 Gazebo 显示会不一致，需重新选 SetInitialPose 再点一次。

**6.2 设目标并开始导航（SetGoal）**

- 在工具列表中选择 **SetGoal**（或 2D Nav Goal，弯曲箭头图标）。
- 在地图**可通行区域**内**单击**目标点，**按住左键拖拽**出车头朝向，**松开**。
- 说明：会向 `/move_base_simple/goal` 发一次目标；A* 会规划路径并发布 `/path`，纯追踪控制小车沿路径移动。若 RViz 中添加了 Path 显示且订阅 `/path`，可看到路径线，小车应开始移动。

**若小车不动**：确认 amcl、nav_bringup 均在运行；已用 SetInitialPose 设好位姿；再设 SetGoal。可检查 `rostopic echo /path -n 1`、`rostopic echo /scout_mini_pro/cmd_vel` 是否有数据。

---

## 七、在 RViz 中设初始位姿与目标（SetInitialPose / SetGoal）

- **SetInitialPose**：在工具栏或 Tool Properties 中选 SetInitialPose（2D Pose Estimate），在地图上**对照 Gazebo 小车实际位置**单击并**拖拽**出朝向箭头后松开。只向 `/initialpose` 发一条消息，由 AMCL 接收；不会在 Gazebo 里移动小车。点偏了会导致 RViz 与 Gazebo 显示不一致，需重选工具再点一次。
- **SetGoal**：选 SetGoal（2D Nav Goal），在地图可通行区域内单击目标并拖拽朝向后松开。向 `/move_base_simple/goal` 发布目标；A* 规划 `/path`，纯追踪发布 `/scout_mini_pro/cmd_vel`，小车开始移动。本包 `display_room_slam.rviz` 已配置 SetGoal 的 Topic 为 `/move_base_simple/goal`，无需改配置。

---

## 八、单点与路点导航

- **单点**：设好 SetInitialPose 后，用 SetGoal 发一个目标即可。
- **路点**：到达当前目标附近后，再在 RViz 中用 SetGoal 发下一个目标；或后续可编写脚本按序向 `/move_base_simple/goal` 发布多个 PoseStamped。

### 自检清单（快速定位卡在哪一步）

在**已按第六节顺序启动**的前提下，可在新终端（先执行 `cd ~/catkin_ws_2`、`source devel/setup.bash`）用下面命令逐条自检：


| 检查项             | 命令                                                  | 预期                                              |
| --------------- | --------------------------------------------------- | ----------------------------------------------- |
| Gazebo 在跑       | `rostopic list | grep -E 'clock|odom'`              | 有 `/clock`、`/scout_mini_pro/odom`               |
| use_sim_time 已设 | `rosparam get /use_sim_time`                        | 输出 `True`                                       |
| 地图在发            | `rostopic echo /map -n 1`                           | 有一帧 map 数据                                      |
| amcl 在跑且已设初始位姿  | `rostopic echo /amcl_pose -n 1`                     | 有一帧位姿（设 SetInitialPose 后）                       |
| 已设目标且 path 在发   | **先**在 RViz 点一次 **SetGoal**，再执行 `rostopic hz /path` | 有发布频率（约 10 Hz）                                  |
| 小车在动            | `rostopic echo /scout_mini_pro/cmd_vel`             | 有非零 linear.x 或 angular.z（设 SetGoal 且 path 有数据后） |


**说明**：`/path` 和 `/scout_mini_pro/cmd_vel` **只有在 RViz 里点过 SetGoal 之后**才会有数据。若运行 `rostopic echo /path -n 1` 出现 “no messages received and simulated time is active. Is /clock being published?”，通常有两种情况：（1）**还没在 RViz 里点 SetGoal**，请先设初始位姿再设目标；（2）Gazebo 未在跑或未发布 `/clock`，请确认终端 1 的 Gazebo 在运行。自检时建议用 `rostopic hz /path`（在 SetGoal 之后），若有频率输出说明 A* 在发路径。

若某一步不符合预期，说明该步之前的节点或操作未就绪，按第六节顺序排查。

---

### 膨胀（expansion_size）说明

**膨胀** 指 A* 规划时对地图上障碍物（墙）的**栅格膨胀**：每个障碍格向四周扩展 `expansion_size` 格，扩展出的格也当作障碍，这样规划出的路径会**至少离墙 expansion_size 格**（地图分辨率 0.05 m/格）。相当于“把墙加厚”，让路径不要贴墙走。

- **与车身的关系**：小车实际宽约 0.4 m（约 8 格），半宽约 0.2 m（约 4 格）。膨胀大（如 4 格）时，路径离墙约 0.2 m，与半车宽一致，车能安全通过；膨胀小（如 1 格）时，路径只离墙 0.05 m，容易钻进比车还窄的缝，车会卡住。
- **为何“设成 1 才可以过”**：门口/拐角在栅格上较窄时，膨胀大了（2、3、4）会把通道堵死，A* 报 “no path”；只有把 `expansion_size` 设为 1 才勉强有路径。但 1 格远小于车宽，路径可能穿过 0.1 m 级别的缝，车体 0.4 m 就会卡住或贴墙。这是**膨胀与车身/门宽之间的取舍**，不是 bug。
- **如何调节**：launch 或 rosparam 可设 `expansion_size`（1/2/3/4）。**1**：最容易出路径，拐角/窄门能过，但路径可能过窄、车易卡；**4**：路径最安全、贴近车身，但门口/拐角易 no path；**2 或 3**：折中。若 no path 多，可试 2 或 1；若车常贴墙卡住，可试 3 或 4（前提是门宽允许）。

### 安全域（safety_zone_radius）说明

**安全域** 用于实现「有路可走时不贴墙、必要时才靠近墙」：在 `expansion_size` 之外、`safety_zone_radius`（格）之内，对离墙距离施加**软代价**（离墙越近代价越高），A* 会偏好走走廊中线；窄门、窄通道处仍可通行，只是代价更高。

- **参数**：`/astar_nav_node/safety_zone_radius`（单位：格），默认 **0** 表示关闭（与之前行为一致）。可选 `safety_zone_weight`（默认 1.0）控制软代价强度（0.5～2.0 可调）。
- **建议取值**：关闭时保持 0；若希望路径尽量不贴墙，可设为 **6～10 格**（约 0.3～0.5 m @ 0.05 m/格）。过大会在窄门、拐角处明显抬高代价，可能出现 no path，可适当减小或配合 `expansion_size` 使用。
- **在线设置**：`rosparam set /astar_nav_node/safety_zone_radius 8`（需在 nav_bringup 已启动的节点上生效，通常需重启 A* 节点或下次 launch 时生效）。

### 参考代码对照与路径贴墙说明

**与参考代码（Pure-tracking-slam-automatic-navigation-system-masters）的差异**  
参考仓库中 A* 在 costmap 之后做了 `data[(data >= -2) & (data <= 5)] = 1`，即把膨胀后的障碍值（5）也设成可通行（1），因此栅格上几乎全是可通行，A* 容易出路径且不严格沿墙。本包**保留障碍**（`data[data > 1] = 100`），规划会真实避障，路径会沿“膨胀后的障碍边界”走，这是预期行为。纯追踪部分：参考用 `(closest_point_idx + i) % len(path_points)` 做前视点（路径首尾相接时适用），本包用沿路径弧长选 lookahead 点，更适合单段起点到终点的路径。

**为什么路径会“贴墙”**  
A* 求的是**最短路径**。在通道里，最短路径往往贴着“可通行区域”的边界，也就是膨胀后的墙边，所以**路径天生会贴墙**。贝塞尔/样条只是把折线拟合成平滑曲线，**不会把路径往通道中间推**，所以平滑后的路径仍然沿墙，只是拐角变圆滑。若要路径更靠通道中间，需要：（1）增大膨胀（路径离墙更远，但窄门易 no path），或（2）后处理：把路径点沿“远离最近障碍”的方向平移一段（需要距离场或碰撞检测），或（3）在 A* 里用“离障碍越远代价越低”的代价函数（如基于距离变换的代价），而不是 0/100 二值。

**能否改成更平滑、少贴墙的曲线**  
- **更平滑**：已用 B 样条平滑；可适当增大平滑后的采样点数（如 `bezier_smoothing(..., max(30, len(path)))`），曲线更密、视觉更顺，但不会改变“贴墙”程度。  
- **少贴墙**：在不改 A* 核心的前提下，可做“路径后处理”：对每个路径点，沿路径切向和法向，向通道内侧偏移一定距离（需结合地图判断是否撞墙）。这属于可选增强，当前实现未做；若需要可在此基础上加一层后处理脚本或参数。

### 导航控制参数（线速度 / 角速度）

- **控制节点与代码位置**：导航控制在 `scripts/pure_pursuit_nav_node.py` 中的 `PurePursuitNavNode.control_cb` 完成：  
  - 线速度由变量 `speed` 决定：`speed = max(0.6, 1.5 - 1.5 * math.sin(0.6 * abs(steering_angle)))`，再在大转角时限制为 `speed = min(speed, 0.5)`。  
  - 角速度由 `steering_angle` 决定，并在 `omega_max` 范围内饱和：`cmd.angular.z = max(-omega_max, min(omega_max, steering_angle))`。

- **在线调整角速度上限（推荐方式）**：  
  - 参数名：`/pure_pursuit_nav_node/omega_max`，默认 `1.0`（单位 rad/s）。  
  - 启动导航前或运行中，在新终端（已 `source devel/setup.bash`）执行，例如：  
    ```bash
    # 把角速度上限调大到 1.5 rad/s
    rosparam set /pure_pursuit_nav_node/omega_max 1.5
    ```  
  - 数值越大，车在转弯时允许的角速度越大，越“灵活”；太大可能导致转弯过猛甚至打滑，一般建议在 **0.8–1.5** 之间试。

- **通过 launch 固定角速度上限**：  
  - 可在 `launch/nav_bringup.launch` 中的 `pure_pursuit_nav_node` 节点下增加一行：  
    ```xml
    <node pkg="scout_mini_pro_description" type="pure_pursuit_nav_node.py" name="pure_pursuit_nav_node" output="screen">
      <param name="omega_max" value="1.5" />
    </node>
    ```  
  - 这样每次 `roslaunch scout_mini_pro_description nav_bringup.launch` 都会带上相同的角速度上限。

- **线速度调节位置说明**：  
  - 线速度的公式目前**写死在代码里**（`speed = max(0.6, 1.5 - 1.5 * sin(...))`），没有单独的 rosparam。  
  - 若需要整体减慢或加快，可在 `pure_pursuit_nav_node.py` 中修改这行里的常数：  
    - 把最小速度 `0.6` 改小，例如 `0.3`，可让转弯时更稳，不会“直冲”；  
    - 把 `1.5` 改小/大，相当于整体缩放最高速度。  
  - 修改后需重新编译工作空间：  
    ```bash
    cd ~/catkin_ws_2
    catkin_make
    ```

### 弯道不转弯（一直往前冲）的原因与修复

**现象**：全局路径（红色）在 RViz 里已经画出弯道，但小车不拐弯，一直朝车头方向冲（甚至冲墙）。

**根本原因（已修复）**：之前 lookahead 点是用**欧氏距离**选的——“从路径上离车最近的点开始，沿路径下标往后找，第一个到车距离 ≥ lookahead_distance 的点”作为目标。在弯道前，离车最近的点往往还在**弯道前的直线段**上，第一个达到 0.3 m 的点也还在同一条直线上，所以目标点一直在车头正前方，**转向角始终≈0**，角速度几乎为 0，看起来就像“没调用转弯代码”。

**修复**：改为用**沿路径的弧长**选 lookahead 点。从“离车最近的路径点”开始，**沿路径向前**累加每一段的长度，第一个累计弧长 ≥ `lookahead_distance` 的点作为目标。这样在弯道处，目标点会落在弯道或弯道之后，控制器会算出非零转向角，小车会拐弯。

**若仍不拐弯**：可适当增大 `lookahead_distance`（如 0.4～0.5），或增大 `omega_max`（见上文「导航控制参数」）。

**话题与角速度都正常仍不转弯**：若已确认 `rostopic info /scout_mini_pro/cmd_vel` 显示 Gazebo 在订阅、且 `rostopic echo /scout_mini_pro/cmd_vel` 能看到非零 `angular.z`，但仿真中小车仍只直行，问题多半在**仿真侧**（差速插件或模型）。可做两件事：（1）检查差速插件的 **wheelTorque** 与 **wheelAcceleration**：本包已在 xacro 中显式设置（如 `gazebo_diff_drive.xacro` 内 `wheel_torque`、`wheel_acceleration`），若 Gazebo 启动日志仍提示 “missing &lt;wheelTorque&gt;” 等，请确认主 xacro 调用该宏时传入了这两项；（2）**用 teleop 做一次“纯转弯”验证**（见下）。

**操作步骤（路径正常、小车仍只往前冲时请按顺序做）**：

1. **确认已用最新 xacro 并重启 Gazebo**  
   修改过 `gazebo_diff_drive.xacro` 或 `scout_mini_pro.xacro` 后，必须执行 `cd ~/catkin_ws_2 && catkin_make`，然后**关掉当前 Gazebo 窗口和 gazebo_room.launch 终端**，再重新执行 `roslaunch scout_mini_pro_description gazebo_room.launch`。否则仿真仍用旧的 `robot_description`，扭矩/加速度不会生效。

2. **Teleop 纯转弯验证（判断是插件问题还是导航逻辑问题）**  
   - **只开 Gazebo**：只运行 `gazebo_room.launch`，不要开 nav_bringup、amcl、RViz。  
   - 新终端：`source devel/setup.bash` 后执行  
     `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/scout_mini_pro/cmd_vel`  
   - 在 teleop 里**只按左转/右转键**（一般是 `j`/`l` 或方向键），不要按前后。看小车在仿真里是否转弯。  
   - **若 teleop 下也不转弯**：说明是 Gazebo 差速插件或模型问题。本包已把默认 `wheel_torque` 设为 **300**、`wheel_acceleration` 设为 **20.0**（在 `gazebo_diff_drive.xacro` 与 `scout_mini_pro.xacro`）。请先**重新编译并完全重启 Gazebo** 使新参数生效；若仍不转弯，可再改为 `wheel_torque="500"`、`wheel_acceleration="50.0"` 后同样重新编译并重启 Gazebo 再测。  
   - **若 teleop 下能转弯**：说明插件正常，问题在导航（如控制频率、坐标系等），可对比 `rostopic echo /scout_mini_pro/cmd_vel` 在 teleop 与 pure_pursuit 下的输出差异。

---

## 九、常见问题与排查


| 现象                                                                  | 可能原因                                       | 建议                                                                                                                                                                                                                                                                                                                                             |
| ------------------------------------------------------------------- | ------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| amcl 报 “Request for map failed” 或段错误                                | amcl 早于 map_server 启动，/map 未发布             | **先启动 nav_bringup（步骤 3），再启动 amcl（步骤 4）**；确认 `rostopic echo /map -n 1` 有数据后再运行 amcl。                                                                                                                                                                                                                                                            |
| amcl 粒子发散                                                           | 初始位姿偏差大、地图与场景不一致                           | 重新 SetInitialPose 设准；确认地图与 Gazebo 场景一致                                                                                                                                                                                                                                                                                                         |
| 小车不导航、不规划                                                           | 未设 SetInitialPose 或 amcl 未发布 map 帧         | 先 SetInitialPose；确认 amcl 在运行且 `odom:=/scout_mini_pro/odom`                                                                                                                                                                                                                                                                                     |
| A* 报 “no path found”、路径不出                                               | 障碍膨胀过大，房间门/窄通道被堵死                         | 默认膨胀为 3 格。若仍无路径，在**启动 nav_bringup 之前**执行 `rosparam set /astar_nav_node/expansion_size 2` 再启动；或改大以更贴近车身（4 格）。                                                                                                                                                                                                                                          |
| SetGoal 后只有 RViz 有反应，A* 无 “received goal”、/path 无数据                 | 目标话题未接到 A* 节点                              | 在运行 nav_bringup 的终端应看到 `A* node started. Goal topic (remapped): /move_base_simple/goal`。执行 `rostopic info /move_base_simple/goal`：Publishers 应有 RViz，Subscribers 应有 **astar_nav_node**。若没有 astar_nav_node，说明 A* 未订阅到该话题，请确认用本包 `nav_bringup.launch` 启动且未改 remap。RViz 请使用 `display_room_slam.rviz`（SetGoal 的 Topic 为 `/move_base_simple/goal`）。 |
| /path 无数据、rostopic echo 报 “no messages… Is /clock being published?” | 未在 RViz 点 SetGoal，或 use_sim_time 下 echo 超时 | **必须先**在 RViz 点 **SetGoal**（设目标）；再在终端用 `rostopic hz /path` 看是否有频率。确认 Gazebo 在跑（发布 /clock）。                                                                                                                                                                                                                                                     |
| cmd_vel 无输出                                                         | 纯追踪未收到 /path 或 /amcl_pose                  | 确认已 SetGoal 且 `rostopic hz /path` 有输出；nav_bringup 与 amcl 在运行。                                                                                                                                                                                                                                                                                  |
| 弯道不转弯、一直往前冲（路径有弯但车直走）                                        | lookahead 选点或差速插件/仿真参数问题                      | 见第八节「弯道不转弯的原因与修复」：已改弧长选 lookahead、可调 `lookahead_distance`/`omega_max`。若话题与 angular.z 都正常仍不转，检查差速插件的 **wheelTorque**、**wheelAcceleration**（xacro 中已设），并用 **teleop 纯转弯**验证（见上文「话题与角速度都正常仍不转弯」）。                                                                                                                                                                                                   |
| TF 报错、map 不存在                                                       | amcl 未启动或未设初始位姿                            | 先启动 nav_bringup（步骤 3），再启动 amcl（步骤 4），再在 RViz 中 SetInitialPose。                                                                                                                                                                                                                                                                                 |
| 仿真时间异常                                                              | use_sim_time 未设置                           | 执行 `rosparam set use_sim_time true`                                                                                                                                                                                                                                                                                                            |
| 小车卡在障碍中、想恢复到之前位置                                                | 碰撞或路径导致卡住                                 | **方式 A**：Gazebo 窗口菜单 **Model → Reset Pose** 将模型复位；或关闭仿真后重新 `roslaunch scout_mini_pro_description gazebo_room.launch`，小车会重新出现在起点。**方式 B**：`rosrun scout_mini_pro_description reset_robot_pose.py [x] [y] [yaw]` 将小车设到指定位姿（默认 0 0 0）；需 Gazebo 在运行。                                                                                                                                   |


**检查 cmd_vel 是否丢包**：纯追踪以约 20 Hz 发布 `/scout_mini_pro/cmd_vel`。若怀疑因性能或队列导致丢包，可在**导航运行时**另开终端执行：

```bash
rosrun scout_mini_pro_description check_cmd_vel_drops.py
```

脚本会订阅该话题并每 2 秒打印：近期接收频率（msg/s）、本窗口消息数、**长间隔次数**（相邻两条消息间隔 > 0.1 s 即计一次，可能表示丢包或严重延迟）。预期约 20 msg/s、长间隔为 0；若频率明显低于 20 或长间隔很多，可考虑关掉其它占 CPU 的程序或检查系统负载。也可用 `rostopic hz /scout_mini_pro/cmd_vel` 快速看发布端频率。

通用排查：`rostopic list`、`rostopic echo /path -n 1`、`rosrun tf tf_echo map base_footprint`。

---

## 十、文档约定与后续步骤

- 每个新终端先 `cd ~/catkin_ws_2`、`source devel/setup.bash`。
- 默认工作空间为 **catkin_ws_2**，包与地图路径以 **scout_mini_pro_description** 与 **maps** 为例。
- **已实现**：A* + 纯追踪导航、amcl 定位、map_server、nav_bringup.launch、RViz 操作说明。
- **可选后续**：在同一 launch 中加入 amcl 实现“一键导航”；编写路点脚本；微调 A* 膨胀、纯追踪前视距离等（需改脚本参数）。

本 README_3 为基于 *A + 纯追踪** 的导航操作手册；若某步与你的环境不一致，以实际路径与包名为准。