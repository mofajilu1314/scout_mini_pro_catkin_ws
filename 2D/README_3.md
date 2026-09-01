# 导航与建图操作指南（ROS Noetic）

本文档说明如何在 **Ubuntu 20.04、ROS Noetic、Gazebo 11.15** 下，基于已有 Scout Mini Pro 仿真与 room 场景，完成**建图**和**基于地图的自主导航**。导航栈采用 **A 路径规划 + 纯追踪（Pure Pursuit）**，不使用 move_base。

> **文件架构 /「我想改 X 去哪改」**：详见同目录 [FILE_STRUCTURE.md](FILE_STRUCTURE.md)（工作空间顶层、各目录职责、关键文件说明与速查表）。**调导航速度/倒车/膨胀等默认参数 → `src/scout_mini_pro_description/launch/nav_bringup.launch`。**
>
> 文档已集中在工作区 `2D/`；ROS 源码仍在 `src/scout_mini_pro_description/`。第二阶段见 [`../3D/`](../3D/README.md)。

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
- **A* / 纯追踪**：由 `nav_bringup.launch` 启动；默认参数已写在 launch 内（`expansion_size=4`、`lookahead_distance=0.9`、`omega_max=4.0` 等），详见第八节「用户可调四要素」与第十一节。

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

**膨胀** 指 A* 规划时对地图障碍的**硬栅格膨胀**：距障碍 `≤ expansion_size` 格的自由格被标为**不可通行（代价 100）**，路径会**至少离墙 expansion_size 格**（地图分辨率多为 **0.05 m/格**）。这与“只加软代价、仍可贴着墙走”不同——软代价 alone 时最短路径仍会贴膨胀边界；硬膨胀才能真正把绿线推离墙。

- **默认值**：**4 格**（约 **0.20 m** @ 0.05 m/格）；`nav_bringup.launch` 与节点默认一致。room 图门口净宽约 **0.45 m**，硬膨胀 **5** 会断开连通性导致 `no path`。
- **推荐区间**：**3～5 格**（约 0.15～0.25 m）。更安全可选 5，但若日志出现 `no path` / `reduced expansion_size`，保持 4 或降到 3。
- **与车身**：车宽约 0.4 m（约 8 格）。膨胀过小（1～2）路径贴墙，车体易蹭墙；过大易堵死门口。
- **自动回退**：配置膨胀过大找不到路径时，节点会从当前值向下试到 **2**，找到则打 throttle 警告并发布路径。
- **起点/终点**：硬膨胀后，起点/终点附近会**局部取消膨胀**（不打开真实墙格），避免目标被膨胀环堵死、无法起步。
- **如何调节**（改完需重启 `nav_bringup`，或下次 launch 生效）：
  ```bash
  # 启动前也可写在 launch 的 <param name="expansion_size" value="4" />
  rosparam set /astar_nav_node/expansion_size 4
  ```

### 安全域（safety_zone_radius）说明

**安全域** 在硬膨胀之外再加**软代价**：`expansion_size < d ≤ safety_zone_radius` 时，离墙越近代价越高，A* 在宽走廊更偏好走中间；窄通道仍可通行（代价更高），不会像硬膨胀那样直接堵死。

- **默认**：`safety_zone_radius=10`（格），`safety_zone_weight=1.0`（见 `nav_bringup.launch`）。
- **建议**：保持 8～12；若窄门代价过高导致绕路/no path，可降到 6 或把 weight 降到 0.5；设为 **0** 关闭软代价（只保留硬膨胀）。
- **在线**：`rosparam set /astar_nav_node/safety_zone_radius 10`（通常需重启 A* / nav_bringup 才稳定生效）。

### 为何曾“贴墙走”（已修）

旧实现里 `expansion_size` 只给近障格**加软代价**，格仍可通行（`!=100`），A* 最短路仍贴墙。现已改为**硬膨胀 + 安全域软代价**，绿线 `/path` 应明显离墙；若仍贴墙，先确认 nav_bringup 日志里 `expansion_size: 4 (hard)`，并完全重启该 launch。

### 导航控制参数（Pure Pursuit）——用户可调四要素（必读）

控制在 `scripts/pure_pursuit_nav_node.py`；**改默认请优先改** `launch/nav_bringup.launch`（一键 `nav_all_in_one.launch` 会 include）。参数在节点**启动时**读取一次，改完需重启 nav。

> **返工根因（对照终端 1 / 2，非编造）**：
> 1. **激光双侧 STOP 把转向钉死**：日志反复 `Laser safety: STOP … v=0.00 ω=0.00`，`rear≈0.10` 时倒车被刹停，且前进逻辑也曾查后向扇区 → 走廊弯道误刹。
> 2. **降 v 拖垮 ω**：早期倒车版 `v=-0.15 → ω≈-0.23`（`ω=2v·sinα/L`），弯道几乎转不动；中间又曾把 `omega_max` 压到 ~1.8。
> 3. **卡住恢复被路径刷新打断 / STOP 后 ω=0**：长时间 `mode=REVERSE hold` 却无 `STUCK → RECOVER_*`。
>
> **现修**：`omega_max=4.0` + `omega_v_floor=0.55` + `omega_gain=1.35`；激光近障**禁止发全零**（自旋/换向打角）；**前进不查后向**；夹角倒车由 `allow_angle_reverse` 控制（默认开），`|α|>~130°` 进入（**不再强制** `x_b<0`）；`REVERSE`/`RECOVER_*` 时激光**不翻**线速度符号；路径刷新不重置卡住状态。

#### 四要素怎么调（文件 / 参数名 / 调大调小）

| # | 要素 | 参数名（`nav_bringup.launch` 与 `~` 私有名） | 当前默认 | 调大 | 调小 |
|---|------|---------------------------------------------|---------|------|------|
| **1** | **线速度基础/上限** | `max_linear_speed`、`min_linear_speed`、`reverse_linear_speed` / `reverse_speed_ratio` | **0.85 / 0.20 / 0.70** m/s | 直线更快 | 更稳 |
| **2** | **角速度上限** | `omega_max` | **4.0** rad/s | 尖弯更敢转 | 更柔（**勿再压到 ≤2.0**） |
| **3** | **线速度随曲率激进度** | `curvature_kappa_soft` / `hard` / `curvature_v_min_scale` | **1.0 / 2.5 / 0.50** | 弯道少降速 | 弯道更早降 `v` |
| **4** | **角速度随夹角激进度** | `omega_v_floor`、`omega_gain` | **0.55** m/s / **1.35** | 降 v 后仍敢转 | 更保守（易再现“转不动”） |

**公式**：`ω = omega_gain · 2 · v_eff · sin(α) / L`，`v_eff = max(|实际 v|, omega_v_floor)`，再饱和到 `±omega_max`；中大 `|α|` 还有最小 `|ω|` 地板。  
**不要**为了转快点把 `lookahead_distance` 改小（默认 **0.9**）；转不动优先调要素 **2** 和 **4**。

```bash
rosparam set /pure_pursuit_nav_node/omega_max 4.0
rosparam set /pure_pursuit_nav_node/omega_v_floor 0.55
rosparam set /pure_pursuit_nav_node/omega_gain 1.35
rosparam set /pure_pursuit_nav_node/max_linear_speed 0.75
# 改完必须重启 nav_bringup / nav_all_in_one
```

#### 倒车 / 卡住 / 激光

1. **Logic B 卡住自纠**：约 **5 s** 位移 < 0.08 m（含激光按住）→  
   - 前进卡住 → `RECOVER_REVERSE`（**必须** `linear.x < 0`）+ 按绿线打角  
   - 倒车卡住 → `RECOVER_FORWARD`（**必须** `linear.x > 0`）+ 打角  
   恢复时长默认 **5.0 s**（建议 4～6）；恢复期间激光**不得翻反**线速度符号。与 `allow_angle_reverse` **无关**（关夹角倒车时脱困仍可倒车）。
2. **Logic C 激光**：仅运动方向扇区；近障减速（**不削 ω**）、换向逃离打角；双侧近 → **自旋**，禁止 `v=ω=0`。`REVERSE` / `RECOVER_*` / `REORIENT` 时只减速、**不翻符号**。扇区最近距离**忽略** `≤ reorient_ignore_below` 的车体回波（现场曾 `rear≈0.10` → `Laser keep-dir` 把 `|v|` 从 0.70 砍到 **0.32**）。
3. **Logic A 夹角倒车**（`allow_angle_reverse:=true` 时）：绿线前视相对车头 `|α| > reverse_enter_angle`（默认 **~130°**）→ `mode=REVERSE` 且 `v<0`；`<~80°` 滞回退出。**不再要求** `x_b<0`（该条件已废弃）。开关关掉则非脱困只前进+转向掉头。
4. **Logic D 倒车回正锁定期**：进入夹角倒车或脱困倒车后，**先强制倒满约 `reorient_min_reverse_sec`（默认 5.0 s），此窗口内不做回正检测**；满时后再同时满足：前后 LaserScan 扇区最近距离够开阔、坐标位移 ≥ 阈值 → `mode=REORIENT`（前进）。**回正锁定期 `reorient_hold_sec`（默认 10.0 s）内禁止夹角倒车**（`allow_angle_reverse` 对进入暂时无效），只做前进+掉头；**仅当回正期间卡住且近障多**才允许脱困倒车。锁定期结束后切 `FORWARD`，但仍抑制夹角倒车，直到 `|α|` 回落到退出角以下或已前进 ≥ `reorient_unlock_disp`。**雷达开阔判定 = 前后扇区最小 `range`（不是点云数量）**；`≤ reorient_ignore_below` 视为车体回波忽略。**REORIENT 打方向**：绿线侧给路径偏好 + 左右雷达扇区最近距离给安全偏好；若一侧明显更空，且（对侧贴墙 **或** 绿线偏向更窄侧），**优先往开阔侧打**；两侧差不多则跟绿线。日志见 `REORIENT steer: path=… lidar L=… R=… → pick …`。
5. **Logic E 瞬时避障转向力**：激光看左前/右前（倒车时左后/右后）对角扇区；近障时在 `cmd_vel.ω` 上叠**小幅**远离偏置（**不改路径**）；障消失偏置归零。默认 `avoid_steer_omega_max=0.40`，故意小于主控，别抢纯追踪。

| 相关参数 | 默认 | 含义 |
|------|------|------|
| `allow_angle_reverse` | **true** | **1/true**：非脱困允许夹角倒车；**0/false**：仅脱困倒车 |
| `allow_reversing` | true | 总闸；false 时连脱困也不能倒车（只能拧） |
| `reverse_enter_angle` / `exit` | ≈**130°** / 80° | 夹角倒车滞回；想更早倒车把 enter **调小**（如 100°≈1.745 rad） |
| `reverse_speed_ratio` / `reverse_linear_speed` | **0.67 / 0.70** | **倒车目标 \|v\|** 看 `reverse_linear_speed`（launch 当前 0.70）；ratio 仅作未显式给绝对速度时的默认比例 |
| `reorient_enabled` | **true** | 倒车回正总开关 |
| `reorient_min_reverse_sec` | **5.0** | 强制倒车最短时长；未满绝不可因位移回正（建议 4～6） |
| `reorient_clear_front` / `rear` | **0.80 / 0.70** | 前后扇区最小 range「开阔」阈值（m）；调大更难回正 |
| `reorient_ignore_below` | **0.15** | 低于此距离视为车体回波，不挡回正 |
| `reorient_min_displacement` | **0.20** | 满强制倒车后，位移至少这么多才算真在动 |
| `reorient_hold_sec` | **10.0** | **回正锁定期**：强制前进+掉头、禁止夹角倒车；太短会倒车↔回正死循环 |
| `reorient_unlock_disp` | **0.30** | 锁定期结束后，须前进这么多（或 α 回正）才恢复夹角倒车 |
| `reorient_stuck_near` | **0.55** | 回正中脱困额外要求近障；调大更易触发脱困倒车 |
| `reorient_steer_lidar_enabled` | **true** | REORIENT 掉头是否用左右雷达覆盖绿线偏好 |
| `reorient_side_half_angle` | **0.70（~40°）** | 左右扇区半角（中心 ±90°） |
| `reorient_side_clear_margin` | **0.40** | 左右最近距离差 ≥ 此值才算「一侧明显更空」 |
| `reorient_side_near_wall` | **0.55** | 较近侧 < 此值视为贴墙（与「绿线偏窄侧」二选一即可覆盖） |
| `avoid_steer_enabled` | **true** | 瞬时避障转向力总开关 |
| `avoid_steer_near` / `far` | **0.50 / 0.90** | ≤near 满偏置，≥far 无偏置，中间线性 |
| `avoid_steer_omega_max` | **0.40** | ω 偏置上限（宜小）；蹭墙可略调大，抢转向则调小 |
| `avoid_steer_half_angle` | **0.52（~30°）** | 对角扇区半角（中心 ±45°/±135°） |
| `stuck_window_sec` | **5.0** | 卡住判定窗口（s） |
| `stuck_recover_duration` | **5.0** | 换向恢复持续时长（s，建议 4～6） |
| `stuck_cooldown_sec` | **1.5** | 恢复结束后冷却，防反复触发 |
| `stuck_steer_omega` | **1.8** | 脱困打角 |
| `laser_front_stop` / `slow` | **0.22 / 0.45** | 前向停车/减速（已收紧，减少误刹） |
| `laser_rear_stop` / `slow` | **0.20 / 0.40** | 仅倒车时启用 |

### 弯道不转弯 / 提前猛转（说明）

- **不转弯**：看日志 `ω` 是否接近 0；若出现 `STOP … ω=0.00` 则是旧激光钉死（本版应改为 `spin/creep`）。仍慢则提高要素 2/4。
- **提前猛转**：保持 `lookahead=0.9`；可略降 `omega_max` 到 3.0，**不要**压到 ≤2.0。

---

## 8.1 转向已通：摩擦与键盘 teleop 调参

**状态（已确认）**：Scout Mini Pro 为**差速驱动**；键盘 teleop 下用 **`j` / `l`** 可原地左右转。若转向“劲儿偏大”、希望略保守又不要太慢，优先调**前轮摩擦**与 teleop **速度上限**，导航侧另见上文 Pure Pursuit 参数。

### 摩擦力在哪里调

| 位置 | 文件 | 当前值 |
|------|------|--------|
| 前轮（被动轮） | `xacro/actuator/wheel.xacro` 中前左/前右调用 `mu1`/`mu2` | **0.15 / 0.15** |
| 后轮（驱动轮） | `xacro/macro/common_macros.xacro` 的 `wheel` 宏默认 | **mu1=1.0，mu2=0.8**（后轮未显式传参，走默认） |

- **前轮摩擦越高** → 转向越“涩/难转”；**越低** → 越灵活（过低可能发飘）。
- **保守建议区间（前轮）**：**0.15～0.3**。当前 **0.15** 比原先 0.1 略稳一点；若仍觉得转得过猛，可试 **0.2**；若又转不动，再降回 **0.1～0.15**。后轮一般保持默认高摩擦，保证驱动不打滑。
- **改完必须完全重启 Gazebo**：`mu` 写在 xacro 里，在 `gazebo_room.launch` 展开进 `robot_description` 后 spawn；**改摩擦无需 `catkin_make`**，但必须关掉当前 Gazebo / launch 再重新 `roslaunch ... gazebo_room.launch`，否则仍是旧模型。

### 键盘 teleop 怎么开、怎么调速

```bash
cd ~/catkin_ws_2 && source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/scout_mini_pro/cmd_vel
```

以本机 `teleop_twist_keyboard` 帮助为准（与终端打印一致）：

| 按键 | 作用 |
|------|------|
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 左转 / 右转（纯转可只按这两个） |
| `k` 或其它未绑定键 | 停止 |
| **`q` / `z`** | **同时**把线速度与角速度上限 **+10% / −10%** |
| **`w` / `x`** | **只**调线速度上限 **+10% / −10%** |
| **`e` / `c`** | **只**调角速度上限 **+10% / −10%** |

转向偏猛时：先连按几次 **`c`**（或 **`z`**）降低角速度上限，再试 `j`/`l`；觉得慢再 `e`/`q` 加回来。导航自动跑时请关掉 teleop，避免抢 `cmd_vel`。

### 验证导航贴墙/猛转是否改善（推荐步骤）

1. **完全重启 Gazebo**（摩擦或 xacro 有改时必须）：关旧窗口 → `roslaunch scout_mini_pro_description gazebo_room.launch`
2. `rosparam set use_sim_time true`
3. `roslaunch scout_mini_pro_description nav_bringup.launch`（或一键 `nav_all_in_one.launch`；确认日志 `omega_max=4.00`、`omega_v_floor=0.55` 一类输出）
4. 加载 amcl 参数并 `rosrun amcl amcl odom:=/scout_mini_pro/odom`
5. RViz：`rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room_slam.rviz`
6. **SetInitialPose** → **SetGoal**，观察：`/path` 是否离墙更远；转弯是否更平滑、少撞墙。

---

## 九、常见问题与排查


| 现象                                                                  | 可能原因                                       | 建议                                                                                                                                                                                                                                                                                                                                             |
| ------------------------------------------------------------------- | ------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| amcl 报 “Request for map failed” 或段错误                                | amcl 早于 map_server 启动，/map 未发布             | **先启动 nav_bringup（步骤 3），再启动 amcl（步骤 4）**；确认 `rostopic echo /map -n 1` 有数据后再运行 amcl。                                                                                                                                                                                                                                                            |
| amcl 粒子发散                                                           | 初始位姿偏差大、地图与场景不一致                           | 重新 SetInitialPose 设准；确认地图与 Gazebo 场景一致                                                                                                                                                                                                                                                                                                         |
| 小车不导航、不规划                                                           | 未设 SetInitialPose 或 amcl 未发布 map 帧         | 先 SetInitialPose；确认 amcl 在运行且 `odom:=/scout_mini_pro/odom`                                                                                                                                                                                                                                                                                     |
| A* 报 “no path found”、路径不出                                               | 障碍硬膨胀过大，房间门/窄通道被堵死                         | 默认膨胀为 **4** 格（硬）；节点会自动回退到 ≥2。若仍无路径，检查目标是否在自由区/地图连通；或把 `expansion_size` 改为 **3** 后重启 nav_bringup。路径贴墙可试 5（窄门可能需依赖自动回退）。                                                                                                                                                                                                                                          |
| SetGoal 后只有 RViz 有反应，A* 无 “received goal”、/path 无数据                 | 目标话题未接到 A* 节点                              | 在运行 nav_bringup 的终端应看到 `A* node started. Goal topic (remapped): /move_base_simple/goal`。执行 `rostopic info /move_base_simple/goal`：Publishers 应有 RViz，Subscribers 应有 **astar_nav_node**。若没有 astar_nav_node，说明 A* 未订阅到该话题，请确认用本包 `nav_bringup.launch` 启动且未改 remap。RViz 请使用 `display_room_slam.rviz`（SetGoal 的 Topic 为 `/move_base_simple/goal`）。 |
| /path 无数据、rostopic echo 报 “no messages… Is /clock being published?” | 未在 RViz 点 SetGoal，或 use_sim_time 下 echo 超时 | **必须先**在 RViz 点 **SetGoal**（设目标）；再在终端用 `rostopic hz /path` 看是否有频率。确认 Gazebo 在跑（发布 /clock）。                                                                                                                                                                                                                                                     |
| cmd_vel 无输出                                                         | 纯追踪未收到 /path 或 /amcl_pose                  | 确认已 SetGoal 且 `rostopic hz /path` 有输出；nav_bringup 与 amcl 在运行。                                                                                                                                                                                                                                                                                  |
| 弯道不转弯、一直往前冲（路径有弯但车直走）                                        | lookahead 选点或差速插件/仿真参数问题                      | 见第八节：已用**弧长**选 lookahead；可调 `lookahead_distance`/`omega_max`。若话题与 angular.z 都正常仍不转，用 **teleop `j`/`l`** 验证（见 8.1）。                                                                                                                                                                                                   |
| 路径贴墙 + 转弯过猛撞墙                                        | 硬膨胀未生效或 omega 过大                      | 确认 A* 为硬膨胀（默认 4）；`omega_max` 建议 **3.0～4.0**（默认 4.0）；近障靠激光；见第八节四要素。                                                                                                                                                                                                   |
| 弯道几乎转不动、角速度极保守                                  | 激光 STOP 清零、或降 v 拖垮 ω、或 omega_max 过低 | 确认启动日志 `omega_max≈4.0`、`omega_v_floor≈0.55`；日志应见 `spin/creep` 而非 `STOP … ω=0`；见第八节。                                                                                                                                                                                                   |
| 对着墙僵住、绿线在另一侧、不倒车不换向                        | 激光双侧 STOP + 卡住检测未生效；或恢复被激光翻符号                | 约 5s 应见 `STUCK → RECOVER_*`；`RECOVER_REVERSE` 时 **`v` 必须为负**（见第十二节）。                                                                                                                                                                                                   |
| 死胡同只会原地拧、不会倒车退出                                  | 激光 escape 盖掉倒车，或 `allow_reversing:=false` | 确认 `allow_reversing=True`；恢复窗口内不应出现 `Laser escape … v_sign=1` 配 `mode=RECOVER_REVERSE`；应见 `v<0` 或 `Laser recover-keep`。                                                                                                                                                                                                   |
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
- **已实现**：A* + 纯追踪导航、amcl 定位、map_server、`nav_bringup.launch`、**`nav_all_in_one.launch` 一键导航**、曲率调速 / 倒车自纠 / 激光防撞（见第十一节）。
- **手调参数**：见**第十二节**（想调 X → 改 Y 文件 / 参数 / 重启哪个 launch）。
- **可选后续**：编写路点脚本；按场景微调 A* 膨胀与 PP/激光参数。

本 README_3 为基于 A* + 纯追踪的导航操作手册；若某步与你的环境不一致，以实际路径与包名为准。

---

## 十一、一键导航启动与 Pure Pursuit 三大优化（2025-08）

### 11.1 一键启动（推荐）

原先室内导航需开 **4 个终端**（Gazebo、nav_bringup、amcl、RViz）。现已提供 **`nav_all_in_one.launch`**，一条命令拉起整条链路：

1. Gazebo room（含机器人 spawn，约 8 s 延迟）
2. `/use_sim_time=true`
3. `map_server` + A* + Pure Pursuit（`nav_bringup.launch`）
4. AMCL（`config/nav/amcl_params.yaml`；`odom` → `/scout_mini_pro/odom`；`scan` → `/scan`；默认延迟约 12 s，等地图与仿真就绪）
5. RViz（`display_room_slam.rviz`）

**每次新开终端只需 source 一次，然后一条 roslaunch：**

```bash
cd ~/catkin_ws_2
source devel/setup.bash
roslaunch scout_mini_pro_description nav_all_in_one.launch
```

- **不要**把 `catkin_make` / `source` 写进 launch（launch 无法做 shell source）。
- 终端内 **Ctrl+C** 会结束整棵 launch 树（Gazebo / 导航 / amcl / RViz）。
- 可选参数：`amcl_delay:=12`、`rviz_delay:=3`、`map_file:=...`。
- 启动后操作与分步相同：RViz 中先 **SetInitialPose**，再 **SetGoal**。若刚起来就设目标，可等 AMCL 延迟结束后再操作（约十几秒）。

### 11.2 附录：旧四终端分步流程（调试用）

排查单节点问题时仍可用分步启动（细节见第五节）：

| 顺序 | 命令 |
|------|------|
| 1 | `roslaunch scout_mini_pro_description gazebo_room.launch` |
| 2 | `rosparam set use_sim_time true` |
| 3 | `roslaunch scout_mini_pro_description nav_bringup.launch`（须先于 amcl） |
| 4 | `rosparam load $(rospack find scout_mini_pro_description)/config/nav/amcl_params.yaml /amcl` 再 `rosrun amcl amcl odom:=/scout_mini_pro/odom` |
| 5 | `rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room_slam.rviz` |

### 11.3 Pure Pursuit 行为说明（返工后）

实现：`scripts/pure_pursuit_nav_node.py`；默认：`launch/nav_bringup.launch`。**手调四要素详见第八节**。

| 逻辑 | 行为 | 日志关键字 |
|------|------|------------|
| A 夹角倒车 | `allow_angle_reverse=true` 且 `\|α\|>~130°` → `mode=REVERSE`，`v<0`（**不强制** `x_b<0`） | `mode=REVERSE` / `Angle reverse ENTER` |
| B 卡住自纠（优先） | ~5s 位移过小 → 前进卡则倒车(`v<0`)、倒车卡则前进(`v>0`)，持续 `stuck_recover_duration`（默认 **5s**）+ 打角；恢复中激光不翻符号 | `STUCK → RECOVER_*`，且 `v` 符号正确 |
| C 激光防撞 | 前进只查前向；近障减速不削 ω；双侧近 → **自旋**，禁止全零；**不覆盖 REVERSE/RECOVER/REORIENT 换向** | `Laser escape` / `spin/creep` / `Laser keep-dir` |
| D 倒车回正 | 强制倒满 ~**5s** 后再查扇区最小 range 开阔 + 位移 → `REORIENT` 前进 | `REORIENT check` / `REORIENT: … → FORWARD` |

角速度：`ω = omega_gain·2·v_eff·sin(α)/L`，`v_eff=max(\|v\|, omega_v_floor)`，上限 `omega_max`（默认 **4.0**）。

### 11.4 参数速查（与 launch 对齐）

| 参数 | 默认 | 属于哪一要素 / 含义 |
|------|------|---------------------|
| `max_linear_speed` / `min_linear_speed` / `reverse_linear_speed` | **0.85 / 0.20 / 0.70** | **要素1**；倒车目标看 `reverse_linear_speed`（`reverse_speed_ratio=0.67` 仅作缺省比例） |
| `omega_max` | **4.0** | **要素2** 角速度上限 |
| `curvature_kappa_soft` / `_hard` / `curvature_v_min_scale` | **1.0 / 2.5 / 0.50** | **要素3** 线速度随曲率 |
| `omega_v_floor` / `omega_gain` | **0.55 / 1.35** | **要素4** 角速度激进度 |
| `stuck_window_sec` / `stuck_recover_duration` | **5.0 / 5.0** | 卡住窗口 / 换向恢复时长 |
| `reorient_min_reverse_sec` / `clear_front` / `rear` / `min_displacement` | **5.0 / 0.80 / 0.70 / 0.20** | Logic D 强制倒车时长 / 扇区开阔 / 位移（见 **12.4.0**） |
| `stuck_steer_omega` / `laser_escape_omega` | **1.8 / 1.6** | 脱困/逃离打角 |
| `laser_front_stop` / `slow` | **0.22 / 0.45** | 前向（已收紧） |

改完重启 `nav_bringup` 或整棵 `nav_all_in_one`。

### 11.5 用户验证步骤

1. `source ~/catkin_ws_2/devel/setup.bash`
2. `roslaunch scout_mini_pro_description nav_all_in_one.launch`
3. 确认启动日志含 `omega_max=4.00`、`omega_v_floor=0.55`
4. RViz：**SetInitialPose** → **SetGoal**
5. 弯道日志 `ω` 应明显非零（常见 >0.8）；不应再长期 `v=0.00 ω=0.00`
6. 死胡同卡住约 5s 后应见 `STUCK → RECOVER_*`；`RECOVER_REVERSE` 时日志 **`v` 必须为负**（不应再出现 `Laser escape … v_sign=1` 盖掉倒车）
7. Ctrl+C 退出整棵 launch

---

## 十二、手调参数一页指南（想调 X → 改 Y）

> **给现场自己调参用**：改完少问、少烧 token。  
> **重启规则（必记）**：
> - 改 `nav_bringup.launch` / PP / A* 参数 → 重启 **`nav_all_in_one.launch`**（或至少停掉再启 `nav_bringup`）。节点只在启动时读 `~` 参数，`rosparam set`  alone **不会**改行为。
> - 改 `xacro` 车轮摩擦 → **必须完全重启 Gazebo**（`gazebo_room` / 整棵 `nav_all_in_one`）。
> - 改地图 `maps/room.*` → 重启 map_server（随 nav 一起重启即可）。

### 12.1 文件地图（各文件干什么）

> **完整目录树、每个 launch/脚本/xacro 说明、官方包关系**：见独立文档 **[FILE_STRUCTURE.md](FILE_STRUCTURE.md)**，避免与此处简表两处维护矛盾。下文仅保留导航调参最常用的锚点。

| 文件 | 作用 | 改它通常要重启什么 |
|------|------|-------------------|
| `launch/nav_all_in_one.launch` | **一键**：Gazebo + map + A* + PP + AMCL + RViz | 整棵 launch |
| **`launch/nav_bringup.launch`** | **★ 导航核心参数所在**：速度/倒车/回正/膨胀等默认几乎都在这 | `nav_all_in_one` 或 `nav_bringup` |
| `launch/gazebo_room.launch` | 只起仿真房间 + 车（不含导航） | 仅 Gazebo |
| `scripts/astar_nav_node.py` / `pure_pursuit_nav_node.py` | 规划与跟踪实现；**调默认优先改 launch** | 改代码后重启 nav |
| `config/nav/amcl_params.yaml` | AMCL；同目录 costmap 等为**旧 move_base 残留，本栈不用** | 重启 amcl / all_in_one |
| `maps/room.*`、`xacro/actuator/wheel.xacro` | 地图 / 车轮摩擦 | map_server / **Gazebo** |
| [FILE_STRUCTURE.md](FILE_STRUCTURE.md) | 全工作空间文件架构与「改 X→Y」速查 | — |

### 12.2 想调「线速度 / 角速度激进程度」

**文件**：`launch/nav_bringup.launch` → `<node name="pure_pursuit_nav_node">` 段。

| 参数 | 默认 | 调大效果 | 调小效果 | 备注 |
|------|------|----------|----------|------|
| `max_linear_speed` | 0.85 | 直线更快 | 更稳、少撞 | 要素1 |
| `min_linear_speed` | 0.20 | 弯道保底更快 | 弯道更慢 | 过小易拖垮机动 |
| `reverse_speed_ratio` | **0.67** | 倒车相对 vmax 更快 | 更慢 | 仅当未设绝对 `reverse_linear_speed` 时作默认 |
| `reverse_linear_speed` | **0.70** | 倒车更快 | 倒车更慢 | **调倒车速度改这个**；卡住倒车也受此影响 |
| `omega_max` | **4.0** | 尖弯更敢转 | 更柔 | **勿压到 ≤2.0**（曾转不动） |
| `omega_v_floor` | **0.55** | 降 v 后仍敢转 | 更保守 | 要素4；转不动优先加大它或 `omega_gain` |
| `omega_gain` | **1.35** | ω 整体放大 | 更柔 | 公式见下 |
| `lookahead_distance` | 0.9 | 更早对准远点 | 更贴路径、可能抖 | **不要为了“更转”而盲目改小** |
| `curvature_kappa_soft` | 1.0 | 更晚开始因曲率降 v | 更早降 v | 要素3；**倒车模式跳过激进夹角降速** |
| `curvature_kappa_hard` | 2.5 | 硬曲率门槛更高 | 更早到最低速 | 与 soft 搭配 |
| `curvature_v_min_scale` | 0.50 | 弯道最低速比例更高 | 弯道更慢 | 相对 `vmax`；倒车保底不低于 0.85 |

**公式**：`ω = omega_gain · 2 · v_eff · sin(α) / L`，`v_eff = max(|v|, omega_v_floor)`，再夹到 `±omega_max`。

**快捷对照**：

| 你的感觉 | 先改谁 |
|----------|--------|
| 直线太慢/太快 | `max_linear_speed` |
| 弯道转不动、ω 日志很小 | `omega_max`↑、`omega_v_floor`↑、`omega_gain`↑ |
| 弯道冲太猛、贴墙扫尾 | `curvature_v_min_scale`↓ 或 `max_linear_speed`↓；路径贴墙则先看 12.3 |
| 倒车段太猛/太怂 | `reverse_linear_speed`（**优先**）或 `reverse_speed_ratio`；日志看 `PP … mode=REVERSE v=` |
| 倒车后不回正 / 一直倒 / 过早回正 | `reorient_*`（见 **12.4.0**） |

改完 → **重启 `nav_all_in_one`（或 nav_bringup）**。

### 12.3 想调「安全域 / 膨胀（绿线离墙远近）」

**文件**：`launch/nav_bringup.launch` → `<node name="astar_nav_node">`。

| 参数 | 默认 | 单位 | 调大 | 调小 | 说明 |
|------|------|------|------|------|------|
| `expansion_size` | **4** | 格（约 0.05 m/格 → 0.20 m） | 绿线更离墙；门口易 `no path` | 路径更贴墙 | **硬膨胀**：≤该格距障碍不可通行 |
| `safety_zone_radius` | **10** | 格 | 宽走廊更走中间 | 软代价范围变窄 | 软代价，不堵死通道 |
| `safety_zone_weight` | **1.0** | — | 更厌恶贴墙 | 更敢贴边走 | 设 `0` 约等于关软代价 |

**推荐**：膨胀 **3～5**；room 门口约 0.45 m，硬膨胀 **5** 可能断连通。  
改完 → 重启 nav；看 A* 日志是否仍 `expansion_size: 4 (hard)`，RViz 绿线是否离墙。

### 12.4 想调「倒车 / 停车 / 卡住恢复 / 激光防撞」

**参数集中位置（勿散落别处）**：

- 文件：`src/scout_mini_pro_description/launch/nav_bringup.launch`
- 节点：`<node … name="pure_pursuit_nav_node">`（倒车/回正/卡住/激光全部在此段）
- A* 安全域：同文件 `<node … name="astar_nav_node">`
- 改完 → **重启 `nav_all_in_one.launch`，或至少重启 `nav_bringup.launch`**（只改 rosparam 不重启节点不生效）

| 目标 | 参数 | 默认 | 怎么调 |
|------|------|------|--------|
| **非脱困是否允许夹角倒车** | `allow_angle_reverse` | **true** | **true/1**：`|α|` 超阈值 → `REVERSE`；**false/0**：**仅脱困**才倒车，平时只前进+转向掉头 |
| 更晚才进入夹角倒车 | `reverse_enter_angle` | **2.269（~130°）** | 再**调大**更难进 REVERSE；**想更早倒车调小**（如 `1.745`≈100°） |
| 更早退出倒车 | `reverse_exit_angle` | 1.396（~80°） | 调大 → 更大角度才退出（滞回） |
| 倒车总闸（含脱困） | `allow_reversing` | true | false 时卡住也只能原地拧，无任何倒车 |
| **倒车相对 vmax 比例** | `reverse_speed_ratio` | **0.67（≈2/3）** | 未显式给绝对速度时的默认；有 `reverse_linear_speed` 时以绝对值为准 |
| **倒车绝对线速度** | `reverse_linear_speed` | **0.70** | **调倒车速度改这个**；日志 `PP mode=REVERSE v=` 应接近 `-该值` |
| **倒车回正开关** | `reorient_enabled` | **true** | false=关掉回正，倒车只靠夹角滞回/脱困结束退出 |
| **强制倒多久再检测回正** | `reorient_min_reverse_sec` | **5.0** | 见下方 **12.4.0**；建议 **4～6**；未满绝不可因位移回正 |
| **前后雷达开阔阈值** | `reorient_clear_front` / `rear` | **0.80 / 0.70** | 见 12.4.0；**调大=更严更晚回正**，调小=更松更早回正 |
| **忽略车体回波** | `reorient_ignore_below` | **0.15** | rear≈0.10 不挡回正，也**不参与**倒车激光降速；勿调得太大以免真贴墙也判开阔 |
| **回正所需位移** | `reorient_min_displacement` | **0.20** | 满强制倒车后才生效；调大 → 必须多倒一段才回正 |
| **回正锁定期（强制前进）** | `reorient_hold_sec` | **10.0** | `REORIENT` 期间禁止夹角倒车；**太短会倒车↔回正死循环**；建议 8～12 |
| **锁后恢复夹角倒车的位移** | `reorient_unlock_disp` | **0.30** | 锁定期结束后还须前进这么多（或 α 回正）才允许夹角倒车 |
| **回正中脱困近障阈值** | `reorient_stuck_near` | **0.55** | 回正卡住还要近障才脱困倒车；调大更易触发 |
| **REORIENT 掉头看雷达** | `reorient_steer_lidar_enabled` | **true** | false=只跟绿线侧打方向（旧行为） |
| **左右「明显更空」差** | `reorient_side_clear_margin` | **0.40** | 调大 → 更难触发雷达覆盖（更信绿线）；调小 → 更容易往开阔侧拧 |
| **对侧「贴墙」阈值** | `reorient_side_near_wall` | **0.55** | 调大 → 稍远也算贴墙、更易覆盖绿线；调小 → 只有很近才覆盖 |
| **左右扇区半角** | `reorient_side_half_angle` | **0.70**(~40°) | 调大 → 左右扇区更宽；调小 → 更窄、更侧向 |
| **瞬时避障转向力开关** | `avoid_steer_enabled` | **true** | false=关掉 ω 偏置（只靠膨胀/激光减速） |
| **避障偏置满/零距离** | `avoid_steer_near` / `far` | **0.50 / 0.90** | 调大 near → 更远就开始推开；far 应 > near |
| **避障 ω 偏置上限** | `avoid_steer_omega_max` | **0.40** | **调大**更敢躲墙（别太大抢主控）；**调小**更贴纯追踪 |
| **避障对角扇区半角** | `avoid_steer_half_angle` | **0.52**(~30°) | 调大扇区更宽；调小更聚焦对角 |
| 多久算卡住 | `stuck_window_sec` | 5.0 | 调小更敏感；调大更迟钝 |
| 位移小于多少算卡住 | `stuck_dist_threshold` | 0.08 m | 调大更容易判卡住 |
| **恢复倒车/前进持续多久** | `stuck_recover_duration` | **5.0** s | 建议 **4～6**；2.5 往往蹭不出去 |
| 恢复后冷却 | `stuck_cooldown_sec` | 1.5 | 防抖 |
| 脱困打角强度 | `stuck_steer_omega` | 1.8 | 调大拧得更狠 |
| 前向几乎停车距离 | `laser_front_stop` | 0.22 | 调大更早刹；过大会误刹 |
| 前向开始减速 | `laser_front_slow` | 0.45 | 应 **> stop** |
| 后向（仅倒车时） | `laser_rear_stop` / `slow` | 0.20 / 0.40 | 前进**不查**后向 |
| 激光短逃逸速度/时长/打角 | `laser_escape_speed` / `duration` / `omega` | 0.32 / 1.4 / 1.6 | 普通近障换向用；**不会覆盖 REVERSE/RECOVER/REORIENT 符号** |

#### 12.4.0 倒车回正（REORIENT）手调专节

**改哪里**：`launch/nav_bringup.launch` → `pure_pursuit_nav_node` 段内全部 `reorient_*`（与夹角倒车、脱困参数同文件同节点，不要改到别的 launch）。

**正确逻辑（当前）**：

1. 进入 `REVERSE`（夹角）或 `RECOVER_REVERSE`（脱困）后 → 日志 `REORIENT arm: … force reverse 5.0s`
2. **先强制倒满 `reorient_min_reverse_sec`（默认 5.0 s）**：此窗口内**不做**开阔/位移回正判定（日志 `pass=False (wait min=…)`）
3. 满时后才同时检测：
   - **雷达开阔**：前后 LaserScan **扇区最近 `range`**（不是点云数量）≥ `reorient_clear_*`；`≤ reorient_ignore_below` 视为车体回波忽略
   - **位移**：相对倒车段起点 ≥ `reorient_min_displacement`
4. 两项都过 → `mode=REORIENT`（前进）进入**锁定期** `reorient_hold_sec`（默认 **10.0 s**）：
   - **禁止**因绿线夹角进入倒车（`allow_angle_reverse` 暂时无效）
   - 正常前进 + 雷达择侧掉头
   - **仅**当坐标几乎不变 **且** 近障多（`reorient_stuck_near`）→ `STUCK → RECOVER_REVERSE`
5. 锁定期结束 → `FORWARD`，但仍见 `angle-reverse still locked`；须 `|α|` 回正或前进 ≥ `reorient_unlock_disp` → `REORIENT unlock: angle-reverse restored`
6. **REORIENT 打方向权衡**（`reorient_steer_lidar_enabled=true` 时）：
   - **绿线偏好**：前视点相对车头 α 的符号 → L（α>0）/ R（α<0）
   - **雷达偏好**：±90° 附近左右扇区 **最近 range**（`L` / `R`）；无回波当开阔
   - **覆盖规则**：若 `|L−R| ≥ reorient_side_clear_margin`，且（较近侧 `< reorient_side_near_wall` **或** 绿线偏向更窄一侧）→ **往开阔侧打**，即使绿线略偏墙侧
   - **否则**（两侧差不多）→ **跟绿线**

| 我想… | 改哪个参数 | 默认 | 调大 | 调小 |
|-------|------------|------|------|------|
| **强制倒车多久再开始检测** | `reorient_min_reverse_sec` | **5.0** s | 倒更久才允许回正（给够驶出空间） | 更早开始检测（易刚倒就切前进） |
| **雷达开阔判定松紧（前）** | `reorient_clear_front` | **0.80** m | **更严、更晚回正**（要更空） | **更松、更早回正** |
| **雷达开阔判定松紧（后）** | `reorient_clear_rear` | **0.70** m | 同上 | 同上 |
| **忽略多近的车体回波** | `reorient_ignore_below` | **0.15** m | 更多近距离被当回波忽略 | 更少忽略（贴车体易挡回正） |
| **位移门槛** | `reorient_min_displacement` | **0.20** m | 必须多倒一段才回正 | 位移稍变就可能过门槛（仍须满 5s） |
| **回正锁定期多久** | `reorient_hold_sec` | **10.0** s | 更长禁止夹角倒车（防死循环） | 太短易立刻又进 REVERSE |
| **锁后须前进多少才恢复夹角倒车** | `reorient_unlock_disp` | **0.30** m | 更晚恢复夹角倒车 | 更早恢复（易再进倒车） |
| **回正中多近才算「近障」可脱困** | `reorient_stuck_near` | **0.55** m | 稍远也算近障、更易脱困倒车 | 只有很近才允许脱困 |
| **关掉回正** | `reorient_enabled` | **true** | — | 设 `false` |
| **REORIENT 掉头是否看左右雷达** | `reorient_steer_lidar_enabled` | **true** | — | 设 `false` 只跟绿线（旧行为） |
| **左右差多大才算「明显更空」** | `reorient_side_clear_margin` | **0.40** m | 更难触发雷达覆盖（更信绿线） | 更容易往开阔侧拧 |
| **对侧多近才算「贴墙」** | `reorient_side_near_wall` | **0.55** m | 稍远也算贴墙、更易覆盖绿线 | 只有很近才覆盖绿线 |
| **左右扇区半角** | `reorient_side_half_angle` | **0.70** rad (~40°) | 扇区更宽 | 扇区更窄、更偏正侧向 |

**如何从日志看雷达判定**（扇区最小 range，不是点云数）：

```text
REORIENT arm: reverse started, force reverse 5.0s before check
REORIENT check: t=2.1s front=0.55 rear=0.10 disp=0.08 pass=False (wait min=5.0s; no reorient yet)
REORIENT check: t=5.2s front=0.82(ok=True thr=0.80) rear=0.70(ok=True thr=0.70) disp=0.25(ok=True thr=0.20) pass=True
REORIENT: reverse reorient → FORWARD (t=5.2s front=0.82 rear=0.70 disp=0.25 hold=10.0s lock)
REORIENT steer: path=L lidar L=0.30 R=0.90 → pick R
REORIENT done → FORWARD (angle-reverse still locked until α<80deg or disp≥0.30m)
REORIENT unlock: angle-reverse restored (disp_ok)
```

| 字段 | 含义 |
|------|------|
| `t=` | 本段倒车已持续秒数 |
| `front=` / `rear=` | 前/后向扇区 **最近有效 range（m）**；无效为 `-1` |
| `ok=` | 该侧是否判为开阔（相对 `thr=`） |
| `thr=` | 当前 `reorient_clear_front` / `rear` |
| `disp=` / `thr=` | 倒车段位移 / `reorient_min_displacement` |
| `pass=` | 是否允许进入 `REORIENT`；未满强制倒车时恒为 `False` |
| `hold=… lock` | 进入回正锁定期时长 |
| `REORIENT steer: path=` | 绿线偏好侧 `L`/`R`/`-` |
| `lidar L=` / `R=` | 左右扇区最近 range（m）；无效 `-1` |
| `→ pick` | **最终掉头侧**；贴墙场景常见 `path=L lidar L=0.30 R=0.90 → pick R` |
| `still locked` / `unlock` | 锁后抑制夹角倒车 / 已恢复夹角倒车检测 |

#### 12.4.0b 瞬时避障转向力（AvoidSteer）手调

**改哪里**：同文件 `pure_pursuit_nav_node` 段 `avoid_steer_*`。

**逻辑**：不改 A*/绿线；用对角扇区（左前/右前，倒车用左后/右后）最近 range；在最终 `cmd_vel.ω` 上叠远离偏置。左侧近 → ω 偏负（右转），右侧近 → ω 偏正；障离开 `avoid_steer_far` 后偏置为 0。

| 我想… | 改哪个 | 默认 | 调大 | 调小 |
|-------|--------|------|------|------|
| **躲墙更积极** | `avoid_steer_omega_max` | **0.40** | 偏置更大（勿超过 ~0.8，易抢主控） | 更弱、更贴绿线 |
| **更早开始躲** | `avoid_steer_far` / `near` | 0.90 / 0.50 | 更远就推开 | 更贴才推 |
| **关掉** | `avoid_steer_enabled` | true | — | `false` |

日志关键字：`AvoidSteer front L=… R=… → bias=±…`；主行 `PP … avoid=±… lock=0/1`。

主行 `PP … front=… rear=…` 里的 front/rear 与上表同源（扇区最小距离）。  
改完 → 重启 **`nav_all_in_one`** 或至少 **`nav_bringup`**。

**夹角倒车开关用法（改完重启 nav）**：

```bash
# launch 里改，或（仅当次）写入后再重启节点才生效：
# <param name="allow_angle_reverse" value="true" />   <!-- 试验夹角倒车：默认开 -->
# <param name="allow_angle_reverse" value="false" />  <!-- 只要脱困倒车 -->
# <param name="reverse_enter_angle" value="2.269" /> <!-- 130°；更早倒车可改 1.745≈100° -->
```

**卡住恢复 / 倒车回正逻辑（当前）**：

| 当前模式 | 事件 | 必须看到的 cmd / 日志 |
|----------|------|----------------------|
| 前进 `FORWARD` | 卡住 → `RECOVER_REVERSE` | `linear.x < 0`，并有打角 |
| 倒车 `REVERSE` | 卡住 → `RECOVER_FORWARD` | `linear.x > 0`，并有打角 |
| `REVERSE` / `RECOVER_REVERSE` | 强制倒满 ≥**5.0s** + 扇区开阔 + 位移 → `REORIENT` | `v > 0`；先见 `REORIENT check`，通过后 `REORIENT: … → FORWARD … hold=10.0s lock` |
| `REORIENT` | 锁定期内禁止夹角倒车；卡住+近障 → `RECOVER_REVERSE`；满 hold → `FORWARD`（仍 lock） | `REORIENT done → FORWARD (angle-reverse still locked…)` 或 `REORIENT stuck+near` |
| `FORWARD` + lock | α 回正或位移够 → 恢复夹角倒车 | `REORIENT unlock: angle-reverse restored` |

日志应类似：`STUCK → RECOVER_REVERSE 5.0s …`，随后 `PP … mode=RECOVER_REVERSE v=-0.xx`（**负号**）。夹角倒车成功时应见 `mode=REVERSE` 且 `v<0`，**|v| 接近你设的 `reverse_linear_speed`（当前 launch 为 0.70）**——若只见 `|v|≈0.32` 且伴 `Laser keep-dir … dist=0.10`，多半是车体回波误减速（应已忽略 `≤reorient_ignore_below`）。可伴 `Angle reverse ENTER`。倒车开始应见 `REORIENT arm`；未满 5s 应见 `REORIENT check … pass=False (wait min=…)`；开阔处倒满约 5 s 且位移够后应见 **`pass=True`** 与 **`REORIENT: … hold=10.0s lock`**、`mode=REORIENT`（`v>0`）。回正约 10 s 内**不应**立刻再出现 `Angle reverse ENTER`；满 hold 后见 `still locked`，前进一段或 α 回正后见 `REORIENT unlock`。若 `mode=REVERSE` 但 `v>0`，说明旧节点未重启或激光仍在翻符号。

### 12.4.1 PP 终端日志字段说明（必读）

主行（约 1 Hz throttle）形如：

```text
PP α=3.12(179°) x_b=-0.94 κ=0.04 mode=REVERSE v=-0.70 ω=-0.03 front=2.45 rear=-1.00 hold=0 avoid=+0.00 lock=0
```

| 字段 | 含义 | 怎么读 |
|------|------|--------|
| **α** | 前视点相对**车头**的方位角（弧度），即纯追踪转向角；括号内为度数 | `α≈±π`（~±180°）表示目标几乎在正后方；`|α|` 超过 `reverse_enter_angle`（默认 ~130°）才应进夹角倒车 |
| **x_b** | 前视点在车体坐标系下的纵向位置（m）：**>0 在车头前方，<0 在车尾后方** | **仅作参考/日志**；夹角倒车与回正**均不强制** `x_b<0`（该条件已废弃） |
| **κ** | 近似曲率 `\|2·sin(α_eff)/L\|`（1/m） | 越大弯越急；**倒车时曲率降速更温和**（不低于约 85%×`reverse_linear_speed`） |
| **mode** | 驱动状态机 | 见下表 |
| **v** | 实际下发的 `cmd_vel.linear.x`（m/s） | **验证倒车速度看这个**：`REVERSE`/`RECOVER_REVERSE` 时期望约 **`-reverse_linear_speed`**（如 -0.70）；若 `|v|≈0.32` 且非真实后向近障，属异常 |
| **ω** | 实际下发的 `cmd_vel.angular.z`（rad/s） | 转向是否够猛主要看这个；含瞬时避障偏置；近 0 且 α 大则“转不动” |
| **front** | 前向 LaserScan **扇区最近 range**（m）；无效/仅车体回波时为 `-1` | 防撞比 `laser_front_stop/slow`；回正比 `reorient_clear_front`（**不是点云数量**） |
| **rear** | 后向扇区最近 range（m）；无效/仅车体回波时为 `-1` | **已忽略 ≤`reorient_ignore_below` 回波**；有真实后向障碍时才应触发倒车降速 |
| **hold** | 是否被激光按住几乎无前进意图（1/0） | `1` 时仍会计入卡住窗口，便于触发脱困 |
| **avoid** | 瞬时避障 ω 偏置（rad/s） | 贴墙时应非 0；开阔时 ≈0 |
| **lock** | 回正锁 / 锁后抑制夹角倒车（1/0） | `1` 时不应出现 `Angle reverse ENTER` |

**mode 取值**：

| mode | 含义 | 期望 v |
|------|------|--------|
| `FORWARD` | 正常前进跟线 | `v > 0` |
| `REVERSE` | 夹角倒车（需 `allow_angle_reverse=true`） | **`v < 0`**，\|v\|≈`reverse_linear_speed`（无真实后向近障时） |
| `RECOVER_REVERSE` | 前进卡住后的强制倒车脱困 | **`v < 0`** |
| `RECOVER_FORWARD` | 倒车卡住后的强制前进脱困 | **`v > 0`** |
| `REORIENT` | 倒车回正锁定期：强制前进、禁止夹角倒车 | **`v > 0`** |

**其它常见日志行**：

| 日志关键字 | 含义 |
|------------|------|
| `Angle reverse ENTER \|α\|=…` | 刚因夹角进入倒车 |
| `STUCK → RECOVER_REVERSE/FORWARD …` | 卡住检测触发，开始脱困换向 |
| `Pure pursuit recover done → …` | 脱困时长结束，回到 `FORWARD`/`REVERSE` |
| `REORIENT arm: … force reverse Xs` | 进入倒车段，开始强制倒车计时（默认 5s） |
| `REORIENT check: t=… front=… rear=… disp=… pass=…` | **回正检测行**：扇区最近距离/位移是否过门槛；未满强制时长恒 `pass=False` |
| `REORIENT: reverse reorient → FORWARD … hold=… lock` | **倒车回正触发**：进入锁定期 |
| `REORIENT steer: path=… lidar L=… R=… → pick …` | **回正掉头侧**：绿线偏好 / 左右雷达距离 / 最终选择（可贴墙覆盖绿线） |
| `REORIENT done → FORWARD (angle-reverse still locked…)` | 锁定期结束，仍抑制夹角倒车 |
| `REORIENT unlock: angle-reverse restored` | 已正常前进 / α 回正 → 恢复夹角倒车 |
| `REORIENT stuck+near → allow recover reverse` | 回正中卡住且近障 → 允许脱困倒车 |
| `AvoidSteer … → bias=±…` | 瞬时避障转向偏置 |
| `Laser keep-dir: mode=… v_sign=±1` | 激光在意图方向上只减速、保住符号（倒车时 `v_sign=-1`） |
| `Laser escape: … v_sign=±1` | 普通近障短时换向逃离（**不应**出现在 `REVERSE`/`RECOVER_*`/`REORIENT` 窗口里盖掉意图方向） |
| `Laser slow v_scale=…` | 近障按比例降线速度，ω 保留 |
| `Laser blocked → spin/creep` | 双侧都近：原地拧或蠕动，禁止全零 |
| `Pure pursuit: path refresh … (state kept)` | A* 刷新路径，**保留**卡住/倒车状态 |
| `Pure pursuit: new goal path …` | 新目标，状态机复位 |

**如何从日志确认倒车 / 回正生效**：找 `mode=REVERSE` 或 `mode=RECOVER_REVERSE`，同一行 **`v` 为负**且 |v| 接近你设的 **`reverse_linear_speed`（当前 0.70）**；无真实后向近障时不应再出现 `Laser keep-dir … |v|=0.32 dist=0.10`。应先见 `REORIENT arm`，未满 5s 见 `REORIENT check … pass=False (wait min=…)`；开阔处倒满约 **5 s** 且 `front/rear/disp` 都 `ok` 后应出现 **`pass=True`**、`REORIENT: … hold=10.0s lock` 与 `mode=REORIENT`（`v>0`，`lock=1`）。约 10 s 内不应立刻 `Angle reverse ENTER`；满 hold 后见 `still locked`，再前进/α 回正见 `REORIENT unlock`。贴墙时应见 `AvoidSteer` / `avoid=±…`。夹角倒车还可看 `α(…°)` 是否超过进入角，以及是否出现 `Angle reverse ENTER`。

### 12.5 「我想调 X」速查表

| 我想… | 去哪个文件 | 改哪个参数 | 重启 |
|-------|------------|------------|------|
| 直线更快/更慢 | `nav_bringup.launch` | `max_linear_speed` | nav / all_in_one |
| **倒车更快/更慢** | 同上 → `pure_pursuit_nav_node` | **`reverse_linear_speed`**（验证：日志 `PP … mode=REVERSE v=`） | 同上 |
| **强制倒更久/更短再检测回正** | `nav_bringup.launch` → `pure_pursuit_nav_node` | `reorient_min_reverse_sec`（默认 **5.0**） | nav / all_in_one |
| **回正锁定期更长/更短** | 同上 | `reorient_hold_sec`（默认 **10.0**） | 同上 |
| **锁后更晚/更早恢复夹角倒车** | 同上 | `reorient_unlock_disp`（默认 **0.30**） | 同上 |
| **贴墙时 ω 偏置更强/更弱** | 同上 | `avoid_steer_omega_max`（默认 **0.40**） | 同上 |
| **雷达开阔更严/更松（更晚/更早回正）** | 同上 | `reorient_clear_front` / `rear`（默认 **0.80 / 0.70**） | 同上 |
| **回正位移门槛** | 同上 | `reorient_min_displacement`（默认 **0.20**） | 同上 |
| **关倒车回正** | 同上 | `reorient_enabled:=false` | 同上 |
| **回正掉头更敢往开阔侧** | 同上 | 降 `reorient_side_clear_margin` 或升 `reorient_side_near_wall` | 同上 |
| **回正掉头更跟绿线** | 同上 | 升 `margin` / 降 `near_wall`，或 `reorient_steer_lidar_enabled:=false` | 同上 |
| 弯道转得更过 | 同上 | `omega_max`、`omega_v_floor`、`omega_gain` | 同上 |
| 弯道少冲 | 同上 | `curvature_*` 或降 `max_linear_speed` | 同上 |
| 绿线离墙远一点 | 同上（astar 节点） | `expansion_size`↑、`safety_zone_*` | 同上 |
| 门口规划不出路 | 同上 | `expansion_size`↓ 到 3 | 同上 |
| **开/关夹角倒车** | 同上（pp 节点） | **`allow_angle_reverse`** | 同上 |
| **更早/更晚夹角倒车** | 同上 | `reverse_enter_angle`（调小更早，如 100°） | 同上 |
| 卡住倒车时间更长 | 同上（pp 节点） | `stuck_recover_duration`（4～6） | 同上 |
| 更早判定卡住 | 同上 | `stuck_window_sec`↓ 或 `stuck_dist_threshold`↑ | 同上 |
| 前向更早刹车 | 同上 | `laser_front_stop` / `slow`↑ | 同上 |
| 键盘/仿真转向涩度 | `xacro/actuator/wheel.xacro` | 前轮 `mu1`/`mu2` | **Gazebo** |
| 换地图 | `maps/` + launch 的 `map_file` | — | nav / all_in_one |
| AMCL 粒子行为 | `config/nav/amcl_params.yaml` | 各 amcl 项 | amcl / all_in_one |

### 12.6 验证卡住恢复 / 夹角倒车 / 倒车回正是否修好

```bash
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_mini_pro_description nav_all_in_one.launch
```

1. 启动日志应含 `reverse_enter=130deg`、`allow_angle_reverse=True`、`vrev=0.70(ratio=0.67)`、`reorient=True`、`reorient_min=5.0s`、`hold=10.0s`、`clear_f/r=0.80/0.70`、`steer_lidar=True`、`avoid_steer=True`、`near/far=0.50/0.90`。
2. RViz：SetInitialPose → SetGoal，故意让车顶到墙/死角。
3. 约 5 s 后日志：`STUCK → RECOVER_REVERSE 5.0s`（或 `RECOVER_FORWARD`）。
4. 同一时段 `PP … mode=RECOVER_REVERSE v=-0.xx`（**必须为负**，无真实后向近障时 |v| 应接近 **0.70**）；可出现 `Laser keep-dir`（真实后向障碍），**不应**再被 `Laser escape … v_sign=1` 盖成前进顶墙，也**不应**因 `dist=0.10` 车体回波把 |v| 砍到 ~0.32。
5. 恢复持续约 4～6 s，车应有明显倒退/前进位移，而不只是原地拧方向蹭墙。
6. 夹角倒车：设目标使绿线落到车尾大角度一侧，当日志 `α(…°)` 绝对值 **> ~130°** 时应出现 `mode=REVERSE` 且 **`v≈-0.70`**；若难触发，把 `reverse_enter_angle` 调到 `1.745`（~100°）后重启再试。
7. **倒车回正锁定期**：进入倒车应见 `REORIENT arm`；前 **5 s** 内 `REORIENT check … pass=False (wait min=5.0s)`，**绝不可**因位移一点点就切前进。满 5 s 后看 `front/rear/disp` 的 `ok=`；都过则 `pass=True` → `REORIENT: … hold=10.0s lock` 与 `mode=REORIENT`（`v>0`）。随后约 **10 s** 内 `lock=1`，**不应**立刻 `Angle reverse ENTER`。满 hold 后见 `still locked`；前进一段或 α 回正后见 `REORIENT unlock`。若从不回正：略降 `reorient_clear_*` 或 `reorient_min_displacement`（仍建议保持 min≥4～5s）；若仍过早又进倒车：加大 `reorient_hold_sec` 或 `reorient_unlock_disp`。
8. **REORIENT 掉头方向**：贴墙场景应见 `REORIENT steer: path=L lidar L=0.30 R=0.90 → pick R`（绿线偏左、右更空则最终往右）。若仍往墙拧：略降 `reorient_side_clear_margin` 或略升 `reorient_side_near_wall`；若误覆盖绿线：反向调，或临时 `reorient_steer_lidar_enabled:=false`。
9. **瞬时避障**：绿线贴墙前进时，应见 `AvoidSteer front L=… R=… → bias=±…` 且 `PP … avoid=±0.xx`（偏置宜小，约 ±0.1～0.4）；离开墙后 `avoid≈0`。仍蹭墙：略升 `avoid_steer_omega_max` 或 `avoid_steer_far`；转向发飘/抢主控：降 `omega_max` 偏置。
10. 将 `allow_angle_reverse` 设为 `false` 重启后：大夹角应仍为 `FORWARD`（只转向），只有卡住才见 `RECOVER_REVERSE`；回正仍可对脱困倒车生效。