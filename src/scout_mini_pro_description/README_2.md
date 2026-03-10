# Room_1 场景与传感器可视化教程

本文档说明如何在 `room_1` 小房间场景中运行 Scout Mini Pro 机器人，并在 RViz 中可视化激光雷达、相机、深度点云、IMU 等传感器数据。

## 一、场景说明

- **room_1**：自定义小房间模型，由多面墙体组成（位于 `world/room/room_1/`）
- **cafe_table**：Gazebo 内置餐桌模型（`model://cafe_table`）
- **传感器**：激光雷达、RGB 相机、深度相机（D435 模拟）、IMU

## 二、传感器话题（depth 下仅一层，无 depth/depth）

深度图及 RViz Camera 显示请使用 **`depth/image_raw`** 与 **`depth/camera_info`**；深度相机的彩色流在 **`depth/rgb/`** 下（可选）。

| 传感器 | 话题 | 消息类型 |
|--------|------|----------|
| 激光雷达 | `/scout_mini_pro/scan` | sensor_msgs/LaserScan |
| **RGB 相机（外层）** | | |
| RGB 图像 | `/scout_mini_pro/image_raw` | sensor_msgs/Image |
| RGB 相机信息 | `/scout_mini_pro/camera_info` | sensor_msgs/CameraInfo |
| **深度相机（depth 下仅 image_raw + points）** | | |
| 深度图（16bit） | `/scout_mini_pro/depth/image_raw` | sensor_msgs/Image |
| 深度图 camera_info | `/scout_mini_pro/depth/camera_info` | sensor_msgs/CameraInfo |
| 深度点云 | `/scout_mini_pro/depth/points` | sensor_msgs/PointCloud2 |
| 深度相机 RGB（可选） | `/scout_mini_pro/depth/rgb/image_raw`、`/scout_mini_pro/depth/rgb/camera_info` | sensor_msgs/Image、CameraInfo |
| IMU | `/scout_mini_pro/imu` | sensor_msgs/Imu |

---

## 三、分步教程：room 仿真 + RViz 传感器可视化

### 环境准备

```bash
cd ~/catkin_ws_2
catkin_make
source devel/setup.bash
```

### 步骤 1：启动 room 仿真

```bash
roslaunch scout_mini_pro_description gazebo_room.launch
```

Gazebo 将加载 room_1 场景，并在原点 (0, 0, 0.5) 生成 Scout Mini Pro 机器人。

### 步骤 2：启动 RViz 并加载 room 配置

新开终端：

```bash
source devel/setup.bash
rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room.rviz
```

`display_room.rviz` 已预设：RobotModel、TF、LaserScan、**Image**（RGB 图像）、PointCloud2，Fixed Frame 为 `odom`。**推荐用 “Image” 显示**订阅 `/scout_mini_pro/image_raw` 看 RGB，不依赖 camera_info 与 TF，避免 “Frame [] does not exist” 等报错；若改用 “Camera” 显示，需将 Image Topic 与 Camera Info 成对设为 `/scout_mini_pro/image_raw`、`/scout_mini_pro/camera_info`，且 Fixed Frame 为 `odom`。

### 步骤 3：在 RViz 中配置传感器显示（若需手动添加）

若 RViz 中某传感器未显示，可按以下步骤手动添加：

**LaserScan（激光雷达）**
1. 点击左下角 Displays 面板的 **Add** 按钮
2. 选择 **By display type** 标签，在列表中选中 **LaserScan** → **OK**
3. 展开新建的 LaserScan 项，将 **Topic** 设为 `/scout_mini_pro/scan`
4. 确认 **Enabled** 已勾选。若仍不显示，可尝试调整 **Style**（如 Flat Squares）、**Size (m)**（如 0.05）或 **Color Transformer**（如 Intensity）

**Image（推荐：RGB 与深度图）**
1. Add → By display type → **Image** → OK
2. **RGB**：Topic 设为 `/scout_mini_pro/image_raw`。**深度图**：再加一个 Image，Topic 设为 `/scout_mini_pro/depth/image_raw`。Image 显示不依赖 camera_info 与 TF，可避免 Camera 的 “No CameraInfo” 与 “Frame [] does not exist” 报错。

**Camera（可选，须成对）**
1. **RGB**：Image Topic = `/scout_mini_pro/image_raw`，Camera Info = `/scout_mini_pro/camera_info`。
2. **深度图**：Image Topic = `/scout_mini_pro/depth/image_raw`，Camera Info = `/scout_mini_pro/depth/camera_info`。
3. Global Options 中 **Fixed Frame** 必须为 `odom`，否则易出现 “No CameraInfo received” 或 “Frame [] does not exist”。

**PointCloud2（深度点云）**
1. Add → By display type → **PointCloud2** → OK
2. 将 **Topic** 设为 `/scout_mini_pro/depth/points`
3. 需深度相机插件正常发布该话题才会显示

**统一检查**
- 在 **Global Options** 中，将 **Fixed Frame** 设为 `odom`

### 步骤 4：话题检查与诊断

启动仿真与 RViz 后，在新终端执行以下命令验证传感器话题：

```bash
source devel/setup.bash

# 查看所有 Scout Mini Pro 相关话题
rostopic list | grep scout_mini_pro
```

**仿真时间与 rostopic hz**：当 `use_sim_time=true` 时，`rostopic hz` 可能显示 "no new messages" 或 "WARNING: may be using simulated time"，这不代表话题未发布。建议改用：
- `rostopic echo /scout_mini_pro/scan --noarr`（查看 LaserScan 是否持续输出）
- `rqt_image_view` 选择 `/scout_mini_pro/image_raw` 验证 RGB 相机

**验证各传感器话题**：

```bash
rostopic echo /scout_mini_pro/scan --noarr          # 激光雷达
rostopic echo /scout_mini_pro/image_raw --noarr   # RGB 图像（会刷屏，按 Ctrl+C 退出）
rostopic echo /scout_mini_pro/depth/points --noarr  # 深度点云
rostopic echo /scout_mini_pro/imu --noarr           # IMU
```

**Gazebo 插件报错**：若 `rostopic list` 中缺少相机或 IMU 话题，请查看 Gazebo 启动终端的输出，是否有 `Failed to load plugin`、`depth_camera_controller`、`imu_plugin` 等错误。可确认依赖：`sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins`

### 步骤 5：键盘控制

新终端中启动键盘控制：

```bash
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/scout_mini_pro/cmd_vel
```

### 步骤 6：rqt 查看图像

```bash
rqt
```

菜单：Plugins → Visualization → Image View，在下拉框中选择：
- `/scout_mini_pro/image_raw`（RGB）
- `/scout_mini_pro/depth/image_raw`（深度图）

### 一步启动（Gazebo + RViz）

```bash
roslaunch scout_mini_pro_description gazebo_room_rviz.launch
```

随后在另一终端启动键盘控制即可。

### 深度排查：插件冲突与 use_sim_time

**1. 插件冲突（camera_link 同 link 多传感器）**

若 `rostopic list` 中缺少 `/scout_mini_pro/image_raw` 或 `/scout_mini_pro/depth/points`，请查看 Gazebo 终端是否有插件加载失败。当前模型同时启用 `gazebo_camera_plugin`（RGB：image_raw、camera_info）与 `gazebo_depth_camera_plugin`（深度：depth/points 等）。若仅需 RGB，可尝试：

- 方案 B：在 `scout_mini_pro.xacro` 中移除 `gazebo_depth_camera_plugin`，只保留 `gazebo_camera_plugin`，获得 RGB 图像（但无深度点云）

**2. use_sim_time 与 rostopic 验证**

仿真中 `use_sim_time=true`，`rostopic hz` 常显示 "no new messages" 或 "simulated time is active"，不代表话题未发布。建议：

```bash
# 先确保本终端使用仿真时间
rosparam set use_sim_time true

# 验证 /clock 是否在发布（Gazebo 运行时应持续输出）
rostopic echo /clock --noarr

# 再验证各传感器（会刷屏，收到数据即表示正常）
rostopic echo /scout_mini_pro/scan --noarr
rostopic echo /scout_mini_pro/image_raw --noarr
rostopic echo /scout_mini_pro/depth/points --noarr
rostopic echo /scout_mini_pro/imu --noarr
```

若 Gazebo 中能看见蓝色雷达射线且世界在运行，LaserScan 通常已正常发布；`rostopic echo` 无输出时，检查 Gazebo 是否完全启动、`/clock` 是否发布。

**3. RViz "source_frame in tf2 frame_ids cannot be empty"**

该警告通常出现在 Camera、LaserScan 等 display 订阅的话题不存在或消息 `header.frame_id` 为空时。修复话题发布后，此类报错会显著减少或消失。确保 Fixed Frame 为 `odom`，且各 display 的 Topic 名称正确。

### 常见问题排查

| 现象 | 可能原因 | 排查建议 |
|------|----------|----------|
| Camera 显示 no image | Fixed Frame 错误、话题未发布、CameraInfo 话题不匹配 | 将 Fixed Frame 设为 `odom`；Image Topic 设为 `/scout_mini_pro/image_raw`，Camera Info 设为 `/scout_mini_pro/camera_info`；避免使用带嵌套的 `/camera/camera/...` 话题 |
| LaserScan 不显示 | Fixed Frame 错误、话题未发布 | 确认 Fixed Frame 为 `odom`；在 Gazebo 中观察是否有蓝色射线（雷达在发布）；用 `rostopic echo /scout_mini_pro/scan --noarr` 验证 |
| rostopic list 中无相机/IMU 话题 | 插件加载失败、依赖未安装、同 link 插件冲突 | 查看 Gazebo 启动终端是否有 `Failed to load plugin` 等报错；执行 `sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins`；参考上文「插件冲突」 |
| rostopic hz 显示 no new messages | 仿真时间（use_sim_time）与墙钟不同步 | 属正常现象，改用 `rostopic echo` 或 `rqt_image_view` 验证 |
| RViz 大量 "source_frame cannot be empty" | 订阅的话题不存在或消息 frame_id 为空 | 先修复 Gazebo 插件使话题正常发布；确认 Fixed Frame 为 `odom`，Topic 拼写正确 |

---

## 四、其他加载方式

### 仅加载 world（不生成机器人）

只启动 Gazebo 并加载 room_1 场景（地面 + 房间 + 餐桌），不生成 Scout Mini Pro 机器人。

```bash
roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find scout_mini_pro_description)/world/room_1.world
```

### 先启动 world，再手动生成机器人

1. 按上文启动 room_1 场景
2. 新终端执行：

```bash
source devel/setup.bash

# 加载机器人描述
rosparam set robot_description "$(rosrun xacro xacro $(rospack find scout_mini_pro_description)/xacro/main/scout_mini_pro.xacro)"

# 在 Gazebo 中生成机器人（可调整 -x -y -z 指定位置）
rosrun gazebo_ros spawn_model -urdf -model scout_mini_pro -param robot_description -x 0 -y 0 -z 0.5

# 机器人状态发布器（订阅 Gazebo 插件发布的 joint_states）
rosrun robot_state_publisher robot_state_publisher --ros-args -r joint_states:=/scout_mini_pro/joint_states
```

3. 键盘控制命令见上文步骤 4。

---

## 五、环境与路径说明

- **GAZEBO_MODEL_PATH**：`cafe_table` 依赖 Gazebo 默认模型路径，通常无需额外设置。
- **GAZEBO_RESOURCE_PATH**：若材质显示异常，可检查该变量是否包含 Gazebo 资源目录，例如：
  ```bash
  export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11
  ```

---

## 六、使用 slam_toolbox 构建导航地图与保存

在 Gazebo room 场景下，可用 **slam_toolbox** 在线建图，并用 **map_server** 将地图保存为 `room.pgm` + `room.yaml`，供后续导航使用。环境：Ubuntu 20.04、ROS Noetic。

### 6.1 安装依赖

```bash
sudo apt update
sudo apt install ros-noetic-slam-toolbox ros-noetic-map-server
```

- **slam_toolbox**：订阅 `/scan`、发布 `/map` 和 `map`→`odom` TF
- **map_server**：提供 `map_saver`，用于保存当前 `/map` 为 pgm + yaml

### 6.2 一键启动（推荐）

一条命令启动 Gazebo room、slam_toolbox 与 RViz（SLAM 专用配置，含 Map 显示、Fixed Frame=map）：

```bash
source ~/catkin_ws_2/devel/setup.bash
roslaunch scout_mini_pro_description gazebo_room_slam.launch
```

若报错找不到 `slam_toolbox/online_async.launch`，可执行 `ls $(rospack find slam_toolbox)/launch` 查看实际文件名（如 `online_async_launch.launch`），并修改 `launch/gazebo_room_slam.launch` 中的 include 文件名。

### 6.3 分步启动（可选）

1. **启动仿真 + RViz**  
   `roslaunch scout_mini_pro_description gazebo_room_rviz.launch`

2. **新终端启动 slam_toolbox**  
   ```bash
   source ~/catkin_ws_2/devel/setup.bash
   rosparam set use_sim_time true
   roslaunch slam_toolbox online_async.launch use_sim_time:=true
   ```
   若包内 launch 名为 `online_async_launch.launch`，则改为 `roslaunch slam_toolbox online_async_launch.launch use_sim_time:=true`。

3. **RViz 中显示地图**  
   若未用一键启动，可手动加载 SLAM 配置：  
   `rviz -d $(rospack find scout_mini_pro_description)/rviz/display_room_slam.rviz`  
   该配置已包含 Map（Topic `/map`）和 Fixed Frame `map`。

### 6.4 键盘控制建图

新终端：

```bash
source ~/catkin_ws_2/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/scout_mini_pro/cmd_vel
```

操纵小车在房间内移动，RViz 中地图会逐渐填充。

### 6.5 保存地图

建图满意后，在新终端执行（地图将保存在当前工作目录）：

```bash
source ~/catkin_ws_2/devel/setup.bash
rosparam set use_sim_time true
cd $(rospack find scout_mini_pro_description)/maps
rosrun map_server map_saver -f room
```

将生成 **room.pgm**（栅格图）和 **room.yaml**（分辨率、原点等元数据），可留作后续导航地图使用。
