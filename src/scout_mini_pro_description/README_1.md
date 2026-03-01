# Scout Mini Pro 机器人模型说明

基于松灵机器人 Scout Mini Pro 小车的极简 XACRO/URDF 模型，用于越野环境自动驾驶项目开发。当前支持 RVIZ 可视化与 Gazebo 空世界仿真。

## 环境要求

- Ubuntu 20.04 LTS
- ROS Noetic
- 依赖：`urdf`、`xacro`、`gazebo_ros`、`gazebo_ros_control`、`robot_state_publisher`、`joint_state_publisher`、`rviz`

---

## 当前工作内容

1. **模块化 XACRO 模型**：车身、车轮、传感器按功能拆分，便于维护和扩展
2. **base_footprint 虚拟部件**：作为 TF 坐标参考（距地 0.01 m），用于导航与定位
3. **惯性属性**：车身、车轮、传感器均按几何形状配置质量和惯性（长方体/圆柱体公式）
4. **半透明材质**：车身、车轮、传感器 alpha=0.5，便于在 RVIZ 中查看 TF 标签
5. **差速驱动**：后轮驱动，支持 Gazebo 仿真与键盘控制
6. **传感器 Gazebo 插件**：激光雷达、相机、IMU 的 Gazebo 传感器插件由 `gazebo_sensor_plugin.xacro` 宏集中管理，便于调整话题名、参数

---

## 文件结构

```
scout_mini_pro_description/
├── CMakeLists.txt
├── package.xml
├── README_1.md
├── launch/
│   ├── display_rviz.launch    # RVIZ 可视化（纯模型，无 Gazebo）
│   ├── gazebo_sim.launch      # Gazebo 仿真
│   └── gazebo_rviz.launch     # Gazebo + RVIZ 同步显示
├── rviz/
│   └── display.rviz           # RVIZ 预设配置
└── xacro/
    ├── main/
    │   └── scout_mini_pro.xacro   # 主文件：参数 + 整合
    ├── macro/
    │   ├── common_macros.xacro    # 宏：车轮、传感器、传动
    │   ├── gazebo_diff_drive.xacro # 宏：差速驱动 + 关节状态插件（参数集中管理）
    │   └── gazebo_sensor_plugin.xacro # 宏：激光雷达、相机、IMU 的 Gazebo 传感器插件
    ├── actuator/
    │   └── wheel.xacro            # 4 轮模块
    └── sensor/
        ├── camera_d435.xacro      # D435 深度相机
        ├── laser_helios.xacro     # RS-Helios-16P 激光雷达
        └── imu_ch110.xacro        # CH110 IMU
```

---

## 文件关系

```
scout_mini_pro.xacro (主文件)
    │
    ├── include common_macros.xacro  (宏定义)
    ├── include gazebo_diff_drive.xacro  # 差速驱动与关节状态
    ├── include gazebo_sensor_plugin.xacro  # 激光雷达、相机、IMU 传感器插件
    ├── include actuator/wheel.xacro (4 轮，调用 wheel 宏)
    ├── include sensor/camera_d435.xacro (调用 sensor_fixed 宏)
    ├── include sensor/laser_helios.xacro
    └── include sensor/imu_ch110.xacro
```

- **scout_mini_pro.xacro**：定义全局参数（车身/车轮尺寸、质量等），声明 `base_footprint`、`base_link`，引用各子模块，调用 `gazebo_diff_drive`、`gazebo_sensor_plugin` 宏
- **gazebo_diff_drive.xacro**：差速驱动插件（cmd_vel、odom、TF 等）与关节状态发布器（joint_states），参数在此集中修改
- **gazebo_sensor_plugin.xacro**：激光雷达、相机、IMU 的 Gazebo 传感器插件宏，话题名、帧名、更新频率等参数在此集中修改
- **common_macros.xacro**：提供 `wheel`、`sensor_fixed`、`transmission` 宏，供 actuator 与 sensor 复用
- **wheel.xacro**：调用 4 次 `wheel` 宏，生成 `front_left/right`、`rear_left/right` 四轮
- **sensor/*.xacro**：调用 `sensor_fixed`，定义位置、尺寸、质量、颜色

---

## 如何将机器人导入 RVIZ

### 方法一：使用 launch 文件（推荐）

```bash
cd ~/catkin_ws_2
catkin_make
source devel/setup.bash

roslaunch scout_mini_pro_description display_rviz.launch
```

该命令会：
1. 加载 XACRO 模型到 `robot_description` 参数
2. 启动 `joint_state_publisher`（GUI）发布车轮关节状态
3. 启动 `robot_state_publisher` 发布 TF
4. 启动 RVIZ 并加载 `rviz/display.rviz`（预设 RobotModel、TF、Fixed Frame=base_footprint）

### 方法二：手动启动

```bash
source devel/setup.bash

# 1. 加载模型
rosparam set robot_description "$(rosrun xacro xacro $(rospack find scout_mini_pro_description)/xacro/main/scout_mini_pro.xacro)"

# 2. 启动关节状态发布器
rosrun joint_state_publisher joint_state_publisher --ros-args -p use_gui:=true &

# 3. 启动机器人状态发布器
rosrun robot_state_publisher robot_state_publisher &

# 4. 启动 RVIZ
rviz
```

在 RVIZ 中：
- 将 **Fixed Frame** 设为 `base_footprint`
- 点击 **Add** → **RobotModel**
- 点击 **Add** → **TF**（可选，用于查看坐标系）

---

## Gazebo 仿真

```bash
roslaunch scout_mini_pro_description gazebo_sim.launch
```

新终端中键盘控制：

```bash
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/scout_mini_pro/cmd_vel
```

---

## Gazebo 与 RVIZ 同步显示

同时启动 Gazebo 和 RVIZ，用键盘控制时，小车会在**两个软件中同步运动**（共享 `/tf`、`robot_description` 等）。

```bash
roslaunch scout_mini_pro_description gazebo_rviz.launch
```

新终端中键盘控制：

```bash
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/scout_mini_pro/cmd_vel
```

---

## 使用 rqt 查看流程图与系统逻辑

### rqt_graph（节点与话题关系图）

1. 启动仿真：`roslaunch scout_mini_pro_description gazebo_sim.launch`（或 `gazebo_rviz.launch`）
2. 新终端运行：`rqt_graph` 或 `rosrun rqt_graph rqt_graph`
3. 左侧选择 **Nodes/Topics (all)**，可看到 `robot_state_publisher`、`spawn_scout_mini_pro`、Gazebo 节点、`/cmd_vel`、`/odom`、`/joint_states` 等
4. 勾选 **Hide** 下的 **Dead Sinks**、**Dead Sources** 可简化视图

### rqt_tf_tree（TF 树）

1. 安装（如未安装）：`sudo apt install ros-noetic-rqt-tf-tree`
2. 运行：`rosrun rqt_tf_tree rqt_tf_tree`
3. 可查看 `odom` → `base_footprint` → `base_link` → 各 link 的 TF 层级

### 查看机器人、插件逻辑

- **差速驱动**：在 rqt_graph 中查看 `/scout_mini_pro/cmd_vel` 被谁订阅、`/scout_mini_pro/odom` 由谁发布
- **关节状态**：`/scout_mini_pro/joint_states` 由 Gazebo 插件发布，被 `robot_state_publisher` 订阅
- **插件配置**：差速与关节状态参数在 `xacro/macro/gazebo_diff_drive.xacro` 中集中管理；传感器插件参数在 `xacro/macro/gazebo_sensor_plugin.xacro` 中集中管理

---

## 传感器话题与 ROS 验证方法

Gazebo 仿真启动后，传感器插件会发布以下话题（命名空间 `/scout_mini_pro`）：

| 传感器 | 话题 | 消息类型 |
|--------|------|----------|
| 激光雷达 | `/scout_mini_pro/scan` | sensor_msgs/LaserScan |
| 相机 | `/scout_mini_pro/camera/image_raw` | sensor_msgs/Image |
| 相机信息 | `/scout_mini_pro/camera/camera_info` | sensor_msgs/CameraInfo |
| IMU | `/scout_mini_pro/imu` | sensor_msgs/Imu |

### 验证话题是否发布

1. 启动仿真：`roslaunch scout_mini_pro_description gazebo_sim.launch`（或 `gazebo_rviz.launch`）

2. 新终端中列出传感器相关话题：
   ```bash
   source devel/setup.bash
   rostopic list | grep -E "scan|imu|camera"
   ```
   应看到 `/scout_mini_pro/scan`、`/scout_mini_pro/imu`、`/scout_mini_pro/camera/image_raw`、`/scout_mini_pro/camera/camera_info`

3. 确认激光雷达（Lidar）话题：
   ```bash
   rostopic info /scout_mini_pro/scan
   rostopic echo /scout_mini_pro/scan --noarr
   rostopic hz /scout_mini_pro/scan
   ```
   - `rostopic info`：查看发布者、订阅者、消息类型
   - `rostopic echo`：若有数据输出则说明 Lidar 正常发布
   - `rostopic hz`：查看发布频率（默认约 10 Hz）

4. 在 RVIZ 中：`display.rviz` 已配置 LaserScan 显示，Topic 为 `/scout_mini_pro/scan`，启动 `gazebo_rviz.launch` 后可看到激光扫描点云。

---

## 核心参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 车身 | 0.6×0.4×0.2 m, 5 kg | base_link |
| 车轮 | φ0.15 m, 宽 0.08 m, 0.5 kg/轮 | 四轮差速 |
| base_footprint | z=0.01 m | 虚拟坐标参考 |
| 传感器几何 | 见各 sensor xacro | 位置、质量按实机近似 |
| 传感器话题 | 见 `gazebo_sensor_plugin.xacro` | 激光 /scan、相机 /camera/*、IMU /imu，支持 namespace、topic 参数调整 |
