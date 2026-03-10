# 操作问题与调试记录

本文档记录使用 Scout Mini Pro 仿真包时遇到的操作问题及解决方案，便于后续排查与参考。

---

## 记录 1：节点未关闭导致 RViz No TF data（2025-02）

### 问题描述

按「方法一」启动 `display_rviz.launch` 后，RViz 显示异常：
- Global Status 显示 **No TF data**
- RobotModel 各 link（base_link、camera_link、轮子、传感器等）报错：**No transform from [base_footprint]**

### 现象/报错

终端出现：
```
shutdown request: [/joint_state_publisher] Reason: new node registered with same name
```

### 原因分析

之前曾按「方法二」手动启动过 `joint_state_publisher`、`robot_state_publisher`（使用 `&` 后台运行），关闭窗口或终端后这些进程可能仍在运行。再次运行「方法一」时，launch 会启动同名节点，ROS 不允许 duplicate node names，launch 启动的节点被强制关闭，TF 链断裂，RViz 无法收到变换数据。

### 解决办法

1. **运行方法一前先清理残留节点**：
   ```bash
   killall joint_state_publisher robot_state_publisher
   # 若仍有残留可加 -9 强制：
   killall -9 joint_state_publisher robot_state_publisher
   ```

2. **或使用全新终端**：打开新终端，确保无其他 ROS 节点在跑，再执行：
   ```bash
   cd ~/catkin_ws_2
   source devel/setup.bash
   roslaunch scout_mini_pro_description display_rviz.launch
   ```

3. **方法二结束后**：若曾用方法二手动启动，结束时建议显式执行上述 `killall` 清理后台进程，避免影响后续操作。

---

## 记录 2：rostopic list 显示 ERROR（2025-02）

### 问题描述

在关闭 `roslaunch` 后，新终端执行 `rostopic list` 显示：
```
ERROR: Unable to communicate with master!
```

### 原因分析

`rostopic`、`rosnode` 等命令需要与 ROS Master 通信。当 `roslaunch` 被 Ctrl+C 终止时，roscore 一并退出，Master 停止运行。此时在**任何终端**执行 `rostopic list` 都会失败。

### 解决办法

- **要检查话题时**：保持 `roslaunch scout_mini_pro_description gazebo_room_rviz.launch`（或相关 launch）在**至少一个终端**中运行，再在**另一终端**执行 `rostopic list`。
- 若单独测试命令，需先运行 `roscore`，再启动相关节点。

---

## 记录 3：RViz 中 Camera 显示 no image、LaserScan 不显示（2025-02）

### 问题描述

- RViz 中 Camera 显示 **no image**
- LaserScan 没有数据/不显示
- `rostopic list | grep scout_mini_pro` 中缺少 `/camera/image_raw`、`/imu` 等
- `rostopic hz /scout_mini_pro/scan` 提示 "no new messages"
- `rostopic echo /scout_mini_pro/camera/image_raw` 提示 "topic does not appear to be published yet"

### 调试步骤

1. **确认 launch 在运行**：话题检查需在 roslaunch 运行期间进行（见记录 2）。
2. **`rostopic hz` 与仿真时间**：当 `use_sim_time=true` 时，仿真时钟与系统时钟不同步，`rostopic hz` 可能报 "no new messages"。可改用：
   - `rostopic echo /scout_mini_pro/scan`（应能看到数据流）
   - 在 Gazebo 中观察激光束（蓝色射线）是否出现，以确认激光插件工作。
3. **Camera / IMU 话题缺失**：可能是 Gazebo 插件未正确加载，或 `libgazebo_ros_depth_camera` 与 Gazebo 11/Noetic 存在兼容问题。
4. **代码侧修复**：在 `scout_mini_pro.xacro` 中增加 `libgazebo_ros_camera.so` 插件作为 RGB 图像备用发布源，通常比深度相机插件更稳定。
5. **RViz 手动配置**：若传感器已订阅但无显示，需在 RViz 中：
   - 添加 LaserScan → Topic 设为 `/scout_mini_pro/scan`，Fixed Frame 与 Global Options 一致（如 `odom`）
   - 添加 Camera → Image Topic 设为 `/scout_mini_pro/camera/image_raw`
   - 添加 PointCloud2 → Topic 设为 `/scout_mini_pro/camera/depth/points`
   - 详见 README_2.md 中「若传感器未显示，手动添加」章节。

### 常见误判

- 在 roslaunch 已关闭的终端执行 `rostopic list`，误以为话题未发布。
- 用 `rostopic hz` 在 `use_sim_time` 下判断发布频率，容易得到 "no new messages"。

---

## 记录 4：TF_REPEATED_DATA 警告（2025-02）

### 问题描述

终端出现：
```
TF_REPEATED_DATA for rear_left_wheel_link / rear_right_wheel_link
```

### 原因分析

`gazebo_diff_drive` 与 `gazebo_joint_state_publisher` 可能同时发布轮子关节状态，导致 TF 数据重复。

### 解决办法

在 `gazebo_diff_drive` 插件中设置 `publish_wheel_joint_state="false"`，由 `gazebo_joint_state_publisher` 统一发布。当前工程已采用此配置。

---

## 记录 5：Gazebo Ctrl+C 退出时崩溃（2025-02）

### 问题描述

使用 Ctrl+C 关闭 Gazebo 仿真时，进程成功退出，但报错：
```
gzserver-11.15.1 crashed with SIGSEGV in ros::NodeHandle::shutdown()
```
堆栈涉及 `libgazebo_ros_imu_sensor.so` 及 "reading NULL VMA"。

### 原因分析

IMU 插件在 shutdown 阶段尝试访问已释放或无效的 ROS NodeHandle，属于关闭顺序导致的段错误。仿真**运行时**不受影响。

### 解决办法

- **一般可忽略**：若仿真、传感器、RViz 显示均正常，该崩溃仅发生在退出时，不影响使用。
- **如需消除**：可临时注释 `gazebo_imu_plugin` 做验证，或向 `gazebo_ros_pkgs` 提交 issue。

---

## 记录 6：Spawn 超时与 image_raw 无数据（2025-02）

### 问题描述

- 运行 `gazebo_room_rviz.launch` 时，终端报：**SpawnModel: Entity pushed to spawn queue, but spawn service timed out waiting for entity to appear in simulation under the name scout_mini_pro**，`spawn_scout_mini_pro` 进程以 exit code 1 退出。
- RViz 中 Camera 无图像；`rostopic echo /scout_mini_pro/camera/image_raw`、`/scout_mini_pro/camera/depth/points` 无数据（已确认 `/clock` 在发布）。

### 原因分析

1. **Spawn 超时**：`room_1.world` 场景较重，Gazebo 加载世界与物理引擎较慢；spawn_model 默认等待时间不足，在实体尚未出现在仿真中时就超时退出。Gazebo 仍会继续加载模型（故能看到 DiffDrive、JointStatePublisher 等插件启动），但 spawn 脚本已退出。
2. **image_raw / depth 无数据**：主模型此前仅启用了 `gazebo_depth_camera_plugin`（libgazebo_ros_depth_camera.so），在 Gazebo 11 + Noetic 下该插件常不稳定或延迟发布；未启用独立的 RGB 相机插件（libgazebo_ros_camera.so）。

### 解决办法

1. **spawn 前延迟**：在 [gazebo_room.launch](scout_mini_pro_description/launch/gazebo_room.launch) 中为 spawn_model 节点增加 `launch-prefix="bash -c 'sleep 18; exec $0 $@'"`，让 Gazebo 与 room_1 世界先稳定约 18 秒再生成机器人，降低超时概率；若本机仍超时，可将 sleep 调大（如 25–30 秒）。
2. **增加独立 RGB 相机插件**：在 [scout_mini_pro.xacro](scout_mini_pro_description/xacro/main/scout_mini_pro.xacro) 中增加对 `gazebo_camera_plugin` 的调用，使用 libgazebo_ros_camera.so 发布 `camera/image_raw` 与 `camera/camera_info`，保证 RViz 至少能稳定收到 RGB 图像；depth/points 仍由深度相机插件提供，若仍无数据可后续单独排查深度插件或 Gazebo 版本。

---

## 记录 7：RViz Camera 有 image_raw 发布但仍显示 no image（相机命名空间）（2025-02）

### 问题描述

- 已确认有 `image_raw` 在发布（如 `rostopic echo` 有数据），但 RViz 中 Camera 仍显示 **no image**。
- 终端日志出现：`Camera Plugin: Using the 'robotNamespace' param: '/'`。
- `rostopic list` 中可见 `/camera/camera/image_raw` 或 `/image_raw`，但**没有** `/scout_mini_pro/camera/image_raw`；而 `/scout_mini_pro/scan`、`/scout_mini_pro/camera/depth/points` 存在。

### 原因分析

`gazebo_ros_camera` 使用的 `GetRobotNamespace()`（gazebo_ros_pkgs 的 gazebo_ros_utils.h）**只读取**插件下的 **`<robotNamespace>`** 子元素，**不读取** `<ros><namespace>`。若 xacro 中仅写了 `<ros><namespace>` 而未写 `<robotNamespace>`，插件会得到空命名空间并退化为根命名空间，话题落在 `/camera/...` 或 `/image_raw`，与 RViz 中配置的 `/scout_mini_pro/camera/image_raw` 不一致，导致无图。

### 解决办法

1. **代码修复**：在 [gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro) 的 `gazebo_camera_plugin` 宏内增加 `<robotNamespace>${robot_namespace}</robotNamespace>`（见上）。
2. **后续简化**：为避免嵌套话题（见记录 8），RGB 相机已改为扁平话题 `/scout_mini_pro/image_raw`、`/scout_mini_pro/camera_info`；RViz Camera 应选上述两话题。

### 相关：TF “source_frame cannot be empty”

终端中大量 `Invalid argument passed to canTransform argument source_frame in tf2 frame_ids cannot be empty` 多来自 RViz 在订阅到空或错误话题时尝试做 TF 查询。**先**修复相机话题并确保 Global Options 中 **Fixed Frame** 为 `odom`，此类警告会明显减少或消失。

---

## 记录 8：话题命名简化与 CameraInfo / 空 Frame 问题（2025-02）

### 问题描述

- 同时打开多个 Camera 显示时：两个显示“有 image 无 CameraInfo”（订阅 `/camera/camera/image_raw`、`/camera/camera/depth/image_raw`），一个显示“有 CameraInfo 无 image”且 TF 报 **Frame [] does not exist**。
- 话题存在多层嵌套（如 `/camera/camera/image_raw`、`/camera/camera/camera_info`），深度插件因未设置 `<robotNamespace>` 发布在根命名空间，与 RGB 插件命名不一致；RViz 按“图像话题”推导 camera_info 话题时易错位，导致 No CameraInfo 或空 frame_id 查询。

### 原因分析

1. 深度插件（libgazebo_ros_depth_camera）未配置 `<robotNamespace>`，实际使用 ns=`/`，话题落在 `/camera/...`，且与 `cameraName` 等组合后出现 `camera/camera/...` 嵌套。
2. RGB 插件虽设置了 robotNamespace，但话题曾配置为 `camera/image_raw`、`camera/camera_info`，部分环境下实际发布为 `/scout_mini_pro/image_raw`、`/scout_mini_pro/camera_info`，与文档/默认 RViz 配置不一致。
3. RViz 中订阅到错误或空 frame_id 的显示会触发 “source_frame in tf2 frame_ids cannot be empty” 和 “Frame [] does not exist”。

### 解决办法（已实施）

1. **RGB 相机**（[gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro) + [scout_mini_pro.xacro](scout_mini_pro_description/xacro/main/scout_mini_pro.xacro)）：话题改为扁平名 `image_raw`、`camera_info`，发布到 **`/scout_mini_pro/image_raw`**、**`/scout_mini_pro/camera_info`**。
2. **深度相机**：为深度插件增加 **`<robotNamespace>${robot_namespace}</robotNamespace>`**，并将 `topic_prefix` 从 `camera` 改为 **`depth`**；点云话题改为 **`${topic_prefix}/points`**。发布话题为：
   - `/scout_mini_pro/depth/image_raw`、`/scout_mini_pro/depth/camera_info`（深度相机 RGB/信息）
   - `/scout_mini_pro/depth/depth/image_raw`、`/scout_mini_pro/depth/depth/camera_info`（深度图及信息）
   - **`/scout_mini_pro/depth/points`**（点云）
3. **RViz**（[display_room.rviz](scout_mini_pro_description/rviz/display_room.rviz)）：默认只保留一个 Camera 显示，Topic 设为 **`/scout_mini_pro/image_raw`**（Camera Info 自动对应 `/scout_mini_pro/camera_info`）；LaserScan 为 **`/scout_mini_pro/scan`**；PointCloud2 为 **`/scout_mini_pro/depth/points`**。避免同时订阅多个命名混乱的相机话题，减少空 frame 与 CameraInfo 错位。
4. **文档**：README_2 与本文档已更新为上述话题列表；建议只使用一个 RGB Camera 显示（`/scout_mini_pro/image_raw`），若需看深度图可另加 Camera 订阅 `/scout_mini_pro/depth/depth/image_raw` 并配对应 camera_info。

---

## 记录 9：depth/depth、depth/depth/depth 冗余话题与 Camera/Image 显示差异（2025-02）

### 问题描述

- `rostopic list` 中出现 `/scout_mini_pro/depth/depth/camera_info`、`/scout_mini_pro/depth/depth/depth/image_raw` 等冗杂话题名。
- RViz 中部分 Camera 显示“有 image 无 CameraInfo”，另一显示“Frame [] does not exist”；用 **Image** 显示订阅 `/scout_mini_pro/image_raw` 则正常。

### 原因分析

1. **depth/depth、depth/depth/depth 来源**：`libgazebo_ros_depth_camera.so` 拼接话题时为 **namespace + cameraName + topicName**。xacro 中若把 `imageTopicName` 写成 `${topic_prefix}/image_raw`（即 `depth/image_raw`）、`depthImageTopicName` 写成 `depth/depth/image_raw`，则实际话题变为 `/scout_mini_pro/depth/depth/image_raw` 与 `/scout_mini_pro/depth/depth/depth/image_raw`，即 cameraName 与 topic 名中的 prefix 重复拼接。
2. **修正方式**：话题名参数中**不再带 topic_prefix**，由插件的 cameraName 提供一层前缀即可。例如：`imageTopicName="image_raw"`、`depthImageTopicName="depth/image_raw"`、`pointCloudTopicName="points"`，则实际发布为 `/scout_mini_pro/depth/image_raw`、`/scout_mini_pro/depth/depth/image_raw`、`/scout_mini_pro/depth/points`，无 depth/depth/depth。
3. **Camera 与 Image 显示差异**：RViz 的 **Image** 显示只订阅图像话题，不查 TF、不依赖 camera_info，故无 “No CameraInfo” 与 “Frame [] does not exist”。**Camera** 显示会做 TF 查询并需要 camera_info，若话题错位或 frame_id 为空则报错。默认配置已改为使用 **Image** 显示 RGB（Topic `/scout_mini_pro/image_raw`），避免上述问题。

---

## 记录 10：depth 话题扁平化与点云朝向修正（2025-02）

### 问题描述

- 要求 **depth 下仅保留**深度图的 image_raw 与点云，**不再出现 depth/depth**；外层 `/scout_mini_pro/image_raw` 仅表示 RGB 彩色图。
- 点云显示在车顶而非车前方。

### 修改内容（已实施）

1. **depth 话题扁平化**（[gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro)）：深度插件话题名改为：`imageTopicName="rgb/image_raw"`、`cameraInfoTopicName="rgb/camera_info"`（深度相机 RGB 迁到 depth/rgb/）；`depthImageTopicName="image_raw"`、`depthImageCameraInfoTopicName="camera_info"`、`pointCloudTopicName="points"`。实际发布：`/scout_mini_pro/depth/image_raw`（深度图）、`/scout_mini_pro/depth/camera_info`、`/scout_mini_pro/depth/points`，以及可选的 `/scout_mini_pro/depth/rgb/image_raw`、`/scout_mini_pro/depth/rgb/camera_info`，**无 depth/depth**。
2. **Camera 显示配对**：若用 Camera 显示，RGB 须成对 `/scout_mini_pro/image_raw` + `/scout_mini_pro/camera_info`；深度须成对 `/scout_mini_pro/depth/image_raw` + `/scout_mini_pro/depth/camera_info`；Fixed Frame 为 `odom`。推荐默认用 Image 显示避免 frame/info 报错。
3. **点云朝向**：Gazebo 深度相机光学轴默认沿 link 的 +Z。原 camera_link 与 base_link 同向（Z 朝上），点云朝上。在 [common_macros.xacro](scout_mini_pro_description/xacro/macro/common_macros.xacro) 的 `sensor_fixed` 中增加可选参数 `rpy_r`、`rpy_p`、`rpy_y`（默认 0）；在 [camera_d435.xacro](scout_mini_pro_description/xacro/sensor/camera_d435.xacro) 中为相机关节传入 **rpy_p="-1.5708"**（pitch -90°），使 camera_link 的 Z 对准 base_link 的 X（车体前方），点云显示在车前方。

---

## 记录 11：Camera 显示 "Frame [] does not exist" 与 No CameraInfo 无反应（2025-02）

### 问题描述

- RViz 中 **Camera** 显示（订阅 `/scout_mini_pro/image_raw`）报错：**"For frame []: Frame [] does not exist"**；Image 与 Topic 状态正常，Camera Info 显示 "received"，但画面黑屏。
- 深度 Camera 显示（订阅 `/scout_mini_pro/depth/image_raw`）手动设置 Camera Info 为 `/scout_mini_pro/depth/camera_info` 后仍无反应（黑屏或仍提示 No CameraInfo）。

### 原因分析

`gazebo_ros_pkgs` 的 **GazeboRosCameraUtils**（被 RGB 与深度相机插件共用）从 SDF 读取的 frame 参数名为 **`frameName`**（驼峰），不是 `frame_name`。xacro 中若只写 `<frame_name>camera_link</frame_name>`，插件读不到 `frameName`，会走默认逻辑，导致发布的 Image/CameraInfo 消息 **header.frame_id 为空**。RViz Camera 显示依赖 frame_id 做 TF 变换，空 frame 即报 "Frame [] does not exist"；深度端同理，若 frame 为空则显示异常或无法正确配对。

### 解决办法（已实施）

在 [gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro) 中：

1. **RGB 相机插件**：增加 **`<frameName>${frame_name}</frameName>`**，与原有 `<frame_name>` 并存，确保插件实际读取到 frame 并写入消息 header。
2. **深度相机插件**：在已有 `<frameName>` 基础上增加 **`<frame_name>${frame_name}</frame_name>`**，兼容不同版本/继承类。

重新启动 Gazebo 仿真后，Image 与 CameraInfo 的 header.frame_id 应为 `camera_link`，RViz Camera 显示可正常渲染；若仍用 Image 显示，则无需改 RViz 配置。

---

## 记录 12：深度 Camera 显示 No CameraInfo（depth/camera_info 被父类占用）（2025-02）

### 问题描述

- RViz 中深度 **Camera** 显示（Image Topic = `/scout_mini_pro/depth/image_raw`）报 **No CameraInfo received on [/scout_mini_pro/depth/camera_info]. Topic may not exist.**

### 原因分析

`GazeboRosDepthCamera` 继承 `GazeboRosCameraUtils`。父类在同一节点（`/scout_mini_pro/depth`）上会发布一组 `image_raw` 与 `camera_info`（对应深度相机的 RGB/IR 流），子类再发布 `depthImageTopicName` / `depthImageCameraInfoTopicName`（深度图与深度 camera_info）。未显式区分时，父类默认使用 `image_raw`、`camera_info`，与子类的深度图话题重名，导致同一话题被重复 advertise，`/scout_mini_pro/depth/camera_info` 可能被父类占用或错用，RViz 收不到与深度图配对的 CameraInfo。

### 解决办法（已实施）

在 [gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro) 的深度相机插件中为父类显式指定 **`<imageTopicName>rgb/image_raw</imageTopicName>`**、**`<cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>`**，使父类只发布 `/scout_mini_pro/depth/rgb/image_raw` 与 `/scout_mini_pro/depth/rgb/camera_info`；子类继续独占 `depth/image_raw`、`depth/camera_info`、`depth/points`。重启仿真后，RViz Camera 显示配对 `/scout_mini_pro/depth/image_raw` + `/scout_mini_pro/depth/camera_info` 即可正常显示。

---

## 记录 13：LaserScan 有接收但无反应（Topic / Fixed Frame / TF 核对）（2025-02）

### 发布与订阅配置（代码侧已一致）

- **发布**（[gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro) + [scout_mini_pro.xacro](scout_mini_pro_description/xacro/main/scout_mini_pro.xacro)）：插件 `libgazebo_ros_ray_sensor.so`，命名空间 `/scout_mini_pro`，话题 **`/scout_mini_pro/scan`**，`header.frame_id` 为 **`laser_link`**。
- **订阅**（[display_room.rviz](scout_mini_pro_description/rviz/display_room.rviz)）：LaserScan Topic = `/scout_mini_pro/scan`，Fixed Frame = **`odom`**。
- **TF 链**：`odom` → `base_footprint`（diff_drive）→ `base_link` → … → `laser_link`（robot_state_publisher）。话题与 frame 配置一致，无需改代码即可正常显示。

### “有接收但无反应”常见原因

| 情况 | 说明 |
|------|------|
| Topic 选错 | 教学常用 `/scan`，本工程发布 **`/scout_mini_pro/scan`**。RViz 若设为 `/scan` 会收不到或收到别的话题，导致无显示。 |
| 未用 room 的 rviz 配置 | 未用 `display_room.rviz` 而自建/拷贝教学配置时，LaserScan Topic 常为 `/scan`，需改为 `/scout_mini_pro/scan`。 |
| Fixed Frame 不对 | 必须为 **`odom`**，否则 scan 无法正确变换到视窗坐标系。 |
| TF 缺或错 | `robot_state_publisher` 未跑或未收到 `joint_states` 时，缺 `base_link`→`laser_link`，RViz 无法绘制。 |
| 显示参数 | Size (m) 过小、Decay Time、Style 等会导致点不可见或误以为不更新。 |

### 建议核对步骤（无需改代码）

1. **RViz 配置**：用 `gazebo_room_rviz.launch` 会加载 `display_room.rviz`（LaserScan 已为 `/scout_mini_pro/scan`）。若自加或拷贝配置，在 Displays 中确认 LaserScan **Topic** = **`/scout_mini_pro/scan`**，Global Options 中 **Fixed Frame** = **`odom`**。
2. **话题在发**：`rostopic echo /scout_mini_pro/scan --noarr`（launch 与 Gazebo 运行中）应持续有输出。
3. **TF 有 laser_link**：`rosrun tf view_frames` 或 `rosrun tf tf_monitor`，确认存在 `odom` → … → **`laser_link`**。
4. **显示参数**：LaserScan 的 **Size (m)** 调大（如 0.05～0.1），**Decay Time** 可试调 1～2 秒观察是否有扫过效果。

若仍异常，请提供：当前 RViz 中 LaserScan 的 Topic 具体值；以及“无反应”是“完全无点”“有点但不更新”还是“点位置错误”。

---

## 记录 14：LaserScan 话题未发布（rostopic 无 scan，By topic 无 scan）（2025-03）

### 现象

- Gazebo 中有激光蓝色射线，但 `rostopic list | grep scan` 无输出，RViz “By topic” 中无 `/scout_mini_pro/scan`，说明 **ray_sensor 插件未向 ROS 发布话题**（插件未加载或加载失败）。

### 原因（同一 link 两个 gazebo 块）

- `laser_link` 在展开的 URDF 中有 **两个** `<gazebo reference="laser_link">` 块：一个来自 [common_macros.xacro](scout_mini_pro_description/xacro/macro/common_macros.xacro) 的 `sensor_fixed`（仅 `<material>`），一个来自 [gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro)（`<sensor>`+ 插件）。Gazebo 对同一 link 多块合并时可能导致 **sensor 插件不加载**，从而没有任何 scan 话题。

### 解决办法（已实施）

1. **common_macros.xacro**：新增 **`sensor_fixed_no_gazebo`** 宏（只生成 link+joint，不生成 `<gazebo>` 块）。
2. **laser_helios.xacro**：激光雷达改用 **`sensor_fixed_no_gazebo`**，不再用 `sensor_fixed`，这样 `laser_link` 不再带第一个 gazebo 块。
3. **gazebo_sensor_plugin.xacro**：在 **`gazebo_laser_plugin`** 的 `<gazebo reference="laser_link">` 内同时写 **`<material>Gazebo/Red</material>`** 与 **`<sensor>...</sensor>`**，保证 `laser_link` 只有一个 gazebo 块且内含 material 与 ray sensor 插件。

插件配置保持 **`<namespace>${robot_namespace}</namespace>` + `<remapping>~/out:=${topic}</remapping>`**（即相对话题 `scan`），发布话题为 **`/scout_mini_pro/scan`**。勿使用绝对话题名 remapping（如 `~/out:=/scout_mini_pro/scan`），否则可能导致插件初始化失败、话题仍不出现。

修改后需 **完全重启仿真**（关掉 Gazebo 与 RViz 后重新 `roslaunch scout_mini_pro_description gazebo_room_rviz.launch`），再用 `rostopic list | grep scan` 确认出现 `/scout_mini_pro/scan`。

### RViz “By topic” 里仍看不到 /scout_mini_pro/scan 时的做法

即使话题已在发布（`rostopic list` 有 `/scout_mini_pro/scan`），RViz “By topic” 有时只显示部分话题或按命名空间折叠后不易找到。在 [gazebo_room_rviz.launch](scout_mini_pro_description/launch/gazebo_room_rviz.launch) 中已增加 **topic_tools relay**：将 `/scout_mini_pro/scan` 中继到顶层话题 **`/scan`**。用 `gazebo_room_rviz.launch` 启动后，在 “By topic” 中可看到 **`/scan`**（LaserScan），选它即可添加激光显示；display_room.rviz 里仍订阅 `/scout_mini_pro/scan`，两种方式均可。

### 现象：Gazebo 有蓝色射线但 RViz 收不到点（ranges 为空）

**原因**：Ray 传感器若只配置 `<horizontal>` 而无 `<vertical>`，或缺少传感器 `<pose>`，部分 Gazebo/插件版本下插件拿到的射线数据为空，发布的 LaserScan 的 `ranges` 为空或全 NaN，RViz 显示 Status Ok 但无点。

**处理**（已实施）：在 [gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro) 的 ray 传感器中增加 **`<pose>0 0 0 0 0 0</pose>`**，并在 `<scan>` 内增加 **`<vertical>`** 段（`samples=1`，`min_angle=0`，`max_angle=0`），保证 2D 扫描至少有一条垂直射线，使插件能正确填充 ranges。重启仿真后可用 `rostopic echo /scout_mini_pro/scan -n 1` 查看 `ranges` 是否有有效数据。

### 根本原因：ROS Noetic 中不存在 libgazebo_ros_ray_sensor.so

**现象**：`rostopic list` 有时能看到 `/scan` 或 `/scout_mini_pro/scan`（由 relay 等产生），但 `rostopic echo` 收不到消息；Gazebo 有蓝色射线，RViz 无点。spawn 日志中只有 Camera、DiffDrive、JointStatePublisher，**没有** LaserScan/ray 相关加载信息。

**原因**：工程中使用的插件库名为 **`libgazebo_ros_ray_sensor.so`**，该库属于 ROS 2 / 新版本 gazebo_ros_pkgs，**在 ROS Noetic 中并未提供**。`/opt/ros/noetic/lib/` 下仅有 `libgazebo_ros_laser.so`、`libgazebo_ros_gpu_laser.so` 等，因此 Gazebo 无法加载该插件，自然不会发布 LaserScan 话题。

**处理**（已实施）：在 [gazebo_sensor_plugin.xacro](scout_mini_pro_description/xacro/macro/gazebo_sensor_plugin.xacro) 中将激光插件改为 **`libgazebo_ros_laser.so`**，并使用 Noetic 下该插件支持的参数：**`<topicName>/scan</topicName>`**、**`<frameName>laser_link</frameName>`**（不再使用 `<ros><namespace>`/`<remapping>`）。话题固定为 **`/scan`**，RViz 与 `rostopic echo /scan` 可正常收到数据。

### 最终验证：/scan 有效数据与 RViz 成功显示

- **终端验证**（见终端记录 `/scan -n 1`）：
  - `header.frame_id = "laser_link"`，`angle_min/angle_max/angle_increment` 与 xacro 配置一致。
  - `ranges` 为从 0.125 开始、逐渐增大的实数数组（数百个元素），**不再为空或全 NaN**。
  - `intensities` 数组存在但为 0，符合教程中简单房间场景未启用强度信息的情况。
- **RViz 验证**：
  - Global Options 中 **Fixed Frame = odom**，LaserScan 显示 Topic 设为 **`/scan`**。
  - 状态为 **OK**，在机器人前方成功看到随时间更新的红色点云/扫描扇形，与 Gazebo 中蓝色射线方向一致。
  
结论：LaserScan 问题的最终根因是 **在 ROS Noetic 中错误引用了不存在的 `libgazebo_ros_ray_sensor.so`**，修正为 **`libgazebo_ros_laser.so` 并改用 `<topicName>/scan</topicName>` + `<frameName>laser_link</frameName>` 后，Gazebo→ROS→RViz 的整条链路恢复正常，`/scan` 话题持续输出有效的激光数据。

---

## 导航相关（历史与当前）（2025-03）

**历史**：此前曾使用 **move_base** 导航栈，并因 costmap 未按地图 resize 等问题增加了补救脚本（如 `apply_global_costmap_config.py`、`wait_for_map_then_move_base.py`）及严格启动顺序；对应的问题与排错见旧版 README_3 与 git 历史（记录 15～24 已合并为此条）。

**当前**：导航已改为 **A\* 路径规划 + 纯追踪（Pure Pursuit）**，不再使用 move_base。若遇导航异常，请优先确认：

1. **启动顺序**：Gazebo → `rosparam set use_sim_time true` → 启动 amcl（`rosparam load ... amcl_params.yaml`，`rosrun amcl amcl odom:=/scout_mini_pro/odom`）→ `roslaunch scout_mini_pro_description nav_bringup.launch`（map_server + astar_nav_node + pure_pursuit_nav_node）→ RViz。
2. **操作顺序**：在 RViz 中**先**用 **SetInitialPose** 设好初始位姿（点在小车实际位置并拖出朝向），**再**用 **SetGoal** 设目标。
3. **话题自检**：`rostopic echo /path -n 1`、`rostopic echo /scout_mini_pro/cmd_vel` 是否有数据；`rostopic echo /amcl_pose -n 1` 是否正常。

SetGoal 仍发布到 `/move_base_simple/goal`，display_room_slam.rviz 中已配置；SetInitialPose 只更新 AMCL 估计位姿，不会移动 Gazebo 中的小车，请将绿色箭头点在小车**实际位置**。详细步骤见 [README_3.md](scout_mini_pro_description/README_3.md)。

---

## 记录 17：AMCL 报错常见原因与排查（2025-03）

### 问题描述

启动 amcl 后出现 “Failed to get transform”“Lookup would require extrapolation”“transform from [base_link] to [odom]” 等，或 amcl 退出、无 map→odom 发布。

### 可能原因

1. 未设置 `use_sim_time` 或 amcl 启动早于步骤 2。
2. `/map` 未发布（map_server 未起或晚于 amcl）。
3. amcl 未 remap `odom`，收不到 `/scout_mini_pro/odom`。
4. `odom`→`base_footprint` 在设 SetInitialPose 前不存在（设计如此），`transform_tolerance` 过小导致 AMCL 报错。

### 解决办法

1. **先执行步骤 2**（`rosparam set use_sim_time true`），再启动 amcl。
2. **先启动 map_server**（步骤 3），用 `rostopic echo /map -n 1` 确认后再启动 amcl。
3. 使用 `rosrun amcl amcl odom:=/scout_mini_pro/odom`。
4. 将 [amcl_params.yaml](scout_mini_pro_description/config/nav/amcl_params.yaml) 中 **transform_tolerance** 调大（已改为 0.5）。
5. **启动顺序**：Gazebo → use_sim_time → map_server → amcl →（可选）设 SetInitialPose 后再启动 move_base。

### 文档

[README_3.md](scout_mini_pro_description/README_3.md) 第六节已补充步骤 4 前确认项与“若 AMCL 报错”说明。

---

## 记录 18：AMCL “ignoring NAN in initial pose Yaw”（2025-03）

### 问题描述

AMCL 终端出现：`[WARN] ignoring NAN in initial pose Yaw`。

### 原因

AMCL 收到过一条 `/initialpose` 消息，其中朝向（yaw）为 NAN。常见于：RViz 刚打开时自动发了一条未完整初始化的位姿；或点选 SetInitialPose 时**只点了一下就松开、未拖拽出朝向箭头**，导致四元数/欧拉角异常。

### 解决办法

用 **SetInitialPose** 在地图上**先点一下机器人大致位置，再按住左键拖拽出朝向箭头**，然后松开。不要只点不拖。若已正确操作后仍偶发，可忽略该条 WARN，后续 “Setting pose” 正常即表示位姿已生效。

