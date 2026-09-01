# 3D / 越野 · 调试记录（DEBUG_RECORDS）

结构：日期 / 现象 / 原因 / 解决 / 相关命令。  
2D 侧更全的记录见 [`../2D/debug_records.md`](../2D/debug_records.md)。

---

## 记录 1：gzserver 11345 被占 → spawn_model 失败（2026-08）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08 |
| **现象** | Gazebo 起不来或立刻退出；`spawn_model` 弹故障框；小车不出现；终端可见 gzserver exit 255 / Address already in use |
| **原因** | 上次仿真残留的 `gzserver` 仍占用 **11345**；再用 `empty_world` 裸启 gzserver 会冲突 |
| **解决** | ① 杀残留；② 官方车用 `gazebo_room_official.launch`（`gzserver_clean_prefix.sh` + `spawn_delay`） |
| **相关命令** | 见下 |

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
pgrep -a gzserver || echo "gzserver 已清"
# 再：
roslaunch scout_mini_pro_description gazebo_room_official.launch
# 或一键训 TD3（内部已 include 官方 room）：
roslaunch scout_offroad_3d train_td3.launch gui:=true
```

---

## 记录 2：TF_REPEATED_DATA（odom→base_link）（2026-08）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08 |
| **现象** | 终端刷 `TF_REPEATED_DATA ... frame base_link (parent odom)` / `unknown_publisher` |
| **原因** | `gazebo_ros_skid_steer_drive` 在 `broadcastTF=true` 时用旧 tf API + 高 updateRate，与其它 TF 源抢发同一变换 |
| **解决** | 插件 `broadcastTF=false`，只发 `/odom`；由 `odom_to_tf_official.py` 独家发 `odom→base_link`（跳过非递增时间戳） |
| **相关命令** | `rostopic info /tf` 应见 `/odom_to_tf_official`；官方：`gazebo_room_official.launch` |

---

## 记录 3：扇区只看前方 / 后方盲区（2026-08）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08 |
| **现象** | 深度点云 `source:=cloud` 时后方扇区长期 ≈`max_range`；倒车可能不知后障/悬崖 |
| **原因** | Kinect/深度相机 FOV 仅前方；点云模式天然看不见车后 |
| **解决** | 阶段 A **默认** `source:=cloud` + `/lidar/points`（16 线全周）；2D `source:=scan` 仅作回退；深度相机仍仅前方 |
| **相关命令** | |

```bash
roslaunch scout_offroad_3d range_features.launch   # 默认 cloud + 全周 16 线
# 仅前方深度（勿当全周唯一眼睛）：
roslaunch scout_offroad_3d range_features.launch source:=cloud full_circle:=false
rostopic echo /offroad/range_features -n 1   # 期望长度 24
```

---

## 记录 4：IMU noise SDF 写错导致 spawn 超时（2026-08）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08 |
| **现象** | 加 IMU 后车不出现 / spawn 超时；Gazebo 日志 SDF 解析失败 |
| **原因** | Gazebo 11 要求 IMU `<noise type="gaussian" .../>` 用 **属性**；写成子元素 `<type>gaussian</type>` 会解析失败 |
| **解决** | 官方车 `scout_mini.gazebo`：插件侧用 `gaussianNoise`，勿用非法 noise 子树；见文件内注释 |
| **相关命令** | 重开官方仿真后：`rostopic echo /imu/data -n 1`（阶段 A 训 TD3 **不依赖** IMU） |

---

## 记录 5：外网拉取 Clearpath/AWS 越野 world 失败（2026-08）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | `git clone` Clearpath `cpr_gazebo` / `gazebo_models` 超时或卡住；AWS RoboMaker 默认分支仅归档 README（无 `.world`）；部分 `curl` raw.githubusercontent.com SSL 校验失败 |
| **原因** | 网络/SSL；AWS 仓归档后默认分支空壳；大仓浅克隆慢 |
| **解决** | ① 规范目录用 `src/scout_offroad_3d/worlds/`；② **复制**本仓已有 `scout_gazebo_sim/worlds/clearpath_playpen.world`；③ 拉取 Gazebo Classic `heightmap.world`（`-k`）；④ 自建 `rough_ramps.world` / `rough_obstacles.world`；⑤ 手动链接见 `worlds/SOURCES.md` 与 README1 §九 |
| **相关命令** | `roslaunch scout_offroad_3d train_td3.launch world:=$(find scout_offroad_3d)/worlds/rough_ramps.world` |

---

## 记录 6：有 gzserver 无窗口 — clean 误杀 gzclient（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | `gazebo_room_official.launch` 已跑、车已 spawn，但看不到 Gazebo 窗口；终端有 `[gazebo_gui-3] process has died ... exit code -9` |
| **原因** | 非 `gui:=false`、非 DISPLAY。启动时 11345 仍被旧进程占用，`gzserver_clean_prefix.sh` 执行 `killall -9 gzserver gzclient`，把 **刚一起动的新 gzclient** 一并 SIGKILL；gzserver 稍后正常起来并完成 spawn |
| **解决** | ① 当前会话可直接另开 `gzclient` / `rosrun gazebo_ros gzclient`；② 正规做法：先 `killall` 清残留再 launch，避免 clean 与新 gui 竞态；③ **已修**：`gzserver_clean_prefix.sh` 只杀占用 11345 的 PID / `gzserver`，不再 `killall gzclient` |
| **相关命令** | 见下 |

```bash
# 仿真已在跑、只要窗口：
rosrun gazebo_ros gzclient
# 或干净重开：
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
source ~/catkin_ws_2/devel/setup.bash
roslaunch scout_mini_pro_description gazebo_room_official.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_ramps.world
```

---

## 记录 7：train_td3_node 立刻 ModuleNotFoundError: td3（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 短冒烟 `train_td3.launch` 后终端停在 Gazebo/spawn 日志，无 TD3 训练刷屏；`[train_td3_node-9] process has died ... exit code 1` |
| **原因** | **未真正开始训练**。`train_td3_node.py` 用了 **两层** `os.path.dirname(__file__)`，把 `scout_offroad_3d/` 包根加入 `sys.path`，而 `td3/` 在 `scripts/td3/` → `No module named 'td3'`。Gazebo 仍在跑，看起来像「卡住」 |
| **解决** | 改为与 `infer_td3_node.py` 一致：只 `dirname` 一次，把 `scripts/` 加入 path |
| **相关命令** | 见下 |

```bash
# 先清残留，再短冒烟确认有 TD3 日志（Exploration / Total T 等）：
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
source ~/catkin_ws_2/devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=2000 start_timesteps:=200
```

---

## 记录 8：训练中按 timestep 换 `.world`（Gazebo Classic 重启）（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 用户要「每 N 步换一张图」；Gazebo Classic **无**热加载整份 `.world` 的服务 |
| **原因** | Classic 只能换模型/状态，不能 `load_world`；整图切换必须重启 `gzserver` |
| **解决** | `td3_train.yaml` 增加 `world_switch_interval`（默认 **10000**，`0`=不换）。`train_td3_node` 在 `t%N==0` 调用 `ScoutGazeboEnv.switch_world`：停车 → 杀 gzserver → `rosrun gazebo_ros gzserver -e ode <new.world>` → `spawn_urdf_model` 再 spawn → `reset()` 随机起终点。**不清** replay。日志：`[world] switch to xxx.world at t=...`。`world:=` / launch 随机只定**第一张**。限制：换图有数秒中断；roslaunch 原 `/gazebo` 节点会报 died（后续由训练节点自管 gzserver）；GUI 下 gzclient 可能需重连；失败见 `~/.ros/scout_offroad_td3_gzserver_switch.log` |
| **相关命令** | 见下 |

```bash
# 默认每 10000 步换图（yaml）；launch 不传则读 yaml
roslaunch scout_offroad_3d train_td3.launch gui:=false

# 冒烟：CLI 覆盖 interval（不必改 yaml）
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false world_switch_interval:=50 max_timesteps:=200

# 锁死第一张：yaml 设 world_switch_interval: 0，或 launch 传 world_switch_interval:=0
```

---

## 记录 9：换图后 spawn 超时 / 训练假死（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | `t=N` 打出 `[world] switch to xxx.world` 后，小车不再出现；约 120s 后 `timeout waiting for /gazebo/spawn_urdf_model`；其后训练环看似卡住（Ctrl-C 才退出）。`~/.ros/scout_offroad_td3_gzserver_switch.log` 可见 `new node registered with same name` |
| **原因** | ① `kill -9 gzserver` 后 ROS master 上 `/gazebo` 常成**僵尸注册**；立刻再起同名节点时 XML-RPC 端口复用，master 把「同名新节点」关机信号发给**新** gzserver → ROS API 插件退出，`/gazebo/spawn_urdf_model` 永不出现。② `/use_sim_time:=true` 下 gzserver 死后 `rospy.sleep` / 等 pose 会**无限等**仿真钟，表现为算法卡住 |
| **解决** | `ros_env.switch_world`：杀进程后 `rosnode cleanup` + 等待 `/gazebo` 从 `rosnode list` 消失并多留端口释放时间；等待 spawn 服务时轮询子进程存活（早失败）；最多重试 2 次；`_wait_pose` / control sleep 用 wall-clock 看门狗。`train_td3_node`：换图失败则尝试 recovery reload，仍失败则结束训练环（不再盲 step） |
| **相关命令** | 见下 |

```bash
# 短冒烟：50 步换图，总共 200 步，确认换图后仍有 ep= 日志
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
source ~/catkin_ws_2/devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false world_switch_interval:=50 max_timesteps:=200 start_timesteps:=50
# 期望日志含：
#   [world] switch to ....world at t=50
#   [world] spawn OK / switch complete
#   ep=...（换图之后仍继续）
# 失败时看：
tail -50 ~/.ros/scout_offroad_td3_gzserver_switch.log
```

---

## 记录 10：gui 换图后 gzclient 抢 /gazebo（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | `t=10000` 换图后 gzserver 进程仍在（`gz_alive=True`）但 `/gazebo/spawn_urdf_model` 60s 超时、小车不出现；`~/.ros/scout_offroad_td3_gzserver_switch.log` 约 0.4s 内出现 `new node registered with same name`；`/gazebo` 节点只剩 get_loggers 无 spawn |
| **原因** | `gui:=true` 时 `switch_world` 与 gzserver **并行**启动 `rosrun gazebo_ros gzclient` 且**未** `__name:=gazebo_gui` → `libgazebo_ros_api_plugin` 以默认名再占 `/gazebo`，把 gzserver 的 ROS API 顶掉（与记录 9 僵尸节点同类表现，但根因是 gzclient 命名） |
| **解决** | `ros_env.switch_world`：gzclient 延后到 spawn 服务就绪后再启，并固定 `__name:=gazebo_gui`；gzserver 改为与 roslaunch 一致的直接脚本路径；杀进程用 `killpg` + `pkill -f gazebo_ros/*`；spawn 8s 未到则早失败并提示查 switch.log |
| **相关命令** | 见下 |

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
source ~/catkin_ws_2/devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true world_switch_interval:=50 max_timesteps:=200 start_timesteps:=50
# 期望：t=50/100/150 均有 [world] spawn OK / switch complete，换图后 ep= 继续
tail -20 ~/.ros/scout_offroad_td3_gzserver_switch.log   # 不应再出现 name conflict
```

---

## 记录 11：heightmap 换图后小车飞起 / z 偏低（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | `gazebo_heightmap.world` 中 spawn 后车悬空、弹飞或贴地穿模；flat world 正常 |
| **原因** | heightmap bowl 地面 z 非 0；统一 `spawn_z: 0.15` 与地形不匹配 |
| **解决** | `ros_env._ensure_spawn_ok`：**门禁** — 等 0.5s → 检 z/\|vz\|/pose 稳定/min sector；失败自动重 spawn（+z / 换 xy，最多 5 次）；`reset` 全失败则 resample 起终点（最多 3 轮）或 abort；yaml `world_spawn_z.gazebo_heightmap.world: 0.35` |
| **相关命令** | 见 README1 §4.5 基础冒烟；日志关键字 `[spawn] OK` / `[spawn] FAIL` |

---

## 记录 12：碰撞 hard done -25 → 前方扇区软惩罚 + 受困终止（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象（旧）** | 任意扇区 `min_d < collision_dist` 即记碰撞，`done=True`，固定 **`r_collision = -25`**；`rough_ramps` / heightmap 上坡、侧后扫地、贴墙慢移频繁误触，episode 过短，策略难学到绕障与恢复 |
| **原因（旧）** | **全周**扇区距离 + **无速度门槛** + **单帧**判定 + **碰撞即 reset**（hard terminal） |
| **解决（新）** | `ros_env.py` / `td3_train.yaml`：**① 前方扇区** `±collision_sector_half_fov_deg`（默认 90°，侧/后地面忽略）；**② 朝向分速度** `v_app > collision_v_min`（贴墙停住不算撞）；**③ 连续帧** `collision_frames`（默认 4 步）防抖后才记 1 次事件；**④ 碰撞不 reset**，罚 `r_collision_base · 2^(k-1)`（默认 `-1/-2/-4/…`，`collision_penalty_double: true`）；**⑤ 受困** `stuck_time_sec` 内位移 `< stuck_dist_threshold` → **`done` + `r_stuck = -20`**；可选 `max_collision_events > 0` 限次结束局 |
| **新行为日志** | 墙：`[collision] wall confirmed event #k ...`（**不**打 `outcome=collision`）；坡：`[terrain] slope suspected`；受困：`[stuck] displacement=... → done` + `outcome=stuck`；局末 `[reward] ... END success/stuck/timeout` |
| **调参提示** | **坡道/台阶误触**：略增 `collision_dist`（如 `0.32`）或 `collision_v_min`（如 `0.12`），或减 `collision_sector_half_fov_deg`（如 `60`）；**空转/骑墙**：确认 `stuck_time_sec`/`stuck_dist_threshold`，必要时略减 threshold；**真撞太轻**：加大 `\|r_collision_base\|` 或保持递增；**碰撞过多仍不结束**：设 `max_collision_events: 5` |
| **相关命令** | 见 [README1 · 一附 2](README1.md#reward-done-collision)；参数 `td3_train.yaml` 中 `collision_*` / `stuck_*` |

```bash
# 坡道专项冒烟：误触 collision 日志可继续局；空转 ~5s 应 stuck
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_ramps.world \
  world_switch_interval:=0
# 期望：无 outcome=collision 硬结束；可有 [collision] 但局继续；[stuck] 终止
```

---

## 记录 13：16 线 3D 点云 + self-filter + batch 512（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 阶段 A 仍走 2D `/scan`；越野训练与真车 16 线不一致；车身/下视射线易污染扇区 |
| **原因** | `scout_mini.gazebo` 原为单线 `libgazebo_ros_laser.so`；`range_features` 默认 `source:=scan`；无 body 系仰角/包围盒过滤 |
| **解决** | ① `scout_mini.gazebo`：**16 线 ±15°** 垂直、360° 水平，`block_laser` → `/lidar/points`；`min_range=0.18`。② `range_features_node.py`：默认 `source:=cloud`，`frame_mode:=base`，仰角带 + body box self-filter。③ `pointcloud_bridge_node.py`：`/lidar/points` → `/lidar/points2`（PointCloud2）。④ `train_td3.launch` / `range_features.launch` 默认 cloud。⑤ `td3_train.yaml` **`batch_size: 512`**。文档：`README1.md` §一附 0 |
| **相关命令** | 见下 |

```bash
# 仅 Gazebo + 点云
roslaunch scout_mini_pro_description gazebo_room_official.launch gui:=false \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_ramps.world
# 扇区
roslaunch scout_offroad_3d range_features.launch
rostopic hz /lidar/points
rostopic echo /lidar/points -n 1
rostopic echo /offroad/range_features -n 1
# 一键训（已含 cloud 链）
roslaunch scout_offroad_3d train_td3.launch gui:=false max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0
```

**坡道说明**：碰撞经多层判别（记录 14）；坡面多见 `[terrain] slope suspected`；真墙 `[collision] wall confirmed`。

---

## 记录 14：爬坡 vs 撞墙 · 多层仰角 + IMU（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | `rough_ramps` / heightmap 上坡时水平 24 扇区仍近距，记录 12 软碰撞后仍频繁 `[collision] event`，策略把坡面当墙 |
| **原因** | TD3 状态仅 **水平投影 28 维**，无仰角分层；碰撞模块无法区分「多层全挡=墙」vs「低层近、顶层开阔=坡」 |
| **解决** | **方案 B**（当时不改 state_dim）：① `range_features_node.py` 额外发布 `/offroad/elevation_features`（4 仰角桶 × 12 前方扇区 = 48）；16 线 ±15° 按 `elevation_bin_edges_deg` 分桶。② `ros_env._classify_obstacle()`：多层 min + 短时近距 + 可选 `/imu/data` pitch + 前进速度；坡 → `[terrain] slope suspected` + `r_slope_penalty`，**不**递增 collision count；墙 → `[collision] wall confirmed`。③ 参数见 `td3_train.yaml` `terrain_*` / `slope_*` / `wall_*`。**后续记录 15** 增加 `/offroad/td3_range_features`（3×24）进 TD3 状态 |
| **维数** | 记录 14 当时 TD3 **仍 28**（24+4）；**记录 15** 已改为 **76** 维（3×24+4），旧 `td3_actor` **不兼容**，须重训 |
| **文献** | 多线 LiDAR 地形常用手法：elevation grid + slope/roughness（如 [IJARS 2018 traversability](https://doi.org/10.1177/1729881417751530)） |
| **相关命令** | 见下 |

```bash
# 验收多层话题
roslaunch scout_offroad_3d range_features.launch
rostopic echo /offroad/elevation_features -n 1   # 4×12=48
rostopic echo /offroad/range_features -n 1       # 24

# 坡道冒烟
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_ramps.world \
  world_switch_interval:=0
# 期望：上坡 [terrain] slope suspected；顶真墙 [collision] wall confirmed
```

**调参**：坡仍记墙 → 增 `slope_upper_clear_dist` / `slope_max_frames`；真墙不罚 → 降 `wall_all_layers_dist`；关 IMU → `use_imu_pitch: false`。

---

## 记录 15：方案 A 部分 · TD3 76 维状态（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 方案 B 下 TD3 仅 28 维水平扇区，策略「看不见」仰角分层；用户希望感觉层（RL）与判别层（规则）分离，且 TD3 纳入 3 仰角层 |
| **原因** | `/offroad/range_features` 为单一水平投影；`/offroad/elevation_features` 仅 4×12 前方，专供 `_classify_obstacle()` |
| **解决** | **方案 A 部分**：① `range_features_node.py` 新增 `/offroad/td3_range_features`（**3×24=72**，边界 `[-15,-5), [-5,5), [5,12.5)`，不含 +20° 顶层；360° 全周）。② `ros_env._build_state()` 订阅 `td3_features_topic` → **state_dim = 3×24+4 = 76**。③ `_classify_obstacle()` / `_update_collision()` **不变**，仍读 4×12 `/offroad/elevation_features` + 水平 24 `/offroad/range_features`（碰撞扫描）。④ `td3_train.yaml` 增 `n_elevation_bins_td3: 3`、`td3_features_topic`；`infer_td3_node.py` 同步 76 维；网络 hidden 256-256 不改 |
| **话题对照** | TD3 感觉层 `/offroad/td3_range_features` **72**；坡墙判别 `/offroad/elevation_features` **48**；碰撞水平 `/offroad/range_features` **24** |
| **维数** | TD3 **76**；旧 28 维 `td3_actor` **不兼容**，须重训 |
| **相关命令** | 见下 |

```bash
# 编译检查
python3 -m py_compile \
  src/scout_offroad_3d/scripts/range_features_node.py \
  src/scout_offroad_3d/scripts/td3/ros_env.py \
  src/scout_offroad_3d/scripts/train_td3_node.py \
  src/scout_offroad_3d/scripts/infer_td3_node.py

# 验收新话题
roslaunch scout_offroad_3d range_features.launch
rostopic echo /offroad/td3_range_features -n 1    # 3×24=72
rostopic echo /offroad/elevation_features -n 1     # 4×12=48 不变

# 重训（示例）
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false max_timesteps:=100000 start_timesteps:=2000 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_ramps.world \
  world_switch_interval:=10000
```

---

## 记录 16：TD3 科研指标 CSV + 曲线图扩展（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 原 `metrics.csv` 仅 12 列；出图只有 reward/success/collision/stuck 累计曲线，缺终止原因组成、碰撞次数、路径效率等科研常用指标 |
| **原因** | 阶段 A 初版只记最小 episode 日志；碰撞改为非终止后需单独统计 `collision_count` 与窗口碰撞率 |
| **解决** | ① `train_td3_node.py`：扩展 CSV（`outcome`、`collision_count`、累计/滑动窗口 success/stuck/timeout/collision 率、`path_efficiency` 等）；参数 `~metrics_window`（默认 50）。② `ros_env.py`：逐步累计 `path_length`，info 返回 `initial_goal_dist` / `path_efficiency`。③ `plot_td3_metrics.py`：新增 `outcome_*_rate.png`、`outcome_stacked_area.png`、`collision_count.png`、`collision_rate_window.png`、`path_efficiency.png` 等；**旧 CSV 可重绘**。④ 文档：`README1.md` §4.8、本记录 |
| **兼容** | **正在跑的旧进程**仍写旧 12 列；**下一次 launch** 起用新列。旧 run 可用新脚本 `--window 20 --stack-bin 50` 重绘 |
| **相关命令** | 见下 |

```bash
python3 -m py_compile \
  src/scout_offroad_3d/scripts/train_td3_node.py \
  src/scout_offroad_3d/scripts/plot_td3_metrics.py \
  src/scout_offroad_3d/scripts/td3/ros_env.py

# 对已有 run 重绘（旧 CSV 亦可）
python3 src/scout_offroad_3d/scripts/plot_td3_metrics.py \
  ~/catkin_ws_2/3D/runs/run_20260822_175118 --window 20 --stack-bin 50
ls ~/catkin_ws_2/3D/runs/run_20260822_175118/*.png
```

---

## 记录 17：`rough_ramps` 薄板坡道镂空 /  crest 悬崖（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **回滚** | 用户要求回滚 world，自行修改 |
| **现象** | `rough_ramps.world` 坡道看起来像悬崖：侧面看坡下镂空；小车从低端上坡时顶到竖直棱边、爬过 crest 后飞出去翻车；逻辑/坡墙判别正常，属 **Gazebo 地形几何** 问题 |
| **原因** | ① 坡道仅为 **0.15 m 薄倾斜 box**（`surface` 有 collision，**不是** visual-only），坡下无支撑 → 物理与视觉均像「架空的板」。② **`<pose>` 的 z 偏低**（0.18 m）：按 pitch ±0.22 rad 估算，低端顶面在 z≈−0.18 m（埋入 ground_plane），地面车先撞 **竖直端面** 而非顺坡上升。③ **crest 无 landing**：高端顶面约 z≈0.69 m 后直接截断，车越顶即悬空。④ 与 `offroad_map` / heightmap 无关；`rough_ramps.world` **无** `<include>` 外部 mesh |
| **解决** | 重写 `rough_ramps.world` 三个 ramp 为 **实心组合**：`surface`（0.25 m 厚行车面）+ `fill`（坡下实心 box）+ `toe`（z≈0 接地）+ `landing`（crest 平台）；修正 `surface` 中心 z≈0.363 m 使低端贴地；行车面加 friction。详见 README1 [§9.5](README1.md#edit-world-verify) |
| **相关命令** | 见下 |

```bash
# 目视 + 碰撞体（改 world 后必做）
killall -9 gzserver gzclient 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_mini_pro_description gazebo_room_official.launch \
  gui:=true world:=$(rospack find scout_offroad_3d)/worlds/rough_ramps.world
# GUI: View → Collisions；坡下应被 fill 填满，crest 后有 landing

# 查 ramp 模型（Gazebo Classic）
gz model -m ramp_east -p
gz model -m ramp_east -l link -i

# 训练冒烟：spawn 后手动或策略上坡
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_ramps.world \
  world_switch_interval:=0
# 另终端：rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.6}}"
# 期望：不穿坡、不 crest 飞出；[terrain] slope suspected 为主
```

**自行改 world 要点**：每个 `<collision>` 都要有对应 `<visual>`（或接受 debug 只看 collision）。

---

## 记录 18：删除 `rough_ramps` + reward/速度/碰撞调参（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 用户要求开训前多项改动：去掉 `rough_ramps` 地图；提高朝目标前进奖励；episode 起始 +5 分；加强 wall 碰撞识别；速度保守化 |
| **原因** | `rough_ramps` 几何/训练价值不足，与 heightmap/obstacles 重复；旧 reward/速度偏激进，episode 分偏低、碰撞 debounce 偏松 |
| **解决** | ① **删除** `worlds/rough_ramps.world` 及 launch/yaml/README 全部引用；world 池剩 4 张：`clearpath_playpen`、`gazebo_heightmap`、`offroad_map`、`rough_obstacles`。② **Reward**：新增 `r_episode_start: 5.0`（每局**首步**一次，非每步 +5）；`r_progress` 1.5→**3.0**；`r_step` 保持 -0.04。③ **碰撞**：`wall_all_layers_dist` / `wall_lower_layers_dist` 0.35→**0.28**；`collision_frames` 4→**5**。④ **速度**：`max_v` 2.2→**1.5**；`max_w` 1.8→**1.5**（train/infer launch 加载 yaml；`ros_env` action clip 同步）。⑤ infer 默认 world 改为 `rough_obstacles.world` |
| **相关命令** | 见下 |

```bash
python3 -m py_compile \
  src/scout_offroad_3d/scripts/train_td3_node.py \
  src/scout_offroad_3d/scripts/infer_td3_node.py \
  src/scout_offroad_3d/scripts/pick_random_world.py \
  src/scout_offroad_3d/scripts/td3/ros_env.py

killall -9 gzserver gzclient roslaunch rosmaster rosout train_td3_node 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false max_timesteps:=100000 start_timesteps:=2000 world_switch_interval:=10000 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world
```

**仍保留的地图**（记录 18 时）：`clearpath_playpen.world`、`gazebo_heightmap.world`、`offroad_map.world`、`rough_obstacles.world`。（`gazebo_heightmap` 已于记录 21 删除，见下。）

---

## 记录 19：全面重塑 · 碰撞/翻车/goal/reward（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 用户停止第二次大训 `run_20260822_195504`：`collision_count` **全程 0**；大量 **stuck**（211）实为翻车；成功率中期 ~30% 末段掉到 0%；spawn/goal ±3.5 m 过小致 timeout；goal 可能在箱顶 |
| **原因** | ① `_classify_obstacle` 中 **`still_moving→slope`** 抑制 wall；record 18 **`wall_all_layers_dist=0.28`** 过严。② 无 IMU rollover，翻车走 stuck 分支。③ 无 goal 可达性检查。④ reward/范围/timeout 与 76 维长训不匹配 |
| **解决** | 见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md)。**碰撞**：去掉 still_moving _slope_；恢复 `wall_all_layers_dist: 0.32`；坡放宽 `slope_upper_clear_dist: 1.3`、`slope_max_frames: 10`。**翻车**：`rollover_*` + `r_rollover`，grace 2 s。**Goal**：`_validate_goal()`。**范围**：spawn/goal ±6 m，`max_ep_steps: 600`。**Reward**：`r_arrive: 50`、`r_progress: 5`、`r_stuck: -12` |
| **日志关键字** | `[collision] wall confirmed` · `[rollover] roll=... → done` · `[goal] reject` / `[goal] OK` · `outcome=rollover` |
| **相关命令** | 见下 |

```bash
python3 -m py_compile \
  src/scout_offroad_3d/scripts/td3/ros_env.py \
  src/scout_offroad_3d/scripts/train_td3_node.py

# 汇报后再训 — 短冒烟 only
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0
```

---

## 记录 20：第三次大训前 reward/碰撞/噪声精进（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | 用户要求 record 20：借鉴 DRL_Navigation 的 `v-\|ω\|` 前进鼓励；到达 +100；碰撞 -25/-50/-100 且 3 次 reset；碰撞判定更严；stuck 窗口拉长；自适应传感器噪声 |
| **原因** | record 19 重塑后仍未开第三次大训；需更强「朝目标前进」信号、更重到达奖、更严墙撞与终止策略；训练早期 success 低时需适度加噪脱困 |
| **解决** | 见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md) **Record 3 训练前配置**。**Reward**：`r_forward_scale`、`r_spin_penalty`、`r_low_effort_penalty`；`r_arrive: 100`、`r_progress: 7`。**碰撞**：`collision_penalty_list: [-25,-50,-100]`、`max_collision_events: 3`；收紧 `collision_dist/frames/v_min`、`wall_all_layers_dist`。**Stuck**：`stuck_time_sec: 10`。**噪声**：`range_features_node` 增 `sensor_noise_std`；`train_td3_node` `_NoiseAdapter` |
| **日志关键字** | `[reward] ... forward/spin/low_effort` · `[collision] wall confirmed event #3` · `outcome=collision` · `[noise_adapt] success_rate_win=...` |
| **相关命令** | 见下 |

```bash
python3 -m py_compile \
  src/scout_offroad_3d/scripts/td3/ros_env.py \
  src/scout_offroad_3d/scripts/train_td3_node.py \
  src/scout_offroad_3d/scripts/range_features_node.py

# 汇报后再训 — 短冒烟 only
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0
```

---

## 记录 21：删除 `gazebo_heightmap.world`（2026-08-22）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-22 |
| **现象** | `gazebo_heightmap.world` spawn 持续异常（悬空/弹飞/穿模，见记录 11）；`world_spawn_z` 调高仍不稳定 |
| **原因** | Gazebo Classic bowl heightmap 地面 z 非 0，与统一 spawn 门禁/重试仍难匹配 |
| **解决** | **删除** `worlds/gazebo_heightmap.world` 及 launch/yaml/README 全部引用；移除 `world_spawn_z` 中 heightmap 项；world 池剩 **3 张**：`clearpath_playpen`、`offroad_map`、`rough_obstacles` |
| **相关命令** | `rosrun scout_offroad_3d pick_random_world.py -e` 同步 launch 列表 |

---

## 记录 22：第四次大训前碰撞重做 + reward 修复（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | Run3（`run_20260822_233910`）collision_count 全程 0；timeout 46.9% + reward hacking（mean +681）；用户确认第四次大训方向 |
| **原因** | ① 碰撞：4 层全近才判墙，低障碍时 +20° 层照不到 → 顶层开阔误判 **slope**；② `r_forward_scale=1.1` + `max_ep_steps=750` 致 timeout 刷分；③ 主图 success_rate 只看累计掩盖窗口改善 |
| **解决** | 见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md) **Record 4**。**碰撞**：`wall_layers_required: 2`；`wall_force_min_dist: 0.22` 强制墙；`wall_all_layers_dist: 0.32`；`collision_penalty_list: [-25,-50,-100]`。**Reward**：`r_forward_scale: 0.08`、`r_progress: 5`、`max_ep_steps: 500`；`noise_adapt_enable: false`。**出图**：`success_rate.png` 主曲线改滑动窗口 |
| **日志关键字** | `[collision] wall confirmed`（必须能出现）· `[terrain] slope suspected` · `[reward] forward`（量级应 << Run3） |
| **相关命令** | 见下 |

```bash
python3 -m py_compile \
  src/scout_offroad_3d/scripts/td3/ros_env.py \
  src/scout_offroad_3d/scripts/train_td3_node.py \
  src/scout_offroad_3d/scripts/plot_td3_metrics.py

# 汇报后再训 — 短冒烟 only
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0
```

---

## 记录 23：方案 A 碰撞参数终版（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | Record 4 草案 `wall_all_layers_dist=0.32` / `wall_force_min_dist=0.22` 可能对小石子、坡面误触墙；极近强制墙仍受 `collision_frames` / `vx` 门控，快接近时可能先落坡判 |
| **原因** | 层距偏宽 + 强制墙距离偏长；`_classify_obstacle` 极近分支要求 `frames≥collision_frames` 或 `vx≤wall_v_forward_max` |
| **解决** | **方案 A 终版**（保留 elevation）：`wall_layers_required: 2`；`wall_all/lower_layers_dist: 0.27`；`wall_force_min_dist: 0.19`；`collision_dist: 0.28`；`slope_upper_clear_dist: 1.0`；`terrain_classify_enable: true`。代码：极近强制墙去掉帧数/速度门控，仅 `min_d < wall_force_min_dist` → `wall`（事件防抖仍在 `_update_collision`） |
| **日志关键字** | 上坡多见 `[terrain] slope suspected`；真顶墙 `[collision] wall confirmed`；极近贴墙应能触发强制墙 |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py`；训前短冒烟见 [TRAIN_RECORDS](TRAIN_RECORDS.md) Record 4 |

---

## 记录 24：spawn grace 翻车作废（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | spawn 后 2 s 内 IMU 检测到翻车仅被 `rollover_grace_sec` 忽略，坏 spawn 仍计入 episode/metrics |
| **原因** | `_check_rollover` 在 grace 内直接 `return False`，无 reset |
| **解决** | `_check_spawn_grace_flip` + `reset(count_episode=False)`；`train_td3_node` 见 `info.episode_abort` 跳过 replay/metrics/ep++ |
| **日志关键字** | `[rollover] spawn grace flip → abort episode (resample)` · `[train] episode aborted` |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py src/scout_offroad_3d/scripts/train_td3_node.py` |

---

## 记录 25：侧翻漏检 rollover（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | 小车明显侧翻（轮朝上）但无 `[rollover]` 日志、episode 不终止 |
| **原因** | ① IMU 订阅嵌套在 `terrain_classify_enable` 内，关地形分类时 `_roll/_pitch` 恒 0。② 单轴 50° 阈值对斜向翻车（roll/pitch 各 ~35–45°）漏检。③ 无 IMU 时无 fallback |
| **解决** | IMU 订阅独立于地形分类；阈值 40° + `\|roll\|+\|pitch\|≥55°`；`get_model_state` 姿态 fallback；grace/未触发时 debug throttle 打 roll/pitch |
| **日志关键字** | `[rollover] roll=... → done` · `[rollover] spawn grace flip roll=...` · `[rollover] tilt roll=...`（debug） |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py`；侧翻时 `rostopic echo /imu/data` 看 `orientation` |

---

## 记录 26：换图后 rollover 消失（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | 首图训练 rollover 正常；`world_switch` 换图后侧翻无 `[rollover]` 日志、episode 不终止 |
| **原因** | ① `switch_world` / `reset` 未清零 `_imu_received` / `_roll` / `_pitch`，换图后仍用 respawn 前旧 IMU 缓存（常为直立 0°），`_rollover_angles_deg` 不再走 model fallback。② gzserver 重启后 IMU publisher 换新，原 `rospy.Subscriber` 可能收不到新 `/imu/data` |
| **解决** | `_reset_imu_state()`（reset + switch_world 后调用）；`_ensure_imu_subscriber()` 重订阅；`_imu_fresh()` + `imu_stale_sec`（默认 1s）过期则 fallback `get_model_state`；`_wait_imu_after_respawn` 换图后等首帧 IMU 并打 `[imu]` 日志 |
| **日志关键字** | `[imu] first message on /imu/data after respawn` · `[imu] no data on /imu/data after 3.0s — rollover uses model fallback` · `[rollover] roll=... imu=ok/fallback` |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py`；换图前后 `rostopic hz /imu/data`；侧翻时 `rostopic echo /imu/data` |

---

## 记录 27：换图崩溃停训 + IMU 信号断续（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | Record 5 @ t=120k 换图 Gazebo 连接拒绝后训练环 `break` 停训；换图后 rollover 仍可能漏检（IMU 首帧 upright 0° 但已 stale / 未重订阅） |
| **原因** | ① `switch_world` 仅重试 2 次，kill/spawn 等待偏短。② `train_td3_node` recovery 失败直接 `break`。③ 换图后 IMU 可能短暂发直立 0° 帧使 `_imu_fresh()` 为真，不走 model fallback |
| **解决** | `world_switch_max_retries: 5` + 加长 kill/boot/spawn/retry sleep；`_log_imu_post_switch_status` 每换图打 `[imu] ok/stale/fallback`；`imu_model_fallback_after_switch_steps` 换图后 N 步强制 model；`imu_wait_after_respawn_sec: 8`；recovery 失败 **skip switch 继续训**（保持旧 world） |
| **日志关键字** | `[imu] ok/stale/fallback after world switch` · `[world] reload attempt N/5` · `[world] recovery FAILED ... continuing on current world` |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py src/scout_offroad_3d/scripts/train_td3_node.py` |

---

## 记录 28：Cardiff 式 occupancy 探索奖励 r_explore（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | Record 5 stuck≈47%；用户同意加 Cardiff 式栅格探索，但值须极小，且不能盖过 progress/step |
| **原因** | 纯 progress shaping 在 local minimum 无脱困激励；Cardiff 用 novelty，我们简化为每局 occupancy 首次访问 |
| **解决** | `ros_env.py`：每局 `_explore_cells` reset；`explore_grid_res` 栅格；**首次进入** `+explore_cell_reward`（默认 **0.03**），重访 0；spawn 格预标记；可选 `explore_cardiff_scale` → `/ max(ρ, explore_rho_min)`。yaml / metrics `r_explore_sum` / plot 已同步 |
| **标定** | 背离 0.2 m/步：`r_progress` −1.4 + `r_step` −0.03 vs explore ≤0.03 → 惩罚约为探索 **47×**（>>3–5× 要求）。见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md) Record 6 标定表 |
| **日志关键字** | `[reward] ... explore +0.0300`（新格时）· 回旧格无 explore 行 |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py src/scout_offroad_3d/scripts/plot_td3_metrics.py` |

---

## 记录 29：launch 空 `max_timesteps` 覆盖 yaml 致节点崩溃（2026-08-23）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | 续训/重启后 `train_td3_node` 立即 exit code 1，小车不动；终端 `ValueError: invalid literal for int() with base 10: ''` |
| **原因** | `train_td3.launch` 中 `max_timesteps` / `start_timesteps` 默认 `""` 并写入 ROS 参数，**覆盖** `td3_train.yaml`；`int('')` 在 `ScoutGazeboEnv()` 创建前即崩溃。**与 IMU 改动无关** |
| **解决** | launch 仅当 CLI **显式传入非空值**时才覆盖 yaml（`if="$(eval arg('max_timesteps') != '')"`）；`train_td3_node.py` 新增 `_get_param_int` 等 helper，空串回退 yaml 默认。**日常调参只改 `td3_train.yaml`** |
| **日志关键字** | `* /train_td3_node/max_timesteps:` 为空 · `[train_td3_node-10] process has died ... exit code 1` |
| **相关命令** | 正常训：`roslaunch scout_offroad_3d train_td3.launch gui:=false`（不传 `max_timesteps:=`）；见 README1 §4 |

---

## 记录 30：DEM 世界入池 + 文档与 yaml 对齐（2026-08-23 · Record 6 训前）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | 已加 `yosemite_heightmap.world` / `volcano_dem.world` 与 `SOURCES.md`，但 README/TRAIN 仍写 Record 4 reward 默认；`train_td3.launch` 随机池曾仅 3 张；DEM 需 `world_spawn_z` 尚未写入 yaml |
| **原因** | 代码/地图迭代快于文档；launch `$(eval)` 列表需与 `worlds/*.world` 手工同步 |
| **解决** | ① `train_td3.launch` 随机池已含 **5 张**（`pick_random_world.py -e`）。② DEM **短冒烟 + `world_spawn_z`** 后再长训；Yosemite 过陡时可从 launch 列表移除。③ 三份文档与 `td3_train.yaml` Record 6 对齐 |
| **相关命令** | `rosrun scout_offroad_3d pick_random_world.py -e`；DEM 冒烟见 [README1 §4.5](README1.md#td3-train) |

---

## 记录 31：DEM 山地形否决并删除（2026-08-23 · Record 6 训前）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-23 |
| **现象** | 用户实测 `yosemite_heightmap.world`、`volcano_dem.world`：**坡度过陡、尺度过大**（无人机/航拍级 DEM）；Scout 小车无法在此地形有效训练 |
| **原因** | 真实 DEM 为高差数十至百米级山岳；非地面车小范围缓坡需求 |
| **解决** | **删除** 两 `.world`；`train_td3.launch` 随机池回 **3 张**；`SOURCES.md` / `README1` §4.5·§9 / `TRAIN_RECORDS` 标记否决；Web 调研结论：**需自建** 20–50 m 缓坡 heightmap |
| **相关命令** | 候选与自建方案见 [`worlds/SOURCES.md`](../src/scout_offroad_3d/worlds/SOURCES.md) |

---

## 记录 32：换图 spawn 仍用旧 world identity → 假 z_jump → 锁死 playpen（2026-08-28 · Record 6）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-28 |
| **现象** | Record 6 整夜 4000+ ep / ~260k 步窗口成功率可到 ~90%，但 **只有 `clearpath_playpen`**。每 20k 步换图 12/12 失败后 `recovery OK → playpen`。用户 t=260000 日志：Gazebo 已加载 `bumpy_offroad`，spawn 仍打印 `z=0.200 (world=clearpath_playpen.world)`，`z_jump z=1.067 target_z=0.200`。`[rollover] imu=fallback` 在 playpen 上也会出现 |
| **原因** | ① `switch_world()` 在 `_spawn_robot()` **成功之后**才写 `current_world`。`_spawn_z_for_current_world()` / 校验 / 日志仍是旧图。yaml `world_spawn_z.bumpy_offroad=1.05` 换图时未用上。heightmap z≈1.06 vs 错误 target 0.20 → 假 z_jump；z 重试最高 0.75，够不到 1.06。② trainer recovery 读 `env.current_world`（仍是 playpen）→ **不会重试 bumpy**，只把已加载的 heightmap 杀掉再起空 playpen。③ 5 轮假失败后再换 `offroad_map`/`rough_obstacles` 易 Connection reset/refused，同样回 playpen。④ recovery 重启 gzserver 触发 `imu_model_fallback_after_switch_steps=50`，故 playpen 上也打 `imu=fallback`。相对 Record 5：R5 是连接拒绝后 **停训**；R6 retries 把失败包装成继续训，整晚锁死同一张图 |
| **解决** | `ros_env._apply_world_identity(world_path)`：**spawn 校验前**绑定 `current_world` + `spawn_z`。bumpy 用 1.05；`|1.067−1.05|=0.017 < 0.25` 不应 z_jump。`_check_spawn_stable` 若实测 z 已贴近 `world_spawn_z` 则不因过期 `target_z` 误杀。换图失败则 **restore 旧 identity** 再 `fallback … NOT a map change`（禁止 `recovery OK` 冒充换图成功）。`plot_td3_metrics`：黄线=真换图，灰线=失败/回退；`switch FAILED` / `fallback` **覆盖**同 t 的 `switch to`（那是尝试开始，不是成功） |
| **地图** | `clearpath_playpen`（offroad 与 gazebo_sim **字节一致**，mtime 08-22 后再未改）= stock 围栏空场，**未被 recovery 写盘覆盖**。±7 m 内约 10 个 pose（多贴边）；`offroad_map` ±7 m≈19；`rough_obstacles` ±7 m≈36 且 mtime **08-28 11:29（开训之后）**。三张密/越野图本 run **从未成功训到**。`uneven_ground` 模型在。次要：`stuck_collision` at (-5.18,-5.72) 再 retry OK |
| **日志关键字** | `[world] identity → bumpy_offroad.world spawn_z=1.050` · `[spawn] OK world=bumpy_offroad` · `[world] fallback to … NOT a map change` · 不应再出现 `spawning ... z=0.200 (world=clearpath_playpen)` 去对 bumpy |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py src/scout_offroad_3d/scripts/train_td3_node.py src/scout_offroad_3d/scripts/plot_td3_metrics.py`。**不要**用本 run 权重当越野结论。bumpy 换图须短冒烟，勿直接长训 |

---

## 记录 33：碰撞阈值过松 · Record 6 事后收紧（2026-08-28）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-28 |
| **现象** | Record 6 大量 `coll_ev=0`；3843 ep 仅 collision_count=19（10 局）、`outcome=collision` 3 局。用户要求 **更严、更容易触发**（不是再放宽） |
| **原因** | 主因是空 playpen（记录 32）上几乎贴不到墙。次因：本 run 仍用 Record 5 档 `collision_dist=0.34` / `wall_all_layers_dist=0.31` / `wall_force_min_dist=0.25` / `collision_frames=3` / `collision_v_min=0.07`。R5 密一些的平地能撞 100 次；空场 + 高 success 把碰撞测没了 |
| **解决** | **保守收紧**（不改分类器结构）：见下表。`wall_layers_required` 仍为 2（降到 1 易把坡当墙）。空图上收紧也救不了已训权重 |
| **相关命令** | 改的是 `td3_train.yaml`；重启 roslaunch 生效。E 表见 README1 |

| 参数 | Record 5 / Record 6 本 run | **收紧后** | 效果 |
|------|---------------------------|-----------|------|
| `collision_dist` | 0.34 | **0.38** | 更远进入碰撞候选 |
| `wall_all_layers_dist` | 0.31 | **0.36** | 更容易层近 → 墙 |
| `wall_lower_layers_dist` | 0.28 | **0.33** | 低层近距更敏感 |
| `wall_force_min_dist` | 0.25 | **0.30** | 稍远即强制墙 |
| `wall_layers_required` | 2 | **2**（保持） | — |
| `collision_frames` | 3 | **2** | 少一帧防抖 |
| `collision_v_min` | 0.07 | **0.05** | 慢蹭也算接近 |

---

## 记录 34：R6 offroad/rough 换图失败 · world 里嵌了第二辆车（2026-08-28）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-28 |
| **现象** | Record 6 换到 `offroad_map` / `rough_obstacles` 时 gzserver Connection reset/refused / lock_error；再 `spawn_urdf` 官方车失败，recovery 回到 playpen |
| **原因** | 两张图曾从 Gazebo **连车带 `<state>` 一起另存**：world 内已有完整 `scout_mini`（mesh + skid_steer 插件 + 车轮 pose）。训练再 spawn 同名车 → 双车。`bumpy_offroad` / `clearpath_playpen` 无此嵌车 |
| **解决** | 已从 `offroad_map.world`、`rough_obstacles.world` 删除嵌套 `scout_mini` 模型及其 `<state>` pose，地形/障碍保留。四张训练图 `grep scout_mini` 现为空。未加新障碍（下一步再做） |
| **相关命令** | `grep -n scout_mini src/scout_offroad_3d/worlds/*.world` 应无输出。**不要**在 Gazebo 里 Save World 时把官方车写进 `.world` |

---

## 记录 35：spawn 失败 3 次误杀整段训练（2026-08-29）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-29 |
| **现象** | 用户过夜命令 `gui:=false max_timesteps:=500000 start_timesteps:=4000 world_switch_interval:=20000`，跑到 **ep=13 / t=3254** 后停。日志：`bumpy_offroad` 上 `[spawn] ABORT after 6 attempts (high_vz/z_unstable)` × 3 full_try → `[spawn] post-episode reset failed — stop train loop` → `Training finished`。**不是** 900 步冒烟，也不是 Ctrl+C / Gazebo 崩。密林/heightmap 卡 spawn 是预期 |
| **原因** | 我们自己的默认：`spawn_full_reset_retries: 3` + `reset()` 抛 `RuntimeError` + trainer `break`。用户**从未**要求「失败 3 次就停训」 |
| **解决** | `reset()` 随机换 **≥40 个 (x,y)**，失败只 `return None`（跳过本局再采样）。trainer `_reset_until_ok` **永不**因 spawn 停训。换图后在**新图**上重试位姿，不 raise、不回锁 playpen。`spawn_xy_max_retries: 40` |
| **相关命令** | `python3 -m py_compile src/scout_offroad_3d/scripts/td3/ros_env.py src/scout_offroad_3d/scripts/train_td3_node.py`。长训仍用过夜命令（见下），不要 `max_timesteps:=900` |

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false max_timesteps:=500000 start_timesteps:=4000 world_switch_interval:=20000
```

---

## 记录 36：spawn 本图试尽应换图，而不是停训（2026-08-29）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-29 |
| **现象** | 用户认为 bumpy 上 spawn 失败几次后应 **SWITCH MAP**，整段 abort 很奇怪。记录 35 已去掉 FATAL；但换图仍只按 `t % world_switch_interval`，本图 pose 试尽不会切池 |
| **原因** | 旧路径：`spawn_z` 约 3 次 FAIL → `spawn_full_reset_retries: 3` → `RuntimeError` → trainer `break`。40-retry 之后仍只在当前 world 重采样 |
| **解决** | 本图 ≥40 个 XY 仍失败 → `[spawn] giving up this world=X after N pose tries → switch to Y` → `pick_next_world(exclude=current)` + `switch_world`（`[world] identity` + spawn）。整池一轮都失败：skip episode / wait / 再扫池。**永不**因 spawn 退出进程 |

---

## 记录 37：训练不得自行终止 + 换图加速（2026-08-29）

| 项 | 内容 |
|----|------|
| **日期** | 2026-08-29 |
| **现象** | ① `run_20260829_002132` 仅 ep≈13：旧代码 spawn 约 3 次 FAIL → trainer `break` 整段停（用户未 Ctrl+C）② 换图体感 ~10s+：换图前固定等满 `world_switch_gzserver_boot_sec=12` 再 poll spawn |
| **原则** | **除用户手动停 / `max_timesteps` 跑满 / ROS shutdown 外，训练进程不得退出** |
| **退出路径审计** | |
| | **安全（保留）**：`t >= max_timesteps` 正常结束；`rospy.is_shutdown()`（Ctrl+C）；`resume:=true` 且无 checkpoint/metrics 时 **启动即 return**（非训到一半） |
| | **已加固**：spawn/reset 异常 → `_reset_until_ok` 无限重试池；本图 pose 用尽 → `switch_world`（记录 36）；定时换图失败 → log `switch FAILED`，**不**锁 playpen、**不** break；`step` 非 shutdown 的 `RuntimeError` → reset 后继续 |
| | **单局跳过（非停训）**：`episode_abort`（pose 丢/ spawn grace flip）；整池 spawn 失败 → skip 本局 + 2s wait |
| **换图耗时** | 去掉换图前 **固定等满 boot 窗口** 的盲等；`_spawn_service_ready()` 就绪即跳出 boot poll。yaml：`kill_wait 1.0→0.5s`，`gzserver_boot 12→8s`（上限，通常 2–5s 即出 spawn svc），`spawn_wait 90→35s`（仍保留 8s fail-fast），`retry_sleep 2→1s`；spawn 后 settle `1.5→0.6s`；定时换图失败会 **连试池内最多 4 张图** |
| **须重启 launch** | Python/yaml 改动 **必须** 重新 `roslaunch train_td3.launch` 才生效 |

---

## 记录模板（后续追加）

```markdown
## 记录 N：标题（YYYY-MM）

| 项 | 内容 |
|----|------|
| **日期** | |
| **现象** | |
| **原因** | |
| **解决** | |
| **相关命令** | |
```
