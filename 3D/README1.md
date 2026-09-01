# 3D · README1 — 阶段 A 操作手册（扇区 + TD3）

前提：官方车 2D 已通（[`2D/README_4.md`](../2D/README_4.md)）。  
**计划 / 阶段 / 块1·块2** 只看 [`README0.md`](README0.md)；**本文 = 怎么跑 + TD3/选型逻辑**。  
踩坑：[`DEBUG_RECORDS.md`](DEBUG_RECORDS.md)。

| 项 | 说明 |
|----|------|
| 包 | `src/scout_offroad_3d` |
| 阶段 A 一键训练 | `roslaunch scout_offroad_3d train_td3.launch` |
| 阶段 A 推理 | `roslaunch scout_offroad_3d infer_td3.launch weights:=...` |
| 默认超参 | `src/scout_offroad_3d/config/td3_train.yaml`（**日常只改此文件**；launch 仅 CLI 显式传参时覆盖） |
| 默认训练产物 | `~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS/`（可用 `output_dir:=` 覆盖） |
| **阶段 A 不要** | SLAM、`map.yaml`、Arena、A\*、块2 整栈（**以后再做**） |

锚点：[3D 越野 vs 2D](#offroad-3d-vs-2d) · [逻辑与论文](#td3-logic) · [碰撞/受困/超时](#reward-done-collision) · [参数在哪改](#where-to-edit) · [训练](#td3-train) · [出图](#td3-plots) · [超参表](#td3-hparams) · [**打开新地图**](#open-new-terrain) · [越野世界](#offroad-worlds) · [分别打开各世界](#open-each-world) · [GUI 覆盖保存](#save-overwrite-world) · [训练逻辑现状](#train-logic-status) · [短跑→服务器](#smoke-to-server) · [推理](#td3-infer) · [真车部署](#real-robot) · [故障](#td3-troubleshoot) · [选型拍板](#stack-choice)

---

<a id="offroad-3d-vs-2d"></a>

## 一附 0、3D 越野定位 · 16 线雷达 · 与 2D 区别

| 项 | **2D 阶段（`2D/README_4`）** | **3D 越野阶段 A（本文）** |
|----|------------------------------|---------------------------|
| 感知 | 单线 `/scan`（LaserScan） | **16 线** `±15°` 垂直 FOV，360° 水平，`/lidar/points`（Gazebo `block_laser` → `sensor_msgs/PointCloud`；可选 bridge → `/lidar/points2` PointCloud2） |
| TD3 状态 | 24 扇区 + ρθ + 上拍 = **28 维** | **76 维**（方案 A 部分）：3 仰角 × 24 全周扇区 + ρθvw；**另发** `/offroad/elevation_features`（4×12=48）专供坡/墙判别 |
| 仿真世界 | 室内 room | `scout_offroad_3d/worlds/*.world` 越野/坡道/障碍 |
| 碰撞机制 | 前方扇区 + 速度 + 防抖（记录 12） | 记录 12 + **坡/墙分层**（记录 14/19）；真墙 `[collision] wall confirmed`；**翻车** IMU rollover 独立于 stuck（记录 19） |

**传感器配置**（`scout_description/urdf/scout_mini.gazebo`）：`laser_link` 上 ray 传感器，16 vertical × 720 horizontal；`min_range=0.18`；插件 `libgazebo_ros_block_laser.so` → `/lidar/points`。

**方案 A 部分（当前）**：TD3 **感觉层** 读 `/offroad/td3_range_features`（**3×24=72** 扇区 min，仰角边界 `[-15,-5), [-5,5), [5,12.5)`，标签约 **-10°/0°/+10°**，**不含**顶层 +20°）；拼 ρθ + 上拍动作 → **state_dim = 3×24+4 = 76**。**判别层**（坡/墙）仍读 `/offroad/elevation_features`（4×12=48）+ `_classify_obstacle()`，逻辑与方案 B 相同。**旧 28 维 `td3_actor` 权重不兼容，必须重训**。

**多层仰角 — TD3（感觉）**（`/offroad/td3_range_features`）：3 仰角桶 × **360° 全周 24 扇区** min range；边界同上，**排除** `[12.5,15]` 顶层。

**多层仰角 — 判别（规则）**（`/offroad/elevation_features`）：16 线 ±15° 垂直 FOV 按边界 `[-15,-5), [-5,5), [5,12.5), [12.5,15]` 分 **4** 桶（标签约 **-10° / 0° / +10° / +20°**）；每桶在 **前方 ±60°** 内再分 12 扇区取 min range。Gazebo 配置见 `scout_mini.gazebo`（16 vertical × 720 horizontal）。

**兼容话题**：`/offroad/range_features`（24 维水平投影）仍发布，供碰撞水平扫描与旧脚本对照；TD3 训练/推理默认改订阅 `/offroad/td3_range_features`。

**Self-filter**（`range_features_node.py`）：`min_range`；TD3 水平带 `elevation_min/max_deg`（默认 ±10°）；可选 **body 包围盒**（`filter_body_box`）忽略打到白箱/底盘的近距离点。

**爬坡 vs 撞墙（记录 14 / Record 4 配置；Record 6 事后收紧距离）**：`ros_env._classify_obstacle()` 读多层 min + 可选 IMU pitch。**坡** → `[terrain] slope suspected`，轻罚 `r_slope_penalty`，**不**递增 collision count；**墙** → `[collision] wall confirmed`，走递增罚分。方案 A：**≥ `wall_layers_required`（2）层**仰角近距即判墙；**`min_d < wall_force_min_dist`（**0.30 m**）** 无条件强制墙（绕过坡判；防抖仍在 `_update_collision` 的 `collision_frames`）。

**batch_size**：`td3_train.yaml` 已设 **`batch_size: 512`**（replay  minibatch）。

**改 TD3 网络大小（默认不改）**：hidden 宽度在 `scripts/td3/TD3.py`：

| 网络 | 行号（约） | 默认 |
|------|-----------|------|
| Actor | `l1=Linear(state, **256**)`，`l2=Linear(256, **256**)`，`l3=Linear(256, action)` | 256-256 |
| Critic Q1/Q2 | `l1/l4=Linear(state+action, **256**)`，`l2/l5`，`l3/l6` | 256-256 |

改完 hidden **必须重新训练**；旧 `td3_actor` 权重不兼容。  
**改 state / 扇区数**：`td3_train.yaml` 的 `n_sectors`、`n_elevation_bins_td3`、`max_range` + `range_features.launch`；`ros_env.py` → `state_dim = n_elevation_bins_td3 * n_sectors + 4`（默认 **76**）。改维数或 hidden **必须重新训练**；旧 28 维权重不兼容。

**TD3 能否上越野？** 客观一句：**仿真里可以学局部避障 + 朝随机目标走；76 维含 3 仰角层但仍无 IMU/全局图，真车复杂越野需 sim-to-real 与块2 分层规划，阶段 A 只是可训骨架，不是量产越野导航。**

---

## 一、强制边界（先读再开终端）

1. **阶段 A = 块1**：Gazebo world + **16 线 3D 点云** `/lidar/points` → 24 扇区 + TD3 → `/cmd_vel`（**不是** 2D `/scan` 默认链）。  
2. **不要**预先建图 / 加载 `map.yaml` / 开 `nav_all_in_one*` / Arena / A\*。  
3. **块2**（建图/SLAM + Arena 子目标 + 粗全局）在阶段 B/C，**不阻塞**现在训 TD3。  
4. 训练用 **官方车** + `gazebo_room_official.launch`（已由 `train_td3.launch` / `infer_td3.launch` include）。**训练默认**在启动时从包内 `worlds/*.world` **随机抽第一张**；若 yaml 里 `world_switch_interval > 0`（默认 **20000**），之后每 N 个训练 timestep 再换一张（重启 gzserver；**不清** replay）。`world:=` 只定第一张；`interval=0` 则整场锁死第一张。

---

<a id="td3-logic"></a>

## 一附、TD3 逻辑深描（状态 / 动作 / 训练环 / 对照开源）

### A. 本项目具体维度（与 `ros_env.py` / `td3_train.yaml` 一致）

| | 符号 | 维度 / 范围 | 来源 |
|--|------|-------------|------|
| **状态** | \(s\) | **76** | `n_elevation_bins_td3(3) * n_sectors(24) + 4` |
| TD3 扇区距离 | \(d_{b,i}\) | 3 仰角桶 × 24 全周扇区 min range，再 `/ max_range` → \([0,1]\) | `/offroad/td3_range_features`（72 维） |
| 水平投影（**不进 state**） | 24 | 近水平带 min range | `/offroad/range_features`（碰撞水平扫描） |
| 多层仰角（**不进 state**） | 4×12 | 每仰角桶 × 前方扇区 min range（m） | `/offroad/elevation_features`（碰撞/坡墙判别） |
| 目标极坐标 | \(\rho,\theta\) | \(\rho/10\) 截断到 1；\(\theta/\pi\) | Gazebo 真值或 `/odom` 相对随机目标 |
| 上拍动作 | \(v_{\mathrm{prev}},\omega_{\mathrm{prev}}\) | 分别 `/ max_v`、`/ max_w` | 环境缓存 |
| **动作** | \(a=(v,\omega)\) | **2**；\(v\in[-0.15,max_v]\)，\(\omega\in[-max_w,max_w]\)（yaml 默认 `max_v=2.0`、`max_w=1.5`） | → `/cmd_vel`（`linear.x` / `angular.z`） |

奖励（默认，**Record 5/6 基线** · 见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md)）：每局首步 `+r_episode_start(4.5)`；靠近 `+r_progress·Δρ(7)`；到达 **+120**；每步 `+r_forward_scale·max(0,v-|ω|)`（**0.13**）；**探索** 新 cell `+r_explore`（**0.03**，Record 6）；原地转 `r_spin_penalty`；墙撞 **-25/-50/-100**（第 3 次 **done**）；**翻车** `r_rollover=-60`；路径受困 `r_stuck=-70`；**超时** `r_timeout=-30`；每步 `-0.03`。  
Episode 结束：`success` / **`rollover`** / `stuck` / `timeout` / **`collision_limit`（3 次墙撞）**。**判定细节**见下节 [碰撞 / 受困 / 超时](#reward-done-collision)。

### B. 训练环：reset → step → reward → replay → update

与 Fujimoto 开源一致的外环，ROS 里由 `train_td3_node.py` + `ScoutGazeboEnv` 实现：

```text
state = env.reset()                    # 随机起点/目标；set_model_state；发 /offroad/train_goal
for t in 1..max_timesteps:
  if t < start_timesteps:  a ~ Uniform      # 纯随机探索
  else:                    a = π(s) + noise # 策略 + 探索噪声
  s', r, done, info = env.step(a)           # 发 /cmd_vel；sleep control_dt；算 reward
  replay.add(s, a, s', r, done_bool)        # 超时 done_bool=0（可 bootstrap）
  if t >= start_timesteps:  policy.train()  # 批采样；双 Q；延迟 actor；目标平滑
  if done:  log；state = env.reset()
  每 save_freq 步：save td3_actor / td3_critic
```

对应文件：`scripts/td3/TD3.py`（网络与 update）、`replay_buffer.py`、`ros_env.py`（reset/step）、`train_td3_node.py`（外环）。

### C. 对照：sfujim TD3 / AUV DRL / Arena → 我们怎么改成 Scout 差速

| 来源 | 他们怎么操作 / 状态怎么喂 | 本项目改编 |
|------|---------------------------|------------|
| **[sfujim/TD3](https://github.com/sfujim/TD3)** | Gym 连续控制：`env.reset/step`；`state`/`action` 为向量；Actor→动作；Twin Critic；`policy_freq` 延迟更新；目标动作加噪声再 clip | **原样移植**网络与 replay（**MLP 全连接**，不是 CNN）；`max_action` 改为向量 `(max_v, max_w)`；权重前缀 `td3`（无 `.pth` 后缀）。**不**拷贝任何预训练 AUV 权重 |
| **[DRL-AUV-navigation](https://github.com/lanyangyang-gogogo/DRL-AUV-navigation)** | ROS+Gazebo episode：置位姿 → 读传感器状态 → 发速度指令 → reward/done；AUV 常为 3D 速度/姿态 | **只借环结构**；载体改地面 skid-steer；指令改 `/cmd_vel`；状态改扇区+\((\rho,\theta)\)+上拍；模型名 `scout_mini`；**reward / v_max / state 维是 Scout 在 yaml 里设的，不是 AUV 原样搬来** |
| **[Arena-Rosnav](https://github.com/ignc-research/arena-rosnav)** ([arXiv:2104.03616](https://arxiv.org/abs/2104.03616)) | 全局规划（如 A\*）→ **中间规划器**按空间前瞻取 **local sub-goal** → DRL 局部跟点子目标 | **阶段 A 未接**；训练目标是环境内随机点（`/offroad/train_goal`）。**阶段 B** 才发 `/offroad/local_goal` 喂给同一 TD3 的 \((\rho,\theta)\) |

阶段 A 等价于 Arena 里「train_mode 时整段终点当唯一子目标」的简化版；长距离分层要等块2。

### D. 可摘论文原句（写综述/论文用 · 标明出处）

**Fujimoto et al., TD3（ICML 2018）** — [arXiv:1802.09477](https://arxiv.org/abs/1802.09477)：

> In value-based reinforcement learning methods such as deep Q-learning, function approximation errors are known to lead to overestimated value estimates and suboptimal policies. We show that this problem persists in an actor-critic setting and propose novel mechanisms to minimize its effects on both the actor and the critic. Our algorithm builds on Double Q-learning, by taking the minimum value between a pair of critics to limit overestimation. We draw the connection between target networks and overestimation bias, and suggest delaying policy updates to reduce per-update error and further improve performance.

本仓库对应实现点：twin critic 取 min、delayed policy update（`policy_freq: 2`）、target policy smoothing（`policy_noise` / `noise_clip`）。

**Kästner et al., Arena-Rosnav（2021）** — [arXiv:2104.03616](https://arxiv.org/abs/2104.03616) 摘要级：

> … deep reinforcement learning approaches are not suitable for long-range navigation due to their proneness to local minima and lack of long-term memory … we propose a navigation system incorporating deep-reinforcement-learning-based local planners into conventional navigation stacks for long-range navigation.

方法侧（中间子目标，摘要/正文一致表述）：用 A\* 等得全局路径后，按机器人位置与 look-ahead 距离取 **moving local sub-goal**，再交给 DRL 局部规划器——这正是本项目块2 → `/offroad/local_goal` → 块1 TD3 的模板。**现在阶段 A 没有该中间层。**

更多摘录见 [`article/record.md`](../article/record.md) 论文2 / 论文4。

---

<a id="reward-done-collision"></a>

## 一附 2、碰撞 / 受困 / 超时（reward & done）

实现：`ros_env.py` 的 `step()` → `_classify_obstacle()` + `_update_collision()` / `_update_stuck()`；参数：`td3_train.yaml` 中 `terrain_*`、`collision_*`、`stuck_*`。踩坑：记录 12（软碰撞）、**记录 14**（坡/墙）。

### A0. 爬坡 vs 撞墙（多层 + IMU，2026-08-22）

| 信号 | 撞墙倾向 | 爬坡倾向 |
|------|----------|----------|
| 多层 front-min **全低** | ✓ | |
| 仅低/中层低，**顶层**（≈+20°）远 | | ✓ |
| `v_forward` 持续低 + 多层近 | ✓ | |
| 近距仅持续 `< slope_max_frames` 帧 | | ✓ |
| IMU **pitch 上升**（可选） | | ✓ |
| 车仍能前进（`v_fwd > wall_v_forward_max`） | | ✓ |

- **坡**：`[terrain] slope suspected ...`；`r_slope_penalty`（默认 -0.2）；**不** `collision_event++`
- **墙**：`[collision] wall confirmed event #k ...`；原 `r_collision_base·2^(k-1)` 递增罚分

话题：`/offroad/elevation_features` layout = `elevation_bin(4) × front_sector(12)`。360° 全周仍只用于 TD3 的 24 扇区；多层特征 **仅前方 ±** `front_half_fov_deg`（默认 60°，launch 可调）。

### A. 三种结束与惩罚（对照表）

| 类型 | 是否 `done` | 默认惩罚 | 判定要点 | 日志关键字 |
|------|-------------|----------|----------|------------|
| **到达（success）** | 是 | `+r_arrive`（**120**） | \(\rho < goal\_tol\)（0.55 m） | `END success` |
| **翻车（rollover）** | 是 | `r_rollover`（**-60**） | IMU \|roll\|/\|pitch\| ≥ 40° 或合角 ≥ 55°，且 ep 时长 **> 2 s** | `[rollover] ... → done` |
| **spawn grace 翻车** | 否（作废重开） | 无 | ep 时长 **≤ 2 s** 且姿态超阈值 → **reset 重采样**，不计入 episode/metrics | `[rollover] spawn grace flip → abort episode (resample)` |
| **受困（stuck）** | 是 | `r_stuck`（**-70**） | 滑动窗口 **6 s** 内位移 \(<\) 0.25 m | `[stuck] ... → done` |
| **超时（timeout）** | 是 | `r_timeout`（**-30**） | `step ≥ max_ep_steps`（**500**） | `END timeout` |
| **碰撞（wall）** | **第 3 次是** | **-25 / -50 / -100** | 多层判别为 **墙** + 防抖；见 A0 | `[collision] wall confirmed` |
| **前进鼓励** | 否 | `r_forward_scale·max(0,v-\|ω\|)` | 每步，参考 DRL_Navigation | `[reward] forward ...` |
| **原地转** | 否 | `r_spin_penalty` | v 低且 \|ω\| 高 | `[reward] spin ...` |
| **低前进量** | 否 | `r_low_effort_penalty` | 持续低 v−\|ω\|（默认 **2.8 s**） | `[reward] low_effort ...` |
| **坡道（slope）** | 否 | `r_slope_penalty`（**-0.05**） | 多层/短时/ pitch / 仍前进 | `[terrain] slope suspected` |

**与旧 hard collision 对比**（记录 12 前）：

| | 旧（hard） | 新（当前默认） |
|--|-----------|----------------|
| 扇区范围 | 全周 24 扇区任一近距 | **仅车头 ±** `collision_sector_half_fov_deg`（**120°**） |
| 速度 | 无 | 朝向障碍 **分速度** `v_app > collision_v_min`（**0.05** m/s） |
| 防抖 | 单帧 | 连续 **`collision_frames`**（**2**）步候选才记 1 次事件 |
| 结束局 | **碰撞即 `done`** | **默认不 reset**；第 **3** 次墙撞 `done`；继续训练/探索 |
| 罚分 | 固定 **-25** | 第 k 次 **-25 / -50 / -100**（`collision_penalty_list`） |
| 空转/骑墙 | 无专门终止 | **`stuck`** 窗口位移过小 → `done` + **-70** |

贴墙慢移或已停住：`v_app ≤ collision_v_min` → **不算碰撞**（避免「抵墙即死局」）。

### B. 碰撞判定逻辑（前方扇区 + 速度 + 多层坡/墙 + 连续帧）

每步 `step()` 在 body 系读 `(vx, vy)`，扫描扇区 `i = 0…n_sectors-1`：

1. **前方扇区**：`|bearing(i)| ≤ collision_sector_half_fov_deg`（相对车头 +x 前向，+y 左）。
2. **距离**：该扇区 `d < collision_dist`（**0.38** m）。
3. **朝向速度**：`v_app > collision_v_min`。
4. 取最近候选扇区 → `_classify_obstacle()` 读 `/offroad/elevation_features` 各层 front-min + 可选 `/imu/data` pitch。
5. **墙判**：≥ `wall_layers_required`（默认 2）层近距，或 `min_d < wall_force_min_dist`（**0.30 m**）→ **墙**；否则坡模式 → 轻罚。
6. **墙** → 连续 **`collision_frames`**（**2**）步后记第 k 次墙撞事件。

**碰撞事件**：同一局内可多次 `[collision] event #1/#2/…`；**第 3 次**（`max_collision_events: 3`）→ `done` + `outcome=collision`；`reset()` 清空 `_collision_event_count`。

**递增扣分**（Record 3 训练前配置，`collision_penalty_list`）：

| 事件 k | 罚分 |
|--------|------|
| 1 | -25 |
| 2 | -50 |
| 3 | -100（并结束 episode） |

旧公式（Record 2 后重塑，`collision_penalty_double: true`）仍可用：\(\text{penalty}_k = r\_collision\_base \cdot 2^{k-1}\)。

### C. 受困（stuck）与超时

- **stuck**：维护 `(t, x, y)` 历史，窗口长度 ≥ `0.85 × stuck_time_sec` 后，若位移 `< stuck_dist_threshold` → `done=True`，加 `r_stuck`，打 `[stuck]`。用于 **原地空转、骑台阶、顶墙打滑** 等「还在动但走不出去」。
- **timeout**：步数到上限 → `done=True`，加 `r_timeout`（默认 **-30**）；replay 里 `done_bool` 可按训练节点配置处理（超时有时当非终止 bootstrap）。

### D. 与 spawn 门禁的区别

| | **spawn 门禁**（`_ensure_spawn_ok`） | **step 内碰撞 / 受困** |
|--|-------------------------------------|-------------------------|
| **时机** | 每局 `reset()` / 换图 `switch_world()` **spawn 后** | 每步 `step()` 控制循环中 |
| **目的** | 车 **没放好** 就不开训（防穿模/飞天/贴脸出生） | 训 **过程中** 的行为反馈 |
| **距离阈值** | **`spawn_collision_dist`**（0.35 m，**全周** min sector） | **`collision_dist`**（**0.38** m，**仅前方扇区**） |
| **速度** | 检 \|vz\|、pose 稳定，**无** `v_app` | 碰撞要 **`v_app > collision_v_min`** |
| **失败后果** | 重 spawn（+z / 换 xy），**不进入 step** | 扣分或 stuck **终止本局** |
| **日志** | `[spawn] OK` / `[spawn] FAIL` | `[collision]` / `[stuck]` / `[reward]` |

二者 **参数独立**：spawn 用更松的全周 clearance；step 碰撞用更严的前方 + 速度逻辑。**不要**把 `spawn_collision_dist` 当训练碰撞阈值。

### E. 参数表（与 `td3_train.yaml` 同步 · 2026-08-28 Record 6 事后）

数值以磁盘 `src/scout_offroad_3d/config/td3_train.yaml` 为准。**调大 / 调小** 指增大或减小该参数的典型效果。Record 6 长训已作废（见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md)）；碰撞列为 **事后收紧**（DEBUG 记录 33）：`collision_dist` 0.34→**0.38**，`wall_all_layers_dist` 0.31→**0.36**，`wall_lower_layers_dist` 0.28→**0.33**，`wall_force_min_dist` 0.25→**0.30**，`collision_frames` 3→**2**，`collision_v_min` 0.07→**0.05**；`wall_layers_required` 仍为 2。

#### E1. Reward

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `r_episode_start` | `4.5` | 每局**首步**一次性奖励 | 开局正反馈更强，可能掩盖早期坏行为 | 开局激励弱，策略更依赖后续 shaping |
| `r_progress` | `7.0` | 靠近目标：`r_progress × Δρ`（ρ 减小为正） | 更激进朝目标冲，易忽视障碍/贴墙 | 目标导向弱，探索更散 |
| `explore_enable` / `explore_cell_reward` / `explore_grid_res` | `true` / `0.03` / `0.5` | 每局 occupancy 栅格**首次**进入 cell 给小额 `r_explore`；重访 0（Record 6） | 探索激励更强，须防与 `r_progress` 抢主导 | 脱 stuck 弱，仅靠 progress |
| `explore_cardiff_scale` / `explore_rho_min` | `false` / `1.0` | `true` 时 `r_explore /= max(ρ, explore_rho_min)`（Cardiff 式距离缩放） | 近目标探索分更大 | 远距探索分更小 |
| `r_arrive` | `120.0` | 到达成功（`ρ < goal_tol`）终止奖 | 更愿冒险冲终点；碰撞代价相对变小 | 到达动力弱，局末徘徊 |
| `r_step` | `-0.03` | 每步时间/存活成本 | 更催短路径、少磨蹭（\|值\| 更大） | 允许更长绕路、超时局增多 |
| `r_forward_scale` | `0.13` | 每步 `× max(0, v−\|ω\|)` 前进鼓励 | 更爱直行加速，可能冲坡/贴障 | 前进激励弱，易原地转/慢挪 |
| `r_spin_penalty` | `0.5` | `v < spin_v_thresh` 且 `\|ω\| > spin_w_thresh` 时额外罚 | 更抑制原地打转 | 允许更多原地对准 |
| `spin_v_thresh` | `0.15` m/s | 判定「低速」的 v 上限 | 高速时仍可能触发 spin 罚 | 更容易被判原地转 |
| `spin_w_thresh` | `0.8` rad/s | 判定「高角速」的 ω 下限 | 需更大 ω 才罚 spin | 轻微转向即罚 spin |
| `r_low_effort_penalty` | `-0.09` | 持续低 `v−\|ω\|` 时每步额外罚 | 更逼车动起来（\|值\| 更大） | 允许长时间慢蹭/微动 |
| `low_effort_thresh` | `0.05` | 低前进量阈值 | 更难触发 low_effort 罚 | 更容易触发 low_effort 罚 |
| `low_effort_time_sec` | `2.8` s | 低前进量须持续的窗口 | 须更久才罚，容忍短停 | 更快判「磨蹭」并罚 |
| `r_slope_penalty` | `-0.05` | 坡道嫌疑每步轻罚（**不**计碰撞次数） | 更避坡/台阶（\|值\| 更大） | 坡道代价小，敢爬坡 |
| `r_collision_base` | `-25.0` | `collision_penalty_list` 为空时的回退单次罚 | 单次撞墙更重（\|值\| 更大） | 撞墙惩罚轻（仅 list 空时生效） |
| `collision_penalty_list` | `[-25,-50,-100]` | 第 k 次**墙撞**事件罚分 | 列表元素更负 → 重复撞墙更痛 | 撞墙代价轻，敢贴墙试错 |
| `collision_penalty_double` | `false` | `true` 时用 `r_collision_base·2^(k−1)` | 指数递增撞墙罚（list 空时） | 固定单次罚（list 空时） |
| `reward_log_interval` | `25` | 每 N 步打 `[reward]` 摘要 | 日志更疏 | 日志更密 |

#### E2. 碰撞 / 坡墙判别

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `terrain_classify_enable` | `true` | 开多层仰角 + IMU 坡/墙分类 | 关则近距一律按墙处理（无坡轻罚） | — |
| `collision_dist` | `0.38` m | 前方扇区 min 距离低于此才进入碰撞候选 | 更远就预警/撞墙，更保守 | 须更近才判，易漏真撞 |
| `collision_v_min` | `0.05` m/s | 朝向障碍分速度 `v_app` 须高于此 | 慢速贴墙/顶墙不算撞，更宽容 | 慢蹭也算撞，顶墙易记事件 |
| `collision_frames` | `2` | 连续 N 步候选才记 1 次墙撞事件 | 更防抖，真撞确认慢 | 更敏感，抖动/噪声易误记 |
| `collision_sector_half_fov_deg` | `120`° | 碰撞仅车头 ±N° 扇区 | 侧向近障也计入，更严 | 仅正前方，侧面擦过不算 |
| `max_collision_events` | `3` | 第 N 次墙撞 → `done`（`0`=不因此结束） | 单局允许更多次撞墙 | 更早因撞墙结束局 |
| `wall_layers_required` | `2` | ≥N 个仰角层 front-min 近距 → 墙模式 | 须更多层同时近才判墙，坡更易过 | 更少层即判墙，真墙更敏感 |
| `wall_all_layers_dist` | `0.36` m | 单层 front-min 低于此计「层近」 | 更容易累计层近 → 判墙 | 层近更难满足 → 坡/漏检 |
| `wall_lower_layers_dist` | `0.33` m | 低/中层近距阈值（坡 pattern） | 低层更易判近，墙/坡都更敏感 | 低层不易近，坡模式难触发 |
| `wall_force_min_dist` | `0.30` m | 水平 min_d 低于此 **强制墙**（绕过坡判） | 稍远即强制墙（Record 6 事后收紧） | 须更近才强制墙，低障碍可能当坡 |
| `slope_upper_clear_dist` | `1.0` m | 顶层（≈+20°）front-min 高于此倾向坡 | 更容易认坡（顶层须更远） | 坡更难认，易当墙 |
| `slope_max_frames` | `8` | 近距持续 ≤N 帧倾向坡（短暂贴障） | 更久近距仍可当坡 | 更快放弃坡解释 → 墙 |
| `slope_pitch_delta_deg` | `4.0`° | IMU pitch 上升超过此支持坡判 | 须更大抬头才认坡 | 轻微抬头即认坡 |
| `wall_v_forward_max` | `0.08` m/s | 多层近 + 前进速度 ≤ 此 → 墙倾向 | 稍快仍可能当墙 | 低速顶障更易当坡放过 |
| `use_imu_pitch` / `use_imu_roll` | `true` / `true` | 坡判用 pitch；翻车用 roll+pitch | 关 pitch 少坡线索；关 roll 翻车只看 pitch | — |
| `n_elevation_bins` | `4` | 判别用仰角桶数（4×12 特征） | 不改 state_dim，仅影响分类粒度 | — |
| `n_elev_front_sectors` | `12` | 判别前方扇区数（±60°） | 前方空间分辨率变化 | — |

#### E3. Stuck / Rollover / Timeout

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `stuck_time_sec` | `6` s | 路径位移滑动窗口长度 | 须更久不动才 stuck，容忍顶墙打滑 | 更快判受困结束局 |
| `stuck_dist_threshold` | `0.25` m | 窗口内位移低于此 → stuck | 须几乎不动才 stuck | 小幅挪动也判 stuck |
| `r_stuck` | `-70.0` | stuck 终止一次性罚 | 更怕受困（\|值\| 更大） | stuck 代价小 |
| `rollover_enable` | `true` | 是否 IMU 翻车终止 | 关则翻车不 done | — |
| `rollover_roll_deg` | `40`° | \|roll\| 超此且超 grace → 翻车 | 更难翻车终止 | 更容易翻车终止 |
| `rollover_pitch_deg` | `40`° | \|pitch\| 超此且超 grace → 翻车 | 同上 | 同上 |
| `rollover_grace_sec` | `2.0` s | spawn 后宽限期：期内翻车 **作废重开** | spawn 后更久不判翻/作废 | 更早可因姿态终止或作废 |
| `rollover_grace_abort_enable` | `true` | grace 内 IMU 翻车 → `reset` 重采样 | 关则 grace 内仅忽略姿态 | 开则 spawn 翻车作废本局 |
| `r_rollover` | `-60.0` | 翻车终止罚 | 更怕翻车（\|值\| 更大） | 翻车代价小 |
| `r_timeout` | `-30.0` | 超时终止一次性罚 | 更怕超时（\|值\| 更大） | 超时代价小 |
| `max_ep_steps` | `500` | 单局最大步数（`control_dt=0.1` → 约 50 s） | 单局更长，超时率低 | 更早 timeout |
| `goal_tol` | `0.55` m | `ρ` 低于此算到达成功 | 须更近才算到，成功率降 | 更容易判 success |

#### E4. Spawn / Goal

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `spawn_x/y_min/max` | `±7.0` m | 起点采样框 | 起点更分散、场景更大 | 起点集中，局间差异小 |
| `goal_x/y_min/max` | `±6.0` m | 目标采样框（可与 spawn 独立） | 目标更远/更散 | 目标更近/更集中 |
| `min_goal_dist` | `2.2` m | 起点到目标直线距离下限 | 任务更远更难 | 短距离任务多，易学局部 |
| `goal_check_enable` | `true` | 采样后做可达性检查 | 关则可能目标在障碍内 | — |
| `goal_min_clearance` | `0.38` m | 目标点周围 min sector 须大于此 | 目标须更开阔 | 允许更贴障的目标 |
| `goal_z_max_delta` | `0.35` m | 目标相对 spawn 高度差上限 | 允许更大高差目标 | 目标高度更接近起点 |
| `goal_resample_max` | `25` | 目标不合格时重采次数上限 | 更多次尝试找合法目标 | 易 fallback 未检目标 |
| `spawn_z` | `0.20` m | 初始 spawn 高度 | 车更高，少穿地 | 更低，易与地面干涉 |
| `spawn_z_retry_delta` | `0.08` m | spawn 失败每次抬高 z | 重试抬得更高 | 重试抬升小，可能仍穿模 |
| `spawn_z_max_retries` | `5` | 单点 z 重试次数 | 同 xy 多试几次 | 更快放弃该 xy |
| `spawn_check_wait` | `0.5` s | spawn 后等物理稳定时间 | 更稳再检，启动慢 | 检得早，可能误判 FAIL |
| `spawn_z_jump_threshold` | `0.25` m | 稳定后 z 跳变超此 → FAIL | 容忍更大 z 抖动 | 更严，易判 spawn 失败 |
| `spawn_vz_threshold` | `0.5` m/s | 竖直速度超此 → FAIL | 容忍更大下落/弹起 | 更严静止要求 |
| `spawn_collision_dist` | `0.35` m | **全周** min sector 低于此 → spawn FAIL | 出生须更远离障碍 | 允许更贴脸出生 |
| `spawn_pose_stable_threshold` | `0.08` m | 位姿稳定判定阈值 | 更松的稳定判定 | 更严，易 FAIL |
| `spawn_full_reset_retries` | `40` | 本图 pose 试尽后换图（非停训） | 更多次换点 spawn | 更快切下一张图 |
| `spawn_xy_jitter` | `0.8` m | FAIL 时 xy 随机抖动半径 | 失败时跳更远 | 失败仍在附近重试 |

#### E5. 速度 / 控制

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `max_v` | `2.0` m/s | 线速度 clip 上限（动作维 1） | 车更快，碰撞冲击大、state 上拍变大 | 更慢更稳，任务耗时增 |
| `max_w` | `1.5` rad/s | 角速度 clip 上限 | 转向更猛，易 spin / 侧向擦障 | 转向慢，对准目标慢 |
| `control_dt` | `0.1` s | 每 env 步 sleep（与 Gazebo 控制节拍对齐） | 每步更久，同样步数覆盖更长时间 | 步频高，仿真负载大 |

#### E6. 训练流程 / 探索噪声

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `max_timesteps` | `200000` | 本次训练总 env 步数 | 训更久（launch 可覆盖，长训常用 `500000`） | 早停，策略未收敛 |
| `start_timesteps` | `3000` | 纯随机动作步数，之后策略+噪声 | 更多随机覆盖 state 空间 | 更早用劣质策略，replay 偏 |
| `world_switch_interval` | `20000` | 每 N 步换一张 `worlds/*.world`；`0` 不换 | 换图更疏，单场景过拟合 | 换图更频，分布更杂、重启多 |
| `expl_noise` | `0.12` | 探索噪声 × `max_action` | 探索更大，动作更抖 | 更贴确定性策略，易陷局部 |
| `save_freq` | `5000` | 每 N 步存 checkpoint | 存盘更稀 | 存盘更密，磁盘占用增 |
| `seed` | `0` | Python/NumPy 随机种子 | 换种子换随机序列 | — |
| `noise_adapt_enable` | `false` | 按滑动窗口 success_rate 调 `sensor_noise_std` | 开则自动加/减感知噪声（Record 4 关以隔离 reward） | — |
| `sensor_noise_min` / `sensor_noise_max` | `0.01` / `0.1` m | 自适应噪声上下界（加在 range） | 噪声幅度范围更大 | 噪声范围更小 |
| `noise_adapt_window` | `50` | 统计 success_rate 的 episode 窗口 | 自适应反应更慢 | 反应更快，易抖动 |
| `noise_adapt_low_rate` | `0.30` | 窗口成功率低于此 → **加**噪声 | 更难触发加噪声 | 更容易加噪声 |
| `noise_adapt_high_rate` | `0.45` | 窗口成功率高于此 → **减**噪声 | 更难触发减噪声 | 更容易减噪声 |
| `noise_adapt_drop_thresh` | `0.09` | 窗口成功率骤降 → 加噪声 | 须更大跌幅才加噪 | 小幅跌率即加噪 |
| `noise_adapt_step` | `0.01` m | 每次调整噪声步长 | 噪声变化更猛 | 调整更细 |
| `state_noise_std` | `0.02` | 拼进 state 的高斯噪声（trainer 可运行时改） | 观测更噪，鲁棒性↑、学习难↑ | state 更干净 |

#### E7. TD3 网络超参（简要）

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `batch_size` | `512` | replay 采样批大小 | 梯度更稳、显存/内存占用↑ | 更新噪、训练不稳 |
| `discount` | `0.99` | γ，远期 reward 折扣 | 更重视长期到达 | 更短视 |
| `tau` | `0.006` | 目标网络软更新率 | 目标网跟得更快 | 目标更平滑、滞后 |
| `policy_noise` | `0.2` | 目标策略平滑噪声幅度 | 目标动作更扰动，抑制过估计 | 平滑弱 |
| `noise_clip` | `0.5` | 目标噪声 clip | 允许更大目标扰动 | 扰动上限更小 |
| `policy_freq` | `2` | 每 N 次 critic 更新才更新 actor | actor 更新更少，更稳 | actor 更频更新 |
| `n_sectors` | `24` | 水平扇区数 | state 维变，**须重训** | 同上 |
| `n_elevation_bins_td3` | `3` | TD3 仰角层数 | state_dim=3×24+4=76，**须重训** | 同上 |
| `max_range` | `5.0` m | 扇区距离归一化分母 | 同距离占 state 比例变小 | 同距离信号更饱和 |

网络宽度（**不在 yaml**）：`scripts/td3/TD3.py` 默认 Actor/Critic **256-256** MLP；改 hidden 须重训。

### F. 调参顺序建议

**原则**：先让「终止条件与传感判据」可信，再塑 reward；否则策略会对错误信号过拟合。

**1. 碰撞 / 坡墙（最先）**

1. 固定 `world` + `world_switch_interval:=0` 冒烟，看 `[terrain] slope` vs `[collision] wall` 比例。  
2. 坡道误当墙 → **先**略增 `slope_upper_clear_dist` / `slope_max_frames`，或略增 `wall_layers_required`；仍误判再略增 `wall_force_min_dist`（更不易强制墙）或略降 `wall_all_layers_dist`（更难层近）。  
3. 真墙不罚 → **增大** `collision_dist` / `wall_all_layers_dist` / `wall_force_min_dist`，或略降 `collision_v_min` / `collision_frames`（注意坡道副作用）。  
4. 确认后再动 `collision_penalty_list` / `max_collision_events`。

**2. Reward（其次）**

1. 保持 `r_forward_scale` 小量级（当前 **0.13**），与 `r_progress`（**7**）同量级微调，避免「刷前进分」hack。  
2. 到达率太低 → 略增 `r_progress` 或 `r_arrive`；碰撞多 → 先回到碰撞表，勿只加大 `r_arrive`。  
3. 原地转多 → 增 `r_spin_penalty` 或降 `spin_w_thresh`；磨蹭不走 → 增 `\|r_low_effort_penalty\|` 或降 `low_effort_time_sec`。  
4. `r_step` 控制局时长；与 `max_ep_steps` 联调。

**3. Stuck / Rollover（再次）**

1. 顶墙打滑不结束 → 略降 `stuck_time_sec` 或 `stuck_dist_threshold`（与 `collision_v_min` 联调：顶墙不算撞时靠 stuck 收尾）。  
2. 正常爬坡误 stuck → 略增 `stuck_time_sec` 或 `stuck_dist_threshold`。  
3. 翻车误杀 → 增 `rollover_grace_sec` 或角度阈值；真翻车漏检 → 略降 `rollover_roll_deg` / `rollover_pitch_deg`。

**4. 训练规模 / TD3 超参（最后）**

碰撞与 reward 稳定后，再拉长 `max_timesteps`、调 `start_timesteps` / `expl_noise`；`batch_size` / `discount` 一般保持默认，除非明显不收敛。

| 现象 | 快速对照 |
|------|----------|
| **坡道仍记 wall** | 增 `slope_upper_clear_dist` / `slope_max_frames`；确认 `/offroad/elevation_features` 在跳 |
| **真墙不罚** | **增大** `collision_dist` / `wall_all_layers_dist` / `wall_force_min_dist`；略降 `collision_v_min` / `collision_frames`（Record 6 事后已收紧一档） |
| **顶墙不算撞** | 略降 `collision_v_min`（注意坡道误触） |
| **骑墙不停** | 降 `stuck_time_sec` / `stuck_dist_threshold` |
| **单局撞墙过多** | 降 `max_collision_events` 或加大 `collision_penalty_list` 负值 |

**坡道/地形冒烟**（`rough_obstacles.world`，`world_switch_interval:=0`）：

```bash
roslaunch scout_offroad_3d train_td3.launch gui:=true max_timesteps:=500 start_timesteps:=50 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world world_switch_interval:=0
```

另开终端验收多层话题：

```bash
rostopic echo /offroad/elevation_features -n 1   # layout: 4×12, data 长度 48
rostopic echo /imu/data -n 1
```

通过标准：`[spawn] OK`；上坡时多见 `[terrain] slope suspected`、**少见** `[collision] wall confirmed`；真顶墙仍有 `wall confirmed`；第 3 次墙撞 `outcome=collision`；`[reward] forward` 在前进时出现。

---

<a id="where-to-edit"></a>

## 参数在哪改

**日常调参只改 `src/scout_offroad_3d/config/td3_train.yaml`**；`train_td3.launch` 仅在显式传 `max_timesteps:=` 等 CLI 时才覆盖同名项。

**改完必须重新 `roslaunch` 才生效**（yaml / launch 参数在节点启动时加载，不会热更新）。

### A. YAML：`src/scout_offroad_3d/config/td3_train.yaml`

**完整分块参数表（含义 + 调大/调小）** 见上文 [一附 2 · E 参数表](#reward-done-collision)（E1–E7）。下表为速查索引；数值与 **Record 5/6** `td3_train.yaml` 一致。

| 键 | 默认 | 单位 | 含义 |
|----|------|------|------|
| `world_switch_interval` | `20000` | 训练 timestep | 每 N 步换一张 `worlds/*.world`；**`0` = 不换图** |
| `world_switch_max_retries` | `5` | 次 | 单张图 reload 最大重试（Record 6） |
| `world_switch_kill_wait_sec` | `0.5` | s | kill gzserver 后等待（原 1.0） |
| `world_switch_gzserver_boot_sec` | `8` | s | boot 上限；spawn svc 就绪即提前跳出（原 12 盲等） |
| `world_switch_spawn_wait_sec` | `35` | s | 等 spawn_urdf_model 上限（原 90；8s fail-fast 仍在） |
| `worlds_dir` | `""` | 路径 | 空=包内 `scout_offroad_3d/worlds/` |
| `max_timesteps` | `200000` | 训练 timestep | 仅 launch 显式 `max_timesteps:=` 时覆盖 |
| `start_timesteps` | `3000` | 训练 timestep | 纯随机探索步数 |
| `seed` | `2` | — | RNG 种子 |
| `expl_noise` | `0.12` | ×max_action | 策略探索噪声 |
| `explore_enable` / `explore_cell_reward` / `explore_grid_res` | `true` / `0.03` / `0.5` | — / — / m | Record 6 occupancy 探索；metrics `r_explore_sum` |
| `batch_size` | `512` | — | replay 批大小 |
| `discount` / `tau` | `0.99` / `0.006` | — | γ；目标网络软更新 |
| `save_freq` | `5000` | 训练 timestep | 周期性存盘 |
| `goal_tol` / `max_ep_steps` | `0.55` / `500` | m / ep 步 | 成功 / 超时 |
| `collision_dist` | `0.38` | m | 前方扇区最小距离阈值（Record 6 事后 0.34→0.38） |
| `collision_v_min` / `collision_frames` | `0.05` / `2` | m/s / 连续步 | 朝向障碍分速度；连续 N 步记 1 次碰撞 |
| `collision_sector_half_fov_deg` | `120` | deg | 碰撞检测仅车头 ±N° 扇区 |
| `wall_layers_required` / `wall_force_min_dist` | `2` / `0.30` | 层 / m | ≥2 层近或极近强制墙（层距 0.36 / 低层 0.33） |
| `r_forward_scale` / `r_spin_penalty` | `0.13` / `0.5` | — | 前进 v−\|ω\|；原地转惩罚 |
| `stuck_time_sec` / `r_stuck` | `6` / `-70` | s / — | 路径受困 → **done** |
| `r_timeout` | `-30` | — | 超时终止罚 |
| `imu_wait_after_respawn_sec` | `8` | s | 换图后等 IMU（Record 6） |
| `noise_adapt_enable` | `false` | bool | 噪声自适应（默认关） |
| `r_episode_start` / `r_progress` / `r_arrive` / `r_step` | `4.5` / `7` / `120` / `-0.03` | — | Scout reward（见 [TRAIN_RECORDS](TRAIN_RECORDS.md) Record 5/6） |
| `world_spawn_z` | `bumpy_offroad.world: 1.05` | m | heightmap spawn 高度；平地走 `spawn_z: 0.20`。**换图前必须写入 identity**，否则 bumpy z≈1.06 会被当成 playpen z_jump（DEBUG 32） |

换图实现：`train_td3_node` 在 `t % N == 0` 时调用 `ScoutGazeboEnv.switch_world`（**杀 gzserver → 用新 `.world` 再起 → spawn 车**）。Gazebo Classic **不能**热加载换 `.world` 文件，故重启仿真进程；**训练节点与 ROS master 不停**，**不清空** replay；随后 `reset()` 仍随机起终点。日志：`[world] switch to xxx.world at t=...`。

### B. Launch 参数：`train_td3.launch`（`gui` 等不在 yaml）

| 参数 | 默认 | 单位 | 含义 |
|------|------|------|------|
| `world:=` | 启动时从 `worlds/*.world` 随机抽一张 | 路径 | **只决定第一张图**；之后仍按 yaml/`world_switch_interval` 换（除非 interval=0） |
| `gui:=` | `false` | bool | Gazebo 是否开窗口（**仅 launch**） |
| `max_timesteps:=` | 不传=用 yaml | 训练 timestep | 可选覆盖 yaml 同名项 |
| `start_timesteps:=` | 不传=用 yaml | 训练 timestep | 可选覆盖 yaml 纯随机探索步数 |
| `world_switch_interval:=` | 不传=用 yaml | 训练 timestep | 可选覆盖；冒烟可 `200`，锁图可 `0` |
| `output_dir:=` | 空 | 路径 | 产物目录；空则 `~/catkin_ws_2/3D/runs/run_...` |
| `spawn_delay:=` / `x/y/z/yaw` | `1` / `0/0/0.15/0` | s / m·rad | 首次 spawn |

### C. 改完怎么生效

```bash
# 1) 编辑 yaml 或准备 launch 覆盖参数
# 2) 若改了 Python / launch，建议 catkin_make 后 source
cd ~/catkin_ws_2 && source devel/setup.bash
# 3) 重新 launch（旧进程先 Ctrl+C，必要时 killall gzserver）
roslaunch scout_offroad_3d train_td3.launch gui:=false
```

---

## 二、前置：杀残留 · source · 编译 · 装 torch

### 2.1 每个新终端都要 source

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

下面命令块均默认「该终端已执行过上面三行」。

### 2.2 杀残留 Gazebo / ROS（换 launch 前强烈建议）

端口占用、`Address already in use`、黑屏卡死、spawn 失败，多半是旧 `gzserver` / `roscore` 没死干净：

```bash
# 先 Ctrl+C 停掉正在跑的 roslaunch
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
killall -9 joint_state_publisher robot_state_publisher 2>/dev/null
sleep 2
# 确认无残留
pgrep -a gzserver || echo "gzserver 已清"
```

### 2.3 编译（改过 `scout_offroad_3d` 或首次）

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack find scout_offroad_3d
# 应打印 .../src/scout_offroad_3d
```

### 2.4 安装 PyTorch（ROS Noetic = Python 3.8）

训练 / 推理节点用 **`#!/usr/bin/env python3`**，必须与系统跑 ROS 的解释器一致：

```bash
python3 --version
# 期望：Python 3.8.x

python3 -m pip install --user torch
# 若上面装到了别的 python，强制：
python3.8 -m pip install --user torch

# 验收（必须无 ImportError）
python3 -c "import torch; print(torch.__version__)"
```

说明：CPU 版即可；本阶段不强制 CUDA。版本过新若在 3.8 上装不上，换官方给 3.8 的 wheel（见下方「常见故障」）。

---

## 三、步骤2 验收扇区（训前建议做一次）

确认 **16 线点云** → `/offroad/td3_range_features` 正常（TD3 状态维 **3×24+4=76**）。

**终端 1 — Gazebo 官方 room / 越野 world：**

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scout_mini_pro_description gazebo_room_official.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world gui:=false
```

等约 8–15 s，车 spawn 完成。

**终端 2 — 扇区节点（3D 默认）：**

```bash
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d range_features.launch
# 默认：source:=cloud，cloud_topic:=/lidar/points，n_sectors:=24，full_circle:=true
```

**终端 3 — 验收点云与扇区：**

```bash
cd ~/catkin_ws_2 && source devel/setup.bash
# Gazebo block_laser 发布 sensor_msgs/PointCloud；bridge 另发 PointCloud2
rostopic echo /lidar/points -n 1
rostopic echo /lidar/points2 -n 1   # 可选：PointCloud2
rostopic hz /lidar/points
rostopic echo /offroad/td3_range_features -n 1
rostopic echo /offroad/range_features -n 1
rostopic echo /offroad/elevation_features -n 1
```

**通过标准：**

- `/offroad/td3_range_features`：`elevation_bin_td3=3`，`sector=24`，共 **72** 个 min range（TD3 感觉层）
- `/offroad/range_features`：`layout` 扇区 **24**（水平投影，碰撞/兼容）
- `/offroad/elevation_features`：`elevation_bin=4`，`front_sector=12`，共 **48** 个 min range（坡/墙判别）
- 车身 self-hit 被滤掉：前方扇区不应长期全是 0.18 m 贴脸值

**2D `/scan` 回退**（旧 2D 激光或对照）：

```bash
roslaunch scout_offroad_3d range_features.launch source:=scan scan_topic:=/scan
```

深度相机（仅前方，**不是**全周 16 线）：

```bash
roslaunch scout_offroad_3d range_features.launch source:=cloud \
  cloud_topic:=/camera/depth/points cloud_msg_type:=pointcloud2 frame_mode:=optical full_circle:=false
```

验收完后 Ctrl+C，按 **2.2** 清残留，再进第四节训练。

---

<a id="td3-train"></a>

## 四、TD3 训练完整操作（阶段 A）

### 4.0 快速打开（复制粘贴 · 每个新终端先 source）

**① 每个新终端必做**（未编译则先 `catkin_make`）：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

**② 换 launch / 开 Gazebo 前先杀残留**（端口占用、黑屏、spawn 失败时必做）：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
pgrep -a gzserver || echo "gzserver 已清"
```

**③ 只看地形（开 Gazebo 窗口，不训练）** — 把 `WORLD` 换成下面任一张：

| 世界 | `WORLD=` 路径 |
|------|----------------|
| 散落箱体/圆柱 | `$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world` |
| 户外平坦 + 锥桶护栏 | `$(rospack find scout_offroad_3d)/worlds/clearpath_playpen.world` |
| 户外杂乱树/墙/托盘 | `$(rospack find scout_offroad_3d)/worlds/offroad_map.world` |
| **颠簸越野（新）** `bumpy_offroad` | `$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world`（需 `~/.gazebo/models/uneven_ground`，见下） |

**③-b 打开新地图 `bumpy_offroad`（PX4 uneven_ground，80×80 m 缓起伏）**

依赖（一次性）：

```bash
test -f ~/.gazebo/models/uneven_ground/model.sdf && echo OK || \
  git clone --depth 1 https://github.com/PX4/PX4-SITL_gazebo-classic /tmp/px4_gz && \
  cp -r /tmp/px4_gz/models/uneven_ground ~/.gazebo/models/
```

只看地形：

```bash
roslaunch scout_mini_pro_description gazebo_room_official.launch \
  gui:=true \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world
```

短冒烟训练：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0 \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world
```

> 已加入 `train_td3.launch` 随机池（共 **4** 张）；`world_spawn_z=1.05`。训前须短冒烟看 `[world] identity → bumpy_offroad.world spawn_z=1.050` 与 `[spawn] OK world=bumpy_offroad`。2 m 总高差仍可能偏陡，不满意见 `worlds/SOURCES.md` 自建更小 heightmap。

```bash
# 示例：打开 rough_obstacles（仅看地形 + 官方车）
roslaunch scout_mini_pro_description gazebo_room_official.launch \
  gui:=true \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world
```

**④ 短冒烟训练（看车 + 确认 TD3 链路，约 2 分钟）**：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world
```

**⑤ Record 6 正式长训（无界面 · 50 万步 · 每 2 万步换图）**：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false max_timesteps:=500000 start_timesteps:=4000 world_switch_interval:=20000
```

产物目录：终端打印 `TD3 train output → ~/catkin_ws_2/3D/runs/run_.../`。更多世界命令见 [§9.2 分别打开各世界](#open-each-world)。

> **注意**：bash 里写 world 路径用 **`$(rospack find scout_offroad_3d)`**，**不要**写 `$(find scout_offroad_3d)`（那是系统 find 命令，会报错）。

### 4.1 一键 launch 里有什么

`train_td3.launch` **一个终端**一次拉起：

1. `gazebo_room_official.launch`（官方 Scout Mini + **启动时随机**第一张 `worlds/*.world`；训练中可按 interval 换图）
2. `range_features.launch`（**16 线 `/lidar/points` → 24 扇区** `/offroad/range_features`）
3. `train_td3_node.py`（在线 TD3；按 `world_switch_interval` 换图；结束写 `metrics.csv` 并自动出图）

**不含** A\* / Arena / SLAM / `map_server`。终端2 仅可选监控，不是必须。

### 4.2 launch 参数（与代码一致）

| 参数 | 默认 | 含义 |
|------|------|------|
| `gui` | `false` | Gazebo 是否开窗口 |
| `spawn_delay` | `1` | spawn 前等待秒数 |
| `world` | 启动时从 `worlds/*.world` **随机抽一张** | **第一张** Gazebo `.world`。之后若 yaml `world_switch_interval>0` 仍会换图；`0` 则整场固定。CLI：`world:=$(rospack find scout_offroad_3d)/worlds/...` |
| `max_timesteps` | 不传=用 yaml | 总环境步数（yaml 默认 `200000`；冒烟可 `max_timesteps:=500`） |
| `start_timesteps` | 不传=用 yaml | 纯随机探索步数（yaml 默认 `3000`） |
| `output_dir` | 空 | 空则自动 `~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS/` |
| `x/y/z/yaw` | `0/0/0.15/0` | 初始 spawn |

换图间隔等见 [参数在哪改](#where-to-edit) 与 `td3_train.yaml` 的 `world_switch_interval`。

### 4.3 本地有界面训练（调试看车）

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 若刚跑过别的仿真，先杀残留（见 2.2）
roslaunch scout_offroad_3d train_td3.launch gui:=true
```

预期：

- Gazebo 打开 **随机抽中的** 一张 `worlds/*.world` 越野场景 + 官方车（不是 2D 房间）
- 终端刷 `ep=... reward=... success=... collision=...`
- 日志行：`TD3 train output → /home/.../catkin_ws_2/3D/runs/run_...`

### 4.4 本地 / 服务器无界面（推荐长训）

**一个终端即可**（Gazebo + 扇区 + TD3 全在此 launch）：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch gui:=false
```

服务器同命令；步数由 `td3_train.yaml` 的 `max_timesteps` / `start_timesteps` 决定。短冒烟可加 `max_timesteps:=500`。

### 4.5 短冒烟（确认链路通，再开长训）

**先清残留**（避免 11345 占用 / 换图 spawn 失败）：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2 && source /opt/ros/noetic/setup.bash && source devel/setup.bash
```

**基础冒烟**（看车、确认 TD3 环）：

```bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  max_timesteps:=2000 \
  start_timesteps:=200
```

**短冒烟换图**（每 50 步换一张 world，共 300 步；确认换图后仍有 `ep=` / `[spawn] OK`）：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  max_timesteps:=300 \
  start_timesteps:=50 \
  world_switch_interval:=50
```

冒烟通过标准：

- 无 `PyTorch not found`
- 有 `/offroad/range_features`（可另开终端 `rostopic hz /offroad/range_features`）
- 终端出现 **`[spawn] OK`**（未通过会看到 `[spawn] FAIL` 并重试，**不会**进入 `[reward]` step）、若干 `[ep=N]` / `[reward]` / `[ep=N] done outcome=...`
- 换图冒烟：`t=50/100/...` 后有 `[world] spawn OK` / `switch complete`，**小车不消失**
- `~/catkin_ws_2/3D/runs/run_*/` 下出现 `config.yaml`、`train.log`、`metrics.csv`；步数够时出现 `td3_actor` / `td3_critic`（`save_freq` 默认 5000，短跑结束也会 **强制再 save 一次**）；正常结束还有若干 `*.png` 曲线图

**颠簸越野短冒烟**（`bumpy_offroad.world` · PX4 `uneven_ground`；Record 6 新增）：

| 地图 | 依赖 | 验收 |
|------|------|------|
| `bumpy_offroad.world` | `~/.gazebo/models/uneven_ground` | `ls ~/.gazebo/models/uneven_ground/model.sdf` |

缺模型：`git clone --depth 1 https://github.com/PX4/PX4-SITL_gazebo-classic /tmp/px4_gz && cp -r /tmp/px4_gz/models/uneven_ground ~/.gazebo/models/`（见 [`worlds/SOURCES.md`](../src/scout_offroad_3d/worlds/SOURCES.md)）。

> **已删除**：`yosemite_heightmap` / `volcano_dem`（用户实测过陡过大，不适合 Scout；见 DEBUG 记录 31）。

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash

roslaunch scout_offroad_3d train_td3.launch gui:=true \
  max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0 \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world
```

通过标准：`[spawn] OK world=bumpy_offroad`（**不要**出现 `target_z=0.200` 对 z≈1.06）；车在缓起伏地面上非悬空/穿地；有 `[ep=]` / `[reward]`。若 2 m 总高差仍偏大，见 SOURCES.md 自建更小 heightmap。

### 4.6 正常训练与指定产物目录

**Record 6 第六次大训（推荐 · 需用户手动执行）**：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=500000 \
  start_timesteps:=4000 \
  world_switch_interval:=20000
```

说明：不传 `max_timesteps` / `start_timesteps` / `world_switch_interval` 时 **完全读 yaml**（`200000` / `3000` / `20000`）。第一张 world 从 **4 张**随机抽（`bumpy_offroad` / `clearpath_playpen` / `offroad_map` / `rough_obstacles`）；reward/探索/碰撞以 yaml 为准。Record 5 权重用于 `infer_td3.launch`。**Record 6 `run_20260828_003323` 已作废**（整晚锁死 playpen），不要当越野续训。详见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md) Record 6。

**同目录续训**（加载 `td3_*` + 从 `metrics.csv` 最后一行的 t/ep 接着写；replay 清空；曲线结束后在同一文件夹重画）：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  output_dir:=/home/test/catkin_ws_2/3D/runs/run_20260828_003323 \
  resume:=true \
  max_timesteps:=500000 \
  start_timesteps:=4000 \
  world_switch_interval:=20000
```

必须 `max_timesteps` **大于** 当前 t（该 run 已到 **t=259949**）。不传时节点会读该目录 `config.yaml`（300000）或自动拉长，避免 yaml 默认 200000 导致立刻结束。先 `killall`，否则残留 gzserver 会 `Connection refused` / spawn 失败。

> **不要用这条当越野续训。** Record 6 权重只见过空 playpen；接着训仍是同一过拟合策略。换图 identity 修好之后应 **新开 run**。命令仅用于「同目录 checkpoint 接口」验收。

**推荐正式长训（指定首张平地 + 换图）**：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=500000 \
  start_timesteps:=4000 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world \
  world_switch_interval:=20000
```

说明：`max_timesteps` / `start_timesteps` / `world_switch_interval` 不传 launch 参数时，**完全使用 `td3_train.yaml`**；仅冒烟或临时试验时再加 `max_timesteps:=500` 等覆盖。

```bash
roslaunch scout_offroad_3d train_td3.launch gui:=false max_timesteps:=50000
```

或显式拉长：

```bash
roslaunch scout_offroad_3d train_td3.launch gui:=false max_timesteps:=200000 start_timesteps:=1000
```

**指定产物目录**（`output_dir` 已由 `train_td3.launch` → `train_td3_node` 接通）：

```bash
# mkdir 与 launch 可在同一终端、先后执行；不必另开终端专门 mkdir
mkdir -p ~/td3_runs/smoke1
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=50000 \
  output_dir:=$HOME/td3_runs/smoke1
```

含义：

- `output_dir` = 本次 run 的权重 / `train.log` / `metrics.csv` / 曲线图写到你指定的文件夹（方便 `scp` 到服务器或事后找结果）
- **不设** `output_dir` 时，用默认 `~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS/`（见下节）
- 目录在 **训练节点一启动就会创建**（打印 `TD3 train output → ...` 时已存在），不是整场训完才有

换图（bash 用 `rospack find`；launch XML 内才用 `$(find ...)`）：

```bash
# 默认：启动时随机第一张；之后每 world_switch_interval 步换图（yaml 默认 20000）
roslaunch scout_offroad_3d train_td3.launch gui:=false

# 指定第一张越野图（覆盖默认随机；之后仍按 interval 换，除非 interval=0）
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  world:=$(rospack find scout_offroad_3d)/worlds/offroad_map.world

# 短冒烟换图：每 50 步换一张，总共 200 步
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false world_switch_interval:=50 max_timesteps:=200 start_timesteps:=50
```

### 4.7 产物路径与文件名（推理 / 汇报必看）

**默认根目录**（未传 `output_dir` 时）：

```text
~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS/
```

工作区根目录解析顺序：`~workspace_root` 参数 → 环境变量 `CATKIN_WS` → `rospack find scout_offroad_3d` 的上两级（`.../src/scout_offroad_3d` → `catkin_ws_2`）→ 兜底 `~/catkin_ws_2`。详见 [`3D/runs/README.md`](runs/README.md)。

**何时生成**：`train_td3_node` **一开始**就会 `makedirs` 该目录并写入 `config.yaml` / 打开 `train.log`、`metrics.csv`（终端有 `TD3 train output → ...`）。训练过程中持续追加 log/metrics；`save_freq` 到达或正常结束时写 `td3_actor` / `td3_critic`；结束（或 Ctrl-C 进 `finally`）再出 `*.png`。**不是**全部训完才突然出现空目录。

| 文件 | 含义 |
|------|------|
| `td3_actor` | Actor 网络参数（**无后缀**，Fujimoto 风格） |
| `td3_critic` | Critic（双 Q）网络参数 |
| `td3_actor_optimizer` / `td3_critic_optimizer` | 优化器状态（续训用；推理可不拷） |
| `train.log` | 每 episode 文本行：`outcome` / `reward` / `success` / `stuck` / `collision_events` / `rho` |
| `metrics.csv` | episode 指标 CSV（列说明见 [§4.8 出图](#td3-plots)） |
| `config.yaml` | 本次 run 的**实际生效**超参快照（`meta` + `train` + `env`；含 launch 覆盖） |
| `params_backup.yaml` | 与 `config.yaml` 相同（双份归档） |
| `reward_curve.png` 等 | 训完自动生成的科研风曲线（见 [§4.8](#td3-plots)） |

保存前缀是目录下的 **`td3`**（代码里 `model_prefix = .../td3`），因此磁盘上是：

```text
.../run_xxx/td3_actor
.../run_xxx/td3_critic
```

推理时 `weights:=` 填的是前缀 **`.../run_xxx/td3`**（**不要**写成 `td3_actor`，也不要带扩展名）。

查看最新 run / 每次 run 里有什么：

```bash
# 哪个 run 最新（按修改时间）
ls -lt ~/catkin_ws_2/3D/runs/ | head
# 每次 run 目录内文件（actor/critic/csv/png 等）
ls -la ~/catkin_ws_2/3D/runs/run_*/
```

若用了 `output_dir:=$HOME/td3_runs/smoke1`，则到该路径下 `ls -la`，不再去 `~/catkin_ws_2/3D/runs/` 找。

<a id="td3-plots"></a>

### 4.8 训练后自动出图（科研曲线）

训练节点每局把 episode 指标 **追加写入** `metrics.csv`（新格式见下表）。  
正常结束（或 Ctrl+C 进 `finally`）会调用 `scripts/plot_td3_metrics.py`，在**同一 run 目录**生成白底清晰 PNG。

#### metrics.csv 列（新 run，2026-08-22 起）

| 列 | 含义 |
|----|------|
| `episode` / `t` | episode 序号；累计训练 timestep |
| `length` | 本局步数（episode length） |
| `reward` | 本局总 reward |
| `success` / `stuck` / `timeout` | 终止原因 0/1（互斥，success 优先） |
| `outcome` | 终止原因字符串：`success` / `stuck` / `timeout` / `collision`（仅 `max_collision_events>0` 时） |
| `collision` | 本局是否发生过 **≥1 次墙撞事件**（0/1；碰撞**不结束局**） |
| `collision_events` / `collision_count` | 本局墙撞事件次数（同值；`collision_count` 为科研常用名） |
| `rho` | 终止时到目标距离 (m) |
| `initial_goal_dist` | 本局起点直线距离 (m) |
| `path_length` | 本局轨迹累计路程 (m) |
| `path_efficiency` | `initial_goal_dist / path_length`（∈[0,1]，越接近 1 越直；与 DRL 文献 **欧氏距离/实际路径** 等价） |
| `buf` | replay buffer 大小 |
| `succ_total` | 累计成功局数 |
| `success_rate_cum` / `stuck_rate_cum` / `timeout_rate_cum` / `collision_rate_cum` | 截至本局的 **累计** 比率 |
| `mean_reward_cum` | 截至本局的平均 episode reward |
| `success_rate_win` / `stuck_rate_win` / `timeout_rate_win` / `collision_rate_win` | **滑动窗口**（默认 50 ep，`~metrics_window`）终止/碰撞比率 |
| `mean_collision_count_win` | 滑动窗口内平均碰撞次数 |
| `r_episode_start_sum` / `r_progress_sum` / `r_explore_sum` / `r_arrive_sum` / `r_forward_sum` | 本局各正向 reward 分项累计（Record 4+） |
| `r_step_sum` / `r_spin_sum` / `r_collision_sum` / `r_stuck_sum` / `r_rollover_sum` / `r_timeout_sum` / `r_slope_sum` / `r_low_effort_sum` | 本局各负向/步惩罚分项累计 |

**文献对齐**（DRL 移动机器人导航常见指标）：Success Rate、Collision Rate、Episode Length、Path Efficiency（欧氏/实际路径，见 [Frontiers in Robotics 2025 ITD3](https://doi.org/10.3389/frobt.2025.1625968) 等）。

**换图竖线**（`train.log` 里有 `[world]` 时自动画在 episode 轴上）：

| 颜色 | 含义 | **不是** |
|------|------|----------|
| **黄/橙虚线**（图例「换图」） | 真正换到新 `.world`（`switch complete` / 成功的 `switch to`） | — |
| **灰虚线**（图例「换图失败/回退」） | `switch FAILED` / `fallback … NOT a map change` / 旧日志 `recovery OK →` 回原图 | 成功换图。Record 6 全部 12 条都是灰线级失败，地图仍是 playpen |

失败换图仍会重启 gzserver，曲线上可能有抖动——**不要把灰线当多图训练证据**。

**旧 CSV 兼容**：仅有 12 列的旧 run 仍可 `plot_td3_metrics.py` 重绘；脚本会从 `success/stuck/timeout/collision_events` 现场推算窗口率与堆积图。新训练才会写入 `outcome`、路径效率等列。

#### PNG 输出

| 图文件 | 内容 |
|--------|------|
| `reward_curve.png` | episode reward 原始曲线 |
| `reward_smooth.png` | reward + 滑动平均 |
| `mean_reward.png` | 累计/平滑 mean reward |
| `success_rate.png` | **滑动窗口成功率**（主曲线，默认 w=50 ep）；累计为虚线参考 |
| `outcome_success_rate.png` | 成功终止比率（滑动窗口，与上同数据） |
| `outcome_stuck_rate.png` | **受困终止比率**（滑动窗口） |
| `outcome_timeout_rate.png` | **超时终止比率**（滑动窗口） |
| `outcome_stacked_area.png` | **堆积面积图**：每 N ep 区间内 success / stuck / timeout / rollover / collision 占比 |
| `reward_components_stacked_area.png` | **奖励分项占比**：每 N ep 内各 `r_*_sum` 的 \|value\| / sum(\|parts\|)（处理正负） |
| `reward_components_curves.png` | 各 reward 分项 episode 总和的滑动平均曲线（高对比度色 + 固定线型区分 11 分项） |
| `episode_length.png` | episode 长度 |
| `collision_rate.png` | 累计碰撞率（有 ≥1 次墙撞事件的 episode 比例） |
| `collision_rate_window.png` | 碰撞率滑动窗口曲线 |
| `collision_count.png` | 每局碰撞次数 + 滑动平均 |
| `stuck_rate.png` | 累计受困率 |
| `path_efficiency.png` | 路径效率（新 CSV 有 `path_efficiency` 列时） |

单独重绘（不必重训）：

```bash
# 传 run 目录或 metrics.csv 均可
rosrun scout_offroad_3d plot_td3_metrics.py ~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS
# 或
python3 $(rospack find scout_offroad_3d)/scripts/plot_td3_metrics.py \
  ~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS --window 20 --stack-bin 50
```

**给 AI / 自己分析**：把单次 run 里的 `metrics.csv` 和已有 `*.png` 直接发来即可解读曲线；没有 PNG 时用上面命令重绘。各文件含义见 [`3D/runs/README.md`](runs/README.md)。

依赖：`python3 -m pip install --user matplotlib`（缺包时训练仍成功，只是出图告警跳过）。

<a id="td3-hparams"></a>

### 4.9 超参 / 权重是不是 AUV 的？（怎么改）

**诚实结论：**

| 部分 | 来源 |
|------|------|
| Actor / Critic 结构与 TD3 更新规则 | **[sfujim/TD3](https://github.com/sfujim/TD3)**（Fujimoto et al., ICML 2018）— **MLP 全连接**（`Linear` 层），**不是** CNN 卷积/池化 |
| ROS episode 环（reset→step→cmd） | 参考 AUV/ROS DRL 写法，**只借流程** |
| reward 权重、`max_v`/`max_w`、扇区维、spawn 框等 | **本项目为 Scout 在 `td3_train.yaml` 设的**，**不是**把 AUV 权重原样搬来 |
| 「权重」口语 | = 训出的 `td3_actor`/`td3_critic` 网络参数 + yaml 里的 reward/超参 |

Fujimoto 论文一句（actor/critic）：算法在 actor-critic 设定下用一对 critic 取最小值抑制过估计，并延迟策略更新以降低每步误差（见上文 [一附 D](#td3-logic) 原句与 arXiv:1802.09477）。

**怎么改：** 改 `src/scout_offroad_3d/config/td3_train.yaml`（或 launch 覆盖 `max_timesteps` 等）→ 重新 `roslaunch` 训练。**不要**手改 `td3_actor` 二进制文件。完整参数表见 [一附 2 · E1–E7](#reward-done-collision)。
---

<a id="td3-infer"></a>

## 五、TD3 推理完整操作

### 5.1 权重路径怎么填

1. 找到训练产物目录，例如：  
   `~/catkin_ws_2/3D/runs/run_20260821_153012/`
2. 确认存在 `td3_actor` 与 `td3_critic`
3. launch 参数 **`weights`** = 该目录 + `/td3`（前缀，无 `_actor`）

正确示例：

```bash
weights:=$HOME/catkin_ws_2/3D/runs/run_20260821_153012/td3
```

错误示例：

```text
.../td3_actor          # 错：多了 _actor
.../td3_actor.pth      # 错：本仓库不带扩展名
.../run_xxx/           # 错：缺前缀名 td3
```

### 5.2 一键推理（自带 Gazebo + 扇区）

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 把 run_xxx 换成你的真实目录名
RUN=~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS

roslaunch scout_offroad_3d infer_td3.launch \
  gui:=true \
  weights:=$RUN/td3
```

`infer_td3.launch` 默认 `start_sim:=true`：会 include `gazebo_room_official` + `range_features`。

**Record 5 权重 + 颠簸越野 `bumpy_offroad`（须 `~/.gazebo/models/uneven_ground`）**：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

RUN=~/catkin_ws_2/3D/runs/run_20260823_132957

roslaunch scout_offroad_3d infer_td3.launch \
  gui:=true \
  weights:=$RUN/td3 \
  spawn_z:=1.05 \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world
```

另开终端发目标（否则车不动）：

```bash
source ~/catkin_ws_2/devel/setup.bash
rostopic pub -1 /offroad/train_goal geometry_msgs/Point "{x: 3.0, y: 2.0, z: 0.0}"
```

说明：Record 5 仅在 3 张平地 world 上训过，**未在 `bumpy_offroad` 上训**——此处为泛化/冒烟观察；预期可能更多 stuck/rollover。`spawn_z:=1.05` 与 `td3_train.yaml` 中 `world_spawn_z.bumpy_offroad` 一致（heightmap 地面 z≈1.06；**不要**再用 0.50，会 z_jump/陷地）。

若仿真已在跑，只起推理节点：

```bash
roslaunch scout_offroad_3d infer_td3.launch \
  start_sim:=false \
  weights:=$RUN/td3
```

### 5.3 发布目标（否则车不动）

推理节点订阅 **`/offroad/train_goal`**（`geometry_msgs/Point`，用 `x,y`；`z` 可 0）。未收到目标时节点空转、不发 `/cmd_vel`。

另开终端：

```bash
cd ~/catkin_ws_2 && source devel/setup.bash
rostopic pub -1 /offroad/train_goal geometry_msgs/Point \
  "{x: 2.0, y: 1.5, z: 0.0}"
```

自检：

```bash
rostopic echo /cmd_vel -n 5
rostopic hz /offroad/range_features
```

说明：阶段 A 推理目标话题与训练环境内部目标一致命名；**块2 的 `/offroad/local_goal` 尚未对接**，勿指望 RViz SetGoal / Arena。

<a id="real-robot"></a>

### 5.4 真车部署（阶段 A 局部策略 · 非「一键上车」）

**诚实边界**：当前是 **阶段 A**——Gazebo 里训出的 **局部避障 + 朝目标点走** 策略。真车上 **没有** Arena 中间层、SLAM、A\* 全局规划；只能把仿真权重当 **局部控制器** 试跑，**必须**有人监护 + 安全层，不能当量产导航栈。

#### 5.4.1 拷哪些文件到工控机

从训练 run 目录（例 `~/catkin_ws_2/3D/runs/run_20260821_153012/`）拷到真车同路径或任意目录：

| 文件 | 是否必需 | 说明 |
|------|----------|------|
| `td3_actor` | **必需** | Actor 权重 |
| `td3_critic` | **必需** | 加载时与 actor 成对校验 |
| `config.yaml` | **强烈建议** | 核对 `n_sectors=24`、`max_range=5.0`、`max_v`/`max_w` 与真车扇区节点一致 |

**不必拷**：`td3_*_optimizer`、`train.log`、`metrics.csv`、`*.png`（分析/续训用）。

工控机还需 **已编译并 source 的** `scout_offroad_3d` 包（含 `infer_td3_node.py`、`td3/`、`range_features_node.py`）及 **PyTorch**（`python3 -m pip install --user torch`）。

示例 scp（在本机执行，按账号改）：

```bash
RUN=run_20260821_153012
scp ~/catkin_ws_2/3D/runs/$RUN/{td3_actor,td3_critic,config.yaml} \
  user@robot-pc:~/catkin_ws_2/3D/runs/$RUN/
```

#### 5.4.2 真车上跑推理

**前置（缺一不可）**：

1. 底盘驱动 + `/cmd_vel` 订阅正常（Scout 官方栈或自研驱动）。
2. **2D 激光** `/scan`（或 **16 线** `/lidar/points`）有数据；**必须先有** `/offroad/range_features`：
   ```bash
   roslaunch scout_offroad_3d range_features.launch
   # 真车 16 线：cloud_topic:=/your_lidar cloud_msg_type:=pointcloud2 frame_mode:=base
   # 真车 2D scan：source:=scan scan_topic:=/your_scan
   ```
3. **里程计** `/odom`（或 LIO 发布的 odom）在跳。
4. 有人遥控急停 / 物理急停可用。

**启动推理**（不启 Gazebo）：

```bash
cd ~/catkin_ws_2 && source devel/setup.bash

RUN=~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS

roslaunch scout_offroad_3d infer_td3.launch \
  start_sim:=false \
  pose_source:=odom \
  weights:=$RUN/td3
```

发目标（世界系 `x,y`，与训练时 Gazebo world 同理；阶段 B 才换 `/offroad/local_goal`）：

```bash
rostopic pub -1 /offroad/train_goal geometry_msgs/Point "{x: 3.0, y: 1.0, z: 0.0}"
```

#### 5.4.3 话题对齐：仿真 vs 真车

| 用途 | 仿真（train/infer 默认） | 真车 |
|------|--------------------------|------|
| 速度输出 | `/cmd_vel` | `/cmd_vel`（或 launch `cmd_vel_topic:=`） |
| 原始感知 | `/lidar/points`（16 线）或 `/scan`（2D 回退） | 车载多线 → `cloud_topic:=`；2D → `source:=scan` |
| 扇区特征（TD3） | `/offroad/td3_range_features`（3×24=72 维） | **同上**；须先跑 `range_features.launch` |
| 水平投影（碰撞） | `/offroad/range_features`（24 维） | 同上 |
| 位姿 | Gazebo `get_model_state` | `/odom`（`pose_source:=odom`） |
| 目标 | `/offroad/train_goal` | 同上；块2 后为 `/offroad/local_goal`（**尚未接**） |

自检：

```bash
rostopic hz /scan /odom /offroad/td3_range_features
rostopic echo /offroad/td3_range_features -n 1   # data 长度应为 72
rostopic echo /cmd_vel -n 3
```

#### 5.4.4 安全层建议（必做）

- **速度上限**：推理节点已 clip `v∈[-0.15,max_v]`（须与训练 `config.yaml` 一致）；真车可再包一层 `twist_mux` / 限速节点。
- **急停**：遥控或硬件 E-stop 能 **立即切断** `/cmd_vel`。
- **障碍兜底**：扇区全近距 / `/scan` 断流时 **禁止** 继续发速度（当前节点未内置，需外层 watchdog）。
- **场地**：开阔、低速、有人跟车；勿直接上复杂越野或公共道路。
- **sim-to-real 差距**：Gazebo 2D scan ≠ 真车多线/振动/标定误差；先短距离、小目标验证。

#### 5.4.5 上车前 Checklist（逐项打勾）

- [ ] 工控机 `scout_offroad_3d` 已 catkin_make + source
- [ ] `python3 -c "import torch"` 无报错
- [ ] `td3_actor`、`td3_critic`、`config.yaml` 已拷到工控机
- [ ] `config.yaml` 中 `state_dim=76`、`n_sectors`/`n_elevation_bins_td3`/`max_range`/`max_v`/`max_w` 与 launch 一致
- [ ] `/lidar/points` 正常 → `range_features.launch` 已启 → `/offroad/td3_range_features` 72 维在跳
- [ ] `/odom` 正常；`infer_td3.launch start_sim:=false pose_source:=odom`
- [ ] 已发 `/offroad/train_goal`（近处小目标试跑）
- [ ] 急停 / 限速 / 人工监护就绪
- [ ] **未**指望 SLAM、地图、Arena、一键全局导航

---

<a id="td3-troubleshoot"></a>

## 六、常见故障

| 现象 | 处理 |
|------|------|
| `Address already in use` / Gazebo 起不来 | 按 **2.2** 杀 `gzserver`/`gzclient`/`roscore` |
| spawn 失败 / 车不出现 | 加大 `spawn_delay:=12`；确认用的是 `gazebo_room_official`（一键 launch 已带）；清残留后重开 |
| 无 `/offroad/range_features` | 确认 `/lidar/points` 有数据：`rostopic hz /lidar/points`；确认 `range_features` 在跑；2D 回退：`source:=scan` |
| `PyTorch not found` / `ImportError: torch` | `python3 -m pip install --user torch`；用 **同一** `python3` 验证 `import torch` |
| torch 装上了但 ROS 节点仍缺 | Noetic 必须是 **Python 3.8**：`python3.8 -m pip install --user torch`；勿只用 conda 里的 python 而 ROS 走系统 python3 |
| `package scout_offroad_3d not found` | 该终端未 `source devel/setup.bash`，或未 `catkin_make` |
| 推理立刻报 `weights_prefix required` | 必须传 `weights:=.../td3` |
| 推理加载失败 / 文件找不到 | 检查 `ls $RUN/td3_actor $RUN/td3_critic`；`weights` 不要带 `_actor` |
| 推理无运动 | 是否发了 `/offroad/train_goal`；是否有 `/cmd_vel`；Gazebo 里车是否卡死 |
| `int('')` / `train_td3_node` 立刻崩溃 | launch 曾用空 `max_timesteps:=` 覆盖 yaml；现仅 CLI 显式传参才覆盖；见 DEBUG 记录 29 |
| 续训立刻结束 / 仍从 ep=0 | `max_timesteps` 必须 **>** 当前 t（yaml 默认 200000 小于长训 t）；必须 `output_dir:=` 指向已有 run；日志应有 `[resume] loaded weights ... t=... ep=...` |
| 续训 spawn `Connection refused` / `pose_unreadable` | 先 `killall -9 gzserver gzclient roslaunch rosmaster rosout`；节点会尝试重启 gzserver 再 reset |
| 短冒烟没有 `td3_actor` | 极短中断可能未跑到最终 `save`；再跑一次并让节点正常跑完，或把 `max_timesteps` 调到 ≥ 几百并等结束日志 `Training finished` |

---

<a id="td3-server"></a>

## 七、服务器：拷什么 · 怎么训 · 权重怎么拉回

### 7.1 建议拷贝的目录

至少：

```text
catkin_ws_2/src/scout_offroad_3d/
catkin_ws_2/src/scout_description/
catkin_ws_2/src/scout_gazebo_sim/
catkin_ws_2/src/scout_control/
catkin_ws_2/src/scout_mini_pro_description/   # gazebo_room_official + room world
catkin_ws_2/3D/README0.md
catkin_ws_2/3D/README1.md
```

或整仓 `catkin_ws_2`（最省事）。服务器需 **ROS Noetic + Gazebo** 与本机同代。

示例 rsync（按你账号改）：

```bash
rsync -avz --progress \
  ~/catkin_ws_2/src/scout_offroad_3d \
  ~/catkin_ws_2/src/scout_description \
  ~/catkin_ws_2/src/scout_gazebo_sim \
  ~/catkin_ws_2/src/scout_control \
  ~/catkin_ws_2/src/scout_mini_pro_description \
  user@server:~/catkin_ws_2/src/
```

### 7.2 服务器上训

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m pip install --user torch

# 无显示（单终端即可）
roslaunch scout_offroad_3d train_td3.launch gui:=false max_timesteps:=50000

# 若无 GPU 显示环境仍要虚拟屏（少数机器需要）：
# xvfb-run -a roslaunch scout_offroad_3d train_td3.launch gui:=false max_timesteps:=50000
```

阶段 A **不要**在服务器上先跑 SLAM / 拷 `map.yaml` 当训练前置。

### 7.3 把权重拉回本机

```bash
# 在服务器上确认
ls ~/catkin_ws_2/3D/runs/run_*/td3_actor

# 本机拉取（示例）
mkdir -p ~/catkin_ws_2/3D/runs
scp -r user@server:~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS \
  ~/catkin_ws_2/3D/runs/
```

然后按 **第五节** 本地 `infer_td3.launch`，`weights:=$HOME/catkin_ws_2/3D/runs/run_.../td3`。

---

## 八、代码从哪抄的、改了什么

详见上文 **[一附](#td3-logic)**。摘要：

| 来源 | 用途 | Scout 改编 |
|------|------|------------|
| [sfujim/TD3](https://github.com/sfujim/TD3) MIT | Actor / twin Critic / replay / delayed update | `max_action` 向量 `(v_max, ω_max)` |
| DRL-AUV-navigation 类 ROS 环 | episode reset / step / cmd | 差速 `/cmd_vel`；状态用扇区+ρθ；模型名 `scout_mini` |
| Arena-Rosnav | 分层 local goal | **未接**；阶段 B/C |

包内溯源：`scripts/td3/NOTICE.md`、`LICENSE_sfujim_TD3`。

```text
scripts/td3/TD3.py, replay_buffer.py, ros_env.py
scripts/train_td3_node.py, infer_td3_node.py, plot_td3_metrics.py, pick_random_world.py
launch/train_td3.launch, infer_td3.launch, range_features.launch
config/td3_train.yaml
worlds/*.world
```

---

<a id="open-new-terrain"></a>

## 八附、打开新地图（颠簸越野 · `bumpy_offroad`）

> **这才是新的颠簸地形**，不是 `clearpath_playpen` / `offroad_map` / `rough_obstacles` 那三张平地/杂物图。  
> 基于 PX4 **`uneven_ground`** heightmap：**80×80 m** 小范围、**~2 m** 缓坡起伏。

### 一次性安装模型（本机若无 `~/.gazebo/models/uneven_ground`）

```bash
# 方式 A：本机已有 PX4 sitl_gazebo（推荐）
cp -a ~/uav_flight_catkin_ws/repos/prometheus_px4/Tools/sitl_gazebo/models/uneven_ground ~/.gazebo/models/

# 方式 B：从 GitHub 拉取
git clone --depth 1 https://github.com/PX4/PX4-SITL_gazebo-classic.git /tmp/px4-gz
cp -a /tmp/px4-gz/models/uneven_ground ~/.gazebo/models/

# 验证
ls ~/.gazebo/models/uneven_ground/model.sdf
```

### 打开新地图（复制粘贴）

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws_2/devel/setup.bash
killall -9 gzserver gzclient 2>/dev/null; sleep 1

# ① 仅看地形 + 官方 Scout（无 TD3，最快目视）
roslaunch scout_mini_pro_description gazebo_room_official.launch \
  gui:=true \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world

# ② 带 TD3 训练栈开新地图（短冒烟）
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0 \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world
```

世界文件：`src/scout_offroad_3d/worlds/bumpy_offroad.world`。模型安装与缩放说明见 [`worlds/SOURCES.md`](../src/scout_offroad_3d/worlds/SOURCES.md)。

---

<a id="offroad-worlds"></a>

## 九、越野世界目录 / 下载来源 / 换 world

阶段 A **只要**仿真 `.world`，**不要** SLAM 建图产物。

### 9.1 规范目录（选定）

**唯一规范放置处：** `src/scout_offroad_3d/worlds/`  
（ROS 包内资源；**launch / XML 内**可用 `$(find scout_offroad_3d)/worlds/...`；**bash 命令行**须用 `$(rospack find scout_offroad_3d)/worlds/...`。**不用** `3D/worlds/`。）

清单与溯源见包内 [`src/scout_offroad_3d/worlds/SOURCES.md`](../src/scout_offroad_3d/worlds/SOURCES.md)。

| 文件 | 来源 | 用途 |
|------|------|------|
| `clearpath_playpen.world` | **复制自** `src/scout_gazebo_sim/worlds/clearpath_playpen.world`（原包未改） | 户外平坦场 + 护栏/锥桶/垃圾桶等障碍；**有用**（避障/杂乱场景），**不是**起伏越野 |
| `rough_obstacles.world` | 本包自建 | 散落箱体/圆柱障碍 |
| `offroad_map.world` | 用户放入本目录（原名无后缀 `offroad_map`，已改为 `.world`） | 户外杂乱场：树、泥箱、墙、托盘/砖块等；需 Gazebo `model://` 资源 |
| `bumpy_offroad.world` | PX4 `uneven_ground` 包装（Apache 2.0） | **缓起伏 heightmap** 80×80 m；`world_spawn_z=1.05`；模型在 `~/.gazebo/models/uneven_ground`；**已入 4 张随机池** |

**当前训练池 = 4 张**（3 平地 clutter + `bumpy_offroad`；`train_td3.launch` 已同步）。DEM 山地形（yosemite / volcano）已删除，见 DEBUG 记录 31。同目录还有 `weston_robot_empty.world`（在 `scout_gazebo_sim/worlds/`）——仅空地，**未拷贝**。外部候选见 [`worlds/SOURCES.md`](../src/scout_offroad_3d/worlds/SOURCES.md)。

说明：`clearpath_playpen.world` **内部只有一个** `<world name='default'>`；用户感觉「两个世界」多半是同目录下的 **playpen + weston_empty** 两个文件。

<a id="open-each-world"></a>

### 9.2 分别打开各个训练世界

**改图 / 只看地形（无车）必须用** `view_world.launch`（只加载 `.world`，**零 spawn**）。  
`gazebo_room_official.launch` **会带车，存盘会嵌车，不要用来改图**。训练带车用 `train_td3.launch`。  
GUI 加植物/障碍后要**覆盖保存同一文件**，见 [§9.5](#save-overwrite-world)。

**无车打开模板**（把 `<name>` 换成下表四个文件名之一）：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
killall -9 gzserver gzclient 2>/dev/null; sleep 1
roslaunch scout_offroad_3d view_world.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/<name>.world
```

**命令行换 world 前必读：**

1. **先 source**：`source /opt/ros/noetic/setup.bash` 且 `source ~/catkin_ws_2/devel/setup.bash`（未编译则先 `catkin_make`），否则 `rospack` / `roslaunch` 会报包找不到（如 `scout_mini_pro_description` / `scout_offroad_3d`）。
2. **bash 命令行**写路径时用 **`$(rospack find scout_offroad_3d)`**，**不要**写 `$(find scout_offroad_3d)`——后者是系统 `find`，会报 `No such file or directory`。
3. **仅 launch / XML 内部**才可用 ROS 替换 `$(find pkg)`（由 roslaunch 解析，不是 bash）。

若 Gazebo 残留可先 `killall -9 gzserver gzclient`。

目录清单（`src/scout_offroad_3d/worlds/*.world`；`train_td3.launch` 默认与 `pick_random_world.py` 均从此列表启动时随机抽一张）：

| 文件 | 简述 |
|------|------|
| `rough_obstacles.world` | 散落箱体/圆柱（±7 m 内最密） |
| `clearpath_playpen.world` | stock 户外平坦场 + 外围护栏；**训练框内几乎空**（DEBUG 32） |
| `offroad_map.world` | 户外杂乱：树 / 泥箱 / 墙 / 托盘等 |
| `bumpy_offroad.world` | heightmap 缓起伏；spawn_z **1.05** |

每条 **训练** 用 `train_td3.launch` 的 `world:=`；短冒烟见 [§4.0 / §4.5](#td3-train)。

#### `rough_obstacles.world`

```bash
# 【改图 / 仅看地形】无车。必须用这条
roslaunch scout_offroad_3d view_world.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world

# 训练（开 GUI；会 spawn 官方车，不要用来改图存盘）
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world

# gazebo_room_official 会带车，存盘会嵌车，不要用来改图
# roslaunch scout_mini_pro_description gazebo_room_official.launch \
#   world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world
```

#### `clearpath_playpen.world`

```bash
# 【改图 / 仅看地形】无车。必须用这条
roslaunch scout_offroad_3d view_world.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/clearpath_playpen.world

# 训练（开 GUI；会 spawn 官方车，不要用来改图存盘）
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  world:=$(rospack find scout_offroad_3d)/worlds/clearpath_playpen.world

# gazebo_room_official 会带车，存盘会嵌车，不要用来改图
```

#### `offroad_map.world`

```bash
# 【改图 / 仅看地形】无车。必须用这条
roslaunch scout_offroad_3d view_world.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/offroad_map.world

# 训练（开 GUI；会 spawn 官方车，不要用来改图存盘）
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  world:=$(rospack find scout_offroad_3d)/worlds/offroad_map.world

# gazebo_room_official 会带车，存盘会嵌车，不要用来改图
```

#### `bumpy_offroad.world`

须 `ls ~/.gazebo/models/uneven_ground/model.sdf`。spawn 高度 yaml `1.05`（heightmap z≈1.06）。

```bash
# 【改图 / 仅看地形】无车。必须用这条
roslaunch scout_offroad_3d view_world.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world

# 训练（开 GUI；会 spawn 官方车，不要用来改图存盘）
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true world_switch_interval:=0 \
  world:=$(rospack find scout_offroad_3d)/worlds/bumpy_offroad.world

# gazebo_room_official 会带车，存盘会嵌车，不要用来改图
```

无头训练时把上面的 `gui:=true` 换成 `gui:=false` 即可。

<a id="edit-world-verify"></a>

### 9.5 修改 `.world` 与验证

**改图必须用无车命令** `view_world.launch`（见 [§9.2](#open-each-world)）。  
`gazebo_room_official.launch` **会带车，存盘会嵌车，不要用来改图**。

**文件位置**：`src/scout_offroad_3d/worlds/*.world`（SDF 1.6/1.7 XML，**无**外部 model include 的自建图可直接改；`offroad_map.world` 引用 `model://` 树/墙等，需本机 `~/.gazebo/models/`）。

**Gazebo world 结构（自建图）**：

```text
<sdf><world>
  ground_plane   → <plane> 碰撞 + visual
  obstacle_*     → 静态 box/cylinder 等
</world></sdf>
```

**GUI / 命令行辅助**（打开某一张；**零 spawn**；把文件名换成池里其它 `.world`）：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
killall -9 gzserver gzclient 2>/dev/null; sleep 1

# 仅开地形 + 障碍（无车）。改图必须用这条，不要用 gazebo_room_official
roslaunch scout_offroad_3d view_world.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world
```

<a id="save-overwrite-world"></a>

#### GUI 覆盖保存（同一文件、同一目录）

1. 用上面的 **`view_world.launch`** 打开（模型树里不应有 `scout_mini`）。
2. Insert 植物 / 箱体 / 障碍（左栏 **Insert** → 拖进场景）。
3. 菜单 **File → Save World As**，选回包内**同一路径并覆盖**（不要另存到 Downloads）：
   `~/catkin_ws_2/src/scout_offroad_3d/worlds/<name>.world`
4. 存完立刻检查，`scout_mini` **必须搜不到**（应为空）：

```bash
grep -n scout_mini \
  $(rospack find scout_offroad_3d)/worlds/<name>.world
```

若 grep 仍有输出：说明用了带车 launch 存盘。用文本编辑器删掉整个 `<model name='scout_mini'>…</model>`，以及 `<state>` 里同名的 `<model name='scout_mini'>` 块，再存盘。Record 6 里 `rough_obstacles` / `offroad_map` 曾因 `gazebo_room_official` 存盘嵌第二辆车，换图再 `spawn_urdf` → 双车 / gzserver `lock_error`。

Gazebo GUI：**View → Transparent / Collisions**（看碰撞体是否实心、是否接地面）。改图窗口里没有车，不要 `rostopic pub /cmd_vel`。

**训练侧验证**（spawn 后看车是否正常避障）：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world \
  world_switch_interval:=0
```

通过标准：`[spawn] OK`；GUI 中 **View → Collisions** 障碍碰撞体合理；真撞墙有 `[collision] wall confirmed`（记录 14）。历史 `rough_ramps.world` 已删除，见 DEBUG 记录 18。

### 9.3 在 `train_td3.launch` 里换 world

已支持 `world:=`（传给 `gazebo_room_official.launch` 的 gzserver）。**命令行**仍须 `$(rospack find ...)`，勿用 bash 的 `$(find ...)`。

```bash
# 默认：启动时从 worlds/*.world 随机抽第一张；训练中按 world_switch_interval 换图
roslaunch scout_offroad_3d train_td3.launch gui:=false

# 指定第一张越野/障碍 world（绝对路径；之后仍按 interval 换，除非 yaml=0）
roslaunch scout_offroad_3d train_td3.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/rough_obstacles.world

roslaunch scout_offroad_3d train_td3.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/clearpath_playpen.world

roslaunch scout_offroad_3d train_td3.launch \
  world:=$(rospack find scout_offroad_3d)/worlds/offroad_map.world

# 短名 → 绝对路径（同样只定第一张）
roslaunch scout_offroad_3d train_td3.launch \
  world:=$(rosrun scout_offroad_3d pick_random_world.py clearpath_playpen)

roslaunch scout_offroad_3d train_td3.launch \
  world:=$(rosrun scout_offroad_3d pick_random_world.py offroad_map)
```

（说明：`train_td3.launch` 默认已在 parse 时用 `$(eval)`+`anon` 从包内 `worlds/*.world` 抽**第一张**。Noetic 的 `$(eval)` 不允许 `__import__`/`subprocess`，故列表写在 launch 里；新增 `.world` 后请同步 launch 列表，或运行 `rosrun scout_offroad_3d pick_random_world.py --print-launch-eval`。训练中换图由 `world_switch_interval` 控制，见 [参数在哪改](#where-to-edit)。）

### 9.4 手动下载链接（外网/LFS 失败时）

把 `.world`（及所需 model）放进 `src/scout_offroad_3d/worlds/`：

| 来源 | URL |
|------|-----|
| Clearpath 额外户外（inspection / orchard / solar） | https://github.com/clearpathrobotics/cpr_gazebo （`noetic-devel`） |
| Clearpath Ignition 户外（pipeline 等） | https://github.com/clearpathrobotics/clearpath_simulator |
| AWS RoboMaker 样本（已归档，可看 `ros1` 分支） | https://github.com/aws-robotics/aws-robomaker-small-warehouse-world |

本机已有 playpen 依赖模型时（常见于 `~/.gazebo/models/` 的 jersey_barrier 等）可直接开 playpen；缺模型时先 `gazebo` 拉一次或按 Clearpath/OSRF 模型说明安装。

---

<a id="train-logic-status"></a>

## 九附、训练逻辑现状（对照用户理解）

用户理解：**随机开 10 张图之一 + 随机起终点 + reward（接近大分、碰撞罚、时间长罚）**。

| 项 | 现状（对照 `ros_env.py` / `td3_train.yaml`） |
|----|-----------------------------------------------|
| 随机起终点 | **已有**：spawn/goal 默认 ±6 m，`min_goal_dist: 2.5`；**goal 可达性** `_validate_goal()`（记录 19） |
| Reward | **已有**：`r_arrive: 120`、`r_progress: 7`、`r_forward_scale: 0.13`；**探索** `r_explore: 0.03`（Record 6）；rollover / stuck 分离；碰撞递增，第 3 次 done |
| Episode 结束 | success / **rollover** / stuck / timeout / collision（第 3 次墙撞） |
| Spawn 高度 | **已有（硬性门禁）**：`reset()` / `switch_world()` spawn 后等物理稳定 → 检 z、\|vz\|、pose 稳定、min sector clearance；**未通过不返回 state**；`bumpy_offroad` 用 `world_spawn_z: 1.05`（须在 spawn 校验**之前**绑定 identity，DEBUG 32） |
| **Spawn 失败不停训** | 本图随机试 **≥40 个 (x,y)** → 仍失败则 **`[spawn] giving up this world=X → switch to Y`**（`pick_next_world` + `switch_world`）。整池一轮都失败：跳过本局、等待、再扫池。**永不**因 spawn 退出进程（DEBUG 35–37） |
| 多 world 切换 | **已有**：`world_switch_interval: 20000`；每 N 步随机换 **4 张**；单张换图失败会 **连试池内最多 4 张**，仍失败则 log `switch FAILED` 并继续训（不锁 playpen） |
| 「10 张图」 | 池子当前 **4 张** `.world`；yosemite/volcano **已删**；逻辑是随机换图 + 随机起终点 + reward，不是固定 10 张 |

---

<a id="smoke-to-server"></a>

## 九附 2、分终端：本地短训 → 拷服务器长训

> **一共开几个终端？**  
> - **最少 1 个**：`roslaunch scout_offroad_3d train_td3.launch gui:=false max_timesteps:=50000` 即可把 **Gazebo + 扇区 + TD3** 全拉起。  
> - **推荐 2 个**：终端① 训练；终端② 监控（`rostopic` / 看 `train.log`）——终端2 **可选**。  
> - 换 launch 前先按 **2.2** 清残留；**每个新终端**都要先 `source`（见 **2.1**）。

---

### ① 终端1 —— 训练主进程（必须）

每个新终端先贴：

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 若刚跑过仿真，先按 2.2 杀残留
```

#### A. 本地短冒烟（看车、确认链路）

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  max_timesteps:=2000 \
  start_timesteps:=200
```

**换图冒烟**（每 50 步换图，看 `[spawn] OK` / `ep=` 是否持续）：

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null
sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  max_timesteps:=300 \
  start_timesteps:=50 \
  world_switch_interval:=50
```

指定某张图（手动 `world:=`）：

```bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  max_timesteps:=2000 \
  start_timesteps:=200 \
  world:=$(rospack find scout_offroad_3d)/worlds/offroad_map.world
```

默认已会随机抽一张 `worlds/*.world`（含 `offroad_map.world`）；也可显式指定：

```bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true \
  max_timesteps:=2000 \
  start_timesteps:=200
```

冒烟通过：无 `PyTorch not found`；有 `[spawn] OK`、若干 `[ep=...]` / `[reward]`；换图后小车不消失；`~/catkin_ws_2/3D/runs/run_*/` 有 `train.log` / `config.yaml` / `metrics.csv`（结束还有曲线 PNG）。

#### B. 正式 / 服务器长训（关窗口、大步数）

```bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=50000
```

更大步数示例：

```bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=200000 \
  start_timesteps:=1500
```

默认已随机 world；无 GUI 长训示例：

```bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=50000
```

> **顺序**：先开 **终端1** 并等到 Gazebo/车 spawn、开始刷 `ep=`；再开 **终端2** 监控（可选）。训完 Ctrl+C 停终端1（仍会尝试出图），再按需要拉权重。

---

### ② 终端2 —— 监控（强烈建议）

另开一个终端，同样先 `source`，再任选一类：

**看扇区话题是否在跳：**

```bash
cd ~/catkin_ws_2 && source devel/setup.bash
rostopic hz /offroad/range_features
# 或抽一帧：
rostopic echo /offroad/range_features -n 1
```

**看训练日志（产物目录以终端1 打印的 `TD3 train output → ...` 为准）：**

每局 `reset` 时终端会打一行 `[ep=N] world=... start=... goal=... spawn_z=...`；训练中周期性 `[reward] approach ... | total_ep=...`；碰撞 `[collision] event #k ...`（不结束局）；受困 `[stuck] displacement=... → reset`；该局结束 `[reward] ... END success/stuck/timeout` 与 `[ep=N] done outcome=...`。

```bash
# 最新 run（示例）
ls -lt ~/catkin_ws_2/3D/runs/ | head
tail -f ~/catkin_ws_2/3D/runs/run_*/train.log
```

**看速度指令是否在发：**

```bash
rostopic echo /cmd_vel -n 5
```

终端2 **不跑** `roslaunch train_td3`；只观察，避免双开 Gazebo。

---

### ③ 本地通过后 → 拷服务器

**建议拷贝：**

```text
catkin_ws_2/src/scout_offroad_3d/          # 含 worlds/（含 offroad_map.world）、TD3、launch
catkin_ws_2/src/scout_description/
catkin_ws_2/src/scout_gazebo_sim/
catkin_ws_2/src/scout_control/
catkin_ws_2/src/scout_mini_pro_description/
catkin_ws_2/3D/README0.md
catkin_ws_2/3D/README1.md
```

或整仓 `catkin_ws_2`。服务器：`catkin_make` + `pip install --user torch`，再只开 **终端1** 跑上面的 **B**（`gui:=false`、大 `max_timesteps`）。权重在 `~/catkin_ws_2/3D/runs/run_*/`，训完拉回本机按 **第五节** 推理。

| 场景 | 终端数 | gui | max_timesteps | world |
|------|--------|-----|---------------|-------|
| 本地短冒烟 | 1～2 | `true`（想看车） | `2000` | 默认随机第一张 / `world:=` 定第一张；yaml `world_switch_interval` 控制后续换图 |
| 服务器长训 | 1（+可选监控） | `false` | `50000` 或更大 | 同上 |

---

<a id="stack-choice"></a>

## 十、选型拍板（建图黑盒 + 粗全局规划 · 与块1/块2一致）

与 [`README0`](README0.md) 一致：**块1=TD3（阶段 A 现在）**；**块2=SLAM+Arena+粗规划（B/C）**。下列选型**不阻塞**当前训练。

### 10.1 建图 / 里程计黑盒推荐表

口令对照：kiss leo → **KISS-ICP**；fast leo → **FAST-LIO2**；leo slam → **LIO-SAM**。

| 场景 | 传感器现实 | **推荐** | 角色 | 原因 / 限制 |
|------|------------|----------|------|-------------|
| **仿真现阶段** | 有 `/imu/data`，**16 线** `/lidar/points` + 深度相机 | **不上 LIO 作主链** | — | 阶段 A 训 TD3 用 Gazebo `/odom` + 扇区投影即可 |
| 仿真旁路试里程计 | 16 线点云或深度点云 | **KISS-ICP** | 可选对照 | 块2 旁路；不阻塞 TD3 |
| **真车越野**（Helios 等多线 + 9 轴 IMU） | 多线 PointCloud2 + IMU | **FAST-LIO2 作主** | 主位姿/建图黑盒 | 实时性与工程落地常见；外参标定后作默认 |
| 真车对照 | 同上 | **LIO-SAM** | 对照 / ablation | 因子图+回环/GPS 友好；论文对照与大场景建图 |
| 真车轻量对照 | 点云为主 | **KISS-ICP** | 轻量对照 | 无紧耦合 IMU 时的保底里程计 |

**阶段接入**：A 不用；C 再开黑盒。勿在 `train_td3.launch` 里绑 LIO。

### 10.2 粗全局规划

| 场景 | 推荐 | 何时接入 |
|------|------|----------|
| **室内仿真**（现有 room / 2D 栅格） | 继续现有 **2D A\***（`scout_mini_pro_description` 导航栈） | 阶段 C 对照；**阶段 A 不启** |
| **越野 / 非平面可通行性** | **可通行性代价地图 + Hybrid A\***（或同类 kinodynamic 全局） | 阶段 C；输出粗路径给 Arena 中间层 |
| **Arena 局部子目标** | 沿粗路径 spatial horizon 取点 → `/offroad/local_goal` | **阶段 B**（可先直线前瞻，再完整 Arena） |

结论一句话：**A 只训 TD3；B 接 local_goal；C 才上 A\*/Hybrid A\* 与建图黑盒。**

### 10.3 Arena 子目标（再次强调）

中间规划器：全局路径 + 车位姿 + look-ahead → **动态 local goal** → TD3 的 \((\rho,\theta)\)。  
**现在阶段 A 没有**：训练用随机目标；推理发 `/offroad/train_goal`。B 再对齐 `/offroad/local_goal`。

IMU：`rostopic echo /imu/data -n 1`（需重开官方仿真加载 URDF）；**训 TD3 不依赖 IMU**。

---

## 十一、还差什么（诚实）

骨架可启训练环并导出权重。已有：启动时随机第一张 `worlds/*.world`、`world:=` 定第一张、`world_switch_interval` 按步换图、`pick_random_world.py`、`metrics.csv` + 训完自动出图、自建 rough 世界、playpen 副本、用户 `offroad_map`。  
仍缺：推理目标与 B 的 `local_goal` 对齐。  
**已有**：前方扇区碰撞防抖 + 受困终止 + **坡/墙多层判别**（记录 14）+ 碰撞不硬 reset（记录 12）+ **方案 A 部分：76 维 TD3 状态**（记录 15）。  
**未做**：完整 4 层 elevation 进 TD3（52 维）、Arena、A\*、SLAM、完整分层栈。

调参速查见 [一附 2 · F 调参指南](#reward-done-collision)。
