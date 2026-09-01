# 3D · TD3 训练记录（TRAIN_RECORDS）

> **编号说明**：本文 **Record 编号 = 第几次大训**（Record 1–6）。[`DEBUG_RECORDS.md`](DEBUG_RECORDS.md) 的「记录 N」是调试流水账，**DEBUG 记录号 ≠ 大训 Record 号**。踩坑见 DEBUG。
>
> **Record 6 `run_20260828_003323` 已作废**（越野/换图/碰撞结论一律无效）。窗口成功率再高也不算。见下文事后验尸。

---

## Run 对照

| 项 | Record 1 `run_20260822_142310` | Record 2 `run_20260822_195504` | Record 3 `run_20260822_233910` | Record 4 冒烟 `run_20260823_114142` | Record 5 `run_20260823_132957` | Record 6 `run_20260828_003323` **作废** |
|----|----------------------------------|------------------------------|------------------------------|--------------------------------------|--------------------------------|----------------------------------------|
| **时间** | 2026-08-22 14:23 | 2026-08-22 19:55 | 2026-08-22 23:39（Ctrl+C @ ~289k） | 2026-08-23 11:41（**116 ep 中止**） | 2026-08-23 13:29（t≈120k 换图失败停） | 2026-08-28 00:33（~260k / 3843 ep） |
| **state_dim** | **28** | **76** | **76** | **76** | **76** | **76** |
| **episodes** | 314 | 369 | 612 | **116** | **1186** | **3843（单图 playpen）** |
| **成功率（累计）** | 25.5% | 28.5% | 17.8% | 1.7% | **47.7%** | 75.1%（**无效**） |
| **成功率（窗口 50）** | 54% | 6% | 30% | 2% | **70%** | 末 60% / 峰 98%（**无效**） |
| **成功率（窗口 100）** | 44% | 10% | 37% | 2% | **61%** | 末 66%（**无效**） |
| **collision_count** | 旧列有碰撞 | **0** | **0** | 3（2 ep） | **100（51 ep）** | **19（10 ep）；collision 终止仅 3 局** |
| **终止构成** | — | timeout 14% / stuck 57% / success 29% | timeout 47% / stuck 29% / success 18% / rollover 7% | timeout 28% / stuck 66% / success 2% | **success 48% / stuck 47% / collision 1.5% / timeout 1.5% / rollover 2%** | success 75% / stuck 17% / rollover 8% / collision **0.08%**（空图刷分） |
| **mean reward** | -12.3 | -5.2 | +681（虚高） | -78.0 | **+42.7** | 不作越野结论 |
| **path_efficiency** | 无 | 0.63 | 0.28 | 0.70 | **0.86（末 50 ep 0.83）** | 不作越野结论 |
| **实际地图** | 池内多图（早期） | 池内 | 3 张平地 | 冒烟 | 3 张平地；t=120k 换图挂 | **整晚只有 `clearpath_playpen`；换图 12/12 失败** |

---

## Record 1 · 第一次大训 `run_20260822_142310`

### 训练前配置

- 16 线点云 + `/offroad/range_features`
- 软碰撞 + stuck 终止
- 坡/墙多层判别 `/offroad/elevation_features`（当时 state 仍 28）
- 换图 `world_switch_interval: 10000`
- spawn 门禁 heightmap

### 结果摘要

**特点**：碰撞事件能触发（旧 metrics 里多数 ep `collision=1`）；策略后期成功率上升。详见上表 Run 对照 Record 1 列。

---

## Record 2 · 第二次大训 `run_20260822_195504`

### 训练前配置

#### 76 维状态与地图池（第二次开训前）

| 改动 | 说明 |
|------|------|
| **76 维 TD3 状态** | `/offroad/td3_range_features` 3×24+4 |
| **删除 `rough_ramps.world`** | world 池剩 4 张 |
| **Reward** | `r_episode_start: 5`、`r_progress: 3`、`r_arrive: 27`、`r_step: -0.04` |
| **速度** | `max_v/max_w: 1.5` |
| **碰撞 yaml** | `wall_*_dist: 0.28`、`collision_frames: 5`、`max_collision_events: 1` |
| **科研 metrics** | outcome / path_efficiency / collision_count 等 |

#### 碰撞 / 翻车 / goal 重塑（第二次大训后、第三次开训前）

第二次大训暴露问题后，在第三次开训前完成代码与 yaml 重塑（**未单独开训**）。

**代码（`ros_env.py`）**

| 功能 | 改动 |
|------|------|
| **碰撞** | 去掉 `still_moving→slope`；低层近 + 顶层**不**开阔且持续 → **wall**；保留 `[collision] wall confirmed` |
| **rollover** | IMU roll/pitch ≥ 50°，spawn 后 **2 s** 宽限 → `outcome=rollover`，`r_rollover`，**早终止** |
| **stuck** | 仅路径位移窗口；与 rollover 分离；`r_stuck` 减轻 |
| **goal 可达** | spawn OK 后 `_validate_goal()`：临时置位 goal，检 min sector + z |
| **范围** | spawn/goal ±6 m；`min_goal_dist: 2.5`；`goal_tol: 0.55`；`max_ep_steps: 600` |

**yaml（`td3_train.yaml`）**

见 [`README1.md`](README1.md) 参数表；关键：墙 `wall_all_layers_dist: 0.32`、坡 `slope_upper_clear_dist: 1.3` / `slope_max_frames: 10`、奖 `r_arrive: 50` / `r_progress: 5` / `r_stuck: -12` / `r_rollover: -25`。

**metrics**

`train_td3_node.py` 新增 CSV 列 `rollover`、`rollover_rate_*`。

### 结果摘要

见上表 Run 对照 Record 2 列。

### 发现的问题

1. **`collision_count` 全程 0**：`_classify_obstacle()` 中 `still_moving`（`v_fwd > wall_v_forward_max`）把真墙判成坡；`wall_all_layers_dist` 降到 0.28，墙更难触发。
2. **翻车当 stuck**：顶墙/大摩擦翻车后在原地滚，位移窗口判成 `stuck`（211 局），无法区分 rollover。
3. **成功率先高后掉**：中期 success ~30%，末期窗口掉到 0%（可能过拟合局部、goal 不可达、翻车后脏状态）。
4. **spawn/goal 范围偏小**：±3.5 m，大量 timeout（ρ 仍 15–30 m）。
5. **坡判定偏严**：用户反馈「C 的范围」过严，坡被当墙或相反。

---

## Record 3 · 第三次大训 `run_20260822_233910`

### 训练前配置

相对 Record 2 后重塑的精进（参考 [DRL_Navigation_Robot_ROS2_Foxy](https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy) 的 `v - |ω|` 与到达 +100）。

#### Reward

| 项 | Record 2 后重塑 | Record 3 训练前 |
|----|-----------------|-----------------|
| `r_arrive` | 50 | **100** |
| `r_progress` | 5 | **7** |
| 前进鼓励 | 无 | **`r_forward_scale * max(0, v-\|ω\|)`** |
| 原地转惩罚 | 无 | **`r_spin_penalty`**（v 低且 \|ω\| 高） |
| 低前进量警告 | 无 | **`r_low_effort_penalty`**（持续低 v-\|ω\|） |

#### 碰撞

| 项 | Record 2 后重塑 | Record 3 训练前 |
|----|-----------------|-----------------|
| 罚分 | -2·2^(k-1)（不结束局） | **-25 / -50 / -100**（`collision_penalty_list`） |
| 结束局 | `max_collision_events: 0` | **`max_collision_events: 3`**（第 3 次墙撞 done） |
| `collision_dist` | 0.30 | **0.28** |
| `collision_v_min` | 0.05 | **0.06** |
| `collision_frames` | 4 | **3** |
| `wall_all_layers_dist` | 0.32 | **0.28** |

#### Stuck / 噪声

| 项 | Record 2 后重塑 | Record 3 训练前 |
|----|-----------------|-----------------|
| `stuck_time_sec` | 8 | **10** |
| 传感器噪声 | 固定 0 | **`noise_adapt_enable`**：按滑动窗口 success_rate 调 `range_features` `sensor_noise_std` + 可选 `state_noise_std` |

#### 地图池

相对 Record 2 后重塑：**删除 `gazebo_heightmap.world`**（spawn 异常，见 DEBUG 记录 21）；world 池剩 **3 张**：`clearpath_playpen`、`offroad_map`、`rough_obstacles`。

#### 保持不变

rollover、goal 可达 `_validate_goal()`、spawn/goal ±6 m、`rough_ramps` 已删、3 张 world 池。

#### 代码

- `ros_env.py`：`r_forward` / `r_spin` / `r_low_effort`；`collision_penalty_list`；`set_state_noise_std()`
- `range_features_node.py`：`sensor_noise_std` 高斯噪声（rosparam 可动态改）
- `train_td3_node.py`：`_NoiseAdapter` 按 `noise_adapt_window` 成功率调噪声

### 结果分析（2026-08-23）

**配置**：Record 3 训练前 yaml（`r_forward_scale: 1.1`、`r_arrive: 100`、`max_ep_steps: 750`、`max_timesteps: 500k`）。用户 Ctrl+C @ t≈289k steps，612 episodes。

#### 1. 数据摘要

| 终止原因 | 局数 | 占比 | 均 reward | 均 path_eff |
|----------|------|------|-----------|-------------|
| **timeout** | 287 | **46.9%** | **1150** | **0.048** |
| stuck | 175 | 28.6% | 217 | 0.452 |
| success | 109 | 17.8% | 384 | 0.464 |
| rollover | 41 | 6.7% | 166 | 0.706 |
| collision | 0 | 0% | — | — |

- **mean reward（累计）**：+681；末 50 ep 窗口 ~851 — 与 success/path_eff **严重背离**。
- **path_efficiency 趋势**：ep 0–99 均 0.45 → ep 300+ 降至 **0.20–0.26**；timeout 局 path_length 均 **155 m**（initial_goal_dist 仅 ~5–10 m）。
- **成功率趋势**：早期 4% → ep 500+ 窗口 **34%** → 末 12 ep **50%**；累计仍仅 17.8% 因前期 timeout 过多。

#### 2. 用户怀疑点验证

| 怀疑 | 结论 | 证据 |
|------|------|------|
| collision_count 全程 0 | **确认** | metrics `collision_count`/`collision_events` 全 0；`train.log` **0 条** `[collision] wall confirmed`；`r_coll=+0.0` 全部 612 局 |
| r_forward 太大致乱跑 | **高度可能（reward hacking）** | timeout 局均奖 **1150**、跑满 750 步；终端日志 ep=626 每步 `forward +1.6~2.1`（`r_forward_scale=1.1` × v≈2）；path_eff 持续下降 |
| timeout 是大头终止 | **确认** | 46.9% 累计；ep 400–499 窗口 timeout **66%**；287 局 length=750 |
| rollover 占 stacked 空白 | **确认** | rollover 6.7%；旧 `outcome_stacked_area` 仅 success/stuck/timeout → **~7% 空白即 rollover**（已修 plot 加 rollover+collision 层） |
| success < 0.2 比第一次差 | **累计是，窗口否** | 累计 17.8% < Record 1 的 25.5%；但末 50 ep 30%、末 12 ep 50% > Record 2 末 20 ep 0% |
| reward↑ success↓ | **确认 reward hacking** | mean_reward 从负变 **+681**；success 累计降、path_eff 腰斩 |

#### 3. 根因分析（按可能性）

1. **Reward hacking（r_forward + 长 timeout）** — `r_forward_scale=1.1` 每步最高 ~2.2（v=2, ω=0），750 步 timeout 可积 **~1000+** 前进奖，远超 `r_arrive=100`。智能体学会 **绕圈刷 v−|ω|**，ρ 仍 3–90 m 但 reward 极高。timeout 均奖 1150 vs success 384。
2. **碰撞检测仍失效（地形分类过宽）** — Record 3 训练前将 `wall_all_layers_dist` 收紧到 **0.26** 且 `terrain_classify_enable=true`：近墙时顶层 elevation 常 ≥ `slope_upper_clear_dist=1.5` → 判 **slope**（仅 `r_slope_penalty=-0.05`，不计 collision event）。与 Record 2 同类问题，阈值更严后更难触发 wall。
3. **max_ep_steps=750 过长** — timeout 占比从 Record 2 的 14% 飙到 47%；给刷分留出更多步数。
4. **rollover 合理但占比小** — 41 局、均 110 步；IMU 50° 阈值正常，非主因。
5. **噪声自适应可能加剧探索跑偏** — success 低时加噪声，配合高 r_forward 更易长距离 wandering（次要）。

#### 4. 建议调整（第四次训前，**先汇报再改 yaml**）

| 优先级 | 参数 | 当前 | 建议 | 理由 |
|--------|------|------|------|------|
| P0 | `r_forward_scale` | 1.1 | **0.15–0.3** | 对齐 DRL 文献量级；防 timeout 刷分 |
| P0 | `max_ep_steps` | 750 | **450–500** | 缩短无效 wandering；Record 2 用 600 时 timeout 仅 14% |
| P0 | 碰撞分类 | wall 0.26 + slope 宽 | **放宽 wall**：`wall_all_layers_dist: 0.32`；或 `min_d < 0.22` 时 **强制 wall** 绕过 slope | 恢复 `wall confirmed` 与 -20/-40/-80 惩罚 |
| P1 | `r_arrive` | 100 | 保持 100 或暂 **50** | 若降 r_forward 后可保持 100 作主要目标 |
| P1 | `r_progress` | 7 | **5** | 略降，避免与虚高 forward 叠加 |
| P1 | `collision_dist` | 0.28 | **0.30** | 略放宽触发（配合分类修） |
| P2 | `slope_upper_clear_dist` | 1.5 | **1.0** | 减少「近墙顶层开阔→坡」误判 |
| P2 | `stuck_time_sec` | 8（yaml 写 8，注释误写 10） | 保持 8 | stuck 29% 尚可 |
| P2 | `noise_adapt_enable` | true | 第四次可先 **false** | 隔离 reward 改动效果 |

**代码（非 yaml）**：`plot_td3_metrics.py` 已加 **rollover + collision** 至 `outcome_stacked_area`；碰撞逻辑建议在 `_classify_obstacle` 增加 **极近距离强制 wall**（`min_d < 0.20`）— 待用户确认后改 `ros_env.py`。

#### 5. 图表

- `outcome_stacked_area.png` 已重绘（含 rollover 橙色层）；collision 层预留（本次数据为 0）。

---

## Record 4 · 第四次大训冒烟 `run_20260823_114142`（2026-08-23，116 ep 中止）

同 Record 4 配置方向，仅跑 **116 ep**（t≈28k）后中止；成功率 1.7%，collision_count 已有 3 次（2 ep），说明碰撞管线在冒烟中可触发。完整第四次/第五次长训见 Record 5。

---

## Record 4 · 第四次大训配置准备（2026-08-23）

当前 `td3_train.yaml` / `ros_env.py` 即为 Record 4 训练前配置。相对 Record 3 教训的精进（用户确认方向）。

### 训练前配置

#### Record 3 教训（必记）

| 问题 | 证据 | Record 4 对策 |
|------|------|---------------|
| **collision_count 全程 0** | 612 ep 无 `[collision] wall confirmed` | 碰撞分类重做：≥2 层近距判墙 + `min_d<0.19` 极近强制墙 |
| **reward hacking** | timeout 均奖 1150；`r_forward_scale=1.1` 每步 +1.6~2.1 | `r_forward_scale: 0.08`；`max_ep_steps: 500`；**`r_timeout: -20`** 抑制超时刷分 |
| **path_eff 腰斩** | 0.45 → 0.20；timeout 跑满 750 步 | 缩短 ep + 降 forward 奖 |
| **累计 success 误导** | 累计 17.8% 但末 50 ep 窗口 30% | 主图 `success_rate.png` 改突出 **滑动窗口**（50 ep） |

#### 碰撞分类重做（`ros_env._classify_obstacle`）

| 项 | Record 3 训练前 | Record 4 训练前 |
|----|-----------------|-----------------|
| 墙判定 | **4 层全** `< wall_all_layers_dist` | **≥ `wall_layers_required`（2）层** 近距 |
| 极近强制墙 | 无 | **`min_d < wall_force_min_dist`（0.19）** 无条件绕过坡判 |
| `wall_all_layers_dist` | 0.26 | **0.27**（方案 A 终版；↓ 自草案 0.32，减石子/坡面误触） |
| `slope_upper_clear_dist` | 1.5 | **1.0**（减「近墙顶层开阔→坡」误判） |
| `collision_dist` | 0.28 | **0.28**（与层距同步收紧） |
| `collision_penalty_list` | [-20,-40,-80] | **[-25,-50,-100]** |

**根因**：低障碍（≤20° 仰角）时顶层 +20° 层照不到 → 旧逻辑 `all_layers_low=false` + 顶层开阔 → **误判 slope**。

#### 方案 A 终版参数（2026-08-23，用户确认）

保留 `terrain_classify_enable: true` 与 elevation 多层判别；在 Record 4 草案基础上**收紧层距、缩短极近强制墙距离**，减少坡面/小石子误触墙，真墙仍可由 ≥2 层近或极近触发。

| 键 | 草案值 | **终版** | 理由 |
|----|--------|----------|------|
| `wall_layers_required` | 2 | **2** | ≥2 层近判墙（非 4 层全近） |
| `wall_all_layers_dist` | 0.32 | **0.27** | 层距略紧，减地面石子/坡面单层误触 |
| `wall_lower_layers_dist` | 0.32 | **0.27** | 与 `wall_all_layers_dist` 同步 |
| `wall_force_min_dist` | 0.22 | **0.19** | 仅极近才强制墙，坡/小障碍不易误触 |
| `collision_dist` | 0.30 | **0.28** | 与层距同步略收紧 |
| `slope_upper_clear_dist` | 1.0 | **1.0** | 保持，顶层开阔仍可豁免坡 |
| `terrain_classify_enable` | true | **true** | 保留 elevation 坡/墙分类 |

**代码**：`_classify_obstacle` 极近强制墙改为 **无帧数/速度门控**（`min_d < wall_force_min_dist` 即返回 `wall`）；事件防抖仍由 `_update_collision` 的 `collision_frames` 负责。

#### Reward

| 项 | Record 3 训练前 | Record 4 训练前 |
|----|-----------------|-----------------|
| `r_forward_scale` | 1.1 | **0.08** |
| `r_progress` | 7 | **5** |
| `r_arrive` | 100 | **100**（保持） |
| `max_ep_steps` | 750 | **500** |
| `max_v` / `max_w` | 2.0 / 1.5 | **不变** |
| `noise_adapt_enable` | true | **false**（隔离变量） |

#### Spawn grace 翻车作废（2026-08-23）

| 项 | 说明 |
|----|------|
| `rollover_grace_abort_enable` | `true`：spawn 后 `rollover_grace_sec`（2 s）内 IMU \|roll\|/\|pitch\| ≥ 50° → **本局作废**、`reset` 重采样起终点 |
| 训练侧 | `info.episode_abort` → **不**写 replay、**不** `metrics.csv`、**不** `ep++` |
| 日志 | `[rollover] spawn grace flip → abort episode (resample)` |
| grace 期外 | 行为不变：`r_rollover=-25` + `outcome=rollover` |

#### 成功率统计

- `metrics.csv` 已有 `success_rate_win`（默认 50 ep，`~metrics_window`）。
- **`success_rate.png`** 主曲线改为 **滑动窗口成功率**；累计为虚线参考。
- 评估训练进度以 **窗口率** 为准，非累计。

#### Path efficiency

- 本项目：`path_efficiency = initial_goal_dist / path_length`（越接近 1 越直）。
- [ITD3 2025](https://doi.org/10.3389/frobt.2025.1625968) 报告 Success / Collision / Time-to-goal；路径效率在 DRL 导航文献中常用 **欧氏距离 / 实际路径长度**（或等价比值），与上式一致。

#### 文献 / GitHub 碰撞借鉴

| 来源 | 借鉴点 |
|------|--------|
| [Cardiff RL 导航](https://orca.cardiff.ac.uk/id/eprint/153473/) | **local minimum / stuck 与 collision 分开**；惩罚导致定位失败或困在边缘态的行为；历史轨迹进 state 脱困 — 我们阶段 A 用路径 stuck + rollover |
| [Cardiff HRL + novelty](https://orca.cardiff.ac.uk/id/eprint/172147/) | 死胡同/走廊 local minimum 用 **内在奖励 + 记忆** 促探索；我们暂用规则终止，后续可考虑 |
| [Frontiers ITD3 2025](https://doi.org/10.3389/frobt.2025.1625968) | **Success rate + Collision rate** 双指标评估；碰撞 = episode 以撞障终止的比例；timeout 计失败 |
| [DRL_Navigation_ROS2_Foxy](https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy) | **LiDAR min range + 速度** 触发碰撞；到达 +100 / 撞 -100；位移停滞单独终止 |
| DRL 导航 GitHub 通用 | 射线 **first-hit min distance** 判碰；近距阈值 + 连续帧防抖；多 mesh raycast 取最近 hit |

#### 碰撞逻辑对比（调研报告 2026-08-23）

> **状态说明**：`ros_env.py` / `td3_train.yaml` 已含 Record 4 草案（≥2 层墙 + 极近强制墙）。**建议先短冒烟验证碰撞能否触发，再开第四次大训**。

| 维度 | Cardiff RL（Grando 等，arXiv:2209.14271） | Frontiers ITD3 2025 + GitHub TD3 nav | **我们当前（Record 4 配置）** | 用户提议（2/4 层） |
|------|------------------------------------------|--------------------------------------|-------------------------------|------------------|
| **传感器** | 平面 LiDAR 光束（无仰角分层） | 2D LaserScan / Velodyne 压成距离向量 | **16 线点云 → 4 仰角桶**（≈-10°/0°/+10°/+20°）+ 前向 12 sector | 同左 |
| **碰撞触发条件** | 仿真接触 / 近距惩罚项 `r_c`（论文侧重 reward，非多层逻辑） | `min(laser) < 0.35 m` → **当步 done**；无连续帧 | 前向 ±100° + `min_d<0.28` + **`v_app>0.06`** + **连续 3 帧** + 墙分类通过 | **≥2/4 层近** + 极近强制墙（0.19 m） |
| **坡 vs 墙** | 无（平地 clutter） | 无（2D 射线） | **slope 豁免**：低/中层近 + 顶层开阔 + 短暂/抬头 → 仅 `r_slope=-0.05` | 同左，但放宽墙触发 |
| **终止 vs 罚分** | 碰撞 `r_c=-100`；timeout `r_t`；local min 靠 shaping **不终止** | 碰撞 **立即 done**，-100；timeout 计失败 | **非立即 done**：-25/-50/-100 递增；**第 3 次墙撞 done**；stuck/rollover 独立终止 | 同 Record 4 方向 |
| **接近速度门控** | 无 | 无 | **`v_app = v·cos(bearing)+v·sin(bearing)`**，低于 0.06 不记碰撞 | 可选保留（文献大多无） |

**Record 3 根因（已确认）**：612 ep `collision_count=0`；`train.log` 无 `[collision] wall confirmed`。主因是 `_classify_obstacle` 把近墙判成 **slope**（顶层 +20° 照不到低障碍 → `upper_clear`；曾用 `still_moving→slope`；`wall_all_layers_dist=0.26` 过严）。

**三方案摘要**：

| 方案 | 要点 | 推荐度 |
|------|------|--------|
| **A** 2-of-4 + 极近强制墙 | 与 Record 4 配置一致；保留坡豁免 | ★★★ 首选（已实现，待冒烟） |
| **B** 2D min_dist + 速度 | 对齐 DRL_Navigation_ROS2_Foxy；去掉 elevation 分类 | ★★ 备选（坡场景误撞风险） |
| **C** 混合 | 水平 `min_d` 触发碰撞；elevation **仅**做坡豁免 | ★★★ 与 A 等价，逻辑更清晰 |

### 训前检查（用户手动开训）

#### 开训时自动备份（Record 4 新增）

`train_td3_node` 启动后立即在 `3D/runs/run_YYYYMMDD_HHMMSS/` 写入：

| 文件 | 内容 |
|------|------|
| `config.yaml` | **实际生效**参数（yaml + launch 覆盖后的 `train` + `env` 段） |
| `params_backup.yaml` | 与 `config.yaml` 相同（双份便于归档） |

`meta` 段含：`started_at`、`initial_world`、`world_basename`、`state_dim`、`action_dim`、`max_action`、`workspace_root`、`record: 4`。

#### Record 4 终版 yaml 快照（`td3_train.yaml`，2026-08-23）

| 类别 | 关键值 |
|------|--------|
| **训练** | `max_timesteps: 200000`（长训 launch 覆盖 **400000**）、`start_timesteps: 2000`、`world_switch_interval: 20000`（长训覆盖 **10000**）、`batch_size: 512`、`tau: 0.006` |
| **Reward** | `r_episode_start: 5`、`r_progress: 5`、`r_arrive: **120**`、`r_step: -0.08`、`r_forward_scale: **0.05**`、`r_stuck: -40`、`r_rollover: -50`、`r_timeout: -20`、`max_ep_steps: 500` |
| **碰撞** | `collision_dist: 0.34`、`wall_all_layers_dist: **0.31**`、`wall_force_min_dist: **0.25**`、`wall_layers_required: 2`、`collision_penalty_list: [-25,-50,-100]`、`max_collision_events: 3` |
| **噪声** | `noise_adapt_enable: false`、`state_noise_std: 0.02` |

#### 奖励分项曲线（Record 4 新增）

- `ros_env.py`：每局累积 `_ep_reward_parts` → `info.reward_parts_episode`
- `metrics.csv`：新增 `r_progress_sum`、`r_forward_sum`、`r_collision_sum` 等 11 列
- `plot_td3_metrics.py`：
  - `reward_components_stacked_area.png` — 各分项 **\|value\| 占比**（按 episode bin）
  - `reward_components_curves.png` — 各分项 episode 总和滑动平均

#### py_compile

```bash
python3 -m py_compile \
  src/scout_offroad_3d/scripts/td3/ros_env.py \
  src/scout_offroad_3d/scripts/train_td3_node.py \
  src/scout_offroad_3d/scripts/plot_td3_metrics.py
```

#### 短冒烟（不自动长训）

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0
```

**期望日志关键字**（至少出现若干条）：

| 关键字 | 含义 |
|--------|------|
| `Saved effective params →` | 参数备份 OK（含 `state_dim=76`） |
| `[spawn] OK` | spawn 门禁通过 |
| `[goal] OK` 或 `[goal] reject` | goal 可达性检测在跑 |
| `[collision] wall confirmed` | 墙撞事件触发（Record 4 核心验证） |
| `[terrain] slope suspected` | 坡豁免仍在工作 |
| `[reward] ep=... END` | 分项 reward 日志 |
| `[ep=...] done outcome=` | episode 结束 + metrics 写入 |

冒烟结束后检查 run 目录：`config.yaml` / `params_backup.yaml` / `metrics.csv`（含 `r_*_sum` 列）。

#### 第四次大训命令（模板，需用户手动执行）

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=400000 \
  start_timesteps:=2000 \
  world_switch_interval:=10000
```

说明：launch 覆盖 yaml 默认的 `max_timesteps: 200000` 与 `world_switch_interval: 20000`；其余 reward / 碰撞 / 地形参数以 `td3_train.yaml` 为准（已写入 `config.yaml` 备份）。

---

## Record 5 · 第五次大训 `run_20260823_132957`（2026-08-23）

### 训练配置摘要

相对 Record 4 冒烟终版，本 run `config.yaml` 关键差异：`r_progress: 7`、`r_arrive: 120`、`r_forward_scale: 0.13`、`r_stuck: -70`、`r_timeout: -30`、`max_steps: 500`；碰撞 `collision_dist: 0.34`、`wall_all_layers_dist: 0.31`、`wall_force_min_dist: 0.25`、`wall_layers_required: 2`。`meta.record` 仍标 4（备份脚本未改号）。

**训练量**：1186 ep，t=119982；`world_switch_interval: 20000`；t=120000 换图失败（Gazebo 连接拒绝）后结束。

### 结果分析（2026-08-23）

#### 1. 数据摘要

| 指标 | 数值 |
|------|------|
| **累计成功率** | **47.7%**（566/1186） |
| **滑动窗口 50 ep（末）** | **70.0%** |
| **滑动窗口 100 ep（末）** | **61.0%** |
| **collision_count 总和** | **100**（51 ep 非零；18 ep 以 collision 终止） |
| **path_efficiency 均值** | **0.856**（success 局 0.841；末 50 ep 0.83） |
| **mean_reward（累计）** | **+42.7**（无 Record 3 式刷分） |
| **timeout 占比** | **1.5%**（18 ep） |

**终止构成**：success 47.7% / stuck 47.3% / rollover 1.9% / timeout 1.5% / collision 1.5%。

**ep~600 前后**：滑动窗口成功率 ep560–580 峰 **~88%** → ep610–670 谷 **42–56%** → 末段回升 **70%**。换图时刻：t=60k→ep≈475（clearpath），t=80k→ep≈745（offroad_map）；**ep600 回落与 t=80k 换图非同一时刻**，更似高峰后策略震荡/泛化回落，换 offroad 后窗口率仍维持 60–72%。

#### 2. 奖励分项（末 100 ep \|value\| 占比）

| 分项 | 占比 |
|------|------|
| r_arrive | 48.6% |
| r_progress | 21.7% |
| r_stuck | 17.2% |
| r_collision | 3.5% |
| r_forward | 3.5% |
| r_timeout | ~0% |

成功局均奖 +162（arrive 120 + progress 36）；timeout 局均 -97；collision 局均 -155（r_collision ≈ -175）。

#### 3. 用户三点结论验证

| 结论 | 判定 | 证据 |
|------|------|------|
| 目的导向/朝向目标奖励有效 | **成立** | arrive+progress 占末段奖励 ~70%；path_eff 0.86；success 局 path_eff 0.84 |
| 碰撞范围+2层线合理 | **成立** | collision_count 100（R1–R3 为 0）；51 ep 有碰撞事件；collision 终止 1.5% |
| max_ep_steps 调小正确；难阶段 ep~600 下降 | **部分成立** | timeout 1.5% vs R3 47%；ep600 窗口回落数据可见，但换图在 ep≈745 |

#### 4. 图表

`success_rate.png`（窗口峰 0.92→谷 0.42→末 0.72）、`outcome_stacked_area.png`（stuck 与 success 对半）、`reward_components_stacked_area.png`、`path_efficiency.png`、`collision_count.png` — 均在 run 目录。

#### 5. 相对 Record 1–4 的进步与短板

**进步**：成功率窗口 61–70%（R3 末窗 30–37%）；path_eff 0.86（R3 0.28）；timeout 近消除；碰撞检测恢复。

**短板**：**stuck 仍占 47%**（主失败模式，类 Cardiff local minimum）；累计成功率被前 200 ep 零成功拖累；绝对成功率仍低于平面 TurtleBot3 文献 75–93%。

#### 6. 暂不实施的改进方向（待用户确认）

- 加强 **stuck / local minimum** 脱困（Cardiff 式惩罚边缘态、内在奖励或历史轨迹进 state）
- 评估 `r_stuck: -70` 与 `r_progress: 7` 比例，减少「贴墙蹭 progress 后 stuck」
- `r_forward_scale: 0.13` 高于 Record 4 草案 0.05，长期观察是否复现刷分
- 换图失败（t=120k）需排查 Gazebo 稳定性后再续训
- 续训优先看 **滑动窗口** 与 stuck 率，非累计

---

## Record 6 · 第六次大训配置准备（2026-08-23）

> **状态**：已于 2026-08-28 00:33 开训（`run_20260828_003323`，~260k / 3843 ep）。**结果作废**，禁止当作越野/换图/碰撞成功。见下一节事后验尸。
>
> 下文是训前清单（历史）。换图 identity bug、空 playpen、碰撞过松均在事后验尸与 DEBUG 记录 32/33 处理。

### Record 6 开训前变更清单

| # | 类别 | 变更 | 状态 |
|---|------|------|------|
| 1 | **探索奖励** | Cardiff occupancy：首次 cell `r_explore`（**0.03**）；`explore_enable: true` | ✅ |
| 2 | **IMU / rollover** | 换图 `_ensure_imu_subscriber`；`imu_wait_after_respawn_sec: 8`；`imu_model_fallback_after_switch_steps: 50` | ✅ |
| 3 | **换图稳定性** | `world_switch_max_retries: 5`；recovery 失败 skip switch、不 break | ✅ |
| 4 | **Launch / yaml** | 日常只改 yaml；`max_timesteps` 空串不 crash（DEBUG 记录 29） | ✅ |
| 5 | **Reward 比例** | **与 Record 5 相同**：`r_arrive=120`, `r_progress=7`, `r_forward_scale=0.13`, `r_stuck=-70` 等 | ✅ 不变 |
| 6 | **DEM 山地形** | `yosemite` / `volcano` **用户否决已删**（DEBUG 记录 31） | ❌ |
| 7 | **颠簸越野** | `bumpy_offroad.world`（PX4 uneven_ground）；已入 launch 池，**须短冒烟** | 📋 训前 |
| 8 | **缓坡自建** | 20–50 m 更小起伏仍缺；见 `SOURCES.md` | 📋 后续 |

### 相对 Record 5：改什么 / 什么不变

| 类别 | Record 5 | Record 6 **增量** |
|------|----------|-------------------|
| **探索奖励** | 无 | `explore_enable: true`、`explore_cell_reward: 0.03`、`explore_grid_res: 0.5`；metrics `r_explore_sum` |
| **IMU / rollover** | 换图后可能漏检 | `_ensure_imu_subscriber`；`imu_wait_after_respawn_sec: 8`；`imu_model_fallback_after_switch_steps: 50` |
| **换图稳定性** | t=120k 失败即停训 | `world_switch_max_retries: 5`；recovery 失败 **skip switch、继续训** |
| **World 池** | 3 张平地 | **+1** `bumpy_offroad`（共 4 张）；DEM 山地形已删 |
| **Reward 基线** | 见下表 | **保持**（用户确认不改） |
| **checkpoint 续训** | — | **未实现** — Record 6 = **全新 run**（见下「续训说明」） |

### 训练前配置（已实现 · 代码）

| 类别 | 改动 |
|------|------|
| **IMU / rollover** | 换图 `_ensure_imu_subscriber`；`_reset_imu_state()`；`_imu_fresh` + stale；spawn grace abort（DEBUG 记录 24） |
| **换图稳定性** | `world_switch_max_retries: 5`；`world_switch_*_sec` 加长；recovery 失败不 `break` |
| **探索** | 每局 occupancy；首次 cell `+0.03`；重访 0 |
| **调参** | **只改 `td3_train.yaml`**；launch 仅 CLI 显式传参时覆盖（DEBUG 记录 29） |

### Cardiff 探索奖励 · 三场景标定

**背景**：Cardiff HRL + novelty（[Gao et al., RAS 2024](https://doi.org/10.1016/j.robot.2024.104815)）。Record 6 **方案 A 简化**：2D occupancy，cell 首次访问给小额 `r_explore`。

| 项 | Cardiff 论文 | 本项目（Record 6） |
|----|-------------|-------------------|
| 公式 | \(r_{\mathrm{int}} = G \cdot \eta / \mathrm{NC}\) | 首次 cell：`r_explore = explore_cell_reward`（**0.03**）；可选 `explore_cardiff_scale` |
| 空间离散 | subgoal 阈值 **0.3 m** | `explore_grid_res: **0.5**` m；重访 0 |
| 开关 | intrinsic on/off 消融 | `explore_enable: true/false` |

**标定（`r_progress: 7`，`r_step: -0.03`）**

| 场景 | 单步分项 | 净效果 |
|------|----------|--------|
| **① 靠近目标** Δρ=+0.2 m | `r_progress` **+1.4** + `r_explore` ≤0.03 + `r_step` -0.03 | **净正**（≈+1.37） |
| **② 背离目标** Δρ=−0.2 m，进 1 新格 | `r_progress` **−1.4** + `r_explore` ≤0.03 + `r_step` -0.03 | **净负**（≈−1.40）；\|progress+step\| / explore ≥ **47×** |
| **③ 回旧格** | `r_explore=0` | 无探索白嫖 |

**代码**：`ros_env.py` `_explore_cells`；`metrics.csv` `r_explore_sum`；`plot_td3_metrics.py` 色板已接入。

### Record 5 基线（Record 6 **保持不变**）

| 键 | 值 |
|----|-----|
| `r_arrive` | **120** |
| `r_progress` | **7** |
| `r_forward_scale` | **0.13** |
| `r_stuck` | **-70** |
| `r_rollover` | -60 |
| `r_timeout` | **-30** |
| `r_episode_start` | 4 |
| `r_step` | -0.03 |
| `max_ep_steps` | 500 |
| `collision_dist` | 0.34 |
| `wall_all_layers_dist` / `wall_force_min_dist` | 0.31 / 0.25 |
| `wall_layers_required` | 2 |

### Record 6 完整 yaml 快照（`td3_train.yaml`，2026-08-23）

| 类别 | 关键值 |
|------|--------|
| **训练** | `max_timesteps: 200000`、`start_timesteps: 2000`、`world_switch_interval: 20000`、`batch_size: 512`、`expl_noise: 0.13` |
| **Reward** | 上表 Record 5 基线 + **探索**：`explore_enable: true`、`explore_cell_reward: 0.03`、`explore_grid_res: 0.5` |
| **IMU / 换图** | `imu_wait_after_respawn_sec: 8`、`imu_model_fallback_after_switch_steps: 50`、`world_switch_max_retries: 5` |
| **噪声** | `noise_adapt_enable: false` |

### 调参方式

- **日常只改** `src/scout_offroad_3d/config/td3_train.yaml`；不传 `max_timesteps:=` 等即读 yaml。
- 改 yaml 后须 **重启 roslaunch**。
- 冒烟/临时才用 CLI：`max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0`。

### World 池（4 张 · launch 随机池已同步）

| 文件 | 地形 | 依赖 / 备注 |
|------|------|-------------|
| `clearpath_playpen.world` | 平地障碍 | Gazebo models |
| `offroad_map.world` | 平地 clutter | `model://` |
| `rough_obstacles.world` | 平地 box | 无 |
| `bumpy_offroad.world` | **缓起伏 heightmap** 80×80 m | `~/.gazebo/models/uneven_ground`；训前须冒烟 |

已删除：`yosemite_heightmap` / `volcano_dem`（过陡过大，DEBUG 记录 31）。清单见 [`worlds/SOURCES.md`](../src/scout_offroad_3d/worlds/SOURCES.md)。

### 开训前检查清单

- [ ] `python3 -m py_compile`：`ros_env.py`、`train_td3_node.py`、`plot_td3_metrics.py`
- [ ] 基础冒烟 → `[spawn] OK`、`[collision] wall confirmed`、无 `int('')` 崩溃
- [ ] 换图冒烟 → 换图后 `[imu]`、`ep=` 持续
- [ ] `bumpy_offroad` 短冒烟 → `[spawn] OK`（见 README1 §4.5）
- [ ] 日志 `TD3 train: max_timesteps=200000 ...`（未 launch 覆盖时）

### 短冒烟

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0
```

`bumpy_offroad` 单独冒烟见 [`README1.md` §4.0 / §4.5](README1.md#td3-train)。

### 第六次大训命令（推荐 · 需用户手动执行）

```bash
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch scout_offroad_3d train_td3.launch \
  gui:=false \
  max_timesteps:=500000 \
  start_timesteps:=4000 \
  world_switch_interval:=20000
```

仅读 yaml（`200000` 步）：

```bash
roslaunch scout_offroad_3d train_td3.launch gui:=false
```

### 续训说明

| 方式 | 现状 |
|------|------|
| **同目录 checkpoint 续训** | ✅ `output_dir:=<run>` + `resume:=true` + `max_timesteps` **>** 当前 t（加载 `td3_*`；replay 仍空） |
| **Record 5 权重推理** | ✅ `infer_td3.launch weights:=.../run_20260823_132957/td3` |
| **Record 6 续训** | 命令可用，但 **该 run 已作废**（仅 playpen）；不要当越野续训 |
| **换图失败** | 修后：回旧图须打 `fallback … NOT a map change`，禁止 `recovery OK` 冒充换图成功 |

**开放缺口（训前，部分已被 Record 6 打脸）**：① 当时 `train_td3_node` 续训已补上（见 README1 resume）。② 更小范围缓坡地形仍缺。③ **换图 identity 未在开训前验证**：`current_world` / `world_spawn_z` 在 spawn 校验之后才更新 → 整晚锁死 playpen。

---

## Record 6 · 第六次大训事后验尸 `run_20260828_003323`（2026-08-28）**作废**

**结论先写死：这晚 ~90% 窗口成功率不能当越野成绩。** 4000+ 局 / ~260k 步 **100% 浪费**（相对「多图越野 + 能撞墙」的开训目标）。数字好看，是因为车在一张空旷 playpen 里刷到达，不是学会了越野。

### 0. 数据（只描述发生了什么，不当成功）

| 项 | 值 |
|----|-----|
| 目录 | `3D/runs/run_20260828_003323` |
| 启动 | 2026-08-28 00:33；`max_timesteps: 300000`；`world_switch_interval: 20000` |
| 规模 | **3843 ep**，末行 t=**259949**（用户在 t=260000 换图现场抓到同样的 z_jump） |
| 首张 / 唯一图 | `scout_offroad_3d/worlds/clearpath_playpen.world` |
| 窗口成功率 | 峰 **98%**（ep≈1483）；ep1000/3000 约 **90%**；末 50 **60%** / 末 100 **66%**；累计 **75.1%** |
| 碰撞 | `collision_count` 合计 **19**（10 局）；`outcome=collision` **3/3843（0.08%）**；大量 `coll_ev=0` |
| 换图 | interval 每 20k 一次；**12 次全部失败**，每次 `recovery OK → clearpath_playpen` |

终止：success 2885 / stuck 650 / rollover 297 / timeout 8 / collision 3。前期大量 rollover，中后期在空场里 success 堆起来。

### 1. 四条作废理由（用户全部成立）

| # | 用户指控 | 判定 | 证据 |
|---|---------|------|------|
| 1 | 整晚只有 `clearpath_playpen` | **成立** | `config.yaml` `initial_world=.../clearpath_playpen.world`；`train.log` 无任何 `switch to …` 成功行；12 条均为 FAILED + recovery 回 playpen |
| 2 | 换图计划 100% 失败 | **成立** | t=20k/40k/60k/140k/160k/200k → `bumpy_offroad` spawn 校验失败（日志仍写 `world=clearpath_playpen`）；t=80k/240k `offroad_map` Connection reset/refused；t=100k/120k/180k/220k `rough_obstacles` 连接失败。0 次 `switch complete` 到新图 |
| 3 | 地图太空，不是用户以为保存的密障碍图 | **成立（机制）** | 见 §3。本 run **从未离开 stock playpen**。围栏在 ~±10 m，训练框 ±7 m，内部几乎是空地 + 一条隔离墩。`offroad_map` / `rough_obstacles` / `bumpy` 整晚没训到 |
| 4 | 碰撞几乎不触发 | **成立** | Record 5 同量级阈值下 1186 ep 有 **100** 次 collision_count；本 run 3843 ep 仅 **19**。空图 + 策略学会绕开仅有的几块障碍 → `coll_ev=0` 是常态，不是「避障完美」 |

相对 Record 5：R5 至少在 3 张平地上跑过、碰撞管线证明能响（100 次）。R6 窗口率更高，**是空场过拟合，不是进步**。

### 2. 换图失败根因（为什么这次挂、以前不完全一样）

**吸烟枪（用户 t=260000 现场，与更早的 20k/40k/… bumpy 失败同一 bug）**：

```
[world] switch to bumpy_offroad.world at t=260000
[world] starting: ... gzserver ... bumpy_offroad.world
[world] spawning model scout_mini at z=0.200 (world=clearpath_playpen.world) ...
[spawn] FAIL ... reason=z_jump z=1.067 ... target_z=0.200 xy=(0.00,0.00)
```

Gazebo **已经加载** `bumpy_offroad`（heightmap 地面 z≈1.06）。Spawn 校验仍用 **旧图名 playpen + spawn_z=0.20**。`|1.067−0.20|=0.867 > spawn_z_jump_threshold 0.25` → 假 z_jump。z 重试最多抬到 `0.20+5×0.11=0.75`，永远够不到 1.06。6 次 abort → `world_switch_max_retries: 5` 用尽 → trainer **recovery 再起 playpen**。

代码位置（修前）：`switch_world()` 把 `self.current_world = world_path` 放在 `_spawn_robot()` **成功之后**。`_spawn_z_for_current_world()` / 日志 `world=` 读的是旧 identity。yaml 里 `world_spawn_z.bumpy_offroad.world: 1.05` **写了但换图时用不上**。

若 identity 正确：target_z=1.05，实测 1.067，dz=0.017 **远低于** 0.25，z_jump 不该响。

**和 Record 5 的差别**：R5 t=120k 是 Gazebo **连接拒绝** 后训练环 `break` 停训。R6 加了 retries=5 和「recovery 失败继续训」，本意是别停。结果变成：**每次换图失败都被包装成 recovery OK → 同一张 playpen**，训练接着刷空图，用户以为在换图。更糟的是 recovery 读 `env.current_world`（从未改成 bumpy）→ **永远回 playpen**，不是重试 bumpy。

`offroad_map` / `rough_obstacles` 的 Connection reset/refused：bumpy 假 z_jump 已经把 gzserver 杀/拉 5 轮，随后更重的 world 更容易连不上，然后又「成功」回到轻量 playpen。次要；主因仍是 identity。

**次要**：`[spawn] FAIL stuck_collision` at (-5.18,-5.72) 再 retry OK — 随机点贴障碍，不是换图主 bug。

**`[rollover] imu=fallback` 在 playpen 上**：每次 recovery 都重启 gzserver，并强制 `imu_model_fallback_after_switch_steps=50`。这是「假换图」的副作用，不代表 playpen 上 IMU 硬件坏了。

**已修**（DEBUG 记录 32）：换图前 `_apply_world_identity`；失败则 restore 旧 identity + `fallback … NOT a map change`；禁止再写 `recovery OK` 冒充换图成功。z_jump：heightmap z≈1.06 vs target 1.05 不得判 FAIL。

### 3. 地图审查（空了？被覆盖了？）

| 文件 | 状态 | 说明 |
|------|------|------|
| `src/scout_offroad_3d/worlds/clearpath_playpen.world` | 与 `scout_gazebo_sim/worlds/clearpath_playpen.world` **字节一致**（md5 `14a81b240ce3…`） | 2026-08-22 10:14 拷贝的 **stock Clearpath playpen**（gazebo_sim 源文件 mtime 08-03）；offroad 副本此后 **未改**。隔离墩围在 ~±10 m；训练 spawn ±7 m 内 pose 约 **10** 个，且多贴边（桶/锥/消防栓）。真正靠近场心的几乎只有 `jersey_barrier_15≈(2.45,0)` + `Dumpster_0≈(-2.35,-4.4)`。内部是空沥青，不是密障碍越野 |
| git | offroad 副本未跟踪（`?? worlds/`）；gazebo_sim 为 `A` | **没有**「用户密障碍 playpen 被 recovery 写盘覆盖」的证据。recovery 只是 **再加载这份 stock 文件**，不会改磁盘 |
| `offroad_map.world` | 在，mtime 08-23 17:42 | ±7 m 内约 **19** 个 clutter pose（树/泥箱/墙/托盘）。本 run **一次都没训到** |
| `rough_obstacles.world` | 在，mtime **08-28 11:29**（开训 00:33 **之后**） | ±7 m 内约 **36** 个 box/柱。若用户后改密了，改的是这张图，**不是 playpen**，也与本 run 无关 |
| `bumpy_offroad.world` | 在；`~/.gazebo/models/uneven_ground` **存在** | heightmap 可加载（否则不会出现 z≈1.06）。卡在错误 spawn_z=0.20，不是缺模型。`\|1.067−1.05\|=0.017 < 0.25`，identity 正确则 z_jump 不该响 |

**诚实句**：本 run 从未成功离开 playpen，用户当晚看到的就是这张空围栏场。密图还在磁盘上，但 **权重没见过它们**。

### 4. 碰撞过松（事后收紧，见 DEBUG 33）

本 run yaml：`collision_dist=0.34` / `wall_all_layers_dist=0.31` / `wall_force_min_dist=0.25` / `wall_layers_required=2` / `collision_frames=3` / `collision_v_min=0.07`（与 Record 5 同档）。R5 能撞是因为图里有东西；R6 空场 + 高成功率 → 几乎贴不上墙。阈值也偏松。

**已收紧（未再开长训）**：`collision_dist 0.38`、`wall_all_layers_dist 0.36`、`wall_lower_layers_dist 0.33`、`wall_force_min_dist 0.30`、`collision_frames 2`、`collision_v_min 0.05`；层数仍为 2。空图上收紧也救不了本 run 的权重。

### 5. 禁止从此 run 得出的结论

- 禁止说「越野导航 ~90%」。这是 **单张空 playpen**。
- 禁止说「换图稳定」。黄线/灰线是失败重启，不是换图成功（`plot_td3_metrics.py`）。
- 禁止说「碰撞逻辑验证通过」。3 局 collision 终止、19 次计数，相对 3843 局等于没测。
- 续训命令在 README1：可从 t=259949 `resume:=true` 接着写目录，但 **接着训仍是这份 playpen 过拟合权重**，不能当越野续训。

### 6. 下一训（用户手动；本文不启动）

须先短冒烟确认：`[world] identity → bumpy_offroad.world spawn_z=1.050`、`[spawn] OK world=bumpy_offroad`、**没有** `target_z=0.200` 对 z≈1.06 的 z_jump。换图成功日志必须是 `switch to` 且 **不是** `fallback … NOT a map change`。

---

## 文献 / GitHub 可借鉴点（简短）

| 来源 | 可借鉴 |
|------|--------|
| [DRL_Navigation_Robot_ROS2_Foxy](https://github.com/toxuandung/DRL_Navigation_Robot_ROS2_Foxy) | reward：`v - \|ω\|` 鼓励前进、惩罚原地转；到达 +100 / 碰撞 -100；**位移停滞**应单独终止而非混进碰撞 |
| [DRL-AUV-navigation](https://github.com/lanyangyang-gogogo/DRL-AUV-navigation) 类 | ROS episode 环：reset→sensor→cmd_vel→reward；**载体相关** reward 必须自调，不能搬 AUV 数值 |
| AUV RL 导航（arXiv:2606.08513 等） | **success rate + path length vs baseline** 评估；失败模式按场景分（stuck in local minimum）；传感器噪声有时能帮助脱困 — 我们阶段 A 用规则 rollover 更直接 |
| Cardiff RL 导航论文 | **惩罚 local minimum / 边缘态** 与 stuck 区分；teacher-student 可后做，现阶段 IMU rollover 是低成本 proprioception |
| [Frontiers ITD3 2025](https://doi.org/10.3389/frobt.2025.1625968) | Success / Collision / Path Efficiency 三指标 — 已写入 metrics.csv |

---

## 训前检查（通用）

```bash
python3 -m py_compile \
  src/scout_offroad_3d/scripts/td3/ros_env.py \
  src/scout_offroad_3d/scripts/train_td3_node.py

# 短冒烟（不自动长训）
killall -9 gzserver gzclient roslaunch rosmaster rosout 2>/dev/null; sleep 2
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_offroad_3d train_td3.launch \
  gui:=true max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0
# 期望日志：[collision] wall confirmed / [rollover] / [goal] OK / [reward] explore +0.0300（新格时）/ 无 int('') 崩溃
```
