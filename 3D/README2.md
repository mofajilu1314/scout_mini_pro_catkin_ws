# 3D · README2 — 科研出图专册

> **版本与归档注记（2026-10-25）**：本册完整保留出图方法、指标含义、命令、公式和验证解释。所有 `$SCOUT_WS_ROOT` 指向工作区根。短期 smoke、临时验证和调试输出在确认结果后应移到 `archive/validation/`，而正式训练 run、其权重和指标须按实验索引保留；不要用“清理”替代证据保存。

> **定位**：训练能跑不需要本册；但**论文 / 答辩**需要的图全在这里——科研曲线、定轨轨迹图、结局热力图、RViz 演示动图、TD3 损失/目标景观。
> 训练本身 → [`README1.md`](README1.md)；总规划 → [`README0.md`](README0.md)；实车对接 → [`README3.md`](README3.md)；LIO / Arena 选型 → [`README1.md` §11](README1.md#ch11) / [`README0.md`](README0.md)。

**前提**：已按 [`README1.md`](README1.md) 训出至少一个 run（`3D/runs/run_YYYYMMDD_HHMMSS/`，含 `metrics.csv`）。出图全部离线可做，**不需要** Gazebo / 服务器。

---

## 目录

0. [四类产物速查](#ch0)
1. [科研曲线（训完自动出）](#ch1)
2. [定轨轨迹图（按 Episode 冻结评测）](#ch2)
3. [结局空间热力图](#ch3)
4. [RViz / 相机演示动图](#ch4)
5. [文件布局](#ch5)
6. [注意](#ch6)
7. [长距评测出图（阶段 B/C）](#ch7)
8. [TD3 损失 / 目标景观图](#ch8)

---

<a id="ch0"></a>

## 0. 四类产物速查

| 图 | 命令 / 来源 | 产物 | 用途 |
|----|------------|------|------|
| **科研曲线** | `plot_td3_metrics` | `run_*/reward_curve.png` 等 16 张 | reward / 成功率 / 碰撞率 / 路径效率 |
| **定轨轨迹图** | `viz_eval`（每 N ep 冻结） | `run_*/viz_eval/epXXXX/traj_*.png`（默认 12 张） | 固定起终点轨迹对比 |
| **结局热力图** | `outcome_xy.csv` → `plot_td3_eval_viz` | `run_*/heatmaps/<world>/terminal.png` + `goals.png` | 按世界分子目录；固定 ±15 m（30×30）画布 |
| **RViz 演示动图** | `infer` + RViz2 / `record_camera_stub` | GIF / PNG 序列 | 答辩演示 |
| **损失 / 目标景观** | `plot_td3_loss_landscape`（§8） | `run_*/loss_landscape/*.png`（+ 可选 GIF） | Critic TD-loss / Actor 目标在参数切片上的曲面 |
| **指令速度统计** | `plot_td3_actions`（训毕自动） | `run_*/action_viz/*.png` | 每 ep 平均 v/ω 折线 + 全局加权均值 + (ep,v,ω) 三维图 |

---

<a id="ch1"></a>

## 1. 科研曲线（训完自动出）

训练结束（或 Ctrl+C 进 `finally`）自动调用 `plot_td3_metrics`，在**同一 run 目录**生成白底 PNG。手动重绘：

```bash
ros2 run scout_offroad_3d plot_td3_metrics $SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS
# 可选窗口 / 堆积 bin
ros2 run scout_offroad_3d plot_td3_metrics $SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS --window 20 --stack-bin 50
```

> **停训注意**：第一次 Ctrl+C 后请等终端出现 `Plotting metrics done` 再离开；二次 Ctrl+C 可能打断出图（权重/`metrics.csv` 仍在）。缺 PNG 时用上面命令补画；有 `outcome_xy.csv` 时再跑 `ros2 run scout_offroad_3d plot_td3_eval_viz <run_dir>` 补 heatmaps。

### PNG 清单（16 张）

| 图文件 | 内容 |
|--------|------|
| `reward_curve.png` | episode reward 原始曲线 |
| `reward_smooth.png` | reward + 滑动平均 |
| `mean_reward.png` | 累计/平滑 mean reward |
| `success_rate.png` | **滑动窗口成功率**（主曲线，w=50 ep）；累计为虚线参考 |
| `outcome_success_rate.png` / `outcome_stuck_rate.png` / `outcome_timeout_rate.png` | 成功 / 受困 / 超时终止比率（滑动窗口） |
| `outcome_stacked_area.png` | **堆积面积图**：每 N ep 内 success/stuck/timeout/rollover/collision 占比 |
| `reward_components_stacked_area.png` | 奖励分项占比（每 N ep 内各 `r_*_sum` 的 \|值\| 占比） |
| `reward_components_curves.png` | 各 reward 分项滑动平均曲线（**14 分项**；含 `r_high_effort`；旧 CSV 的 `r_forward` 作 legacy alias） |
| `episode_length.png` | episode 长度 |
| `collision_rate.png` / `collision_rate_window.png` / `collision_count.png` | 碰撞率（累计/窗口）与每局碰撞次数 |
| `stuck_rate.png` | 累计受困率 |
| `path_efficiency.png` | 路径效率（欧氏/实际路径，DRL 文献常用） |

**换图竖线**：`train.log` 有 `[world]` 时自动画在 episode 轴上。**黄/橙虚线** = 真换图成功；**灰虚线** = 换图失败/回退（**不要**当多图训练证据）。

**文献对齐指标**：Success Rate、Collision Rate、Episode Length、Path Efficiency（见 [Frontiers in Robotics 2025 ITD3](https://doi.org/10.3389/frobt.2025.1625968) 等）。

> **Record 7 后奖励已重标定**（高/低努力、对称倒车等）：分项含义与公式见 [`README1.md` §2.1](README1.md#ch2)；新 CSV 含 `r_high_effort_sum`（旧 run 可能只有 `r_forward_sum`，绘图脚本兼容）。

### metrics.csv 列（给 AI / 自己分析）

`episode` `t` `length` `reward` `success`/`stuck`/`timeout` `outcome` `collision` `collision_events`/`collision_count` `rho` `initial_goal_dist` `path_length` `path_efficiency` **`mean_v`/`mean_w`**（本 ep 发出的 cmd_vel 均值；`action_stats_enable`） `buf` `succ_total` `*_rate_cum`（累计）/ `*_rate_win`（滑动窗口）/ `mean_reward_cum` / `r_*_sum`（各 reward 分项，含 **`r_high_effort_sum`**；旧 CSV 可能仅有 `r_forward_sum`）。把 `metrics.csv` + 已有 PNG 发来即可解读。

依赖：`python3 -m pip install --user matplotlib`（缺包时训练仍成功，只是出图告警跳过）。

### 指令速度统计（`action_viz/`）

训练中每个 step 在内存累加发出的 `|v|/|ω|`（控制指令仍带符号），**episode 结束**写入 `mean_v`/`mean_w`（**无逐步 CSV**）。训毕自动（或手动）：

```bash
ros2 run scout_offroad_3d plot_td3_actions $SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS
```

| 图 | 内容 |
|----|------|
| `mean_v_vs_episode.png` | 每 ep 平均绝对值线速度；**图旁**全局加权 \(\bar v=\sum(\mathrm{mean\_v}\cdot L)/\sum L\) |
| `mean_w_vs_episode.png` | 每 ep 平均绝对值角速度；旁注全局 \(\bar\omega\) |
| `ep_v_w_3d.png` | 三维：episode / |v| / |ω|；轴限从 0 到 `max_v` / `max_w` |

旧 run 无 `mean_v`/`mean_w` 列则跳过。

---

<a id="ch2"></a>

## 2. 定轨轨迹图（按 Episode 冻结评测 · 主产物）

训练环每满 **`viz_eval_every_n_ep`（默认 100）** 个训练 episode：暂停训练、**冻结策略**（无探索噪声 / 不写 replay / 不 update），用开训前固定好的起终点跑 benchmark，输出轨迹图。评测**不计入**训练 `metrics.csv`。

### 训前预生成点对（关键）

开训前（TD3 主循环前）按世界切换，在与训练相同的 **`spawn/goal ±10` 框**内：

1. 每批采样 **20** 个 `dist ≥ viz_eval_min_pair_dist`（默认 **20 m**）的几何点对；不够距离的直接丢掉重采  
2. 再做与训练相同的 spawn / goal 放置校验  
3. 最多 **5** 批（约 100 候选）；每图尽量取 **3** 对（`viz_eval_pairs_per_world`）  
4. 仍不足 3 对：**少几对也开训**，写入 `viz_eval/pair_gen_report.yaml` + 日志 warn（不拒训）

产物：`run_*/viz_eval/benchmark_pairs.yaml`（整场复用，保证 ep100 / ep200 可比）。

**规模（默认）**：4 图 × 3 组 = **12** 条轨迹（可把 `pairs_per_world` 改回 5 → 20）。

### 代价（墙钟时间）

开 `viz_eval_enable` 后，每 N ep 要跑完当时所有点对（最坏约 `12 × max_ep_steps` 步 + 换图）。长训默认 **关**；要看量变→质变过程再开。`gui:=true` 时评测会先关 **gzclient**，跑完再开（`viz_eval_gui_pause`）。

### 与训练同源

- 成功 / stuck / timeout / rollover / collision：同一套 `env.step`  
- 步数上限：`viz_eval_max_steps ≤ 0` 时跟随 **`max_ep_steps`**（当前 600）

### 产物（`run_*/viz_eval/ep0100/` 等）

| 文件 | 内容 |
|------|------|
| `traj_<world>_p0N.png` | **单对轨迹，透明底**（默认最多 12 张；绿=success 黄=stuck 红=timeout 紫=rollover） |
| `traj_<world>_all.png` | 该图若干对合成一张（速览） |
| `traj_grid.png` | 多图网格总览 |
| `traj_on_map_<world>.png` | **已停用**（见下「俯视拍照已删除」） |
| `failure_pie.png` / `trajectories.json` / `meta.yaml` | 结局饼图、原始点、快照元数据 |

俯视底图缓存：`run_*/basemaps/<world>_clip5.5.png`。

### 开启（`td3_train.yaml`，改完重启 launch）

```yaml
viz_eval_enable: true
viz_eval_every_n_ep: 100          # 每隔多少训练 ep 评一次
viz_eval_pairs_per_world: 3       # ×4 = 12 张轨迹
viz_eval_min_pair_dist: 20.0      # 定轨最短距离；不够重采
viz_eval_pair_batch_size: 20
viz_eval_pair_batch_rounds: 5
viz_eval_max_steps: 0             # 0 = 跟随 max_ep_steps
viz_eval_gui_pause: true
viz_eval_log_train_xy: true
viz_eval_heatmap_goals: true
```

### 俯视拍照已删除（2026-09-07）

开训前选完 12 组起终点之后，原先会再对每张地图：**换图 → 删树冠 → 再 reload gzserver → 画一张绿色示意图**（`basemaps/<world>_clip5.5.png`，以及 `traj_on_map_*.png`）。

实际效果是一团糊绿，看不清地形，而且每张图多一次 Gazebo 重启，墙钟浪费很大。**这条拍照链路已经从训练流程里删掉**，不再生成那些绿图，也不再为此 hide/restore 世界。

定轨仍保留：透明底的 `traj_<world>_p0N.png`（以及训毕 `viz_by_pair/*/overlay_eps_transparent.png`）。不要把 `viz_topdown_enable` 再开回去。

> 现网 yaml：`viz_eval_enable: true`（每 100 ep 定轨）；`viz_topdown_enable: false`。

### 离线重画 / 冒烟（不启 Gazebo）

```bash
ros2 run scout_offroad_3d plot_td3_eval_viz $SCOUT_WS_ROOT/3D/runs/<run_dir>
VIZ_SMOKE="$SCOUT_WS_ROOT/3D/debug_logs/viz_smoke"
python3 -m scout_offroad_3d.plot_td3_eval_viz --smoke "$VIZ_SMOKE"
```

确认 smoke 结论后，把 `$VIZ_SMOKE` 移入 `archive/validation/`；不要把它留成活动 run 的旁支。

---

<a id="ch3"></a>

## 3. 结局空间热力图（按 world · 两套语义）

训练写入 `outcome_xy.csv`：

| `point_kind` | 点是什么 | 读法 |
|--------------|----------|------|
| **`terminal`** | episode 结束时**车的位姿** | 红堆≈常卡/撞处；绿≈常到达处 |
| **`goal`** | 该局设定的 **goal (x,y)** | 哪些目标区常失败 |

**不是**把四张地图混成一张主图；主产物按 world 分子目录：`heatmaps/<world>/terminal.png`、`goals.png`，画布固定 −15…15 m。旧版扁平 `heatmap_*` 可忽略。

产物示例：

- `heatmaps/<world>/terminal.png`（终局位姿；画布固定世界框 −15…15）
- `heatmaps/<world>/goals.png`（目标点；同上）

训后补画：`ros2 run scout_offroad_3d plot_td3_eval_viz <run_dir>`。

---

<a id="ch4"></a>

## 4. RViz / 相机演示动图（答辩用）

> 仅**本地 demo**（训出好权重后）；服务器长训不要开。需 `gui:=true`（相机要渲染；纯无头通常只有雷达）。

### 典型话题（Humble + scout_mini）

| 内容 | Topic |
|------|-------|
| RGB | `/camera/color/image_raw` |
| Depth | `/camera/depth/image_raw` |
| Depth points | `/camera/depth/points` |
| Lidar cloud | `/lidar/points`（无 GUI 也可） |
| 训练目标点 | `/offroad/train_goal` |
| cmd_vel | `/cmd_vel` |

用 `ros2 topic list | grep -E 'image|camera|lidar'` 核对。

### 演示启动

```bash
# 强权重推理 + Gazebo（可开 gui）
ros2 launch scout_offroad_3d infer_td3.launch.py gui:=true weights:=/path/to/run/td3

# 另开终端：RViz2（自行加 Image / LaserScan / TF / Marker）
rviz2
```

### 训练中途看车（方法1：不开 Gazebo 窗口）

满血 `gzclient` 会卡在树木/贴图加载；**不停训**时用 RViz 看点云 + 车 + 目标 + 车载相机。

**推荐启动顺序**（每个命令一个终端；不要关训练那个 launch）：

```bash
source /opt/ros/humble/setup.bash
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash

# 1) 目标绿球
ros2 run scout_offroad_3d train_goal_marker --ros-args -p use_sim_time:=true

# 2) 深度图转 RViz 友好格式（见下方「深度全黑」）
ros2 run scout_offroad_3d depth_viz_bridge --ros-args -p use_sim_time:=true

# 3) RViz（必须 use_sim_time）
rviz2 --ros-args -p use_sim_time:=true -- \
  -d $(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/config/train_watch.rviz
```

配置 [`config/train_watch.rviz`](../ros2_ws/src/navigation_3d/scout_offroad_3d/config/train_watch.rviz) 已开：

| 显示 | Topic | 说明 |
|------|--------|------|
| LidarCloud | `/lidar/points2` | 雷达障碍轮廓 |
| Odometry / RobotModel | `/odom`、`/robot_description` | 车位姿与模型 |
| TrainGoal | `/offroad/train_goal_marker` | 本局目标（绿球） |
| CameraRGB | `/camera/color/image_raw` | 车载 RGB（无头训练也在发） |
| CameraDepth | `/camera/depth/image_viz` | 深度可视化（见下） |

Fixed Frame 用 **`odom`**。若仍无点云，临时改成 **`base_link`**。

**能看见 / 看不见**

- 能：车、雷达轮廓、目标、RGB、深度灰度图。
- 不能：Gazebo 整图美化树林（方法2，等**本次训练结束后**再做圆柱 visual）。

#### 深度相机在 RViz 里全黑？

原始话题 `/camera/depth/image_raw` 是 Gazebo 的 **`32FC1`（米）**，且大量无效像素为 **`+inf`**。RViz 的 Image 控件对这种格式经常整窗发黑（RGB 正常、深度黑就是这个原因）。

处理：跑 `depth_viz_bridge`，把深度转成 **`16UC1`（毫米）** 发到 `/camera/depth/image_viz`（`inf→0`，裁到 6 m）。**不改**原始深度话题，不影响训练。

#### 轮子 RobotModel `No transform`

原因：`joint_state_publisher` 曾用墙钟时间戳，与 RViz 仿真时间对不上。若确认该 JSP 是**自己单独启动**的，先回到它原来的终端按 `Ctrl+C` 并等待退出；不要结束训练或 Gazebo。可先只读确认节点状态：

```bash
ros2 node list | rg '/joint_state_publisher' || true
# URDF 路径以训练启动时 /tmp/scout_mini_*.urdf 为准
ros2 run joint_state_publisher joint_state_publisher /tmp/scout_mini_*.urdf \
  --ros-args -p use_sim_time:=true
ros2 param set /robot_state_publisher ignore_timestamp true
```

下次 `train_td3.launch` 已在 [`gazebo_room.launch.py`](../ros2_ws/src/shared/scout_mini_pro_description/launch/gazebo_room.launch.py) 里改为 JSP `use_sim_time:=true` + RSP `ignore_timestamp:=true`。

#### 换图后 TF 丢了

换图会重置 `/clock`。若点云对不齐，**另起**一个 odom→TF（不要结束训练相关进程）：

```bash
ros2 run scout_mini_pro_description odom_to_tf_official --ros-args \
  -r __node:=odom_to_tf_watch -p use_sim_time:=true
```

阶段 A 默认**不做** AMCL/Nav2，仅预留话题与录制流程。

### ros2 bag 录图像

```bash
BAG_OUT="$SCOUT_WS_ROOT/3D/debug_logs/bags/scout_demo_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$BAG_OUT" \
  /camera/color/image_raw /camera/depth/image_raw /tf /tf_static /cmd_vel /odom
```

录制结束、核对用途后，把 `$BAG_OUT` 连同命令和结论移到 `archive/validation/`；不要长期留在活动调试目录。

### 轻量 GIF / 序列（stub）

```bash
CAM_SEQ="$SCOUT_WS_ROOT/3D/debug_logs/camera_seq"
ros2 run scout_offroad_3d record_camera_stub --ros-args \
  -p topic:=/camera/color/image_raw -p out_dir:="$CAM_SEQ" -p max_frames:=120 -p every_n:=2
# PNG 序列 → GIF（ImageMagick）
convert -delay 5 -loop 0 "$CAM_SEQ"/*.png "$SCOUT_WS_ROOT/3D/debug_logs/scout_demo.gif"
```

**Phase-2（未做）**：RViz 离屏截帧 / 配置自动化、深度伪彩与 RGB 并排 GIF。

---

<a id="ch5"></a>

## 5. 文件布局

```
ros2_ws/src/navigation_3d/scout_offroad_3d/
  config/td3_train.yaml            # viz_eval_* / landscape_ckpt_enable（俯视拍照已删）
  config/train_watch.rviz          # 训练中途 RViz（点云/车/目标/RGB/深度viz）
  scout_offroad_3d/
    plot_td3_metrics.py            # 科研曲线（§1）
    plot_td3_eval_viz.py           # 定轨轨迹 + 热力图（§2/§3；俯视绿图已删）
    plot_td3_loss_landscape.py     # Critic/Actor 损失景观（§8）
    plot_td3_actions.py            # 每 ep 平均 v/ω + 三维联合图
    record_camera_stub.py          # 轻量录帧（§4）
    train_goal_marker_node.py      # /offroad/train_goal → Marker
    depth_viz_bridge_node.py       # 32FC1 深度 → /camera/depth/image_viz（16UC1）
    train_td3_node.py              # 训练环钩子（viz 失败只 warn，不中断训练）
runs/<run_id>/
  metrics.csv  *.png               # §1 曲线（含 mean_v/mean_w）
  action_viz/                      # 指令速度统计（训毕自动）
    mean_v_vs_episode.png
    mean_w_vs_episode.png
    ep_v_w_3d.png
  outcome_xy.csv  viz_eval_state.yaml
  viz_eval/benchmark_pairs.yaml
  viz_eval/pair_gen_report.yaml    # 预生成降级说明（若有）
  viz_eval/ep0100/traj_<world>_p0N.png  …
  heatmaps/<world>/terminal.png
  heatmaps/<world>/goals.png
  loss_landscape/                  # §8（离线脚本生成）
    critic_td_loss_3d.png
    critic_td_loss_topo.png
    actor_objective_3d.png
    critic_td_loss_evol.gif        # 需 ckpts/
    ckpts/epXXXX/td3_*             # landscape_ckpt_enable: true 时写入
```

<a id="ch6"></a>

## 6. 注意

- 里程碑评测会**暂停训练**并可能换图；冒烟可 `viz_eval_milestones: [0.01]`、`viz_eval_pairs_per_world: 1`。
- 俯视 Gazebo 拍照 / 绿色示意图已删除（糊、且浪费换图时间）；轨迹图用透明底。
- 坐标系 world 米制。
- viz 异常**不中断训练**。

---

<a id="ch7"></a>

## 7. 长距评测出图（阶段 B/C）

长途栈的**短期验证**指标在结论确认后统一归入 **`archive/validation/long_nav/`**；正式实验则归入 **`archive/experiments/`** 并保留配置、指标和索引。**不要**把它们写进训练用的 `3D/runs/`，也不要长期堆在活动目录。

| 图 | 命令 | 产物 |
|----|------|------|
| 长距总览（成功率/碰撞/路径效率） | `plot_long_nav_metrics <run_dir>` | `long_nav_overview.png` |
| 粗路径 vs 实际 vs local_goal（示意/真数据） | 同上 | `path_overlay.png` |
| 合成 demo（无仿真） | `python3 -m scout_long_nav.plot_long_nav_metrics --demo` | 先在临时目录生成，确认后移入 `archive/validation/long_nav/` |

```bash
cd $SCOUT_WS_ROOT/ros2_ws
# 需已 install scout_long_nav，或：
PYTHONPATH=src/navigation_3d/scout_long_nav python3 -m scout_long_nav.plot_long_nav_metrics --demo
```

`metrics.csv` 建议列：`episode,success,collision,path_efficiency,path_length,straight_line`（空窗联调后再填真值）。算法与 launch 见 [`README4.md`](README4.md) §12 · 包 [`scout_long_nav`](../ros2_ws/src/navigation_3d/scout_long_nav/)。

---

<a id="ch8"></a>

## 8. TD3 损失 / 目标景观图

> **自动出图（推荐）**：训练正常结束或 Ctrl+C 进入 `finally` 且 `auto_plot: true` 时，`_plot_metrics` 会在曲线/热力图之后 **自动** 写 `loss_landscape/` 静图；若存在 `ckpts/ep*/` 再自动出 GIF。**不必再手敲** `python3 -m …plot_td3_loss_landscape`。  
> **样式**：全部科研图（曲线 / 热力 / 轨迹 / 景观）图内文字为 **英文**，字体优先 **Times New Roman**（本机无则 Liberation Serif / Nimbus Roman）。  
> 脚本：`scout_offroad_3d/plot_td3_loss_landscape.py`（亦可离线补跑）。

### 8.1 这几张图在讲什么

在参数空间里取当前权重 \(\theta\)，再取两个 filter-normalized 随机方向 \(\delta,\eta\)，在网格 \((\alpha,\beta)\) 上算目标值并画成曲面（经典 loss-landscape 切片）。

| 产物 | 竖轴 / 颜色 | 谷底 | 尖峰 / 陡脊 |
|------|-------------|------|-------------|
| `critic_td_loss_3d.png` | Critic TD / Bellman loss | Low TD error near current weights | Steep rise → TD error sensitive to that weight direction |
| `critic_td_loss_topo.png` | Same (top-down contours) | Same, planar view | Dense contours → steeper loss change |
| `actor_objective_3d.png` | Actor objective \(-\mathbb{E}[Q_1(s,\pi(s))]\) | Critic rates current policy highly nearby | Peaks → objective worsens under weight perturbation |
| `critic_td_loss_evol.gif` | Critic surface per ep checkpoint | How the valley evolves during training | Needs versioned weight snapshots (below) |

Axes \(\alpha,\beta\) are parameter-perturbation steps (not map meters).

### 8.2 离线补跑（可选）

下面以 **Record 9（150 维 / hidden 1024）仿真基线**为例；它是只读历史实验，不是实车权重。其他 run 必须使用与其权重匹配的 `config.yaml`。

```bash
cd $SCOUT_WS_ROOT/ros2_ws
PYTHONPATH=src/navigation_3d/scout_offroad_3d:$PYTHONPATH \
  python3 -m scout_offroad_3d.plot_td3_loss_landscape \
  $SCOUT_WS_ROOT/archive/experiments/td3/runs/run_20260907_120841
# GIF when ckpts exist:
#   ... plot_td3_loss_landscape <run_dir> --gif
```

Output: `run_*/loss_landscape/`.

### 8.3 GIF checkpoints

`save_freq` still overwrites the main `td3_*`. For GIF frames, keep:

```yaml
landscape_ckpt_enable: true
landscape_ckpt_every_n_ep: 20   # weights-only snapshot each N train EPs (not traj eval)
# → run_*/loss_landscape/ckpts/epXXXX/td3_{actor,critic}
```

Independent of `viz_eval` (no Gazebo / 12-pair freeze). Old runs without `ckpts/` only get static PNGs at shutdown.

### 8.4 Notes

- Probe batch: fixed-seed synthetic `(s,a,s',r)` from `config.yaml` / `params_backup.yaml`; optional `loss_landscape/landscape_probe.pt`.
- Surface is a 2D random slice (seed/direction change the shape). Complements trajectory / heatmap figures.
