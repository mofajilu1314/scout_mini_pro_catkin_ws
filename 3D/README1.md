# 3D · README1 — TD3 算法训练操作手册

> **版本与安全注记（2026-10-25）**：本文件保留完整的 TD3 公式、训练逻辑、参数表、命令、验证条件和历史实验脉络；不以摘要替代原文。先设定工作区根：`export SCOUT_WS_ROOT=/你的/catkin_ws_2/绝对路径`。当前 3D 代码在 `ros2_ws/src/navigation_3d/`，共享模型在 `ros2_ws/src/shared/`。服务器示例一律使用 `TRAIN_HOST`、`TRAIN_USER`、`SSH_PORT`、`RDP_PORT` 占位；真实地址、账号、端口、密码和实车连接资料绝不写进公开仓库。停止自己启动的 launch 用 `Ctrl+C`，仅用 `ss` 做只读检查，禁止以广泛杀进程命令“清场”。

前提：官方车 2D 已完成时，可参考[外置 2D 仓库 `scout-navigation-2d`](https://github.com/mofajilu1314/scout-navigation-2d)；本 3D 工作区不再包含或依赖本地 2D 源码。
**计划 / 阶段 / 块1·块2** 只看 [`README0.md`](README0.md)；**本文 = TD3 算法 + 奖励设计 + 怎么跑**。
踩坑：[`DEBUG_RECORDS.md`](DEBUG_RECORDS.md)；历次大训复盘：[`TRAIN_RECORDS.md`](TRAIN_RECORDS.md)；出图专册：[`README2.md`](README2.md)。

| 项 | 说明 |
|----|------|
| ROS2 包 | `ros2_ws/src/navigation_3d/scout_offroad_3d` |
| **Humble 快速命令** | 见 [`HUMBLE_QUICKSTART.md`](HUMBLE_QUICKSTART.md) 或本文 §5 |
| 阶段 A 一键训练 (Humble) | `bash 3D/scripts/humble_train_smoke.sh`（冒烟）· `ros2 launch scout_offroad_3d train_td3.launch.py`（长跑） |
| 阶段 A 推理 (Humble) | `ros2 launch scout_offroad_3d infer_td3.launch.py weights:=.../td3` |
| 默认超参 | **`ros2_ws/src/navigation_3d/scout_offroad_3d/config/td3_train.yaml`**（改 yaml 后重新 launch 生效） |
| 默认训练产物 | `$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/`（短验证 `smoke_*` 完成后应移入 `archive/validation/`，不堆在活动目录） |
| **阶段 A 不要** | SLAM、`map.yaml`、Arena、A\*、块2 整栈 |

---

## 目录

1. [算法基础与状态/动作维度](#ch1)（含 §1.6 TD3 公式）
2. [奖励评估函数、判据与训练逻辑](#ch2)（含 §2.1 全套 $A$/$P$ 公式）
3. [参数总表（与 td3_train.yaml 同步）](#ch3)
4. [调参顺序建议](#ch4)
5. [训练操作（冒烟 / 长训 / 续训）](#ch5)
6. [出图与科研可视化](#ch6)
7. [地图管理（打开 / 编辑 / 换图）](#ch7)
8. [服务器训练与权重回拉](#ch8)（含 SSH SSH_PORT / Remmina RDP）
9. [推理与真车部署](#ch9)
10. [常见故障](#ch10)
11. [选型拍板（块1/块2）](#ch11)
12. [还差什么（诚实清单）](#ch12)

---

<a id="ch1"></a>

## 1. 算法基础与状态/动作维度

### 1.1 阶段定位：3D 越野 vs 2D

| 项 | **外置 2D 阶段（`scout-navigation-2d`）** | **3D 越野阶段 A（本文）** |
|----|------------------------------|---------------------------|
| 感知 | 单线 `/scan`（LaserScan） | **16 线** `±15°` 垂直 FOV，360° 水平，`/lidar/points`（Gazebo `block_laser` → `sensor_msgs/PointCloud`；可选 bridge → `/lidar/points2` PointCloud2） |
| TD3 状态 | 24 扇区 + ρθ + 上拍 = 28 维 | **150 维**（Record 9）：4 仰角 × 36 扇区 + ρθvw + IMU roll/pitch；**另发** `/offroad/elevation_features`（4×12=48）专供坡/墙判别 |
| 仿真世界 | 室内 room | `scout_offroad_3d/worlds/*.world` 越野/坡道/障碍（**4 张 bumpy heightmap**） |
| 碰撞机制 | 前方扇区 + 速度 + 防抖（记录 12） | 记录 12 + **坡/墙分层**（记录 14/19）；真墙 `[collision] wall confirmed`；**翻车** IMU rollover 独立于 stuck（记录 19） |

**传感器配置**（`scout_description/urdf/scout_mini.gazebo` + yaml）：`laser_link` 上 ray 传感器；**训练默认** `lidar_v_samples=16` × `lidar_h_samples=360`（±15° 垂直 FOV）；`min_range=0.18`；插件 `libgazebo_ros_ray_sensor.so` → `/lidar/points`。评测/对齐旧分布时可改更高水平采样（如 720），RTF 会下降。

**强制边界（先读再开终端）**：

1. **阶段 A = 块1**：Gazebo world + **16 线 3D 点云** `/lidar/points` → **4 仰角 × 36 扇区 + IMU roll/pitch** → TD3 → `/cmd_vel`（**不是** 2D `/scan` 默认链）。
2. **不要**预先建图 / 加载 `map.yaml` / 开 `nav_all_in_one*` / Arena / A\*。
3. **块2**（建图/SLAM + Arena 子目标 + 粗全局）在阶段 B/C，**不阻塞**现在训 TD3。
4. 训练用 **官方车** + `gazebo_room_official.launch`（已由 `train_td3.launch` / `infer_td3.launch` include）。**训练默认**在启动时从包内 `worlds/*.world` **随机抽第一张**；若 yaml 里 `world_switch_interval > 0`（默认 **20000**），之后每 N 个训练 timestep 再换一张（重启 gzserver；**不清** replay）。`world:=` 只定第一张；`interval=0` 则整场锁死第一张。

### 1.2 状态 / 动作维度（与 `ros_env.py` / `td3_train.yaml` 一致）

| | 符号 | 维度 / 范围 | 来源 |
|--|------|-------------|------|
| **状态** | $s$ | **150** | `n_elevation_bins_td3(4) * n_sectors(36) + 4 + 2`（ρθvw + roll/π,pitch/π） |
| TD3 扇区距离 | $d_{b,i}$ | **4** 仰角桶 × 36 全周扇区 min range，再 `/ max_range` → $[0,1]$ | `/offroad/td3_range_features`（**144** 维） |
| 水平投影（**不进 state**） | 36 | 近水平带 min range | `/offroad/range_features`（碰撞水平扫描） |
| 多层仰角（**不进 state**） | 4×12 | 每仰角桶 × 前方扇区 min range（m） | `/offroad/elevation_features`（碰撞/坡墙判别） |
| 目标极坐标 | $\rho,\theta$ | $\rho/10$ 截断到 1；$\theta/\pi$ | Gazebo 真值或 `/odom` 相对随机目标 |
| 上拍动作 | $v_{\mathrm{prev}},\omega_{\mathrm{prev}}$ | 分别 `/ max_v`、`/ max_w` | 环境缓存 |
| IMU 姿态（**进 state**） | $\mathrm{roll},\mathrm{pitch}$ | 各 $/ \pi$ → $[-1,1]$；`use_imu_in_state: true` | `/imu/data`（静默时 model_state fallback） |
| **动作** | $a=(v,\omega)$ | **2**；对称倒车 $v\in[-max_v,max_v]$（`allow_reverse: true`），$\omega\in[-max_w,max_w]$（yaml 默认 `max_v=2.3`、`max_w=1.8`） | → `/cmd_vel`（`linear.x` / `angular.z`） |

> **`(v,ω)` 如何变成四轮转速**（Gazebo `diff_drive` / 真机 `scout_base`）：见 [`README3.md` §9](README3.md#ch9)。网络**不**直控四个电机。

**多层仰角 — TD3（感觉）**（`/offroad/td3_range_features`）：**4** 仰角桶 × **360° 全周 36 扇区** min range；`td3_elevation_bin_edges_deg = [-15,-7.5,0,7.5,15]`，四等分 ±15° FOV（中点 **−11.25° / −3.75° / +3.75° / +11.25°**）。最后一桶闭区间，+15° 射线进顶层（不用 15.01 补丁）。

**多层仰角 — 判别（规则）**（`/offroad/elevation_features`）：同一组边界 `[-15,-7.5,0,7.5,15]` 分 **4** 桶；每桶在 **前方 ±60°** 内再分 12 扇区取 min range。仿真射线数见 yaml `lidar_v/h_samples`（训练默认 **16×360**）。

**兼容话题**：`/offroad/range_features`（36 维水平投影）仍发布，供碰撞水平扫描与旧脚本对照；TD3 训练/推理默认改订阅 `/offroad/td3_range_features`。

**Self-filter**（`range_features_node.py`）：`min_range`；TD3 水平带 `elevation_min/max_deg`（默认 ±10°）；可选 **body 包围盒**（`filter_body_box`）忽略打到白箱/底盘的近距离点。

**改 TD3 网络大小**：改 yaml `hidden_dim`（默认 **1024**）后 **必须重新训练**；旧 `td3_actor` 权重不兼容。
**改 state / 扇区数**：`td3_train.yaml` 的 `n_sectors`、`n_elevation_bins_td3`、`use_imu_in_state`、`max_range` + `range_features.launch`；`ros_env.py` → `state_dim = n_elevation_bins_td3 * n_sectors + 4 (+2 if IMU)`（默认 **150**）。改维数或 hidden **必须重新训练**；旧权重（含 Record 8 的 112/768）不兼容。

**TD3 能否上越野？** 客观一句：**仿真里可以学局部避障 + 朝随机目标走；150 维含 4 仰角层 + IMU 倾角，仍无全局图；真车复杂越野需 sim-to-real 与块2 分层规划，阶段 A 只是可训骨架，不是量产越野导航。**

### 1.3 训练环：reset → step → reward → replay → update

与 Fujimoto 开源一致的外环，ROS 里由 `train_td3_node.py` + `ScoutGazeboEnv` 实现：

```text
state = env.reset()                    # 随机起点/目标；set_model_state；发 /offroad/train_goal
for t in 1..max_timesteps:
  if t < start_timesteps:  a ~ Uniform([-max_v, max_v] × [-max_w, max_w])   # 纯随机探索（含倒车）
  else:                    a = π(s) + noise # 策略 + 探索噪声（σ 由成功率退火）
  s', r, done, info = env.step(a)           # 发 /cmd_vel；sleep control_dt；算 reward
  replay.add(s, a, s', r, done_bool)        # 超时 done_bool=0（可 bootstrap）
  if t >= start_timesteps:  policy.train()  # 批采样；双 Q；延迟 actor；目标平滑
  if done:  log；退火更新 σ/lr；state = env.reset()
  每 save_freq 步：save td3_actor / td3_critic
```

对应文件：`scout_offroad_3d/td3/TD3.py`（网络与 update）、`replay_buffer.py`、`ros_env.py`（reset/step）、`train_td3_node.py`（外环）。

### 1.4 对照：sfujim TD3 / AUV DRL / Arena → 我们怎么改成 Scout 差速

| 来源 | 他们怎么操作 / 状态怎么喂 | 本项目改编 |
|------|---------------------------|------------|
| **[sfujim/TD3](https://github.com/sfujim/TD3)** | Gym 连续控制：`env.reset/step`；`state`/`action` 为向量；Actor→动作；Twin Critic；`policy_freq` 延迟更新；目标动作加噪声再 clip | **原样移植**网络与 replay（**MLP 全连接**，不是 CNN）；`max_action` 改为向量 `(max_v, max_w)`；权重前缀 `td3`（无 `.pth` 后缀）。**不**拷贝任何预训练 AUV 权重 |
| **[DRL-AUV-navigation](https://github.com/lanyangyang-gogogo/DRL-AUV-navigation)** | ROS+Gazebo episode：置位姿 → 读传感器状态 → 发速度指令 → reward/done；AUV 常为 3D 速度/姿态 | **只借环结构**；载体改地面 skid-steer；指令改 `/cmd_vel`；状态改扇区+$(\rho,\theta)$+上拍；模型名 `scout_mini`；**reward / v_max / state 维是 Scout 在 yaml 里设的，不是 AUV 原样搬来** |
| **[Arena-Rosnav](https://github.com/ignc-research/arena-rosnav)** ([arXiv:2104.03616](https://arxiv.org/abs/2104.03616)) | 全局规划（如 A\*）→ **中间规划器**按空间前瞻取 **local sub-goal** → DRL 局部跟点子目标 | **阶段 A 未接**；训练目标是环境内随机点（`/offroad/train_goal`）。**阶段 B** 才发 `/offroad/local_goal` 喂给同一 TD3 的 $(\rho,\theta)$ |

阶段 A 等价于 Arena 里「train_mode 时整段终点当唯一子目标」的简化版；长距离分层要等块2。

### 1.5 可摘论文原句（写综述/论文用 · 标明出处）

**Fujimoto et al., TD3（ICML 2018）** — [arXiv:1802.09477](https://arxiv.org/abs/1802.09477)：

> In value-based reinforcement learning methods such as deep Q-learning, function approximation errors are known to lead to overestimated value estimates and suboptimal policies. We show that this problem persists in an actor-critic setting and propose novel mechanisms to minimize its effects on both the actor and the critic. Our algorithm builds on Double Q-learning, by taking the minimum value between a pair of critics to limit overestimation. We draw the connection between target networks and overestimation bias, and suggest delaying policy updates to reduce per-update error and further improve performance.

本仓库对应实现点：twin critic 取 min、delayed policy update（`policy_freq: 2`）、target policy smoothing（`policy_noise` / `noise_clip`）。

**Kästner et al., Arena-Rosnav（2021）** — [arXiv:2104.03616](https://arxiv.org/abs/2104.03616) 摘要级：

> … deep reinforcement learning approaches are not suitable for long-range navigation due to their proneness to local minima and lack of long-term memory … we propose a navigation system incorporating deep-reinforcement-learning-based local planners into conventional navigation stacks for long-range navigation.

方法侧（中间子目标，摘要/正文一致表述）：用 A\* 等得全局路径后，按机器人位置与 look-ahead 距离取 **moving local sub-goal**，再交给 DRL 局部规划器——这正是本项目块2 → `/offroad/local_goal` → 块1 TD3 的模板。**现在阶段 A 没有该中间层。**

更多摘录见 [`article/record.md`](../article/record.md) 论文2 / 论文4。

### 1.6 TD3 算法公式（与 `TD3.py` / yaml 同步）

记状态 $s\in\mathbb{R}^{150}$，动作 $a=(v,\omega)\in\mathbb{R}^{2}$，动作界

$$
a_{\max} = (v_{\max},\omega_{\max}) = (2.3,\ 1.8),\qquad
a_{\mathrm{clip}}(x)=\mathrm{clip}\bigl(x,-a_{\max},a_{\max}\bigr).
$$

**Actor**（两层 ReLU MLP，隐宽 $h=\texttt{hidden\_dim}=1024$）：

$$
\pi_\phi(s) = a_{\max}\odot\tanh\bigl(W_3\,\mathrm{ReLU}(W_2\,\mathrm{ReLU}(W_1 s))\bigr).
$$

**Twin Critic** $Q_{\theta_1},Q_{\theta_2}$：各自输入 $(s,a)$ 拼接后同宽 MLP → 标量。

**在线探索动作**（`train_td3_node`，$t\ge\texttt{start\_timesteps}$）：

$$
a_t = a_{\mathrm{clip}}\Bigl(\pi_\phi(s_t) + \varepsilon_t\Bigr),\quad
\varepsilon_t\sim\mathcal{N}\bigl(0,\ (\sigma_{\mathrm{expl}}\,a_{\max})^2\bigr).
$$

$t<\texttt{start\_timesteps}=5000$ 时改为均匀随机：$a_t\sim\mathrm{Unif}([-a_{\max},a_{\max}])$。

**Critic 目标**（目标策略平滑 + clipped double Q；`policy_noise`$\sigma_p=0.2$，`noise_clip`$c=0.5$，`discount`$\gamma=0.99$）：

$$
\tilde a' = a_{\mathrm{clip}}\Bigl(\pi_{\phi'}(s') + \mathrm{clip}(\mathcal{N}(0,(\sigma_p a_{\max})^2),\,-c\,a_{\max},\,c\,a_{\max})\Bigr),
$$

$$
y = r + d\,\gamma\,\min_{i=1,2} Q_{\theta_i'}(s',\tilde a'),
$$

其中 $d=\texttt{not\_done}$：success / stuck / rollover / 第 3 次墙撞 → $d=0$；**timeout → $d=1$**（Fujimoto 式可 bootstrap）。

**损失**（批大小 $B=\texttt{batch\_size}=1024$）：

$$
L_{\mathrm{critic}} = \sum_{i=1}^{2}\mathbb{E}_{(s,a,r,s',d)\sim\mathcal{D}}\bigl[(Q_{\theta_i}(s,a)-y)^2\bigr],
$$

$$
L_{\mathrm{actor}} = -\mathbb{E}_{s\sim\mathcal{D}}\bigl[Q_{\theta_1}(s,\pi_\phi(s))\bigr]
\quad\text{（每 }\texttt{policy\_freq}=2\text{ 次 critic 更新一次）}.
$$

**目标网络软更新**（`tau`$\tau=0.006$）：

$$
\theta' \leftarrow \tau\theta+(1-\tau)\theta',\qquad
\phi' \leftarrow \tau\phi+(1-\tau)\phi'.
$$

实现：[`scout_offroad_3d/td3/TD3.py`](../ros2_ws/src/navigation_3d/scout_offroad_3d/scout_offroad_3d/td3/TD3.py)。详细奖励见 §2.1；成功率退火见 §2.1.5。

---

<a id="ch2"></a>

## 2. 奖励评估函数、判据与训练逻辑

### 2.1 奖励评估函数（Reward / Return，论文用公式）

数值以磁盘 [`td3_train.yaml`](../ros2_ws/src/navigation_3d/scout_offroad_3d/config/td3_train.yaml) 为准；实现 [`ros_env.py`](../ros2_ws/src/navigation_3d/scout_offroad_3d/scout_offroad_3d/td3/ros_env.py) 的 `step()`。

**时间步约定（读按步奖必先看）**：

$$
\Delta t = \texttt{control\_dt} = \frac{1}{10}\,\mathrm{s}.
$$

记墙钟时间长度为 $T$ 秒。若某「按步」项在这 $T$ 秒内**每一步都触发**，则步数与累计分为

$$
N = \frac{T}{\Delta t} = 10\,T,\qquad
\sum_{\mathrm{steps}} = (\text{单步值})\times 10\,T.
$$

例：前进 $\Delta\rho=1\,\mathrm{m}$ 平地 progress $=+12$；同距爬坡且 $s_{\mathrm{climb}}=0.25$ 时额外 $A_{\mathrm{climb}}=0.25\times 12\times 1=+3.0$。
**按米**项（progress、爬/下坡）与**终端一次性**项（arrive / stuck / rollover / timeout / 墙撞事件）**不**乘 $10T$；仅「每步常数」类（step / low-effort / high-effort 等）才用 $\times 10T$。

记本步目标距离 $\rho_t$，上步 $\rho_{t-1}$，

$$
\Delta\rho_t=\rho_{t-1}-\rho_t
\quad(\Delta\rho_t>0\text{ 表示靠近目标}).
$$

#### 2.1.1 单步总奖励

$$
r_t = A_t + P_t.
$$

- $A_t$：**Award**（到达、进展、高努力、探索、爬/下坡、开局）。
- $P_t$：**Penalty**（墙撞、stuck、翻车、超时、低努力、原地转、时间、坡道轻罚）。

多数 $A$/$P$ 子项在未触发时为 $0$；终端项只在该步 `done` 时非零。

#### 2.1.2 Award 项（公式全集）

$$
A_t = A_{\mathrm{start}}+A_{\mathrm{progress}}+A_{\mathrm{high}}+A_{\mathrm{explore}}+A_{\mathrm{climb}}+A_{\mathrm{descend}}+A_{\mathrm{arrive}}.
$$

**（1）开局**（一次性，仅每局第 $0$ 步；**不**乘 $10T$）：

$$
A_{\mathrm{start}}=
\begin{cases}
4.5 & \text{本局第 0 步}\\
0 & \text{otherwise}
\end{cases}
$$

**（2）目标进展（按米，可正可负；与 $\Delta t$ 无关）**：

$$
A_{\mathrm{progress}} = 12.0\cdot\Delta\rho_t.
$$

靠近 $1\,\mathrm{m}$ → $+12$；远离 $1\,\mathrm{m}$ → $-12$。**不是** $\max(0,\Delta\rho)$，也**不**写成 $10T$。

**（3）高努力（按步；持续 $T$ 秒则 $\times 10T$）**：

$$
E_t = |v_t| + \alpha \, |\omega_t|,\qquad \alpha=0.8
$$

$$
A_{\mathrm{high}} =
\begin{cases}
\beta E_t & \text{if } E_t \ge e_{\mathrm{hi}} \\
0 & \text{otherwise}
\end{cases}
,\qquad \beta\approx 1/3\ (0.33),\quad e_{\mathrm{hi}}=0.7
$$

即单步：$E_t\ge 0.7$ 时 $A_{\mathrm{high}}=\beta E_t$，否则 $0$。  
若连续 $T$ 秒都满足高努力（$\bar E$ 为该时段平均努力）：

$$
\sum A_{\mathrm{high}} \approx (\beta\,\bar E)\times 10\,T.
$$

（yaml `r_forward_scale`$=0$ 时不再叠加旧 $v-|\omega|$。）

**（4）探索（按首次进格；触发当步给一次，不是每秒刷）**：栅格边长 $g=0.5\,\mathrm{m}$，cell $=(\lfloor x/g\rfloor,\lfloor y/g\rfloor)$。本局**第一次**踏入该格：

$$
A_{\mathrm{explore}}=
\begin{cases}
0.30 & \text{first visit of cell}\\
0 & \text{otherwise}
\end{cases}
$$

复访为 $0$；**不是**持续 $T$ 秒就 $\times 10T$。若 `explore_cardiff_scale=true`（现网 false）：再除以 $\max(\rho_t,1.0)$。

**（5）爬坡 / 下坡（按进度米；Record 9）**：

令 $\psi_t=\mathrm{pitch}_t$（度），$\psi_{\mathrm{lo}}=2$，$\psi_{\mathrm{hi}}=35$；$r_{\mathrm{progress}}=12$；$s_{\mathrm{climb}}=s_{\mathrm{climb}}(s_W)$ 由成功率余弦退火（§2.1.5）。

$$
A_{\mathrm{climb}}=
\begin{cases}
s_{\mathrm{climb}}\cdot r_{\mathrm{progress}}\cdot\Delta\rho_t
& \Delta\rho_t>0\ \land\ \psi_{\mathrm{lo}}\le\psi_t\le\psi_{\mathrm{hi}}\\
0 & \text{otherwise}
\end{cases}
$$

$$
A_{\mathrm{descend}}=
\begin{cases}
\dfrac{1}{3}\,s_{\mathrm{climb}}\cdot r_{\mathrm{progress}}\cdot\Delta\rho_t
& \Delta\rho_t>0\ \land\ -\psi_{\mathrm{hi}}\le\psi_t\le-\psi_{\mathrm{lo}}\\
0 & \text{otherwise}
\end{cases}
$$

爬/下坡互斥：上坡走 \(A_{\mathrm{climb}}\)，下坡走 \(A_{\mathrm{descend}}\)（同一 \(s_{\mathrm{climb}}\cdot r_{\mathrm{progress}}\cdot\Delta\rho\) 再 ÷3）。\(|\psi|>35^\circ\) 时两项均为 \(0\)。  
yaml 中 `r_slope_climb`/`r_slope_descend` 为 **legacy 常数**，主路径已不用。

**（6）到达（终端一次性；**不**乘 $10T$）**：

$$
A_{\mathrm{arrive}}=
\begin{cases}
700 & \rho_t < 0.55\\
0 & \text{otherwise}
\end{cases}
$$

#### 2.1.3 Penalty 项（公式全集）

$$
P_t = P_{\mathrm{step}}+P_{\mathrm{spin}}+P_{\mathrm{low}}+P_{\mathrm{slope}}+P_{\mathrm{collision}}+P_{\mathrm{stuck}}+P_{\mathrm{rollover}}+P_{\mathrm{timeout}}.
$$

**（1）时间（按步；整局时长 $T$ 秒则 $\times 10T$）**：

$$
P_{\mathrm{step}} = -0.05 \quad\text{（每步）}.
$$

整局跑满墙钟 $T$ 秒（约 $10T$ 步）时：

$$
\sum P_{\mathrm{step}} = (-0.05)\times 10\,T = -0.5\,T.
$$

**（2）原地转（按步；持续则 $\times 10T$）**：

$$
P_{\mathrm{spin}}=
\begin{cases}
-0.2\cdot\bigl(|\omega_t|-1.4\bigr)
& |v_t|<0.1\ \land\ |\omega_t|>1.4\\
0 & \text{otherwise}
\end{cases}
$$

持续原地转 $T$ 秒：$\sum P_{\mathrm{spin}}\approx \overline{P_{\mathrm{spin}}}\times 10\,T$。

**（3）低努力（按步；窗条件满足后的每一步都罚；持续则 $\times 10T$）**：  
维护最近 $1.5\,\mathrm{s}$ 内 $E$ 历史；窗长 $\ge 0.85\times 1.5$ 且窗内全部 $E<0.25$ 时：

$$
P_{\mathrm{low}} = -0.72 \quad\text{（该步）}.
$$

此后若连续磨蹭 $T$ 秒：$\sum P_{\mathrm{low}} = (-0.72)\times 10\,T = -7.2\,T$。

**（4）坡道嫌疑轻罚（按步；持续则 $\times 10T$）**：判为 slope、非墙时

$$
P_{\mathrm{slope}} = -0.02 \quad\text{（该步）},\qquad
\sum P_{\mathrm{slope}} = (-0.02)\times 10\,T = -0.2\,T.
$$

（**不计**碰撞次数。）

**（5）墙撞（按事件次数；**不**乘 $10T$）**：第 $k$ 次墙事件

$$
P_{\mathrm{collision}}=c_k,\qquad (c_1,c_2,c_3)=(-12,-30,-60).
$$

$k=3$ → `done`。

**（6）受困 stuck（终端一次性；**不**乘 $10T$）**：路径窗 $5.6\,\mathrm{s}$，跨度 $\ge 0.85\times 5.6$ 且位移 $<0.25\,\mathrm{m}$，且 $\rho_t\ge 0.7$：

$$
P_{\mathrm{stuck}} = -160,\quad \texttt{done}.
$$

$\rho_t<0.7$：**不判 stuck**；磨到步数上限走 timeout。

**（7）翻车（终端一次性；**不**乘 $10T$）**：令 $\varphi=|\mathrm{roll}|^\circ$，$\psi=|\mathrm{pitch}|^\circ$。局已过 $2\,\mathrm{s}$ 且

$$
\varphi\ge 60\ \lor\ \psi\ge 60\ \lor\ (\varphi+\psi)\ge 80
$$

→ $P_{\mathrm{rollover}}=-160$，`done`。前 $2\,\mathrm{s}$ 内翻车 → spawn grace abort（重采样，无此项）。

**（8）超时（终端一次性；**不**乘 $10T$）**：

$$
P_{\mathrm{timeout}}=
\begin{cases}
-120 & \text{step}\ge 600\ \land\ \text{非其它终端}\\
0 & \text{otherwise}
\end{cases}
$$

（满 $600$ 步 $=60\,\mathrm{s}$ 墙钟上限。）

#### 2.1.4 Episode Return 与 TD 备份

记一局共 $N$ 个 env 步（$N\le 600$），折扣回报

$$
R=\sum_{n=0}^{N-1}\gamma^{n} r_n,\qquad \gamma=0.99.
$$

Critic 备份 $y$ 见 **§1.6**（含 timeout bootstrap）。单局经验写入 replay 后按批更新，**不**在 env 内显式算 $R$。

#### 2.1.5 成功率定义与余弦退火

**成功率**（与 `metrics.csv` 一致）：窗口 $W=50$，

$$
s_W=\frac{1}{W}\sum_{i=1}^{W}\mathbf{1}\{\texttt{outcome}_i=\mathrm{success}\},\qquad
s_{\mathrm{cum}}=\frac{N_{\mathrm{succ}}}{N_{\mathrm{ep}}}.
$$

退火仅当开启且 episode $\ge 100$。令 $s_{\mathrm{lo}}=0.10$，对探索噪声/LR 用 $s_{\mathrm{hi}}=0.90$，

$$
u=\mathrm{clip}\!\left(\frac{s_W-s_{\mathrm{lo}}}{s_{\mathrm{hi}}-s_{\mathrm{lo}}},\,0,\,1\right).
$$

对 $x\in\{\sigma_{\mathrm{expl}},\,\mathrm{lr}_{\mathrm{actor}},\,\mathrm{lr}_{\mathrm{critic}}\}$：

$$
x(s_W)=x_{\min}+\frac{x_{\max}-x_{\min}}{2}\bigl(1+\cos(\pi u)\bigr).
$$

现网：$(\sigma_{\max},\sigma_{\min})=(0.1,0.001)$，$(\mathrm{lr}_{\max},\mathrm{lr}_{\min})=(3\times10^{-4},\,1\times10^{-4})$。

**爬坡 scale 余弦退火（Record 9，独立通道）**：不改 $r_{\mathrm{progress}}$ 等固定系数；每局把

$$
s_{\mathrm{climb}}(s_W)=s_{\min}+\frac{s_{\max}-s_{\min}}{2}\bigl(1+\cos(\pi u_{\mathrm{climb}})\bigr)
$$

写入 `env.slope_climb_scale`。其中 $s_{\max}=0.25$，$s_{\min}=0.0005$，

$$
u_{\mathrm{climb}}=\mathrm{clip}\!\left(\frac{s_W-s_{\mathrm{lo}}}{s_{\mathrm{hi}}^{\mathrm{climb}}-s_{\mathrm{lo}}},\,0,\,1\right),\qquad
s_{\mathrm{hi}}^{\mathrm{climb}}=\texttt{slope\_climb\_anneal\_success\_hi}=0.80
$$

（注意 climb 的 hi 是 **0.80**，不是 expl 的 0.90。）ep&lt;100 或关退火时 $s_{\mathrm{climb}}\equiv 0.25$。  
日志可见 `[anneal] ... climb_scale=...`。

#### 2.1.6 量纲速查（现网 Record 9）

| 项 | 计量 | 单步 / 单次值 | 备注 |
|----|------|---------------|------|
| $A_{\mathrm{start}}$ | **/局** | $4.5$ | 仅第 0 步 |
| $A_{\mathrm{progress}}$ | **/米** | $12\cdot\Delta\rho$ | 主头 |
| $A_{\mathrm{climb}}$ | **/米·进度** | $s_{\mathrm{climb}}\cdot 12\cdot\Delta\rho$ | $s_{\mathrm{climb}}\in[0.0005,0.25]$ |
| $A_{\mathrm{descend}}$ | **/米·进度** | $\tfrac13 s_{\mathrm{climb}}\cdot 12\cdot\Delta\rho$ | 与 climb 互斥 |
| $A_{\mathrm{high}}$ | **/步** | $\beta E$，$\beta\approx 1/3$ | $E\ge 0.7$；持续则 $\times 10T$ |
| $P_{\mathrm{step}}$ | **/步** | $-0.05$ | $\times 10T=-0.5\,T$ |
| $P_{\mathrm{low}}$ | **/步** | $-0.72$ | $\times 10T=-7.2\,T$ |
| $P_{\mathrm{slope}}$ | **/步** | $-0.02$ | 轻量误分类 |
| $A_{\mathrm{explore}}$ | **/新格子** | $0.30$ | 不按 $10T$ 刷 |
| $A_{\mathrm{arrive}}$ | **/次** | $700$ | 终端，$\rho<0.55$ |
| $P_{\mathrm{stuck}}$ / $P_{\mathrm{rollover}}$ | **/次** | $-160$ / $-160$ | 终端；$\rho<0.7$ 不判 stuck |
| $P_{\mathrm{timeout}}$ | **/次** | $-120$ | 满 600 步 |

记忆口诀：$\Delta t=\frac{1}{10}\,\mathrm{s}$ ⇒ **按步分 $\times 10T$ = 持续 $T$ 秒总分**；爬坡按 **Δρ×scale**，不走旧「每步 0.15」。

#### 2.1.7 与论文的对应关系

- $A_t$ / $P_t$：reward shaping vs unsafe-behavior penalties。
- $E_t$：effort-based shaping（替代旧 $v-|\omega|$）。
- $s_W$ 余弦退火：adaptive exploration / mild LR schedule。
- Twin Q + delayed actor + target smoothing：TD3 三大件（§1.5–1.6）。

### 2.2 判据细节：碰撞 / 受困 / 翻车 / 超时

实现：`ros_env.py` 的 `step()` → `_classify_obstacle()` + `_update_collision()` / `_update_stuck()`；参数：`td3_train.yaml` 中 `terrain_*`、`collision_*`、`stuck_*`。踩坑：记录 12（软碰撞）、**记录 14**（坡/墙）。

#### 2.2.1 爬坡 vs 撞墙（多层 + IMU）

| 信号 | 撞墙倾向 | 爬坡倾向 |
|------|----------|----------|
| 多层 front-min **全低** | ✓ | |
| 仅低/中层低，**顶层**（[+7.5°, +15°]）远 | | ✓ |
| `v_forward` 持续低 + 多层近 | ✓ | |
| 近距仅持续 `< slope_max_frames` 帧 | | ✓ |
| IMU **pitch 上升**（可选） | | ✓ |
| 车仍能前进（`v_fwd > wall_v_forward_max`） | | ✓ |

- **坡**：`[terrain] slope suspected ...`；`r_slope_penalty`（默认 -0.02）；**不** `collision_event++`
- **墙**：`[collision] wall confirmed event #k ...`；按 `collision_penalty_list` 递增罚分

话题：`/offroad/elevation_features` layout = `elevation_bin(4) × front_sector(12)`。360° 全周只用于 TD3 的 36 扇区；多层特征 **仅前方 ±** `front_half_fov_deg`（默认 60°，launch 可调）。

#### 2.2.2 三种结束与惩罚（对照表）

| 类型 | 是否 `done` | 默认惩罚 | 判定要点 | 日志关键字 |
|------|-------------|----------|----------|------------|
| **到达（success）** | 是 | `+r_arrive`（**700**） | $\rho < goal\_tol$（0.55 m） | `END success` |
| **翻车（rollover）** | 是 | `r_rollover`（**-160**） | IMU \|roll\|/\|pitch\| ≥ **60°** 或合角 ≥ **80°**，且 ep 时长 **> 2 s** | `[rollover] ... → done` |
| **spawn grace 翻车** | 否（作废重开） | 无 | ep 时长 **≤ 2 s** 且姿态超阈值 → **reset 重采样**，不计入 episode/metrics | `[rollover] spawn grace flip → abort episode (resample)` |
| **受困（stuck）** | 是 | `r_stuck`（**-160**） | 窗 **5.6 s** 位移 $<$ 0.25 m，且 $\rho\ge 0.7$；$\rho<0.7$ **不判 stuck**（磨到 600 步 → timeout） | `[stuck] ... → done` |
| **超时（timeout）** | 是 | `r_timeout`（**-120**） | `step ≥ max_ep_steps`（**600**） | `END timeout` |
| **碰撞（wall）** | **第 3 次是** | **-12 / -30 / -60** | 多层判别为 **墙** + 防抖；见 2.2.1 | `[collision] wall confirmed` |
| **高努力** | 否 | $\beta E$（$E\ge e_{\mathrm{hi}}$）；持续 $T$ 秒 $\approx(\beta\bar E)\times 10T$ | α=0.8，β≈**1/3**，$e_{\mathrm{hi}}=0.7$ | `[reward] ... high ...` |
| **原地转** | 否 | $-r_{\mathrm{spin}}(|\omega|-w_{\mathrm{spin}})$；持续则 $\times 10T$ | 近零 v + 很高 \|ω\| | `[reward] spin ...` |
| **低努力** | 否 | $-0.72$/步；持续 $T$ 秒 $= -7.2\,T$ | 持续低 $E$（默认 **1.5 s**） | `[reward] low_effort ...` |
| **爬坡 / 下坡** | 否 | $s_{\mathrm{climb}}\cdot 12\cdot\Delta\rho$（下坡 /3）；$s_{\mathrm{climb}}\in[0.0005,0.25]$ | $\Delta\rho>0$ 且 pitch∈[2°,35°] | `[reward] ...` |
| **坡道误类（wall 轻罚）** | 否 | `r_slope_penalty`（**-0.02**） | 多层/短时/ pitch / 仍前进 | `[terrain] slope suspected` |

**与旧 hard collision 对比**（记录 12 前）：

| | 旧（hard） | 新（当前默认） |
|--|-----------|----------------|
| 扇区范围 | 全周 36 扇区任一近距 | **仅车头 ±** `collision_sector_half_fov_deg`（**120°**） |
| 速度 | 无 | 朝向障碍 **分速度** `v_app > collision_v_min`（**0.07** m/s） |
| 防抖 | 单帧 | 连续 **`collision_frames`**（**2**）步候选才记 1 次事件 |
| 结束局 | **碰撞即 `done`** | **默认不 reset**；第 **3** 次墙撞 `done`；继续训练/探索 |
| 罚分 | 固定 **-25** | 第 k 次 **-12 / -30 / -60**（`collision_penalty_list`，已放宽） |
| 空转/骑墙 | 无专门终止 | **`stuck`** 窗口位移过小 → `done` + **-160** |

贴墙慢移或已停住：`v_app ≤ collision_v_min` → **不算碰撞**（避免「抵墙即死局」）。

#### 2.2.3 碰撞判定逻辑（前方扇区 + 速度 + 多层坡/墙 + 连续帧）

每步 `step()` 在 body 系读 `(vx, vy)`，扫描扇区 `i = 0…n_sectors-1`：

1. **前方扇区**：`|bearing(i)| ≤ collision_sector_half_fov_deg`（相对车头 +x 前向，+y 左）。
2. **距离**：该扇区 `d < collision_dist`（**0.32** m）。
3. **朝向速度**：`v_app > collision_v_min`。
4. 取最近候选扇区 → `_classify_obstacle()` 读 `/offroad/elevation_features` 各层 front-min + 可选 `/imu/data` pitch。
5. **墙判**：≥ `wall_layers_required`（默认 2）层近距，或 `min_d < wall_force_min_dist`（**0.30 m**）→ **墙**；否则坡模式 → 轻罚。
6. **墙** → 连续 **`collision_frames`**（**2**）步后记第 k 次墙撞事件。

**碰撞事件**：同一局内可多次 `[collision] event #1/#2/…`；**第 3 次**（`max_collision_events: 3`）→ `done` + `outcome=collision`；`reset()` 清空 `_collision_event_count`。

旧公式（`collision_penalty_double: true`）仍可用：$\text{penalty}_k = r\_collision\_base \cdot 2^{k-1}$。

#### 2.2.4 受困（stuck）与超时

- **stuck**：维护 `(t, x, y)` 历史，窗口长度 ≥ `0.85 × stuck_time_sec`（默认 **5.6 s**）后，若位移 `< stuck_dist_threshold` → `done=True`，加 `r_stuck`（**-160**），打 `[stuck]`。用于 **原地空转、骑台阶、顶墙打滑** 等「还在动但走不出去」。
- **近终点豁免**：当 `goal_tol ≤ ρ < stuck_near_goal_rho`（默认 **0.7 m**）时**不判 stuck**（清空位移窗），以便倒车入库微动；若步数耗尽仍未到达 → 走 **timeout**（不是 stuck）。
- **timeout**：步数到上限 → `done=True`，加 `r_timeout`（默认 **-120**）；replay 里 `done_bool` 可按训练节点配置处理（超时当非终止 bootstrap）。

#### 2.2.5 与 spawn 门禁的区别

| | **spawn 门禁**（`_ensure_spawn_ok`） | **step 内碰撞 / 受困** |
|--|-------------------------------------|-------------------------|
| **时机** | 每局 `reset()` / 换图 `switch_world()` **spawn 后** | 每步 `step()` 控制循环中 |
| **目的** | 车 **没放好** 就不开训（防穿模/飞天/贴脸出生） | 训 **过程中** 的行为反馈 |
| **距离阈值** | **`spawn_collision_dist`**（0.28 m，**全周** min sector） | **`collision_dist`**（**0.32** m，**仅前方扇区**） |
| **速度** | 检 \|vz\|、pose 稳定，**无** `v_app` | 碰撞要 **`v_app > collision_v_min`** |
| **失败后果** | 重 spawn（+z / 换 xy），**不进入 step** | 扣分或 stuck **终止本局** |
| **日志** | `[spawn] OK` / `[spawn] FAIL` | `[collision]` / `[stuck]` / `[reward]` |

二者 **参数独立**：spawn 用更松的全周 clearance；step 碰撞用更严的前方 + 速度逻辑。**不要**把 `spawn_collision_dist` 当训练碰撞阈值。

### 2.3 训练逻辑现状（对照用户理解）

用户理解：**随机开图 + 随机起终点 + reward（接近大分、碰撞罚、时间长罚）**。

| 项 | 现状（对照 `ros_env.py` / `td3_train.yaml`） |
|----|-----------------------------------------------|
| 随机起终点 | **已有**：spawn/goal 默认 ±10 m，`min_goal_dist: 4`；**goal 可达性** `_validate_goal()` |
| Reward | **Record 9**：见 §2.1；`r_arrive: 700`、`r_progress: 12`；高努力 $E=\|v\|+\alpha\|\omega\|$（α=0.8, β≈1/3）；`r_low_effort=-0.72`；爬坡 `$s_{\mathrm{climb}}\cdot r_{\mathrm{progress}}\cdot\Delta\rho$（退火 0.25→0.0005）；碰撞 **-12/-30/-60**；stuck/rollover **-160**；timeout **-120**；近终点 `stuck_near_goal_rho=0.7` |
| Episode 结束 | success / **rollover** / stuck / timeout / collision（第 3 次墙撞） |
| 动作 | 对称倒车 `v∈[-2.3,2.3]`，`allow_reverse: true`；`hidden_dim: 1024`，`batch_size: 1024`；`state_dim: 150` |
| 退火 | ep≥100 后按近 50 局成功率余弦退火 `expl_noise` + mild LR + **climb_scale 0.25→0.0005** |
| Spawn 高度 | **已有（硬性门禁）**：无头梯子**先试** `world_spawn_z`（四图均 **0.23**），再扫 0.5→3.5；GUI 轻量从 world_z 抬几档。稳定后检 z、\|vz\|、clearance；**未通过不返回 state**（DEBUG 32） |
| **Spawn 失败不停训** | 本图随机试 **≥40 个 (x,y)** → 仍失败则 **`[spawn] giving up this world=X → switch to Y`**（`pick_next_world` + `switch_world`）。整池一轮都失败：跳过本局、等待、再扫池。**永不**因 spawn 退出进程（DEBUG 35–37） |
| 多 world 切换 | **已有**：`world_switch_interval: 20000`；每 N 步随机换 **4 张**；单张换图失败会 **连试池内最多 4 张**，仍失败则 log `switch FAILED` 并继续训（不锁 playpen） |
| 「10 张图」 | 池子当前 **4 张** `.world`；yosemite/volcano **已删**；逻辑是随机换图 + 随机起终点 + reward，不是固定 10 张 |

---

<a id="ch3"></a>

## 3. 参数总表（与 `td3_train.yaml` 同步）

**日常调参只改 `ros2_ws/src/navigation_3d/scout_offroad_3d/config/td3_train.yaml`**；`train_td3.launch.py` 仅在显式传 `max_timesteps:=` 等 CLI 时才覆盖同名项（**不传 = 纯 yaml**）。**改完必须重新 `ros2 launch` 才生效**（yaml / launch 参数在节点启动时加载，不会热更新）。下表的“默认”已按当前 yaml 校正；任何 Record 8/旧 run 参数只作为历史复盘，不能覆盖当前基线。

数值以磁盘 `ros2_ws/src/navigation_3d/scout_offroad_3d/config/td3_train.yaml` 为准。**调大 / 调小** 指增大或减小该参数的典型效果。奖励/动作已于 Record 7 停训后 **重标定**（见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md) Record 7）：对称倒车、高/低努力 shaping，**不**再靠 `r_forward_scale·(v-|ω|)`。

### 3.1 Reward

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `r_episode_start` | `4.5` | 每局**首步**一次性奖励（Record 9） | 开局正反馈更强 | 开局激励弱 |
| `r_progress` | `12.0` | 靠近目标：`r_progress × Δρ`（**按米**；远离为负） | 更激进朝目标冲 | 目标导向弱 |
| `explore_enable` / `explore_cell_reward` / `explore_grid_res` | `true` / `0.30` / `0.5` | 每局 occupancy 栅格**首次**进入 cell 给小额 `r_explore` | 探索激励更强 | 脱 stuck 弱 |
| `explore_cardiff_scale` / `explore_rho_min` | `false` / `1.0` | `true` 时 `r_explore /= max(ρ, explore_rho_min)` | 近目标探索分更大 | 远距探索分更小 |
| `r_arrive` | `700.0` | 到达成功终止奖（Record 9） | 更愿冒险冲终点 | 到达动力弱 |
| `r_step` | `-0.05` | 每步时间成本 | 更催短路径 | 允许更长绕路 |
| `r_forward_scale` | `0.0` | **legacy**；默认关闭（高努力替代） | 叠加旧 v−\|ω\| 前进项 | — |
| `effort_omega_weight` (α) | `0.8` | $E=\|v\|+\alpha\|\omega\|$ | 更奖励转向努力 | 更偏纯线速度 |
| `effort_reward_scale` (β) | `0.33` | $E\ge e_{\mathrm{hi}}$ 时 $r_{\mathrm{high}}=\beta E$（≈1/3） | 高努力激励更强 | 更依赖 progress |
| `high_effort_thresh` | `0.7` | 触发高努力奖励的 E 下限 | 更难拿高努力分 | 更容易拿分 |
| `r_spin_penalty` | `0.2` | 近零 v + 很高 \|ω\| 时极轻罚 | 更抑制原地打转 | 几乎不罚转 |
| `spin_v_thresh` | `0.1` m/s | 判定「近零线速」 | 高速时仍可能触发 | 更容易判原地转 |
| `spin_w_thresh` | `1.4` rad/s | 判定「很高角速」 | 需更大 ω 才罚 | 轻微转向即罚 |
| `r_low_effort_penalty` | `-0.72` | 持续低 E 时每步罚 | 更逼车动起来 | 允许长时间慢蹭 |
| `low_effort_thresh` | `0.25` | 低努力 E 阈值 | 更难触发 | 更容易触发 |
| `low_effort_time_sec` | `1.5` s | 低努力须持续的窗口 | 须更久才罚 | 更快判磨蹭 |
| `r_slope_climb` / `r_slope_descend` | `0.15` / `0.05` | **legacy**；主路径不用 | — | — |
| `slope_climb_scale` / `_max` / `_min` | `0.25` / `0.25` / `0.0005` | $A_{\mathrm{climb}}=s_{\mathrm{climb}}\cdot r_{\mathrm{progress}}\cdot\Delta\rho$；trainer 按 SR 写 scale | 早期更敢爬 | 学会后淡出 |
| `slope_climb_anneal_success_hi` | `0.80` | 窗 SR 到此 → climb scale 近 min（独立于 expl 的 0.90） | 更早淡出爬坡奖 | 更晚淡出 |
| `slope_climb_pitch_min/max_deg` | `2.0` / `35.0` | 爬/下坡奖 pitch 带 | 更宽坡奖 | 更窄 |
| `r_slope_penalty` | `-0.02` | 坡道嫌疑每步轻罚（**不**计碰撞次数） | 更避坡 | 坡道代价小 |
| `r_collision_base` | `-12.0` | list 空时的回退单次罚 | 单次撞墙更重 | 撞墙惩罚轻 |
| `collision_penalty_list` | `[-12,-30,-60]` | 第 k 次**墙撞**事件罚分 | 列表更负 → 更痛 | 敢贴墙试错 |
| `collision_penalty_double` | `false` | `true` 时用指数递增 | 指数递增 | 固定单次罚 |
| `reward_log_interval` | `25` | 每 N 步打 `[reward]` 摘要 | 日志更疏 | 日志更密 |

### 3.2 碰撞 / 坡墙判别

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `terrain_classify_enable` | `true` | 开多层仰角 + IMU 坡/墙分类 | 关则近距一律按墙 | — |
| `collision_dist` | `0.32` m | 前方扇区 min 距离低于此才进入碰撞候选 | 更远就预警，更保守 | 须更近才判 |
| `collision_v_min` | `0.07` m/s | 朝向障碍分速度须高于此 | 慢速贴墙不算撞 | 慢蹭也算撞 |
| `collision_frames` | `2` | 连续 N 步候选才记 1 次墙撞 | 更防抖 | 更敏感 |
| `collision_sector_half_fov_deg` | `120`° | 碰撞仅车头 ±N° 扇区 | 侧向也计入 | 仅正前方 |
| `max_collision_events` | `3` | 第 N 次墙撞 → `done` | 单局允许多次撞 | 更早结束局 |
| `wall_layers_required` | `2` | ≥N 层 front-min 近距 → 墙 | 坡更易过 | 真墙更敏感 |
| `wall_all_layers_dist` | `0.36` m | 单层 front-min「层近」阈值 | 更容易判墙 | 坡/漏检 |
| `wall_lower_layers_dist` | `0.33` m | 低/中层近距阈值 | 墙/坡都更敏感 | 坡模式难触发 |
| `wall_force_min_dist` | `0.30` m | 水平 min_d 低于此 **强制墙** | 稍远即强制墙 | 须更近才强制墙 |
| `slope_upper_clear_dist` | `1.2` m | 顶层 front-min 高于此倾向坡 | 更容易认坡 | 坡更难认 |
| `slope_max_frames` | `9` | 近距持续 ≤N 帧倾向坡 | 更久近距仍可当坡 | 更快当墙 |
| `slope_pitch_delta_deg` | `4.0`° | IMU pitch 上升超过此支持坡判 | 须更大抬头 | 轻微抬头即认坡 |
| `wall_v_forward_max` | `0.08` m/s | 多层近 + 前进速度 ≤ 此 → 墙倾向 | 稍快仍可能当墙 | 低速更易当坡 |
| `use_imu_pitch` / `use_imu_roll` | `true` / `true` | 坡判用 pitch；翻车用 roll+pitch | — | — |
| `n_elevation_bins` | `4` | 判别用仰角桶数 | 仅影响分类粒度 | — |
| `elevation_bin_edges_deg` / `td3_*` | `[-15,-7.5,0,7.5,15]` | ±15° 四等分；末桶闭区间含 +15° | 语义变，**须重训**（维数可不变） | — |
| `n_elev_front_sectors` | `12` | 判别前方扇区数 | 前方分辨率变化 | — |

### 3.3 Stuck / Rollover / Timeout

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `stuck_time_sec` | `5.6` s | 路径位移滑动窗口长度 | 须更久不动才 stuck | 更快判受困 |
| `stuck_dist_threshold` | `0.25` m | 窗口内位移低于此 → stuck | 须几乎不动 | 小幅挪动也 stuck |
| `stuck_near_goal_rho` | `0.7` m | ρ 小于此不判 stuck（近终点倒车容错） | 更大近区免 stuck | 更小，终点旁仍易误 stuck |
| `r_stuck` | `-160.0` | stuck 终止一次性罚（Record 9） | 更怕受困 | stuck 代价小 |
| `r_rollover` | `-160.0` | 翻车终止罚（Record 9，与 stuck 同级） | 更怕翻车 | 翻车代价小 |
| `r_arrive` | `700.0` | 到达一次性奖 | — | — |
| `rollover_enable` | `true` | 是否 IMU 翻车终止 | 关则翻车不 done | — |
| `rollover_roll_deg` | `60`° | \|roll\| 超此且超 grace → 翻车 | 更难翻车终止 | 更容易翻车终止 |
| `rollover_pitch_deg` | `60`° | \|pitch\| 超此且超 grace → 翻车 | 同上 | 同上 |
| `rollover_sum_deg` | `80`° | \|roll\|+\|pitch\| 超此 → 翻车 | 更难对角翻车 | 更容易 |
| `rollover_grace_sec` | `2.0` s | spawn 后宽限期 | spawn 后更久不判翻 | 更早可因姿态终止 |
| `rollover_grace_abort_enable` | `true` | grace 内翻车 → 重采样 | 关则 grace 内仅忽略 | 开则作废本局 |
| `r_timeout` | `-120.0` | 超时终止一次性罚 | 更怕超时 | 超时代价小 |
| `max_ep_steps` | `600` | 单局最大步数（`control_dt=0.1` → 约 60 s） | 单局更长 | 更早 timeout |
| `goal_tol` | `0.55` m | `ρ` 低于此算到达成功 | 须更近才算到 | 更容易 success |

### 3.4 Spawn / Goal

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `spawn_x/y_min/max` | `±10.0` m | 起点采样框 | 起点更分散 | 起点集中 |
| `goal_x/y_min/max` | `±10.0` m | 目标采样框 | 目标更远/更散 | 目标更近 |
| `min_goal_dist` | `4` m | 起点到目标直线距离下限 | 任务更远更难 | 短距离任务多 |
| `goal_check_enable` | `true` | 采样后做可达性检查 | 关则可能目标在障碍内 | — |
| `goal_min_clearance` | `0.38` m | 目标点周围 min sector 须大于此 | 目标须更开阔 | 允许更贴障目标 |
| `goal_z_max_delta` | `5.0` m | 目标相对 spawn 高度差上限（Record 8/现网；0.55 会卡死采样） | 允许更大高差 | 目标高度更接近起点 |
| `goal_resample_max` | `25` | 目标不合格时重采次数上限 | 更多次尝试 | 易 fallback |
| `spawn_z` | `0.20` m | 无 world 覆盖时的默认高度 | 车更高 | 更低易穿地 |
| `world_spawn_z_entries` | 四图全 `0.23` | 按 world stem 覆盖 spawn 高度 | — | — |
| `spawn_z_scan_enable` | `true` | 无头：world_z 后再扫高；`gui:=true` 强制轻量梯子 | — | — |
| `spawn_z_scan_min/max/step` | `0.5` / `3.5` / `0.5` | **续档**抬高（首档代码强制 world_z，勿当第一档） | 扫得更高 | 更少档 |
| `spawn_z_retry_delta` | `0.5` m | GUI 轻量路径每次抬高 | 重试抬得更高 | 抬升小 |
| `spawn_z_max_retries` | `3` | 单点 z 重试次数 | 同 xy 多试几次 | 更快放弃 |
| `spawn_check_wait` | `0.45` s | spawn 后等物理稳定 | 更稳再检 | 检得早 |
| `spawn_z_jump_threshold` | `0.35` m | 稳定后 z 跳变超此 → FAIL | 容忍更大抖动 | 更严 |
| `spawn_vz_threshold` | `0.5` m/s | 竖直速度超此 → FAIL | 容忍更大下落 | 更严 |
| `spawn_collision_dist` | `0.28` m | **全周** min sector 低于此 → FAIL | 出生须更远离障碍 | 允许更贴脸 |
| `spawn_pose_stable_threshold` | `0.12` m | 位姿稳定判定阈值 | 更松 | 更严 |
| `spawn_full_reset_retries` | `50` | 本图 pose 试尽后换图 | 更多次换点 | 更快切图 |
| `spawn_xy_jitter` | `0.8` m | FAIL 时 xy 随机抖动半径 | 失败时跳更远 | 仍在附近重试 |

### 3.5 速度 / 控制

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `max_v` | `2.3` m/s | 线速度 clip 上限；`allow_reverse` 时 $v\in[-max_v,max_v]$ | 车更快 | 更慢更稳 |
| `max_w` | `1.8` rad/s | 角速度 clip 上限 | 转向更猛 | 转向慢 |
| `allow_reverse` | `true` | 对称倒车（去掉旧 −0.15 下限） | — | `false` → $v\ge0$ |
| `control_dt` | `0.1` s | 每 env 步 sleep | 每步更久 | 步频高 |

### 3.6 训练流程 / 探索噪声 / 成功率退火

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `max_timesteps` | `2000000` | 本次训练总 env 步数 | 训更久 | 早停 |
| `start_timesteps` | `5000` | 纯随机动作步数 | 更多随机覆盖 | 更早用策略 |
| `world_switch_interval` | `20000` | 每 N 步换图；`0` 不换 | 换图更疏 | 换图更频 |
| `expl_noise` | `0.1` | 初始探索噪声 × `max_action`（退火前/上限） | 探索更大 | 更贴确定性策略 |
| `anneal_enable` | `true` | 按窗口成功率余弦退火 **仅** `expl_noise` + mild LR | — | 关则固定噪声/LR |
| `anneal_start_ep` | `100` | 从该 episode 起启用退火 | 更晚才开始收噪声 | 更早收噪声 |
| `anneal_window_ep` | `50` | 成功率滑动窗口（与 metrics 对齐） | 反应更慢 | 反应更快 |
| `anneal_success_lo` / `anneal_success_hi` | `0.10` / `0.90` | $s\le lo$→噪声/LR 最大；$s\ge hi$→最小 | 更晚才收紧 | 更早收紧 |
| `expl_noise_max` / `expl_noise_min` | `0.1` / `0.001` | 退火 σ 上下界 | 探索范围更大 | 下限更接近确定性 |
| `actor_lr` / `critic_lr` | `3e-4` | Adam lr **上限**（低成功率） | 更新更猛 | 更稳 |
| `actor_lr_min` / `critic_lr_min` | `1e-4` | Adam lr **下限**（高成功率） | 高成功率时仍较猛 | 更保守 |
| `save_freq` | `5000` | 每 N 步存 checkpoint | 存盘更稀 | 存盘更密 |
| `seed` | `2` | RNG 种子 | 换种子 | — |
| `noise_adapt_enable` | `true` | 按成功率调感知噪声 | 开则自动加/减噪 | — |
| `state_noise_std` | `0.01` | 拼进 state 的高斯噪声基线（`noise_adapt` 会覆盖） | 观测更噪 | state 更干净 |
| `sensor_noise_min` / `max` | `0.005` / `0.03` | 自适应噪声上下界 | 更噪 | 更干净 |
| `action_stats_enable` | `true` | 每 ep 把发出的 |v|/|ω| 均值写入 `mean_v`/`mean_w` | — | 关则无速度统计图 |
| `landscape_ckpt_enable` | `true` | 每 N ep 另存权重供景观 GIF | — | 关则只有静图 |
| `landscape_ckpt_every_n_ep` | `20` | 景观快照间隔（训练 ep） | 帧更密、盘更大 | 帧更疏 |

> **重要**：退火 **不** 调整固定 reward 系数（`r_progress` / `r_arrive` 等）；**例外**：Record 9 的 $s_{\mathrm{climb}}$（`slope_climb_scale`）随窗成功率余弦衰减（§2.1.5）。

### 3.7 TD3 网络超参

| 参数 | 默认 | 含义 | 调大 → | 调小 → |
|------|------|------|--------|--------|
| `batch_size` | `1024` | replay 采样批大小 | 梯度更稳、显存↑ | 更新噪 |
| `hidden_dim` | `1024` | Actor/Critic MLP 隐层宽度（Record 9） | 容量更大，须重训 | 容量更小 |
| `discount` | `0.99` | γ | 更重视长期到达 | 更短视 |
| `tau` | `0.006` | 目标网络软更新率 | 目标跟得更快 | 更滞后 |
| `policy_noise` | `0.2` | 目标策略平滑噪声 | 更扰动 | 平滑弱 |
| `noise_clip` | `0.5` | 目标噪声 clip | 更大目标扰动 | 上限更小 |
| `policy_freq` | `2` | 每 N 次 critic 更新才更新 actor | actor 更少更新 | 更频 |
| `n_sectors` | `36` | 水平扇区数 | state 维变，**须重训** | 同上 |
| `n_elevation_bins_td3` | `4` | TD3 仰角层数（±15° 四等分） | state_dim=150，**须重训** | 同上 |
| `use_imu_in_state` | `true` | 状态追加 roll/π,pitch/π | 关则 148 维 | — |
| `lidar_v_samples` / `h` | `16` / `360` | 仿真射线密度 | RTF↓ | 更稀更快 |
| `max_range` | `5.0` m | 扇区距离归一化分母 | 同距离占 state 比例变小 | 更饱和 |

### 3.8 仿真物理 / 碰撞网格（`sim_*`，由 yaml 统一控制）

地图内容（树/墙/地形）不变，这里只调**物理求解与碰撞分辨率**。launch 时把参数打进 `.world` 的**缓存副本**（`$SCOUT_WS_ROOT/3D/debug_logs/world_cache/`）再喂给 gzserver，**源 `.world` 永不被改**；换图时 backend 对每张图同样套用。它是当前会话的可再生调试产物，验证结束后按归档规则移动，不要把它误当成训练权重或手工编辑源 world。

| 键 | 默认 | 含义 | 调大 / 调小 |
|----|------|------|------------|
| `sim_max_step_size` | `0.01` | 物理步长 (s)，0.01=100Hz | 调小更准更慢；0.02 略快、坡上接触略粗 |
| `sim_real_time_update_rate` | `100` | 物理速率上限 (Hz) | **CPU 跑不满时无影响**（RTF 上限 = rate × step）；仅当 CPU 有余量时调大可超实时 |
| `sim_solver_iters` | `20` | ODE quickstep 迭代数 | 30=更准（原生成值）；20=略快。评测保真可回 30 |
| `sim_collision_heightmap_res` | `65` | 碰撞地形网格 | **65=快（训练，RTF×3，实测 0.22s/step）**；129=满血（评测/展示，0.63s/step）。视觉始终 129² 不变 |
| `lidar_update_rate` | `10` | 雷达频率 (Hz) | 5=省 ~16% 步时（特征偶有一步延迟）；10=满血 |

> **地面摩擦不由 yaml 补丁改写。** heightmap 源 `.world` 保持裸 `<ode/>`（与 Record 8 当晚一致）。轮子 `scout_mini.gazebo` 原厂 **μ1=1.0 / μ2=0.9**。禁止再加 `sim_ground_mu` 或改轮 μ。

> **为什么 65² 碰撞够真**：0.75m 鼓包在 0.46m 网格下仍占 5~10 格，形状保持；接触模型（ODE heightfield）不变。出论文数据时用 `sim_collision_heightmap_res: 129` + `sim_solver_iters: 30` 跑评测即可（DEBUG 记录 47 有完整 A/B 证据）。

### 3.9 Launch 参数：`train_td3.launch.py`（`gui` 等不在 yaml）

| 参数 | 默认 | 单位 | 含义 |
|------|------|------|------|
| `world:=` | `random`（4 张随机） | 路径 | **只决定第一张图**；之后仍按 yaml `world_switch_interval` 换 |
| `gui:=` | `false` | bool | Gazebo 是否开窗口 |
| `max_timesteps:=` | 不传=用 yaml | 训练 timestep | 可选覆盖 |
| `start_timesteps:=` | 不传=用 yaml | 训练 timestep | 可选覆盖 |
| `world_switch_interval:=` | 不传=用 yaml | 训练 timestep | 可选覆盖；锁图 `0` |
| `output_dir:=` | 空 | 路径 | 产物目录 |
| `spawn_delay:=` / `x/y/z/yaw` | `1` / `0/0/0.15/0` | s / m·rad | 首次 spawn |

### 3.10 改完怎么生效

```bash
# 1) 编辑 ros2_ws/src/navigation_3d/scout_offroad_3d/config/td3_train.yaml
# 2) 若改了 Python / launch，重新 colcon build
cd $SCOUT_WS_ROOT/ros2_ws && colcon build --symlink-install --packages-select scout_offroad_3d
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash
# 3) 重新 launch（回到自己启动的旧 launch 终端按 Ctrl+C；只读检查端口后再启动）
ros2 launch scout_offroad_3d train_td3.launch.py gui:=false
```

换图实现：`train_td3_node` 在 `t % N == 0` 时调用 `ScoutGazeboEnv.switch_world`（**受控停止它自己启动的 gzserver → 用新 `.world` 再起 → spawn 车**）。Gazebo Classic **不能**热加载换 `.world` 文件，故重启仿真进程；**训练节点与 ROS master 不停**，**不清空** replay；随后 `reset()` 仍随机起终点。日志：`[world] switch to xxx.world at t=...`。

---

<a id="ch4"></a>

## 4. 调参顺序建议

**原则**：先让「终止条件与传感判据」可信，再塑 reward；否则策略会对错误信号过拟合。

**1. 碰撞 / 坡墙（最先）**

1. 固定 `world` + `world_switch_interval:=0` 冒烟，看 `[terrain] slope` vs `[collision] wall` 比例。
2. 坡道误当墙 → **先**略增 `slope_upper_clear_dist` / `slope_max_frames`，或略增 `wall_layers_required`；仍误判再略增 `wall_force_min_dist`（更不易强制墙）或略降 `wall_all_layers_dist`（更难层近）。
3. 真墙不罚 → **增大** `collision_dist` / `wall_all_layers_dist` / `wall_force_min_dist`，或略降 `collision_v_min` / `collision_frames`（注意坡道副作用）。
4. 确认后再动 `collision_penalty_list` / `max_collision_events`。

**2. Reward（其次）**

1. 保持高努力 β 与 `r_progress` 的相对量级可解释（现网 β≈1/3、`r_progress=12`），避免「刷努力分」hack；**不要**退火固定 `r_*`（爬坡用独立的 $s_{\mathrm{climb}}$ 退火）。
2. 到达率太低 → 略增 `r_progress` 或 `r_arrive`；碰撞多 → 先回到碰撞表，勿只加大 `r_arrive`。
3. 原地转多 → 略增 `r_spin_penalty` 或降 `spin_w_thresh`（默认已极轻）；磨蹭不走 → 增 `|r_low_effort_penalty|` 或降 `low_effort_time_sec` / `low_effort_thresh`。
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
| **真墙不罚** | **增大** `collision_dist` / `wall_all_layers_dist` / `wall_force_min_dist`；略降 `collision_v_min` / `collision_frames` |
| **顶墙不算撞** | 略降 `collision_v_min`（注意坡道误触） |
| **骑墙不停** | 降 `stuck_time_sec` / `stuck_dist_threshold` |
| **单局撞墙过多** | 降 `max_collision_events` 或加大 `collision_penalty_list` 负值 |

**颠簸地形冒烟**（`bumpy_offroad.world`，锁图）：

```bash
ros2 launch scout_offroad_3d train_td3.launch.py gui:=true \
  max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0 \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world
```

另开终端验收多层话题：

```bash
ros2 topic echo /offroad/elevation_features --once   # layout: 4×12, data 长度 48
ros2 topic echo /imu/data --once
```

通过标准：`[spawn] OK`；上坡时多见 `[terrain] slope suspected`、**少见** `[collision] wall confirmed`；真顶墙仍有 `wall confirmed`；第 3 次墙撞 `outcome=collision`；`[reward] high` 在大油门/大转向时出现。

**键盘试冲陡坡**（无 TD3，手控 `/cmd_vel`）：

```bash
# 依赖: sudo apt install ros-humble-teleop-twist-keyboard
bash 3D/scripts/keyboard_climb_test.sh              # 默认 bumpy_offroad
bash 3D/scripts/keyboard_climb_test.sh bumpy_offroad_2
```

焦点在 teleop 终端；冲不上先考虑绕坡。轮 μ 原厂 **1.0/0.9**，地面不补丁；**不要**收紧 `goal_z_max_delta`（须保持 5.0），**不要**改摩擦。

---

<a id="ch5"></a>

## 5. 训练操作（冒烟 / 长训 / 续训）

### 5.1 前置：确认无自己启动的残留 · source · 编译 · 装 torch

**每个新终端都要 source：**

```bash
source /opt/ros/humble/setup.bash
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash
```

下面命令块均默认「该终端已执行过上面两行」。

**确认 Gazebo 是否仍被自己启动的旧任务占用（换 launch 前强烈建议）：** 端口占用、`Address already in use`、黑屏卡死、spawn 失败，多半是旧 `gzserver` 尚未退出。不要广泛杀进程：回到启动该任务的终端按 `Ctrl+C`，等退出后只读检查：

```bash
# 先回到自己启动该任务的 ros2 launch 终端，按 Ctrl+C 并等待退出。
# 只读检查；若显示陌生任务，不要终止它，先确认所属终端/使用者。
ss -ltnp | rg ':11345\b' || true
pgrep -a gzserver || true
```

**编译（改过 `scout_offroad_3d` 或首次）：**

```bash
source /opt/ros/humble/setup.bash
cd $SCOUT_WS_ROOT/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix scout_offroad_3d
# 应打印 .../ros2_ws/install/scout_offroad_3d
```

**安装 PyTorch（Jammy = Python 3.10）：**

```bash
python3 -m pip install --user -r $SCOUT_WS_ROOT/3D/scripts/requirements-ros2.txt
python3 -c "import torch; print(torch.__version__)"
```

### 5.2 训前验收扇区（建议做一次）

确认 **16 线点云** → `/offroad/td3_range_features` 正常（TD3 特征 **4×36=144**；状态维 **150**）。

**终端 1 — Gazebo 官方车 / 越野 world：**

```bash
ros2 launch scout_mini_pro_description gazebo_room.launch.py gui:=false \
  lidar_v_samples:=16 lidar_h_samples:=360 \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world
```

等约 8–15 s，车 spawn 完成。

**终端 2 — 扇区节点：**

```bash
ros2 launch scout_offroad_3d range_features.launch.py
```

**终端 3 — 验收：**

```bash
ros2 topic echo /lidar/points --once
ros2 topic hz /lidar/points
ros2 topic echo /offroad/td3_range_features --once
ros2 topic echo /offroad/range_features --once
ros2 topic echo /offroad/elevation_features --once
```

**通过标准：**

- `/offroad/td3_range_features`：`elevation_bin_td3=4`，`sector=36`，共 **144** 个 min range（TD3 感觉层）
- `/offroad/range_features`：`layout` 扇区 **36**（水平投影，碰撞/兼容）
- `/offroad/elevation_features`：`elevation_bin=4`，`front_sector=12`，共 **48** 个 min range（坡/墙判别）
- 车身 self-hit 被滤掉：前方扇区不应长期全是 0.18 m 贴脸值

**2D `/scan` 回退**（旧 2D 激光或对照）：

```bash
ros2 launch scout_offroad_3d range_features.launch.py source:=scan scan_topic:=/scan
```

深度相机（仅前方，**不是**全周 16 线）：

```bash
ros2 launch scout_offroad_3d range_features.launch.py source:=cloud \
  cloud_topic:=/camera/depth/points cloud_msg_type:=pointcloud2 frame_mode:=optical full_circle:=false
```

验收完后 Ctrl+C，按 §5.1 清残留，再进训练。

### 5.3 一键 launch 里有什么

`train_td3.launch.py` **一个终端**一次拉起：

1. `gazebo_room.launch.py`（官方 Scout Mini + **启动时 random** 第一张 `worlds/*.world`；训练中按 interval 换图）
2. `range_features.launch.py`（16 线 `/lidar/points` → 扇区）
3. `train_td3_node`（在线 TD3；按 `world_switch_interval` 换图；结束写 `metrics.csv` 并自动出图）

**不含** A\* / Arena / SLAM / `map_server`。

### 5.4 短冒烟（确认链路通，再开长训）

**推荐（唯一入口，含 flock 互斥）：**

```bash
bash $SCOUT_WS_ROOT/3D/scripts/humble_train_smoke.sh
# 已编译: bash $SCOUT_WS_ROOT/3D/scripts/humble_train_smoke.sh --no-build
```

**手动换图短测：**（先按上面的 `Ctrl+C` / `ss` 流程确认旧 launch 已退出。）

```bash
ros2 launch scout_offroad_3d train_td3.launch.py gui:=false \
  max_timesteps:=80 start_timesteps:=10 world_switch_interval:=25
```

冒烟通过标准：

- 无 `PyTorch not found`
- 日志有 **`[spawn] OK`**、`[ep=]` / `[reward]`
- 换图冒烟：`[world] switch complete`
- `$SCOUT_WS_ROOT/3D/runs/smoke_*/` 含 `config.yaml`、`train.log`、`metrics.csv`、`td3_actor`

### 5.5 当前默认长训（Record 9）

```bash
source /opt/ros/humble/setup.bash
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash

ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false
```

说明：**不传** `max_timesteps` / `start_timesteps` / `world_switch_interval` 时 **完全读当前 yaml**：默认 `max_timesteps=2000000`、`start_timesteps=5000`、`world_switch_interval=20000`。首张 world 默认 **random**（4 张）。调 reward / 碰撞 / 换图间隔 → 只改 `ros2_ws/src/navigation_3d/scout_offroad_3d/config/td3_train.yaml`。

> **Record 8 注意（历史）**：当时 `hidden_dim=768`、`state_dim=112`，历史训练常使用 `max_timesteps=500000` / `start_timesteps=4000`；这些不是当前默认，勿与 Record 9 混用。  
> **Record 9 注意（现网）**：`hidden_dim=1024`、`state_dim=150`（4 仰角 + IMU）、雷达训练默认 **16×360**、爬坡 $s_{\mathrm{climb}}$ 余弦退火。**禁止 resume Record 8**；必须全新 run。复盘见 [`TRAIN_RECORDS.md`](TRAIN_RECORDS.md)。

**指定首张平地 + 换图：**

```bash
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false \
  max_timesteps:=2000000 \
  start_timesteps:=5000 \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world \
  world_switch_interval:=20000
```

**本地有界面训练（调试看车）：**

```bash
ros2 launch scout_offroad_3d train_td3.launch.py gui:=true
```

### 5.6 同目录续训

加载 `td3_*` + 从 `metrics.csv` 最后一行的 t/ep 接着写；replay 清空；曲线结束后在同一文件夹重画。**续训绝不会自动发生**：必须明确传 `resume:=true`、显式指定已有 `output_dir:=`，并显式给出大于当前 t 的 `max_timesteps:=N`：

```bash
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false \
  output_dir:=$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS \
  resume:=true \
  max_timesteps:=2000000 \
  start_timesteps:=5000 \
  world_switch_interval:=20000
```

必须 `max_timesteps` **大于** 当前 t，且该值必须作为 launch 参数显式传入；节点会拒绝省略 horizon 的续训，不会从 yaml 静默拉长。新训练若指向非空 `output_dir` 也会拒绝，避免覆盖/拼接旧 run。若有旧 launch，回到**自己启动**它的终端 `Ctrl+C`，再按 §5.1 的只读 `ss` 检查确认退出。

> **续训仅适用于同维数权重**（现网 **150** 维 / hidden **1024**）。旧 Record（含 R8 的 112/768）不能跨维续训。

### 5.7 指定产物目录

`output_dir` 已由 `train_td3.launch` → `train_td3_node` 接通：

```bash
# 仅为“全新 run”指定一个此前不存在的目录；若目录非空，fresh run 会拒绝启动。
OUT="$SCOUT_WS_ROOT/3D/runs/manual_run_YYYYMMDD_HHMMSS"
test ! -e "$OUT"
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false \
  max_timesteps:=50000 \
  output_dir:="$OUT"
```

含义：

- `output_dir` = 本次 run 的权重 / `train.log` / `metrics.csv` / 曲线图写到你指定的**新建且为空**的文件夹（方便 `scp` 到服务器或事后找结果）；非空目录只允许在明确 `resume:=true` 时使用
- **不设** `output_dir` 时，用默认 `$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/`
- 目录在 **训练节点一启动就会创建**（打印 `TD3 train output → ...` 时已存在），不是整场训完才有

### 5.8 产物路径与文件名（推理 / 汇报必看）

**默认根目录**（未传 `output_dir` 时）：

```text
$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/
```

工作区根目录解析顺序：`SCOUT_WS_ROOT` → `CATKIN_WS` → `WS_ROOT` → ROS package share 推导；不要把某位用户的 home 路径写进命令。详见 [`runs/README.md`](runs/README.md)。

**何时生成**：`train_td3_node` **一开始**就会 `makedirs` 该目录并写入 `config.yaml` / 打开 `train.log`、`metrics.csv`（终端有 `TD3 train output → ...`）。训练过程中持续追加 log/metrics；`save_freq` 到达或正常结束时写 `td3_actor` / `td3_critic`；结束（或 Ctrl-C 进 `finally`）再出 `*.png`。**不是**全部训完才突然出现空目录。

| 文件 | 含义 |
|------|------|
| `td3_actor` | Actor 网络参数（**无后缀**，Fujimoto 风格） |
| `td3_critic` | Critic（双 Q）网络参数 |
| `td3_actor_optimizer` / `td3_critic_optimizer` | 优化器状态（续训用；推理可不拷） |
| `train.log` | 每 episode 文本行：`outcome` / `reward` / `success` / `stuck` / `collision_events` / `rho` |
| `metrics.csv` | episode 指标 CSV（列说明见 §6 出图） |
| `config.yaml` | 本次 run 的**实际生效**超参快照（`meta` + `train` + `env`；含 launch 覆盖） |
| `params_backup.yaml` | 与 `config.yaml` 相同（双份归档） |
| `reward_curve.png` 等 | 训完自动生成的科研风曲线（见 §6） |

保存前缀是目录下的 **`td3`**（代码里 `model_prefix = .../td3`），因此磁盘上是：

```text
.../run_xxx/td3_actor
.../run_xxx/td3_critic
```

推理时 `weights:=` 填的是前缀 **`.../run_xxx/td3`**（**不要**写成 `td3_actor`，也不要带扩展名）。

查看最新 run / 每次 run 里有什么：

```bash
# 哪个 run 最新（按修改时间）
ls -lt $SCOUT_WS_ROOT/3D/runs/ | head
# 每次 run 目录内文件（actor/critic/csv/png 等）
ls -la $SCOUT_WS_ROOT/3D/runs/run_*/
```

### 5.9 分终端：本地短训 → 监控

> **一共开几个终端？**
> - **最少 1 个**：`ros2 launch scout_offroad_3d train_td3.launch.py gui:=false` 即可把 **Gazebo + 扇区 + TD3** 全拉起。
> - **推荐 2 个**：终端① 训练；终端② 监控（`ros2 topic` / 看 `train.log`）——终端2 **可选**。
> - 换 launch 前先按 §5.1 清残留；**每个新终端**都要先 `source`。

**终端2 监控（强烈建议）**，另开一个终端，同样先 `source`，再任选一类：

```bash
# 看扇区话题是否在跳
ros2 topic hz /offroad/range_features
ros2 topic echo /offroad/td3_range_features --once

# 看训练日志（产物目录以终端1 打印的 TD3 train output → ... 为准）
ls -lt $SCOUT_WS_ROOT/3D/runs/ | head
tail -f $SCOUT_WS_ROOT/3D/runs/run_*/train.log

# 看速度指令是否在发
ros2 topic echo /cmd_vel -n 5
```

每局 `reset` 时终端会打一行 `[ep=N] world=... start=... goal=... spawn_z=...`；训练中周期性 `[reward] ...`；碰撞 `[collision] event #k ...`（不结束局）；受困 `[stuck] displacement=... → reset`；该局结束 `[ep=N] done outcome=...`。

终端2 **不跑** `ros2 launch train_td3`；只观察，避免双开 Gazebo。

| 场景 | 终端数 | gui | max_timesteps | world |
|------|--------|-----|---------------|-------|
| 本地短冒烟 | 1～2 | `true`（想看车） | `2000` | 默认随机第一张 / `world:=` 定第一张；yaml `world_switch_interval` 控制后续换图 |
| 服务器长训 | 1（+可选监控） | `false` | yaml 默认 `2000000` | 同上；现网 `viz_eval_enable: true`，嫌慢可关 |

---

<a id="ch6"></a>

## 6. 出图与科研可视化

> **全部移到 [`README2.md`](README2.md)**：科研曲线（`plot_td3_metrics`）、定轨（`viz_eval` 默认 **4×3=12** 张）、热力图、指令速度 `action_viz/`（`mean_v`/`mean_w`）、损失景观 + 可选 GIF。训毕 `auto_plot: true` 自动出；详见 README2 §1–§8。

---

<a id="ch7"></a>

## 7. 地图管理（打开 / 编辑 / 换图）

> **TD3 地图池 = 恰好 4 张 bumpy**（训练换图 + viz_eval 共用）。
> 包内 `models/uneven_ground_td3_1…4`。大图归档 `arena/bumpy_offroad_full.world`。细节见 [`worlds/SOURCES.md`](../ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/SOURCES.md)。

### 7.1 规范目录与清单

阶段 A **只要**仿真 `.world`，**不要** SLAM 建图产物。

**唯一规范放置处：** `ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/`
（ROS 包内资源；**launch / XML 内**可用 `$(find scout_offroad_3d)/worlds/...`；**bash 命令行**须用 `$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/...`。**不用** `3D/worlds/`。）

清单与溯源见包内 [`ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/SOURCES.md`](../ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/SOURCES.md)。

| 文件 | 来源 | 用途 |
|------|------|------|
| `bumpy_offroad.world` (#1) | `uneven_ground_td3_1` | **train**；spawn_z **0.23** |
| `bumpy_offroad_2.world` | `uneven_ground_td3_2` | **train**；spawn_z **0.23** |
| `bumpy_offroad_3.world` | `uneven_ground_td3_3` | **train**；spawn_z **0.23** |
| `bumpy_offroad_4.world` | `uneven_ground_td3_4` | **train**；spawn_z **0.23** |

**当前训练池 = 恰好 4 张 bumpy**（`train_worlds`）。`_5`/`_6` 已删除。Arena 大图在 `arena/`，不入随机池。

**模型（训练图已打进包，通常无需再装）：**

```bash
# TD3 训练用（包内，GAZEBO_MODEL_PATH 已含 share/.../models）
ls $(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/models/uneven_ground_td3_1/model.sdf

# 仅 Arena 大图才需要外部 uneven_ground；不要写入用户模型缓存。
# 用你受控的外部模型根，并显式提供给 Gazebo：
# EXTERNAL_GAZEBO_MODELS="$SCOUT_WS_ROOT/3D/external_models"
# test -f "$EXTERNAL_GAZEBO_MODELS/uneven_ground/model.sdf"
# export GAZEBO_MODEL_PATH="$EXTERNAL_GAZEBO_MODELS${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

### 7.2 打开地图（无车看地形 / 带车短冒烟）

```bash
source /opt/ros/humble/setup.bash
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash
# 若此前由你启动过 Gazebo，先回到对应终端 Ctrl+C；只读检查：
ss -ltnp | rg ':11345\b' || true

# ① 仅看地形（无车；改图用这条）
ros2 launch scout_offroad_3d view_world.launch.py \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world

# ② 带 TD3 训练栈开新地图（短冒烟）
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false \
  max_timesteps:=500 start_timesteps:=50 world_switch_interval:=0 \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world
```

**改图 / 只看地形（无车）必须用** `view_world.launch`（只加载 `.world`，**零 spawn**）。
`gazebo_room_official.launch` **会带车，存盘会嵌车，不要用来改图**。训练带车用 `train_td3.launch`。

**命令行换 world 前必读：**

1. **先 source**：`source /opt/ros/humble/setup.bash` 且 `source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash`（未编译则先 `colcon build --symlink-install`），否则 `ros2 pkg prefix` / `ros2 launch` 会报包找不到。
2. **bash 命令行**写路径时用 **`$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d`**，**不要**写 `$(find scout_offroad_3d)`——后者是系统 `find`，会报 `No such file or directory`。
3. **仅 launch / XML 内部**才可用 ROS 替换 `$(find pkg)`（由 ros2 launch 解析，不是 bash）。

**四张训练图分别打开（#1–#4）：**

```bash
for w in bumpy_offroad bumpy_offroad_2 bumpy_offroad_3 bumpy_offroad_4; do
  ros2 launch scout_offroad_3d view_world.launch.py \
    world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/${w}.world
done
```

`world_spawn_z`（yaml stem）：四张 bumpy **全 0.23**（与 Record 8 `params_backup` 一致）。

**Gazebo world 结构（自建图）：**

```text
<sdf><world>
  ground_plane   → <plane> 碰撞 + visual
  obstacle_*     → 静态 box/cylinder 等
</world></sdf>
```

### 7.3 GUI 覆盖保存（同一文件、同一目录）

**Save As「没用」的唯一常见原因（2026-09-03）：** 写到了历史 Noetic `src/` 树里的同名 world（该树不是当前入口），而 Humble 打开的是
`$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/`。存盘成功了，但 reopen 读的是另一份文件。
`view_world.launch.py` 现在会把 world **realpath** 打到终端——**只覆盖那一行路径**。

1. 用上面的 **`view_world.launch.py`** 打开（模型树里不应有 `scout_mini`）。看终端：
   `[view_world] Loading world (realpath):` 后面那一行。
2. Insert 植物 / 箱体 / 障碍（左栏 **Insert** → 拖进场景）。
3. 菜单 **File → Save World As**，覆盖终端打印的路径（不要另存到 Downloads，也不要选没有 `ros2_ws` 的旧 `src/`）：
   `$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/<name>.world`
4. 存完立刻检查 mtime + 无车：

```bash
ls -la $SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/<name>.world
grep -n scout_mini \
  $SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/<name>.world
```

若 grep 仍有输出：说明用了带车 launch 存盘。用文本编辑器删掉整个 `<model name='scout_mini'>…</model>`，以及 `<state>` 里同名的 `<model name='scout_mini'>` 块，再存盘。历史：旧平地 clutter 曾因 `gazebo_room_official` 存盘嵌第二辆车 → 双车 / `lock_error`；现训练池仅 bumpy，仍勿用带车 launch 存盘。

Gazebo GUI：**View → Transparent / Collisions**（看碰撞体是否实心、是否接地面）。改图窗口里没有车，不要 `ros2 topic pub /cmd_vel`。
终端里 `Node::Advertise(): Error advertising topic [.../joint_cmd]` **不是** Save 失败，可忽略。

**训练侧验证**（spawn 后看车是否正常避障）：

```bash
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=true max_timesteps:=500 start_timesteps:=50 \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world \
  world_switch_interval:=0
```

通过标准：`[spawn] OK`；GUI 中 **View → Collisions** 障碍碰撞体合理；真撞墙有 `[collision] wall confirmed`（记录 14）。历史 `rough_ramps.world` 已删除，见 DEBUG 记录 18。

### 7.4 在 `train_td3.launch` 里换 world

已支持 `world:=`（传给 `gazebo_room_official.launch` 的 gzserver）。**命令行**仍须 `$(ros2 pkg prefix ...)`，勿用 bash 的 `$(find ...)`。

```bash
# 默认：启动时从 worlds/*.world 随机抽第一张；训练中按 world_switch_interval 换图
ros2 launch scout_offroad_3d train_td3.launch.py gui:=false

# 指定第一张越野/障碍 world（绝对路径；之后仍按 interval 换，除非 yaml=0）
ros2 launch scout_offroad_3d train_td3.launch.py \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world

# 短冒烟换图：每 50 步换一张，总共 200 步
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false world_switch_interval:=50 max_timesteps:=200 start_timesteps:=50

# 短名 → 绝对路径（同样只定第一张）
ros2 launch scout_offroad_3d train_td3.launch.py \
  world:=$(ros2 run scout_offroad_3d pick_random_world.py bumpy_offroad)
```

（说明：`train_td3.launch` 默认已在 parse 时从包内 `worlds/*.world` 抽**第一张**；新增 `.world` 后请同步 launch 列表，或运行 `ros2 run scout_offroad_3d pick_random_world.py --print-launch-eval`。训练中换图由 `world_switch_interval` 控制，见 §3。）

### 7.5 手动下载链接（外网/LFS 失败时）

把 `.world`（及所需 model）放进 `ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/`：

| 来源 | URL |
|------|-----|
| Clearpath 额外户外（inspection / orchard / solar） | https://github.com/clearpathrobotics/cpr_gazebo （`noetic-devel`） |
| Clearpath Ignition 户外（pipeline 等） | https://github.com/clearpathrobotics/clearpath_simulator |
| AWS RoboMaker 样本（已归档，可看 `ros1` 分支） | https://github.com/aws-robotics/aws-robomaker-small-warehouse-world |

若 playpen 依赖外部模型（例如 `jersey_barrier`），把它们放在**受控的外部模型根**并显式设置 `GAZEBO_MODEL_PATH`；本项目不会扫描或自动写入用户模型缓存。缺模型时应按 Clearpath/OSRF 的来源说明准备到该模型根，并记录外部依赖，而不是让 Gazebo 静默下载。

---

<a id="ch8"></a>

## 8. 服务器训练与权重回拉

> **路径策略（强烈建议）**：本机和训练服务器都明确设置自己的 `SCOUT_WS_ROOT`；不假定用户名或 home 目录相同。当前代码的根目录解析优先级是 `SCOUT_WS_ROOT` → `CATKIN_WS` → `WS_ROOT` → package share 推导，禁止把某台机器的绝对路径写死进仓库。
> **RViz / 相机录制仅本地 demo 用**；服务器长训保持 `gui:=false`。现网 yaml `viz_eval_enable: true`（每 100 ep 定轨）；服务器若嫌慢可改 `false`。

### 8.1 连上训练机（SSH + 图形桌面）

在本机受控终端中临时设置占位值；真实值从管理员或本地凭据管理器取得，**绝不写入 README、脚本、Git 配置或提交记录**：

```bash
export TRAIN_HOST='训练机主机名或地址'
export TRAIN_USER='训练机账号'
export SSH_PORT='SSH 端口'
export RDP_PORT='RDP 端口'
export REMOTE_WS_ROOT='/训练机上的/catkin_ws_2/绝对路径'
```

| 用途 | 怎么连 | 说明 |
|------|--------|------|
| 命令行、`rsync`/`scp`、看 `train.log` | **`SSH_PORT`** | 无图形；与桌面会话独立。拷文件：`ssh -p "$SSH_PORT"`、`scp -P "$SSH_PORT"`、`rsync -e "ssh -p $SSH_PORT"` |
| 看服务器屏幕（已开的桌面 / Gazebo） | **`RDP_PORT`**（服务器上 xrdp） | 与 Windows「远程桌面」**同一路会话**。该桌面一般是 **`DISPLAY=:10`**，训练 GL（`gzclient` 等）走这个 display |

**本机 Ubuntu 看屏：用 Remmina（已装），协议 RDP。不要把 NoMachine 当主路径。**

1. Remmina → 新建连接：协议 **RDP**，服务器 `$TRAIN_HOST`，端口 `$RDP_PORT`，用户名 `$TRAIN_USER`。
2. 首次证书提示应按学校/实验室的主机指纹流程核验；不确定时先联系管理员。
3. 若画面是黑底上一个很小的矩形：xrdp 分辨率偏低。Remmina **左侧栏切全屏**，或把分辨率设成 **1920×1080** / 使用客户端分辨率，然后重连。

看图形走 **`RDP_PORT`**；SSH **`SSH_PORT`** 只负责终端，进不去桌面、也看不到 Gazebo 窗口。

### 8.2 拷什么（Humble · 最小 vs 完整）

**最小（阶段 A 无头训练够用）：**

```text
$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/      # worlds/、config/td3_train.yaml、TD3、launch
$SCOUT_WS_ROOT/ros2_ws/src/shared/scout_description/             # URDF / meshes
$SCOUT_WS_ROOT/ros2_ws/src/shared/scout_mini_pro_description/    # gazebo_room.launch.py
$SCOUT_WS_ROOT/3D/scripts/                                       # requirements-ros2.txt、humble_train_smoke.sh 与当前验证脚本
$SCOUT_WS_ROOT/3D/README1.md                                     # 可选，对照命令
```

**建议再带（少踩坑）：**

```text
# 2D 是外置仓库 scout-navigation-2d；3D 训练不需同步其源码。
# Gazebo 模型（playpen clutter 依赖；缺则世界缺障碍）
# 外部 Gazebo 模型根（若 Arena/playpen 需要，显式用 GAZEBO_MODEL_PATH 提供；训练 bumpy 的 uneven_ground_td3_1…4 已在包内）
```

**可选：**

| 项 | 何时拷 |
|----|--------|
| `3D/runs/<某 run>/td3_*` + `config.yaml` | **续训** (`resume:=true` + `output_dir:=`，须同维数 **150/1024**) |
| `3D/runs/` 全量 | **不要**（体积极大；训完再 scp 拉回需要的 run） |
| `3D/debug_logs/`、`verify_*` 日志 | **不要** |
| ROS1 `src/`（Noetic：`scout_gazebo_sim` 等） | **不要**（Humble 训练走 `ros2_ws`） |
| 外置 2D 导航 / RViz 录制脚本 | **不要**（服务器训练无关） |
| `ros2_ws/build`、`install`、`log` | **不要**（服务器上重新 `colcon build`） |

更稳妥的同步：

```bash
# 所有真实地址与账号来自本地环境变量；不要把值填回本文。
rsync -avz --progress -e "ssh -p $SSH_PORT" "$SCOUT_WS_ROOT/ros2_ws/src/" \
  "$TRAIN_USER@$TRAIN_HOST:$REMOTE_WS_ROOT/ros2_ws/src/"
rsync -avz --progress -e "ssh -p $SSH_PORT" "$SCOUT_WS_ROOT/3D/scripts/" \
  "$TRAIN_USER@$TRAIN_HOST:$REMOTE_WS_ROOT/3D/scripts/"
# 可选续训权重
# rsync -avz -e "ssh -p $SSH_PORT" "$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/" \
#   "$TRAIN_USER@$TRAIN_HOST:$REMOTE_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/"
```

整仓同步时排除大目录：

```bash
rsync -avz --progress -e "ssh -p $SSH_PORT" \
  --exclude 'ros2_ws/build' --exclude 'ros2_ws/install' --exclude 'ros2_ws/log' \
  --exclude '3D/runs' --exclude '3D/debug_logs' --exclude 'src/' \
  "$SCOUT_WS_ROOT/" "$TRAIN_USER@$TRAIN_HOST:$REMOTE_WS_ROOT/"
```

服务器需 **Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11**。旧的 Noetic→Humble 系统迁移脚本会改写 apt 源、卸载系统包并修改 shell 配置，已作为历史维护资料归档；它不是当前工作区的安装命令。请使用经过确认的 Ubuntu 22.04/Humble 环境镜像或学校／实验室的系统部署流程，先由管理员完成系统层准备，再执行下面的工作区步骤。

### 8.3 服务器上训（Humble · 命令顺序）

```bash
# 0) 路径（远端；若未设置则先按自己的真实绝对路径设置）
# export SCOUT_WS_ROOT=/训练机上的/catkin_ws_2/绝对路径

# 1) 系统层前提（先确认，勿用工作区脚本改写 apt 源或卸载系统包）
ros2 --version
gazebo --version

# 2) Python 依赖
python3 -m pip install --user -r $SCOUT_WS_ROOT/3D/scripts/requirements-ros2.txt

# 3) 编译
source /opt/ros/humble/setup.bash
cd $SCOUT_WS_ROOT/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# 4) 可选：冒烟
bash $SCOUT_WS_ROOT/3D/scripts/humble_train_smoke.sh --no-build

# 5) 无头长跑（无 RViz；当前 yaml 的 viz_eval_enable=true，会按配置做定轨评测）
# 若有旧任务，只回到自己启动它的终端 Ctrl+C；端口检查只读：
ss -ltnp | rg ':11345\b' || true
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false
```

阶段 A **不要**在服务器上先跑 SLAM / 拷 `map.yaml` / 开 RViz 录制。

### 8.4 把权重拉回本机

```bash
# 在服务器上确认
ls $SCOUT_WS_ROOT/3D/runs/run_*/td3_actor

# 本机拉取（示例；所有变量为上文占位符）
mkdir -p $SCOUT_WS_ROOT/3D/runs
scp -P "$SSH_PORT" -r "$TRAIN_USER@$TRAIN_HOST:$REMOTE_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS" \
  "$SCOUT_WS_ROOT/3D/runs/"
```

然后按 §9 本地 `infer_td3.launch`，`weights:=$SCOUT_WS_ROOT/3D/runs/run_.../td3`。

---

<a id="ch9"></a>

## 9. 推理与真车部署

> **控制落地**：推理发布的仍是 `/cmd_vel` 的 `(v,ω)`；仿真里由 Gazebo `diff_drive`、真机上由 `scout_base` 转成四轮转速。详解见 [`README3.md` §9](README3.md#ch9)。

### 9.1 权重路径怎么填

1. 找到训练产物目录，例如：
   `$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/`
2. 确认存在 `td3_actor` 与 `td3_critic`
3. launch 参数 **`weights`** = 该目录 + `/td3`（前缀，无 `_actor`）

正确示例：

```bash
weights:=$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/td3
```

错误示例：

```text
.../td3_actor          # 错：多了 _actor
.../td3_actor.pth      # 错：本仓库不带扩展名
.../run_xxx/           # 错：缺前缀名 td3
```

### 9.2 一键推理（自带 Gazebo + 扇区）

```bash
source /opt/ros/humble/setup.bash
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash

RUN=$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS

ros2 launch scout_offroad_3d infer_td3.launch.py \
  gui:=true \
  weights:="${RUN}/td3"
```

`infer_td3.launch.py` 默认 `start_sim:=true`：会拉起 `gazebo_room.launch.py` + `range_features.launch.py`。

**颠簸越野 `bumpy_offroad`：**

```bash
ros2 launch scout_offroad_3d infer_td3.launch.py gui:=true \
  weights:="${RUN}/td3" spawn_z:=1.05 \
  world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world
```

若仿真已在跑，只起推理节点：

```bash
ros2 launch scout_offroad_3d infer_td3.launch.py \
  start_sim:=false weights:="${RUN}/td3"
```

推理默认与训练一致：`allow_reverse:=true`、`max_v:=2.3`、`max_w:=1.8`、`hidden_dim:=1024`、`n_elevation_bins_td3:=4`、`n_sectors:=36`（launch 默认值已同步；加载旧维数权重会失败）。

### 9.3 发布目标（否则车不动）

推理节点订阅 **`/offroad/train_goal`**（`geometry_msgs/Point`）。未收到目标时节点空转、不发 `/cmd_vel`。launch 默认 `auto_goal:=true`，约 2 s 后自动设 (3, 2)。

另开终端手动发目标：

```bash
ros2 topic pub --once /offroad/train_goal geometry_msgs/Point "{x: 2.0, y: 1.5, z: 0.0}"
```

自检：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic hz /offroad/td3_range_features
```

说明：阶段 A 推理目标话题与训练环境内部目标一致命名；**块2 的 `/offroad/local_goal` 尚未对接**，勿指望 RViz SetGoal / Arena。

### 9.4 真车部署（阶段 A 局部策略 · 非「一键上车」）

**诚实边界**：当前是 **阶段 A**——Gazebo 里训出的 **局部避障 + 朝目标点走** 策略。真车上 **没有** Arena 中间层、SLAM、A\* 全局规划；只能把仿真权重当 **局部控制器** 试跑，**必须**有人监护 + 安全层，不能当量产导航栈。

#### 9.4.1 拷哪些文件到工控机

从训练 run 目录（例 `$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS/`）拷到真车同路径或任意目录：

| 文件 | 是否必需 | 说明 |
|------|----------|------|
| `td3_actor` | **必需** | Actor 权重 |
| `td3_critic` | **必需** | 加载时与 actor 成对校验 |
| `config.yaml` | **强烈建议** | 核对 `n_sectors=36`、`n_elevation_bins_td3=4`、`hidden_dim=1024`、`use_imu_in_state`、`max_range=5.0`、`max_v`/`max_w` 与真车扇区节点一致 |

**不必拷**：`td3_*_optimizer`、`train.log`、`metrics.csv`、`*.png`（分析/续训用）。

工控机还需 **已编译并 source 的** `scout_offroad_3d` 包（含 `infer_td3_node.py`、`td3/`、`range_features_node.py`）及 **PyTorch**（`python3 -m pip install --user torch`）。

示例 scp（在本机执行，按账号改）：

```bash
RUN=run_YYYYMMDD_HHMMSS
scp $SCOUT_WS_ROOT/3D/runs/$RUN/{td3_actor,td3_critic,config.yaml} \
  user@robot-pc:$SCOUT_WS_ROOT/3D/runs/$RUN/
```

#### 9.4.2 真车上跑推理

**前置（缺一不可）**：

1. 底盘驱动 + `/cmd_vel` 订阅正常（Scout 官方栈或自研驱动）。
2. **2D 激光** `/scan`（或 **16 线** `/lidar/points`）有数据；**必须先有** `/offroad/td3_range_features`：
   ```bash
   ros2 launch scout_offroad_3d range_features.launch.py
   # 真车 16 线：cloud_topic:=/your_lidar cloud_msg_type:=pointcloud2 frame_mode:=base
   # 真车 2D scan：source:=scan scan_topic:=/your_scan
   ```
3. **里程计** `/odom`（或 LIO 发布的 odom）在跳。
4. 有人遥控急停 / 物理急停可用。

**启动推理**（不启 Gazebo）：

```bash
cd $SCOUT_WS_ROOT && source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash

RUN=$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS

ros2 launch scout_offroad_3d infer_td3.launch.py \
  start_sim:=false \
  pose_source:=odom \
  weights:=$RUN/td3
```

发目标（世界系 `x,y`，与训练时 Gazebo world 同理；阶段 B 才换 `/offroad/local_goal`）：

```bash
ros2 topic pub --once /offroad/train_goal geometry_msgs/Point "{x: 3.0, y: 1.0, z: 0.0}"
```

#### 9.4.3 话题对齐：仿真 vs 真车

| 用途 | 仿真（train/infer 默认） | 真车 |
|------|--------------------------|------|
| 速度输出 | `/cmd_vel` | `/cmd_vel`（或 launch `cmd_vel_topic:=`） |
| 原始感知 | `/lidar/points`（16 线）或 `/scan`（2D 回退） | 车载多线 → `cloud_topic:=`；2D → `source:=scan` |
| 扇区特征（TD3） | `/offroad/td3_range_features`（4×36=144 维） | **同上**；须先跑 `range_features.launch` |
| 水平投影（碰撞） | `/offroad/range_features`（36 维） | 同上 |
| 位姿 | Gazebo `get_model_state` | `/odom`（`pose_source:=odom`） |
| 目标 | `/offroad/train_goal` | 同上；块2 后为 `/offroad/local_goal`（**尚未接**） |

自检：

```bash
ros2 topic hz /scan /odom /offroad/td3_range_features
ros2 topic echo /offroad/td3_range_features --once   # data 长度应为 144
ros2 topic echo /cmd_vel --once
```

#### 9.4.4 安全层建议（必做）

- **速度上限**：推理节点已 clip `v∈[-max_v,max_v]`（`allow_reverse: true`；须与训练 `config.yaml` 一致）；真车可再包一层 `twist_mux` / 限速节点。真车建议用 `config/real_robot_scout.yaml`（`allow_reverse: false`、`max_v: 0.3`）起步。
- **急停**：遥控或硬件 E-stop 能 **立即切断** `/cmd_vel`。
- **障碍兜底**：扇区全近距 / `/scan` 断流时 **禁止** 继续发速度（当前节点未内置，需外层 watchdog）。
- **场地**：开阔、低速、有人跟车；勿直接上复杂越野或公共道路。
- **sim-to-real 差距**：Gazebo 2D scan ≠ 真车多线/振动/标定误差；先短距离、小目标验证。

#### 9.4.5 上车前 Checklist（逐项打勾）

- [ ] 工控机 `scout_offroad_3d` 已 colcon build --symlink-install + source
- [ ] `python3 -c "import torch"` 无报错
- [ ] `td3_actor`、`td3_critic`、`config.yaml` 已拷到工控机
- [ ] `config.yaml` 中 `state_dim=150`、`n_sectors`/`n_elevation_bins_td3=4`/`hidden_dim=1024`/`max_range`/`max_v`/`max_w` 与 launch 一致
- [ ] `/lidar/points` 正常 → `range_features.launch` 已启 → `/offroad/td3_range_features` **144** 维在跳
- [ ] `/odom` 正常；`infer_td3.launch start_sim:=false pose_source:=odom`
- [ ] 已发 `/offroad/train_goal`（近处小目标试跑）
- [ ] 急停 / 限速 / 人工监护就绪
- [ ] **未**指望 SLAM、地图、Arena、一键全局导航

---

<a id="ch10"></a>

## 10. 常见故障

| 现象 | 处理 |
|------|------|
| `Address already in use` / Gazebo 起不来 | 按 §5.1 回到自己启动的 launch 终端 `Ctrl+C`，再用只读 `ss` / `pgrep` 确认；不要终止陌生任务 |
| spawn 失败 / 车不出现 | 加大 `spawn_delay:=12`；确认用的是 `gazebo_room_official`（一键 launch 已带）；清残留后重开 |
| 无 `/offroad/td3_range_features` | 确认 `/lidar/points` 有数据：`ros2 topic hz /lidar/points`；确认 `range_features` 在跑；2D 回退：`source:=scan` |
| `PyTorch not found` / `ImportError: torch` | `python3 -m pip install --user torch`；用 **同一** `python3` 验证 `import torch` |
| `package scout_offroad_3d not found` | 该终端未 `source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash`，或未 `colcon build --symlink-install` |
| 推理立刻报 `weights_prefix required` | 必须传 `weights:=.../td3` |
| 推理加载失败 / 文件找不到 | 检查 `ls $RUN/td3_actor $RUN/td3_critic`；`weights` 不要带 `_actor` |
| 推理加载报 shape/size mismatch | 权重维数与 launch 不一致：旧权重（如 R8 的 768/112、更早的 512/76）不能配现网默认（**1024 / 150**）；用同一次 run 的 `config.yaml` 核对 |
| 推理无运动 | 是否发了 `/offroad/train_goal`；是否有 `/cmd_vel`；Gazebo 里车是否卡死 |
| `int('')` / `train_td3_node` 立刻崩溃 | launch 曾用空 `max_timesteps:=` 覆盖 yaml；现仅 CLI 显式传参才覆盖；见 DEBUG 记录 29 |
| 续训立刻结束 / 仍从 ep=0 | `max_timesteps` 必须 **>** 当前 t；必须 `output_dir:=` 指向已有 run；日志应有 `[resume] loaded weights ... t=... ep=...` |
| 续训 spawn `Connection refused` / `pose_unreadable` | 确认自己启动的旧 launch 已 `Ctrl+C` 退出；再用 §5.1 的只读检查确认端口状态。节点会对它自己管理的 world-server 做受控重启后再 reset |
| 短冒烟没有 `td3_actor` | 极短中断可能未跑到最终 `save`；再跑一次并让节点正常跑完，或把 `max_timesteps` 调到 ≥ 几百并等结束日志 `Training finished` |

---

<a id="ch11"></a>

## 11. 选型拍板（建图黑盒 + 粗全局规划 · 与块1/块2一致）

与 [`README0`](README0.md) 一致：**块1=TD3（阶段 A 现在）**；**块2=SLAM+Arena+粗规划（B/C）**。下列选型**不阻塞**当前训练。

### 11.1 建图 / 里程计黑盒推荐表

口令对照：kiss leo → **KISS-ICP**；fast leo → **FAST-LIO2**；leo slam → **LIO-SAM**。

| 场景 | 传感器现实 | **推荐** | 角色 | 原因 / 限制 |
|------|------------|----------|------|-------------|
| **仿真现阶段** | 有 `/imu/data`，**16 线** `/lidar/points` + 深度相机 | **不上 LIO 作主链** | — | 阶段 A 训 TD3 用 Gazebo `/odom` + 扇区投影即可 |
| 仿真旁路试里程计 | 16 线点云或深度点云 | **KISS-ICP** | 可选对照 | 块2 旁路；不阻塞 TD3 |
| **真车越野**（Helios 等多线 + 9 轴 IMU） | 多线 PointCloud2 + IMU | **FAST-LIO2 作主** | 主位姿/建图黑盒 | 实时性与工程落地常见；外参标定后作默认 |
| 真车对照 | 同上 | **LIO-SAM** | 对照 / ablation | 因子图+回环/GPS 友好；论文对照与大场景建图 |
| 真车轻量对照 | 点云为主 | **KISS-ICP** | 轻量对照 | 无紧耦合 IMU 时的保底里程计 |

**阶段接入**：A 不用；C 再开黑盒。勿在 `train_td3.launch` 里绑 LIO。

### 11.2 粗全局规划

| 场景 | 推荐 | 何时接入 |
|------|------|----------|
| **室内仿真**（现有 room / 2D 栅格） | 继续现有 **2D A\***（`scout_mini_pro_description` 导航栈） | 阶段 C 对照；**阶段 A 不启** |
| **越野 / 非平面可通行性** | **可通行性代价地图 + Hybrid A\***（或同类 kinodynamic 全局） | 阶段 C；输出粗路径给 Arena 中间层 |
| **Arena 局部子目标** | 沿粗路径 spatial horizon 取点 → `/offroad/local_goal` | **阶段 B**（可先直线前瞻，再完整 Arena） |

结论一句话：**A 只训 TD3；B 接 local_goal；C 才上 A\*/Hybrid A\* 与建图黑盒。**

### 11.3 Arena 子目标（再次强调）

中间规划器：全局路径 + 车位姿 + look-ahead → **动态 local goal** → TD3 的 $(\rho,\theta)$。
**现在阶段 A 没有**：训练用随机目标；推理发 `/offroad/train_goal`。B 再对齐 `/offroad/local_goal`。

IMU：`ros2 topic echo /imu/data --once`（需重开官方仿真加载 URDF）。**当前 Record 9 TD3 默认将 roll/pitch 写入 150 维 state**（`use_imu_in_state: true`），并同时用于坡/墙与翻车判据；IMU 静默时仅按代码的 model-state fallback 诊断，不应把它误写成“训练不依赖 IMU”。

### 11.4 代码从哪抄的、改了什么

详见 §1.4。摘要：

| 来源 | 用途 | Scout 改编 |
|------|------|------------|
| [sfujim/TD3](https://github.com/sfujim/TD3) MIT | Actor / twin Critic / replay / delayed update | `max_action` 向量 `(v_max, ω_max)` |
| DRL-AUV-navigation 类 ROS 环 | episode reset / step / cmd | 差速 `/cmd_vel`；状态用扇区+ρθ；模型名 `scout_mini` |
| Arena-Rosnav | 分层 local goal | **未接**；阶段 B/C |

包内溯源：`scout_offroad_3d/td3/NOTICE.md`、`LICENSE_sfujim_TD3`。

```text
scout_offroad_3d/td3/TD3.py, replay_buffer.py, ros_env.py
scout_offroad_3d/train_td3_node.py, infer_td3_node.py, plot_td3_metrics.py, pick_random_world.py
launch/train_td3.launch.py, infer_td3.launch.py, range_features.launch.py
config/td3_train.yaml
worlds/*.world
```

---

<a id="ch12"></a>

## 12. 还差什么（诚实）

骨架可启训练环并导出权重。已有：启动时随机第一张训练池图、`world:=` 定第一张、`world_switch_interval` 按步换图、`pick_random_world.py`、`metrics.csv` + 训完自动出图、**恰好 4 张** bumpy heightmap、**150 维 TD3 状态**（4 仰角 × 36 扇区 + ρθvw + IMU roll/pitch）、训练雷达 **16×360**、**对称倒车**、**高/低努力奖励**、**成功率余弦退火**（含爬坡 $s_{\mathrm{climb}}$ 0.25→0.0005）、Record 9 奖惩（arrive 700 / stuck·rollover −160）。短验证的输出确认后会移入归档，避免把活动 `3D/runs/` 堆满。
仍缺：推理目标与 B 的 `local_goal` 对齐；Record 8 仅为历史长训口径（112/768），任何当前训练都须用 Record 9 参数和全新目录，勿加载旧维度权重。
**已有**：前方扇区碰撞防抖 + 受困终止 + **坡/墙多层判别**（记录 14）+ 碰撞不硬 reset（记录 12）。
**未做**：Arena、A\*、SLAM、完整分层栈；完整 4 层 elevation 已进入当前 TD3 state（150 维）。

调参速查见 §4。
