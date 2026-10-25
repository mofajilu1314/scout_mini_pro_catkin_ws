# 3D · README3 — 控制链路、公式与 ScoutNav 仿真操作手册（公开版）

> 本文是 3D 导航链的**完整公开操作说明**：保留状态构成、控制公式、节点职责、GUI 按钮流程、话题关系、验证标准与排障证据链。它不是一个只放入口的索引页。
>
> 本文覆盖 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 的**隔离仿真工作流**。默认模型 Record 9 是仿真基线，不是物理平台的安全认证。
>
> 受控现场资料：仅限本地 `archive/private/README3.md` 保存；该文件不属于公开导出，也不是本文任何仿真步骤的前置条件。

---

## 目录

1. [阅读边界与当前基线](#ch1)
2. [工作区、源码和运行产物的关系](#ch2)
3. [端到端控制链路：从点云到四轮运动](#ch3)
4. [状态向量、Actor 动作与兼容性公式](#ch4)
5. [Gazebo 差速驱动的轮速公式与数值例子](#ch5)
6. [TD3 训练、短验证和推理的操作链](#ch6)
7. [ScoutNav 长导航的地图、安全边界与话题链](#ch7)
8. [ScoutNav 图形界面：逐按钮仿真流程](#ch8)
9. [RViz 操作、到点行为与安全边界反馈](#ch9)
10. [命令行备用流程与安全停机方式](#ch10)
11. [排障表、验证证据和产物归档规则](#ch11)
12. [源码定位与公开交付边界](#ch12)

---

<a id="ch1"></a>

## 1. 阅读边界与当前基线

### 1.1 本文解决什么问题

这条 3D 主线分成三个互相衔接、但不能混为一谈的层次：

```text
TD3 训练层
  多层点云特征 + 目标相对量 + 历史动作 + 姿态
        │
        ▼
局部控制层
  Actor 输出 [v, ω] → geometry_msgs/Twist → /cmd_vel
        │
        ▼
长导航层
  地图 / 安全边界 / 全局路径 → 局部目标 → TD3 → /cmd_vel
        │
        ▼
Gazebo 车辆层
  差速插件 → 左右侧轮速 → 四轮关节 → /odom / TF / 点云
```

- **TD3 训练层**负责学习从状态 `s_t` 到动作 `a_t=[v,ω]` 的策略，不直接计算单个电机的电流或力矩。
- **局部控制层**把 Actor 的两个输出填进 `Twist.linear.x` 与 `Twist.angular.z`；它的接口终点是 `/cmd_vel`。
- **长导航层**负责地图、边界、目标校验和局部目标的衔接；它不替代 Actor 的局部避障/越野控制语义。
- **Gazebo 车辆层**把差速速度转换成左右轮、前后轮关节速度，并由物理引擎推进位姿。

### 1.2 当前公开仿真基线

| 项目 | 当前说明 | 证据位置 |
|---|---|---|
| ROS 版本 | ROS 2 Humble | `/opt/ros/humble` |
| 仿真器 | Gazebo Classic + RViz2 | 训练 launch 与 ScoutNav GUI |
| 默认长导航模型 | Record 9，`state_dim=150`、`hidden_dim=1024` | `3D/scout_nav_app/config/models.yaml` |
| 状态特征 | 4 层 × 36 扇区，外加目标、历史动作和 roll/pitch | `scout_offroad_3d/td3/ros_env.py` |
| Actor 输出 | 2 维 `[v, ω]` | `scout_offroad_3d/td3/TD3.py` |
| 地图输出 | `3D/maps_long/` | `scout_long_nav` 建图流程 |
| 加边界后的地图输出 | `3D/maps_security/` | ScoutNav 边界保存流程 |

**最重要的兼容性原则**：权重不能只看文件名。加载某个 run 前，必须同时核对该 run 的 `config.yaml`、`params_backup.yaml`、`state_dim`、`hidden_dim`、`action_dim` 与 Actor/critic 的完整 checkpoint 前缀。不同状态维度或网络宽度的权重不能互换加载。

### 1.3 两类“能跑”的含义不同

| 级别 | 目标 | 合格信号 | 不代表什么 |
|---|---|---|---|
| 构建检查 | 包可发现、Python/launch 可解析 | `colcon build` 成功 | 不代表策略已收敛 |
| 短验证 | 训练节点、日志、权重写出链路完整 | `SMOKE PASS`，产物齐全 | 不代表导航效果好 |
| 仿真推理 | 权重、场景、目标、话题链匹配 | 小车在 Gazebo 中按目标运动 | 不代表所有场景都安全 |
| 长导航 | 地图、边界、规划与局部策略协同 | 能接受合法目标、拒绝越界目标 | 不代表物理平台已验收 |

---

<a id="ch2"></a>

## 2. 工作区、源码和运行产物的关系

### 2.1 唯一活动工作区

所有命令从工作区根变量开始，不把任何个人目录写死到脚本或说明中：

```bash
export SCOUT_WS_ROOT=/absolute/path/to/catkin_ws_2
source /opt/ros/humble/setup.bash

cd "$SCOUT_WS_ROOT/ros2_ws"
colcon build --symlink-install --packages-select \
  scout_workspace_common \
  scout_description \
  scout_mini_pro_description \
  scout_offroad_3d \
  scout_long_nav
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"
```

`SCOUT_WS_ROOT` 是首选变量；旧 shell 若使用 `CATKIN_WS` 或 `WS_ROOT`，共享路径解析也会兼容，但新命令应优先使用 `SCOUT_WS_ROOT`。

### 2.2 目录职责表

| 路径 | 角色 | 可以放什么 | 不应放什么 |
|---|---|---|---|
| `ros2_ws/src/shared/` | 共享描述、模型与工作区路径工具 | 共享包 | 训练 run、临时日志 |
| `ros2_ws/src/navigation_3d/scout_offroad_3d/` | TD3 训练、推理、特征与 Gazebo 场景 | launch、config、Python、world、模型资源 | 长期验证输出 |
| `ros2_ws/src/navigation_3d/scout_long_nav/` | 建图、边界、规划、局部目标桥接 | launch、地图算法、RViz 配置、测试 | 模型权重副本 |
| `3D/scout_nav_app/` | ScoutNav 图形界面 | GUI 源码、图标、配置、单元测试 | 运行日志、缓存 |
| `3D/maps_long/` | 未加安全边界的可通行性图 | `.yaml`、`.pgm`、明确命名的地图 | 临时截图或训练权重 |
| `3D/maps_security/` | 已烘焙边界的安全图 | 地图与边界描述 | 未校验的草稿图 |
| `3D/runs/` | 当前正在进行的训练 | 当前 `run_*` | 已完成的 smoke、历史 run |
| `3D/debug_logs/` | 当前问题排查期间的诊断记录 | 正在排查的日志 | 长期积累的旧验证物 |
| `archive/` | 可追溯历史、旧文件与验证证据 | 已结束实验、旧日志、迁移清单 | 当前命令的唯一依赖 |

### 2.3 运行产物最小合同

一个可复查的 TD3 run 至少应保留下列内容；缺任何一项，都不能把它当作可恢复训练或可比较实验的完整记录：

```text
run_*/
├── config.yaml                 # 有效配置快照
├── params_backup.yaml          # 参数备份
├── metrics.csv                 # 每回合或阶段指标
├── train.log                   # 训练过程与报错上下文
├── td3_actor                   # Actor 权重
├── td3_actor_optimizer         # Actor 优化器状态
├── td3_critic                  # Critic 权重
└── td3_critic_optimizer        # Critic 优化器状态
```

短验证产生的 `smoke_*` 与普通 run 有不同的用途：前者验证“路径、launch、节点、保存链能否工作”，后者才可能是实验结果。短验证完成后应移动到 `archive/validation/`，不要与当前训练 run 混在一起。

---

<a id="ch3"></a>

## 3. 端到端控制链路：从点云到四轮运动

### 3.1 总体数据流

```text
Gazebo 多线点云 / 姿态 / 里程计 / 目标
             │
             ▼
range_features_node 或 ros_env
  点云按仰角层与水平扇区统计距离
             │
             ▼
状态 s_t = [多层扇区距离, 目标极坐标, 上一步动作, 姿态]
             │
             ▼
TD3 Actor
  a_t = [v, ω]
             │
             ▼
geometry_msgs/Twist
  linear.x = v
  angular.z = ω
             │
             ▼
/cmd_vel
             │
             ▼
gazebo_ros_diff_drive
  左右侧线速度 → 四个轮关节速度
             │
             ▼
Gazebo 物理积分 → /odom、TF、传感器数据
```

这条链的关键边界是：**Actor 输出的是整车的线速度与角速度，不是四个轮子的独立控制量。** 前后同侧轮的同步关系由差速插件保证；因此不应在 TD3 节点里另写一套四轮速度方程并与插件竞争。

### 3.2 模块与源码对应

| 环节 | 当前源码 | 输入 | 输出 |
|---|---|---|---|
| 点云扇区特征 | `scout_offroad_3d/range_features_node.py` | 点云或扫描 | `/offroad/td3_range_features` 等特征话题 |
| 训练环境 | `scout_offroad_3d/td3/ros_env.py` | Gazebo、点云、里程计、目标 | 状态、奖励、`/cmd_vel` |
| TD3 网络 | `scout_offroad_3d/td3/TD3.py` | 状态向量 | 2 维动作 |
| 训练节点 | `scout_offroad_3d/train_td3_node.py` | 环境交互 | checkpoint、metrics、日志 |
| 推理节点 | `scout_offroad_3d/infer_td3_node.py` | 特征、里程计、目标、权重 | `Twist` |
| 长导航规划 | `scout_long_nav/hybrid_astar_node.py` | 地图、边界、`/goal_pose` | 合法的 `/long_nav/goal` |
| 速度门控 | `scout_long_nav/nav_cmd_gate_node.py` | 局部 TD3 速度与规划状态 | 仿真 `/cmd_vel` |
| GUI 编排 | `3D/scout_nav_app/scout_nav_app/` | 按钮、路径、事件 | launch、日志、弹窗 |

### 3.3 话题角色，不是文件名猜测

| 话题 | 主要方向 | 在仿真链中的意义 |
|---|---|---|
| `/cmd_vel` | 控制 → Gazebo | 最终整车速度命令 |
| `/cmd_vel_td3` | Actor → 长导航门控 | 长导航场景中的 TD3 原始输出 |
| `/odom` | Gazebo → 算法 | 车辆在 `odom` 坐标系下的估计位姿 |
| `/goal_pose` | RViz → 规划器 | 操作者设置的候选终点 |
| `/long_nav/goal` | 规划器 → 下游 | 已通过边界/路径检查的终点 |
| `/offroad/local_goal` | 长导航 → TD3 | 给局部策略的近距离目标 |
| `/offroad/train_goal` | 目标桥 → TD3 | TD3 使用的目标表达 |
| `/long_nav/nav_abort` | 规划器 → GUI/门控 | 当前导航必须停止的原因 |
| `/long_nav/goal_rejected` | 规划器 → GUI | 新终点未获接受的原因 |

**单一速度发布者纪律**：同一仿真时间段只应由当前链路中的一个已知流程拥有最终 `/cmd_vel`。建图、训练、推理、长导航不能在同一个 Gazebo 会话中随意叠加；如果不确定哪个流程仍在运行，先回到其启动终端正常停止，再启动下一步。

---

<a id="ch4"></a>

## 4. 状态向量、Actor 动作与兼容性公式

### 4.1 当前 Record 9 状态公式

训练环境和推理节点使用同一状态构成：

```text
state_dim = n_elevation_bins_td3 × n_sectors + 4 + imu_terms
```

对 Record 9：

```text
n_elevation_bins_td3 = 4
n_sectors             = 36
imu_terms             = 2  (roll, pitch)

state_dim = 4 × 36 + 4 + 2 = 150
```

更完整地写成：

```text
s_t = [ d̂₁, …, d̂₁₄₄,
        ρ/10,
        θ/π,
        v_(t-1)/v_max,
        ω_(t-1)/ω_max,
        roll/π,
        pitch/π ]
```

其中：

- `d̂ᵢ = clip(dᵢ / max_range, 0, 1)`：第 `i` 个点云扇区距离归一化值；144 来自 4 个仰角层乘 36 个水平扇区。
- `ρ`：车体到当前目标的平面距离；训练中被截断归一到 `[0,1]`。
- `θ`：以车体朝向为参考的目标夹角，归一为 `θ/π`。
- `v_(t-1)`、`ω_(t-1)`：上一控制周期的速度，帮助策略理解动作惯性。
- `roll`、`pitch`：姿态项；Record 9 中启用，用于区分坡面、墙面和姿态变化。

状态构成来自 `ros_env.py::_build_state()`；推理节点使用同一维度公式。因此“能加载”不是只看权重文件是否存在，而是要让这条公式的每项与训练快照完全一致。

### 4.2 Actor 的动作语义

```text
a_t = Actor(s_t) = [v_t, ω_t]
```

动作进入 `Twist` 的映射固定为：

```text
Twist.linear.x  = v_t       # m/s
Twist.angular.z = ω_t       # rad/s
```

训练与推理都会进行范围裁剪：

```text
v_t ∈ [v_min, v_max]
ω_t ∈ [-ω_max, ω_max]
```

当 `allow_reverse=true` 时，`v_min=-v_max`；当它为 `false` 时，`v_min=0`。当前公开仿真训练配置中的上限来自 run 的有效 `config.yaml`，不应为了“复用权重”而随意替换为另一套数值。

### 4.3 权重兼容性清单

加载一个模型前，按顺序核对：

1. `state_dim`：必须等于运行时状态公式结果。
2. `action_dim`：当前 Actor 是 2，顺序固定为 `[v,ω]`。
3. `hidden_dim`：Actor/Critic 隐层宽度必须与 checkpoint 参数形状一致。
4. `n_sectors`、`n_elevation_bins_td3` 与 `use_imu_in_state`：共同决定状态维度。
5. `max_v`、`max_w`、`allow_reverse`：决定输出缩放与控制语义。
6. 权重前缀：完整保存 Actor、Critic 及两个优化器状态，不能只搬一个 Actor 文件后误判为可续训。

| 常见不匹配 | 会发生什么 | 正确处理 |
|---|---|---|
| 112 维权重加载到 150 维输入 | 第一层矩阵形状不一致 | 使用与权重配套的配置，或重新训练 |
| `hidden_dim=768` 配 1024 宽模型 | checkpoint 参数键形状不一致 | 从权重所属 run 读取配置 |
| 旧权重只有 Actor | 可做有限推理，不能完整恢复训练 | 明确标为推理副本，不作为续训基线 |
| 忘记姿态项 | 运行时状态长度错误 | 显式核对 `use_imu_in_state` |
| 改了动作上限 | 行为分布被重新缩放 | 保留 run 的有效参数快照 |

### 4.4 为什么不直接输出四个轮子

TD3 的研究对象是局部导航策略，而不是底层电机控制器。将策略空间限定为 `[v,ω]` 有三个好处：

1. 训练与推理的接口稳定：Gazebo、长导航和车辆描述都围绕 `Twist` 协作。
2. 轮距、轮径和轮关节映射集中在车辆插件，避免同一几何关系在多个节点重复实现。
3. 奖励函数能直接对“前进、转向、到目标距离、碰撞与卡住”建模，不把硬件实现细节混进策略输出。

---

<a id="ch5"></a>

## 5. Gazebo 差速驱动的轮速公式与数值例子

### 5.1 插件负责的几何变换

当前 Humble 仿真使用官方差速插件 `gazebo_ros_diff_drive`。它接收 `/cmd_vel`，用左右轮距和轮半径把整车命令分解为左右侧轮速；同侧前后轮共享指令。

设：

- `v`：Actor 发出的整车线速度；
- `ω`：Actor 发出的偏航角速度；
- `L`：左右轮距；
- `r`：轮半径；
- `v_L`、`v_R`：左、右侧轮心线速度；
- `ω_L`、`ω_R`：左、右侧车轮角速度。

则差速几何为：

```text
v_L = v - ω × L/2
v_R = v + ω × L/2

ω_L = v_L / r
ω_R = v_R / r
```

仿真车辆配置中使用：

```text
L = 0.58 m
wheel_diameter = 0.16 m
r = 0.08 m
```

### 5.2 数值例子

给定：

```text
v = 0.50 m/s
ω = 0.20 rad/s
L = 0.58 m
r = 0.08 m
```

先求左右侧线速度：

```text
v_L = 0.50 - 0.20 × 0.29 = 0.442 m/s
v_R = 0.50 + 0.20 × 0.29 = 0.558 m/s
```

再求轮角速度：

```text
ω_L = 0.442 / 0.08 ≈ 5.53 rad/s
ω_R = 0.558 / 0.08 ≈ 6.98 rad/s
```

因此：

```text
front_left_wheel 与 rear_left_wheel  → 约 5.53 rad/s
front_right_wheel 与 rear_right_wheel → 约 6.98 rad/s
```

最后由 Gazebo 的物理引擎积分车辆姿态并发布 `/odom`。本栈中里程计到 TF 的职责被明确分离，避免同一 TF 被多处重复发布。

### 5.3 需要检查的插件参数

若出现“速度话题存在但车不动”“只转不走”“轮子方向相反”等问题，先检查模型/插件参数，而不要先怀疑 TD3：

| 参数类别 | 应检查什么 |
|---|---|
| 关节映射 | 左右前后四个轮关节是否被正确列入插件 |
| `num_wheel_pairs` | 是否与双轴四轮结构一致 |
| `wheel_separation` | 左右轮距是否与模型几何一致 |
| `wheel_diameter` | 轮径是否与模型尺度一致 |
| `/cmd_vel` remap | 插件订阅名是否真的是当前流程发布的 `/cmd_vel` |
| 关节轴方向 | 左右轮正方向是否一致；反向会导致原地异常旋转 |
| TF/odom | 是否只有约定节点发布相应变换 |

这些参数属于车辆仿真模型，不应通过修改 Actor 的动作含义去“补偿”。

---

<a id="ch6"></a>

## 6. TD3 训练、短验证和推理的操作链

### 6.1 先构建，再运行

在第一次运行、更新 Python 包或移动源码后，先执行第 2.1 节的构建命令。构建成功后，所有终端都应重新执行：

```bash
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"
```

避免只 source 系统 ROS 环境而漏掉当前工作区；那会导致 launch 能存在、Python 包却仍加载旧版本的错觉。

### 6.2 短验证：只验证链路，不产出研究结论

下面命令执行很短的训练冒烟验证，目的只是检查 launch、环境、模型、日志和 checkpoint 的写出链是否完整：

```bash
bash "$SCOUT_WS_ROOT/3D/scripts/humble_train_smoke.sh" --no-build
```

合格时应看到 `SMOKE PASS`，并在一个唯一的 `smoke_*` 目录中出现：

```text
train.log
metrics.csv
config.yaml
params_backup.yaml
td3_actor
td3_actor_optimizer
td3_critic
td3_critic_optimizer
```

短验证不等价于策略收敛。确认结果后，应把整目录连同命令、日期和结论移动至 `archive/validation/`；不要把这些仅用于验证的图、日志和权重长期堆在 `3D/runs/`。

### 6.3 新训练：默认新建唯一 run

```bash
ros2 launch scout_offroad_3d train_td3.launch.py gui:=false
```

训练器默认应新建唯一的 `run_*` 输出目录。一个非空目录不应被静默覆盖，也不应因为发现旧权重就自动续训。

新训练前确认：

1. 当前没有自己启动的训练、Gazebo 或长导航进程占用场景。
2. 这次任务确实要做训练，而不是只做构建或接口检查。
3. 输出目录是本次独有的 run，而不是历史实验目录。
4. 日志会记录配置快照、状态维度、动作上限和场景信息。

### 6.4 续训：必须明确、可验证

续训不是“把旧权重放在目录里即可”。只有显式传入 `resume:=true` 并使用配套的输出目录时，才允许恢复；恢复前至少核对：

```text
权重前缀完整
metrics 与参数快照存在
state_dim 一致
action_dim 一致
hidden_dim 一致
状态特征开关一致
```

若其中任一项不一致，应该新建 run 而不是硬接。把不匹配的恢复当成“可以跑”会损坏实验可比性。

### 6.5 推理与长导航的关系

纯 TD3 推理和长导航的职责不同：

| 模式 | 谁提供目标 | 谁决定路线 | Actor 的作用 |
|---|---|---|---|
| 单纯推理 | 自动目标或训练目标 | 没有全局路径 | 直接局部控制 |
| 建图 | 仿真操作者的移动输入 | 只采集/生成图 | 不作为导航策略运行 |
| 长导航 | RViz 的目标，先经检查 | Hybrid A* 与边界约束 | 跟随局部目标并避障 |

因此在同一 Gazebo 会话中不能把训练、建图和长导航当成三个可以并排启动的窗口。先结束当前流程，再进入下一流程；这既避免 `/cmd_vel` 竞争，也避免多个 Gazebo/RViz 实例读写同一资源。

---

<a id="ch7"></a>

## 7. ScoutNav 长导航的地图、安全边界与话题链

### 7.1 两种地图目录的语义

| 目录 | 内容 | 什么时候用 |
|---|---|---|
| `3D/maps_long/` | 可通行性 costmap，未加入安全红线 | 刚完成建图、检查原始地图 |
| `3D/maps_security/` | 在 costmap 上烘焙闭合安全边界后的版本 | 需要边界约束的导航 |

使用 `maps_long` 导航时没有红线约束，这是预期行为；使用 `maps_security` 才会把边界规则纳入路径检查。不要因为路径“能走”就误以为边界已经启用。

### 7.2 安全边界的逻辑

边界不是一个只在 GUI 上显示的装饰线。保存安全地图时，边界外或不可通过区域被写入高代价，使规划器不能把它当作正常可走区域。可以用下式概念化：

```text
C_safe(x, y) = C_map(x, y) + C_fence × I_outside(x, y)
```

其中：

- `C_map(x,y)` 是原始可通行性代价；
- `I_outside(x,y)` 是边界外取 1、边界内取 0 的指示函数；
- `C_fence` 是远高于正常可通行代价的惩罚值；
- 规划器因此会拒绝必须越过边界的终点或当前路径。

保存前的覆盖率检查也不是可选装饰。它回答的是：边界内部有多少地图格子确实被建图过程观测到。概念上：

```text
coverage = N_known_inside / N_all_inside
```

当前 GUI 以 **70%** 作为可保存安全地图的最低覆盖率阈值。边界未闭合、自交、覆盖率不足时，保存会被拒绝；应补图或重画边界，而不是绕过检查。

### 7.3 长导航的目标链

```text
RViz 2D Goal Pose
        │
        ▼
/goal_pose
        │  候选终点检查：地图可达性、边界、路径
        ▼
Hybrid A* / goal relay
        │
        ├── 合法 → /long_nav/goal → 局部目标 → TD3
        │
        └── 不合法 → /long_nav/goal_rejected → GUI 弹窗

当前路径若后来触及红线
        │
        ▼
/long_nav/nav_abort → GUI 提示、速度门控停止导航
```

### 7.4 为什么全局规划与 TD3 不冲突

| 问题 | 长导航层 | TD3 层 |
|---|---|---|
| 最终目标能否接受 | 检查地图、边界和连通性 | 不负责 |
| 远距离路线怎么走 | 计算全局路径/局部航点 | 不负责 |
| 眼前障碍和起伏怎么处理 | 提供局部目标与安全规则 | 根据点云和状态输出 `[v,ω]` |
| 最终速度命令 | 只在合格目标和有效状态下放行 | 给出原始局部动作 |

这是一种分层关系：规划给策略一个可追踪的局部目标，策略处理短时观测与地形反馈；两者的接缝在 `/offroad/local_goal`、`/offroad/train_goal` 与速度门控。

---

<a id="ch8"></a>

## 8. ScoutNav 图形界面：逐按钮仿真流程

### 8.1 启动 GUI

第一次使用 GUI 时，按需安装其 Python 依赖：

```bash
python3 -m pip install --user -r "$SCOUT_WS_ROOT/3D/scripts/requirements-ros2.txt"
python3 -m pip install --user -r "$SCOUT_WS_ROOT/3D/scout_nav_app/requirements.txt"
```

随后在已经 source 当前工作区的终端中启动：

```bash
bash "$SCOUT_WS_ROOT/3D/scout_nav_app/run.sh"
```

也可以从应用目录直接执行：

```bash
cd "$SCOUT_WS_ROOT/3D/scout_nav_app"
python3 -m scout_nav_app
```

启动时右侧终端会显示依赖自检。关键项目包括 Python、PySide6、`ros2`、Gazebo、RViz2、`scout_long_nav` 与可选的 Torch。缺任一关键依赖时，先根据自检输出修复环境，不要靠重复点击按钮“试出来”。

### 8.2 界面布局

```text
┌──────────────────────────────── ScoutNav 主窗口 ────────────────────────────────┐
│ 顶栏：模型下拉 · 加载模型 · 依赖自检状态                                        │
├───────────────────────────────┬────────────────────────────────────────────────┤
│ 左侧：仿真流程面板             │ 右侧：终端日志 / 事件告警                       │
│                               │                                                │
│  · Gazebo 世界                │  建图、导航、边界检查、错误原因、成功提示       │
│  · 建图 / costmap             │                                                │
│  · 安全边界                   │                                                │
│  · 导航                       │                                                │
│  · 任务重生与监测             │                                                │
└───────────────────────────────┴────────────────────────────────────────────────┘
```

| 区域 | 按钮或输入 | 作用 | 成功后的证据 |
|---|---|---|---|
| 顶栏 | 模型下拉、加载模型 | 选择并验证权重与维度 | 日志确认模型已加载 |
| 世界 | 世界路径 | 选择 Gazebo 场景 | 路径通过存在性检查 |
| 建图 | 开始建图 | 启动 Gazebo + RViz 的建图流程 | 日志显示 `long_nav_map` 存活 |
| 建图 | 结束建图并保存 costmap | 保存原始可通行性图 | `maps_long/` 有 yaml/pgm |
| 边界 | 加载已有 costmap | 指定 `maps_long` 输入 | 地图被读取 |
| 边界 | 开始设置安全边界 | 打开绘制边界流程 | RViz 可添加顶点 |
| 边界 | 闭合 / 检查覆盖率 / 保存 | 校验并写安全图 | `maps_security/` 生成结果 |
| 导航 | 开始导航 / 停止导航 | 启动或正常结束导航 | 日志显示导航状态 |
| 监测 | 设置重生位等 | 仿真内重新放置车辆 | 旧任务被明确作废 |

### 8.3 标准仿真流程（按顺序执行）

```text
1. 在顶栏选择兼容模型，点击“加载模型”。
2. 检查世界路径；默认大图为 bumpy_offroad_full.world。
3. 点击“开始建图”，等待 Gazebo 与 RViz 都出现。
4. 在仿真中完成扫图后，点击“结束建图并保存 costmap”。
5. 点击“加载已有 costmap”，选择 maps_long 中的结果。
6. 点击“开始设置安全边界”。
7. 在 RViz 用 Publish Point 依次标出边界顶点，再在 GUI 中闭合边界。
8. 点击“检查安全边界内覆盖率”；达到阈值后保存安全 costmap。
9. 在导航区将 costmap 设置为 maps_security，再点击“开始导航”。
10. 在 RViz 用 2D Goal Pose 设置终点，观察规划、局部目标与小车运动。
11. 到点或停止导航后，确认日志中有明确结束结果，再开始下一个任务。
```

不要把第 3 步和第 9 步并行执行。建图流程与导航流程各自拥有仿真资源；一个流程未结束时直接启动另一个，GUI 会提示占用或 launch 很快退出。

### 8.4 世界、模型与地图路径

| 对象 | 推荐来源 | 说明 |
|---|---|---|
| 默认世界 | `ros2_ws/src/navigation_3d/scout_offroad_3d/arena/bumpy_offroad_full.world` | 长导航默认大场景 |
| 原始地图 | `$SCOUT_WS_ROOT/3D/maps_long/` | 不含安全边界 |
| 安全地图 | `$SCOUT_WS_ROOT/3D/maps_security/` | 含边界高代价 |
| 训练权重 | 某个完整 `run_*` 或记录基线 | 必须带兼容配置 |
| GUI 模型登记 | `3D/scout_nav_app/config/models.yaml` | 标明状态维度、网络宽度与基线路径 |

路径不能靠复制粘贴旧的个人目录。若工作区位置变化，应更新 `SCOUT_WS_ROOT` 并让共享路径解析从当前 checkout 推导其余位置。

---

<a id="ch9"></a>

## 9. RViz 操作、到点行为与安全边界反馈

### 9.1 三个关键工具

| RViz 工具 | 使用时机 | 输入的含义 | 预期结果 |
|---|---|---|---|
| Publish Point | 画安全边界 | 一个边界顶点 | GUI 收集多边形顶点 |
| 2D Pose Estimate | 重生或调整初始姿态 | 车辆初始位姿 | 当前导航任务作废，需重新设目标 |
| 2D Goal Pose | 导航时指定终点 | 候选终点姿态 | 先检查，再接受或拒绝 |

RViz 的 Fixed Frame 应遵循当前 launch 的配置，长导航流程通常使用 `odom`。如果 Fixed Frame 不对，点出的目标与边界会看似“落在错误位置”，这属于坐标系问题而不是规划器问题。

### 9.2 到点、切换终点与拒绝行为

| 情况 | 软件行为 | 应如何理解 |
|---|---|---|
| 到达目标容差内 | 清除本次红色路径、停止导航、日志记录成功 | 当前任务完成，可设置新目标 |
| 新终点可达且不越界 | 切换到新目标 | 合法的任务更新 |
| 新终点必须穿越红线 | 发布拒绝事件、GUI 弹窗 | 不是 GUI 崩溃，而是约束生效 |
| 当前路径后来无法保持安全 | 发布中止事件、停止导航 | 应换目标、改起点或修改边界/地图 |
| 操作者取消或重生 | 当前任务作废 | 不应把它误记为“穿线失败” |

### 9.3 常见弹窗/日志的解释

| 现象 | 典型原因 | 处理顺序 |
|---|---|---|
| 依赖自检失败 | Python、ROS、Gazebo、RViz 或包未准备 | 阅读右侧缺失项，修复后重开 GUI |
| Gazebo / RViz 正在使用 | 上一个流程仍持有资源 | 回到启动它的终端正常结束，再重试 |
| `long_nav_map` 很快退出 | 场景冲突、构建未 source、已有仿真 | 先检查终端末尾日志和资源占用 |
| 未加载模型 | 未完成顶栏模型校验 | 先加载匹配权重 |
| 没有 costmap | 未建图或路径为空 | 先保存/加载 `maps_long` |
| 边界未闭合、自交或覆盖不足 | 多边形无效或地图信息不够 | 重画或补图，不要强行保存 |
| 新终点被拒绝 | 终点或必经路径触及边界/不可达区 | 换一个合法目标 |
| 导航中止 | 当前路径违背安全约束 | 停下后检查图、边界与起点 |
| 小车不动但无报错 | 没有有效局部目标、门控未放行或仿真未推进 | 依次检查目标、日志、Gazebo 时钟 |

### 9.4 模型加载失败的定位顺序

```text
1. 确认选择的是 run 目录，而不是只选了单个 actor 文件。
2. 确认配置快照存在且可读。
3. 对比 state_dim、hidden_dim、action_dim。
4. 对比 n_sectors、仰角层数和是否带姿态项。
5. 确认权重前缀完整。
6. 仍失败时，保留完整报错和配置副本，到 archive/validation/ 建立记录。
```

---

<a id="ch10"></a>

## 10. 命令行备用流程与安全停机方式

GUI 是日常入口；命令行流程用于理解 GUI 背后实际启动了什么，或在 GUI 故障时做最小诊断。

### 10.1 建图流程

```bash
export SCOUT_WS_ROOT=/absolute/path/to/catkin_ws_2
source /opt/ros/humble/setup.bash
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"

ros2 launch scout_long_nav long_nav_map.launch.py
```

该流程负责启动建图所需的仿真和 RViz。结束时只在**自己启动的这个终端**按 `Ctrl+C`，让 launch 执行自己的收尾和地图写出。不要使用面向全系统的终止命令；那会影响其他工作区、其他用户或尚未归档的任务。

### 10.2 导航流程

在已有地图且建图流程已经完全结束后，使用：

```bash
export MAP_DIR="$SCOUT_WS_ROOT/3D/maps_security"

ros2 launch scout_long_nav long_nav_nav.launch.py \
  map_dir:="$MAP_DIR"
```

若只想检查没有边界的原始地图，可把 `MAP_DIR` 指向 `3D/maps_long`；这会明确关闭红线约束的效果，不应被误当作安全地图测试。

### 10.3 训练流程

```bash
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"
ros2 launch scout_offroad_3d train_td3.launch.py gui:=false
```

正常训练可能持续很久；不要为“确认命令存在”而额外启动一次。短验证请使用第 6.2 节的 smoke 脚本，确认结果后归档其输出。

### 10.4 运行前与结束后的最小检查

运行前：

```text
□ 当前终端已 source ROS 2 与本工作区 install。
□ 没有同模块遗留的 Gazebo、RViz、训练或长导航会话。
□ 世界、权重、地图路径存在。
□ 本次是训练、建图、导航还是短验证，目标明确且只选一种。
```

结束后：

```text
□ 当前终端已显示 launch/训练正常结束或明确失败原因。
□ run 产物是否完整已检查。
□ smoke、图表、临时日志是否已移入 archive/validation/。
□ 对本次失败/成功的命令、环境和结论是否已记录。
```

---

<a id="ch11"></a>

## 11. 排障表、验证证据和产物归档规则

### 11.1 快速排障表

| 现象 | 优先检查 | 不要做什么 |
|---|---|---|
| `ros2 launch` 找不到包 | 是否 source 当前 `install/setup.bash`，是否刚完成构建 | 不要复制源码到别处绕过构建 |
| TD3 权重加载报形状错误 | `state_dim`、`hidden_dim`、仰角层、姿态项 | 不要手改 checkpoint 张量形状 |
| 小车原地抖动 | 动作上限、轮关节方向、时钟、局部目标 | 不要先用大幅修改奖励掩盖模型问题 |
| 仅一侧轮动 | 模型关节映射、插件参数 | 不要把 Actor 改成四输出 |
| 地图保存失败 | 输出路径、边界是否闭合、覆盖率 | 不要把半成品覆盖到已有安全地图 |
| 导航一开始就退出 | 建图流程是否仍占 Gazebo，地图是否完整 | 不要同时启动多个 Gazebo 会话 |
| 目标总被拒绝 | 目标是否越界、地图是否连通、边界是否正确 | 不要关闭边界检查来伪造成功 |
| 无 episode 完成 | smoke 步数很短或目标距离较远 | 不要把短验证指标当训练效果 |

### 11.2 一个验证记录应包含什么

每次短验证、launch 检查或地图检验都应单独形成最小证据包：

```text
验证名称与日期
执行命令
运行环境（ROS、工作区、包版本）
通过/失败结论
关键日志或报错摘要
输出目录原始位置
为什么它不再属于活动工作区
```

推荐组织方式：

```text
archive/validation/YYYYMMDD/<scope>/
├── MANIFEST.md
├── command.txt 或命令记录
├── 原始日志
└── 验证生成物
```

`archive/validation/` 的目的不是掩盖失败，也不是马上删除数据；它把历史检查从活动源码、当前训练和地图目录中分离出来，让用户可以先审查、再决定是否删除。

### 11.3 哪些东西应立即归档而不是留在活动目录

| 类型 | 归档时机 | 推荐去向 |
|---|---|---|
| `smoke_*` 训练输出 | 已确认通过或失败后 | `archive/validation/` |
| 临时 plot/诊断图 | 检查结论已写入后 | `archive/validation/` |
| 旧 Gazebo/RViz 诊断日志 | 当前故障已关闭后 | `archive/validation/` |
| 已结束的历史 `run_*` | 有明确实验结论后 | `archive/experiments/td3/` |
| 旧文档、旧接口草稿 | 已建立映射后 | `archive/docs/` |

**不应移动**：正在训练的 run、正在排查的问题日志、当前地图、当前源码、当前环境的 `build/` 与 `install/`。分类不确定时，先写清单，等用户确认后再移动；不能把“清理”理解为删除。

### 11.4 验证通过的最小判据

| 检查 | 通过判据 |
|---|---|
| 包发现 | `colcon list --base-paths "$SCOUT_WS_ROOT/ros2_ws/src"` 能发现五个活动包 |
| 构建 | 目标包构建完成，无导入错误 |
| launch 参数 | `ros2 launch … --show-args` 可解析 |
| 短训练 | 有 `SMOKE PASS` 且八类产物齐全 |
| GUI 启动 | 依赖自检通过，能显示明确错误而非静默退出 |
| 建图 | 原始地图写入 `maps_long` |
| 边界 | 闭合、无自交、覆盖率达标，安全地图写入 `maps_security` |
| 导航 | 合法目标可接受，越界目标被拒绝，终止事件可见 |

---

<a id="ch12"></a>

## 12. 源码定位与公开交付边界

### 12.1 公开版依赖的真实文件

本文引用的所有活动文件都位于当前工作区中：

```text
3D/
├── README1.md                         # TD3 训练主线与算法说明
├── README2.md                         # 结果、图表与验证说明
├── README4.md                         # 长导航与安全边界的补充说明
├── scout_nav_app/
│   ├── README.md
│   ├── run.sh
│   └── scout_nav_app/
│       ├── main_window.py
│       ├── sim_panel.py
│       ├── sim_bridge.py
│       ├── safety_boundary.py
│       ├── coverage_utils.py
│       ├── nav_abort_watch.py
│       └── event_monitor.py
├── maps_long/
├── maps_security/
└── scripts/

ros2_ws/src/
├── shared/
│   ├── scout_workspace_common/
│   ├── scout_description/
│   └── scout_mini_pro_description/
└── navigation_3d/
    ├── scout_offroad_3d/
    │   ├── config/td3_train.yaml
    │   ├── launch/train_td3.launch.py
    │   ├── launch/infer_td3.launch.py
    │   └── scout_offroad_3d/
    │       ├── td3/TD3.py
    │       ├── td3/ros_env.py
    │       ├── train_td3_node.py
    │       ├── infer_td3_node.py
    │       └── range_features_node.py
    └── scout_long_nav/
        ├── launch/long_nav_map.launch.py
        ├── launch/long_nav_nav.launch.py
        └── scout_long_nav/
            ├── hybrid_astar_node.py
            ├── nav_cmd_gate_node.py
            ├── boundary_overlay_node.py
            ├── coverage_supervisor_node.py
            └── goal_bridge_node.py
```

本文不把任何历史私有目录或本机绝对路径作为公开运行依赖。公开导出中的所有操作都应能从当前工作区、`SCOUT_WS_ROOT` 和公开源码本身理解。

### 12.2 文档之间如何分工

| 文档 | 主要内容 |
|---|---|
| `README.md` | 3D 模块总览与入口 |
| `README0.md` | 历史/结构性说明和迁移脉络 |
| `README1.md` | TD3 训练、状态、奖励、参数与算法推导 |
| `README2.md` | 训练结果、可视化、检查方法与实验记录 |
| `README3.md`（本文） | 控制链路、轮速公式、ScoutNav GUI、话题、地图、边界、排障 |
| `README4.md` | 长导航、安全边界、规划与更长流程的补充说明 |

这些文档是互补关系，不是“用一页摘要取代其余细节”的关系。移动或整理其中任一份前，先保留可读原文和文件映射；公式、操作链、故障结论与实验推理都是项目资产。

### 12.3 公开导出前的自检

在推送到公开仓库前，至少确认：

```text
□ README 中没有个人绝对路径、现场连接信息或凭据。
□ README 中没有不存在的文件链接。
□ 训练、建图、导航命令都以 SCOUT_WS_ROOT 为入口。
□ 公开说明保留公式、命令、参数语义、验证标准与排障步骤。
□ 短验证与旧日志已归档，而不是混在源码目录。
□ 受控现场资料仍只保留在本地受控位置，不随公开导出分发。
```

---

## 变更说明（公开版重建）

- 保留并当前化：TD3 `[v,ω]` 控制链、状态维度公式、差速轮速推导、ScoutNav GUI 操作、地图/边界逻辑、话题表、排障和验证证据链。
- 明确排除：任何受控现场接入资料、物理平台操作步骤、地址/账户/凭据和部署细节。
- 所有当前路径均改为 `SCOUT_WS_ROOT` 与 `ros2_ws/src/navigation_3d/...` 结构；本文没有把历史目录当作公开运行依赖。
