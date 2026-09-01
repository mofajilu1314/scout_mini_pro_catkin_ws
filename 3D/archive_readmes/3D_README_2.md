# 3D_README_2：第二阶段目标架构

在 [3D_README.md](3D_README.md) 方向与 [3D_README_1.md](3D_README_1.md) 取舍之上，本文给出**可落地的模块划分**，并说明如何挂在现有 Scout / Noetic 工作空间上。

---

## 一、设计原则

1. **不破坏第一阶段**：`nav_all_in_one.launch` / `README_3` 室内 2D 导航继续可用。  
2. **新能力进新包（推荐）**：例如 `scout_offroad_3d/` 或 `scout_drl_nav/`，避免把 TD3 训练脚本塞进 `astar_nav_node.py`。  
3. **先几何、后学习视觉**：可通行性第一版用深度点云阈值（高度、坡度）；接口预留语义层。  
4. **分层决策**：全局给「往哪一片走」，局部 RL 给「这一秒怎么打方向」。  
5. **仿真优先、真机同话题**：深度 `/…/depth/points`、控制 `/cmd_vel` 或 `/scout_mini_pro/cmd_vel` 保持可 remap。

---

## 二、逻辑架构

```text
                    ┌──────────────────────────────────────┐
                    │           Gazebo 越野世界             │
                    │   Scout + 深度相机 + 激光 + IMU       │
                    └───────────────┬──────────────────────┘
                                    │ 传感器话题
          ┌─────────────────────────┼─────────────────────────┐
          ▼                         ▼                         ▼
   ┌──────────────┐        ┌────────────────┐        ┌────────────────┐
   │ 深度点云预处理 │        │ LIO-SAM（并行） │        │ 里程计 / TF    │
   │ 滤地、扇区距离 │        │ 3D 位姿+点云图  │        │ 第一阶段可复用  │
   └──────┬───────┘        └────────┬───────┘        └────────┬───────┘
          │                         │                         │
          ▼                         ▼                         ▼
   ┌──────────────────────────────────────────────────────────┐
   │              局部可通行性 / 高程层（2.5D Grid）            │
   │     layers: elevation | slope | step | traversability    │
   └────────────────────────────┬─────────────────────────────┘
                                │
              ┌─────────────────┴─────────────────┐
              ▼                                   ▼
   ┌────────────────────┐              ┌────────────────────┐
   │ 全局规划（2.5D）     │              │ TD3 局部策略        │
   │ A* / RRT* on cost  │──引导点/路径──►│ state→(v, ω)       │
   └────────────────────┘              └─────────┬──────────┘
                                                 │
                                                 ▼
                                            /cmd_vel
```

**与第一阶段对照**

| 模块 | 第一阶段 | 第二阶段 |
|------|----------|----------|
| 地图 | `OccupancyGrid` 平面 | 高程 / traversability 多层 grid（2.5D） |
| 定位 | AMCL + 2D 激光 | 仿真可过渡；目标 **LIO-SAM** |
| 全局规划 | 2D A\* | 2.5D A\* / RRT\*（代价=不可通行性） |
| 局部控制 | Pure Pursuit + 手写激光逻辑 | **TD3**（Pure Pursuit 可作 Teacher/基线） |
| 感知 | 激光当墙 | **深度点云几何**（语义后置） |

---

## 三、推荐软件包拆分

```text
src/
├── scout_mini_pro_description/   # 第一阶段：模型、室内导航、本文档
├── scout_description/            # 官方车模
├── scout_gazebo_sim/
├── scout_control/
└── scout_offroad_3d/             # 【建议新建】第二阶段主包
    ├── worlds/                   # 越野 .world
    ├── launch/
    ├── scripts/
    │   ├── depth_to_trav_node.py      # 点云 → 局部可通行性
    │   ├── global_planner_2p5d.py     # 2.5D 规划
    │   └── rl/
    │       ├── train_td3.py
    │       ├── test_td3.py
    │       └── gazebo_env.py          # 对接 Scout 的 Gym 环境
    ├── config/
    └── models/                   # 训练好的 TD3 权重
```

初期也可把脚本暂放在 `scout_mini_pro_description/scripts/offroad_3d/`，但**独立包更清晰**。

---

## 四、关键话题约定（草案）

命名可随方块车 / 官方车 remap；下表以「逻辑名」为准。

| 逻辑名 | 建议话题 | 消息 | 生产者 → 消费者 |
|--------|----------|------|-----------------|
| 深度点云 | `/scout_mini_pro/depth/points` 或 `/camera/depth/points` | `PointCloud2` | Gazebo → 预处理 |
| 局部距离特征 | `/offroad/range_features` | `Float32MultiArray` 或自定义 | 预处理 → TD3 |
| 可通行性格网 | `/offroad/traversability` | `OccupancyGrid` 或 `grid_map` | 预处理 → 全局规划 / 可视化 |
| 机器人位姿 | `/offroad/pose` 或 TF `map→base` | Pose / TF | 里程计或 LIO-SAM → 规划与 RL |
| 全局路径 | `/offroad/global_path` | `nav_msgs/Path` | 2.5D 规划 → RL 或跟踪 |
| 局部目标 | `/offroad/local_goal` | `PoseStamped` | 全局规划 → TD3（极坐标输入） |
| 控制 | `/scout_mini_pro/cmd_vel` 或 `/cmd_vel` | `Twist` | TD3 → Gazebo |

第一阶段的 `/path`、`/move_base_simple/goal` **不要强行混用**；第二阶段用 `/offroad/*` 前缀，避免 RViz SetGoal 打到旧 A\*。

---

## 五、TD3 状态 / 动作 / 奖励（建议初版）

对齐 DRL-AUV / DRL-robot-navigation 的成熟做法，降低调参难度：

**状态 `s`（示例）**

- 深度点云投影的 **N 维最近距离扇区**（如 N=20～36；先滤地面再取最小距离）  
- 到**局部目标**的距离与方位角（极坐标）  
- 上一拍动作 `(v, ω)`  

**动作 `a`**

- 连续 `(v, ω)`，再限幅到 Scout 安全范围（可参考 `nav_bringup.launch` 里的速度上限作物理限幅）

**奖励 `r`（初版）**

- 靠近目标：正奖励（距离减小）  
- 碰撞 / 翻车 / 进入不可通行格：大负奖励并 `done`  
- 鼓励前进、惩罚原地空转  
- （可选）高程变化过大、侧倾过大：负奖励，促使「懂地形」

**训练环境**

- 每 episode：随机起点与目标（目标需落在可通行区域）  
- `use_sim_time=true`；注意 ROS 回调与 PyTorch 训练线程分离（参考 AUV 项目双终端模式）

---

## 六、2.5D「3D 路径规划」怎么落地

不要第一步就写完整 OctoMap 体素 A\*。推荐阶梯：

| 级别 | 表示 | 规划器 | 何时用 |
|------|------|--------|--------|
| L0 | 深度扇区距离（无全局图） | 仅 TD3 随机目标 | 验证 RL 闭环最快 |
| L1 | 局部 `OccupancyGrid`（高度阈值二值化） | 2D A\*（代价来自高度） | 接上「规划+跟踪」叙事 |
| L2 | `grid_map`：elevation + slope + traversability | A\* / 采样 RRT\* | 正式越野表达 |
| L3 | MLS / 多层地表 + 运动基元 | ugv_nav4d 类 | 远期增强 |

第二阶段验收以 **L0→L1→L2** 为序即可。

开源积木（按需引入，非必须一次全上）：

- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) + [grid_map](https://github.com/ANYbotics/grid_map)  
- [traversability_estimation](https://github.com/leggedrobotics/traversability_estimation)  
- OMPL RRT\*（在 traversability 代价上采样）

---

## 七、与现有传感器的对接

### 方块车（`scout_mini_pro_description`）

- 深度点云：`/scout_mini_pro/depth/points`（见 README_2）  
- 控制：`/scout_mini_pro/cmd_vel`  
- IMU：插件若未开，越野 RL 的姿态惩罚与 LIO-SAM 前需启用  

### 官方车（README_4）

- 深度：`/camera/depth/points`  
- 控制：`/cmd_vel`  
- 第二阶段 launch 用参数切换命名空间即可  

### 真机 EDU

- D435 + Helios-16P + CH110：与「深度几何 + LIO-SAM」长期一致；仿真先跑通再迁。  

---

## 八、明确不做什么（本阶段边界）

- 不把 ORFD / OFF-Net 当作第二阶段必交付。  
- 不把仓库整体升级到 ROS 2 Jazzy。  
- 不删除第一阶段 A\* + Pure Pursuit。  
- 不以「端到端一张图出 cmd_vel、无任何几何层」为唯一方案（可作实验分支，不作唯一主线）。

下一篇：[3D_README_3.md](3D_README_3.md) —— 里程碑与验收。
