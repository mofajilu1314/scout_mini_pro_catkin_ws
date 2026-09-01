# 仿真相关算法文献清单

面向 **Gazebo + Scout（或同类地面无人车）** 的导航 / 位姿 / 建图算法调研，供挑选下载与开题对照。  
检索侧重约 **2018–2026**；经典奠基文单独标注。只列算法文献，不含实现代码。

---

## 使用说明（如何挑、如何下载）

1. **先看「推荐优先级」一节**，再按类别深入条目。
2. 优先打开 **arXiv PDF** 或 **DOI**；无开放 PDF 时用 DOI / IEEE / MDPI / PMC 页面。
3. 每条标注类型：**综述** / **算法提出** / **应用**。
4. 建议下载策略：每类先下 **1 篇综述 + 2～3 篇开源友好算法文**，再按创新方向补深度学习 / 混合规划文。
5. 与本工作区常见设定（**AMCL + 2D `/scan`**）对照时，重点看「3D/2.5D 定位替代 AMCL」与「云图→可导航地图」两类。

---

## 范围是否够用：客观判断

用户原先点名的 **TD3、LIO-SAM（及音近的 IOAM）、LOAM、AMCL** 作为入门线索 **够用做对照与基线**，但 **不够覆盖当前 Scout+Gazebo 上更主流、更好上手做实验与创新的选择**：

| 方向 | 仅盯原关键词 | 更现实的判断 |
|------|--------------|--------------|
| 导航 | TD3 是连续控制强基线，但近年仿真/论文里 **SAC、PPO** 同样常见，且常与 **TEB/DWA**、CADRL 类方法对照；纯端到端 TD3 不一定是仿真里最稳的起步点 | 建议：**TEB/DWA 工程基线 + SAC/PPO/TD3 三选一对比**，再视需要加 learning local planner / safety shield |
| 位姿 | LIO-SAM / LOAM 系仍重要；但 **FAST-LIO2、KISS-ICP、DLIO、hdl_localization** 在开源与仿真复现上往往更轻、更常被用作 3D 替代 AMCL 的起点 | 建议：**FAST-LIO2（里程计）+ hdl/NDT scan-to-map（先验图定位）** 比「只换 LIO-SAM」更贴合导航栈 |
| 建图 | LOAM 系偏里程计+稀疏特征图；导航还常需要 **OctoMap / 2D costmap / NDT 地图** | 建议：建图与「给导航用的占用/NDT 图」分开选型 |
| 缩写核实 | **未找到标准缩写「IOAM」「RIOSM / SM 的 3D 仿真位姿」对应的通行论文名** | 近义检索结果见第 2 节说明（优先按 **LIO-SAM / SC-LIO-SAM / 2.5D·BEV 定位** 理解） |

**结论**：原关键词适合作为「要对比的经典线」；若目标是仿真实验与可写创新点，应把下文中的 **FAST-LIO2 / KISS-ICP / SAC·PPO / TEB / Scan Context·OverlapTransformer / OctoMap** 一并纳入候选，而不是只扩写 TD3+LIO-SAM+LOAM。

---

## 1. 导航算法（含 TD3，并扩至 SAC/PPO/传统局部规划）

### 1.1 综述

- **Deep Reinforcement Learning of Mobile Robot Navigation in Dynamic Environment: A Review** (2025) — 综述  
  - 作者：多位（Sensors 期刊文） / *Sensors*  
  - 链接：https://www.mdpi.com/1424-8220/25/11/3394  
  - 系统梳理 DQN / DDPG / **TD3** / **SAC** 等在动态环境移动机器人导航中的用法、优劣与训练难点；适合作为 DRL 导航总览后再选算法。

- **Survey on Path Planning Based on Deep Reinforcement Learning** (2025) — 综述  
  - 作者：Xu 等 / PMLR  
  - 链接：https://proceedings.mlr.press/v278/xu25a.html  
  - 覆盖 PPO、DDPG、TD3 等策略梯度/Actor-Critic 在路径规划中的进展；偏方法分类，便于写 related work。

- **A survey on autonomous navigation for mobile robots: From traditional techniques to deep learning and large language models** (2025) — 综述  
  - 作者：多位 / *Journal of Big Data* 系列（Springer）  
  - 链接：https://doi.org/10.1007/s44443-025-00216-x  
  - 从传统图搜索到 RL/深度学习再到 LLM 导航的宽综述；适合判断「传统 vs 学习」边界，不必全文精读。

### 1.2 DRL 算法基线（含 TD3）

- **Addressing Function Approximation Error in Actor-Critic Methods (TD3)** (2018) — 算法提出（奠基）  
  - 作者：Fujimoto, Hoof, Meger / ICML  
  - 链接：https://arxiv.org/abs/1802.09477  
  - TD3 原文：双 Q、延迟策略更新、目标策略平滑；连续控制导航论文几乎都会引用。

- **Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor** (2018) — 算法提出（奠基）  
  - 作者：Haarnoja 等 / ICML  
  - 链接：https://arxiv.org/abs/1801.01290 · PDF：https://arxiv.org/pdf/1801.01290  
  - **SAC** 在近年机器人连续控制/导航仿真中常比 TD3 更「好训、好探索」；建议与 TD3 并列对照，而非只盯 TD3。

- **Proximal Policy Optimization Algorithms** (2017) — 算法提出（奠基，略早但必引）  
  - 作者：Schulman 等 / arXiv  
  - 链接：https://arxiv.org/abs/1707.06347  
  - **PPO** 实现简单、稳定性好；Isaac/Gazebo 类并行仿真与 NavRL 等近年工作常用。

- **Continuous control with deep reinforcement learning (DDPG)** (2015) — 算法提出（奠基）  
  - 作者：Lillicrap 等 / ICLR  
  - 链接：https://arxiv.org/abs/1509.02971  
  - TD3/SAC 的重要前身；写演进关系时引用即可，不必作为首选实现。

### 1.3 DRL 导航应用 / 对照实验

- **Deep Reinforcement Learning for Autonomous Mobile Robot Navigation: Comparing DQN, DDPG, and TD3** (2025) — 应用  
  - 作者：会议作者组 / IEEE ICERA  
  - 链接：https://doi.org/10.1109/icera66156.2025.11087316  
  - **ROS2+Gazebo+TurtleBot3** 上直接对比 DQN/DDPG/**TD3**；与 Scout+Gazebo 仿真设定接近，适合作为「TD3 是否值得做」的实证参考。

- **A Comparison Study between Traditional and Deep-Reinforcement-Learning-Based Algorithms for Indoor Autonomous Navigation in Dynamic Scenarios** (2023) — 应用  
  - 作者：多位 / *Sensors*  
  - 链接：https://pmc.ncbi.nlm.nih.gov/articles/PMC10748032/  
  - 对照 **DWA、TEB、CADRL、SAC**；结论按场景分化，避免「DRL 一定更好」的空话，很适合写实验设计。

- **Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems** (2021) — 应用  
  - 作者：Kastner 等 / arXiv  
  - 链接：https://arxiv.org/abs/2104.03616 · HTML：https://ar5iv.labs.arxiv.org/html/2104.03616  
  - 把 DRL 局部避障接入常规导航栈，并与 TEB/DWA/MPC/**CADRL** 对比；对「学习局部规划器替换 move_base 局部层」很有参考价值。

- **Benchmarking Deep Reinforcement Learning for Navigation in Denied Sensor Environments** (2025) — 应用  
  - 作者：多位 / *Journal of Intelligent & Robotic Systems*  
  - 链接：https://doi.org/10.1007/s10846-025-02309-1  
  - 在传感器缺失/噪声下对比 PPO、**TD3**、DreamerV3 等；提醒仿真里「完美雷达」与真实退化差很大。

- **World Models for Autonomous Navigation of Terrestrial Robots from LIDAR observations** (2025) — 应用  
  - 作者：多位 / arXiv  
  - 链接：https://arxiv.org/abs/2512.03429 · HTML：https://arxiv.org/html/2512.03429v1  
  - 用世界模型+LiDAR 与 SAC/DDPG/**TD3** 对比；适合「全分辨率激光 + model-based RL」创新方向，算力与实现成本更高。

### 1.4 传统 / 优化局部规划与近年热点

- **The Dynamic Window Approach to Collision Avoidance** (1997) — 算法提出（经典/奠基）  
  - 作者：Fox, Burgard, Thrun / IEEE RAM  
  - 链接：https://doi.org/10.1109/100.580977  
  - **DWA**：ROS `move_base` 默认局部规划之一；写「学习 vs 传统」时作对照即可。

- **Timed-Elastic-Bands for time-optimal point-to-point nonlinear model predictive control** (TEB 相关工作线，2010s) — 算法提出（工程主流）  
  - 作者：Rösmann 等 / 系列 IEEE 文  
  - 入口综述式介绍与实现生态见导航对照文；DOI 检索关键词：`Timed Elastic Band` Rösmann  
  - 示例入口：Arena-Rosnav 文中 TEB 引用链 https://ar5iv.labs.arxiv.org/html/2104.03616  
  - **TEB** 仍是地面车 Gazebo 仿真里最稳、最好调的局部规划基线之一。

- **EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors** (2021) — 算法提出  
  - 作者：Zhou, Wang, Ye, Xu, Gao / RA-L  
  - 链接：https://arxiv.org/abs/2008.08835 · PDF：https://zhepeiwang.github.io/pubs/ral_2021_egoplan.pdf  
  - 无 ESDF、梯度局部规划，开源成熟；**原作为无人机**，地面 Scout 需改动力学/约束，但「轻量局部轨迹优化」思路可迁移，适合作为创新对照而非直接抄装。

- **Socially Aware Motion Planning with Deep Reinforcement Learning (CADRL)** (2017) — 算法提出  
  - 作者：Chen, Everett, Liu, How / IROS  
  - 链接：https://arxiv.org/abs/1703.08862  
  - 动态行人/智能体避障经典 DRL；后续 CrowdNav 等大量跟进；适合「动态障碍仿真」创新，静态仓库场景收益有限。

- **NavRL: Learning Safe Flight in Dynamic Environments** (2024) — 算法提出/应用  
  - 作者：多位 / arXiv  
  - 链接：https://arxiv.org/abs/2409.15634 · PDF：https://arxiv.org/pdf/2409.15634  
  - **PPO + safety shield（速度障碍思想）+ 并行仿真**；平台是无人机/Isaac，但对「RL + 安全层」写创新点很有用，地面车可借鉴结构而非照搬。

- **MPPI（Model Predictive Path Integral）控制** — 算法提出（采样 MPC 族）  
  - 代表入口：Williams 等 MPPI 工作（检索 `Model Predictive Path Integral Control` Williams）  
  - 近年常与 nav2 采样局部规划、GPU 并行采样一起出现；适合「非学习、但比 DWA 更现代」的局部控制对照。若只做 DRL 论文可后读。

---

## 2. 位姿估计算法（3D / 2.5D，替代平面 AMCL）

### 2.0 缩写说明（IOAM / RIOSM）

- **IOAM**：在常见 LiDAR SLAM 文献中 **未检索到被广泛承认的同名标准算法**；音近且高度相关的是 **LIO-SAM**（Lidar-Inertial Odometry via Smoothing and Mapping）、以及社区包 **SC-LIO-SAM**（Scan Context + LIO-SAM）。下文按此理解。  
- **RIOSM /「SM 的 3D 仿真位姿」**：无稳定公开论文缩写命中；更可能是口语混读 **LIO-SAM / LeGO-LOAM-SM（某应用文中的改进名）/ ROS 仿真位姿**。请以 **LIO-SAM 族 + 3D scan-to-map 定位** 为主线检索。  
- **2.5D**：近年多指 **BEV / 多层水平切片 / 高程图** 上的定位，而非完整 6DoF 稠密 3D；见 MB-Loc、GroundLoc、BEVDiffLoc。

### 2.1 综述

- **LiDAR-based SLAM for robotic mapping: state of the art and new frontiers** (2023) — 综述  
  - 作者：Yue, Zhang, He / arXiv  
  - 链接：https://arxiv.org/abs/2311.00276 · PDF：https://arxiv.org/pdf/2311.00276  
  - 覆盖 LOAM 系、LIO-SAM、FAST-LIO 等脉络，适合快速建立「谁取代谁」的地图。

- **A Survey on Global LiDAR Localization: Challenges, Advances and Open Problems** (2023) — 综述  
  - 作者：多位 / arXiv  
  - 链接：https://arxiv.org/abs/2302.07433 · PDF：https://arxiv.org/pdf/2302.07433  
  - 全局定位、重定位、BEV/深度学习描述子；对应「有先验图时如何替代/增强 AMCL」。

### 2.2 经典平面定位（对照用）

- **Monte Carlo Localization for Mobile Robots** (相关奠基线，约 1999–2001) — 算法提出（经典/奠基）  
  - 作者：Fox, Dellaert, Thrun, Burgard 等  
  - 入口：检索 `Monte Carlo Localization` Fox Thrun；ROS 实现生态即 **AMCL**  
  - 平面粒子滤波 + 2D 激光似然场；当前工程若仍用 AMCL+OccupancyGrid，本文是理论源头。3D/起伏地面时假设易失效，故需 3D/2.5D 替代。

### 2.3 LiDAR(-Inertial) 里程计 / SLAM（替代「纯 2D 位姿源」）

- **LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping** (2020) — 算法提出  
  - 作者：Shan 等 / IROS  
  - 链接：https://arxiv.org/abs/2007.00258 · PDF：https://arxiv.org/pdf/2007.00258  
  - 因子图紧耦合 LIO，可接回环/GPS；社区生态大，但依赖 IMU 质量与外参，Gazebo 里要认真对齐时间与外参。

- **FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter** (2020/2021) — 算法提出  
  - 作者：Xu, Zhang / RA-L/IROS 线  
  - 链接：https://arxiv.org/abs/2010.08196  
  - 迭代卡尔曼紧耦合，算力友好；**FAST-LIO2** 的前身。

- **FAST-LIO2: Fast Direct LiDAR-inertial Odometry** (2022) — 算法提出  
  - 作者：Xu 等 / TRO  
  - 链接：https://arxiv.org/abs/2107.06829 · PDF：https://arxiv.org/pdf/2107.06829  
  - **直接点到图 + ikd-Tree**，开源极火；Scout+多线/固态雷达仿真里常是 **首选 3D 里程计基线**，比「只上 LIO-SAM」更轻。

- **Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry** (2023) — 算法提出  
  - 作者：He 等 / *Advanced Intelligent Systems*  
  - 链接：https://doi.org/10.1002/aisy.202200459 · 代码说明：https://github.com/hku-mars/Point-LIO  
  - 高带宽、抗剧烈运动/IMU 饱和；地面车一般不如 FAST-LIO2「刚需」，但适合「高动态/震动」场景创新。

- **Direct LiDAR-Inertial Odometry (DLIO)** (2022) — 算法提出  
  - 作者：Chen 等 / RA-L/IROS  
  - 链接：https://arxiv.org/abs/2203.03749 · HTML：https://ar5iv.labs.arxiv.org/html/2203.03749  
  - 轻量 LIO + 连续时间去畸变；常与 FAST-LIO2/LIO-SAM 同台对比，仿真复现友好。

- **KISS-ICP: In Defense of Point-to-Point ICP** (2023) — 算法提出  
  - 作者：Vizzo 等 / RA-L  
  - 链接：https://arxiv.org/abs/2209.15397 · HTML：https://arxiv.org/html/2209.15397  
  - **几乎无调参的纯激光里程计**；无 IMU 也能跑，Gazebo 里很适合做「最简 3D 里程计」基线或消融对照。

- **CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure** (2022) — 算法提出  
  - 作者：Dellenbach 等 / ICRA  
  - 链接：https://arxiv.org/abs/2109.12979 · PDF：https://arxiv.org/pdf/2109.12979  
  - 连续时间弹性 ICP + 高程图回环；精度强，参数与工程复杂度高于 KISS-ICP。

- **LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain** (2018) — 算法提出  
  - 作者：Shan, Englot / IROS  
  - 链接：https://doi.org/10.1109/iros.2018.8594299  
  - 面向地面车、利用地面分割；与 Scout 场景契合，可作 LOAM 系轻量化对照。

### 2.4 先验地图上的 3D / NDT / MCL 定位（更接近「替换 AMCL」）

- **A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement (hdl_graph_slam / 相关定位生态)** (2019) — 应用/系统  
  - 作者：Koide, Miura, Menegatti 等  
  - 入口：https://github.com/koide3/hdl_graph_slam · 定位常用配套 **hdl_localization**（NDT scan-to-map）  
  - ROS 生态里用 **3D 点云地图 + NDT** 做定位的事实标准之一；接口习惯接近 AMCL（`map→odom`），对「替换 AMCL」工程路径最直接。

- **The Normal Distributions Transform: A New Approach to Laser Scan Matching** (2003) — 算法提出（经典/奠基）  
  - 作者：Biber, Straßer  
  - 链接：检索 DOI / IEEE：`Normal Distributions Transform` Biber  
  - **NDT** 是 hdl_localization、许多 3D MCL 变体的匹配内核；理解 scan-to-map 必备。

- **Improved Monte Carlo Localization … with the Normal Distributions Transform** (2025) — 应用  
  - 作者：多位 / IJACSA  
  - 链接：https://doi.org/10.14569/ijacsa.2025.01603100  
  - **AMCL 初值 + NDT 精修** 一类流水线；说明「粒子滤波与 NDT」可组合，而非二选一。

### 2.5 2.5D / BEV 定位（替代纯平面假设的折中）

- **MB-Loc: Multi-planar Bird’s-eye-view Localization in outdoor LiDAR scenes** (2026 预印本) — 算法提出  
  - 作者：多位 / arXiv  
  - 链接：https://arxiv.org/abs/2606.08744 · HTML：https://arxiv.org/html/2606.08744v2  
  - **多层 2.5D BEV** + 学习定位；保留高度结构又比全 3D 卷积轻，适合作为「2.5D 替代 AMCL」的新方向（注意预印本年份新，引用时核对正式发表状态）。

- **GroundLoc: LiDAR-Only Localization for Outdoor Robotics** (2025) — 算法提出  
  - 作者：多位 / arXiv  
  - 链接：https://arxiv.org/abs/2510.24623  
  - 地面区域 BEV + 特征匹配；地图存储小、偏室外大场景，思路可借鉴到仿真园区。

- **BEVDiffLoc: End-to-End LiDAR Global Localization in BEV View based on Diffusion Model** (2025) — 算法提出  
  - 作者：多位 / arXiv  
  - 链接：https://arxiv.org/pdf/2503.11372  
  - BEV + 扩散模型端到端位姿；创新性强，仿真/数据与算力门槛高，适合「深度学习定位」选题而非首周试跑。

---

## 3. 点云与建图算法（LOAM 系、云图、导航用地图）

### 3.1 综述（与第 2 节共享）

第 2.1 的 LiDAR SLAM 综述、Global LiDAR Localization 综述同样覆盖建图与回环；建图专项可再读 OctoMap 原文与 VoxelMap。

### 3.2 LOAM 系与建图里程计

- **LOAM: Lidar Odometry and Mapping in Real-time** (2014) — 算法提出（经典/奠基）  
  - 作者：Zhang, Singh / RSS  
  - 链接：https://www.roboticsproceedings.org/rss10/p07.pdf （或检索 `LOAM Zhang Singh`）  
  - 特征点 LOAM 奠基；后续 LeGO-LOAM / LIO-SAM / F-LOAM 均由此分支。

- **LeGO-LOAM** (2018) — 算法提出  
  - 见 §2.3；同时输出可用于后续建图/回环的轨迹与特征图。

- **F-LOAM: Fast LiDAR Odometry and Mapping** (2021) — 算法提出  
  - 作者：Wang 等 / IROS  
  - 链接：检索 `F-LOAM` arXiv / IEEE；综述引用见 https://arxiv.org/pdf/2311.00276  
  - 非迭代两阶段畸变补偿，偏效率；适合作为 LOAM 系轻量对照。

- **SC-LIO-SAM（Scan Context + LIO-SAM）** (工程集成，算法来自 2018+2020) — 应用/系统  
  - 维护：gisbi-kim 等  
  - 代码/说明：https://github.com/gisbi-kim/SC-LIO-SAM  
  - **不是单独顶会新算法名**，但是「LIO-SAM + 全局描述子回环」的常用组合；若用户说的「SM」近音，优先核对此仓库与 Scan Context 原文。

### 3.3 地图表示（给导航用）

- **OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees** (2013) — 算法提出（经典/奠基，导航仍常用）  
  - 作者：Hornung 等 / *Autonomous Robots*  
  - 链接：https://octomap.github.io/ · 论文常见入口 DOI 检索 `OctoMap Hornung`  
  - 八叉树占用概率图；SLAM 输出点云后常投影/转为 OctoMap 再切 2D costmap，**与导航衔接最直接**。

- **Efficient and Probabilistic Adaptive Voxel Mapping for Accurate Online LiDAR Odometry (VoxelMap)** (2022) — 算法提出  
  - 作者：Yuan, Xu, Zhang 等 / RA-L  
  - 链接：https://arxiv.org/abs/2109.07082 · 代码：https://github.com/hku-mars/VoxelMap  
  - 自适应体素平面+不确定性，服务高精度里程计；适合「地图表示创新」，不是 nav2 开箱 costmap。

- **KISS-SLAM: A Simple, Robust, and Accurate 3D LiDAR SLAM System…** (2025) — 算法提出  
  - 作者：多位 / arXiv  
  - 链接：https://arxiv.org/pdf/2503.12660  
  - 在 KISS-ICP 思路上补 SLAM/回环；追求少调参泛化，适合仿真多场景对比。

### 3.4 深度学习相关（定位/回环/描述子，可做创新点）

- **Scan Context: Egocentric Spatial Descriptor for Place Recognition Within 3D Point Cloud Map** (2018) — 算法提出  
  - 作者：Kim, Kim / IROS  
  - 链接：https://doi.org/10.1109/iros.2018.8593953 · 代码：https://github.com/gisbi-kim/scancontext  
  - 手工全局描述子，易接入 LOAM/LIO/FAST-LIO；**仿真里加回环、做「描述子+里程计」创新**门槛低。

- **OverlapTransformer: An Efficient and Yaw-Angle-Invariant Transformer Network for LiDAR-Based Place Recognition** (2022) — 算法提出  
  - 作者：Ma 等 / RA-L  
  - 链接：https://ieeexplore.ieee.org/document/9785497 · 代码：https://github.com/haomo-ai/OverlapTransformer  
  - Transformer 回环/重识别；比 Scan Context 更「深度学习向」，适合发创新，数据与训练成本更高。

- **PointNetVLAD: Deep Point Cloud Based Retrieval for Large-Scale Place Recognition** (2018) — 算法提出  
  - 作者：Uy, Lee / CVPR  
  - 链接：检索 `PointNetVLAD` CVPR 2018（IEEE / 作者页）  
  - 点云检索经典；后续大量 LPR 工作以此为基线。

- **SegMap: Segment-based mapping and localization using data-driven descriptors** (2019) — 算法提出  
  - 作者：Dubé 等 / IJRR  
  - 链接：检索 `SegMap` IJRR 2019  
  - 分割级语义/数据驱动描述子做建图定位；适合「结构化场景 + 语义创新」，实现重。

- **Research on Underground Coal Mine Map Construction Method Based on LeGO-LOAM Improved Algorithm (LeGO-LOAM-SM)** (2022) — 应用  
  - 作者：多位 / *Energies*  
  - 链接：https://doi.org/10.3390/en15176256 · PDF：https://www.mdpi.com/1996-1073/15/17/6256/pdf  
  - LeGO-LOAM + SegMatch 回环，并讨论 **OctoMap / 2D 占用栅格** 以服务导航；名称含 **「-SM」**，可能与口语「SM」混淆，下载时勿与 LIO-SAM 搞混。

---

## 4. 推荐优先级 / 试跑建议（哪个好试用哪个）

面向 **Scout + Gazebo 仿真做实验** 的务实排序（非「论文影响力」总榜）。

### 4.1 好上手、适合先跑通仿真（工程稳）

| 优先级 | 方向 | 建议起点 | 理由 |
|--------|------|----------|------|
| P0 | 局部导航基线 | **TEB 或 DWA** | ROS 导航栈成熟，易与现有 AMCL+costmap 对齐，先拿到可复现指标 |
| P0 | 3D 里程计 | **FAST-LIO2** 或 **KISS-ICP** | 开源活跃、文档/复现多；KISS-ICP 甚至可不依赖 IMU |
| P1 | 替换 AMCL 的定位形态 | **hdl_localization 类 NDT scan-to-map** | 最接近「有先验图的定位」问题，而不是只做里程计 |
| P1 | 云图→导航图 | **SLAM 点云 → OctoMap / 2D 投影 costmap** | 否则 3D 建图与 move_base 脱节 |

### 4.2 适合做算法对比实验（写论文 related work / ablation）

- 导航：**SAC vs TD3 vs PPO**，并固定同一感知输入（激光 beam / 局部 costmap）；对照 **TEB**。  
- 位姿：**FAST-LIO2 vs LIO-SAM vs KISS-ICP**（同传感器、同轨迹指标 ATE/RPE）。  
- 回环：**无回环 vs Scan Context vs（可选）OverlapTransformer**。

### 4.3 更适合冲创新点（成本更高）

- **DRL + 安全层**（借鉴 NavRL / Arena 的「学习局部规划接入传统栈」）。  
- **2.5D/BEV 定位**（MB-Loc / GroundLoc / BEVDiffLoc）对比 AMCL 平面假设失效场景（斜坡、多层结构）。  
- **学习描述子回环**（OverlapTransformer 等）或 **世界模型+LiDAR**（算力敏感）。  
- **Ego-Planner 思路迁移到地面差速/阿克曼**（需改模型，不是开箱）。

### 4.4 可后置或谨慎选

- 只追 **TD3** 而不做 SAC/PPO/TEB 对照：叙事偏窄。  
- 一上来 **Point-LIO / CT-ICP / 扩散定位**：精度/噱头强，但调参与算力成本高，不适合作为第一周基线。  
- **纯视觉或 LLM 导航综述向工作**：与当前「激光/Scout 仿真」主线较远，除非明确改传感器配置。

---

## 可选对照：与当前工程（AMCL + 2D `/scan`）的衔接建议

1. **短期**：保留 AMCL+2D 导航作对照基线；并行接入 **3D 里程计（FAST-LIO2/KISS-ICP）** 只替换 `odom` 质量来源，观察定位/规划是否变稳。  
2. **中期**：用 3D SLAM 建先验图，改 **NDT/hdl 类 scan-to-map** 做 `map→odom`，在起伏/非水平结构场景对比 AMCL 失效案例。  
3. **导航层**：DRL（TD3/SAC）更适合先做 **局部避障插件**（Arena 思路），而不是一次性端到端替换全局规划。  
4. **建图产物**：点云地图需落到 **OctoMap 或 2D 占用图** 才能喂给常用局部规划；否则「云图」与「能跑」之间仍缺一层。

---

## 文献条目速查（按下载紧急度）

**本周建议先下（约 8 篇）**

1. TD3 原文 https://arxiv.org/pdf/1802.09477  
2. SAC 原文 https://arxiv.org/pdf/1801.01290  
3. DRL 动态导航综述 https://www.mdpi.com/1424-8220/25/11/3394  
4. Arena-Rosnav https://arxiv.org/pdf/2104.03616  
5. LIO-SAM https://arxiv.org/pdf/2007.00258  
6. FAST-LIO2 https://arxiv.org/pdf/2107.06829  
7. KISS-ICP https://arxiv.org/pdf/2209.15397  
8. LiDAR SLAM 综述 https://arxiv.org/pdf/2311.00276  

**第二周补（定位/建图/深度）**

9. Global LiDAR Localization 综述 https://arxiv.org/pdf/2302.07433  
10. DLIO https://arxiv.org/pdf/2203.03749  
11. CT-ICP https://arxiv.org/pdf/2109.12979  
12. VoxelMap https://arxiv.org/pdf/2109.07082  
13. Scan Context（IEEE）https://doi.org/10.1109/iros.2018.8593953  
14. OverlapTransformer（IEEE）https://ieeexplore.ieee.org/document/9785497  
15. TEB/DWA/SAC/CADRL 对照 https://pmc.ncbi.nlm.nih.gov/articles/PMC10748032/  

---

*生成说明：条目经 Web 检索（arXiv / IEEE / MDPI / 综述与开源仓库）整理；个别会议 PDF 需机构权限时以 DOI/作者主页为准。未找到标准缩写 IOAM/RIOSM 的通行论文，已在文中如实标注近义检索结果。*
