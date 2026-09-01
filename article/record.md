# 论文1-深度强化学习导航算法综述：**Deep Reinforcement Learning of Mobile Robot Navigation in Dynamic Environment: A Review**

| 类别               | 方法                  | 年份 | 计算成本 | 样本效率 | 对环境变化的鲁棒性 | 优势                                                                                  | 劣势                                                         |
| ------------------ | --------------------- | ---- | -------- | -------- | ------------------ | ------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| **基于值**   | DQN [33,58,63]        | 2013 | 中       | 低       | 低                 | 使用深度神经网络解决Q学习可扩展性问题；通过经验回放和目标网络稳定学习                 | 易出现过估计偏差；神经网络计算复杂度高；对学习率等超参数敏感 |
|                    | Double DQN [57,60,64] | 2015 | 中       | 中       | 中                 | 通过分离动作选择和值估计减少过估计偏差；策略学习更稳定准确                            | 维护两个Q网络增加计算复杂度；对学习率和更新频率等超参数敏感  |
|                    | Dueling DQN [61,65]   | 2016 | 中       | 中       | 中                 | 分离状态价值和动作优势，实现更精确的Q值估计；在动作价值差异小的场景中提高学习效率     | 额外网络架构增加计算复杂度；对超参数和网络设计选择敏感       |
| **基于策略** | TRPO [8,37]           | 2015 | 高       | 中       | 高                 | 通过信任区域约束确保稳定单调的策略改进；适用于高维连续动作空间                        | 计算昂贵（约束优化）；实现比简单策略梯度复杂                 |
|                    | PPO [37,66]           | 2017 | 中       | 高       | 中                 | 通过裁剪机制简化信任区域优化，提高计算效率；平衡策略稳定性和学习效率，适合实时任务    | 对裁剪阈值等超参数敏感；在高度复杂环境中可能性能退化         |
| **混合**     | A3C [67,68]           | 2016 | 低       | 中       | 中                 | 通过多个并行智能体的异步更新提高训练效率；无需经验回放，减少内存需求                  | 样本效率低；需要大量并行计算资源                             |
|                    | DDPG [16,69]          | 2015 | 中       | 中       | 低                 | 有效处理高维连续动作空间；利用演员-评论家框架结合策略梯度和基于值方法的优势           | 易过拟合和不稳定；需要大量超参数调优，对噪声设置敏感         |
|                    | SAC [36,70]           | 2018 | 高       | 中       | 高                 | 利用最大熵框架鼓励探索，提高鲁棒性；使用双Q网络减少过估计偏差，增强稳定性             | 计算密集（同时训练多个网络）；性能对熵权重系数高度敏感       |
|                    | TD3 [35,71]           | 2018 | 中       | 中       | 高                 | 使用双Q网络减少过估计偏差，提高值估计准确性；通过延迟策略更新和目标策略平滑增强稳定性 | 计算昂贵（训练多个网络）；对更新延迟和噪声设置等超参数敏感   |

增强感知和决策能力是提升动态环境导航性能的另一个关键方向。

动态环境的不可预测性对导航系统的实时感知和决策能力提出了严格要求。当前DRL算法在适应实时动态变化方面存在明显局限，

多模态感知对于增强动态环境中的导航鲁棒性至关重要，但现有数据融合技术在实际部署中仍面临显著瓶颈。

未来研究应聚焦于利用图神经网络和基于Transformer的架构进行传感器数据的静态和时间对齐，提高异构模态下的泛化能力[122]。

造成这种差距的因素包括：第一，仿真物理引擎无法完全再现现实世界的复杂动力学，如摩擦、气动效应和传感器噪声[143,144]。第二，传感器-模型不匹配（例如仿真与物理LiDAR之间的差异）导致状态估计误差，严重破坏学习到的策略[145,146]。第三，训练环境的奖励结构常未能捕捉部署时出现的关键安全和延迟约束，导致现场场景中的脆弱行为[147,148]。最后，大多数DRL模型假设完美可观测性和平稳动力学，这在实践中很少满足，导致执行期间适应失败[149]。

因此，未来研究应着力于开发轻量级、具备实时能力的仿真环境，保留任务相关动态同时保持计算高效[150]。领域随机化、自适应仿真校准和对抗性仿真-现实增强等技术应进一步完善以提高对领域偏移的鲁棒性[151,152]。此外，仿真感知策略架构，包括特征解耦和隐空间对齐，可提供更好的泛化[153]。优化DRL代理以实现跨平台兼容性，并在部署后集成在线策略修正机制，对于缩小仿真-现实循环至关重要[154]。最终，仿真到现实迁移必须从离线预训练流程演变为在线自适应范式，动态对齐学习策略与现实世界的不确定性和感知变异性[155]。

动态环境中的导航任务常涉及高风险场景，一次决策错误可能导致任务失败或人身伤害。在此背景下，确保DRL算法的安全性和可解释性成为现实部署的关键先决条件。

未来研究应优先在DRL框架内整合自适应安全防护和在线风险感知学习模块，以在执行期间主动减轻不安全动作[162]。此外，将DRL与符号逻辑、因果模型或基于模型检查的验证相结合，可在保持学习灵活性的同时增强策略透明度[163]。最后，应努力建立用于DRL可解释性和安全性的基准数据集和评估协议，促进可重复性和可信部署[164]。通过解决这些领域特定差距，未来系统可在动态和高风险环境中实现鲁棒、安全和负责任的导航。

* **Deep Reinforcement Learning of Mobile Robot Navigation in Dynamic Environment: A Review** (2025) — 综述
  * 作者：多位（Sensors 期刊文） / *Sensors*
  * 链接：[https://www.mdpi.com/1424-8220/25/11/3394](https://www.mdpi.com/1424-8220/25/11/3394)
  * 系统梳理 DQN / DDPG / **TD3** / **SAC** 等在动态环境移动机器人导航中的用法、优劣与训练难点；适合作为 DRL 导航总览后再选算法。路

# 论文2TD3：**Addressing Function Approximation Error in Actor-Critic Methods (TD3)**

* **Addressing Function Approximation Error in Actor-Critic Methods (TD3)** (2018) — 算法提出（奠基）
  * 作者：Fujimoto, Hoof, Meger / ICML
  * 链接：[https://arxiv.org/abs/1802.09477](https://arxiv.org/abs/1802.09477)
  * TD3 原文：双 Q、延迟策略更新、目标策略平滑；连续控制导航论文几乎都会引用。

鉴于近期对可重复性的关注（Henderson et al., 2017），我们使用大量随机种子和公平的评估指标进行实验，对每个贡献进行了消融研究，并开源了我们的代码和学习曲线（[https://github.com/sfujim/TD3](https://github.com/sfujim/TD3)）。

### TD3是在 **DDPG（深度确定性策略梯度）** 算法基础之上进行改进的。

DDPG是处理连续控制的开山鼻祖，但它有两个致命伤：**价值估计疯狂过估计** 和  **策略更新太莽撞** 。TD3针对这两个痛点，做了三大核心手术（这也是它名字“Twin Delayed”的由来）：

* **更改一：双网络取小（Clipped Double-Q）** —— 不用一个评论家（Critic），而是 **用两个评论家** 。计算目标值时，谁给的价值低就听谁的。这一刀直接砍掉了DDPG常见的“盲目自信”（过估计偏差），大幅减少了训练初期的暴毙概率。
* **更改二：延迟演员更新（Delayed Policy Updates）** —— DDPG是“一边学走路一边修正姿势”，容易摔跤。TD3强制 **评论家（价值网络）多更新几次（通常2次），演员（策略网络）才更新1次** 。让价值估计先收敛一点，策略再跟着动，避免了“用不准的尺子量长度”导致的震荡。
* **更改三：目标策略平滑（Target Policy Smoothing）** —— 计算目标值时，在目标动作上 **故意加一点随机噪声并裁剪** 。相当于告诉策略：“别钻牛角尖去找那个最精确的点，附近区域都行”。这极大地增强了动作输出的平滑性和抗干扰能力。

在MuJoCo物理仿真环境的7个标准任务中，TD3取得了 **碾压性的统治力** ：

* **性能碾压** ：在Hopper（跳跃）任务中，TD3得分3564，原始DDPG只有2020；在Walker2d（双足行走）中，TD3得分4682，DDPG只有1843，性能翻倍还不止。
* **收敛更快** ：TD3的学习曲线（图5）比当时最火的PPO和SAC上升更快，且波动更小。
* **实战意义** ：通过消融实验证明，只有三大修改**同时生效**时性能才达到巅峰，缺一个都会有明显短板。证明了“稳定且不冒进”的策略在连续控制中极其有效。

# 论文3 SAC算法：Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor

* **Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor** (2018) — 算法提出（奠基）
  * 作者：Haarnoja 等 / ICML
  * 链接：[https://arxiv.org/abs/1801.01290](https://arxiv.org/abs/1801.01290) · PDF：[https://arxiv.org/pdf/1801.01290](https://arxiv.org/pdf/1801.01290)
  * **SAC** 在近年机器人连续控制/导航仿真中常比 TD3 更「好训、好探索」；建议与 TD3 并列对照，而非只盯 TD3。

我们提出了Soft Actor-Critic（SAC），一种离策略最大熵深度强化学习算法，它在保留熵最大化和稳定性优势的同时提供了样本高效的学习。我们的理论结果推导了软策略迭代，我们证明其收敛到最优策略。从这一结果出发，我们可以公式化Soft Actor-Critic算法，并且我们凭经验证明它优于最先进的无模型深度RL方法，包括离策略DDPG算法和在策略PPO算法。事实上，该方法的样本效率实际上大幅超过了DDPG。我们的结果表明，随机的、熵最大化的强化学习算法可以为提高鲁棒性和稳定性提供有前途的途径，而对最大熵方法的进一步探索——包括结合二阶信息（例如信任域（Schulman等，2015））或更具表达力的策略类的方法——是令人兴奋的未来工作方向。


提出了  **“最大熵强化学习（Max-Entropy RL）”** 。

* **核心操作** ：它给算法强行加了一个“贪玩”的设定。在标准的奖励（Reward）后面，硬生生加了一个 **熵奖励（Entropy bonus）** 。
* **怎么玩** ：智能体每走一步，除了拿到环境给的任务奖励，还会因为“自己这一步走得有多随机”而额外拿一笔钱。动作越丰富、越不重复，这笔钱就越多。

#### 它的算法到底是怎么运作的？（三张网络各司其职）

为了让这个“贪玩”的设定跑起来，SAC的架构和TD3不一样。它用了 **三个网络** （TD3只有两个）：

1. **演员（Actor，策略网络）** ：负责输出 **“均值和方差”** 。它不再像TD3那样只输出一个确定的动作，而是输出一个 **动作范围（高斯分布）** ，告诉机器：“在这个状态下，你可能有10种走法，每种走的概率是多少”。
2. **软评论家（Soft Critic，Q网络）** ：和TD3一样，两个Q网络取最小值，判断状态-动作的价值。
3. **独有的价值网络（Value Network，V网络）** ：这是SAC的 **定海神针** 。它专门用来估算“当前状态有多大的潜力”。它不关心具体动作，只关心“这个局面好不好”。

# 论文4 Arena-Rosnav：**Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems**

* **Arena-Rosnav** (2021) — 应用
  * 作者：Kästner 等 / arXiv
  * 链接：https://arxiv.org/abs/2104.03616 · 本地：`article/导航/2104.03616v2.pdf`
  * 把 DRL 局部避障接入常规导航栈，并与 TEB/DWA/MPC/CADRL 对比；本项目「全局粗路径 → 局部子目标 → TD3」的直接模板。

**有用句子 / 可引用点（摘自摘要与引言，未编造页码）：**

* Abstract：DRL 因易陷局部极小、缺乏长期记忆，**不适合单独做长距离导航**；作者提出把 **DRL 局部规划器接入传统导航栈**，以支持 long-range navigation。
* Abstract：在与多种传统规划器对比中，其 DRL 增强系统在 **safety、efficiency、robustness** 上表现更好。
* Intro：经典方法擅长静态环境，高度动态避障仍难；工业上常过度保守或干脆避开复杂区域。
* Intro：DRL 的主要瓶颈是 **仅适合局部导航（myopic / 缺长期记忆）**；用 RNN 缓解代价高、收益有限。
* 贡献要点：提供 **中间路点规划器（intermediate way-point planner）**，把 DRL 与全局规划（RRT / A\* / Dijkstra）接起来；框架同时支持训练与测试，可与传统局部规划公平对照。

**与本项目相关性**：本仓库已定 **Arena 式分层**——全局仍用粗路径，局部用 TD3 跟 `/offroad/local_goal`；先别端到端替换全局规划。

---

# 论文5 LIO-SAM：**LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping**

* **LIO-SAM** (2020) — 算法提出
  * 作者：Shan 等 / IROS
  * 链接：https://arxiv.org/abs/2007.00258 · 本地：`article/建图/2007.00258v3.pdf`
  * 因子图紧耦合激光–惯性里程计；可接回环与 GPS；真车 / 大场景后期黑盒首选之一。

**算法要点 + 与本项目相关性：**

* 用 **因子图** 融合 IMU 预积分、激光里程计、回环与可选 GPS；IMU 用于去畸变并给激光匹配初值，激光结果再估计 IMU bias。
* 为实时性：**边缘化旧扫描**、关键帧 + 滑动窗口局部匹配，而不是每帧对全局稠密地图做匹配。
* 相对 LOAM：更易接入回环与绝对测量，大场景漂移问题有更好处理路径。
* **本项目**：仿真当前官方车 **无 IMU、无多线点云** → **第一周不上 LIO-SAM**；真机 Helios+IMU / 室外大图再作建图黑盒。TD3 训练阶段位姿用 Gazebo 真值或 `/odom` 即可。

---

# 论文6 FAST-LIO2：**FAST-LIO2: Fast Direct LiDAR-inertial Odometry**

* **FAST-LIO2** (2022) — 算法提出
  * 作者：Xu 等 / TRO
  * 链接：https://arxiv.org/abs/2107.06829 · 本地：`article/建图/2107.06829v1.pdf`
  * 直接点到图 + ikd-Tree + 迭代卡尔曼紧耦合；仿真/开源里常作 **首选轻量 3D 里程计基线**。

**有用句子 / 要点（摘要）：**

* 两大新意：① **直接把原始点配准到地图**（不做特征提取），再更新地图；② 用增量 **ikd-Tree** 维护地图，支持高效插入/删除/最近邻。
* 目标：在有限机载算力下获得 **准确、低延迟** 的状态估计与稠密 3D 地图，供控制与轨迹规划。
* 开源：FAST-LIO / ikd-Tree（GitHub hku-mars）。

**与本项目相关性**：有多线点云 + IMU 后，优先于 LIO-SAM 作为仿真里程计黑盒；**不依赖其公式推导**，只接位姿输出。当前无 IMU 时让位于 KISS-ICP。

---

# 论文7 KISS-ICP：**KISS-ICP: In Defense of Point-to-Point ICP**

* **KISS-ICP** (2023) — 算法提出
  * 作者：Vizzo 等 / RA-L
  * 链接：https://arxiv.org/abs/2209.15397 · 本地：`article/建图/2209.15397v2.pdf`
  * **几乎无调参的纯激光点到点 ICP 里程计**；可不依赖 IMU —— 与当前官方 Gazebo 车配置最匹配。

**有用句子 / 要点（摘要）：**

* 与「越加越复杂」的里程计路线相反：删繁就简，聚焦 point-to-point ICP + 自适应阈值对应、鲁棒核、简单运动补偿、点云下采样。
* **同一组参数**可在汽车、无人机、Segway、手持 Livox 等多种平台/传感器上达到与 SOTA 相当的效果。
* **不要求 IMU**，仅依赖 3D 点云；开源系统在文中数据集上快于传感器帧率。

**与本项目相关性**：**仿真 P0 建图/里程计黑盒首选**；深度相机伪点云或后续多线插件即可试跑。TD3 第一版仍可不绑死该里程计。

---

# 论文8 LiDAR SLAM 综述：**LiDAR-based SLAM for robotic mapping: state of the art and new frontiers**

* **LiDAR-based SLAM …** (2023) — 综述
  * 作者：Yue, Zhang, He 等
  * 链接：https://arxiv.org/abs/2311.00276 · 本地：`article/建图/2311.00276v1.pdf`
  * 按 LiDAR 类型/配置梳理 LOAM 系、LIO-SAM、FAST-LIO 等脉络，写 related work「谁取代谁」用。

**有用句子 / 要点：**

* 综述意图：为机器人建图研究者提供 **按 2D/3D LiDAR 形态与配置** 的文献地图（作者称此类视角的全面调查较少）。
* Findings：未来趋势指向 **多机器人协同建图**、以及基于 **3D LiDAR + 深度学习的多源融合 SLAM**。
* 相对纯 2D 激光 SLAM 综述：本文覆盖 3D 与多配置，更贴「替 AMCL 平面假设」叙事。

**与本项目相关性**：已读作选型背景即可；**不必反复精读公式**。具体落地仍看 FAST-LIO2 / KISS-ICP / LIO-SAM 三选一黑盒表（见 [`3D/README0.md`](../3D/README0.md) §3）。

---

# 论文9（待精读）Global LiDAR Localization 综述：**A Survey on Global LiDAR Localization**

* 链接：https://arxiv.org/pdf/2302.07433
* **来源**：arXiv 公开摘要 / `article.md` 条目（PDF 未下，以下为摘要级要点，非页码引用）

**待精读摘要要点：**

* 主题是 **有先验图时的全局定位与重定位**（非纯里程计），含 BEV/学习描述子等。
* **对本项目**：对应「以后用 NDT/hdl 类 scan-to-map 替 AMCL」；**现在暂缓下载精读**，等做先验图定位再开。

---

# 论文10（待精读）DLIO：**Direct LiDAR-Inertial Odometry**

* 链接：https://arxiv.org/pdf/2203.03749
* **来源**：arXiv 公开页 / `article.md`（未下 PDF）

**待精读摘要要点：**

* 轻量 LIO + 连续时间去畸变；常与 FAST-LIO2 / LIO-SAM **同台对比**。
* **对本项目**：选型已定后作 ablation 对照；**暂缓**，不挡 TD3 闭环。

---

# 论文11（待精读）CT-ICP：**Real-time Elastic LiDAR Odometry with Loop Closure**

* 链接：https://arxiv.org/pdf/2109.12979
* **来源**：arXiv / `article.md`（未下 PDF）

**待精读摘要要点：**

* 连续时间弹性 ICP + 回环，精度强但工程更重。
* **对本项目**：`article.md` §4.4 已标不适合第一周基线；**暂缓**。

---

# 论文12（待精读）VoxelMap：**Efficient and Probabilistic Adaptive Voxel Mapping…**

* 链接：https://arxiv.org/pdf/2109.07082
* **来源**：arXiv / `article.md`（未下 PDF）

**待精读摘要要点：**

* 自适应体素地图表示，服务高精度里程计；**不是** nav costmap 开箱方案。
* **对本项目**：暂缓；导航用地图优先 OccupancyGrid / OctoMap 投影思路。

---

# 论文13（待精读）Scan Context：**Egocentric Spatial Descriptor for Place Recognition**

* 链接：https://doi.org/10.1109/iros.2018.8593953
* **来源**：公开条目 / `article.md`（未下 PDF）

**待精读摘要要点：**

* 手工全局描述子做地点识别/回环，易挂到 LOAM/LIO/FAST 系。
* **对本项目**：**第二阶段中后期**（加回环时）再读；现在不下。

---

# 论文14（待精读）OverlapTransformer

* 链接：https://ieeexplore.ieee.org/document/9785497
* **来源**：IEEE / `article.md`（未下 PDF）

**待精读摘要要点：**

* Transformer 学习型回环/重识别；数据与训练成本高于 Scan Context。
* **对本项目**：暂缓；创新向回环再考虑。

---

# 论文15（待精读·建议本周读）Sensors 2023 对照：**A Comparison Study between Traditional and Deep-Reinforcement-Learning-Based Algorithms for Indoor Autonomous Navigation in Dynamic Scenarios**

* 链接：https://pmc.ncbi.nlm.nih.gov/articles/PMC10748032/
* **来源**：PMC 公开页 / `article.md`（本地未见 PDF，建议本周打开网页精读实验设计）

**待精读摘要要点：**

* 对照 **DWA、TEB、CADRL、SAC** 等；结论随场景分化，避免「DRL 一定更好」。
* **对本项目**：写实验/ablation 的模板文；与 Arena 一起，是第二周条目里 **唯一建议马上读** 的一篇（可只读方法与实验表，不必等建图论文）。

