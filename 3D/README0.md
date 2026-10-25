# 3D / 越野导航 · 总规划 + 执行路标（README0）

> **版本与安全注记（2026-10-25）**：本册保留原始规划、阶段边界和推理链；它不是简略替代版。所有当前命令以 `$SCOUT_WS_ROOT` 为工作区根，当前 3D 源码位于 `ros2_ws/src/navigation_3d/`，共享模型位于 `ros2_ws/src/shared/`。2D 已是外置仓库，不能把本地旧 2D 路径当作当前依赖。

**22.04 / ROS2 Humble → [`HUMBLE_QUICKSTART.md`](HUMBLE_QUICKSTART.md)** · 训练 [`README1.md`](README1.md) · 出图 [`README2.md`](README2.md) · 实车 [`README3.md`](README3.md) · **长途 B/C [`README4.md`](README4.md)**

详见 [`README1.md`](README1.md) 文首 **ROS2 Humble 快速命令**（§二～§七 已改 Humble；§4.0-b 等为 Noetic 历史存档）。  
调试踩坑：[`DEBUG_RECORDS.md`](DEBUG_RECORDS.md)。  
文献清单仍在 [`article/article.md`](../article/article.md)。长途导航启发（点云→代价图 / Hybrid A\* / Arena）：[`README4.md`](README4.md)。旧草稿只在本地受控归档中保留，不作为当前或公开入口。

| 基线 | [外置 2D 仓库 `scout-navigation-2d`](https://github.com/mofajilu1314/scout-navigation-2d)（官方车 2D 历史/对照） |
|------|-----------------------------------------------|
| ROS1 代码包 | `src/scout_offroad_3d` |
| ROS2 迁移包 | `ros2_ws/src/navigation_3d/scout_offroad_3d`（Phase 4–5 可在线训练） |

**进度**：步骤1 官方 2D ✅ · 步骤2 扇区 ✅ · 步骤3 阶段 A TD3 ✅（**Humble** `train_td3.launch.py` + `humble_train_smoke.sh`；Noetic 基线仍可用）· Arena/A\*/SLAM ❌

---

## 当前唯一下一步

**22.04 用户**：按 [`HUMBLE_QUICKSTART.md`](HUMBLE_QUICKSTART.md) 跑 `humble_train_smoke.sh` 冒烟，再 `ros2 launch … train_td3.launch.py` 长跑。操作细节见 [`README1.md`](README1.md) 文首 Humble 快速命令。

完整可复制命令：

**[`HUMBLE_QUICKSTART.md`](HUMBLE_QUICKSTART.md)** · **[`README1.md` · ROS2 Humble 快速命令](README1.md#ros2-humble-quick)** · **[README1 · 第四节 TD3 训练（Noetic）](README1.md#td3-train)** · **[第五节 TD3 推理](README1.md#td3-infer)** · **[第六节故障](README1.md#td3-troubleshoot)** · **[第七节服务器](README1.md#td3-server)** · **[逻辑与选型](README1.md#td3-logic)**

阶段 A **不要** SLAM / `map.yaml` / Arena / A\*；**块2 以后再做**。

---

## 0. 术语

| 说法 | 含义 | 阶段 A |
|------|------|--------|
| Gazebo **世界** | `.world` 训练场 | **要** |
| SLAM **地图** | `map.yaml` / LIO 图 | **不要** |
| A\* / Arena 子目标 | 分层上层 | 最终要；A 可不要 |

---

## 1. 两块架构（强制）

| 块 | 内容 | 何时 |
|----|------|------|
| **块1 强化学习实时控制** | TD3：扇区 + 目标相对位姿 → `(v,ω)` → `/cmd_vel` | **阶段 A（现在）** |
| **块2 建图/SLAM + Arena 子目标 + 粗全局规划** | local_goal、A\*、可选建图黑盒 | **阶段 B/C**，不阻塞 A |

```text
[块2 粗路径 → /offroad/local_goal]     ← B/C
        ↓
[块1 TD3] 扇区+(ρ,θ)+上拍动作 → /cmd_vel  ← A 起
```

诚实边界：**未**集成 Arena+A\*+SLAM+TD3 四件套；阶段 A = 局部 TD3 训练。

---

## 2. 背景与选型（摘要）

室内 2D：`LaserScan → 栅格 → AMCL → A* → Pure Pursuit → cmd_vel`。  
越野：可通行性 ≠ 2D 墙；局部适合学习策略；**不**在 A 同时开语义 CNN + 自研 LIO + RL。

| 局部算法 | **TD3**（双 Q、延迟更新、目标平滑）→ `(v,ω)` |
|----------|-----------------------------------------------|
| 感知 A | `/scan` → `/offroad/range_features`（非 CNN） |
| 位姿 A | Gazebo 真值或 `/odom` |
| 分层模板 | Arena-Rosnav 思路 → B/C |

**建图黑盒（块2，阶段 C）拍板**：仿真现阶段（有 `/imu/data` 但点云主为深度/2D scan）**不上** FAST-LIO2/LIO-SAM 主链；旁路可试 **KISS-ICP**。真车多线+IMU：**主推 FAST-LIO2**，对照 LIO-SAM，轻量对照 KISS-ICP。详见 [README1 §选型](README1.md#stack-choice)。

**粗全局规划**：室内仿真继续 **2D A\***；越野阶段 C 用 **可通行性代价 + Hybrid A\***（或同类）。阶段 B 只接 Arena 式 `/offroad/local_goal`，不阻塞 A。

TD3 原文 [arXiv:1802.09477](https://arxiv.org/abs/1802.09477) · 代码仿 [sfujim/TD3](https://github.com/sfujim/TD3)（MIT）· ROS 环参考 [DRL-AUV-navigation](https://github.com/lanyangyang-gogogo/DRL-AUV-navigation)（改地面车）。

状态 / 动作（阶段 A）：

- `s = [d_1…d_N, ρ, θ, v_prev, ω_prev]`（N=24 → **28 维**；距离归一化）
- `a = (v, ω)` → `/cmd_vel`（**2 维**）
- 奖励：靠近 +、到达 ++、碰撞大罚、步进小罚

---

## 3. 执行步骤（勾选）

| 步骤 | 内容 | 状态 |
|------|------|------|
| 1 | `nav_all_in_one_official.launch` 到点 | ✅ |
| 2 | `/scan` → `/offroad/range_features` | ✅ |
| 3 | **阶段 A** 最小 TD3 → `/cmd_vel`，导出权重 | ✅ 骨架；收敛调参进行中 |
| 4 | **阶段 B** Arena 式 `/offroad/local_goal` | 待 |
| 5 | **阶段 C** 全局规划 + 可选建图黑盒 | 待 |

成功标准（步骤3）：随机目标有到达趋势；碰撞能 `done`；能导出 `td3_actor` / `td3_critic`。

### 阶段砍刀

| 阶段 | 交付 | 不做 |
|------|------|------|
| **A** | 扇区 + 随机目标 + TD3 → `/cmd_vel` | A\*、Arena、`map.yaml`、LIO 公式、CNN |
| **B** | 粗引导 → local_goal → TD3 | 一上来完整 Arena 全家桶 |
| **C** | 2D 与越野全局对照；建图旁路 | 重写 SLAM |

命令与验收 → **[`README1.md`](README1.md)**（勿在本文件重复长操作文）。踩坑 → **[`DEBUG_RECORDS.md`](DEBUG_RECORDS.md)**。

---

## 4. 位姿黑盒（后期 · 与上表一致）

阶段 A：Gazebo / `/odom`。仿真旁路可试 KISS-ICP；真车后期 **FAST-LIO2 主 / LIO-SAM 对照**。官方仿真无多线点云 → 勿在 A 硬接 FAST-LIO2。

---

## 5. 下一步（展开）

1. **22.04**：`humble_train_smoke.sh` 冒烟 → `ros2 launch … train_td3.launch.py` 长跑，看 `train.log` 到达率；**20.04**：按 [README1 §四](README1.md#td3-train) `train_td3.launch`  
2. 多 world / 随机障碍（减过拟合）  
3. 再进阶段 B（local_goal），勿假装已接 Arena；B/C 算法与文献书单见 [`README4.md`](README4.md)
