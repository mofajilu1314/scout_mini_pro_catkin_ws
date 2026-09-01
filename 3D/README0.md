# 3D / 越野导航 · 总规划 + 执行路标（README0）

**日常只看本文件定方向；怎么 launch / 训 TD3 看 [`README1.md`](README1.md)。**  
调试踩坑：[`DEBUG_RECORDS.md`](DEBUG_RECORDS.md)。  
文献清单仍在 [`article/article.md`](../article/article.md)。旧草稿：[`archive_readmes/`](archive_readmes/)。

| 基线 | [`2D/README_4.md`](../2D/README_4.md) 官方车 2D |
|------|-----------------------------------------------|
| 代码包 | `src/scout_offroad_3d` |

**进度**：步骤1 官方 2D ✅ · 步骤2 扇区 ✅ · 步骤3 阶段 A TD3 骨架 ✅（可 `train_td3.launch`）· Arena/A\*/SLAM ❌

---

## 当前唯一下一步

→ **按操作手册跑通阶段 A TD3（训练 → 看 log → 推理）**，不要先做 SLAM / 块2。

完整可复制命令：

**[`README1.md` · 第四节 TD3 训练](README1.md#td3-train)** · **[第五节 TD3 推理](README1.md#td3-infer)** · **[第六节故障](README1.md#td3-troubleshoot)** · **[第七节服务器](README1.md#td3-server)** · **[逻辑与选型](README1.md#td3-logic)**

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
| **C** | 2D/越野全局对照；建图旁路 | 重写 SLAM |

命令与验收 → **[`README1.md`](README1.md)**（勿在本文件重复长操作文）。踩坑 → **[`DEBUG_RECORDS.md`](DEBUG_RECORDS.md)**。

---

## 4. 位姿黑盒（后期 · 与上表一致）

阶段 A：Gazebo / `/odom`。仿真旁路可试 KISS-ICP；真车后期 **FAST-LIO2 主 / LIO-SAM 对照**。官方仿真无多线点云 → 勿在 A 硬接 FAST-LIO2。

---

## 5. 下一步（展开）

1. 本机 / 服务器按 [README1 §四](README1.md#td3-train) 跑通 `train_td3.launch`，看 `train.log` 到达率  
2. 多 world / 随机障碍（减过拟合）  
3. 再进阶段 B（local_goal），勿假装已接 Arena
