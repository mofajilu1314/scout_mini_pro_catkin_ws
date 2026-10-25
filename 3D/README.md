# 3D

> **版本与安全注记（2026-10-25）**：这是完整 3D 工作说明书的入口，不是删节索引；公式、操作、验证证据与设计推理分别保留在下列分册。所有 `$SCOUT_WS_ROOT` 均指向本工作区根目录；先执行 `export SCOUT_WS_ROOT=/你的/catkin_ws_2/绝对路径`。公开版本不写入服务器凭据、设备地址或实车登录信息；这些受控资料只保留在本地，不随 GitHub 导出。

**22.04 / Humble 用户 → [`HUMBLE_QUICKSTART.md`](HUMBLE_QUICKSTART.md)**（复制粘贴命令）

**文档体系（按职责分册）：**

| 册 | 内容 |
|----|------|
| **[README0.md](README0.md)** | 总规划 + 执行路标（两块架构、阶段 A/B/C、下一步） |
| **[README1.md](README1.md)** | **TD3 训练**：环境、参数、换图、spawn、推理、服务器 |
| **[README2.md](README2.md)** | **科研出图**：曲线 / 定轨轨迹 / 热力图 / RViz 演示动图（论文·答辩用） |
| **[README3.md](README3.md)** | **控制链路说明** + **ScoutNav 仿真操作手册** + 安全边界／自检／录包说明（受控现场连接资料不在公开版） |
| **[README4.md](README4.md)** | **长途导航**：分层 L1–L5 · traversability/Hybrid A\*/Arena 公式 · **安全红线无限代价** · 仿真→实车 · 文献精华卡 |

调试踩坑 → **[DEBUG_RECORDS.md](DEBUG_RECORDS.md)** · 训练记录 → **[TRAIN_RECORDS.md](TRAIN_RECORDS.md)**。旧草稿与历史迁移资料保留在本地归档区，不随 GitHub 导出；当前分册已经保留可执行的公式、操作与验证说明。
