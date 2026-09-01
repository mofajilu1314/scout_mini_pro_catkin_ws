# 3D · 第二阶段文档索引

面向 **越野 / 非结构化地形**：深度相机几何感知 + 深度强化学习（TD3）+ 2.5D 可通行性规划。  
**本目录只放文档与后续笔记；可运行代码请放到 `src/` 下独立 ROS 包**（建议名 `scout_offroad_3d`），避免和第一阶段 `nav_bringup` 搅在一起，打包更干净。

> **日常请读 [`README0.md`](README0.md)**（总规划已集成）。下列旧分册保留作历史。  
> 第一阶段 2D 基线 → [`../2D/`](../2D/README.md)  
> 文献 PDF / 调研 → [`../article/`](../article/)（保持不动）

## 文档列表

| 文件 | 内容 |
|------|------|
| **[README0.md](README0.md)** | **日常总规划**（背景、选型、三阶段、唯一下一步） |
| [3D_README.md](3D_README.md) | （历史）总纲：方向拍板、与 2D 差异、成功标准 |
| [3D_README_1.md](3D_README_1.md) | （历史）开源资料取舍（ORFD、LIO-SAM、DRL-AUV 等） |
| [3D_README_2.md](3D_README_2.md) | （历史）目标架构、话题约定、TD3 状态/动作 |
| [3D_README_3.md](3D_README_3.md) | （历史）里程碑 M1～M5、验收与风险 |

建议：先读 **README0**；需要细节再翻旧分册。

## 推荐代码落盘（尚未创建时可按此规划）

```text
src/scout_offroad_3d/     # 新建 catkin 包：越野 world、深度特征、TD3 训练/推理
3D/                       # 只放 md / 设计图 / 训练笔记（可选再加 notes/）
article/                  # 论文 PDF 仍放这里
```

当前工作区**尚未**创建 `scout_offroad_3d`；以 `3D_README_3.md` 的 M1 为下一开发动作。
