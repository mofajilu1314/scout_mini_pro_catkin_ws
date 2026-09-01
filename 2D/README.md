# 2D · 第一阶段文档索引

室内 **2D 占据栅格** 建图与导航（规则代码：AMCL + A\* + Pure Pursuit）。  
对应代码在 `src/scout_mini_pro_description` 等包内；**本文档目录不参与 catkin 编译**。

> 第二阶段（3D / 强化学习）→ 见 [`../3D/`](../3D/README.md)  
> 论文与文献 → 见 [`../article/`](../article/)（勿往本目录塞 PDF）

## 阅读顺序

| 顺序 | 文件 | 内容 |
|------|------|------|
| 0 | [FILE_STRUCTURE.md](FILE_STRUCTURE.md) | 工作区 / 包内文件地图，「改 X 找 Y」 |
| 1 | [README_1.md](README_1.md) | 自制 Scout 模型、传感器话题、空世界 / RViz |
| 2 | [README_2.md](README_2.md) | room 场景、传感器可视化、slam_toolbox 建图存图 |
| 3 | [README_3.md](README_3.md) | **导航主手册**：一键启动、调参、故障排查 |
| 4 | [README_4.md](README_4.md) | 官方 Scout 真车模接入与 official 导航（= 越野 **步骤1 / 2D**） |
| — | [debug_records.md](debug_records.md) | 现场踩坑流水账（按关键词搜索） |

**与越野后续衔接**：步骤1（上表 README_4）完成后 → 步骤2 感知扇区见 [`../3D/README1.md`](../3D/README1.md)；总计划 [`../3D/README0.md`](../3D/README0.md)。

## 常用命令（提醒）

```bash
cd ~/catkin_ws_2 && source devel/setup.bash
roslaunch scout_mini_pro_description nav_all_in_one.launch          # 方块车
roslaunch scout_mini_pro_description nav_all_in_one_official.launch # 官方车
```

细节以 `README_3.md` / `README_4.md` 为准。
