# 工作区模块地图与迁移索引

这是 `catkin_ws_2` 的活动结构总览。它回答三个问题：什么文件应放在哪里、本机 3D 依赖什么、需要独立交付时应怎样导出。2D 已从本机活动工作区退役，独立维护在 [scout-navigation-2d](https://github.com/mofajilu1314/scout-navigation-2d)。本地保留位置、GitHub 仓库和快照状态见 [`BACKUP_LOCATIONS.md`](BACKUP_LOCATIONS.md)。

当前运行目标是 **Ubuntu 22.04 + ROS2 Humble + Gazebo Classic**。`archive/` 中的内容仅作历史参考，不能作为当前命令或运行时依赖。

## 目标目录

```text
catkin_ws_2/
├── AGENTS.md
├── MODULE_MAP.md
├── 3D/                              # 3D 文档、脚本、地图、活跃 run 与调试日志
├── archive/
│   ├── legacy_ros1/                 # ROS1 兼容层与旧顶层 src
│   ├── legacy_compat/               # 已淘汰的 ROS2 转发入口
│   ├── docs/                        # 历史说明、迁移旧文
│   ├── experiments/td3/             # 历史 run_*、runs_long 与实验索引
│   ├── validation/                  # 根目录遗留的构建/验收日志
│   ├── pending_deletion/            # 已判定为可再生产物，待用户复核后手工删除
│   ├── asset_backups/               # .bak world、旧地图快照与清理记录
│   └── private/                     # 本地实车交接与连接资料；不入版本库/导出包
├── ros2_ws/src/
│   ├── shared/
│   │   ├── scout_description/
│   │   ├── scout_mini_pro_description/
│   │   └── scout_workspace_common/
│   ├── navigation_3d/
│   │   ├── scout_offroad_3d/
│   │   └── scout_long_nav/
│   └── third_party/
├── tools/
└── article/                         # 独立研究资料，不参与运行时依赖
```

`colcon` 会递归发现这些嵌套 ROS 包；运行构建时仍从 `ros2_ws/` 执行。
2D 的源码、说明和运行入口不属于本机 `colcon` 工作区；需要使用或维护时，请进入其独立 GitHub 仓库。

## 活动模块边界

| 模块 | 活动源码 | 配套文件 | 直接依赖 | 不应混入 |
|---|---|---|---|---|
| 2D 室内导航（外置） | [scout-navigation-2d](https://github.com/mofajilu1314/scout-navigation-2d) | 独立仓库 | 其仓库内的共享副本 | 本机 3D 工作区、构建产物与日志 |
| 3D 越野 TD3 | `ros2_ws/src/navigation_3d/scout_offroad_3d/` | `3D/` | 共享车辆模型、路径工具 | ROS1 兼容层、2D 产物 |
| 3D 长途导航 | `ros2_ws/src/navigation_3d/scout_long_nav/` | `3D/maps_long/`、3D 文档 | 3D TD3、共享车辆模型 | 旧 `runs_long` 结果 |
| 共享层 | `ros2_ws/src/shared/` | 无独立运行产物 | 无 | 模块专属算法和地图 |

```text
scout_description + scout_mini_pro_description + scout_workspace_common
                               ↓
                      scout_offroad_3d
                               ↓
                        scout_long_nav
                               ↓
                        3D/scout_nav_app
```

`scout_workspace_common` 统一解析工作区根：优先 `SCOUT_WS_ROOT`，兼容 `CATKIN_WS`、`WS_ROOT`，最后由 ROS 包位置推导。活动代码不应再依赖用户目录或旧顶层 `src/`。

## 迁移对照

| 原位置 | 目标位置 | 处理与用途 |
|---|---|---|
| `ros2_ws/src/scout_description/` | `ros2_ws/src/shared/scout_description/` | 活动共享车辆模型 |
| `ros2_ws/src/scout_mini_pro_description/` | `ros2_ws/src/shared/scout_mini_pro_description/` | 活动共享仿真车与环境 |
| 新建路径工具 | `ros2_ws/src/shared/scout_workspace_common/` | 活动共享路径解析 |
| 原本机 2D 活动路径 | [scout-navigation-2d](https://github.com/mofajilu1314/scout-navigation-2d) | 2026-10-25 已外置；本机不再保留 2D 源码或运行入口 |
| `ros2_ws/src/scout_offroad_3d/` | `ros2_ws/src/navigation_3d/scout_offroad_3d/` | 活动 3D TD3 ROS 包 |
| `ros2_ws/src/scout_long_nav/` | `ros2_ws/src/navigation_3d/scout_long_nav/` | 活动 3D 长途导航 ROS 包 |
| `3D/ros1_scout_td3/`、旧顶层 `src/`、`scout_description/launch/*.launch` | `archive/legacy_ros1/` | 保留 ROS1 历史兼容层，不参与 Humble 构建 |
| `scout_mini_pro_description/launch/nav_all_in_one_official.launch.py` | `archive/legacy_compat/scout_mini_pro_description/` | 旧共享包→2D 转发入口；2D 已有独立官方启动入口，不再保留反向耦合 |
| `3D/archive_readmes/`、旧迁移说明 | `archive/docs/` | 历史参考，不作当前入口 |
| `ros2_humble/` | `archive/docs/ros2_humble_historical/` | 2026-09 的迁移快照；包数、路径和功能状态已过期，不能作当前入口 |
| 原 `2D/README*`、文件结构说明与调试记录 | `archive/docs/2d_historical/active_replaced_20261020/` | 保留原文以便追溯；当前 2D 入口已外置到独立仓库 |
| 原 `3D/README0`–`README4`、旧快速上手、训练/调试记录与 R8/R9 对照 | `archive/docs/3d_historical/active_replaced_20261020/` | 保留原文以便追溯；活动 `3D/` 维护已校审的**完整**公式、操作、命令与验证手册，只局部修正旧路径、隐式续训或广泛杀进程命令 |
| 含真实车接入信息的交接文档 | `archive/private/` | 本地受控资料；公开文档仅保留脱敏的安全说明 |
| `3D/runs/run_*`、`3D/runs_long/` | `archive/experiments/td3/` | 保留历史实验原件与索引 |
| `*.bak*`、旧 world、清理记录 | `archive/asset_backups/` | 仅保留溯源用途 |
| `3D/maps_long/*_backup_*` | `archive/asset_backups/maps_long/` | 已被工作地图替代的时刻快照；活动地图只保留明确索引的工作图和命名快照目录 |
| 工作区根 `.catkin_workspace` | `archive/legacy_ros1/workspace_markers/` | 旧 Catkin 识别标记；活动 ROS2 根不再保留它 |
| 工作区根 `log/` | `archive/validation/root_colcon_logs_20261020/` 与按日期的 `archive/validation/YYYYMMDD/` | 根目录旧 Colcon/版本检查日志；运行期间日志可暂存于 `3D/debug_logs/`，结束后统一归档 |
| 工作区根 `tmp/pdfs/` | `article/导航/预览图/` | 研究文献首页预览，不属于运行时资产 |
| `ros2_ws/.pytest_cache/` | `archive/asset_backups/ros2_ws_pytest_cache_20261020/` | 整理前测试缓存，保留以便溯源但不作为活动状态 |
| `3D/gzsim_demo/`、`3D/scripts/gzsim_demo.sh` | `archive/legacy_compat/gzsim_demo_20261020/` | 独立 Gazebo Sim/Fortress 演示；当前运行基线是 Gazebo Classic，不能混作活动 3D 链路 |
| 空 `3D/maps/long_nav/`、`3D/runs_long/` | `archive/legacy_compat/empty_placeholders_20261020/` | 已被 `3D/maps_long/` 与 `archive/experiments/td3/runs_long/` 取代的空占位 |
| 一次性验证产物、缓存、过期探针 | `archive/validation/20261025/` 等归档区 | `archive/pending_deletion/20261025/` | 仅暂存待用户复核的可再生内容；不是活动输入，也不包含原始手册、地图、world 或 Record 9 |
| `module_exports/` | 不保留在工作区 | 以脚本按需生成导出包 |

逐项执行状态和可追溯移动清单见 [`MIGRATION_MANIFEST.md`](MIGRATION_MANIFEST.md)。

## 训练记录与基线

- 活动 `3D/runs/` 只在运行期间容纳新的 `run_*`、`smoke_*` 与验证输出；短测结束后移至 `archive/validation/YYYYMMDD/`，不保留历史 `runs_long/` 空目录。
- 全部历史训练与长途结果保存在 `archive/experiments/td3/`；索引需记录原始目录、时间戳、Record 编号、状态、关键配置与权重完整性。
- `3D/maps_long/` 仅保留当前工作图、明确命名的地图快照目录和索引；散落的 `*_backup_*` 资产转入 `archive/asset_backups/maps_long/`，不作为运行入口。
- `3D/README*.md` 是当前入口；同名主题的长篇旧手册、训练流水账和调试记录已转入 `archive/docs/3d_historical/active_replaced_20261020/`，只可作为历史证据读取，不能复制其中命令执行。
- Record 9：`archive/experiments/td3/runs/run_20260907_120841/` 是默认仿真推理基线。它是训练实验权重，**不可直接用于实车控制**。
- 当前活动 TD3 状态为 150 维；历史 28 / 76 / 112 维只保留在训练史中，不能混作当前配置。

## 按需导出，而非维护副本

在已构建或准备导出前设置工作区根：

```bash
cd /path/to/scout-workspace
export SCOUT_WS_ROOT="$PWD"
bash "$SCOUT_WS_ROOT/tools/export_navigation_module.sh" 3d /目标目录 --with-baseline
```

- 2D 已是独立 GitHub 仓库，不再从本机导出。
- 3D 导出：3D TD3 与长途导航 ROS 包、共享模型、3D 文档/地图/脚本；`--with-baseline` 才附带 Record 9 仿真基线。
- 3D 的精确公开交付边界见 [`3D/DELIVERY_MANIFEST.md`](3D/DELIVERY_MANIFEST.md)；它排除 2D、归档、私有实车资料和未经审查的大型运行产物。
- 默认不导出全部历史实验、`runs_long` 或 `article/`；如有需要，先通过归档实验索引选择具体 run。

## 变更检查

每次移动文件、修改路径、改变训练行为或新增导出内容后：

1. 更新本地图与对应活动 README。
2. 搜索活动目录中的旧路径、硬编码用户目录与失效链接。
3. 从 `ros2_ws/` 验证 `colcon` 能发现共享与 3D 包。
4. 将验证日志放入独立目录，不把调试日志或临时文件混入源码、`.cursor/` 或训练记录。
