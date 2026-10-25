# 工作区备份与查找索引

最后核对：2026-10-25。

本文件回答“某个模块现在在哪里、有没有上传、以后从哪里找”。本地工作区仍是当前 3D 工作的完整保存位置；没有因为上传 GitHub 而删除本地源码、文档、归档或 Git 历史。

## GitHub 位置

| 内容 | GitHub 位置 | 状态 |
|---|---|---|
| 2D 独立模块 | https://github.com/mofajilu1314/scout-navigation-2d | 已上传；本机不再作为活动 2D 工作区 |
| 3D TD3／长途导航模块、完整操作手册和公式 | https://github.com/mofajilu1314/scout-navigation-3d | 已上传；不含私有实车资料、全部历史实验权重或一次性日志 |
| 原始根工作区历史仓库 | https://github.com/mofajilu1314/scout_mini_pro_catkin_ws | 已存在，但当前网页上的 `main` 仍是整理前的历史版本；仓库当前为 Public |

## 本地完整保存位置

- 工作区根：`/home/test/catkin_ws_2/`
- 当前可运行 3D 源码：`/home/test/catkin_ws_2/ros2_ws/src/navigation_3d/`
- 当前 3D 操作手册、脚本、地图和界面资源：`/home/test/catkin_ws_2/3D/`
- 原始详细 README、公式和历史说明：`/home/test/catkin_ws_2/archive/docs/3d_historical/`
- Record 9 仿真推理基线和历史 TD3 实验：`/home/test/catkin_ws_2/archive/experiments/td3/`
- 私有实车交接资料：`/home/test/catkin_ws_2/archive/private/`（只留本地，不上传）
- 待人工复核删除的一次性产物：`/home/test/catkin_ws_2/archive/pending_deletion/20261025/`

## 根工作区 Git 快照状态

本地已创建一份不改动现有暂存区的快照提交：

```text
分支：codex/workspace-cleanup-snapshot-20261025
提交：2e160496ff2035f48a960ad1186219a3341736a3
内容：当前 ROS2 3D 源码、完整活动手册、模块地图与整理记录
```

该快照已推送并核对到根工作区 GitHub 的独立分支：

```text
https://github.com/mofajilu1314/scout_mini_pro_catkin_ws/tree/codex/workspace-cleanup-snapshot-20261025
```

根仓库目前为 Public；该快照没有包含 `archive/private/`、待删区、原始未脱敏手册、历史实验权重或论文 PDF。公开的活动 3D 代码与完整操作手册则已另外保存在 `scout-navigation-3d`。

## 不删除原则

- 上传 GitHub 是副本，不替代本地工作区。
- 未确认前，不删除 `.git/`、原始 README、公式、代码解释、地图、world、私有资料或 Record 9。
- 只有 `archive/pending_deletion/20261025/` 内的内容在人工复核后可删除；删除前请以该目录的 README 为准。
