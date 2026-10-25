# ROS2 活动工作区规则

本目录是 Scout 的唯一活动 ROS2 工作区；根目录的 [`../AGENTS.md`](../AGENTS.md) 与 [`../MODULE_MAP.md`](../MODULE_MAP.md) 优先于本文件。

## 源码归属

- 只在 `src/shared/`、`src/navigation_3d/`、`src/third_party/` 新增或修改可编译源码。
- 共享车辆描述和路径工具只保留一份，放在 `src/shared/`；2D 或 3D 不得复制共享源码。
- TD3、可通行图、Hybrid A* 与长途链只属于 `navigation_3d/`。2D 已由独立 GitHub 仓库维护，不是本机 `colcon` 的组成部分。
- 历史 ROS1、旧世界、`.bak*`、旧训练记录和实验权重不参与 `colcon`，应有索引地放入根目录 `archive/`。

## 路径、输出与安全

- 运行时路径按 `SCOUT_WS_ROOT`、`CATKIN_WS`、`WS_ROOT`、ROS package share 的顺序解析；不得新增用户目录、旧顶层 `src/` 或 `.cursor/` 的硬编码路径。
- 地图、world、模型、权重和验证日志均通过工作区根或 package share 获取；调试日志只写调用模块的 `debug_logs/`。
- 训练默认必须新建唯一 `run_*` / `smoke_*` 目录；非空输出目录一律拒绝。只有显式 `resume:=true` 且检查通过时才能恢复。
- 不得按进程名、端口或 `killall`/`pkill` 清理未知进程；只管理本次启动且可确认归属的子进程。
- 所有实车控制保持默认关闭；基础验证仅运行仿真和无 GUI 冒烟。

## 变更与验证

- 移动文件前先更新 `MODULE_MAP.md` 和相关 README 的引用，再做小范围构建或冒烟验证。
- 从本目录执行 `colcon`；构建输出 `build/`、`install/`、`log/` 不纳入源码包或 GitHub 交付包。
- 每次验证独立记录命令、时间、环境、结果、失败原因和未满足的外部依赖。
- 准备独立 3D 交付时，只带两个 3D 包、三个共享包、必要的第三方说明及 `3D/` 配套资源；不得带入 2D、历史实验、文章或敏感信息。
