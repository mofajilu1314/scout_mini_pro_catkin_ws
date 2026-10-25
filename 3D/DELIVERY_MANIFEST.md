# 3D 独立交付清单

本清单定义未来单独上传或交付 3D 导航链时的边界。它面向 ROS2 Humble + Gazebo Classic 仿真源码，不等同于实车部署包。

## 必须包含

```text
ros2_ws/src/navigation_3d/scout_offroad_3d/
ros2_ws/src/navigation_3d/scout_long_nav/
ros2_ws/src/shared/scout_description/
ros2_ws/src/shared/scout_mini_pro_description/
ros2_ws/src/shared/scout_workspace_common/
ros2_ws/src/third_party/                 # 只包含依赖说明/安全克隆脚本
3D/                                       # 已清理的 3D 文档、脚本、地图和 UI 资源
```

其中 `scout_offroad_3d` 是 TD3 局部导航链，`scout_long_nav` 是长距离分层链。三个 shared 包是运行所需的共同依赖，不能分别维护第二份副本。

## 默认排除

- 所有 2D 源码、2D 文档、2D 验证产物；
- `archive/` 内的历史实验、ROS1、备份资产和本地私有资料；
- `3D/runs/`、`3D/runs_long/`、`3D/debug_logs/`、`.last*`、`.boundary_live.json`、缓存、构建目录和日志；
- 含实车连接信息的文档、默认会启用实车控制的材料，以及未经公开审查的 `real_*` 配置；
- 嵌套 Git 元数据、环境文件、密钥和未审查的第三方源码。

## 可选仿真基线

默认不带权重。需要仿真基线时，只能显式选择 Record 9，并带以下最小兼容集：

```text
td3_actor
td3_actor_optimizer
td3_critic
td3_critic_optimizer
config.yaml
params_backup.yaml
BASELINE.md                 # 150 维 / hidden 1024 / 仅仿真
```

完整的 Record 9 实验目录属于归档，不进源码仓库；特别是可视化大产物不应随 GitHub 源码提交。任何权重都不构成实车安全声明。

## 导出与发布闸门

1. 从根目录运行 `tools/export_navigation_module.sh 3d <空目录>`；只有显式 `--with-baseline` 才导出最小仿真基线。
2. 对导出目录检查：没有 `navigation_2d`、`2D/`、`archive/private`、历史 run、日志、缓存或密钥；所有文档链接和工作区路径均有效。
3. 在干净目录中完成包发现、构建和无 GUI 3D 最小冒烟，并保存验证报告。
4. 在创建 GitHub 仓库前单独决定许可证、可见性和大文件策略；没有许可证时不要默认宣称可再分发。

依赖、外部模型和未验证项目必须在交付 README 中明确标为外部前提，不应伪装为随包可运行。
