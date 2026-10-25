# 工作区迁移清单

本清单记录本轮整理的**源位置 → 目标位置**及状态。移动必须保持可追溯：不删除训练数据、不覆盖已有文件，并在完成后核对源目录消失、目标目录完整、引用已校审。表中的 2026-10-20 记录保留当时的事实；当前活动状态以 2026-10-25 的外置记录为准。

状态说明：`已移动，待校审` 表示目标已到位、旧位置已清空（必要的空目录除外），但仍需由构建、路径检查或文档链接验证确认。最终验证结论应写入验证报告，而不是只凭本表推定可运行。

| 类别 | 原位置 | 目标位置 | 状态 | 说明 |
|---|---|---|---|---|
| 共享源码 | `ros2_ws/src/scout_description/` | `ros2_ws/src/shared/scout_description/` | 已移动，待校审 | 2D/3D 共用，活动源码只保留一份 |
| 共享源码 | `ros2_ws/src/scout_mini_pro_description/` | `ros2_ws/src/shared/scout_mini_pro_description/` | 已移动，待校审 | 2D/3D 共用仿真模型 |
| 共享源码 | 新建 | `ros2_ws/src/shared/scout_workspace_common/` | 已创建，待校审 | 统一解析工作区根与活动路径 |
| 2D 源码 | `ros2_ws/src/scout_offroad_2d/` | `ros2_ws/src/navigation_2d/scout_offroad_2d/` | 已移动，待校审 | 2D 独立 ROS2 包 |
| 3D 源码 | `ros2_ws/src/scout_offroad_3d/` | `ros2_ws/src/navigation_3d/scout_offroad_3d/` | 已移动，待校审 | TD3、点云特征与越野 world |
| 3D 源码 | `ros2_ws/src/scout_long_nav/` | `ros2_ws/src/navigation_3d/scout_long_nav/` | 已移动，待校审 | 可通行图、Hybrid A*、长途链 |
| 第三方 | `ros2_ws/src/third_party/` | `ros2_ws/src/third_party/` | 保持原位 | 不作为 2D/3D 私有源码移动 |
| ROS1 历史 | `3D/ros1_scout_td3/` | `archive/legacy_ros1/ros1_scout_td3/` | 已移动，待校审 | 保留兼容参考，不参与 Humble 构建 |
| ROS1 历史 | 顶层 `src/` | `archive/legacy_ros1/src/` | 已移动，待校审 | 旧 Noetic/ROS1 入口 |
| ROS1 历史 | `ros2_ws/src/shared/scout_description/launch/*.launch` | `archive/legacy_ros1/scout_description_launch/` | 已归档，待校审 | 6 个 ROS1 XML launch 经无活动引用检查后原样移动；不再被 ament 安装 |
| ROS2 兼容入口 | `ros2_ws/src/shared/scout_mini_pro_description/launch/nav_all_in_one_official.launch.py` | `archive/legacy_compat/scout_mini_pro_description/` | 已归档，待校审 | 共享描述包不再转发 2D；`scout_offroad_2d` 的官方 launch 已完成独立冒烟 |
| 历史文档 | `3D/archive_readmes/`、旧迁移说明 | `archive/docs/` | 已移动，待校审 | 迁移后标为历史参考 |
| 旧 ROS2 迁移入口 | `ros2_humble/` | `archive/docs/ros2_humble_historical/` | 已归档，待校审 | 2026-09 的包清单、路径与功能状态已过期，不再作为活动文档入口 |
| 当前入口替换前的 2D 文档 | `2D/README.md`、`README_1.md`–`README_4.md`、`FILE_STRUCTURE.md`、`debug_records.md` | `archive/docs/2d_historical/active_replaced_20261020/` | 已归档，待校审 | 原文完整保留；活动 2D 文档只含当前源码边界和安全验证入口 |
| 当前入口替换前的 3D 文档 | `3D/README0.md`–`README4.md`（保留脱敏 README3）、`HUMBLE_QUICKSTART.md`、`TRAIN_RECORDS.md`、`DEBUG_RECORDS.md`、`AUDIT_R8_vs_R9.md` | `archive/docs/3d_historical/active_replaced_20261020/` | 已归档，待校审 | 原文完整保留以便复盘；活动同名文档恢复为完整、经校审的公式、操作和验证手册，不能再用摘要替代 |
| 私有实车资料 | `3D/README3.md`、`3D/REAL_ROBOT_HANDOFF.md` 原件 | `archive/private/` | 已归档，待校审 | 原件保留在本地 Git 忽略区；活动 3D 目录仅保留脱敏安全说明 |
| 训练实验 | `3D/runs/run_*` | `archive/experiments/td3/runs/` | 已移动，待校审 | 原始文件、时间戳、Record 与权重必须完整 |
| 长途实验 | `3D/runs_long/` | `archive/experiments/td3/runs_long/` | 已归档，已校审 | 历史长途结果只在归档中保留；活动目录不再创建空占位 |
| 默认基线 | `3D/runs/run_20260907_120841/` | `archive/experiments/td3/runs/run_20260907_120841/` | 已移动，待校审 | Record 9，仿真基线，不可直接上实车 |
| 资产备份 | `*.bak*`、旧 world、清理记录 | `archive/asset_backups/` | 已移动，待校审 | 保留溯源，不作活动资产 |
| 地图备份 | `3D/maps_long/*_backup_*` | `archive/asset_backups/maps_long/` | 已归档，待校审 | 工作图已独立；10 个散落时刻备份经无引用检查后原样移动 |
| 临时导出 | `module_exports/` | 不保留 | 已移除，待校审 | 改为 `tools/export_navigation_module.sh` 按需导出 |
| 应用运行残留 | `3D/scout_nav_app/.last_*`、`.boundary_live.json` | `archive/asset_backups/scout_nav_app_runtime_20261020/` | 已归档，待校审 | 保留旧会话日志与边界状态；新日志、状态只在会话期间写入 `3D/debug_logs/`，结束后转入 `archive/validation/` |
| 旧工作区标记 | 工作区根 `.catkin_workspace` | `archive/legacy_ros1/workspace_markers/.catkin_workspace_20261020` | 已归档，待校审 | 唯一活动入口为 ROS2 `ros2_ws/`；原 Catkin 标记只作历史线索 |
| 根目录构建日志 | 工作区根 `log/` | `archive/validation/root_colcon_logs_20261020/` | 已归档，待校审 | 原根目录 Colcon list/version-check 记录；运行中的模块日志暂存于调试目录，结束后按日期归档 |
| 研究预览图 | 工作区根 `tmp/pdfs/` | `article/导航/预览图/pdfs/` | 已移动，待校审 | 论文资料，不参与 ROS2 运行或模块导出 |
| 测试缓存 | `ros2_ws/.pytest_cache/` | `archive/asset_backups/ros2_ws_pytest_cache_20261020/` | 已归档，待校审 | 可再生缓存，不作为活动工作区状态 |
| 外部仿真演示 | `3D/gzsim_demo/`、`3D/scripts/gzsim_demo.sh` | `archive/legacy_compat/gzsim_demo_20261020/` | 已归档，待校审 | Gazebo Sim/Fortress 独立演示；当前支持链为 Gazebo Classic |
| 空占位 | `3D/maps/long_nav/`、`3D/runs_long/` | `archive/legacy_compat/empty_placeholders_20261020/` | 已归档，待校审 | 分别由 `3D/maps_long/`、`archive/experiments/td3/runs_long/` 替代 |

## 验收后的应有状态

- `ros2_ws/src/` 只保留 `shared/`、`navigation_3d/`、`third_party/` 三个职责分区。
- 本机只保留 `3D/` 的活动说明、脚本、地图和**正在进行的**验证产物；已结束的验证产物都在 `archive/validation/`。2D 由独立 GitHub 仓库维护，不再承担本机运行或构建职责。
- `archive/experiments/td3/` 内有每个历史 run 的索引，包含原路径、Record、时间戳、关键配置、状态与权重完整性。
- Record 9 的新位置已被推理与长途导航示例引用，且文档明确其仅限仿真。
- 工作区根没有持续占空间的 `module_exports/`；导出脚本默认不携带全部历史实验和论文资料。
- 所有活动代码、脚本和文档均不依赖旧顶层 `src/` 或硬编码 `/home/test/catkin_ws_2`。

## 2026-10-25 2D 外置记录

- 本机活动 2D 目录 `2D/`、`ros2_ws/src/navigation_2d/`、`ros2_ws/build/scout_offroad_2d/` 与 `ros2_ws/install/scout_offroad_2d/` 已移除；历史 2D 文档仍保留在 `archive/docs/2d_historical/`，没有被删除或改写。
- 当前 2D 独立入口为 [scout-navigation-2d](https://github.com/mofajilu1314/scout-navigation-2d)。已核对该仓库包含 2D 文档、源码包和必要共享依赖，且不含 `build/`、`install/`、`log/` 或运行日志。
- 本机 `colcon` 当前发现 5 个活动包：三个共享包、`scout_offroad_3d` 与 `scout_long_nav`。3D 活动 launch、源码与包依赖中没有对已外置 2D 包的运行时引用。

## 2026-10-25 验证产物归档记录

| 类别 | 原位置 | 目标位置 | 状态 | 说明 |
|---|---|---|---|---|
| TD3 短训练冒烟 | `3D/runs/smoke_20261025_020335/` | `archive/validation/20261025/3d_td3_smoke/` | 已移动，已记录 | `humble_train_smoke.sh --no-build` 通过；它只验证链路，不是基线或可续训实验 |
| 3D 调试与验证日志 | `3D/debug_logs/` | `archive/validation/20261025/3d_debug_logs/` | 已移动，已记录 | 包含历史 Humble、Gazebo、切图、绘图、会话安全与本轮短训日志；活动目录已清空 |
| ROS2 构建日志 | `ros2_ws/log/` | `archive/validation/20261025/ros2_colcon_logs/` | 已移动，已记录 | 包含旧 build/list/graph 日志与相对 `latest*` 链接；不是运行依赖 |
| 根目录临时日志 | `log/` | `archive/validation/20261025/root_colcon_logs/` | 已移动，已记录 | 一次性 `colcon list` 输出；根目录已清空 |
| 遗留 ROS2 内层日志目录 | `ros2_ws/3D/debug_logs/` | `archive/validation/20261025/legacy_ros2_debug_logs/` | 已移动，已记录 | 不属于当前模块边界的空/残余验证目录 |
| 整理后构建与离线检查产物 | 本轮 `ros2_ws/log/`、源码内 `__pycache__/` 与 `.pytest_cache/` | `archive/validation/20261025/post_cleanup_static_validation/` | 已移动，已记录 | 5 包构建、42 项单测、13 个 launch 参数加载和两条离线长途导航入口均通过；生成缓存不留在活动源码内 |
| 过期验证与一次性爬坡脚本 | `3D/scripts/smoke_r9_verify.sh`、`_run_climb_probe.sh`、`_climb_probe_once.py` | `archive/validation/20261025/stale_one_off_scripts/` | 已归档，已复验 | 旧 smoke 仍断言 `r_arrive=200`（当前为 700）；其余为无文档入口的一次性自动爬坡探针，均不删除但不再留作活动命令 |
| 历史系统迁移脚本 | `3D/scripts/jammy_post_upgrade_setup.sh` | `archive/legacy_compat/jammy_system_migration_20261025/` | 已归档，已复验 | 会改写 apt 源、卸载系统包并修改 shell，不作为普通 Humble 工作区安装入口 |
| 过期工作区审计 | 工作区根 `WORKSPACE_AUDIT.md` | `archive/docs/workspace_audits/WORKSPACE_AUDIT_20261020.md` | 已归档，已复验 | 报告保留当时证据，但仍把本地 2D 写为活动模块，与当前外置 2D 边界冲突 |
| 最终 3D 代码复验产物 | 本轮 `ros2_ws/log/`、源码内 Python／pytest 缓存 | `archive/validation/20261025/final_3d_code_audit/` | 已归档，已验证 | 修正 Record 元数据、直接依赖和过期系统脚本入口后：5 包构建、43 项离线测试、13 个 launch 参数加载、8+19 项长途离线检查通过 |

该批次的来源、命令、结果与“仅归档、不删除”的原始证据已随归档批次保存；日常操作只使用本清单、模块地图和 3D 活动手册，不把归档材料作为入口。

## 2026-10-25 待用户确认删除区

用户要求先集中查看无用内容，再亲自决定删除。因此以下项目从原归档位置**可逆地移动**到 `archive/pending_deletion/20261025/`；没有删除任何文件。该区仅包含可再生的验证输出、缓存或已被替代的一次性脚本，**不**包含详细 README、公式、地图、world、实车资料、历史实验原件或 Record 9 基线。

| 待删区内容 | 原归档位置 | 原因 | 保留的证据 |
|---|---|---|---|
| `3d_td3_smoke/` | `archive/validation/20261025/3d_td3_smoke/` | 30-step 冒烟的非基线 checkpoint 与图表，可重新生成 | `archive/validation/20261025/README.md` 的通过结论 |
| `3d_debug_logs/`、`ros2_colcon_logs/`、`root_colcon_logs/`、`legacy_ros2_debug_logs/` | `archive/validation/20261025/` | 结束会话的调试/构建日志 | `archive/validation/20261025/README.md` |
| 两批复验的生成日志与 Python/pytest 缓存 | `post_cleanup_static_validation/`、`final_3d_code_audit/` | 可再生构建日志与缓存 | 两个原目录中的 `README.md`，记录验证命令与通过结果 |
| 三个过期探针脚本 | `archive/validation/20261025/stale_one_off_scripts/` | 旧参数断言或无文档入口的一次性自动运动脚本 | 原目录 `README.md`，记录替代入口和原因 |
| `ros2_ws_pytest_cache_20261020/` | `archive/asset_backups/` | 纯测试缓存 | 无需保留；可按需重新生成 |
| `empty_placeholders_20261020/` | `archive/legacy_compat/` | 已替代的空目录占位 | 模块地图与本清单的迁移记录 |
| `jammy_post_upgrade_setup.sh` | `archive/legacy_compat/jammy_system_migration_20261025/` | 危险且过期的一次性系统迁移脚本 | 同目录 README 保留风险说明 |

删除须由用户在复核后执行；在确认前，任何活动脚本、训练、推理或构建都不依赖该待删区。

## 2026-10-20 完成记录

- 已确认 `ros2_ws/src/` 只保留 `shared/`、`navigation_2d/`、`navigation_3d/`、`third_party/` 四个分区；`colcon` 发现 6 个活动包并完成共享、2D、3D 构建。
- 此处记录的是 2026-10-20 当时的 2D／3D 复核结论；当前本机仅保留 3D 活动工作区，2D 已转为独立仓库。现行操作命令、模块入口和验证边界以 [`MODULE_MAP.md`](MODULE_MAP.md)、[`3D/README.md`](3D/README.md) 与 [`3D/README1.md`](3D/README1.md) 为准；原始审计材料仅留在归档区，不作为日常操作入口。
- 仍未验证的外部边界仅包括真实机器人、现场网络、真实传感器和实车安全控制。它们不能因仿真验证通过而视为可用。
- 表中“已移动，待校审”的历史移动项以本记录和审计报告为准：已完成活动引用、构建和受影响模块的最小运行验证；原始归档文件未被重写。
