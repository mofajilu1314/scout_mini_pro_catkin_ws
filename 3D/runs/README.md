# 活动 TD3 输出

默认训练在开训瞬间创建唯一的 `run_YYYYMMDD_HHMMSS/`；短验证创建唯一的
`smoke_*` 或 `verify_switch_*` 目录。它们只在**当前会话**期间暂存，不是历史实验库。

本目录中的权重、配置快照、metrics 和图像一律由 `.gitignore` 排除，不能作为 GitHub
源码提交物。完整验证日志在运行期间写入 `3D/debug_logs/`；短验证结束后，run 目录与相应日志必须
一起移至 `archive/validation/YYYYMMDD/<主题>/`，并附执行命令和结论。失败验证证据、历史正式 `run_*` 和长时实验均按工作区规则独立归档，不是本目录的日常入口。Record 9 也只作为受控的仿真推理基线保留在历史资料中。

## 单次输出的用途

- `td3_actor`、`td3_critic` 和两个优化器文件共同构成 TD3 checkpoint 前缀；不要只复制其中一部分。
- `config.yaml` 与 `params_backup.yaml` 保存实际生效的 `meta`、`train`、`env` 参数和状态维度。
- `metrics.csv`、`train.log` 和可视化图只用于该次训练的分析；短测没有完成 episode 时图表可为空。验证
  结束后连同本说明所列文件归档，活动目录恢复为空。
- 只有显式 `resume:=true`，且完整 checkpoint、metrics、参数快照、状态维度和进度均匹配时，才允许以某个已有输出续训。普通训练绝不复用本目录中的旧输出。

这些 checkpoint 仅用于仿真研究和复现；它们不构成实车控制许可。需要离线画图时：

```bash
: "${SCOUT_WS_ROOT:?Set SCOUT_WS_ROOT to the workspace root first}"
ros2 run scout_offroad_3d plot_td3_metrics \
  "$SCOUT_WS_ROOT/3D/runs/run_YYYYMMDD_HHMMSS"
```
