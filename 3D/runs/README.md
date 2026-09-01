# 训练产物目录

默认 TD3 训练输出：`run_YYYYMMDD_HHMMSS/`（由 `train_td3_node` 在**开训瞬间**自动创建）。

**勿提交大文件**：权重、日志、曲线图体积大，已在仓库 `.gitignore` 忽略 `3D/runs/run_*`。

## 单次 run 内文件

| 文件 | 含义 |
|------|------|
| `td3_actor` / `td3_critic` | 推理必需（前缀 `td3`，无扩展名） |
| `td3_actor_optimizer` / `td3_critic_optimizer` | 续训用；真车推理可不拷 |
| `config.yaml` | **实际生效**超参快照（`meta` + `train` + `env`；含 launch 覆盖） |
| `params_backup.yaml` | 与 `config.yaml` 相同（双份归档） |
| `train.log` | 每 episode 文本日志 |
| `metrics.csv` | episode 指标 CSV（含 outcome、滑动窗口率、path_efficiency、`r_*_sum` 奖励分项；见 README1 §4.8） |
| `reward_curve.png` 等 | 训完自动生成；缺图可 `rosrun scout_offroad_3d plot_td3_metrics.py <run_dir>` |
| `reward_components_stacked_area.png` | 各 reward 分项 **\|value\| 占比** 堆积图（新 run，Record 4+） |
| `reward_components_curves.png` | 各分项 episode 总和滑动平均 |

### `config.yaml` / `params_backup.yaml` 结构

```yaml
meta:
  started_at: "2026-08-23T12:00:00"
  run_dir: /home/test/catkin_ws_2/3D/runs/run_...
  workspace_root: /home/test/catkin_ws_2
  initial_world: /path/to/world.world
  world_basename: clearpath_playpen.world
  state_dim: 76
  action_dim: 2
  max_action: [2.0, 1.5]
  source_yaml: scout_offroad_3d/config/td3_train.yaml
  record: 4
train:
  seed: 1
  max_timesteps: 400000   # launch 覆盖后的值
  ...
env:
  r_forward_scale: 0.05
  collision_dist: 0.34
  ...
```

真车推理 / 复现训练：以 run 目录内 `config.yaml` 的 `env` 段为准，而非仓库默认 yaml（launch 可能改过 `max_timesteps` 等）。

分析：`metrics.csv` 与 `*.png` 可直接发给 AI 做曲线解读，或本地：

```bash
rosrun scout_offroad_3d plot_td3_metrics.py ~/catkin_ws_2/3D/runs/run_YYYYMMDD_HHMMSS
```
