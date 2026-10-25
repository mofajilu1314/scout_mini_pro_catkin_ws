# 3D ROS2 Humble 当前快速入口

适用环境：Ubuntu 22.04、ROS2 Humble、Gazebo Classic。以下命令只针对 3D 模块；不会构建或启动 2D 导航。

## 1. 建立明确的工作区根

```bash
cd /path/to/scout-workspace
export SCOUT_WS_ROOT="$PWD"
source /opt/ros/humble/setup.bash
cd "$SCOUT_WS_ROOT/ros2_ws"
colcon build --symlink-install --packages-select \
  scout_workspace_common scout_description scout_mini_pro_description \
  scout_offroad_3d scout_long_nav
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"
```

路径解析优先级固定为 `SCOUT_WS_ROOT`、`CATKIN_WS`、`WS_ROOT`，之后才从包位置推导。不要把用户目录或旧顶层 `src/` 写进命令、配置或脚本。

## 2. 先做最小验证

长途规划的离线检查不会启动 Gazebo：

```bash
bash "$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav/scripts/smoke_long_nav_planning.sh"
```

TD3 冒烟会占用仿真资源，因此运行前确认没有自己已启动但尚未退出的 3D 任务：

```bash
bash "$SCOUT_WS_ROOT/3D/scripts/humble_train_smoke.sh" --no-build
```

脚本检测到未知的 Gazebo/端口占用时应中止并报告；不要复制历史资料中按名称或端口强制终止未知会话的做法。

## 3. 训练与续训

默认训练永远新建唯一 `3D/runs/run_*` 或 `smoke_*` 目录；已有目录不会自动复用，也不会自动续训。

```bash
ros2 launch scout_offroad_3d train_td3.launch.py gui:=false
```

续训必须明确表达意图，并指定一个已有且兼容的 run；新的步数必须大于该 run 已记录的步数：

```bash
ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false resume:=true \
  output_dir:="$SCOUT_WS_ROOT/3D/runs/<existing-run>" \
  max_timesteps:=<larger-total>
```

续训前会校验 TD3 四个权重/优化器文件、metrics、参数快照与状态/动作/网络维度。不通过就停止，不会猜测或修补历史 run。

详细规则见 [`README1.md`](README1.md)，实际验证结果见后续的工作区验证报告。
