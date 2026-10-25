# arena/ — 长距离导航用地形归档（非 TD3 短训）

## 用途

本目录存放 **大范围** 颠簸越野世界，供后续：

- **FAST-LIO / LIO-SAM** 建图与定位
- **Arena** 长距离导航评测 / 任务
- 与 TD3 局部策略的组合（全局规划 + 局部颠簸通行）

保留 PX4 `uneven_ground` 的 **80×80 m** 大 heightmap 优势，不要用本目录做日常 TD3 短 episode 训练。

## 文件

| 文件 | 说明 |
|------|------|
| `bumpy_offroad_full.world` | 完整大图：`uneven_ground` **80×80×2 m** + 四周边墙 + 同学摆的树/水塔等；默认 `real_time_factor=5` |

## 依赖

大图的 `uneven_ground` 是本仓库不随附的外部 Gazebo 资产（**不要**用
TD3 的 `uneven_ground_td3_*` 覆盖它）。将该模型放在你自行管理的模型根
目录，并在启动前显式提供路径：

```bash
EXTERNAL_GAZEBO_MODELS=/absolute/path/to/gazebo-models
test -f "$EXTERNAL_GAZEBO_MODELS/uneven_ground/model.sdf"
export GAZEBO_MODEL_PATH="$EXTERNAL_GAZEBO_MODELS${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
```

`gazebo_scout`、`view_world` 和 TD3 切图都会保留调用方显式设置的
`GAZEBO_MODEL_PATH`，但不会自动扫描用户缓存。除了 `uneven_ground`，此大图还
引用 `water_tower`、`oak_tree` 和 `pine_tree`；它们也必须存在于同一个显式模型根
或已有的 `GAZEBO_MODEL_PATH` 中。本仓库不随附这些外部资产，也没有验证它们的
下载/安装流程；[`worlds/SOURCES.md`](../worlds/SOURCES.md) 只说明随包 TD3 训练图，
不是本 arena 的外部资产安装手册。缺少任一模型时，本入口应视为“外部依赖未满足”，
而不是开箱可运行的验证命令。

## 如何打开（大图）

空窗、确认无 TD3 训练抢占 `gzserver` 后再开。加速：world 内 `real_time_factor`（默认 5）或 Gazebo 左下角实时因子。

```bash
source /opt/ros/humble/setup.bash
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"

# 仅看世界（无车）；使用 package share 路径
ros2 launch scout_offroad_3d view_world.launch.py \
  world:="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/arena/bumpy_offroad_full.world"

# 或 install 后的 share 路径
# ros2 launch scout_offroad_3d view_world.launch.py \
#   world:=$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/arena/bumpy_offroad_full.world
```

## 与 TD3 训练图的关系

| | **arena（本目录）** | **worlds/bumpy_offroad{,_2,_3,_4}.world** |
|--|---------------------|--------------------------------|
| 地形 | `uneven_ground` 80×80 m | 本地 `uneven_ground_td3_1…4` ~30×30 m |
| 用途 | 长距离 LIO + Arena | TD3：**恰好 4 张** |
| 性能 | Gazebo 更重、易卡 | 更轻，适合反复 reset |

**请勿**把 `arena/*.world` 放进 `pick_random_world` / 训练随机池。

## 长导航两段式操作

与 TD3 训练硬隔离的建图 / 导航流程见 **[`3D/README4.md`](../../../../../3D/README4.md)**：

- `ros2 launch scout_long_nav long_nav_map.launch.py` → Ctrl+C 存 `3D/maps_long/arena_trav.*`
- `ros2 launch scout_long_nav long_nav_nav.launch.py` → RViz 2D Goal 点终点
- 空图捷径：`python3 -m scout_long_nav.bake_heightmap_trav` 默认使用随包的
  TD3 地形纹理，便于可复现的离线检查；要让静态图精确对应本 arena，请显式
  传入外部模型的纹理：`--terrain "$EXTERNAL_GAZEBO_MODELS/uneven_ground/materials/textures/terrain.png"`。
