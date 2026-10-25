# Off-road / outdoor worlds — sources

Canonical editable directory for stage-A training worlds: **this folder**
(`ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/`). Do **not** put
training worlds under `3D/worlds/`. At runtime, resolve the installed copy
through the `scout_offroad_3d` package share rather than a user-specific path.

## TD3 地图池（**恰好 4 张**）

全部为 bumpy heightmap（用户已在 Gazebo 里摆好树 + 围边墙）。模型在包内
`models/uneven_ground_td3_1` … `_4`。
Heightmap 文件名互异：`terrain_td3_N.png`（勿再共用 `terrain.png`）。

**碰撞/视觉分离（2026-09-03，DEBUG 记录 47）**：`<collision>` 用 65×65 `terrain_td3_N_65.png`，
`<visual>` 保持 129×129 `terrain_td3_N.png`。129² heightfield 碰撞让 RTF 掉到 0.16（每步 0.63s）；
65² 后 0.22s/step（RTF≈0.45）。视觉与布局零变化。

| World | Model | 角色 | `world_spawn_z` |
|-------|-------|------|-----------------|
| **`bumpy_offroad.world`** (#1) | `uneven_ground_td3_1` | train / viz | **0.23** |
| `bumpy_offroad_2.world` | `uneven_ground_td3_2` | train / viz | **0.23** |
| `bumpy_offroad_3.world` | `uneven_ground_td3_3` | train / viz | **0.23** |
| `bumpy_offroad_4.world` | `uneven_ground_td3_4` | train / viz | **0.23** |

- **训练换图 + viz_eval** 共用这 4 张（`train_worlds`；`pick_random_world` / launch 同此）。
- **viz_eval**：每图 **3** 组起–终点 → 默认 **12** 张透明底轨迹 PNG（`traj_overlay_<stem>_p01.png` … `_p03.png`）。
- 大图归档（非训练池）：`../arena/bumpy_offroad_full.world`。

### 已删除

| File | 原因 |
|------|------|
| ~~`clearpath_playpen` / `offroad_map` / `rough_obstacles`~~ | 平地 clutter |
| ~~`bumpy_offroad_5` / `_6` + `uneven_ground_td3_5/6`~~ | 不做 5/6 图；已从磁盘移除 |

### Install / path

```bash
PKG_SHARE="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d"
ls "$PKG_SHARE/models/uneven_ground_td3_1/model.sdf"
# 可编辑源码：$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/models/uneven_ground_td3_{1..4}/
```

**编辑 world 时只改源码树**：`$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/worlds/`。
不要直接编辑安装后的 package share；重新构建并 source overlay 后，运行时会使用其对应副本。

## 查看 / 训练

```bash
: "${SCOUT_WS_ROOT:?Set SCOUT_WS_ROOT to the workspace root first}"
source /opt/ros/humble/setup.bash
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"

WORLD_DIR="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds"

ros2 launch scout_offroad_3d train_td3.launch.py \
  world:=$(ros2 run scout_offroad_3d pick_random_world)

ros2 launch scout_offroad_3d view_world.launch.py \
  world:="$WORLD_DIR/bumpy_offroad.world"
# … _2 / _3 / _4 同理
```
