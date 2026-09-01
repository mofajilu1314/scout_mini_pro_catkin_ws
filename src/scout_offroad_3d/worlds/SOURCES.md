# Off-road / outdoor worlds — sources

Canonical directory for stage-A training worlds: **this folder**
(`src/scout_offroad_3d/worlds/`). Do **not** put training worlds under `3D/worlds/`.

| File | Source | Use for TD3 |
|------|--------|-------------|
| **`bumpy_offroad.world`** | Wrapper around PX4 **`uneven_ground`** heightmap (`80×80×2 m` gentle bumps). Model from [PX4-SITL_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/main/models/uneven_ground). | **Primary bumpy off-road terrain** — small-area heightmap, not flat clutter. |
| `clearpath_playpen.world` | Copied from `src/scout_gazebo_sim/worlds/clearpath_playpen.world` (Clearpath BSD). **One** SDF world (`name=default`), not two. Outdoor flat lot + jersey barriers / cones / barrels / dumpsters. Needs Gazebo model:// assets (usually `~/.gazebo/models`). | Useful for **obstacle clutter** / avoidance; **not** true rough heightmap terrain. |
| `rough_obstacles.world` | Self-authored for this package (scattered boxes/cylinders). | Cluttered field; no external meshes. |
| `offroad_map.world` | User-added outdoor clutter map (renamed from extensionless `offroad_map`). Trees / mud boxes / walls / pallets; needs Gazebo `model://` assets. | Outdoor clutter / avoidance; included in `pick_random_world.py` `*.world` pool. |

### Install `uneven_ground` model (required for `bumpy_offroad.world`)

```bash
# One-liner (if you have PX4 sitl_gazebo locally):
cp -a ~/uav_flight_catkin_ws/repos/prometheus_px4/Tools/sitl_gazebo/models/uneven_ground ~/.gazebo/models/

# Or clone from GitHub:
git clone --depth 1 https://github.com/PX4/PX4-SITL_gazebo-classic.git /tmp/px4-gz
cp -a /tmp/px4-gz/models/uneven_ground ~/.gazebo/models/
```

Verify: `ls ~/.gazebo/models/uneven_ground/model.sdf`

**Not copied:** `src/scout_gazebo_sim/worlds/weston_robot_empty.world` — empty ground only; little value for off-road training.

## Rejected / removed (2026-08-23 · Record 6 训前实测)

| File | Source | Why removed |
|------|--------|-------------|
| ~~`yosemite_heightmap.world`~~ | RotorS/OSRF `model://yosemite` DEM | **km 级山谷 DEM**，坡度过陡、尺度为无人机/航拍级；Scout 小车无法在此地形上有效训练。 |
| ~~`volcano_dem.world`~~ | Gazebo 11 `media/dem/volcano.tif` | 虽仅 ~129×129 m，但 **~80 m 高差、极陡**；同为 DEM 山地形，不适合小型地面车。 |

**结论**：真实 DEM / 高山 heightmap **不入训练池**。下阶段目标为 **20–50 m 小范围、缓坡起伏（0.2–0.8 m 高差）** 的 off-road 地面，优先 **自建** 而非再下载陡峭 DEM。

## 外部候选（Web 调研 · 2026-08-23）

> 需求：Gazebo Classic · 小 footprint（约 20–50 m）· 平缓颠簸/小丘 · **非** 陡峭 DEM 山脉 · Scout 地面车尺度。

| # | 名称 | 链接 | 尺度 / 地形 | Scout 适配 | 备注 |
|---|------|------|-------------|------------|------|
| 1 | **PX4 `uneven_ground`** | https://github.com/PX4/PX4-SITL_gazebo-classic/tree/main/models/uneven_ground | **80×80×2 m** PNG heightmap，轻微起伏 | ⭐ **最接近** | 需拷 `models/uneven_ground` 到 `~/.gazebo/models/` 并写 `.world` 包装；2 m 总高差仍偏无人机测试用，可缩放到 40×40×0.5 m 再试。 |
| 2 | **Clearpath Construction** | https://github.com/clearpathrobotics/cpr_gazebo (`cpr_construction_gazebo`, `noetic-devel`) | 室内工地；**小土堆/碎屑坡** | 部分 | 非户外土路；依赖 office 布局，起伏不连续。 |
| 3 | **Clearpath Orchard** | https://github.com/clearpathrobotics/cpr_gazebo (`cpr_orchard_gazebo`) | 户外；**小坡度、基本平坦** | 部分 | 场景大、树行为主；起伏弱，非典型 bumpy off-road。 |
| 4 | **Clearpath Obstacle** | https://github.com/clearpathrobotics/cpr_gazebo (`cpr_obstacle_gazebo`) | 室内；**非平面地面** | 弱 | 封闭障碍场，非户外越野。 |
| 5 | **gazebo_terrain_generator** | https://github.com/saiaravind19/gazebo_terrain_generator | 用户框选 → heightmap zip | 需裁剪 | 默认真 DEM；须 **手动限制区域 + 压低 Z 尺度**（如 30×30×0.5 m），否则重蹈 DEM 覆辙。 |

**未列入（已排除）**：RotorS Yosemite、Gazebo volcano/Mt St Helens DEM、Clearpath Inspection / Pipeline（丘陵过大、依赖重）、Ignition `solar_farm`（Harmonic，非 Classic）。

**调研结论**：**无开箱即用的「20–50 m 缓坡越野」Classic 世界**；候选 #1 可作参考起点，**推荐自建**小范围 heightmap（见下）。

### 推荐自建路径（下一步）

1. **程序化灰度 PNG heightmap**（Gazebo Classic 教程）：`size` 设为 `40 40 0.5`（40 m 边长、0.5 m 最大起伏）；图像尺寸 `(2^n)+1`（如 257×257）。教程：https://classic.gazebosim.org/tutorials?tut=heightmap
2. **Blender  sculpt → DAE mesh**：适合固定小丘/沟壑；导出静态 trimesh。参考：https://robotics.stackexchange.com/questions/35714/create-rough-terrain-for-used-in-gazebo
3. **借鉴 PX4 `uneven_ground`**：复制模型，把 `<size>80 80 2</size>` 改为 `30 30 0.4`，用低频噪声重绘 `terrain.png`。

## Manual download links (reference only)

- PX4 uneven_ground model: https://github.com/PX4/PX4-SITL_gazebo-classic/tree/main/models/uneven_ground
- Clearpath extra worlds (inspection / orchard / construction): https://github.com/clearpathrobotics/cpr_gazebo (branch `noetic-devel`)
- Gazebo Classic heightmap tutorial (custom PNG, **not** DEM): https://classic.gazebosim.org/tutorials?tut=heightmap
- AWS RoboMaker sample worlds: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world — **indoor / flat**, not off-road

Place any extra `.world` files into **this directory**, then:

```bash
roslaunch scout_offroad_3d train_td3.launch world:=$(find scout_offroad_3d)/worlds/YOUR.world
# random among *.world here:
roslaunch scout_offroad_3d train_td3.launch world:=$(rosrun scout_offroad_3d pick_random_world.py)
```
