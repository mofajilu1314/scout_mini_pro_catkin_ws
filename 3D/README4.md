# 3D · README4 — 长途导航控制（分层栈 · 公式 · 仿真→实车）

> **版本与安全注记（2026-10-25）**：本册完整保留分层导航的公式、操作、验证与设计推理。所有 `$SCOUT_WS_ROOT` 指向工作区根；当前包路径为 `ros2_ws/src/navigation_3d/scout_long_nav` 与 `ros2_ws/src/navigation_3d/scout_offroad_3d`。**Record 8（112 维 / hidden 768）仅为历史实验，不可与当前权重混用；Record 9（150 维 / hidden 1024）是当前仿真推理基线，不能直接用于实车控制。** 只停止自己启动的任务，用 `Ctrl+C` 和只读 `ss` 排查占用，不使用广泛杀进程命令。

> **本文件自洽**：分层栈与公式在此；**ScoutNav 软件怎么点、弹什么窗** → [`README3.md` §12](README3.md#ch12)。  
> **代码包**：[`ros2_ws/src/navigation_3d/scout_long_nav`](../ros2_ws/src/navigation_3d/scout_long_nav/)  
> **分工（别的专题才打开）**：TD3 训练 → [`README1.md`](README1.md)；出图 → [`README2.md`](README2.md)；真车 SSH + GUI 手册 → [`README3.md`](README3.md)；总规划 → [`README0.md`](README0.md)

| 项 | 内容 |
|----|------|
| 仿真主路径 | Gazebo `/odom` → `/long_nav/costmap` → Hybrid A* → Arena 子目标 → TD3 → `nav_cmd_gate` → `/cmd_vel` |
| 真车主路径 | FAST-LIO2 → 同套中间层 → 车端 ROS1 TD3（接口见 §7.2） |
| 大图世界文件 | `ros2_ws/src/navigation_3d/scout_offroad_3d/arena/bumpy_offroad_full.world`（80×80 m，有围墙+树；**勿**放进 TD3 训练 4 图池） |
| 当前仿真试跑权重 | **Record 9**：`$SCOUT_WS_ROOT/archive/experiments/td3/runs/run_20260907_120841/td3`（150 维 / hidden 1024；只读、仅仿真） |
| 工作图 | `$SCOUT_WS_ROOT/3D/maps_long/arena_trav.yaml` + `.pgm` |
| 时刻备份 | `maps_long/map_YYYYMMDD_HHMMSS/` + `INDEX.md` |
| 自动建图 | ~~已弃用（易乱撞）~~；请用阶段1 **Flysky 自驾建图** |
| 仿真回环 | **已通**：建图 → 存图/续建 → 导航点终点 → 到点锁轮 → 翻车重生解锁 |

为避免在命令中反复写长路径，可在**本机受控终端**中设：

```bash
export RECORD9_WEIGHTS="$SCOUT_WS_ROOT/archive/experiments/td3/runs/run_20260907_120841/td3"
```

> GitHub 导出可以保留本手册和代码，但不应强行打包大型历史权重；若本机未保留上述 Record 9 目录，请取得**相同 150/1024 配置**的受控仿真权重后，显式传 `weights:=.../td3`。不要把 Record 8 权重或仿真权重直接用于实车。
---

<a id="ch0"></a>

## 0. 仿真两段试跑（先看这里 · 会弹窗口）

**硬规则**：不能与 `train_td3` 同时开（共用一台 `gzserver`）。开之前先检查：

```bash
bash $SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav/scripts/check_long_nav_idle.sh
# 若提示已有 gzserver：只回到自己启动它的终端按 Ctrl+C，等待退出。
# 只读检查端口；未知任务不要终止，先确认使用者/所属终端：
ss -ltnp | rg ':11345\b' || true

source /opt/ros/humble/setup.bash
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=41
```

### 阶段 1 — 建图（开车扫 / 看代价图 → 存盘）

**用途**：起 arena 大图 + Scout 小车 + RViz 代价图 + **Flysky USB 遥控**（默认）；关掉时把 `/long_nav/costmap` 存成静态图。本阶段 **不起** A* / Arena / TD3。

**安全**：控仿真前请 **关真车电源**。遥控器 USB 模拟器只进电脑、发仿真 `/cmd_vel`，**拦不住** 摇杆对真车的 2.4G 无线电。实车要用时：关掉本 launch / 拔 USB，按原厂 SWB 控车。

```bash
ros2 launch scout_long_nav long_nav_map.launch.py
# 只要键盘：  use_joy:=false use_keyboard:=true
# 续建（加载已存图继续扫，不从零）：
# ros2 launch scout_long_nav long_nav_map.launch.py \
#   seed_map_yaml:=$SCOUT_WS_ROOT/3D/maps_long/arena_trav.yaml \
#   spawn_x:=0 spawn_y:=0 spawn_z:=0.23 spawn_yaw:=0
# 摇杆+键盘都开会抢 /cmd_vel，一般不要同时开
```

| 应弹出 | 你做什么 |
|--------|----------|
| Gazebo | 大图 + 小车（**须有 GUI**：相机靠渲染出图） |
| RViz | 贴地清新热力图 `CostmapHeat` + **前置相机** `CameraRGB`（`/camera/color/image_raw`） |
| Flysky（默认） | 遥控开机+USB：左杆油门、右杆转向；建图限幅约 **2.5 m/s / 1.8 rad/s**。**勿改遥控器**；详见 **§0.1** |
| xterm 键盘 | 仅当 `use_keyboard:=true`：点进窗口后 `i`/`,`/`j`/`l`/`k` |

**Signal lost**：遥控器找不到真车接收机时的提示。真车关着就会一直显示，**与 USB 控 Gazebo 无关，可忽略**。

若车不动：看 launch 终端——若反复「摇杆还没有任何变化」，是 **USB 没收到推杆数据**（换能传数据的线、拔插、确认遥控电池已开），不是 Gazebo 坏了。临时用键盘：`use_joy:=false use_keyboard:=true`。

**翻车监护（建图默认开）**：主检测源为 Gazebo `/get_entity_state`（IMU 备份）。阈值约 **roll/pitch≥55° 或之和≥75°** 时：立刻停车并静音遥控 → **暂停雷达写入代价图** → 清除翻车点附近约 **12 m** 错误橘色 → 在翻车坐标从 **z=0.23 起每次 +0.5** 阶梯重生（期间仍不写图）→ 车身摆正后再打开雷达建图；若该点仍失败则改到 **原点 (0,0)** 同样阶梯。终端可见 `tilt watch` / `>>> ROLLOVER DETECTED` / `respawn try`。

前进/转向手感只改 **建图 launch** 里电脑节点限幅（默认 **`teleop_max_v:=2.5`**、**`teleop_max_w:=-1.8`**），**不要改遥控器**。发的是 **`/cmd_vel` 速度**（m/s、rad/s），不是力。松杆回中后节点持续发 **零速**（锁轮）；勿把顶上拨杆通道当油门。

自检摇杆 USB：

```bash
# 另开终端，大力推杆，数字应跳动
python3 - <<'PY'
import os,struct,time
fd=os.open('/dev/input/js0',os.O_RDONLY)
t0=time.time()
while time.time()-t0<15:
    data=os.read(fd,8)
    t,val,typ,num=struct.unpack('IhBB',data)
    if typ&0x80: continue
    if typ&0x02: print(f'axis{num}={val/32767:.2f}')
    if typ&0x01: print(f'btn{num}={val}')
PY
```

开一会儿后 → 贴地 **青/薄荷绿/琥珀** 热力图应逐渐铺开。若仍只有脚下一块：重新 `colcon build --packages-select scout_long_nav` 并 `source install/setup.bash` 后再开。

### 阶段 1b — 自动覆盖建图（不推荐 · 已弃用）

**不要用。** 无人值守 TD3 扫图会乱撞、代价图易被标脏。建图请用上面 **阶段1 Flysky/键盘**。  
若仍要试：`ros2 launch scout_long_nav long_nav_auto_map.launch.py`（自担风险）。

**大图已更新**：同学改好的带树版已从 U 盘 `arena_edit_kit/RETURN/` 覆盖到同名路径  
`ros2_ws/src/navigation_3d/scout_offroad_3d/arena/bumpy_offroad_full.world`（旧文件有 `.bak_*`）。**命令不用改**，直接再开阶段 1 即可看到树等物件。

**地图尺度 / 雷达 / 限速（建图 `long_nav_map` 默认）**：

| 项 | 数值 | 说明 |
|----|------|------|
| 围墙 / heightmap | **约 80×80 m** | `bumpy_offroad_full.world` |
| 代价图窗口 | **80×80 m**（`size_m:=80`） | 固定中心 |
| 栅格边长 | **0.08 m** | `resolution:=0.08` → 约 1000×1000 格 |
| 雷达（默认 mapping） | **720×16 @ 20 Hz**，**gpu_ray** | 比真车更密更快，少漏扫 |
| 雷达（可选真车档） | **1800×16 @ 10 Hz** | `lidar_profile:=real_rs16`，贴近 RS-16 |
| 噪点抑制 | `min_points_per_cell:=4`，`beta_rough:=25`，`slope_max:=0.55` | 减轻过坡震颤假橙色 |
| 遥控满杆 | **2.5 m/s**，**1.8 rad/s**（符号 -1.8） | `teleop_max_v` / `teleop_max_w`；**仅建图**，不改 TD3 |

覆盖示例：

```bash
ros2 launch scout_long_nav long_nav_map.launch.py
# 可选：对齐真车 RS-16 密度/频率（建图会稍易漏扫，开慢点）
# ros2 launch scout_long_nav long_nav_map.launch.py lidar_profile:=real_rs16
```

**透明格（RViz 里没颜色的洞）**：

| 格子 | 含义 | Hybrid A* |
|------|------|-----------|
| 透明 / 未知（-1） | 还没扫到或被翻车清理掉 | **可穿**，只加一点代价，不是墙 |
| 青/低代价 | 已观测、较平 | 优先走 |
| 橙（1–89） | 坡/粗糙 | 能走但贵 |
| ≥90 / 白 | 致命 | 不通 |

不必把 80×80 每个格子都填满。主通道来回慢开一两次即可；树影、遮挡处留透明很正常。真正麻烦的是错误橘色/致命，不是透明。

**橙色点与导航（Hybrid A*）**：

| 格子 | 规划行为 |
|------|----------|
| 青/低代价（0 附近） | 优先走 |
| 橙/中高代价（1–89） | **能走**，但更贵 → 会绕开，走橙色更少的缝 |
| 接近白/lethal（≥90，默认阈值） | **不能踩**，当作墙 |
| 周围全是 lethal | **无解**，路径为空，车不会硬闯 |
| 周围全是「中等橙」但都 <90 | 仍会选一条总代价最低的路挤过去 |

零星假橙色：提高雷达密度 + 每格最少 4 点后会少很多；真树/陡坡仍会留橙。

**不改** TD3 训练默认雷达（仍多为 96×8）与 `td3_train.yaml` 的 `max_v`/`max_w`。

**RViz 颜色（清新热力图，仅显示）**：显示话题 `/long_nav/costmap_rgb`（`CostmapHeat`）。规划仍读 `/long_nav/costmap`，数值不变。

| 颜色倾向 | 含义 |
|----------|------|
| **浅蓝 / 青** | 好走（代价低） |
| **薄荷绿** | 一般 / 偏难 |
| **琥珀橙** | 很难或 lethal |
| 不画点 | 还没扫到（unknown） |

左侧勾选 `CostmapHeat` + `CameraRGB` + `RobotModel`。相机话题：`/camera/color/image_raw`（须 `gui:=true`，Gazebo 在渲染才有图）。若黑屏：另开终端 `ros2 topic hz /camera/color/image_raw`。

**RViz「大黄片」**：多半是 **Odom → Covariance**（Gazebo 未用轴协方差极大）。建图/导航配置默认关 Odom 协方差；若仍出现，取消勾选 Odom，或把 Covariance 关掉。

**怎么存图 / 备份**：

| 方式 | 结果 |
|------|------|
| 建图终端 **Ctrl+C** | 自动写工作图 `maps_long/arena_trav.{yaml,pgm}` |
| 不关 launch | `ros2 service call /save_costmap_node/save std_srvs/srv/Trigger {}` |
| 时刻备份（建议扫完做） | 拷到 `maps_long/map_YYYYMMDD_HHMMSS/`，索引见 `maps_long/INDEX.md` |

示例备份（把当前工作图固化成带时刻名）：

```bash
stamp=$(date +%Y%m%d_%H%M%S)
mkdir -p $SCOUT_WS_ROOT/3D/maps_long/map_${stamp}
cp -a $SCOUT_WS_ROOT/3D/maps_long/arena_trav.pgm \
  $SCOUT_WS_ROOT/3D/maps_long/map_${stamp}/map_${stamp}.pgm
# yaml 里 image: 改成 map_${stamp}.pgm 后另存为同目录 map_${stamp}.yaml
```

阶段 2 默认读工作图 `arena_trav.*`。续建：`seed_map_yaml:=…/arena_trav.yaml`（或某次 `map_*/*.yaml`）。地图方向锁在 `odom`（原点约 (-40,-40)），存盘/加载不会翻转。

### 0.1 遥控器怎么用（AGX-i6S · 仿真对齐真车）

机顶从左到右 **4 根拨杆**（开关）：**SWA | SWB | SWC | SWD**。下面还有 **左右两个摇杆**（各 2 轴，共 4 轴）。

#### 实车（无线电 · 学长说法 + 交接文档）

学长教的「低速手开」流程（**只用中间两根拨杆**）：

1. 从左往右数，**第 2 根 = SWB**：往下拨  
2. **第 3 根 = SWC**：拨一下（切到低速档）  
3. 之后只用 **摇杆** 开车；两边最外的 SWA / SWD 先别动  

与仓库交接文档对照（同一根 SWB 是三档）：

| SWB 位置 | 文档含义 |
|----------|----------|
| **最上** | **Command** — 电脑 `/cmd_vel` 可接管（自动/TD3 前必须打到这） |
| **中间** | **遥控模式** — 只听遥控器摇杆 |
| **最下** | **定速模式** |

学长说的「第二根往下」若开不起来，再试 **SWB 打到中间（遥控）** + **SWC 低速**；以现场能低速动车为准。  
真车限速：低档约 10 km/h 量级；自动控车前 SWB → Command，人盯急停。

摇杆（Mode2，与多数 Scout 套件一致）：

| 摇杆 | 动作 | 车 |
|------|------|-----|
| **左摇杆 上下** | 前推前进、后拉后退 | 线速度 |
| **右摇杆 左右** | 左/右打方向 | 角速度 |
| 左摇杆左右、右摇杆上下 | 一般不用管 | — |

#### 仿真（USB 插电脑 · Gazebo）

目标：**摇杆手感对齐真车**；拨杆档位无法 1:1 复制到底盘固件，所以：

| 真车 | 仿真里怎么对应 |
|------|----------------|
| SWB/SWC 拨到低速遥控 | 建图电脑节点限幅（默认 **2.5 m/s**、`teleop_max_w=-1.8`）；**勿改遥控器**；**勿改** TD3 `max_v`/`max_w` |
| 左杆上下 = 油门 | 同左：USB 轴 2 → `/cmd_vel.linear.x`（速度，不是力） |
| 右杆左右 = 转向 | 同右：USB 轴 0 → `/cmd_vel.angular.z` |
| **松杆回中** | 电脑节点 **持续发零速**（对齐真车锁轮）；**不会**用拨杆轴当油门 |
| 屏幕 Signal lost | **关真车时正常，可忽略**（只表示找不到车载接收机） |

**控仿真前请关真车电源**（USB 拦不住 2.4G）。  
USB 无推杆数据时用键盘：`use_joy:=false use_keyboard:=true`。

**存图**：Ctrl+C 或 `ros2 service call /save_costmap_node/save std_srvs/srv/Trigger {}` →  
`$SCOUT_WS_ROOT/3D/maps_long/arena_trav.yaml` + `.pgm`。

开一会儿后 → 回到启动该 launch 的终端 **Ctrl+C** → 自动写入：

- `$SCOUT_WS_ROOT/3D/maps_long/arena_trav.yaml`
- `$SCOUT_WS_ROOT/3D/maps_long/arena_trav.pgm`

（也可不关：`ros2 service call /save_costmap_node/save std_srvs/srv/Trigger {}`）

### 阶段 2 — 导航（点终点 → Arena + TD3 跟车）

**用途**：再起 Gazebo+车+RViz，加载静态代价图；Hybrid A* 出全局路径；Arena 取局部子目标；TD3（只读 **Record 9 仿真权重**）发 `/cmd_vel_td3`；**`nav_cmd_gate`** 到点锁轮后再出 `/cmd_vel`。另挂 **翻车监护**（与建图同逻辑，恢复后自动解锁）。

**先确认阶段 1 已关**（不要两台 gzserver），再：

```bash
bash $SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav/scripts/check_long_nav_idle.sh

ros2 launch scout_long_nav long_nav_nav.launch.py \
  weights:="$RECORD9_WEIGHTS"
```

| 应弹出 | 你做什么 |
|--------|----------|
| Gazebo | 大图 + 小车 |
| RViz | Fixed Frame = **odom**；看热力图 + **鲜红粗路径** + **Arena 彩色球** |

然后：工具栏 **「2D Goal Pose」** → 在地图上点终点。

| 显示 / 行为 | 说明 |
|-------------|------|
| **GlobalPath**（红） | Hybrid A* `/long_nav/global_path`；Billboards 线宽约 **1.44 m**（大图可见） |
| **ArenaWaypoints** | Arena 前瞻段上的局部点 `/long_nav/arena_waypoints`（青→粉球）；近弯远直是正常现象 |
| LocalGoal 箭头 | 默认关（`LocalGoal_OFF`）；控制仍用 `/offroad/local_goal` → bridge → TD3 |
| **到点锁轮** | 距终点 ≤ **2 m** → `GOAL REACHED`，`/cmd_vel` 持续零速；**再点一次 2D Goal Pose** 才继续 |
| **翻车** | 停控 + Z 阶梯原地扶正；`cmd_hold` 心跳；恢复结束应见 `cmd_hold OFF`；超时/丢消息也会自动解锁 |
| Odom 显示 | 默认关；勿开 Covariance（会出大黄盘） |

数据流（导航）：

```text
/goal_pose → /long_nav/goal → Hybrid A* → /long_nav/global_path
  → Arena /offroad/local_goal (+ 球 /long_nav/arena_waypoints)
  → bridge /offroad/train_goal → infer_td3 → /cmd_vel_td3
  → nav_cmd_gate → /cmd_vel
```

### 不要当成试跑的命令

| 命令 | 实际用途 | 为何不是「长导航试跑」 |
|------|----------|------------------------|
| `view_world.launch.py` + arena 世界 | 只看/改地形（无车） | **没有小车、没有导航栈** |
| U 盘 `arena_edit_kit/.../open_world.sh` | 给别人摆树 | 同上 |
| `python3 -m scout_long_nav.bake_heightmap_trav` | 无 Gazebo 时从 heightmap 写 `arena_trav.*` | **不弹任何窗口**；仅缺图文件时用 |

缺图、又不想开阶段 1 时，才跑烘焙，然后再开阶段 2：

```bash
python3 -m scout_long_nav.bake_heightmap_trav \
  --out-dir $SCOUT_WS_ROOT/3D/maps_long --stem arena_trav
```

术语（本文件内自洽）：**traversability** = 坡度/粗糙度「多难开」；实现话题是 **`/long_nav/costmap`**（OccupancyGrid：`-1` unknown，`0–99` 代价，`100` lethal）。没有第二个叫 traversability_map 的话题。

---

## 目录

0. [仿真两段试跑](#ch0)
1. [系统架构与端到端数据流](#ch1)
2. [第 1 层：位姿与建图（FAST-LIO2）](#ch2)
3. [第 2 层：可通行性代价图](#ch3)
3.6. [安全边界 / 无限代价（公式）](#ch3b)
4. [第 3 层：粗全局规划（Hybrid A*）](#ch4)
5. [第 4 层：中间子目标（Arena look-ahead）](#ch5)
6. [第 5 层：局部控制（TD3）](#ch6)
7. [仿真落地 → 实车迁移](#ch7)
8. [文献精华卡](#ch8)
9. [代码命令与验收](#ch9)
10. [状态与缺口](#ch10)

---

<a id="ch1"></a>

## 1. 系统架构与端到端数据流

分层依据：Arena-Rosnav（Kästner et al., 2021）——纯 DRL 缺乏长期记忆、易陷局部极小，长距离须 **全局规划 + 滑动子目标 + 学习局部策略**。

### 1.1 符号

| 符号 | 含义 |
|------|------|
| $\mathbf{q}=(x,y,\theta)$ | 车体平面位姿（地图/里程计系） |
| $\mathbf{g}^{*}$ | 任务全局终点 |
| $\mathcal{M}_{\mathrm{cloud}}$ | LIO/SLAM 累积点云地图 |
| $c(i,j)$ | 栅格可通行代价（越大越难走；lethal / $\infty$ 不可通行） |
| $\mathcal{P}=(\mathbf{p}_{0},\ldots,\mathbf{p}_{M})$ | 粗全局路径折线 |
| $d_{\mathrm{la}}$ | Arena 空间前瞻距离（m） |
| $\mathbf{g}_{\ell}(t)$ | $t$ 时刻局部子目标 |
| $s_{t},\ a_{t}=(v,\omega)$ | TD3 状态与差速动作 |

### 1.2 端到端数据流

```text
LiDAR (+ IMU)
    │
    ▼
┌──────────────────────────┐
│ L1  FAST-LIO2 / 仿真里程计 │  →  q̂ ,  M_cloud
└────────────┬─────────────┘
             │ 投影 / 高程 / 坡度
             ▼
┌──────────────────────────┐
│ L2  Traversability map   │  →  c(i,j)  OccupancyGrid
└────────────┬─────────────┘
             │ Hybrid A*
             ▼
┌──────────────────────────┐
│ L3  粗全局路径 P           │  →  nav_msgs/Path
└────────────┬─────────────┘
             │ look-ahead d_la
             ▼
┌──────────────────────────┐
│ L4  Arena 子目标 g_ℓ      │  →  /offroad/local_goal
│     + 前瞻球可视化         │  →  /long_nav/arena_waypoints
└────────────┬─────────────┘
             │ Point 桥接；(ρ,θ)
             ▼
┌──────────────────────────┐
│ L5  TD3 局部策略          │  →  /cmd_vel_td3
└────────────┬─────────────┘
             │ nav_cmd_gate（到点锁轮 / 翻车 hold）
             ▼
                    /cmd_vel
```

对应形式：$\hat{\mathbf{q}},\ \mathcal{M}_{\mathrm{cloud}}\ \rightarrow\ c(i,j)\ \rightarrow\ \mathcal{P}\ \rightarrow\ \mathbf{g}_{\ell}\ \rightarrow\ (v,\omega)$（仿真经门控）。

### 1.3 话题契约

| 话题 | 类型 | 层 |
|------|------|----|
| `/long_nav/odom` | `nav_msgs/Odometry` | L1 输出（统一下游） |
| `/rslidar_points` 或 `/lidar/points2` | `sensor_msgs/PointCloud2` | L1/L2 输入 |
| `/long_nav/costmap` | `nav_msgs/OccupancyGrid` | L2 |
| `/long_nav/costmap_rgb` | `sensor_msgs/PointCloud2` | L2 仅可视化 |
| `/long_nav/goal` | `geometry_msgs/PoseStamped` | L3 终点 |
| `/long_nav/global_path` | `nav_msgs/Path` | L3 |
| `/offroad/local_goal` | `geometry_msgs/PoseStamped` | L4 |
| `/long_nav/arena_waypoints` | `visualization_msgs/MarkerArray` | L4 可视化 |
| `/offroad/train_goal` | `geometry_msgs/Point` | L4→L5 桥（兼容现 infer） |
| `/cmd_vel_td3` | `geometry_msgs/Twist` | L5 原始（导航） |
| `/long_nav/cmd_hold` | `std_msgs/Bool` | 翻车恢复心跳 hold |
| `/long_nav/goal_arrived` | `std_msgs/Bool` | 到点锁轮状态 |
| `/cmd_vel` | `geometry_msgs/Twist` | 门控后下发底盘 |

坐标系：真车 `map`（LIO）→ `odom` → `base_link`；仿真阶段可令 $map \equiv odom \equiv world$。

---

<a id="ch2"></a>

## 2. 第 1 层：位姿与建图（FAST-LIO2）

### 2.1 问题

为规划与 TD3 提供连续六自由度/平面位姿 $\hat{\mathbf{q}}$，并维护可用于投影的点云地图 $\mathcal{M}_{\mathrm{cloud}}$。  
**不**直接把点云交给 A*。

### 2.2 算法选型

| 场景 | 算法 | 角色 |
|------|------|------|
| 真车多线 + IMU | **FAST-LIO2**（Xu et al., TRO 2022） | **主链** |
| 仿真阶段 A/B | Gazebo `/odom` 或 `pose_stub_node` | 不强制上 LIO |
| 对照 | LIO-SAM / KISS-ICP | ablation，非必装 |

文献：[`article/建图/2107.06829v1.pdf`](../article/建图/2107.06829v1.pdf) · 开源 https://github.com/hku-mars/FAST_LIO  
综述脉络：[`2311.00276v1.pdf`](../article/建图/2311.00276v1.pdf)

### 2.3 公式要点（写材料用）

FAST-LIO2：紧耦合迭代卡尔曼滤波（iEKF），状态含位姿、速度、IMU 偏置等。观测为 **点到地图** 残差。记一次迭代中误差状态为 $\delta\mathbf{x}$，则：

$$
\delta\mathbf{x}_{k+1} = (I - \mathbf{K}\mathbf{H})\,\delta\mathbf{x}_{k}^{\mathrm{prop}} + \mathbf{K}\,\mathbf{z}
$$

其中 $\mathbf{H}$ 为点到平面/点到图雅可比，$\mathbf{K}$ 为卡尔曼增益，$\mathbf{z}$ 为堆叠残差。地图侧用 **ikd-Tree** 维护动态体素，支持直接点配准（相对特征 LOAM 系更轻）。

工程输出（需 remap）：

- 里程计 → `/long_nav/odom`
- 配准点云 / 地图 → 供 L2 投影

### 2.4 命令：位姿接入（仿真转发 / 真车拉 LIO 源码）

**仿真日常试跑不必手敲下面**：§0 的 `long_nav_map` / `long_nav_nav` 已内含 `pose_stub`。  
下面仅在你 **单独调试某一层** 或准备真车编译 FAST-LIO 时用。

```bash
# 拉取 FAST-LIO 源码（真车用；本机网络可用时）
bash $SCOUT_WS_ROOT/ros2_ws/src/third_party/clone_fast_lio.sh
# 编译状态备忘：ros2_ws/src/third_party/BUILD_NOTES.md

# 单独把 Gazebo /odom 转成下游统一话题 /long_nav/odom
ros2 run scout_long_nav pose_stub_node --ros-args \
  -p input_odom_topic:=/odom -p output_odom_topic:=/long_nav/odom
```

真车外参占位参数文件：[`scout_long_nav/config/real_robot_long.yaml`](../ros2_ws/src/navigation_3d/scout_long_nav/config/real_robot_long.yaml)。

---

<a id="ch3"></a>

## 3. 第 2 层：可通行性代价图（Traversability / Costmap）

### 3.1 问题

将 $\mathcal{M}_{\mathrm{cloud}}$（或在线点云）转为规划器可读的二维代价场 $c(i,j)$。越野场景 **占用二值不够**：空地上的陡坡、峭壁仍应高代价或 lethal。

术语：

| 名称 | 含义 |
|------|------|
| Occupancy grid | 占用概率 / 离散代价（ROS `OccupancyGrid`） |
| Traversability map | 连续或分级「多难开」；本项目用几何坡度+粗糙度实现第一版 |
| DEM / 2.5D | 每格高程 $h_{ij}$ 及方差 |

文献（可通行性）[`article/建图/`](../article/建图/)：Borges *Field Robotics* 2022 综述、IEEE Access 2022 综述、Fan risk-aware costmaps、Castro self-supervised costmap、OctoMap。

### 3.2 本项目几何模型（与 `costmap_geom.py` 一致）

对局部窗口栅格（分辨率 $r$，边长 $L$，中心为车位姿），落入格 $(i,j)$ 的点集估高程均值 $\bar{h}_{ij}$ 与粗糙度（高程标准差）$\sigma_{ij}$。邻域坡度：

$$
s_{ij} = \max_{(i',j')\in\mathcal{N}(i,j)} \frac{|\bar{h}_{i'j'}-\bar{h}_{ij}|}{r}
$$

通行代价（$\alpha,\beta$ 为权重，$s_{\max}$ 为坡度阈值）：

若 $s_{ij} > s_{\max}$，则 $c_{ij} = 100$（lethal）；否则

$$
c_{ij} = \mathrm{clip}\!\left(\alpha\frac{s_{ij}}{s_{\max}}+\beta\,\sigma_{ij},\ 0,\ 99\right)
$$

未观测格记为 $-1$（unknown）。Hybrid A* 边代价对 $c$ 单调递增惩罚；lethal 不可扩展。

默认参数（`long_nav.yaml`）：$r=0.2\,\mathrm{m}$，$L=40\,\mathrm{m}$，$s_{\max}=0.45$。

### 3.3 与 OctoMap 的关系

OctoMap（Hornung et al., 2013）用八叉树存三维占用概率；导航仍需 **切片/投影** 到 2D 或 2.5D。第一版不强制运行 OctoMap，几何投影已够 Hybrid A*；第二迭代可：

$$
\mathcal{M}_{\mathrm{cloud}} \rightarrow \mathrm{OctoMap} \rightarrow \mathrm{2D\ projection} \rightarrow c(i,j)
$$

PDF：[`建图/hornung13auro_OctoMap.pdf`](../article/建图/hornung13auro_OctoMap.pdf) · https://github.com/OctoMap/octomap

### 3.4 学习型代价（材料引用，非当前必装）

Fan et al.：风险感知 costmap，在几何/语义特征上回归不确定代价（[`2106.08114v4.pdf`](../article/建图/2106.08114v4.pdf)）。  
Castro et al.：自监督 proprioception→cost（[`2209.10788v3.pdf`](../article/建图/2209.10788v3.pdf)）。  
比赛/结项第一版用 **几何 $c_{ij}$**；学习代价作 related work / 后续。

### 3.5 命令：单独起「点云 → 代价图」节点

**用途**：只测 L2。订阅点云与里程计，发布 `/long_nav/costmap`。  
**仿真完整试跑不必手敲**：§0 阶段 1/2 的 launch 已带上 costmap（在线或静态加载）。

```bash
ros2 run scout_long_nav cloud_to_costmap_node --ros-args \
  -p cloud_topic:=/lidar/points2 \
  -p odom_topic:=/long_nav/odom \
  -p costmap_topic:=/long_nav/costmap
```

离线几何单测（不启 ROS）：`python3 -m pytest ros2_ws/src/navigation_3d/scout_long_nav/tests/test_costmap.py -q`

<a id="ch3b"></a>

### 3.6 安全边界 / 无限代价（公式）

> **软件怎么点边界、弹什么窗** → [`README3.md` §12](README3.md#ch12)。本节只写数学与实现约定。

#### 代价码（与 `cost_codes.py` 一致）

| 语义 | 盘上 / 内存 | OccupancyGrid（int8 线载） | PGM 像素 |
|------|-------------|---------------------------|----------|
| 未知 | $-1$ | $-1$ | $255$ |
| 地形可通行代价 | $c_{\mathrm{terr}}\in[0,100]$ | 同左（有限） | 同左 |
| **安全红线** | $c_{\mathrm{sec}}=1000$ | $127$（`OCC_SECURITY`） | $254$ |

规划硬约束：安全格 **不可通过**，等价于无限代价：

$$
c(i,j)=\begin{cases}
\infty, & (i,j)\in\mathcal{S}\ \text{（安全围栏格）}\\
c_{\mathrm{terr}}(i,j), & \text{否则（含未知的有限惩罚）}
\end{cases}
$$

实现上：触碰安全格的运动直接拒绝（返回「无可行段」），而不是用很大的有限标量去「绕一下」。

#### 围栏栅格化（`fence_bake.py`）

闭合多边形顶点 $\{p_k\}_{k=0}^{N-1}$（$p_N:=p_0$），半宽 $w$（如 `fence_half_width_m\approx0.45\,\mathrm{m}$）。对每条边与栅格中心做走廊判定：落在边的 $w$-邻域内的格写入

$$
c(i,j)\leftarrow 1000.
$$

保存到 `maps_security`；`maps_long` 保持未烘焙的建图结果。

#### 段代价（`hybrid_astar._segment_traversal_cost`）

沿段 $\overline{\mathbf{x}_a\mathbf{x}_b}$ 以约半格步长采样 $\{\mathbf{x}^{(t)}\}$。若任一样本落在安全格（或出图）：

$$
J(\overline{\mathbf{x}_a\mathbf{x}_b})=\infty
$$

（代码返回 `None`）。否则累加有限代价（未知/地形加权），供 Hybrid / 2D 搜索使用。

#### 连通性快检（拒终点 / 判真穿红线）

仅把安全格当障碍的四邻域 BFS：

$$
\texttt{security\_blocks\_connection}(\mathbf{s},\mathbf{g})=\mathbf{1}\{\mathbf{s}\text{ 在 }\mathcal{S}\text{-障碍图上不可达 }\mathbf{g}\}
$$

- 为真且用户点了新 Goal → 发 `/long_nav/goal_rejected`（**不切换**当前任务）。  
- 为真且当前任务持续无安全路径 → 发 `/long_nav/nav_abort`（**中止**任务并清红路径）。

地形 $0..100$ **不**参与该快检的硬阻断（昂贵但可走）。

#### 可视化颜色（与规划码一致）

- 地形 $0\to100$：薄荷绿 → 日落橙。  
- 仅安全码（$127/1000$）：纯亮红（与地形 $100$ 的橙区分）。

---

<a id="ch4"></a>

## 4. 第 3 层：粗全局规划（Hybrid A*）

### 4.1 问题

在代价场 $c$ 上求从 $\mathbf{q}$ 到 $\mathbf{g}^{*}$ 的 **运动学可行** 粗路径 $\mathcal{P}$。室内平坦可用 2D A*；越野 + 差速/近似自行车模型用 Hybrid A*（Dolgov et al., 2008）。

PDF：[`导航/dolgov_gpp_stair08.pdf`](../article/导航/dolgov_gpp_stair08.pdf)  
实现参考：PythonRobotics HybridAStar（MIT，本仓简化移植，无 Reeds-Shepp 解析扩展）。

### 4.2 普通 A*（对照）

离散格节点 $(i,j)$，边权常为欧氏长度（可再乘 $c$）。启发式 $h$ 多为到目标的欧氏距离。输出折线，**不保证**满足最小转弯半径。

### 4.3 Hybrid A* 状态与运动学

搜索节点携带连续位姿，并量化到网格键：

$$
\mathbf{s}=(x,y,\theta)
$$

$$
(i_{x},i_{y},i_{\theta}) = \Big(\mathrm{round}\frac{x-x_{0}}{\Delta_{xy}},\ \mathrm{round}\frac{y-y_{0}}{\Delta_{xy}},\ \mathrm{round}\frac{\theta}{\Delta_{\theta}}\Big)
$$

自行车模型一步扩展（步长 $\Delta s$，转角 $\phi$，轴距 $L$）：

$$
x' = x + \Delta s\cos\theta,\quad
y' = y + \Delta s\sin\theta,\quad
\theta' = \theta + \frac{\Delta s}{L}\tan\phi
$$

$\phi\in[-\phi_{\max},\phi_{\max}]$ 离散采样；$\Delta s$ 可取正（前进）或负（倒车，附加惩罚）。

### 4.4 代价与启发式

记从父节点走到子节点的路径代价增量为：

$$
g(\mathbf{s}') = g(\mathbf{s}) + |\Delta s|\cdot\big(1+\kappa\,c(x',y')\big) + \lambda_{\mathrm{rev}}\cdot\mathbf{1}[\Delta s<0] + \lambda_{\phi}\,|\phi|
$$

其中 $c(x',y')$ 由 OccupancyGrid 最近邻读取；若格为 lethal（$\ge 90$）则剪枝。启发式（可采纳简化版）：

$$
h(\mathbf{s}) = \|\mathbf{p}-\mathbf{g}^{*}\|_{2} + \mu\,|\mathrm{wrap}(\theta-\theta_{g})|
$$

优先队列按 $f=g+h$ 弹出。目标条件：$\|\mathbf{p}-\mathbf{g}^{*}\|_{2}\le \varepsilon_{xy}$ 且航向差 $\le\varepsilon_{\theta}$。

本仓参数默认：`xy_resolution=0.5`，`max_iterations=15000`，局部窗口随 L2 的 $40\,\mathrm{m}$ 图。

### 4.5 命令：单独起 Hybrid A* + 手发终点

**用途**：只测 L3。有 costmap 与 odom 时，往 `/long_nav/goal` 发 PoseStamped，看 `/long_nav/global_path`。  
**完整试跑不必手敲**：§0 阶段 2 的 `long_nav_nav` 已含本节点；RViz「2D Goal Pose」会转到 `/long_nav/goal`。

```bash
ros2 run scout_long_nav hybrid_astar_node --ros-args \
  -p costmap_topic:=/long_nav/costmap \
  -p odom_topic:=/long_nav/odom \
  -p goal_topic:=/long_nav/goal \
  -p path_topic:=/long_nav/global_path

# 不用 RViz 时，命令行发一次终点示例
ros2 topic pub --once /long_nav/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 20.0, y: 5.0}, orientation: {w: 1.0}}}"
```

离线单测：`python3 -m pytest ros2_ws/src/navigation_3d/scout_long_nav/tests/test_hybrid_astar.py -q`

---

<a id="ch5"></a>

## 5. 第 4 层：中间子目标（Arena look-ahead）

### 5.1 问题

DRL 状态中的目标极坐标 $(\rho,\theta)$ 在长距离上会饱和；需沿 $\mathcal{P}$ 提供 **滑动局部目标** $\mathbf{g}_{\ell}$，使 TD3 始终跟踪有限地平线内的点。

文献：Arena-Rosnav [`导航/2104.03616v2.pdf`](../article/导航/2104.03616v2.pdf) · https://github.com/ignc-research/arena-rosnav  
本仓 **不**整仓依赖 Arena，仅实现 look-ahead 节点。

### 5.2 公式

设 $\mathcal{P}$ 为折线。先求 $\mathbf{q}$ 在折线上的最近点（投影），再沿弧长前进 $d_{\mathrm{la}}$：

$$
\mathbf{g}_{\ell}(t) = \mathrm{LookAhead}\big(\mathcal{P},\ \hat{\mathbf{q}}(t),\ d_{\mathrm{la}}\big)
$$

若剩余弧长 $<d_{\mathrm{la}}$，则 $\mathbf{g}_{\ell}=\mathbf{p}_{M}$（终点）。默认 $d_{\mathrm{la}}=4\,\mathrm{m}$。

实现上还会沿「投影点 → 前瞻点」按约 `waypoint_spacing_m=0.8` 采样，发布 **彩色球** `/long_nav/arena_waypoints`（近青远粉），便于 RViz 核对局部轨迹；TD3 仍只吃单个 `local_goal`。

### 5.3 命令：单独起 Arena look-ahead + 目标桥

**用途**：只测 L4。沿全局路径按前瞻距离取局部子目标，并桥成 TD3 要的 `Point`。  
**完整试跑不必手敲**：§0 阶段 2 已含这两节点。

```bash
ros2 run scout_long_nav arena_local_goal_node --ros-args \
  -p path_topic:=/long_nav/global_path \
  -p odom_topic:=/long_nav/odom \
  -p local_goal_topic:=/offroad/local_goal \
  -p waypoints_marker_topic:=/long_nav/arena_waypoints \
  -p lookahead_m:=4.0

ros2 run scout_long_nav goal_bridge_node --ros-args \
  -p local_goal_topic:=/offroad/local_goal \
  -p train_goal_topic:=/offroad/train_goal
```

阶段 B（无代价图、假折线路径联调 look-ahead）：

```bash
ros2 launch scout_long_nav stage_b_fake_path.launch.py
# 训练占用 Gazebo 时不要 live launch；默认 smoke 用 DRY_RUN=1
```

---

<a id="ch6"></a>

## 6. 第 5 层：局部控制（TD3）

### 6.1 问题

在扇区距离特征与相对子目标 $(\rho,\theta)$ 下输出连续动作 $(v,\omega)$。网络与奖励见 [`README1.md`](README1.md)；长途栈 **只更换目标源**。

### 6.2 状态（当前 Record 9：150 维）

$$
s = \big[d_{b,i}\ (b=1\ldots4,\ i=1\ldots36),\ \rho/10,\ \theta/\pi,\ v_{\mathrm{prev}}/v_{\max},\ \omega_{\mathrm{prev}}/\omega_{\max},\ \mathrm{roll}/\pi,\ \mathrm{pitch}/\pi\big]
$$

$$
\rho=\|\mathbf{g}_{\ell}-\hat{\mathbf{p}}\|_{2},\quad
\theta=\mathrm{wrap}\big(\mathrm{atan2}(\Delta y,\Delta x)-\hat{\theta}\big)
$$

$$
a=(v,\omega)
$$

即 $4\times36+4+2=150$ 维。`long_nav_nav` 将 TD3 的输出先发到 `/cmd_vel_td3`，再经 `nav_cmd_gate`（到点锁轮、翻车监护）转到 `/cmd_vel`；单独推理的默认输出才是 `/cmd_vel`。
TD3 三要素（Fujimoto et al., 2018，[`导航/1802.09477v3.pdf`](../article/导航/1802.09477v3.pdf)）：双 Q 取 min、延迟策略更新、目标动作平滑。

### 6.3 命令：单独起 TD3 推理（接长导航目标）

**用途**：只测 L5。加载冻结权重，订阅 `/offroad/train_goal`，发 `/cmd_vel`。  
**完整试跑不必手敲**：§0 阶段 2 的 `long_nav_nav` 已按下面参数 include（`start_sim:=false`，不另起训练短图）。

```bash
ros2 launch scout_offroad_3d infer_td3.launch.py \
  start_sim:=false \
  auto_goal:=false \
  pose_source:=odom \
  goal_topic:=/offroad/train_goal \
  n_elevation_bins_td3:=4 hidden_dim:=1024 use_imu_in_state:=true \
  weights:="$RECORD9_WEIGHTS"
```

**禁止**在 `train_td3` 占用 `gzserver` 时再开带仿真的 infer。日常联调用文首 §0。

---

<a id="ch7"></a>

## 7. 仿真落地 → 实车迁移

### 7.1 仿真（当前优先）

**完整有车导航试跑 = 文首 §0**（`long_nav_map` → 存图 → `long_nav_nav`）。不要用下面的 `view_world` 代替 §0。

栈顺序（§0 的 launch 已串好）：

1. 确认训练未占 Gazebo  
2. `/odom` → `/long_nav/odom`（pose_stub）  
3. 代价图 → Hybrid A* → Arena → goal_bridge → infer_td3  
4. 若有短期评测曲线，验证结论后移入 `archive/validation/long_nav/`；正式实验须保留配置、指标和索引（出图工具在 README2）

大图文件路径：`$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/arena/bumpy_offroad_full.world`

- 地形：`model://uneven_ground`，**80×80×2 m**（约 $[-40,40]^2$）  
- 围墙：`boundary_wall_n/s/e/w`（厚 1 m、高 5 m）  
- 默认 `real_time_factor=5`（GUI 左下角可再调）  
- **勿**用 TD3 的 `uneven_ground_td3_*` 覆盖外部管理的 `uneven_ground` 模型
- 大图依赖一个显式管理的外部模型根；例如设置 `EXTERNAL_GAZEBO_MODELS=/绝对/模型根` 后：`export GAZEBO_MODEL_PATH="$EXTERNAL_GAZEBO_MODELS${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"`，并验证 `uneven_ground/model.sdf` 存在

**仅看地形、不带车、不导航**（改图 / 验围墙时用）：

```bash
source /opt/ros/humble/setup.bash
source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash
ros2 launch scout_offroad_3d view_world.launch.py \
  world:=$SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_offroad_3d/arena/bumpy_offroad_full.world
```

### 7.2 实车（要点写在本文件；SSH 探车纪要才去 README3）

| 项 | 定稿 |
|----|------|
| 点云 | `/rslidar_points` |
| 底盘里程计 | `/odom`（scout_base）；LIO 输出 remap 为 `/long_nav/odom` |
| 控制 | TD3 → `/cmd_vel`；勿与厂方 `navigation_4wd` / move_base 同时发 |
| ROS | 车端 Noetic；ROS1 中间层只保留为历史草稿，未纳入当前工作区，也不是可执行入口 |
| 安全 | `enable_cmd_vel` 默认 **关**；首测 **`max_v≤0.3` m/s**、`max_w≤0.8`；真车 **`allow_reverse: false`**（不开倒车）；人持遥控可接管 |

迁移检查：当前仿真接口是特征维 $4\times36+4+2=150$、`hidden_dim=1024`，与 Record 9 配置一致。**这只验证仿真接口，不能构成实车部署许可。** 实车须在独立安全计划下完成传感器外参、接口限幅、急停、低速人工监护和重新验证后，才讨论 FAST-LIO2 与策略迁移；不得直接加载 Record 9 仿真权重上车。

---

<a id="ch8"></a>

## 8. 文献精华卡（写材料用）

文件已按类归档。下表只列 **结项/比赛写作直接用得上的点**。

### 8.1 `article/导航/`

| 文件 | 用途 | 可引用要点 |
|------|------|------------|
| `2104.03616v2.pdf` Arena-Rosnav | 分层长距离 | DRL 不适单独 long-range；全局路径 + moving local sub-goal + DRL 局部 |
| `1802.09477v3.pdf` TD3 | 局部策略 | twin Q / delayed update / target policy smoothing |
| `1801.01290v2.pdf` SAC | 对照 | 最大熵；related work 并列 |
| `sensors-25-03394-v2.pdf` | DRL 导航综述 | 动态环境 DRL 分类与难点 |
| `dolgov_gpp_stair08.pdf` Hybrid A* | 粗全局 | 运动学状态网格；可执行路径；启发式组合 |

### 8.2 `article/建图/`

| 文件 | 用途 | 可引用要点 |
|------|------|------------|
| `2107.06829v1.pdf` FAST-LIO2 | 真车位姿主链 | iEKF + ikd-Tree 直接点配准 |
| `2007.00258v3.pdf` LIO-SAM | 对照 | 因子图；回环/GPS |
| `2209.15397v2.pdf` KISS-ICP | 轻量对照 | 少调参点到点 ICP |
| `2311.00276v1.pdf` | SLAM 综述 | LOAM→LIO→FAST-LIO 脉络 |
| `hornung13auro_OctoMap.pdf` | 地图表示 | 八叉树占用；需投影才能 2D 规划 |
| `A_Survey_on_Terrain_Traversability_…pdf` Borges | 可通行性综述 | 几何 / 外观 / 学习分类 |
| `A_Survey_of_Traversability_Estimation_….pdf` | 可通行性综述 | elevation → traversability 管线 |
| `2106.08114v4.pdf` Fan | 学习 costmap | risk-aware 代价 |
| `2209.10788v3.pdf` Castro | 自监督 costmap | 越野 proprioception |

开源速查：FAST_LIO、OctoMap、PythonRobotics HybridAStar、Arena-Rosnav、sfujim/TD3（见各节链接与 [`scout_long_nav/NOTICE.md`](../ros2_ws/src/navigation_3d/scout_long_nav/NOTICE.md)）。

---

<a id="ch9"></a>

## 9. 代码命令与验收

### 9.1 代码期（不抢仿真）

```bash
cd $SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav
python3 -m pytest tests -q

cd $SCOUT_WS_ROOT/ros2_ws
colcon build --packages-select scout_long_nav
bash src/navigation_3d/scout_long_nav/scripts/smoke_long_nav_stage_b.sh   # DRY_RUN=1
```

### 9.2 空窗联调

与文首 **§0** 相同（不必另开文件）：

```bash
export ROS_DOMAIN_ID=41
bash $SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav/scripts/check_long_nav_idle.sh
source /opt/ros/humble/setup.bash && source $SCOUT_WS_ROOT/ros2_ws/install/setup.bash
ros2 launch scout_long_nav long_nav_map.launch.py
# Ctrl+C 存图后再：
ros2 launch scout_long_nav long_nav_nav.launch.py \
  weights:="$RECORD9_WEIGHTS"
```

离线 smoke（默认不启 Gazebo）：

```bash
DRY_RUN=0 I_CONFIRM_TRAINING_IDLE=yes \
  bash $SCOUT_WS_ROOT/ros2_ws/src/navigation_3d/scout_long_nav/scripts/smoke_long_nav_stage_b.sh
```

### 9.3 验收标准

| 阶段 | 标准 |
|------|------|
| 建图 | Flysky/键盘扫图；Ctrl+C 或 save 服务写出 `arena_trav.*`；可选 `map_*` 备份；翻车可 Z 阶梯重生 |
| 续建 | `seed_map_yaml` 加载后 RViz 仍见旧图，继续累积 |
| B | 假/`global_path` 上 `local_goal` 滑动；有 Arena 球；桥接后 TD3 收到 `Point` |
| C / 导航 | Hybrid A* 非空红路径；TD3 跟车；到点≤2 m 锁轮；新 Goal Pose 再开；翻车恢复后 `cmd_hold OFF` 可继续 |
| 真车 | **不以本仿真通过作为上车验收**；须另立安全计划、接口审查与人工监护的低速验证 |

---

<a id="ch10"></a>

## 10. 状态与缺口

| 已具备 | 待标定 / 后续 |
|--------|----------------|
| 仿真回环：`long_nav_map` ↔ `long_nav_nav` + 隔离门禁 | 真车 FAST-LIO2 编译与外参 |
| 建图：Flysky、翻车监护、续建 `seed_map_yaml`、时刻备份 | 真长距实验的独立配置、指标与归档索引 |
| 导航：红路径 + Arena 球 + 到点锁轮 + 翻车解锁 | 学习型 traversability（可选） |
| `nav_cmd_gate` / `rollover_guard` / `arena` 球可视化 | 真车端同契约试车 |
| 离线单测 + map_io + Record 9 仿真权重只读 | — |

---

## 相关文档

- 本文件文首 **§0** = 仿真试跑唯一入口  
- 专题：[`README0.md`](README0.md) · [`README1.md`](README1.md)（TD3 训练）· [`README2.md`](README2.md)（出图）· [`README3.md`](README3.md)（真车 SSH 纪要）  
- [`article/article.md`](../article/article.md) · [`article/record.md`](../article/record.md)
