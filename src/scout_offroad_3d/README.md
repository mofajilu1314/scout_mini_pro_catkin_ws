# scout_offroad_3d

越野感知 + **阶段 A TD3** 局部策略包。

| 能力 | 状态 |
|------|------|
| `/scan` → `/offroad/range_features` | ✅ |
| TD3 网络 / replay / 在线训练 / `train_td3.launch` | ✅ 骨架可启 |
| Arena / A\* / SLAM | ❌ 阶段 B/C（**现在不要做**） |

文档：**总规划** [`3D/README0.md`](../../3D/README0.md) · **完整操作手册（必读）** [`3D/README1.md`](../../3D/README1.md)

---

## 前置（每个新终端）

```bash
cd ~/catkin_ws_2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 换 launch 前建议：killall -9 gzserver gzclient roslaunch rosmaster 2>/dev/null
python3 -m pip install --user torch   # Noetic = Python 3.8
```

---

## 一键训练（阶段 A）

```bash
# 有界面
roslaunch scout_offroad_3d train_td3.launch gui:=true

# 无界面（默认 gui:=false）
roslaunch scout_offroad_3d train_td3.launch gui:=false

# 短冒烟
roslaunch scout_offroad_3d train_td3.launch gui:=true max_timesteps:=2000 start_timesteps:=200

# 正常训练（默认 max_timesteps=50000，见 config/td3_train.yaml）
roslaunch scout_offroad_3d train_td3.launch gui:=false max_timesteps:=50000
```

产物：`~/.ros/scout_offroad_td3/run_YYYYMMDD_HHMMSS/`  
内含 `td3_actor` / `td3_critic`、`train.log`、`config.yaml`。

详情：[README1 · TD3 训练](../../3D/README1.md#td3-train)

---

## 推理

```bash
# weights = 训练目录下的前缀「td3」（不要写 td3_actor）
RUN=$HOME/.ros/scout_offroad_td3/run_YYYYMMDD_HHMMSS
roslaunch scout_offroad_3d infer_td3.launch gui:=true weights:=$RUN/td3

# 另开终端发目标（否则不动）
rostopic pub -1 /offroad/train_goal geometry_msgs/Point "{x: 2.0, y: 1.5, z: 0.0}"
```

详情：[README1 · TD3 推理](../../3D/README1.md#td3-infer)

---

## 扇区自检（步骤2 复习）

```bash
roslaunch scout_mini_pro_description gazebo_room_official.launch
# 另终端
roslaunch scout_offroad_3d range_features.launch
rostopic echo /offroad/range_features -n 1
```
