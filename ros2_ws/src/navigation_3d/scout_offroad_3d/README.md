# scout_offroad_3d

ROS2 Humble 的越野感知与 TD3 局部策略包：点云特征 → TD3 → 仿真 `/cmd_vel`。

## 当前入口

从工作区根目录设置 `SCOUT_WS_ROOT` 后，按 [`3D/HUMBLE_QUICKSTART.md`](../../../../3D/HUMBLE_QUICKSTART.md) 构建并运行最小验证。训练语义、显式续训条件和 Record 9 仿真基线见 [`3D/README1.md`](../../../../3D/README1.md)。

不要使用本包以外的 2D launch 作为 3D 扇区或训练前置条件；共享仿真模型由 `scout_mini_pro_description` 提供。

## 包内边界

- `config/td3_train.yaml`：唯一活动 TD3 参数源。
- `launch/train_td3.launch.py`：默认新训练；续训必须显式 `resume:=true`。
- `launch/infer_td3.launch.py`：仿真推理入口；使用历史权重前先确认维度兼容。
- `worlds/`、`models/`、`arena/`：活动仿真资产；备份资产不应留在这里。

实车控制不属于本包的默认行为，见 [`3D/README3.md`](../../../../3D/README3.md)。
