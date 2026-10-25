# scout_mini_pro_description

方块车 xacro / room 世界，以及官方车 room 仿真。它是共享描述包，
不承载 2D 或 3D 的导航算法入口。

## 启动

```bash
source /opt/ros/humble/setup.bash
source "$SCOUT_WS_ROOT/ros2_ws/install/setup.bash"

# 仅启动共享仿真环境
ros2 launch scout_mini_pro_description gazebo_room.launch.py
```

导航算法不属于这个共享描述包。本机的 3D 入口见仓库根目录的 [`3D/README.md`](../../../../3D/README.md)；
2D 已外置到 [scout-navigation-2d](https://github.com/mofajilu1314/scout-navigation-2d)。
