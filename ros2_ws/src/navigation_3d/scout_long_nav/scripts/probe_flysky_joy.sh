#!/usr/bin/env bash
# 标定 Flysky USB 手柄：打印 /joy，推杆时看哪根轴/哪个按钮在变。
set -euo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-41}"
DEV_NAME="${1:-Flysky FS-AGX-i6S emulator}"

echo "设备名: $DEV_NAME"
echo "安全：控仿真请先关真车电源。本脚本只读 USB HID，不发 /cmd_vel。"
echo "另开终端可: ros2 topic echo /joy"
echo "Ctrl+C 结束。"
exec ros2 run joy joy_node --ros-args \
  -p device_name:="$DEV_NAME" \
  -p deadzone:=0.05 \
  -p autorepeat_rate:=20.0
