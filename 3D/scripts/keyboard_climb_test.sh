#!/usr/bin/env bash
# Keyboard climb test: bumpy world + Scout + teleop_twist_keyboard.
# Use i/j/k/l (or arrows, depending on teleop version) to drive; check steep slopes.
# Ctrl+C cleans up.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS_ROOT="${SCOUT_WS_ROOT:-${CATKIN_WS:-${WS_ROOT:-${DEFAULT_WS_ROOT}}}}"
ROS2_WS="${WS_ROOT}/ros2_ws"
WORLD_STEM="${1:-bumpy_offroad}"
GZ_PID=""

port_11345_busy() {
  if command -v ss >/dev/null 2>&1; then
    ss -lnt 2>/dev/null | awk '
      NR > 1 && $4 ~ /:11345$/ { found = 1 }
      END { exit !found }
    '
    return
  fi
  if command -v fuser >/dev/null 2>&1; then
    fuser 11345/tcp >/dev/null 2>&1
    return
  fi
  return 1
}

stop_owned_launch() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return 0
  kill -0 "${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }
  echo "==> 停止本次 Gazebo 启动（pid=${pid}）..."
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 25); do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.2
  done
  if kill -0 "${pid}" 2>/dev/null; then
    echo "WARN: 本次启动未及时退出，发送 SIGKILL 给该已记录 PID。" >&2
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  stop_owned_launch "${GZ_PID:-}"
  GZ_PID=""
}
trap cleanup EXIT

if port_11345_busy; then
  echo "FAIL: 端口 11345 正由其他 Gazebo/ROS 会话使用；不会终止未知进程。" >&2
  echo "请在原终端停止占用会话后重试。" >&2
  exit 1
fi

# shellcheck source=/dev/null
set +u
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "${ROS2_WS}/install/setup.bash"
set -e
export SCOUT_WS_ROOT="${WS_ROOT}"

OFFROAD_MODELS="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/models"
if [[ -d "${OFFROAD_MODELS}" ]]; then
  export GAZEBO_MODEL_PATH="${OFFROAD_MODELS}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
fi

WORLD="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/${WORLD_STEM}.world"
if [[ ! -f "${WORLD}" ]]; then
  echo "FAIL: world not found: ${WORLD}"
  exit 1
fi

if ! ros2 pkg prefix teleop_twist_keyboard >/dev/null 2>&1; then
  echo "FAIL: teleop_twist_keyboard 未安装。sudo apt install ros-humble-teleop-twist-keyboard"
  exit 1
fi

echo "==> Gazebo GUI + Scout on ${WORLD_STEM}"
echo "    另开终端已自动起 teleop；本脚本前台跑键盘节点（需焦点在此终端）。"
echo "    试冲陡坡：全速前进能否上去；打滑/翻车则记下。轮 μ 原厂 1.0/0.9，地面不补丁；勿擅自改摩擦。"
echo ""

if port_11345_busy; then
  echo "FAIL: 端口 11345 在启动前被其他会话占用；不会终止未知进程。" >&2
  exit 1
fi
ros2 launch scout_mini_pro_description gazebo_room.launch.py \
  gui:=true \
  world:="${WORLD}" \
  z:=0.23 &
GZ_PID=$!

echo "==> 等待 spawn / cmd_vel ..."
for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q '^/cmd_vel$'; then
    break
  fi
  sleep 1
done

echo "==> teleop_twist_keyboard → /cmd_vel（speed 可在节点内调）"
# speed/turn defaults; user can raise for climb test
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=false

wait "${GZ_PID}" || true
