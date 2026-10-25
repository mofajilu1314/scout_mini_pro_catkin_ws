#!/usr/bin/env bash
# Sidecar RViz watch for an already-running TD3 train. Does NOT start/stop training.
# Usage (server RDP desktop, DISPLAY=:10):
#   SCOUT_WS_ROOT=/path/to/workspace bash 3D/scripts/watch_train_rviz.sh
set -euo pipefail

# Never sudo. Never kill gzserver / train_td3 / ros2 launch.
if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  echo "refuse: do not run this as root/sudo" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS_ROOT="${SCOUT_WS_ROOT:-${CATKIN_WS:-${WS_ROOT:-${DEFAULT_WS_ROOT}}}}"
ROS2_WS="${WS_ROOT}/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "refuse: ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${ROS2_WS}/install/setup.bash" ]]; then
  echo "refuse: workspace overlay is missing: ${ROS2_WS}/install/setup.bash" >&2
  echo "Build the active ROS 2 workspace before starting this RViz sidecar." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROS2_WS}/install/setup.bash"

export DISPLAY="${DISPLAY:-:10}"
unset LD_PRELOAD

if [[ -z "${DISPLAY}" ]]; then
  echo "refuse: no DISPLAY. Open this from RDP desktop (:10), not a headless SSH." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null || ! command -v rviz2 >/dev/null; then
  echo "refuse: ros2/rviz2 not on PATH after source" >&2
  exit 1
fi

CFG="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/config/train_watch.rviz"
if [[ ! -f "${CFG}" ]]; then
  echo "refuse: missing ${CFG}" >&2
  exit 1
fi

_node_running() {
  local pat="$1"
  ros2 node list 2>/dev/null | grep -qE "${pat}" && return 0
  pgrep -u "$(id -un)" -f "${pat}" >/dev/null 2>&1
}

HELPER_PIDS=()
_start_helper() {
  local exe="$1"
  local pat="$2"
  if _node_running "${pat}"; then
    echo "[watch] already running: ${pat}"
    return 0
  fi
  echo "[watch] start ${exe}"
  ros2 run scout_offroad_3d "${exe}" --ros-args -p use_sim_time:=true &
  HELPER_PIDS+=("$!")
}

echo "[watch] DISPLAY=${DISPLAY}  (Ctrl+C closes RViz only; training stays)"
_start_helper train_goal_marker train_goal_marker
_start_helper depth_viz_bridge depth_viz_bridge

_cleanup() {
  local pid
  for pid in "${HELPER_PIDS[@]}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap _cleanup EXIT INT TERM

rviz2 --ros-args -p use_sim_time:=true -- -d "${CFG}"
