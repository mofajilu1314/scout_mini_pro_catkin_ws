#!/usr/bin/env bash
# Planning-stack smoke. Default DRY_RUN=1.
set -euo pipefail

DRY_RUN="${DRY_RUN:-1}"
I_CONFIRM_TRAINING_IDLE="${I_CONFIRM_TRAINING_IDLE:-}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS2_WS="${ROOT}/../../.."

# The default dry run executes Python tests too, so load the same Humble and
# workspace overlay before branching on DRY_RUN.  Otherwise a newly added
# shared package can be discoverable by ros2 yet unavailable to pytest.
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${ROS2_WS}/install/setup.bash" ]]; then
  echo "[smoke_planning] missing workspace overlay: ${ROS2_WS}/install/setup.bash" >&2
  exit 2
fi
source "${ROS2_WS}/install/setup.bash"
set -u

echo "[smoke_planning] DRY_RUN=$DRY_RUN"
python3 -m pytest "$ROOT/tests/test_costmap.py" "$ROOT/tests/test_hybrid_astar.py" -q

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[smoke_planning] dry-run only. Idle window: DRY_RUN=0 I_CONFIRM_TRAINING_IDLE=yes"
  exit 0
fi

if [[ "$I_CONFIRM_TRAINING_IDLE" != "yes" ]]; then
  echo "[smoke_planning] need I_CONFIRM_TRAINING_IDLE=yes" >&2
  exit 2
fi

if pgrep -a gzserver >/dev/null 2>&1; then
  echo "[smoke_planning] gzserver running — abort" >&2
  exit 3
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-41}"
ros2 launch scout_long_nav stage_c_planning.launch.py
