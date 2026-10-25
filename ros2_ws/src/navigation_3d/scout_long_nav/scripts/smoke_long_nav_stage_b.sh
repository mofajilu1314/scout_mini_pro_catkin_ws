#!/usr/bin/env bash
# Smoke check for scout_long_nav. Default DRY_RUN=1 — never launches Gazebo.
set -euo pipefail

DRY_RUN="${DRY_RUN:-1}"
I_CONFIRM_TRAINING_IDLE="${I_CONFIRM_TRAINING_IDLE:-}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "[smoke] scout_long_nav root=$ROOT DRY_RUN=$DRY_RUN"

need=(
  "$ROOT/package.xml"
  "$ROOT/launch/stage_b_fake_path.launch.py"
  "$ROOT/launch/stage_c_planning.launch.py"
  "$ROOT/launch/long_nav_full_sim.launch.py"
  "$ROOT/config/long_nav.yaml"
  "$ROOT/scout_long_nav/lookahead.py"
  "$ROOT/scout_long_nav/costmap_geom.py"
  "$ROOT/scout_long_nav/hybrid_astar.py"
)
for f in "${need[@]}"; do
  if [[ ! -e "$f" ]]; then
    echo "[smoke] MISSING $f" >&2
    exit 1
  fi
  echo "[smoke] ok $f"
done

echo "[smoke] running offline pytest..."
python3 -m pytest "$ROOT/tests" -q

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[smoke] DRY_RUN=1 → not launching ROS. Set DRY_RUN=0 I_CONFIRM_TRAINING_IDLE=yes only when training Gazebo is stopped."
  exit 0
fi

if [[ "$I_CONFIRM_TRAINING_IDLE" != "yes" ]]; then
  echo "[smoke] Refusing live launch: set I_CONFIRM_TRAINING_IDLE=yes" >&2
  exit 2
fi

if pgrep -a gzserver >/dev/null 2>&1; then
  echo "[smoke] gzserver is running — abort live launch to protect training." >&2
  exit 3
fi

echo "[smoke] live launch stage_b on ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-41} (Ctrl+C to stop)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-41}"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source "${ROOT}/../../../install/setup.bash" 2>/dev/null || true
ros2 launch scout_long_nav stage_b_fake_path.launch.py
