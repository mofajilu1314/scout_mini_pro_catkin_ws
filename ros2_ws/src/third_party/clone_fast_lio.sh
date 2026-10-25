#!/usr/bin/env bash
# Clone FAST_LIO when network allows. Do NOT build/run during TD3 Gazebo training.
set -euo pipefail
DEST="$(cd "$(dirname "$0")" && pwd)/FAST_LIO"
if [[ -d "$DEST/.git" && -f "$DEST/package.xml" ]]; then
  echo "FAST_LIO already present at $DEST"
  exit 0
fi
if [[ -e "$DEST" ]]; then
  echo "Refusing to replace existing FAST_LIO path: $DEST" >&2
  echo "Inspect or move the partial checkout manually, then retry." >&2
  exit 2
fi
git clone --depth 1 https://github.com/hku-mars/FAST_LIO.git "$DEST"
echo "Cloned. Idle-window build only: colcon build --packages-select fast_lio"
