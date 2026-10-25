#!/usr/bin/env bash
# ScoutNav 启动脚本。工作区根优先由 SCOUT_WS_ROOT 指定。
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="${SCOUT_WS_ROOT:-$(cd "${script_dir}/../.." && pwd)}"

if [[ ! -d "${workspace_root}/ros2_ws/src" || ! -d "${workspace_root}/3D" ]]; then
  echo "Scout workspace not found. Set SCOUT_WS_ROOT to the workspace root." >&2
  exit 2
fi

export SCOUT_WS_ROOT="$(cd "${workspace_root}" && pwd)"
# Optional local Qt/XCB compatibility libraries; no workspace-specific home path.
export LD_LIBRARY_PATH="${HOME}/.local/lib/xcb/usr/lib/x86_64-linux-gnu:${HOME}/.local/lib:${LD_LIBRARY_PATH:-}"

cd "${script_dir}"
exec python3 -m scout_nav_app "$@"
