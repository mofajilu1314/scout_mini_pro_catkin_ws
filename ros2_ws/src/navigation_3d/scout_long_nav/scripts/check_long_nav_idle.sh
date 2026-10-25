#!/usr/bin/env bash
# Fail if TD3 training (or unexpected gzserver) would collide with long-nav.
# Override: LONG_NAV_FORCE=1 or LONG_NAV_ALLOW_GZ=1
set -euo pipefail

echo "============================================================"
echo "[scout_long_nav] TD3 TRAINING ISOLATION CHECK"
echo "  - Do NOT run train_td3 alongside long-nav"
echo "  - Arena world only (not worlds/bumpy_offroad_*.world train pool)"
echo "  - infer weights read-only (run_20260905_222831)"
echo "============================================================"

if pgrep -af 'train_td3' 2>/dev/null | grep -v pgrep >/dev/null; then
  echo "BLOCKED: train_td3 process found:"
  pgrep -af 'train_td3' | grep -v pgrep || true
  echo "Stop training first, then re-run long-nav."
  exit 1
fi

FORCE="${LONG_NAV_FORCE:-}"
ALLOW_GZ="${LONG_NAV_ALLOW_GZ:-}"
if pgrep -af 'gzserver' 2>/dev/null | grep -v pgrep >/dev/null; then
  if [[ "${FORCE}" == "1" || "${FORCE}" == "yes" || "${ALLOW_GZ}" == "1" || "${ALLOW_GZ}" == "yes" ]]; then
    echo "WARN: gzserver present but override set — continuing"
    pgrep -af 'gzserver' | grep -v pgrep || true
  else
    echo "BLOCKED: gzserver already running. No process was terminated."
    echo "Stop the owning Gazebo session in its original terminal, then re-run this check."
    echo "If you intentionally share that Gazebo session, explicitly override with: LONG_NAV_ALLOW_GZ=1 $0"
    pgrep -af 'gzserver' | grep -v pgrep || true
    exit 1
  fi
else
  echo "OK: no train_td3; no gzserver"
fi

exit 0
