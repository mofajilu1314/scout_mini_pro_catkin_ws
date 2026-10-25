#!/usr/bin/env bash
# Gazebo launch prefix: verify that the master port is available before this
# invocation starts gzserver. It deliberately never terminates a listener it
# did not start itself: an occupied port may belong to another terminal or
# another robot simulation.
set -euo pipefail

port_busy() {
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

  # No supported inspector is available. Let gzserver perform its own bind
  # check rather than making an unsafe guess about another process.
  return 1
}

report_listener() {
  echo "[gzserver_preflight] port 11345 is already in use; no process was stopped." >&2
  echo "[gzserver_preflight] Stop the owning Gazebo/ROS launch in its original terminal, then retry." >&2

  if command -v ss >/dev/null 2>&1; then
    ss -lntp 2>/dev/null | awk 'NR == 1 || $4 ~ /:11345$/' >&2 || true
  elif command -v fuser >/dev/null 2>&1; then
    fuser -v 11345/tcp >&2 || true
  fi
}

if port_busy; then
  report_listener
  exit 125
fi

exec "$@"
