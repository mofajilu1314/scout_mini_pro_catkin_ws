#!/usr/bin/env bash
# roslaunch launch-prefix：在真正启动 gzserver 前清掉占用 11345 的残留进程。
# 现象：上次 Ctrl-C / 异常退出后 gzserver 残留 → 新 gzserver bind 失败 exit 255
#       → /gazebo/spawn_urdf_model 永不出现 → spawn_model 弹故障对话框、小车不出现。
# 注意：不可 killall gzclient —— gzclient 与本 prefix 并行启动，会被误杀 (exit -9)。
set -euo pipefail

port_busy() {
  ss -lnt 2>/dev/null | grep -qE ':11345[[:space:]]' \
    || netstat -lnt 2>/dev/null | grep -qE ':11345[[:space:]]'
}

kill_port_11345() {
  # 只杀占用 11345 的监听进程（旧 gzserver），不动并行启动的新 gzclient
  local pids=""
  if command -v fuser >/dev/null 2>&1; then
    pids=$(fuser 11345/tcp 2>/dev/null || true)
  fi
  if [[ -z "${pids// }" ]]; then
    pids=$(ss -lntp 2>/dev/null | awk '/:11345[[:space:]]/ {
      while (match($0, /pid=[0-9]+/)) {
        print substr($0, RSTART+4, RLENGTH-4)
        $0 = substr($0, RSTART+RLENGTH)
      }
    }' | sort -u | tr '\n' ' ')
  fi
  if [[ -n "${pids// }" ]]; then
    # shellcheck disable=SC2086
    kill -9 $pids 2>/dev/null || true
  else
    # 端口忙但拿不到 PID 时，只杀 gzserver，绝不 killall gzclient
    killall -9 gzserver 2>/dev/null || true
  fi
}

if port_busy; then
  echo "[gzserver_clean] port 11345 busy — killing listener PID(s) only (not gzclient)"
  kill_port_11345
  for _ in $(seq 1 50); do
    port_busy || break
    sleep 0.1
  done
  if port_busy; then
    echo "[gzserver_clean] WARNING: 11345 still busy after kill; gzserver may fail to bind"
  else
    echo "[gzserver_clean] 11345 free, starting gzserver"
  fi
fi

exec "$@"
