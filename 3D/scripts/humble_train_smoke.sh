#!/usr/bin/env bash
# Single entry: Humble TD3 training smoke (one launch, flock-guarded).
set -eo pipefail

LOCK_FILE="/tmp/scout_offroad_train.lock"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
# Prefer an explicit workspace root; the script location is only a fallback.
WS_ROOT="${SCOUT_WS_ROOT:-${CATKIN_WS:-${WS_ROOT:-${DEFAULT_WS_ROOT}}}}"
ROS2_WS="${WS_ROOT}/ros2_ws"
RUNS_DIR="${WS_ROOT}/3D/runs"
DEBUG_DIR="${WS_ROOT}/3D/debug_logs"
DO_BUILD=1
SELF_TEST_SESSION=0

usage() {
  echo "用法: $0 [--no-build] [--self-test-session]"
  echo "  唯一训练/冒烟入口；勿并行启动第二个 ros2 launch train_td3。"
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build) DO_BUILD=0; shift ;;
    --self-test-session) SELF_TEST_SESSION=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "未知参数: $1"; usage 1 ;;
  esac
done

exec 200>"${LOCK_FILE}"
if ! flock -n 200; then
  echo "已有训练/冒烟在跑，请先停止"
  exit 1
fi

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

report_port_11345_owner() {
  echo "FAIL: 端口 11345 正由其他 Gazebo/ROS 会话使用；本脚本不会终止未知进程。" >&2
  echo "请在原终端停止占用会话后重试。监听信息：" >&2
  if command -v ss >/dev/null 2>&1; then
    ss -lntp 2>/dev/null | awk 'NR == 1 || $4 ~ /:11345$/' >&2 || true
  elif command -v fuser >/dev/null 2>&1; then
    fuser -v 11345/tcp >&2 || true
  fi
}

require_port_11345_free() {
  if port_11345_busy; then
    report_port_11345_owner
    return 1
  fi
}

wait_for_port_11345_free() {
  local label="$1"
  for _ in $(seq 1 15); do
    if ! port_11345_busy; then
      return 0
    fi
    sleep 1
  done
  echo "FAIL: ${label} 后端口 11345 仍被占用；不会清理未由本脚本直接管理的进程。" >&2
  report_port_11345_owner
  return 1
}

# Each launch gets a private session/process group.  The record includes the
# session leader's start time and an argv marker, so a reused PID is never
# mistaken for a launch created by this script.
LAUNCH_PID=""
LAUNCH_PGID=""
LAUNCH_SID=""
LAUNCH_START_TICKS=""
LAUNCH_TOKEN=""
LAUNCH_LABEL=""
LAUNCH_LOG=""
LAUNCH_ACTIVE=0
LAUNCH_TERM_SENT=0

proc_start_ticks() {
  local pid="${1:-}"
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/stat" ]] || return 1
  awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null
}

launch_leader_matches() {
  [[ "${LAUNCH_ACTIVE}" -eq 1 ]] || return 1
  [[ "${LAUNCH_PID}" =~ ^[0-9]+$ && "${LAUNCH_PGID}" =~ ^[0-9]+$ && "${LAUNCH_SID}" =~ ^[0-9]+$ ]] || return 1

  local current_pgid current_sid current_start current_args
  current_pgid="$(ps -o pgid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d '[:space:]')"
  current_sid="$(ps -o sid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d '[:space:]')"
  current_start="$(proc_start_ticks "${LAUNCH_PID}" 2>/dev/null || true)"
  current_args="$(ps -o args= -p "${LAUNCH_PID}" 2>/dev/null || true)"

  [[ "${current_pgid}" == "${LAUNCH_PGID}" ]] || return 1
  [[ "${current_sid}" == "${LAUNCH_SID}" ]] || return 1
  [[ "${current_start}" == "${LAUNCH_START_TICKS}" ]] || return 1
  [[ "${LAUNCH_PID}" == "${LAUNCH_PGID}" && "${LAUNCH_PID}" == "${LAUNCH_SID}" ]] || return 1
  [[ "${current_args}" == *"${LAUNCH_TOKEN}"* ]]
}

launch_group_has_members() {
  [[ "${LAUNCH_PGID}" =~ ^[0-9]+$ && "${LAUNCH_SID}" =~ ^[0-9]+$ ]] || return 1
  ps -eo pgid=,sid= 2>/dev/null | awk -v pgid="${LAUNCH_PGID}" -v sid="${LAUNCH_SID}" '
    $1 == pgid && $2 == sid { found = 1 }
    END { exit !found }
  '
}

post_term_group_matches() {
  [[ "${LAUNCH_TERM_SENT}" -eq 1 ]] || return 1
  # If the old leader PID exists again, it must still be the exact recorded
  # wrapper; otherwise a PID reuse makes escalation unsafe.
  if [[ -e "/proc/${LAUNCH_PID}" ]] && ! launch_leader_matches; then
    return 1
  fi
  launch_group_has_members
}

clear_launch_record() {
  LAUNCH_PID=""
  LAUNCH_PGID=""
  LAUNCH_SID=""
  LAUNCH_START_TICKS=""
  LAUNCH_TOKEN=""
  LAUNCH_LABEL=""
  LAUNCH_LOG=""
  LAUNCH_ACTIVE=0
  LAUNCH_TERM_SENT=0
}

start_owned_launch() {
  local label="$1"
  local log_file="$2"
  shift 2

  if ! command -v setsid >/dev/null 2>&1; then
    echo "FAIL: 缺少 setsid，无法为 ${label} 建立可验证的独立会话。" >&2
    return 1
  fi

  mkdir -p "$(dirname -- "${log_file}")"
  : > "${log_file}"
  clear_launch_record
  LAUNCH_LABEL="${label}"
  LAUNCH_LOG="${log_file}"
  LAUNCH_TOKEN="scout_owned_${label}_$$_${RANDOM}_$(date +%s%N)"

  # Non-interactive Bash normally has job control disabled.  State it here so
  # GNU setsid can exec the wrapper directly rather than creating an unknown
  # intermediate process.  Do not use setsid --fork.
  set +m 2>/dev/null || true
  # shellcheck disable=SC2016 # Literal body is executed by the child bash.
  setsid bash -c '
    "$@" &
    child=$!
    wait "${child}"
  ' "${LAUNCH_TOKEN}" "$@" > "${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID=$!

  local _
  for _ in $(seq 1 40); do
    LAUNCH_PGID="$(ps -o pgid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d '[:space:]')"
    LAUNCH_SID="$(ps -o sid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d '[:space:]')"
    LAUNCH_START_TICKS="$(proc_start_ticks "${LAUNCH_PID}" 2>/dev/null || true)"
    LAUNCH_ACTIVE=1
    if launch_leader_matches; then
      echo "==> ${label}: 已记录自建会话 pid=${LAUNCH_PID} pgid=${LAUNCH_PGID} sid=${LAUNCH_SID}"
      return 0
    fi
    LAUNCH_ACTIVE=0
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.05
  done

  echo "FAIL: 无法验证 ${label} 的独立会话；不会向任何进程组发送信号。" >&2
  # The exact child is still waitable by this shell; do not guess at a group.
  wait "${LAUNCH_PID}" 2>/dev/null || true
  clear_launch_record
  return 1
}

wait_for_launch_group_exit() {
  local seconds="$1"
  local _
  for _ in $(seq 1 "${seconds}"); do
    if ! launch_group_has_members; then
      return 0
    fi
    sleep 1
  done
  ! launch_group_has_members
}

stop_owned_launch() {
  local reason="${1:-脚本结束}"
  [[ "${LAUNCH_ACTIVE}" -eq 1 ]] || return 0

  # Before the first group signal, require the leader's PID, start time,
  # session/PGID, and argv token all to match our record.
  if ! launch_leader_matches; then
    echo "FAIL: ${LAUNCH_LABEL} 的会话身份已无法验证；拒绝终止未知进程组。" >&2
    return 1
  fi

  echo "==> 停止本脚本创建的 ${LAUNCH_LABEL}（${reason}，pgid=${LAUNCH_PGID}）"
  if ! kill -TERM -- "-${LAUNCH_PGID}" 2>/dev/null; then
    echo "FAIL: 无法向已验证的 ${LAUNCH_LABEL} 进程组发送 TERM。" >&2
    return 1
  fi
  LAUNCH_TERM_SENT=1

  if wait_for_launch_group_exit 5; then
    wait "${LAUNCH_PID}" 2>/dev/null || true
    clear_launch_record
    return 0
  fi

  # The session leader may already have exited after TERM.  A surviving member
  # with the recorded PGID+SID can only belong to this private session during
  # this short, already-signalled grace period; re-check before escalation.
  if post_term_group_matches; then
    echo "WARN: ${LAUNCH_LABEL} 未在宽限期退出；仅向已记录会话组发送 KILL。" >&2
    if ! kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null; then
      echo "FAIL: 无法终止已验证的 ${LAUNCH_LABEL} 进程组。" >&2
      return 1
    fi
    if wait_for_launch_group_exit 3; then
      wait "${LAUNCH_PID}" 2>/dev/null || true
      clear_launch_record
      return 0
    fi
  fi

  echo "FAIL: ${LAUNCH_LABEL} 会话仍存在；不会扩展清理范围。" >&2
  return 1
}

wait_for_owned_launch() {
  local timeout_seconds="$1"
  local elapsed=0
  local launch_exit=0
  local recorded_log="${LAUNCH_LOG}"

  while launch_group_has_members; do
    if (( elapsed >= timeout_seconds )); then
      echo "WARN: ${LAUNCH_LABEL} 超过 ${timeout_seconds}s；只停止本脚本记录的会话。" >&2
      stop_owned_launch "达到 ${timeout_seconds}s 超时" || true
      launch_exit=124
      break
    fi
    if ! launch_leader_matches; then
      echo "FAIL: ${LAUNCH_LABEL} leader 已退出但会话组仍存在；拒绝猜测其归属。" >&2
      launch_exit=1
      break
    fi
    sleep 1
    ((elapsed += 1))
  done

  if ! launch_group_has_members; then
    if [[ "${LAUNCH_PID}" =~ ^[0-9]+$ ]]; then
      local wait_exit=0
      if wait "${LAUNCH_PID}" 2>/dev/null; then
        wait_exit=0
      else
        wait_exit=$?
      fi
      [[ "${launch_exit}" -ne 124 ]] && launch_exit="${wait_exit}"
    fi
    clear_launch_record
  fi

  # Keep the original log path locally because clear_launch_record deliberately
  # erases mutable process identity state before returning to the caller.
  [[ -s "${recorded_log}" ]] && cat "${recorded_log}"
  return "${launch_exit}"
}

run_owned_launch() {
  local label="$1"
  local timeout_seconds="$2"
  local log_file="$3"
  shift 3
  start_owned_launch "${label}" "${log_file}" "$@" || return 1
  wait_for_owned_launch "${timeout_seconds}"
}

run_session_self_test() {
  local self_test_stamp
  self_test_stamp="$(date +%Y%m%d_%H%M%S)"
  local natural_log="${DEBUG_DIR}/session_selftest_natural_${self_test_stamp}.log"
  local self_test_log="${DEBUG_DIR}/session_selftest_${self_test_stamp}.log"
  echo "==> 运行受控会话自检（不会启动 ROS/Gazebo）"
  run_owned_launch "session_selftest_natural" 5 "${natural_log}" \
    bash -c 'sleep 0.1; exit 0' || return 1
  start_owned_launch "session_selftest" "${self_test_log}" \
    bash -c 'trap "" TERM; sleep 30 & wait' || return 1
  [[ "${LAUNCH_PID}" == "${LAUNCH_PGID}" && "${LAUNCH_PID}" == "${LAUNCH_SID}" ]] || return 1
  stop_owned_launch "受控自检" || return 1
  if launch_group_has_members; then
    echo "FAIL: 受控会话自检后仍有成员。" >&2
    return 1
  fi
  echo "PASS: 受控会话自检只终止了本脚本创建的独立进程组。"
}

on_script_exit() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ "${LAUNCH_ACTIVE}" -eq 1 ]]; then
    stop_owned_launch "脚本退出（status=${status}）" || true
  fi
  exit "${status}"
}

trap on_script_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

# ROS launch must not write into a user-global ~/.ros directory.  Keep this
# smoke run's runtime logs with the other 3D diagnostics instead.
mkdir -p "${DEBUG_DIR}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${DEBUG_DIR}/ros}"
mkdir -p "${ROS_LOG_DIR}"
export GAZEBO_LOG_PATH="${GAZEBO_LOG_PATH:-${DEBUG_DIR}/gazebo}"
mkdir -p "${GAZEBO_LOG_PATH}"

if [[ "${SELF_TEST_SESSION}" -eq 1 ]]; then
  run_session_self_test
  exit $?
fi

require_port_11345_free

# shellcheck source=/dev/null
set +u
source /opt/ros/humble/setup.bash
set -e
cd "${ROS2_WS}"

need_build=0
if [[ ! -f install/setup.bash ]]; then
  need_build=1
elif [[ ! -f install/scout_offroad_3d/share/scout_offroad_3d/package.xml ]]; then
  need_build=1
fi

if [[ "${DO_BUILD}" -eq 1 && "${need_build}" -eq 1 ]]; then
  echo "==> colcon build --symlink-install ..."
  colcon build --symlink-install
elif [[ "${DO_BUILD}" -eq 1 && "${need_build}" -eq 0 ]]; then
  echo "==> install 已存在，跳过 colcon（可用 --no-build 显式跳过）"
fi

# shellcheck source=/dev/null
set +u
source install/setup.bash
set -e

TS="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${RUNS_DIR}/smoke_${TS}"
mkdir -p "${OUTPUT_DIR}"

WORLD="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world"

echo "==> 启动单次冒烟: max_timesteps=30, world=bumpy_offroad"
echo "    output_dir=${OUTPUT_DIR}"
require_port_11345_free

LAUNCH_EXIT=0
set +e
# A private session lets the script clean up only its own launch descendants
# if the time limit is reached; it never signals by process name or port.
run_owned_launch "train_td3_smoke" 480 "${DEBUG_DIR}/train_smoke_${TS}.launch.log" \
  ros2 launch scout_offroad_3d train_td3.launch.py \
  gui:=false \
  world:="${WORLD}" \
  max_timesteps:=30 \
  start_timesteps:=10 \
  viz_eval_enable:=false \
  viz_eval_pick_at_start:=false \
  output_dir:="${OUTPUT_DIR}"
LAUNCH_EXIT=$?
set -e

echo "==> launch 退出码: ${LAUNCH_EXIT}"
echo "==> 验收产物..."

# The recorded private session has already been waited for or safely stopped.
# A remaining port listener is therefore reported, never killed by name/port.
wait_for_port_11345_free "训练启动结束" || exit 1

FAIL=0
for f in train.log metrics.csv config.yaml; do
  if [[ -f "${OUTPUT_DIR}/${f}" ]]; then
    echo "  OK  ${f}"
  else
    echo "  缺少 ${f}"
    FAIL=1
  fi
done

if grep -q 'Training finished' "${OUTPUT_DIR}/train.log" 2>/dev/null; then
  echo "  OK  Training finished (train.log)"
elif [[ -f "${OUTPUT_DIR}/td3_actor" ]]; then
  echo "  OK  Training finished (td3_actor present; message may be ROS-only)"
else
  echo "  缺少 Training finished / td3_actor"
  FAIL=1
fi

# Record 9 checks
if grep -Eq 'state_dim[=:] ?150|state_dim=150' "${OUTPUT_DIR}/train.log" "${OUTPUT_DIR}/config.yaml" 2>/dev/null; then
  echo "  OK  state_dim=150"
else
  # config.yaml may nest under train keys
  if python3 - <<PY2
import yaml,sys
p="${OUTPUT_DIR}/config.yaml"
d=yaml.safe_load(open(p))
def find(obj,key):
  if isinstance(obj,dict):
    if key in obj: return obj[key]
    for v in obj.values():
      r=find(v,key)
      if r is not None: return r
  return None
sd=find(d,'state_dim'); hd=find(d,'hidden_dim'); imu=find(d,'use_imu_in_state')
print('found',sd,hd,imu)
sys.exit(0 if sd==150 and hd==1024 else 1)
PY2
  then
    echo "  OK  config state_dim/hidden"
  else
    echo "  FAIL state_dim/hidden not Record9"
    FAIL=1
  fi
fi

if ! grep -q 'climb_scale' "${OUTPUT_DIR}/train.log" 2>/dev/null; then
  echo "  WARN no climb_scale in log (ok if anneal not yet logged)"
fi

# 0 = clean exit; 124 = timeout; 137 = SIGKILL during Shutdown — OK if Training finished
if [[ "${LAUNCH_EXIT}" -ne 0 ]]; then
  if grep -q 'Training finished' "${OUTPUT_DIR}/train.log" 2>/dev/null; then
    echo "  launch exit ${LAUNCH_EXIT} but Training finished — OK"
  else
    FAIL=1
  fi
fi

if [[ "${FAIL}" -eq 0 ]]; then
  echo ""
  echo "========================================"
  echo "  SMOKE PASS"
  echo "  产物: ${OUTPUT_DIR}"
  echo "========================================"
  exit 0
else
  echo ""
  echo "========================================"
  echo "  SMOKE FAIL"
  echo "  目录: ${OUTPUT_DIR}"
  echo "========================================"
  exit 1
fi
