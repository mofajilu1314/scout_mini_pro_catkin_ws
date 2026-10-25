#!/usr/bin/env bash
# 3D Humble command verification — run after README/doc updates.
# Logs and generated summaries stay under 3D/debug_logs/ (NOT 3D/runs/).
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WS="${SCOUT_WS_ROOT:-${CATKIN_WS:-${WS_ROOT:-${DEFAULT_WS_ROOT}}}}"
ROS2_WS="${WS}/ros2_ws"
LOCK_FILE="/tmp/scout_offroad_verify_all.lock"
DEBUG_DIR="${WS}/3D/debug_logs"
SELF_TEST_SESSION=0

if [[ $# -gt 0 ]]; then
  case "$1" in
    --self-test-session) SELF_TEST_SESSION=1; shift ;;
    -h|--help)
      echo "用法: $0 [--self-test-session]"
      echo "  --self-test-session 仅验证本脚本的独立进程组清理，不启动 ROS/Gazebo。"
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 2
      ;;
  esac
fi
if [[ $# -gt 0 ]]; then
  echo "参数过多。用法: $0 [--self-test-session]" >&2
  exit 2
fi

mkdir -p "${DEBUG_DIR}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${DEBUG_DIR}/ros}"
mkdir -p "${ROS_LOG_DIR}"
export GAZEBO_LOG_PATH="${GAZEBO_LOG_PATH:-${DEBUG_DIR}/gazebo}"
mkdir -p "${GAZEBO_LOG_PATH}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="${DEBUG_DIR}/verify_humble_${STAMP}.log"

exec 201>"${LOCK_FILE}"
if ! flock -n 201; then
  echo "已有 verify_all_humble 在跑，退出" >&2
  exit 1
fi

exec > >(tee -a "${LOG}") 2>&1

pass() { echo "[PASS] $*"; }
fail() { echo "[FAIL] $*"; FAILED=1; }
FAILED=0

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
  echo "[FAIL] 端口 11345 正由其他 Gazebo/ROS 会话使用；验证脚本不会终止未知进程。" >&2
  echo "请在原终端停止占用会话后重试。监听信息：" >&2
  if command -v ss >/dev/null 2>&1; then
    ss -lntp 2>/dev/null | awk 'NR == 1 || $4 ~ /:11345$/' >&2 || true
  elif command -v fuser >/dev/null 2>&1; then
    fuser -v 11345/tcp >&2 || true
  fi
}

wait_port_free() {
  local label="${1:-下一项验证}"
  for _ in $(seq 1 15); do
    if ! port_11345_busy; then
      return 0
    fi
    sleep 1
  done
  echo "[FAIL] ${label} 前端口 11345 仍被占用；不会清理未管理的进程。" >&2
  report_port_11345_owner
  return 1
}

# Each ros2 launch below is put in a private session/process group.  We retain
# the leader PID, PGID, SID, Linux /proc start time, and a unique argv token.
# This prevents a reused PID from ever authorising a signal to an unrelated
# process group.
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
  # A completed wrapper can briefly remain as a zombie until this shell reaps
  # it.  Zombies neither hold Gazebo resources nor need a signal, so only live
  # members keep the session active.
  ps -eo pgid=,sid=,stat= 2>/dev/null | awk -v pgid="${LAUNCH_PGID}" -v sid="${LAUNCH_SID}" '
    $1 == pgid && $2 == sid && $3 !~ /^Z/ { found = 1 }
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
    echo "[FAIL] 缺少 setsid，无法为 ${label} 建立可验证的独立会话。" >&2
    return 1
  fi

  mkdir -p "$(dirname -- "${log_file}")"
  : > "${log_file}"
  clear_launch_record
  LAUNCH_LABEL="${label}"
  LAUNCH_LOG="${log_file}"
  LAUNCH_TOKEN="scout_owned_${label}_$$_${RANDOM}_$(date +%s%N)"

  # Keep job control disabled so `setsid` (without --fork) is the session
  # leader we record.  The bash wrapper preserves the unique argv token while
  # it waits for the ros2-launch child.
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

  echo "[FAIL] 无法验证 ${label} 的独立会话；不会向任何进程组发送信号。" >&2
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

  # The first group signal is allowed only when every recorded identity field
  # still describes the private session leader created by this script.
  if ! launch_leader_matches; then
    echo "[FAIL] ${LAUNCH_LABEL} 的会话身份已无法验证；拒绝终止未知进程组。" >&2
    return 1
  fi

  echo "==> 停止本脚本创建的 ${LAUNCH_LABEL}（${reason}，pgid=${LAUNCH_PGID}）"
  if ! kill -TERM -- "-${LAUNCH_PGID}" 2>/dev/null; then
    echo "[FAIL] 无法向已验证的 ${LAUNCH_LABEL} 进程组发送 TERM。" >&2
    return 1
  fi
  LAUNCH_TERM_SENT=1

  if wait_for_launch_group_exit 5; then
    wait "${LAUNCH_PID}" 2>/dev/null || true
    clear_launch_record
    return 0
  fi

  # After the verified TERM, a member still carrying both recorded IDs is a
  # remnant of this private session.  Re-check that membership immediately
  # before the narrow KILL escalation; never inspect names or ports.
  if post_term_group_matches; then
    echo "[WARN] ${LAUNCH_LABEL} 未在宽限期退出；仅向已记录会话组发送 KILL。" >&2
    if ! kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null; then
      echo "[FAIL] 无法终止已验证的 ${LAUNCH_LABEL} 进程组。" >&2
      return 1
    fi
    if wait_for_launch_group_exit 3; then
      wait "${LAUNCH_PID}" 2>/dev/null || true
      clear_launch_record
      return 0
    fi
  fi

  echo "[FAIL] ${LAUNCH_LABEL} 会话仍存在；不会扩展清理范围。" >&2
  return 1
}

wait_for_owned_launch() {
  local timeout_seconds="$1"
  local elapsed=0
  local launch_exit=0
  local recorded_log="${LAUNCH_LOG}"

  while launch_group_has_members; do
    if (( elapsed >= timeout_seconds )); then
      echo "[WARN] ${LAUNCH_LABEL} 超过 ${timeout_seconds}s；只停止本脚本记录的会话。" >&2
      stop_owned_launch "达到 ${timeout_seconds}s 超时" || true
      launch_exit=124
      break
    fi
    if ! launch_leader_matches; then
      # `ros2 launch` may return after its required node finishes while its
      # own children are still processing the launch shutdown.  Do not signal
      # the group after the leader identity disappears; allow that known
      # private session a short, read-only grace period to drain naturally.
      if wait_for_launch_group_exit 15; then
        break
      fi
      echo "[FAIL] ${LAUNCH_LABEL} leader 已退出且会话组在被动收尾后仍存在；拒绝猜测其归属。" >&2
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
  # The direct child exits first while its own short-lived child remains in
  # the same private session.  This reproduces ROS launch's normal shutdown
  # tail and must drain without a false unknown-session failure or a signal.
  run_owned_launch "session_selftest_natural_tail" 5 "${natural_log}.tail" \
    bash -c 'sleep 0.3 & sleep 0.15; exit 0' || return 1
  start_owned_launch "session_selftest" "${self_test_log}" \
    bash -c 'trap "" TERM; sleep 30 & wait' || return 1
  [[ "${LAUNCH_PID}" == "${LAUNCH_PGID}" && "${LAUNCH_PID}" == "${LAUNCH_SID}" ]] || return 1
  stop_owned_launch "受控自检" || return 1
  if launch_group_has_members; then
    echo "[FAIL] 受控会话自检后仍有成员。" >&2
    return 1
  fi
  echo "[PASS] 受控会话自检只终止了本脚本创建的独立进程组。"
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

if [[ "${SELF_TEST_SESSION}" -eq 1 ]]; then
  run_session_self_test
  exit $?
fi

write_validation_summary() {
  local status="$1"
  local validation_dir="${DEBUG_DIR}/validation"
  local report="${validation_dir}/verify_humble_${STAMP}.md"
  mkdir -p "${validation_dir}"
  {
    echo "# Humble 3D 验收摘要"
    echo ""
    echo "- 日期：$(date '+%Y-%m-%d %H:%M')"
    echo "- 结果：${status}"
    echo "- 完整日志：\`3D/debug_logs/verify_humble_${STAMP}.log\`"
    echo "- 说明：训练产物只可位于 \`3D/runs/smoke_*\` / \`run_*\`；验证日志不写入训练目录。"
  } > "${report}"
  echo "summary → ${report}"
}

echo "========== VERIFY 3D HUMBLE $(date) =========="
echo "log → ${LOG}"

# 1) source + colcon build
source /opt/ros/humble/setup.bash
cd "${ROS2_WS}"
echo "==> colcon build shared + 3D packages"
colcon build --symlink-install --packages-select \
  scout_workspace_common scout_description scout_mini_pro_description \
  scout_offroad_3d scout_long_nav
source install/setup.bash

ros2 pkg prefix scout_offroad_3d >/dev/null && pass "ros2 pkg prefix scout_offroad_3d" || fail "ros2 pkg prefix"
ros2 pkg prefix scout_long_nav >/dev/null && pass "ros2 pkg prefix scout_long_nav" || fail "ros2 pkg prefix scout_long_nav"

WORLD=$(ros2 run scout_offroad_3d pick_random_world random)
[[ -f "${WORLD}" ]] && pass "pick_random_world: ${WORLD}" || fail "pick_random_world"

# 2) 3D smoke
wait_port_free "3D 冒烟" || exit 1
echo "==> 3D humble_train_smoke.sh"
if bash "${WS}/3D/scripts/humble_train_smoke.sh" --no-build; then
  pass "3D humble_train_smoke.sh"
else
  fail "3D humble_train_smoke.sh"
fi

SMOKE_DIR=$(ls -td "${WS}/3D/runs"/smoke_* 2>/dev/null | head -1)
if [[ -n "${SMOKE_DIR}" && -f "${SMOKE_DIR}/metrics.csv" ]]; then
  pass "3D smoke artifacts in ${SMOKE_DIR}"
else
  fail "3D smoke artifacts missing"
fi

# 3) plot (use old run with data if smoke metrics empty).  Archived metrics
# stay read-only: validation PNGs go below 3D/debug_logs instead.
PLOT_DIR="${SMOKE_DIR}"
PLOT_OUTPUT="${PLOT_DIR}"
if ! grep -q '^1,' "${SMOKE_DIR}/metrics.csv" 2>/dev/null; then
  PLOT_DIR=$(find "${WS}/archive/experiments/td3/runs" "${WS}/3D/runs" \
    -mindepth 1 -maxdepth 1 -type d -name 'run_*' 2>/dev/null | sort -r | while read d; do
      grep -q '^1,' "${d}/metrics.csv" 2>/dev/null && echo "$d" && break
    done)
  PLOT_OUTPUT="${DEBUG_DIR}/plot_validation_${STAMP}"
fi
if [[ -n "${PLOT_DIR}" ]]; then
  if ros2 run scout_offroad_3d plot_td3_metrics "${PLOT_DIR}" --output-dir "${PLOT_OUTPUT}"; then
    PNG_N=$(ls "${PLOT_OUTPUT}"/*.png 2>/dev/null | wc -l)
    [[ "${PNG_N}" -ge 10 ]] && pass "plot_td3_metrics (${PNG_N} PNG)" || fail "plot_td3_metrics PNG count=${PNG_N}"
  else
    fail "plot_td3_metrics"
  fi
else
  fail "no run dir for plot test"
fi

# 4) world switch short test (lock world + short horizon for the session limit)
wait_port_free "world-switch 短测" || exit 1
echo "==> 3D world switch short test"
SW_OUT="${WS}/3D/runs/verify_switch_$(date +%Y%m%d_%H%M%S)"
SW_WORLD="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world"
# A fresh training output must not contain a pre-created launch log.  Keep the
# verifier log with other diagnostics and let train_td3 create SW_OUT itself.
SW_LOG="${DEBUG_DIR}/verify_world_switch_${STAMP}.log"
set +e
run_owned_launch "world_switch_train" 420 "${SW_LOG}" \
  ros2 launch scout_offroad_3d train_td3.launch.py gui:=false \
  output_dir:="${SW_OUT}" max_timesteps:=55 start_timesteps:=5 \
  world_switch_interval:=25 world:="${SW_WORLD}" \
  viz_eval_enable:=false viz_eval_pick_at_start:=false
SW_EC=$?
set -e
if grep -q 'switch complete' "${SW_LOG}" 2>/dev/null \
  || grep -q 'switch complete' "${SW_OUT}/train.log" 2>/dev/null; then
  pass "world switch (switch complete in log)"
else
  fail "world switch (no switch complete)"
fi
if [[ "${SW_EC}" -eq 0 ]]; then
  pass "world switch launch exit 0"
elif [[ "${SW_EC}" -eq 137 ]] && grep -q 'Training finished' "${SW_OUT}/train.log" 2>/dev/null; then
  pass "world switch launch exit 137 (shutdown after train done)"
else
  fail "world switch exit=${SW_EC}"
fi

# 5) infer (use latest smoke/run with td3_actor)
wait_port_free "3D 推理冒烟" || exit 1
WEIGHTS=""
# Unmatched globs simply fail the file test; this keeps path names intact.
for d in "${WS}/3D/runs"/smoke_* "${WS}/3D/runs"/run_* "${WS}/3D/runs"/verify_switch_*; do
  if [[ -f "${d}/td3_actor" ]]; then WEIGHTS="${d}/td3"; break; fi
done
if [[ -n "${WEIGHTS}" ]]; then
  echo "==> infer_td3.launch.py weights=${WEIGHTS}"
  INF_LOG="${DEBUG_DIR}/verify_infer_${STAMP}.log"
  set +e
  run_owned_launch "infer_td3" 120 "${INF_LOG}" \
    ros2 launch scout_offroad_3d infer_td3.launch.py \
    gui:=false weights:="${WEIGHTS}"
  set -e
  grep -q 'infer_td3 loaded' "${INF_LOG}" && pass "infer_td3 loaded weights" || fail "infer_td3"
  grep -q 'spawn OK' "${INF_LOG}" && pass "infer spawn OK" || fail "infer spawn"
else
  fail "no td3_actor for infer test"
fi

# 6) gazebo_room.launch.py (brief)
wait_port_free "gazebo_room 短测" || exit 1
echo "==> scout_mini_pro_description gazebo_room.launch.py"
GR_LOG="${DEBUG_DIR}/verify_gazebo_room_${STAMP}.log"
set +e
run_owned_launch "gazebo_room" 20 "${GR_LOG}" \
  ros2 launch scout_mini_pro_description gazebo_room.launch.py gui:=false
GR_EC=$?
set -e
grep -q 'gzserver' "${GR_LOG}" && pass "gazebo_room gzserver started" || fail "gazebo_room"
[[ "${GR_EC}" -eq 124 || "${GR_EC}" -eq 0 ]] && pass "gazebo_room brief run" || fail "gazebo_room exit=${GR_EC}"

# 7) view_world.launch.py (brief)
wait_port_free "view_world 短测" || exit 1
echo "==> scout_offroad_3d view_world.launch.py"
VW_WORLD="$(ros2 pkg prefix scout_offroad_3d)/share/scout_offroad_3d/worlds/bumpy_offroad.world"
VW_LOG="${DEBUG_DIR}/verify_view_world_${STAMP}.log"
set +e
run_owned_launch "view_world" 15 "${VW_LOG}" \
  ros2 launch scout_offroad_3d view_world.launch.py gui:=false world:="${VW_WORLD}"
VW_EC=$?
set -e
grep -q 'gzserver' "${VW_LOG}" && pass "view_world gzserver started" || fail "view_world"
[[ "${VW_EC}" -eq 124 || "${VW_EC}" -eq 0 ]] && pass "view_world brief run" || fail "view_world exit=${VW_EC}"

wait_port_free "验收结束" || exit 1
echo ""
echo "========== LOG: ${LOG} =========="
if [[ "${FAILED}" -eq 0 ]]; then
  echo "========== 3D VERIFY PASS =========="
  write_validation_summary "PASS"
  exit 0
else
  echo "========== 3D VERIFY HAD FAILURES =========="
  write_validation_summary "FAIL (see log)"
  exit 1
fi
