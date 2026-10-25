"""ROS1 实车桥接：SSH 到 Xavier（连接诊断 / 自检 / 急停 / 锁车 / 录包）。"""

from __future__ import annotations

import math
import os
import re
import shlex
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from PySide6.QtCore import QObject, QProcess, QThread, QTimer, Signal

from .data_recorder import record_topics
from .safety_monitor import AlarmLevel, SafetyMonitor

PRIVATE_ROS_SETUP_ENV = "SCOUT_ROBOT_ROS_SETUP"
PRIVATE_ROBOT_WORKSPACE_ENV = "SCOUT_ROBOT_WORKSPACE"
PRIVATE_LONG_NAV_SOURCE_ENV = "SCOUT_ROBOT_LONG_NAV_SOURCE"
DEFAULT_ROBOT_HOST = ""
DEFAULT_ROBOT_USER = ""

LAN_FAIL_MSG = (
    "无法访问车辆。\n"
    "请确认工作站与车辆接入同一现场局域网。\n"
    "车辆用户名和主机地址须从本机私有配置或现场信息输入。"
)

AUTH_FAIL_MSG = (
    "网络已通，SSH 认证未通过。\n"
    "输入车辆系统密码。该密码仅用于本次会话，不写入磁盘。"
)

SAFETY_POLL_REMOTE = (
    "echo '===LOG==='; "
    "tail -n 30 /tmp/scout_nav_infer.log /tmp/scout_nav_gate.log /tmp/scout_nav_feat.log "
    "2>/dev/null | tail -n 50; "
    "echo '===ODOM==='; "
    "timeout 1.2 rostopic echo -n 1 /odom 2>/dev/null | head -n 35; "
    "echo '===IMU==='; "
    "timeout 1.2 rostopic echo -n 1 /imu/data_raw 2>/dev/null | head -n 40; "
    "echo '===FEAT==='; "
    "timeout 1.2 rostopic echo -n 1 /offroad/td3_range_features 2>/dev/null | head -n 25; "
    "echo '===END==='"
)

# Scout 2.0 24V pack: empty / alarm / full (user manual).
BATTERY_V_EMPTY = 22.0
BATTERY_V_ALARM = 24.0
BATTERY_V_FULL = 29.2

BATTERY_POLL_REMOTE = (
    "echo '===BAT==='; "
    "timeout 4 rostopic echo -n 1 /scout_status 2>/dev/null | head -n 120; "
    "echo '===BAT2==='; "
    "timeout 3 rostopic echo -n 1 /scout_mini_status 2>/dev/null | head -n 80; "
    "echo '===BAT3==='; "
    "timeout 3 rostopic echo -n 1 /battery_state 2>/dev/null | head -n 40; "
    "echo '===BATLIST==='; "
    "rostopic list 2>/dev/null | grep -iE 'battery|scout_status|BMS|bms' || true; "
    "echo '===ENDBAT==='"
)


@dataclass(frozen=True)
class _RemoteProcessRecord:
    """A process this bridge started and can later verify before stopping."""

    pid: str
    start_time: str
    expected_terms: Tuple[str, ...]
    label: str


def local_ipv4s() -> List[str]:
    ips: List[str] = []
    try:
        r = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2)
        ips.extend((r.stdout or "").split())
    except Exception:
        pass
    return [i.strip() for i in ips if i.count(".") == 3]


def same_prefix24(ip_a: str, ip_b: str) -> bool:
    a, b = ip_a.split("."), ip_b.split(".")
    if len(a) != 4 or len(b) != 4:
        return False
    return a[:3] == b[:3]


def lan_hint_for_host(host: str) -> Tuple[bool, str]:
    """Local-only check (does not touch the robot). True if some NIC shares /24 with host."""
    ips = [i for i in local_ipv4s() if not i.startswith("127.")]
    likely = any(same_prefix24(i, host) for i in ips)
    listing = ", ".join(ips) if ips else "(没有拿到本机 IPv4)"
    if likely:
        return True, f"本机 IPv4: {listing}（与 {host} 看起来同一网段）"
    return (
        False,
        f"本机 IPv4: {listing}\n与 {host} 不在同一网段。\n"
        "请确认工作站与车辆接入同一现场局域网。",
    )


def classify_connect_error(
    ping_ok: bool, ssh_rc: Optional[int], stderr: str, timed_out: bool
) -> Tuple[str, str]:
    """Return (kind, human_message). kind: net|auth|timeout|other."""
    err = (stderr or "").lower()
    if not ping_ok:
        return ("net", LAN_FAIL_MSG)
    if timed_out:
        return (
            "timeout",
            "SSH 登录超时。须确认车辆 SSH 服务已启动。",
        )
    if "permission denied" in err or "publickey" in err or "authentication" in err:
        return ("auth", AUTH_FAIL_MSG)
    extra = (stderr or "").strip()[:400]
    return ("other", "SSH 失败" + (f"：{extra}" if extra else "（未知原因）"))


def _section(text: str, name: str) -> str:
    key = f"==={name}==="
    if key not in text:
        return ""
    rest = text.split(key, 1)[1]
    nxt = rest.find("===")
    return rest if nxt < 0 else rest[:nxt]


def _first_float(block: str, field: str) -> Optional[float]:
    m = re.search(rf"{field}:\s*([-+0-9.eE]+)", block)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def _quat_to_rp(x: float, y: float, z: float, w: float) -> Tuple[float, float]:
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    return roll, pitch


def parse_safety_sample(text: str) -> dict:
    """Parse SSH safety-poll stdout into odom / imu / min-range / log flags."""
    out: dict = {"log": "", "x": None, "y": None, "roll": None, "pitch": None, "front": None}
    log = _section(text, "LOG")
    out["log"] = log
    odom = _section(text, "ODOM")
    out["x"] = _first_float(odom, "x")
    out["y"] = _first_float(odom, "y")
    imu = _section(text, "IMU")
    qx = _first_float(imu, "x")
    qy = _first_float(imu, "y")
    qz = _first_float(imu, "z")
    qw = _first_float(imu, "w")
    if None not in (qx, qy, qz, qw) and abs(qw) + abs(qx) + abs(qy) + abs(qz) > 0.1:
        out["roll"], out["pitch"] = _quat_to_rp(qx, qy, qz, qw)
    feat = _section(text, "FEAT")
    nums = [float(n) for n in re.findall(r"[-+]?\d+\.\d+(?:[eE][-+]?\d+)?", feat)]
    if nums:
        n = min(6, len(nums))
        out["front"] = min(nums[:n])
    return out


def voltage_to_pct(volts: float) -> int:
    """Map Scout 24V pack voltage to 0–100 using factory empty/alarm/full points."""
    v = float(volts)
    if v <= BATTERY_V_EMPTY:
        return 0
    if v >= BATTERY_V_FULL:
        return 100
    if v < BATTERY_V_ALARM:
        span = BATTERY_V_ALARM - BATTERY_V_EMPTY
        return int(round((v - BATTERY_V_EMPTY) / span * 15))
    span = BATTERY_V_FULL - BATTERY_V_ALARM
    return int(round(15 + (v - BATTERY_V_ALARM) / span * 85))


def parse_battery(text: str) -> Tuple[Optional[float], Optional[int]]:
    """Parse rostopic echo of ScoutStatus / BatteryState. Return (volts, percent)."""
    blob = text or ""
    pct: Optional[int] = None
    m = re.search(r"\bpercentage:\s*([-+0-9.eE]+)", blob)
    if not m:
        m = re.search(r"\bSOC:\s*([-+0-9.eE]+)", blob, re.I)
    if m:
        raw = float(m.group(1))
        pct = int(round(raw * 100.0 if 0.0 <= raw <= 1.5 else raw))
        pct = max(0, min(100, pct))
    volts: Optional[float] = None
    m = re.search(r"\bbattery_voltage:\s*([-+0-9.eE]+)", blob)
    if not m:
        m = re.search(r"\bbat(?:tery)?_volt(?:age)?:\s*([-+0-9.eE]+)", blob, re.I)
    if not m:
        m = re.search(r"(?m)^\s*voltage:\s*([-+0-9.eE]+)", blob)
    if m:
        volts = float(m.group(1))
        if volts > 80.0:
            volts = volts / 1000.0
        if 100.0 < volts <= 400.0:
            volts = volts / 10.0
        if pct is None and 15.0 < volts < 40.0:
            pct = voltage_to_pct(volts)
    return volts, pct


def format_battery_chip(
    connected: bool,
    volts: Optional[float],
    pct: Optional[int],
    pending: bool = False,
) -> Tuple[str, int, str]:
    """Return (label, pct_or_-1, tooltip) for the header chip."""
    if not connected:
        return ("未连接", -1, "未连接车辆，无电量数据")
    if pct is not None:
        tip = f"约 {pct}%"
        if volts is not None:
            tip += f"  ·  {volts:.1f} V"
        return (f"{pct}%", pct, tip)
    if volts is not None:
        return (f"{volts:.1f}V", -1, f"{volts:.1f} V（还没有百分比）")
    if pending:
        return ("…", -1, "正在读取电量")
    return ("—", -1, "读不到电量（车上 /scout_status）")


def run_ssh(
    user: str,
    host: str,
    remote: str,
    timeout: int = 12,
    password: Optional[str] = None,
) -> Tuple[int, str, str, bool]:
    """Run one remote command. password is optional and never logged."""
    target = f"{user}@{host}"
    cmd = [
        "ssh",
        "-o",
        f"ConnectTimeout={min(timeout, 8)}",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-o",
        "NumberOfPasswordPrompts=1",
    ]
    env = os.environ.copy()
    askpass = None
    popen_kw = {}
    if password:
        fd, askpass = tempfile.mkstemp(prefix="scout_nav_askpass_", suffix=".sh")
        os.write(fd, f"#!/bin/sh\nprintf '%s\\n' {shlex.quote(password)}\n".encode())
        os.close(fd)
        os.chmod(askpass, 0o700)
        env["SSH_ASKPASS"] = askpass
        env["SSH_ASKPASS_REQUIRE"] = "force"
        env.setdefault("DISPLAY", ":0")
        cmd += [
            "-o",
            "PreferredAuthentications=password,keyboard-interactive,publickey",
            "-o",
            "BatchMode=no",
        ]
        popen_kw["start_new_session"] = True
    else:
        cmd += ["-o", "BatchMode=yes"]
    cmd += [target, f"bash -lc {shlex.quote(remote)}"]
    timed_out = False
    try:
        r = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout + 2,
            env=env,
            **popen_kw,
        )
        rc, out, err = r.returncode, r.stdout or "", r.stderr or ""
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        rc, out, err = None, "", str(exc)
    except Exception as exc:
        rc, out, err = 1, "", str(exc)
    finally:
        if askpass:
            try:
                os.remove(askpass)
            except OSError:
                pass
    return rc if rc is not None else 1, out, err, timed_out


def _ssh_base_cmd(user: str, host: str, timeout: int, password: Optional[str]):
    target = f"{user}@{host}"
    cmd = [
        "ssh",
        "-o",
        f"ConnectTimeout={min(timeout, 8)}",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-o",
        "NumberOfPasswordPrompts=1",
    ]
    env = os.environ.copy()
    askpass = None
    extra = {}
    if password:
        fd, askpass = tempfile.mkstemp(prefix="scout_nav_askpass_", suffix=".sh")
        os.write(fd, f"#!/bin/sh\nprintf '%s\\n' {shlex.quote(password)}\n".encode())
        os.close(fd)
        os.chmod(askpass, 0o700)
        env["SSH_ASKPASS"] = askpass
        env["SSH_ASKPASS_REQUIRE"] = "force"
        env.setdefault("DISPLAY", ":0")
        cmd += [
            "-o",
            "PreferredAuthentications=password,keyboard-interactive,publickey",
            "-o",
            "BatchMode=no",
        ]
        extra["start_new_session"] = True
    else:
        cmd += ["-o", "BatchMode=yes"]
    return target, cmd, env, askpass, extra


def run_scp(
    user: str,
    host: str,
    remote_path: str,
    local_path: str,
    timeout: int = 30,
    password: Optional[str] = None,
) -> Tuple[int, str]:
    """Copy remote_path from the robot to local_path."""
    target, ssh_cmd, env, askpass, extra = _ssh_base_cmd(user, host, timeout, password)
    scp = ["scp"] + ssh_cmd[1:] + [f"{target}:{remote_path}", local_path]
    try:
        r = subprocess.run(
            scp, capture_output=True, text=True, timeout=timeout + 2, env=env, **extra
        )
        return r.returncode, (r.stderr or r.stdout or "")
    except Exception as exc:
        return 1, str(exc)
    finally:
        if askpass:
            try:
                os.remove(askpass)
            except OSError:
                pass


class _ConnectWorker(QObject):
    log_line = Signal(str)
    finished = Signal(bool, str, str)  # ok, kind, message

    def __init__(
        self,
        user: str,
        host: str,
        ping_timeout: int = 3,
        ssh_timeout: int = 8,
        password: Optional[str] = None,
        skip_ping: bool = False,
    ):
        super().__init__()
        self._user = user
        self._host = host
        self._ping_timeout = ping_timeout
        self._ssh_timeout = ssh_timeout
        self._password = password
        self._skip_ping = skip_ping

    def run(self) -> None:
        target = f"{self._user}@{self._host}"
        self.log_line.emit(f"正在连接 {target}…")
        same, hint = lan_hint_for_host(self._host)
        self.log_line.emit("· " + hint.replace("\n", " | "))
        ping_ok = True
        if not self._skip_ping:
            ping_cmd = ["ping", "-c", "2", "-W", "1", self._host]
            self.log_line.emit("· 正在 ping 车辆…")
            ping = subprocess.run(
                ping_cmd, capture_output=True, text=True, timeout=self._ping_timeout + 3
            )
            ping_ok = ping.returncode == 0
            if ping_ok:
                self.log_line.emit("· ping 成功，车辆在线。")
            else:
                self.log_line.emit("· ping 失败，请检查是否接入同一现场局域网。")
                if ping.stdout:
                    for ln in ping.stdout.strip().splitlines()[-2:]:
                        self.log_line.emit("    " + ln)

        if self._password:
            self.log_line.emit("· 正在用密码登录…")
        else:
            self.log_line.emit("· 正在登录…")
        ssh_rc, stdout, stderr, timed_out = run_ssh(
            self._user,
            self._host,
            "echo OK && hostname",
            timeout=self._ssh_timeout,
            password=self._password,
        )

        if not timed_out and ssh_rc == 0 and "OK" in stdout:
            hostn = stdout.replace("OK", "").strip().splitlines()
            extra = hostn[-1] if hostn else ""
            self.log_line.emit("· 登录成功" + (f"（{extra}）" if extra else ""))
            self.finished.emit(True, "ok", f"已连接到 {target}")
            return

        kind, msg = classify_connect_error(ping_ok, ssh_rc, stderr, timed_out)
        if kind == "auth" and self._password:
            msg = "认证失败。重新输入车辆系统密码。"
        self.log_line.emit("· " + msg.split("\n")[0])
        self.finished.emit(False, kind, msg)


def _names_in(text: str) -> List[str]:
    return [ln.strip() for ln in (text or "").splitlines() if ln.strip() and not ln.strip().startswith("ERROR")]


def _has_any(text: str, needles: Tuple[str, ...]) -> List[str]:
    blob = text or ""
    return [n for n in needles if n in blob]


class _FnWorker(QObject):
    finished = Signal(object)

    def __init__(self, fn: Callable):
        super().__init__()
        self._fn = fn

    def run(self) -> None:
        self.finished.emit(self._fn())


class RealBridge(QObject):
    """实车模式：SSH 连接、自检、急停、锁车心跳、录包。"""

    log_message = Signal(str)
    connected = Signal(bool)
    connect_finished = Signal(bool, str, str)  # ok, kind, message
    check_result = Signal(str, bool, str)  # name, ok, detail
    locked_changed = Signal(bool)
    safety_alarm = Signal(str, str)  # level, message
    battery_changed = Signal(str, int, str)  # label, pct_or_-1, tooltip

    def __init__(
        self, host: str = DEFAULT_ROBOT_HOST, user: str = DEFAULT_ROBOT_USER
    ) -> None:
        super().__init__()
        self._host = host
        self._user = user
        self._connected = False
        self._locked = False
        self._enable_cmd = False
        self._thread: Optional[QThread] = None
        self._worker: Optional[_ConnectWorker] = None
        self._check_thread: Optional[QThread] = None
        self._bag_remote_process: Optional[_RemoteProcessRecord] = None
        self._remote_launches: dict[str, _RemoteProcessRecord] = {}
        self._safety_en = {"stuck": True, "rollover": True, "collision": True}
        self._safety = SafetyMonitor()
        self._safety_busy = False
        self._last_critical = 0.0
        self._safety_timer = QTimer(self)
        self._safety_timer.setInterval(2800)
        self._safety_timer.timeout.connect(self._kick_safety_poll)
        self._safety_proc: Optional[QProcess] = None
        self._check_worker = None
        self._ssh_password: Optional[str] = None
        self._bat_busy = False
        self._bat_logged_ok = False
        self._bat_logged_miss = False
        self._bat_last_low = 0.0
        self._bat_timer = QTimer(self)
        self._bat_timer.setInterval(8000)
        self._bat_timer.timeout.connect(self._kick_battery_poll)

    @property
    def connected_ok(self) -> bool:
        return self._connected

    @property
    def locked(self) -> bool:
        return self._locked

    def set_target(self, user: str, host: str) -> None:
        user = user.strip()
        host = host.strip()
        if (user, host) != (self._user, self._host):
            # A PID/start-time record has meaning only on the host that created
            # it. Forget it before switching targets rather than ever sending
            # a stop request to another vehicle.
            if self._remote_launches or self._bag_remote_process is not None:
                self.log_message.emit(
                    "已切换车辆目标；已忘记先前目标的进程记录，未向其发送终止信号。"
                )
            self._remote_launches.clear()
            self._bag_remote_process = None
        self._user = user
        self._host = host

    @staticmethod
    def _private_ros_prefix() -> Tuple[Optional[str], str]:
        """Build a ROS environment prefix from locally held deployment config."""
        setup = os.environ.get(PRIVATE_ROS_SETUP_ENV, "").strip()
        if not setup:
            return (
                None,
                "未配置车辆端 ROS 环境。请在本机私有配置中设置 "
                f"{PRIVATE_ROS_SETUP_ENV} 为车辆端 setup.bash 的绝对路径。",
            )
        if not setup.startswith("/"):
            return (
                None,
                f"{PRIVATE_ROS_SETUP_ENV} 必须是车辆端的绝对路径。",
            )
        safe_setup = shlex.quote(setup)
        return (
            "source /opt/ros/noetic/setup.bash 2>/dev/null; "
            f"if [ ! -f {safe_setup} ]; then "
            "echo SCOUT_NAV_PRIVATE_ROS_SETUP_MISSING; exit 125; fi; "
            f"source {safe_setup} 2>/dev/null || "
            "{ echo SCOUT_NAV_PRIVATE_ROS_SETUP_FAILED; exit 125; }; "
            "export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}; ",
            "",
        )

    @staticmethod
    def _private_robot_workspace() -> Tuple[Optional[str], str]:
        """Return the remote workspace only when supplied through private config."""
        workspace = os.environ.get(PRIVATE_ROBOT_WORKSPACE_ENV, "").strip().rstrip("/")
        if not workspace:
            return (
                None,
                "未配置车辆端工作区。请在本机私有配置中设置 "
                f"{PRIVATE_ROBOT_WORKSPACE_ENV} 为车辆端工作区的绝对路径。",
            )
        if not workspace.startswith("/"):
            return (
                None,
                f"{PRIVATE_ROBOT_WORKSPACE_ENV} 必须是车辆端的绝对路径。",
            )
        return workspace, ""

    @staticmethod
    def _private_long_nav_ros1_source() -> Tuple[Optional[Path], str]:
        """Get an explicitly maintained local ROS1 package copy for deployment.

        The workspace archive is historical evidence, not a deployment source.
        A real-robot operator must deliberately point to a reviewed, maintained
        package copy outside this application's default data paths.
        """
        raw_source = os.environ.get(PRIVATE_LONG_NAV_SOURCE_ENV, "").strip()
        if not raw_source:
            return (
                None,
                "未配置本机 ROS1 长途导航包。请在本机私有配置中设置 "
                f"{PRIVATE_LONG_NAV_SOURCE_ENV} 为经过审核的 "
                "scout_long_nav_ros1 包目录绝对路径。",
            )

        source = Path(raw_source).expanduser()
        if not source.is_absolute():
            return None, f"{PRIVATE_LONG_NAV_SOURCE_ENV} 必须是本机绝对路径。"
        if not source.is_dir() or not (source / "package.xml").is_file():
            return (
                None,
                f"{PRIVATE_LONG_NAV_SOURCE_ENV} 不是有效的 scout_long_nav_ros1 包目录。",
            )
        return source, ""

    def ssh_run(self, remote_cmd: str, timeout: int = 20) -> Tuple[int, str, str]:
        """Run a command on the robot. Returns (rc, stdout, stderr)."""
        rc, out, err, _to = run_ssh(
            self._user,
            self._host,
            remote_cmd,
            timeout=timeout,
            password=self._ssh_password,
        )
        return rc, out, err

    def ssh_ros(self, ros_cmd: str, timeout: int = 25) -> Tuple[int, str, str]:
        prefix, error = self._private_ros_prefix()
        if prefix is None:
            return 2, "", error
        return self.ssh_run(prefix + ros_cmd, timeout=timeout)

    @staticmethod
    def _launch_key(package: str, launch_file: str) -> str:
        return f"{package}:{launch_file}"

    @staticmethod
    def _remote_launch_scan(launch_file: str) -> str:
        """Read matching launches while excluding this SSH command's ancestry.

        The remote shell itself contains the launch filename in its command
        line, so a plain name scan would falsely report that shell as an
        external launch. Excluding the complete ancestor chain keeps this a
        read-only external-conflict check.
        """
        return (
            f"launch_pattern={shlex.quote(launch_file)}; "
            'ignored_pids=" ${BASHPID:-$$} "; scan_pid=${BASHPID:-$$}; '
            'while [ "$scan_pid" -gt 1 ] 2>/dev/null; do '
            "scan_pid=$(awk '{print $4}' \"/proc/$scan_pid/stat\" 2>/dev/null || echo 1); "
            'ignored_pids="$ignored_pids $scan_pid "; '
            "done; "
            'pgrep -af -- "$launch_pattern" 2>/dev/null | '
            "awk -v ignored=\"$ignored_pids\" "
            "'BEGIN { n=split(ignored, ids, \" \"); for (i=1; i<=n; i++) skip[ids[i]]=1 } "
            "!($1 in skip)' || true"
        )

    @staticmethod
    def _remote_record_command(record: _RemoteProcessRecord, *, terminate: bool) -> str:
        """Build a remote, exact-PID operation guarded by process identity.

        A numeric PID alone is not sufficient proof of ownership because it
        can be reused.  Before every stop we check its Linux start time and
        command line against the record created by this bridge.
        """
        if not record.pid.isdigit() or not record.start_time.isdigit():
            raise ValueError("invalid recorded remote process identity")
        term_checks = " && ".join(
            f'printf "%s" "$cmd" | grep -F -- {shlex.quote(term)} >/dev/null'
            for term in record.expected_terms
        )
        if not term_checks:
            raise ValueError("record must include command-line identity terms")
        action = (
            'if kill -TERM -- "$pid"; then echo SCOUT_NAV_STOPPED; '
            'else echo SCOUT_NAV_STOP_FAILED; exit 5; fi'
            if terminate
            else "echo SCOUT_NAV_OWNED"
        )
        return (
            f"pid={record.pid}; expected_start={record.start_time}; "
            'if [ ! -r "/proc/$pid/stat" ] || [ ! -r "/proc/$pid/cmdline" ]; then '
            "echo SCOUT_NAV_GONE; exit 3; fi; "
            "actual_start=$(awk '{print $22}' \"/proc/$pid/stat\" 2>/dev/null || true); "
            "cmd=$(tr '\\000' ' ' < \"/proc/$pid/cmdline\" 2>/dev/null || true); "
            'if [ "$actual_start" != "$expected_start" ]; then '
            "echo SCOUT_NAV_UNOWNED; exit 4; fi; "
            f"if {term_checks}; then {action}; "
            "else echo SCOUT_NAV_UNOWNED; exit 4; fi"
        )

    def _probe_remote_process(self, record: _RemoteProcessRecord) -> Tuple[str, str]:
        """Return owned/gone/unowned/unknown without changing remote state."""
        try:
            remote = self._remote_record_command(record, terminate=False)
        except ValueError as exc:
            return "unowned", str(exc)
        rc, out, err = self.ssh_run(remote, timeout=8)
        text = (out or "") + "\n" + (err or "")
        if "SCOUT_NAV_OWNED" in text:
            return "owned", "身份已验证"
        if "SCOUT_NAV_GONE" in text:
            return "进程已经退出"
        if "SCOUT_NAV_UNOWNED" in text:
            return "PID 或命令行与本应用记录不一致"
        detail = text.strip()[-300:]
        return "unknown", detail or f"远端检查失败（exit={rc}）"

    def _stop_owned_remote_process(self, record: _RemoteProcessRecord) -> Tuple[str, str]:
        """Stop one previously recorded remote process, never a name match."""
        state, detail = self._probe_remote_process(record)
        if state != "owned":
            return state, detail
        try:
            remote = self._remote_record_command(record, terminate=True)
        except ValueError as exc:
            return "unowned", str(exc)
        rc, out, err = self.ssh_run(remote, timeout=8)
        text = (out or "") + "\n" + (err or "")
        if "SCOUT_NAV_STOPPED" in text:
            return "stopped", "已向本应用启动的精确 PID 发送 SIGTERM"
        if "SCOUT_NAV_GONE" in text:
            return "gone", "进程已在停止前退出"
        if "SCOUT_NAV_UNOWNED" in text:
            return "unowned", "进程身份在停止前发生变化，未发送终止信号"
        detail = text.strip()[-300:]
        return "unknown", detail or f"远端停止检查失败（exit={rc}）"

    def _remote_launch_conflicts(self, launch_files: Tuple[str, ...]) -> List[str]:
        """Read-only report for matching remote launches not owned by this UI."""
        lines: List[str] = []
        for launch_file in launch_files:
            _rc, out, _err = self.ssh_run(self._remote_launch_scan(launch_file), timeout=8)
            lines.extend(line.strip() for line in (out or "").splitlines() if line.strip())
        return list(dict.fromkeys(lines))

    def _remote_start_time(self, pid: str) -> Optional[str]:
        """Read a just-started remote process's birth marker without signalling it."""
        if not pid.isdigit():
            return None
        rc, out, _err = self.ssh_run(
            f"if [ -r /proc/{pid}/stat ]; then awk '{{print $22}}' /proc/{pid}/stat; fi",
            timeout=8,
        )
        if rc != 0:
            return None
        values = re.findall(r"^\s*(\d+)\s*$", out or "", re.MULTILINE)
        return values[-1] if values else None

    def connect_async(self, password: Optional[str] = None, skip_ping: bool = False) -> None:
        if self._thread is not None:
            self.log_message.emit("正在连接。")
            return
        if not self._user or not self._host:
            message = "请先从本机私有配置或现场信息填写“用户名@车辆主机”。"
            self._connected = False
            self.connected.emit(False)
            self.connect_finished.emit(False, "config", message)
            self.log_message.emit("✗ " + message)
            return
        self._connected = False
        self._pending_password = password
        thread = QThread(self)
        worker = _ConnectWorker(
            self._user, self._host, password=password, skip_ping=skip_ping
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.log_line.connect(self.log_message)
        worker.finished.connect(self._on_connect_worker_done)
        worker.finished.connect(thread.quit)
        thread.finished.connect(self._cleanup_connect_thread)
        self._thread = thread
        self._worker = worker
        thread.start()

    def _on_connect_worker_done(self, ok: bool, kind: str, message: str) -> None:
        self._connected = ok
        self.connected.emit(ok)
        self.connect_finished.emit(ok, kind, message)
        if ok:
            self._ssh_password = getattr(self, "_pending_password", None)
            self._safety_timer.start()
            self._bat_logged_ok = False
            self._bat_logged_miss = False
            self._emit_battery(True, None, None, pending=True)
            self._bat_timer.start()
            QTimer.singleShot(500, self._kick_battery_poll)
            self.log_message.emit("· 已连接。")
        else:
            self._safety_timer.stop()
            self._bat_timer.stop()
            self._emit_battery(False, None, None)
            if kind != "auth":
                self._ssh_password = None

    def _cleanup_connect_thread(self) -> None:
        if self._worker is not None:
            self._worker.deleteLater()
            self._worker = None
        if self._thread is not None:
            self._thread.deleteLater()
            self._thread = None

    def _spawn_check(self, fn: Callable) -> None:
        if self._check_thread is not None:
            self.log_message.emit("上一自检尚未结束。")
            return

        def _wrap():
            try:
                return fn()
            except Exception as exc:
                return ("自检", False, str(exc))

        thread = QThread(self)
        worker = _FnWorker(_wrap)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)

        def _done(result) -> None:
            name, ok, detail = result
            self.check_result.emit(name, ok, detail)

        worker.finished.connect(_done)
        worker.finished.connect(thread.quit)
        thread.finished.connect(lambda: self._cleanup_check_thread(thread, worker))
        self._check_thread = thread
        self._check_worker = worker
        thread.start()

    def _cleanup_check_thread(self, thread: QThread, worker: _FnWorker) -> None:
        worker.deleteLater()
        thread.deleteLater()
        if self._check_thread is thread:
            self._check_thread = None
        if getattr(self, "_check_worker", None) is worker:
            self._check_worker = None

    def check_nodes(self) -> None:
        if not self._connected:
            self.check_result.emit("节点", False, "未连接")
            return
        self.log_message.emit("检查节点")
        self.log_message.emit("  scout_base：底盘驱动。发布 /cmd_vel 订阅端、/odom、电池状态。遥控与运动控制依赖该节点。")
        self.log_message.emit("  scout_mini_base：Scout Mini 底盘驱动。本车为 Scout 2.0，该节点缺席不影响。")
        self.log_message.emit("  rslidar：16 线激光雷达驱动。输出 /rslidar_points。遥控不依赖；建图与厂方导航依赖。")
        self.log_message.emit("  realsense：车载 D435。输出彩色/深度图像。遥控、厂方 2D 导航与当前 TD3 均不依赖。")

        def work():
            rc, out, err = self.ssh_ros("rosnode list 2>&1", timeout=20)
            text = (out or "") + "\n" + (err or "")
            if rc != 0 or "Unable to communicate with master" in text or "command not found" in text:
                self.log_message.emit("  ✗ 读不到 rosnode（车上 ROS 没进环境或 roscore 未开）")
                if text.strip():
                    self.log_message.emit("    " + text.strip()[:500].replace("\n", "\n    "))
                return ("节点", False, "读不到节点列表")
            chassis = _has_any(text, ("scout_base", "scout_minimal"))
            mini = _has_any(text, ("scout_mini_base",))
            lidar = _has_any(text, ("rslidar", "rslidar_sdk"))
            cam = _has_any(text, ("realsense",))
            self.log_message.emit(f"  {'✓' if chassis else '✗'}  scout_base（底盘，必须）")
            self.log_message.emit(
                f"  {'·' if mini else '—'}  scout_mini_base（Mini 专用，这台车可忽略）"
            )
            self.log_message.emit(f"  {'✓' if lidar else '✗'}  rslidar（雷达，可选）")
            self.log_message.emit(f"  {'✓' if cam else '✗'}  realsense（相机，可选）")
            ok = bool(chassis)
            detail = "底盘节点在" if ok else "没有底盘节点"
            extras = []
            if ok and not lidar:
                extras.append("雷达未起")
            if ok and not cam:
                extras.append("相机未起")
            if extras:
                detail += "；" + "、".join(extras) + "（遥控不受影响）"
            return ("节点", ok, detail)

        self._spawn_check(work)

    def check_topics(self) -> None:
        if not self._connected:
            self.check_result.emit("话题", False, "未连接")
            return
        self.log_message.emit("检查话题")
        self.log_message.emit("  /cmd_vel：速度指令（geometry_msgs/Twist）。本自检只检测话题是否存在，不发布。")
        self.log_message.emit("  /odom：里程计（nav_msgs/Odometry）。位姿与速度反馈。")
        self.log_message.emit("  /rslidar_points：16 线激光雷达点云（sensor_msgs/PointCloud2）。")
        self.log_message.emit("  /scan：由点云投影得到的平面激光（sensor_msgs/LaserScan），供厂方 gmapping 使用。")
        self.log_message.emit("  /camera/color/image_raw：车载相机彩色图像。")

        def work():
            rc, out, err = self.ssh_ros("rostopic list 2>&1", timeout=20)
            text = (out or "") + "\n" + (err or "")
            if rc != 0 or "Unable to communicate with master" in text or "command not found" in text:
                self.log_message.emit("  ✗ 读不到话题列表")
                if text.strip():
                    self.log_message.emit("    " + text.strip()[:500].replace("\n", "\n    "))
                return ("话题", False, "读不到话题列表")
            listed = set(text.split())
            items = [
                ("/cmd_vel", True, "速度"),
                ("/odom", True, "里程计"),
                ("/rslidar_points", False, "雷达点云"),
                ("/scan", False, "2D 激光（厂方建图用）"),
                ("/camera/color/image_raw", False, "相机彩色图"),
            ]
            missing_req = []
            for name, required, meaning in items:
                hit = name in listed
                tag = "必须" if required else "可选"
                self.log_message.emit(
                    f"  {'✓' if hit else '✗'}  {name}  {meaning}（{tag}）"
                )
                if required and not hit:
                    missing_req.append(name)
            ok = not missing_req
            detail = "速度和里程计话题都在" if ok else "缺少 " + ", ".join(missing_req)
            return ("话题", ok, detail)

        self._spawn_check(work)

    def check_tf(self) -> None:
        if not self._connected:
            self.check_result.emit("TF", False, "未连接")
            return
        self.log_message.emit("检查 TF")
        self.log_message.emit("  需要：odom、base_link（底盘坐标系）")

        def work():
            _rc, out, err = self.ssh_ros(
                "timeout 4 rosrun tf tf_echo odom base_link 2>&1 | head -n 20; "
                "echo '---'; timeout 3 rostopic echo -n 1 /tf 2>/dev/null | head -n 30",
                timeout=16,
            )
            text = (out or "") + "\n" + (err or "")
            if "command not found" in text and "At time" not in text and "Translation" not in text:
                self.log_message.emit("  ✗ 读不到 TF")
                if text.strip():
                    self.log_message.emit("    " + text.strip()[:400].replace("\n", "\n    "))
                return ("TF", False, "读不到 TF")
            has_odom = "odom" in text
            has_base = "base_link" in text or "base_footprint" in text
            has_tf = (
                "Translation" in text
                or "At time" in text
                or "child_frame_id" in text
                or (has_odom and has_base)
            )
            self.log_message.emit(f"  {'✓' if has_odom else '✗'}  odom")
            self.log_message.emit(f"  {'✓' if has_base else '✗'}  base_link")
            ok = bool(has_tf)
            detail = "TF 有 odom / base_link" if ok else "没有 odom↔base_link"
            return ("TF", ok, detail)

        self._spawn_check(work)

    def check_storage(self) -> None:
        if not self._connected:
            self.check_result.emit("存储", False, "未连接")
            return
        self.log_message.emit("检查存储")
        self.log_message.emit("  看车上 HOME 剩余空间，少于 1 GiB 为不过")

        def work():
            rc, out, err = self.ssh_run("df -h $HOME; echo '---'; df -P $HOME | tail -1", timeout=10)
            text = (out or err or "").strip()
            if text:
                self.log_message.emit("    " + text.splitlines()[0][:120])
            ok = False
            detail = "df 失败"
            try:
                last = text.splitlines()[-1].split()
                avail_kb = int(last[3])
                avail_gb = avail_kb / (1024 * 1024)
                ok = rc == 0 and avail_gb >= 1.0
                self.log_message.emit(f"  {'✓' if ok else '✗'}  剩余约 {avail_gb:.1f} GiB")
                detail = f"剩余约 {avail_gb:.1f} GiB"
            except Exception:
                self.log_message.emit("  ✗ 无法解析 df")
                detail = text[:300] or detail
                ok = False
            return ("存储", ok, detail)

        self._spawn_check(work)

    def set_safety_enable(self, key: str, enabled: bool) -> None:
        if key in self._safety_en:
            self._safety_en[key] = enabled
        self.log_message.emit(f"安全开关 {key}: {'开' if enabled else '关'}（实车走 SSH 抽样 / 车上日志）")

    def _emit_battery(
        self,
        connected: bool,
        volts: Optional[float],
        pct: Optional[int],
        pending: bool = False,
    ) -> None:
        label, shown, tip = format_battery_chip(connected, volts, pct, pending=pending)
        self.battery_changed.emit(label, shown, tip)

    def _kick_battery_poll(self) -> None:
        if not self._connected or self._bat_busy:
            return
        self._bat_busy = True

        def work():
            _rc, out, _err = self.ssh_ros(BATTERY_POLL_REMOTE, timeout=20)
            return out or ""

        thread = QThread(self)
        worker = _FnWorker(work)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)

        def _done(raw) -> None:
            self._on_battery_text(raw if isinstance(raw, str) else "")

        worker.finished.connect(_done)
        worker.finished.connect(thread.quit)
        thread.finished.connect(lambda: self._cleanup_battery_thread(thread, worker))
        thread.start()

    def _cleanup_battery_thread(self, thread: QThread, worker: _FnWorker) -> None:
        worker.deleteLater()
        thread.deleteLater()
        self._bat_busy = False

    def _on_battery_text(self, raw: str) -> None:
        if not self._connected:
            self._emit_battery(False, None, None)
            return
        volts, pct = parse_battery(raw)
        if volts is None and pct is None:
            self._emit_battery(True, None, None)
            if not self._bat_logged_miss:
                self._bat_logged_miss = True
                self.log_message.emit("电量：未能解析 battery_voltage。须确认底盘已发布 /scout_status。")
                snippet = (raw or "").strip().replace("\n", " | ")[:240]
                if snippet:
                    self.log_message.emit("  原始输出: " + snippet)
            return
        self._emit_battery(True, volts, pct)
        if not self._bat_logged_ok:
            self._bat_logged_ok = True
            bits = []
            if pct is not None:
                bits.append(f"{pct}%")
            if volts is not None:
                bits.append(f"{volts:.1f} V")
            self.log_message.emit("· 电量 " + " · ".join(bits))
        low = (pct is not None and pct <= 15) or (
            volts is not None and volts <= BATTERY_V_ALARM
        )
        if low:
            import time as _t

            now = _t.time()
            if now - self._bat_last_low >= 90.0:
                self._bat_last_low = now
                extra = []
                if pct is not None:
                    extra.append(f"{pct}%")
                if volts is not None:
                    extra.append(f"{volts:.1f} V")
                self.log_message.emit("⚠ 小车电量偏低（" + " · ".join(extra) + "）")

    def _kick_safety_poll(self) -> None:
        if not self._connected or self._safety_busy:
            return
        if not any(self._safety_en.values()):
            return
        if self._check_thread is not None:
            return
        self._safety_busy = True

        def work():
            _rc, out, _err = self.ssh_ros(SAFETY_POLL_REMOTE, timeout=12)
            return out

        thread = QThread(self)
        worker = _FnWorker(work)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)

        def _done(raw) -> None:
            self._on_safety_text(raw if isinstance(raw, str) else "")

        worker.finished.connect(_done)
        worker.finished.connect(thread.quit)
        thread.finished.connect(lambda: self._cleanup_safety_thread(thread, worker))
        self._safety_thread = thread
        self._safety_worker = worker
        thread.start()

    def _cleanup_safety_thread(self, thread: QThread, worker: _FnWorker) -> None:
        worker.deleteLater()
        thread.deleteLater()
        self._safety_busy = False

    def _on_safety_text(self, raw: str) -> None:
        sample = parse_safety_sample(raw)
        log = sample.get("log") or ""
        if self._safety_en.get("rollover") and (
            "ROLLOVER" in log.upper() or "翻车" in log
        ):
            self._emit_critical("翻车", "车上日志出现翻车关键字 → 急停+锁车")
            return
        if self._safety_en.get("collision") and re.search(r"COLLISION|碰撞", log, re.I):
            self.safety_alarm.emit("warn", "碰撞风险（车上日志）")
            self.log_message.emit("⚠ 碰撞风险（仅报警，不自动停车）")
        roll, pitch = sample.get("roll"), sample.get("pitch")
        if self._safety_en.get("rollover") and roll is not None and pitch is not None:
            alarm = self._safety.check_rollover(roll, pitch)
            if alarm and alarm.level == AlarmLevel.CRITICAL:
                self._emit_critical("翻车", alarm.message + " → 急停+锁车")
                return
        x, y = sample.get("x"), sample.get("y")
        if self._safety_en.get("stuck") and x is not None and y is not None:
            commanding = self._enable_cmd and not self._locked
            alarm = self._safety.update_odom(x, y, commanding)
            if alarm:
                if alarm.level == AlarmLevel.CRITICAL:
                    self._emit_critical("卡死", alarm.message + " → 急停+锁车")
                    return
                self.log_message.emit("· " + alarm.message)
        front = sample.get("front")
        if self._safety_en.get("collision") and front is not None:
            alarm = self._safety.check_collision_risk(front)
            if alarm:
                self.safety_alarm.emit("warn", alarm.message)
                self.log_message.emit("⚠ " + alarm.message)

    def _emit_critical(self, kind: str, message: str) -> None:
        import time as _t

        now = _t.time()
        if now - self._last_critical < 8.0:
            return
        self._last_critical = now
        self.log_message.emit(f"⚠⚠ {kind}: {message}")
        self.safety_alarm.emit("critical", message)
        self.estop()

    def _guard_motion(self, action: str) -> bool:
        if not self._connected:
            self.log_message.emit(f"✗ {action}：须先连接车辆")
            return False
        if self._locked:
            self.log_message.emit("✗ 车辆已锁定，须先解锁")
            return False
        return True

    def set_locked(self, locked: bool) -> bool:
        """PC-side software lock only. Does not publish /cmd_vel or touch factory/RC."""
        if not self._connected:
            self.log_message.emit("✗ 未连接，无法锁车")
            return False
        self._locked = locked
        self.locked_changed.emit(locked)
        if locked:
            self._enable_cmd = False
            self.log_message.emit("✓ 已锁，本软件不再下发运动")
        else:
            self.log_message.emit("✓ 已解锁")
        return True

    def set_enable_cmd_vel(self, enable: bool) -> bool:
        if enable:
            self.log_message.emit("✗ 速度输出：关闭。本模式不向 /cmd_vel 发布。")
            return False
        if not self._guard_motion("关闭速度输出"):
            return False
        self._enable_cmd = False
        return True

    def estop(self) -> None:
        self.log_message.emit("急停：本软件停止控车")
        self._enable_cmd = False
        self._locked = True
        self.locked_changed.emit(True)

    def start_mapping(self) -> Tuple[bool, str]:
        if not self._guard_motion("开始建图"):
            return False, "车辆已锁定，须先解锁。" if self._locked else "须先连接车辆。"
        rc, out, err = self.ssh_ros("rostopic list", timeout=15)
        text = out or ""
        if "/rslidar_points" not in text or "/odom" not in text:
            return False, (
                "建图数据源未就绪。\n"
                "所需话题：/rslidar_points，/odom。\n"
                "前提：厂方底盘与雷达已启动。"
            )
        return True, "雷达与里程计话题已确认。"

    def start_roslaunch(
        self,
        package: str,
        launch_file: str,
        title: str,
        require_unlock: bool = True,
    ) -> Tuple[bool, str]:
        if require_unlock:
            if not self._guard_motion(title):
                return False, "车辆已锁定，须先解锁。" if self._locked else "须先连接车辆。"
        elif not self._connected:
            return False, "须先连接车辆。"
        prefix, config_error = self._private_ros_prefix()
        if prefix is None:
            return False, config_error
        rc, out, _ = self.ssh_run(prefix + "rostopic list 2>&1 | head -n 8", timeout=12)
        if rc != 0 or "Unable to communicate with master" in (out or ""):
            return False, "ROS master 未就绪。须先启动底盘（scout_minimal）。"
        key = self._launch_key(package, launch_file)
        previous = self._remote_launches.get(key)
        if previous is not None:
            state, detail = self._probe_remote_process(previous)
            if state == "owned":
                return False, f"本应用本次连接已启动 {launch_file}（pid={previous.pid}）。"
            if state == "unknown":
                return False, f"无法验证先前启动的 {launch_file}；为安全起见不会重复启动：{detail}"
            self._remote_launches.pop(key, None)
            if state == "unowned":
                return False, (
                    f"先前记录的 {launch_file} 已不再可证明归本应用所有；"
                    "未终止该进程，请在其原终端检查后再试。"
                )
        self.log_message.emit(f"启动厂方：roslaunch {package} {launch_file}")
        safe_package = shlex.quote(package)
        safe_launch = shlex.quote(launch_file)
        safe_log = shlex.quote(f"/tmp/scout_nav_{Path(launch_file).name}.log")
        scan = self._remote_launch_scan(launch_file)
        remote = (
            prefix
            + f"matches=$({scan}); "
            "if [ -n \"$matches\" ]; then "
            "echo SCOUT_NAV_CONFLICT; printf '%s\\n' \"$matches\"; exit 17; fi; "
            f"nohup roslaunch {safe_package} {safe_launch} > {safe_log} 2>&1 & "
            "pid=$!; "
            "for _ in 1 2 3 4 5; do "
            'if [ -r "/proc/$pid/stat" ]; then '
            "start=$(awk '{print $22}' \"/proc/$pid/stat\" 2>/dev/null || true); "
            'if [ -n "$start" ]; then printf "SCOUT_NAV_STARTED %s %s\\n" "$pid" "$start"; exit 0; fi; '
            "fi; sleep 0.1; done; "
            "echo SCOUT_NAV_START_UNVERIFIED; exit 1"
        )
        launch_rc, launch_out, launch_err = self.ssh_run(remote, timeout=15)
        if "SCOUT_NAV_CONFLICT" in (launch_out or ""):
            detail = "\n".join(
                line for line in launch_out.splitlines() if line != "SCOUT_NAV_CONFLICT"
            )
            return False, (
                f"车辆端已存在 {launch_file}，可能来自其他终端；"
                "本应用未启动或终止它。"
                + (f"\n{detail[-500:]}" if detail else "")
            )
        match = re.search(
            r"^SCOUT_NAV_STARTED\s+(\d+)\s+(\d+)\s*$", launch_out or "", re.MULTILINE
        )
        if launch_rc != 0 or match is None:
            detail = ((launch_err or launch_out) or "").strip()[-400:]
            return False, f"{launch_file} 启动失败或无法记录进程身份：{detail}"
        record = _RemoteProcessRecord(
            pid=match.group(1),
            start_time=match.group(2),
            expected_terms=("roslaunch", package, launch_file),
            label=f"{package} {launch_file}",
        )
        state, detail = self._probe_remote_process(record)
        if state != "owned":
            return False, (
                f"{launch_file} 启动后无法验证进程归属（{detail}）；"
                "为安全起见，本应用以后不会尝试终止它。"
            )
        self._remote_launches[key] = record
        return True, title

    def start_factory_launch(self, launch_file: str, title: str) -> Tuple[bool, str]:
        return self.start_roslaunch("scout_bringup", launch_file, title)

    def start_lidar(self) -> Tuple[bool, str]:
        ok, msg = self.start_factory_launch("open_rslidar.launch", "开雷达")
        if not ok:
            return False, msg
        return True, "已启动雷达。\n确认：检查话题 /rslidar_points。"

    def _ensure_long_nav_ros1(self) -> Tuple[bool, str]:
        rc, out, _ = self.ssh_ros(
            "rospack find scout_long_nav_ros1 >/dev/null && echo OK", timeout=12
        )
        if rc == 0 and "OK" in (out or ""):
            return True, "ok"
        src, source_error = self._private_long_nav_ros1_source()
        if src is None:
            return False, source_error
        workspace, config_error = self._private_robot_workspace()
        if workspace is None:
            return False, config_error
        self.log_message.emit("正在将 scout_long_nav_ros1 同步到车辆端私有工作区…")
        remote_package = f"{workspace}/src/scout_long_nav_ros1"
        dest = f"{self._user}@{self._host}:{remote_package}/"
        _tgt, ssh_cmd, env, askpass, extra = _ssh_base_cmd(
            self._user, self._host, 20, self._ssh_password
        )
        try:
            rsync = [
                "rsync",
                "-az",
                "--exclude",
                "__pycache__",
                "-e",
                " ".join(shlex.quote(c) for c in ssh_cmd),
                str(src) + "/",
                dest,
            ]
            r = subprocess.run(
                rsync, capture_output=True, text=True, timeout=60, env=env, **extra
            )
            if r.returncode != 0:
                return False, "同步失败: " + ((r.stderr or r.stdout or "")[:400])
        except Exception as exc:
            return False, str(exc)
        finally:
            if askpass:
                try:
                    os.remove(askpass)
                except OSError:
                    pass
        remote_node_dir = f"{remote_package}/src/scout_long_nav_ros1"
        cloud_node = shlex.quote(f"{remote_node_dir}/cloud_to_costmap_node")
        save_node = shlex.quote(f"{remote_node_dir}/save_costmap_node")
        rc, out, err = self.ssh_ros(
            f"chmod +x {cloud_node} {save_node} 2>/dev/null; "
            f"cd {shlex.quote(workspace)} && catkin_make --pkg scout_long_nav_ros1",
            timeout=180,
        )
        text = (out or "") + (err or "")
        if rc != 0:
            return False, "车辆端编译 scout_long_nav_ros1 失败:\n" + text[-500:]
        return True, "ok"

    def start_trav_mapping(self) -> Tuple[bool, str]:
        if not self._connected:
            return False, "须先连接车辆。"
        rc, out, _ = self.ssh_ros("rostopic list", timeout=15)
        text = out or ""
        if "/rslidar_points" not in text:
            return False, "未检测到 /rslidar_points。须先启动雷达。"
        if "/odom" not in text:
            return False, "未检测到 /odom。须先启动底盘节点。"
        ok, msg = self._ensure_long_nav_ros1()
        if not ok:
            return False, msg
        ok, msg = self.start_roslaunch(
            "scout_long_nav_ros1",
            "long_nav_map.launch",
            "开始建图",
            require_unlock=False,
        )
        if not ok:
            return False, msg
        return True, (
            "已启动 traversability costmap 建图。\n"
            "算法：scout_long_nav。\n"
            "显示：车辆端 RViz，话题 /long_nav/costmap，Fixed Frame = odom。\n"
            "操作：遥控扫场。\n"
            "结束：结束建图并保存。输出：本机 maps_long。"
        )

    def end_trav_mapping(self, local_dir: str) -> Tuple[bool, str]:
        if not self._connected:
            return False, "须先连接车辆。"
        workspace, config_error = self._private_robot_workspace()
        if workspace is None:
            return False, config_error
        rc, out, err = self.ssh_ros(
            "rosservice call /save_costmap_node/save \"{}\"",
            timeout=25,
        )
        text = (out or "") + (err or "")
        dest = Path(local_dir)
        dest.mkdir(parents=True, exist_ok=True)
        remote = f"{workspace}/scout_nav_maps/arena_trav"
        copied = []
        for ext in (".yaml", ".pgm"):
            lp = str(dest / f"arena_trav{ext}")
            code, msg = run_scp(
                self._user,
                self._host,
                remote + ext,
                lp,
                timeout=40,
                password=self._ssh_password,
            )
            if code == 0:
                copied.append(lp)
        map_key = self._launch_key("scout_long_nav_ros1", "long_nav_map.launch")
        map_record = self._remote_launches.get(map_key)
        stop_note = ""
        if map_record is None:
            conflicts = self._remote_launch_conflicts(("long_nav_map.launch",))
            stop_note = (
                "未记录本应用启动的 long_nav_map；未终止任何车辆端进程。"
                + ("\n仍检测到：\n" + "\n".join(conflicts[:8]) if conflicts else "")
            )
        else:
            outcome, detail = self._stop_owned_remote_process(map_record)
            if outcome in ("stopped", "gone", "unowned"):
                self._remote_launches.pop(map_key, None)
            stop_note = f"long_nav_map 停止结果：{detail}"
        if not copied:
            return False, "保存或回传失败。车辆输出: " + text[-400:] + "\n" + stop_note
        return True, "costmap 已保存至\n" + "\n".join(copied) + "\n" + stop_note

    def start_camera(self) -> Tuple[bool, str]:
        chosen = None
        for pkg, lf in (
            ("robot_ros", "open_camera.launch"),
            ("realsense2_camera", "rs_camera.launch"),
        ):
            rc, out, _ = self.ssh_ros(
                f"rospack find {shlex.quote(pkg)} >/dev/null && echo OK", timeout=10
            )
            if rc == 0 and "OK" in (out or ""):
                chosen = (pkg, lf)
                break
        if chosen is None:
            return False, "未找到相机包（robot_ros / realsense2_camera）。"
        pkg, lf = chosen
        ok, msg = self.start_roslaunch(pkg, lf, "开相机", require_unlock=False)
        if not ok:
            return False, msg
        return True, (
            "已启动相机。\n"
            "设备：RealSense D435。\n"
            "显示：车辆端 rqt_image_view。\n"
            "话题：/camera/color/image_raw。"
        )

    def start_factory_mapping(self) -> Tuple[bool, str]:
        rc, out, _ = self.ssh_ros("rostopic list", timeout=15)
        text = out or ""
        if "/rslidar_points" not in text and "/scan" not in text:
            return False, "未检测到雷达话题。须先启动雷达。"
        ok, msg = self.start_factory_launch("gmapping.launch", "开建图")
        if not ok:
            return False, msg
        return True, (
            "已启动厂方 gmapping。\n"
            "显示：车辆端 RViz。\n"
            "图层：Map、LaserScan。\n"
            "操作：遥控扫场。"
        )

    def start_factory_nav(self) -> Tuple[bool, str]:
        ok, msg = self.start_factory_launch("navigation_4wd.launch", "开厂方导航")
        if not ok:
            return False, msg
        return True, (
            "已启动厂方 navigation_4wd。\n"
            "显示：车辆端 RViz。\n"
            "操作：2D Nav Goal。"
        )

    def stop_factory_stack(self) -> Tuple[bool, str]:
        if not self._connected:
            return False, "须先连接车辆。"
        specs = (
            ("scout_bringup", "navigation_4wd.launch"),
            ("scout_bringup", "gmapping.launch"),
            ("scout_bringup", "open_rslidar.launch"),
            ("robot_ros", "open_camera.launch"),
            ("realsense2_camera", "rs_camera.launch"),
        )
        records = [
            (self._launch_key(package, launch_file), package, launch_file)
            for package, launch_file in specs
            if self._launch_key(package, launch_file) in self._remote_launches
        ]
        if not records:
            conflicts = self._remote_launch_conflicts(tuple(launch for _, launch in specs))
            if conflicts:
                return False, (
                    "检测到未由本应用本次连接启动的厂方进程；为安全起见未终止：\n"
                    + "\n".join(conflicts[:12])
                )
            return False, "未记录本应用本次连接启动的厂方进程；未终止任何车辆端进程。"

        notes = []
        all_stopped = True
        for key, _package, launch_file in records:
            record = self._remote_launches[key]
            outcome, detail = self._stop_owned_remote_process(record)
            if outcome in ("stopped", "gone", "unowned"):
                self._remote_launches.pop(key, None)
            if outcome not in ("stopped", "gone"):
                all_stopped = False
            notes.append(f"{launch_file}: {detail}")

        message = "\n".join(notes)
        if all_stopped:
            self.log_message.emit("已停止本应用启动的厂方雷达、建图、导航、相机。底盘保持运行。")
            return True, "已停止本应用启动的厂方进程。\n" + message
        self.log_message.emit("部分厂方进程未停止：仅允许停止经本应用验证归属的 PID。")
        return False, "部分进程未停止（未知进程未触碰）：\n" + message

    def save_costmap(self, remote_dir: str) -> Tuple[bool, str]:
        if not self._connected:
            return False, "须先连接车辆。"
        rc, out, err = self.ssh_run(
            f"mkdir -p {shlex.quote(remote_dir)} && touch {shlex.quote(remote_dir)}/.scout_nav_writable && echo OK",
            timeout=10,
        )
        if rc != 0 or "OK" not in (out or ""):
            return False, f"车上路径不可写: {remote_dir}\n{(err or out)[:300]}"
        return True, f"路径可写：{remote_dir}。"

    def start_navigation(self, model_path: str) -> Tuple[bool, str]:
        if not self._guard_motion("开始导航"):
            return False, "车辆已锁定，须先解锁。" if self._locked else "须先连接车辆。"
        _ = model_path
        rc, out, _ = self.ssh_ros("rostopic list", timeout=15)
        if "/odom" not in (out or ""):
            return False, "未检测到 /odom。须先启动底盘节点。"
        return True, (
            "TD3 接口已就绪。\n"
            "速度输出：关闭。不向 /cmd_vel 发布。\n"
            "可视化：仿真模式 RViz。\n"
            "平面点到点：厂方导航，2D Nav Goal。"
        )

    def stop_navigation(self) -> None:
        self.log_message.emit("停止导航：本机禁止再发运动（不杀厂方节点，不 pub /cmd_vel）")
        self._enable_cmd = False

    def start_recording(self, include_lidar: bool = False) -> Tuple[bool, str]:
        if not self._connected:
            return False, "须先连接车辆。"
        if self._bag_remote_process is not None:
            state, detail = self._probe_remote_process(self._bag_remote_process)
            if state == "owned":
                return False, f"已在记录（本应用 pid={self._bag_remote_process.pid}）"
            if state == "unknown":
                return False, f"无法验证先前 rosbag；为安全起见不重复启动：{detail}"
            self._bag_remote_process = None
            if state == "unowned":
                return False, "先前 rosbag 已不再可证明归本应用所有；未终止它。"
        workspace, config_error = self._private_robot_workspace()
        if workspace is None:
            return False, config_error
        remote_bag_dir = f"{workspace}/scout_nav_bags"
        topic_s = " ".join(record_topics(include_lidar=include_lidar))
        remote = (
            f"mkdir -p {shlex.quote(remote_bag_dir)} && cd {shlex.quote(remote_bag_dir)} && "
            f"nohup rosbag record -O scout_$(date +%Y%m%d_%H%M%S) {topic_s} "
            "> /tmp/scout_nav_bag.log 2>&1 & echo $!"
        )
        rc, out, err = self.ssh_ros(remote, timeout=12)
        values = re.findall(r"^\s*(\d+)\s*$", out or "", re.MULTILINE)
        pid = values[-1] if values else ""
        if rc != 0 or not pid.isdigit():
            return False, f"rosbag 启动失败: {(err or out)[:300]}"
        start_time = self._remote_start_time(pid)
        if start_time is None:
            return False, (
                f"rosbag 已返回 pid={pid}，但无法记录可验证的进程身份；"
                "为安全起见，本应用不会尝试终止它。"
            )
        record = _RemoteProcessRecord(
            pid=pid,
            start_time=start_time,
            expected_terms=("rosbag", "record"),
            label="rosbag record",
        )
        state, detail = self._probe_remote_process(record)
        if state != "owned":
            return False, (
                f"rosbag 启动后无法验证进程归属（{detail}）；"
                "为安全起见，本应用不会尝试终止它。"
            )
        self._bag_remote_process = record
        return True, f"车上 rosbag pid={pid}；记录到车辆端私有目录。话题: {topic_s}"

    def stop_recording(self) -> Tuple[bool, str]:
        record = self._bag_remote_process
        if record is None:
            return False, "未记录本应用本次连接启动的 rosbag；未终止未知记录进程。"
        outcome, detail = self._stop_owned_remote_process(record)
        if outcome in ("stopped", "gone", "unowned"):
            self._bag_remote_process = None
        if outcome in ("stopped", "gone"):
            return True, "已停止本应用启动的记录（数据保留在车辆端私有目录）。"
        if outcome == "unowned":
            return False, f"rosbag 身份无法验证；未发送终止信号：{detail}"
        return False, f"无法安全停止本应用记录：{detail}"

    def shutdown(self) -> None:
        self._safety_timer.stop()
        self._bat_timer.stop()
        if self._locked:
            self._locked = False
            self.locked_changed.emit(False)
        self._connected = False
        self._emit_battery(False, None, None)
        self.log_message.emit("已断开（未向底盘发过速度）")
