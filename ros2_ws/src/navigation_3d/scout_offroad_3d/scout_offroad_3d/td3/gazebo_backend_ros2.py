"""ROS2 Gazebo Classic backend: services, spawn, world switch (Phase 4)."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time
from typing import TYPE_CHECKING, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState, ModelState
from gazebo_msgs.srv import (
    DeleteEntity,
    GetEntityState,
    SetEntityState,
    SpawnEntity,
)
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node
from std_srvs.srv import Empty

from scout_offroad_3d.td3.gazebo_ownership import (
    handoff_gzserver_identity,
    listener_pids_from_ss,
)

if TYPE_CHECKING:
    from scout_offroad_3d.td3.ros_env import ScoutGazeboEnv


def _gazebo_ros_script(name: str) -> str:
    for distro in ("humble", "noetic"):
        path = os.path.join("/opt/ros", distro, "lib", "gazebo_ros", name)
        if os.path.isfile(path):
            return path
    for candidate in (f"/usr/bin/{name}", f"/usr/local/bin/{name}"):
        if os.path.isfile(candidate):
            return candidate
    return os.path.join("/opt/ros/humble/lib/gazebo_ros", name)


def _resolve_package_mesh_uris(urdf_xml: str) -> str:
    """package:// → file:// so Gazebo Classic can load scout meshes after spawn."""
    import re

    def _repl(m):
        pkg, rel = m.group(1), m.group(2)
        try:
            share = get_package_share_directory(pkg)
        except Exception:
            return m.group(0)
        return 'filename="file://%s/%s"' % (share, rel)

    return re.sub(r'filename="package://([^/]+)/([^"]+)"', _repl, urdf_xml)

def _gzserver_ros_plugins() -> List[str]:
    """Match gazebo_scout.launch.py so spawn/state services come up after switch."""
    return [
        "-slibgazebo_ros_init.so",
        "-slibgazebo_ros_factory.so",
        "-slibgazebo_ros_force_system.so",
    ]


def _gazebo_classic_env() -> dict:
    """Sane GAZEBO_* paths (same idea as gazebo_scout.launch.py).

    Launch-started gzserver gets these via ExecuteProcess(additional_env=...).
    World-switch restarts must set them explicitly — bare Popen inherits a thin
    train-node env and gzclient then fails OpenGL/media → Force Quit / invisible robot.

    NVIDIA PRIME vars match gazebo_scout ``nvidia_gl`` (via SCOUT_NVIDIA_GL / auto).
    """
    env = os.environ.copy()
    prefix = env.get("GAZEBO_INSTALL_PREFIX", "/usr")
    base = os.path.join(prefix, "share", "gazebo-11")
    plugins = os.path.join(prefix, "lib", "x86_64-linux-gnu", "gazebo-11", "plugins")
    models = os.path.join(base, "models")
    # Preserve a caller-provided GAZEBO_MODEL_PATH below, but do not infer
    # external assets from a user-local cache during world switching.
    # Package-local models (uneven_ground_td3_1..5; _6 retired) for TD3 worlds
    try:
        pkg_models = os.path.join(
            get_package_share_directory("scout_offroad_3d"), "models"
        )
        if os.path.isdir(pkg_models):
            models = pkg_models + os.pathsep + models
    except Exception:
        src_models = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "models")
        )
        if os.path.isdir(src_models):
            models = src_models + os.pathsep + models

    def _merge(key: str, default: str) -> str:
        cur = (env.get(key) or "").strip()
        if cur in ("", ":"):
            return default
        if default and default not in cur.split(os.pathsep):
            return default + os.pathsep + cur
        return cur

    env["GAZEBO_MODEL_PATH"] = _merge("GAZEBO_MODEL_PATH", models)
    env["GAZEBO_PLUGIN_PATH"] = _merge("GAZEBO_PLUGIN_PATH", plugins)
    env["GAZEBO_RESOURCE_PATH"] = _merge("GAZEBO_RESOURCE_PATH", base)

    try:
        from scout_offroad_3d.gazebo_gl_env import merge_nvidia_gl_env

        merge_nvidia_gl_env(env)
    except Exception:
        pass
    return env


def _port_11345_status() -> Tuple[bool, str]:
    """Return whether the Gazebo master port is occupied, with diagnostics.

    This is observation only. A listener not represented by this lifecycle's
    ``Popen`` object is treated as another session and is never terminated.
    """
    details: List[str] = []
    try:
        out = subprocess.check_output(
            ["ss", "-lntp"], stderr=subprocess.DEVNULL
        ).decode("utf-8", "replace")
    except Exception as exc:
        # Do not start another Gazebo session when the port cannot be checked.
        return True, "unable to inspect port 11345: %s" % exc

    for line in out.splitlines():
        fields = line.split()
        if len(fields) >= 4 and fields[3].endswith(":11345"):
            details.append(line.strip())
    if details:
        return True, "\n".join(details)
    return False, ""


def _port_11345_busy() -> bool:
    return _port_11345_status()[0]


def _kill_proc_group(proc) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _ros2_node_list() -> List[str]:
    try:
        out = subprocess.check_output(
            ["ros2", "node", "list"],
            stderr=subprocess.DEVNULL,
            timeout=8,
        ).decode("utf-8", "replace")
        return [ln.strip() for ln in out.splitlines() if ln.strip()]
    except Exception:
        return []


def _ros2_node_alive(name: str) -> bool:
    want = name if name.startswith("/") else ("/" + name)
    for n in _ros2_node_list():
        if n == want or n.endswith(want):
            return True
    return False


def _wall_sleep(dt: float) -> None:
    if dt > 0:
        time.sleep(dt)


class ThrottleLogger:
    """Minimal throttle wrapper matching rospy.log*_throttle usage."""

    def __init__(self, logger):
        self._logger = logger
        self._last: dict = {}

    def debug(self, msg, *args):
        self._logger.debug(msg % args if args else msg)

    def info(self, msg, *args):
        self._logger.info(msg % args if args else msg)

    def warn(self, msg, *args):
        self._logger.warning(msg % args if args else msg)

    def err(self, msg, *args):
        self._logger.error(msg % args if args else msg)

    def fatal(self, msg, *args):
        self._logger.fatal(msg % args if args else msg)

    def _throttle(self, level: str, period: float, msg, *args):
        key = (level, msg if not args else (msg, args))
        now = time.time()
        last = self._last.get(key, 0.0)
        if now - last >= period:
            self._last[key] = now
            getattr(self, level)(msg, *args)

    def debug_throttle(self, period, msg, *args):
        self._throttle("debug", period, msg, *args)

    def info_throttle(self, period, msg, *args):
        self._throttle("info", period, msg, *args)

    def warn_throttle(self, period, msg, *args):
        self._throttle("warn", period, msg, *args)


class _ModelStateView:
    """Adapter: ros_env expects GetModelState-shaped responses."""

    __slots__ = ("success", "pose", "twist")

    def __init__(self, success: bool, pose, twist):
        self.success = success
        self.pose = pose
        self.twist = twist


class GazeboLifecycle:
    """Gazebo Classic lifecycle + service clients for ScoutGazeboEnv."""

    SPAWN_SVC = "/spawn_entity"
    DELETE_SVC = "/delete_entity"
    # ROS2 Humble: legacy /get_model_state removed; gazebo_ros_state plugin exposes:
    GET_STATE_SVC = "/get_entity_state"
    SET_STATE_SVC = "/set_entity_state"
    UNPAUSE_SVC = "/unpause_physics"

    def __init__(self, node: Node, env: "ScoutGazeboEnv"):
        self._node = node
        self._env = env
        self._log = ThrottleLogger(node.get_logger())
        self._gz_proc = None
        self._gzclient_proc = None
        self._owns_gz = False
        # The first server is a ros2-launch sibling, not a Popen child of this
        # node.  A launch-generated nonce makes that one explicit handoff safe.
        # All later servers remain ordinary Popen children in ``_gz_proc``.
        self._launch_owner_token = str(
            getattr(env, "gazebo_owner_token", "") or ""
        ).strip()
        self._handoff_identity: Optional[Tuple[int, str]] = None
        self._get_client = None
        self._set_client = None
        self._spawn_client = None
        self._delete_client = None
        self._unpause_client = None
        self.bind_model_services()

    def bind_model_services(self) -> None:
        """Create (or recreate) Gazebo service clients for Humble gazebo_ros."""
        if self._get_client is not None:
            self._node.destroy_client(self._get_client)
        if self._set_client is not None:
            self._node.destroy_client(self._set_client)
        if self._spawn_client is not None:
            self._node.destroy_client(self._spawn_client)
        if self._delete_client is not None:
            self._node.destroy_client(self._delete_client)
        if self._unpause_client is not None:
            self._node.destroy_client(self._unpause_client)
        self._get_client = self._node.create_client(GetEntityState, self.GET_STATE_SVC)
        self._set_client = self._node.create_client(SetEntityState, self.SET_STATE_SVC)
        self._spawn_client = self._node.create_client(SpawnEntity, self.SPAWN_SVC)
        self._delete_client = self._node.create_client(DeleteEntity, self.DELETE_SVC)
        self._unpause_client = self._node.create_client(Empty, self.UNPAUSE_SVC)

    def _wait_service(self, client, timeout: float, label: str) -> bool:
        deadline = time.monotonic() + timeout
        last_log = 0.0
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                return True
            rclpy.spin_once(self._node, timeout_sec=0.0)
            if not rclpy.ok():
                return False
            now = time.monotonic()
            if now - last_log >= 5.0:
                self._log.info(
                    "still waiting for %s (%.0fs / %.0fs)...",
                    label,
                    now - (deadline - timeout),
                    timeout,
                )
                last_log = now
        self._log.warn("timeout waiting for %s (%.1fs)", label, timeout)
        return False

    def wait_model_services(self, timeout: float = 60.0) -> None:
        self._log.info(
            "waiting for Gazebo entity-state services (%s, %s)...",
            self.GET_STATE_SVC,
            self.SET_STATE_SVC,
        )
        if not self._wait_service(self._get_client, timeout, self.GET_STATE_SVC):
            raise RuntimeError(
                "Gazebo %s unavailable — add gazebo_ros_state plugin to the .world file"
                % self.GET_STATE_SVC
            )
        if not self._wait_service(self._set_client, min(30.0, timeout), self.SET_STATE_SVC):
            raise RuntimeError(
                "Gazebo %s unavailable — add gazebo_ros_state plugin to the .world file"
                % self.SET_STATE_SVC
            )

    def get_model_state(self, model_name: str, relative_entity_name: str = "world"):
        if not self._get_client.service_is_ready():
            if not self._wait_service(self._get_client, 5.0, self.GET_STATE_SVC):
                raise RuntimeError("%s not ready" % self.GET_STATE_SVC)
        req = GetEntityState.Request()
        req.name = model_name
        req.reference_frame = relative_entity_name or "world"
        future = self._get_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("get_entity_state call failed")
        resp = future.result()
        st = resp.state
        return _ModelStateView(bool(resp.success), st.pose, st.twist)

    def set_model_state(self, state: ModelState) -> None:
        if not self._set_client.service_is_ready():
            if not self._wait_service(self._set_client, 5.0, self.SET_STATE_SVC):
                raise RuntimeError("%s not ready" % self.SET_STATE_SVC)
        entity = EntityState()
        entity.name = state.model_name
        entity.pose = state.pose
        entity.twist = state.twist
        entity.reference_frame = state.reference_frame or "world"
        req = SetEntityState.Request()
        req.state = entity
        future = self._set_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

    def _delete_model(self, model_name: str) -> None:
        if not self._wait_service(self._delete_client, 5.0, self.DELETE_SVC):
            return
        req = DeleteEntity.Request()
        req.name = model_name
        future = self._delete_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

    def _spawn_entity(self, model_name: str, urdf: str, pose: Pose) -> Tuple[bool, str]:
        if not self._wait_service(self._spawn_client, 60.0, self.SPAWN_SVC):
            return False, "spawn_entity unavailable"
        urdf = _resolve_package_mesh_uris(urdf)
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = urdf
        req.robot_namespace = "/"
        req.initial_pose = pose
        req.reference_frame = "world"
        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=60.0)
        if not future.done() or future.result() is None:
            return False, "spawn_entity call failed"
        resp = future.result()
        return bool(resp.success), str(resp.status_message)

    def _unpause_physics(self) -> None:
        if not self._wait_service(self._unpause_client, 10.0, self.UNPAUSE_SVC):
            self._log.warn("unpause_physics unavailable")
            return
        future = self._unpause_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)

    def _gzserver_clean_prefix(self) -> str:
        try:
            pkg = get_package_share_directory("scout_mini_pro_description")
            script = os.path.join(
                os.path.dirname(pkg),
                "lib",
                "scout_mini_pro_description",
                "gzserver_clean_prefix.sh",
            )
            if os.path.isfile(script):
                return script
            alt = os.path.join(
                get_package_share_directory("scout_mini_pro_description"),
                "..",
                "..",
                "src",
                "scout_mini_pro_description",
                "scripts",
                "gzserver_clean_prefix.sh",
            )
            alt = os.path.normpath(alt)
            if os.path.isfile(alt):
                return alt
        except Exception:
            pass
        here = os.path.dirname(os.path.abspath(__file__))
        fallback = os.path.normpath(
            os.path.join(here, "..", "..", "..", "..", "scout_mini_pro_description", "scripts", "gzserver_clean_prefix.sh")
        )
        return fallback if os.path.isfile(fallback) else ""

    def _find_handoff_gzserver(self) -> Optional[Tuple[int, str]]:
        """Locate the initial launch server only if its nonce proves ownership."""
        if not self._launch_owner_token:
            return None
        _busy, details = _port_11345_status()
        matches = []
        for pid in listener_pids_from_ss(details):
            identity = handoff_gzserver_identity(pid, self._launch_owner_token)
            if identity is not None:
                matches.append(identity)
        if len(matches) > 1:
            raise RuntimeError(
                "multiple port-11345 listeners present the active training "
                "handoff token; refusing ambiguous process stop"
            )
        if not matches:
            return None
        identity = matches[0]
        if self._handoff_identity is not None and identity != self._handoff_identity:
            raise RuntimeError(
                "Gazebo handoff PID identity changed before stop; refusing "
                "to signal a potentially reused process"
            )
        self._handoff_identity = identity
        return identity

    def _handoff_identity_still_matches(self, identity: Tuple[int, str]) -> bool:
        current = handoff_gzserver_identity(
            identity[0], self._launch_owner_token
        )
        return current == identity

    def _stop_handoff_gzserver(self) -> bool:
        """Stop only the token-proven first launch-owned gzserver.

        It is intentionally PID-directed rather than ``killpg``: the server's
        launch process group also contains unrelated RSP/JSP/training actions.
        The exact cmdline, token and start-time identity are checked again
        immediately before TERM and (only if needed) KILL.
        """
        identity = self._find_handoff_gzserver()
        if identity is None:
            return False
        pid, _start_ticks = identity
        if not self._handoff_identity_still_matches(identity):
            raise RuntimeError(
                "Gazebo handoff identity changed before TERM; refusing to stop it"
            )
        self._log.info(
            "[world] stopping token-owned initial launch gzserver pid=%s", pid
        )
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            self._handoff_identity = None
            return True
        except OSError as exc:
            raise RuntimeError(
                "cannot TERM token-owned initial gzserver pid=%s: %s" % (pid, exc)
            )

        # Give Gazebo a bounded grace period before an identity-rechecked KILL.
        grace = max(
            1.0,
            min(5.0, float(self._env.world_switch_kill_wait_sec) + 2.0),
        )
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if not self._handoff_identity_still_matches(identity):
                self._handoff_identity = None
                return True
            _wall_sleep(0.1)

        if self._handoff_identity_still_matches(identity):
            self._log.warn(
                "[world] token-owned initial gzserver pid=%s ignored TERM; sending KILL",
                pid,
            )
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except OSError as exc:
                raise RuntimeError(
                    "cannot KILL token-owned initial gzserver pid=%s: %s"
                    % (pid, exc)
                )
        self._handoff_identity = None
        return True

    def shutdown_owned(self) -> None:
        # ``_gz_proc`` / ``_gzclient_proc`` are always Popen children.  The
        # optional handoff server is not: it is stopped only through the nonce
        # proof above, never by name, port, or launch process group.
        for proc in (self._gzclient_proc, self._gz_proc):
            _kill_proc_group(proc)
        self._gz_proc = None
        self._gzclient_proc = None
        self._owns_gz = False
        self._stop_handoff_gzserver()

    def _wait_gazebo_unregistered(self, timeout: float = 15.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if not _ros2_node_alive("/gazebo"):
                _wall_sleep(0.4)
                if not _ros2_node_alive("/gazebo"):
                    return True
            _wall_sleep(0.4)
        still = _ros2_node_alive("/gazebo")
        self._log.warn(
            "[world] /gazebo still in node list after %.1fs (stale=%s)",
            timeout,
            still,
        )
        return not still

    def _kill_gazebo_processes(self) -> None:
        """Stop only Gazebo processes started by this lifecycle before reload."""
        self._log.info("[world] stopping lifecycle-owned gzserver/gzclient before reload...")
        self.shutdown_owned()
        _wall_sleep(self._env.world_switch_kill_wait_sec)

        for _ in range(80):
            if not _port_11345_busy():
                break
            _wall_sleep(0.1)

        port_busy, port_details = _port_11345_status()
        if port_busy:
            raise RuntimeError(
                "Gazebo master port 11345 is still in use after stopping only "
                "this lifecycle's owned child process(es). Refusing to stop an "
                "unowned Gazebo session. Stop its original launch, then retry. "
                "Listener details: %s" % (port_details or "unavailable")
            )

        ok = self._wait_gazebo_unregistered(timeout=20.0)
        if ok:
            self._log.info("[world] old /gazebo unregistered; port ready")
        else:
            self._log.warn(
                "[world] /gazebo still appears in discovery while port 11345 is free; "
                "no unowned process was stopped, so the next launch may report a "
                "name conflict"
            )

    def _gazebo_switch_log_path(self) -> str:
        try:
            from scout_workspace_common import workspace_path

            log_path = workspace_path(
                "3D",
                "debug_logs",
                "gazebo",
                "scout_offroad_td3_gzserver_switch.log",
                package_name="scout_offroad_3d",
                start_path=__file__,
            )
            log_path.parent.mkdir(parents=True, exist_ok=True)
            return str(log_path)
        except Exception as exc:
            # Never fall back to ~/.ros.  A missing workspace path should not
            # make a controller write into an unrelated checkout or home dir.
            self._log.warn("[world] workspace log path unavailable: %s", exc)
            return os.devnull

    def _start_gzserver(self, world_path: str) -> None:
        port_busy, port_details = _port_11345_status()
        if port_busy:
            raise RuntimeError(
                "refusing to start gzserver: port 11345 belongs to another "
                "session and will not be terminated here. Stop the owning launch "
                "and retry. Listener details: %s" % (port_details or "unavailable")
            )
        prefix = self._gzserver_clean_prefix()
        gzserver_bin = _gazebo_ros_script("gzserver")
        # Apply yaml sim params (physics / collision grid) to a cached copy.
        try:
            from scout_offroad_3d.world_sim import prepare_world

            cfg = {
                "sim_max_step_size": self._env._param("sim_max_step_size", 0.01),
                "sim_real_time_update_rate": self._env._param("sim_real_time_update_rate", 100),
                "sim_solver_iters": self._env._param("sim_solver_iters", 20),
                "sim_collision_heightmap_res": self._env._param("sim_collision_heightmap_res", 65),
            }
            patched = prepare_world(world_path, cfg)
            if patched != world_path:
                self._log.info("[world] sim params applied → %s", os.path.basename(patched))
            world_path = patched
        except Exception as e:
            self._log.warn("[world] sim patch skipped: %s", e)
        gz_args = [world_path] + _gzserver_ros_plugins()
        if prefix and os.path.isfile(prefix):
            cmd = ["bash", prefix, gzserver_bin] + gz_args
        else:
            cmd = [gzserver_bin] + gz_args
        self._log.info("[world] starting: %s", " ".join(cmd))
        log_path = self._gazebo_switch_log_path()
        try:
            log_f = open(log_path, "a")
            log_f.write(
                "\n--- %s ---\n%s\n"
                % (time.strftime("%Y-%m-%d %H:%M:%S"), " ".join(cmd))
            )
            log_f.flush()
        except Exception:
            log_f = subprocess.DEVNULL
        gz_env = _gazebo_classic_env()
        self._gz_proc = subprocess.Popen(
            cmd,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            env=gz_env,
            preexec_fn=os.setsid if hasattr(os, "setsid") else None,
        )
        self._owns_gz = True

    def _start_gzclient(self, force: bool = False) -> None:
        if not self._env._want_gui:
            return
        if force and self._gzclient_proc is not None:
            _kill_proc_group(self._gzclient_proc)
            self._gzclient_proc = None
        if self._gzclient_proc is not None and self._gzclient_proc.poll() is None:
            return
        gzclient_bin = "/usr/bin/gzclient"
        if not os.path.isfile(gzclient_bin):
            gzclient_bin = _gazebo_ros_script("gzclient")
        gz_env = _gazebo_classic_env()
        gz_env.setdefault("QT_X11_NO_MITSHM", "1")
        if not gz_env.get("DISPLAY"):
            self._log.warn(
                "[world] DISPLAY unset — gzclient will not show a window"
            )
        log_path = self._gazebo_switch_log_path()
        try:
            log_f = open(log_path, "a")
            log_f.write(
                "\n--- %s gzclient ---\n%s DISPLAY=%s\n"
                % (
                    time.strftime("%Y-%m-%d %H:%M:%S"),
                    gzclient_bin,
                    gz_env.get("DISPLAY", ""),
                )
            )
            log_f.flush()
        except Exception:
            log_f = subprocess.DEVNULL
        try:
            self._gzclient_proc = subprocess.Popen(
                [gzclient_bin],
                stdout=log_f,
                stderr=subprocess.STDOUT,
                env=gz_env,
                preexec_fn=os.setsid if hasattr(os, "setsid") else None,
            )
            self._owns_gz = True
            time.sleep(0.5)
            if self._gzclient_proc.poll() is not None:
                self._log.warn(
                    "[world] gzclient exited immediately (code=%s) — see %s",
                    self._gzclient_proc.returncode,
                    log_path,
                )
                self._gzclient_proc = None
                return
            self._log.info(
                "[world] gzclient started pid=%s DISPLAY=%s",
                self._gzclient_proc.pid,
                gz_env.get("DISPLAY", "?"),
            )
        except Exception as e:
            self._log.warn("[world] gzclient start failed: %s", e)

    def ensure_gzclient(self) -> None:
        """Start gzclient only if none is running (after world switch)."""
        if not self._env._want_gui:
            return
        if self._gzclient_proc is not None and self._gzclient_proc.poll() is not None:
            self._gzclient_proc = None
        self._start_gzclient(force=False)

    def pause_gzclient_for_eval(self) -> bool:
        """Stop gzclient during frozen-policy eval to save render cost.

        Returns True if a client was running and should be restored afterward.
        """
        if not getattr(self._env, "_want_gui", False):
            return False
        if self._gzclient_proc is None or self._gzclient_proc.poll() is not None:
            self._gzclient_proc = None
            self._log.info(
                "[viz_eval] no lifecycle-owned gzclient to pause; leaving any "
                "launch/external GUI untouched"
            )
            return False
        self._log.info("[viz_eval] pausing gzclient for eval")
        _kill_proc_group(self._gzclient_proc)
        self._gzclient_proc = None
        time.sleep(0.3)
        return True

    def restore_gzclient_after_eval(self, was_paused: bool) -> None:
        if not was_paused:
            return
        if not getattr(self._env, "_want_gui", False):
            return
        self._log.info("[viz_eval] restoring gzclient after eval")
        self._start_gzclient(force=True)

    def gz_child_ok(self) -> bool:
        if self._gz_proc is None:
            return True
        return self._gz_proc.poll() is None

    def _spawn_service_ready(self) -> bool:
        return self._spawn_client.wait_for_service(timeout_sec=0.15)

    def _wait_spawn_service(self, timeout: float = 60.0) -> None:
        svc = self.SPAWN_SVC
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < timeout:
            if not rclpy.ok():
                raise RuntimeError("ROS shutdown while waiting for %s" % svc)
            if not self.gz_child_ok():
                code = self._gz_proc.returncode if self._gz_proc else "?"
                raise RuntimeError(
                    "gzserver exited (code=%s) before %s — see %s"
                    % (code, svc, self._gazebo_switch_log_path())
                )
            elapsed = time.time() - t0
            if elapsed > 8.0 and self.gz_child_ok():
                raise RuntimeError(
                    "gzserver alive but %s missing after %.0fs — name conflict?"
                    % (svc, elapsed)
                )
            if self._spawn_service_ready():
                self._log.info("[world] %s ready (%.1fs)", svc, elapsed)
                return
            now = time.time()
            if now - last_log >= 5.0:
                self._log.info(
                    "[world] waiting for %s ... %.0fs/%.0fs (gz_alive=%s)",
                    svc,
                    now - t0,
                    timeout,
                    self.gz_child_ok(),
                )
                last_log = now
            _wall_sleep(0.2)
        raise RuntimeError(
            "timeout (%.0fs) waiting for %s (gz_alive=%s)"
            % (timeout, svc, self.gz_child_ok())
        )

    def _spawn_robot(self, world_path: Optional[str] = None) -> None:
        if not self._node.has_parameter("robot_description"):
            raise RuntimeError("robot_description param missing; cannot respawn")
        urdf = str(self._node.get_parameter("robot_description").value).strip()
        if not urdf:
            raise RuntimeError("robot_description is empty")
        if world_path:
            self._env._apply_world_identity(world_path)
        self._env.spawn_z = self._env._spawn_z_for_current_world()
        self._wait_spawn_service(timeout=60.0)
        try:
            self._delete_model(self._env.model_name)
            _wall_sleep(0.3)
        except Exception:
            pass

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = float(self._env.spawn_z)
        pose.orientation = Quaternion()
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        world_name = self._env._world_basename()
        self._log.info(
            "[world] spawning model %s at z=%.3f (world=%s) ...",
            self._env.model_name,
            self._env.spawn_z,
            world_name,
        )
        ok, msg = self._spawn_entity(self._env.model_name, urdf, pose)
        if not ok:
            raise RuntimeError("spawn_entity failed: %s" % msg)
        self._log.info("[world] spawn OK: %s", msg)
        try:
            self._unpause_physics()
        except Exception as e:
            self._log.warn("[world] unpause_physics: %s", e)

        if not self._env._place_robot_random_xy(context="spawn"):
            self._log.err(
                "[spawn] world spawn: no clear XY on %s (spawn_z=%.3f) — "
                "reset() will switch map, not abort",
                world_name,
                self._env.spawn_z,
            )

    def switch_world(self, world_path: str, timestep: Optional[int] = None) -> str:
        world_path = os.path.abspath(world_path)
        if not os.path.isfile(world_path):
            raise RuntimeError("world not found: %s" % world_path)

        self._env._stop()
        base = os.path.basename(world_path)
        previous_world = self._env.current_world
        self._env._world_before_switch = previous_world
        self._env._apply_world_identity(world_path)
        if timestep is None:
            self._log.info("[world] switch to %s", base)
        else:
            self._log.info("[world] switch to %s at t=%d", base, int(timestep))
        self._log.info(
            "[world] spawn_z for %s is %.3f (previous=%s)",
            base,
            self._env.spawn_z,
            os.path.basename(previous_world) if previous_world else "(none)",
        )

        last_err = None
        max_retries = self._env.world_switch_max_retries
        for attempt in range(1, max_retries + 1):
            try:
                self._log.info(
                    "[world] reload attempt %d/%d for %s", attempt, max_retries, base
                )
                self._kill_gazebo_processes()
                self._start_gzserver(world_path)

                t0 = time.time()
                boot_limit = self._env.world_switch_gzserver_boot_sec
                while time.time() - t0 < boot_limit:
                    if not self.gz_child_ok():
                        code = self._gz_proc.returncode if self._gz_proc else "?"
                        raise RuntimeError(
                            "gzserver exited early (code=%s) loading %s"
                            % (code, base)
                        )
                    if self._spawn_service_ready():
                        self._log.info(
                            "[world] gzserver boot OK (%.1fs); spawn service up",
                            time.time() - t0,
                        )
                        break
                    _wall_sleep(0.15)
                else:
                    self._log.warn(
                        "[world] boot window %.1fs without spawn service",
                        boot_limit,
                    )

                self._wait_spawn_service(timeout=self._env.world_switch_spawn_wait_sec)
                # Restart GUI after every switch: old gzclient died with gzserver.
                self._start_gzclient(force=True)
                self.bind_model_services()
                self.wait_model_services(timeout=30.0)
                self._spawn_robot(world_path)
                self._env._reset_imu_state()
                self._env._ensure_imu_subscriber()
                _wall_sleep(0.6)

                self._env._wait_pose(timeout=20.0)
                self._env._wait_imu_after_respawn()
                self._env._post_switch_model_only_steps = (
                    self._env.imu_model_fallback_after_switch_steps
                )
                self._env._log_imu_post_switch_status()

                try:
                    self._env._set_global_param("offroad.current_world", world_path)
                except Exception:
                    pass
                import numpy as np

                self._env._features = np.full(
                    self._env.n_sectors, self._env.max_range, dtype=np.float32
                )
                self._env._td3_features = np.full(
                    self._env._td3_feature_count,
                    self._env.max_range,
                    dtype=np.float32,
                )
                self._env._odom_pose = None
                self._log.info("[world] switch complete: %s (scout_mini ready)", base)
                return world_path
            except Exception as e:
                last_err = e
                self._log.err(
                    "[world] reload attempt %d/%d failed: %s",
                    attempt,
                    max_retries,
                    e,
                )
                try:
                    self._kill_gazebo_processes()
                except Exception:
                    pass
                _wall_sleep(self._env.world_switch_retry_sleep_sec)

        raise RuntimeError(
            "world switch failed after %d attempts (%s): %s"
            % (max_retries, base, last_err)
        )
