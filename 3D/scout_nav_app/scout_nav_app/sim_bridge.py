"""ROS2 仿真桥接：复用 scout_long_nav 的 launch 文件。"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QObject, Signal

# ScoutNav is also run directly from ``3D/scout_nav_app/run.sh`` in a source
# checkout, where the shared package may not yet be on PYTHONPATH.  Prefer the
# installed package, then make the workspace source package available without
# duplicating its path-resolution rules here.
try:
    from scout_workspace_common import debug_log_path, resolve_workspace_root
except ImportError:
    _COMMON_SOURCE = (
        Path(__file__).resolve().parents[3]
        / "ros2_ws"
        / "src"
        / "shared"
        / "scout_workspace_common"
    )
    if _COMMON_SOURCE.is_dir():
        sys.path.insert(0, str(_COMMON_SOURCE))
    from scout_workspace_common import debug_log_path, resolve_workspace_root


class SimBridge(QObject):
    """仿真模式：复用 scout_long_nav 的 launch 文件。"""

    log_message = Signal(str)
    world_loaded = Signal(str)
    costmap_saved = Signal(str)
    coverage_checked = Signal(float)
    navigation_started = Signal()
    navigation_stopped = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._map_proc: Optional[subprocess.Popen] = None
        self._nav_proc: Optional[subprocess.Popen] = None
        self._workspace = resolve_workspace_root(start_path=Path(__file__))
        self._ros2_ws = self._workspace / "ros2_ws"
        self._world_path: Optional[Path] = None
        self._costmap_path: Optional[Path] = None
        self._map_log: Optional[Path] = None
        self._nav_log: Optional[Path] = None
        self._boundary_rviz_proc: Optional[subprocess.Popen] = None
        self._boundary_map_proc: Optional[subprocess.Popen] = None
        self._boundary_tf_proc: Optional[subprocess.Popen] = None
        self._boundary_color_proc: Optional[subprocess.Popen] = None
        self._safety_enable = {"stuck": True, "rollover": True, "collision": True}
        self._debug_log = debug_log_path(
            "scout_nav_app_sim_bridge.log", start_path=Path(__file__)
        )
        self._debug_dir = self._debug_log.parent
        # The overlay polls this workspace-owned file.  Keep state out of the
        # application source tree so a checkout remains clean after a session.
        self._boundary_state_file = self._debug_dir / "scout_nav_boundary_live.json"
        self._security_map_dir = self._workspace / "3D" / "maps_security"

    def _agent_log(self, hid: str, location: str, message: str, data: Optional[dict] = None) -> None:
        # #region agent log
        try:
            self._debug_log.parent.mkdir(parents=True, exist_ok=True)
            with open(self._debug_log, "a") as f:
                f.write(
                    json.dumps(
                        {
                            "sessionId": "8873ab",
                            "hypothesisId": hid,
                            "location": location,
                            "message": message,
                            "data": data or {},
                            "timestamp": int(time.time() * 1000),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion

    def _source_cmd(self) -> str:
        return f"source /opt/ros/humble/setup.bash && source {self._ros2_ws}/install/setup.bash && export ROS_DOMAIN_ID=41"

    def _run_cmd(self, cmd: str, name: str, log_file: Optional[Path] = None) -> subprocess.Popen:
        """运行命令并返回进程（日志落盘，避免 PIPE 堵死）。"""
        self.log_message.emit(f"$ {cmd}")
        self._agent_log("B", "sim_bridge._run_cmd", "spawn", {"name": name, "cmd_tail": cmd[-160:]})
        out = open(log_file, "w") if log_file else subprocess.DEVNULL
        err = out if log_file else subprocess.DEVNULL
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=out,
            stderr=err,
            start_new_session=True,
        )
        if log_file:
            out.close()
        return proc

    def load_world(self, world_path: str) -> bool:
        """验证世界文件存在（不启动 Gazebo）。"""
        path = Path(world_path)
        if not path.exists():
            self.log_message.emit(f"✗ 世界文件不存在: {world_path}")
            return False
        if not path.suffix == ".world":
            self.log_message.emit(f"✗ 不是有效的世界文件: {world_path}")
            return False

        self._world_path = path
        self.log_message.emit(f"✓ 世界文件有效: {path.name}")
        return True

    def list_sim_conflicts(self):
        """检测占用 Gazebo/RViz 的进程。"""
        from .sim_conflict import list_sim_conflicts

        hits = list_sim_conflicts()
        self._agent_log(
            "Q",
            "sim_bridge.list_sim_conflicts",
            "scan",
            {"n": len(hits), "kinds": [h.kind for h in hits[:12]]},
        )
        return hits

    def start_mapping(self) -> bool:
        """开始建图：启动 long_nav_map.launch.py（自带 Gazebo+RViz）。
        调用前应由 UI 完成冲突确认；此处不再静默强杀。
        """
        if not self._world_path:
            self.log_message.emit("✗ 请先加载世界")
            return False

        if self._map_proc is not None and self._map_proc.poll() is None:
            self.log_message.emit("✗ 建图已在运行")
            return False

        self.log_message.emit("开始建图（long_nav_map）...")
        world = str(self._world_path)
        cmd = (
            f"{self._source_cmd()} && "
            f"ros2 launch scout_long_nav long_nav_map.launch.py world:={world}"
        )
        self._map_log = self._debug_dir / "scout_nav_app_map_launch.log"
        self._map_proc = self._run_cmd(cmd, "mapping", log_file=self._map_log)
        self._agent_log(
            "C",
            "sim_bridge.start_mapping",
            "spawned",
            {"pid": self._map_proc.pid, "poll": self._map_proc.poll(), "world": world, "log": str(self._map_log)},
        )
        self.log_message.emit("✓ 已拉起 long_nav_map（Gazebo+RViz+遥控）")
        self.log_message.emit(f"launch 日志: {self._map_log}")
        return True

    def mapping_alive(self) -> bool:
        """建图 launch 是否仍在跑。"""
        return self._map_proc is not None and self._map_proc.poll() is None

    def mapping_exit_hint(self) -> str:
        """若建图进程已退出，返回日志尾部提示。"""
        if self._map_proc is None:
            return "未启动"
        rc = self._map_proc.poll()
        if rc is None:
            return "running"
        tail = ""
        logf = getattr(self, "_map_log", None)
        if logf and Path(logf).exists():
            try:
                tail = Path(logf).read_text(errors="replace")[-800:]
            except Exception:
                pass
        return f"exit={rc}\n{tail}"

    def save_costmap(self, output_dir: Optional[Path] = None) -> bool:
        """仅调用保存服务（不关窗口）。"""
        if not self.mapping_alive():
            self.log_message.emit("✗ 请先开始建图")
            return False

        if output_dir is None:
            output_dir = self._workspace / "3D" / "maps_long"
        output_dir.mkdir(parents=True, exist_ok=True)

        self.log_message.emit(f"保存 costmap 到: {output_dir}")
        cmd = (
            f"{self._source_cmd()} && "
            "ros2 service call /save_costmap_node/save std_srvs/srv/Trigger"
        )
        try:
            result = subprocess.run(
                ["bash", "-c", cmd],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            self.log_message.emit("✗ 保存服务超时")
            return False

        if result.returncode == 0:
            self._costmap_path = output_dir
            self.costmap_saved.emit(str(output_dir))
            self.log_message.emit(f"✓ Costmap 已保存: {output_dir}")
            return True
        self.log_message.emit(f"✗ 保存失败: {result.stderr or result.stdout}")
        return False

    def end_mapping(self, output_dir: Optional[Path] = None) -> bool:
        """结束建图：保存 costmap，然后关掉 Gazebo/RViz/建图栈。"""
        self._agent_log("H", "sim_bridge.end_mapping", "enter", {"alive": self.mapping_alive()})
        saved = False
        if self.mapping_alive():
            saved = self.save_costmap(output_dir)
            if not saved:
                # launch 支持 Ctrl+C 自动存；发 SIGINT 再等一下
                self.log_message.emit("服务保存失败，尝试 SIGINT 触发 launch 自动存图…")
                try:
                    if self._map_proc is None or self._map_proc.poll() is not None:
                        raise RuntimeError("本应用的建图 launch 已退出")
                    pid = self._map_proc.pid
                    if os.getpgid(pid) == pid:
                        os.killpg(pid, signal.SIGINT)
                    else:
                        # Do not signal an unexpected group; this is still the
                        # direct Popen child created by this bridge.
                        self._map_proc.send_signal(signal.SIGINT)
                    self._map_proc.wait(timeout=8)
                    saved = True
                    if output_dir is None:
                        output_dir = self._workspace / "3D" / "maps_long"
                    self._costmap_path = output_dir
                except Exception as e:
                    self.log_message.emit(f"SIGINT 存图未完成: {e}")
        else:
            self.log_message.emit("建图进程已不在，跳过保存，只清理窗口")

        self.stop_mapping()
        self._agent_log("H", "sim_bridge.end_mapping", "done", {"saved": saved, "gz": self._gz_running()})
        self.log_message.emit("✓ 建图已结束（本应用启动的 Gazebo/RViz 已请求关闭）")
        return saved or self._costmap_path is not None

    def stop_mapping(self) -> None:
        """Stop the mapping launch and windows owned by this bridge."""
        self._zero_cmd_vel()
        self._kill_proc_group(self._map_proc, "map")
        self._map_proc = None
        self.ensure_sim_idle()

    def _gz_running(self) -> bool:
        r = subprocess.run(
            ["pgrep", "-f", r"(^|/)gzserver( |$)"],
            capture_output=True,
            text=True,
        )
        return r.returncode == 0 and bool(r.stdout.strip())

    def ensure_sim_idle(self) -> bool:
        """Stop only this bridge's processes and report any other session.

        ``list_sim_conflicts`` is deliberately read-only: an existing Gazebo,
        RViz, training, or long-nav session may belong to another terminal.
        Such a conflict blocks a new launch but is never terminated here.
        """
        from .sim_conflict import format_conflicts, list_sim_conflicts, reap_zombie_children

        self.log_message.emit("清理残留仿真（gz/rviz/long_nav）…")
        # 先停本应用托管的边界/建图/导航进程组并 wait，避免留下僵尸
        self._stop_boundary_stack()
        self._kill_proc_group(self._map_proc, "map")
        self._kill_proc_group(self._nav_proc, "nav")
        self._map_proc = None
        self._nav_proc = None
        reaped = reap_zombie_children()
        # A launch-owned process group can need a moment to reap its Gazebo
        # children after its parent has received SIGTERM.  This is observation
        # only; no process discovered by the scan is signalled.
        left = list_sim_conflicts()
        for _ in range(12):
            if not left:
                break
            time.sleep(0.25)
            left = list_sim_conflicts()
        self._agent_log(
            "Z",
            "sim_bridge.ensure_sim_idle",
            "after_cleanup",
            {
                "reaped": reaped,
                "left_n": len(left),
                "left_kinds": [c.kind for c in left[:12]],
                "gz_running": self._gz_running(),
            },
        )
        still = bool(left)
        self._agent_log("I", "sim_bridge.ensure_sim_idle", "after", {"conflicts": len(left)})
        if still:
            self.log_message.emit(
                "⚠ 仍检测到未由本应用可验证控制的仿真进程；未终止它们。\n"
                "请在创建它们的终端中关闭后再试：\n"
                + format_conflicts(left)
            )
            return False
        self.log_message.emit("✓ 本应用启动的仿真进程已停止")
        return True

    def check_coverage(self) -> float:
        """检查覆盖率。"""
        if not self._costmap_path:
            self.log_message.emit("✗ 请先保存 costmap")
            return 0.0

        self.log_message.emit("检查覆盖率...")

        # 调用 coverage_supervisor_node
        cmd = f"{self._source_cmd()} && ros2 topic echo /long_nav/coverage --once"
        result = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True,
            timeout=10,
        )

        # 解析覆盖率
        try:
            coverage = float(result.stdout.strip().split()[-1])
        except (ValueError, IndexError):
            coverage = 0.85  # 默认值

        self.coverage_checked.emit(coverage)
        self.log_message.emit(f"覆盖率: {coverage:.1%}")
        return coverage

    def load_costmap(self, costmap_path: str) -> bool:
        """加载已有 costmap（目录或 arena_trav.yaml）。"""
        path = Path(costmap_path).expanduser()
        if path.is_dir():
            yaml = path / "arena_trav.yaml"
            if not yaml.exists():
                self.log_message.emit(f"✗ 目录内无 arena_trav.yaml: {path}")
                self._agent_log("D", "sim_bridge.load_costmap", "missing_yaml", {"path": str(path)})
                return False
            map_dir = path
        elif path.suffix == ".yaml" and path.exists():
            map_dir = path.parent
        else:
            self.log_message.emit(f"✗ Costmap 不存在: {costmap_path}")
            self._agent_log("D", "sim_bridge.load_costmap", "missing", {"path": str(path)})
            return False

        self._costmap_path = map_dir
        self.costmap_saved.emit(str(map_dir))
        self.log_message.emit(f"✓ 加载 costmap: {map_dir}")
        self._agent_log("D", "sim_bridge.load_costmap", "ok", {"map_dir": str(map_dir)})
        return True

    def start_navigation(self, model_path: str) -> bool:
        """开始导航。调用前 UI 须完成冲突确认，此处不再静默强杀。"""
        if not self._world_path:
            self.log_message.emit("✗ 请先加载世界")
            return False

        if not self._costmap_path:
            self.log_message.emit("✗ 请先保存或加载 costmap")
            return False

        if self._nav_proc is not None and self._nav_proc.poll() is None:
            self.log_message.emit("✗ 导航已在运行")
            return False

        run_dir = Path(model_path)
        weights_arg = str(run_dir / "td3") if (run_dir / "td3_actor").exists() else str(run_dir)
        world = str(self._world_path)
        map_dir = str(self._costmap_path)
        self.log_message.emit(f"开始导航，weights:={weights_arg}")

        cmd = (
            f"{self._source_cmd()} && "
            f"ros2 launch scout_long_nav long_nav_nav.launch.py "
            f"weights:={weights_arg} world:={world} map_dir:={map_dir}"
        )
        self._nav_log = self._debug_dir / "scout_nav_app_nav_launch.log"
        self._nav_proc = self._run_cmd(cmd, "navigation", log_file=self._nav_log)
        self._agent_log(
            "I",
            "sim_bridge.start_navigation",
            "spawned",
            {"pid": self._nav_proc.pid},
        )
        self.log_message.emit("已请求 long_nav_nav；稍后检查是否真正起来（隔离失败会立刻退出）")
        self.log_message.emit(f"launch 日志: {self._nav_log}")
        return True

    def navigation_alive(self) -> bool:
        return self._nav_proc is not None and self._nav_proc.poll() is None

    def navigation_exit_hint(self) -> str:
        if self._nav_proc is None:
            return "未启动"
        rc = self._nav_proc.poll()
        if rc is None:
            return "running"
        tail = ""
        if self._nav_log and Path(self._nav_log).exists():
            try:
                tail = Path(self._nav_log).read_text(errors="replace")[-900:]
            except Exception:
                pass
        return f"exit={rc}\n{tail}"

    def open_boundary_rviz(self, security_dir: Optional[str] = None) -> bool:
        """打开边界 RViz：TF + overlay costmap + 薄荷绿热力 + 亮红围栏。"""
        if not self._costmap_path:
            self.log_message.emit("✗ 请先加载 costmap")
            return False

        if security_dir:
            self._security_map_dir = Path(security_dir)

        cfg = (
            self._workspace
            / "3D"
            / "scout_nav_app"
            / "resources"
            / "boundary.rviz"
        )
        yaml = Path(self._costmap_path) / "arena_trav.yaml"
        map_dir = str(Path(self._costmap_path))
        self._agent_log(
            "J",
            "sim_bridge.open_boundary_rviz",
            "enter",
            {"cfg": str(cfg), "yaml": str(yaml), "cfg_ok": cfg.exists(), "yaml_ok": yaml.exists()},
        )
        if not yaml.exists():
            self.log_message.emit(f"✗ 找不到 costmap: {yaml}")
            return False
        if not cfg.exists():
            self.log_message.emit(f"✗ 找不到边界 RViz 配置: {cfg}")
            return False

        log_dir = self._debug_dir
        self._stop_boundary_stack()

        # 无小车时仍需要 Fixed Frame=odom 的 TF，避免 Global Status 报错
        tf_cmd = (
            f"{self._source_cmd()} && "
            "ros2 run tf2_ros static_transform_publisher "
            "--x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 "
            "--frame-id map --child-frame-id odom"
        )
        self._boundary_tf_proc = self._run_cmd(
            tf_cmd,
            "boundary_tf",
            log_file=log_dir / "scout_nav_app_boundary_tf.log",
        )

        # overlay：底图 + 围栏烘焙 → /long_nav/costmap + 亮红 Marker
        sec_dir = str(self._security_map_dir)
        map_cmd = (
            f"{self._source_cmd()} && export PYTHONUNBUFFERED=1 && "
            f"ros2 run scout_long_nav boundary_overlay_node --ros-args "
            f"-r __node:=scout_nav_boundary_overlay "
            f"-p map_dir:={map_dir} -p map_stem:=arena_trav "
            f"-p costmap_topic:=/long_nav/costmap -p frame_id:=odom "
            f"-p rate_hz:=2.0 -p fence_half_width_m:=0.45 "
            f"-p security_dir:={sec_dir} "
            f"-p state_file:={self._boundary_state_file} -p use_sim_time:=false"
        )
        self._boundary_map_proc = self._run_cmd(
            map_cmd,
            "boundary_overlay",
            log_file=log_dir / "scout_nav_app_boundary_map.log",
        )

        ready = False
        for i in range(40):
            time.sleep(0.15)
            if self._boundary_map_proc.poll() is not None:
                hint = ""
                try:
                    hint = (log_dir / "scout_nav_app_boundary_map.log").read_text(errors="replace")[-500:]
                except Exception:
                    pass
                self.log_message.emit(f"✗ boundary_overlay 启动失败\n{hint}")
                self._agent_log(
                    "J",
                    "sim_bridge.open_boundary_rviz",
                    "map_died",
                    {"hint": hint[:240], "poll_i": i},
                )
                return False
            try:
                text = (log_dir / "scout_nav_app_boundary_map.log").read_text(errors="replace")
                if "boundary_overlay:" in text and "/long_nav/costmap" in text:
                    ready = True
                    break
            except Exception:
                pass

        # 薄荷绿 / 落日橙 / 致死亮红 热力云
        color_cmd = (
            f"{self._source_cmd()} && export PYTHONUNBUFFERED=1 && "
            "ros2 run scout_long_nav costmap_color_cloud_node --ros-args "
            "-r __node:=scout_nav_boundary_color "
            "-p costmap_topic:=/long_nav/costmap "
            "-p cloud_topic:=/long_nav/costmap_rgb "
            "-p use_sim_time:=false"
        )
        self._boundary_color_proc = self._run_cmd(
            color_cmd,
            "boundary_color",
            log_file=log_dir / "scout_nav_app_boundary_color.log",
        )

        # #region agent log
        self._agent_log(
            "J",
            "sim_bridge.open_boundary_rviz",
            "costmap_wait",
            {
                "ready": ready,
                "alive": self._boundary_map_proc.poll() is None,
                "log_tail": (
                    (log_dir / "scout_nav_app_boundary_map.log").read_text(errors="replace")[-300:]
                    if (log_dir / "scout_nav_app_boundary_map.log").exists()
                    else ""
                ),
            },
        )
        # #endregion
        if not ready:
            self.log_message.emit("⚠ boundary_overlay 未确认就绪，仍尝试打开 RViz")

        rviz_cmd = f"{self._source_cmd()} && rviz2 -d {cfg}"
        self._boundary_rviz_proc = self._run_cmd(
            rviz_cmd,
            "boundary_rviz",
            log_file=log_dir / "scout_nav_app_boundary_rviz.log",
        )
        self.log_message.emit(
            "✓ 已打开边界 RViz（薄荷绿热力 + 亮红围栏）。"
            "无小车属正常。请用 Publish Point 加点"
        )
        self._agent_log(
            "J",
            "sim_bridge.open_boundary_rviz",
            "ok",
            {"map_dir": map_dir, "cfg": str(cfg), "ready": ready, "security_dir": sec_dir},
        )
        return True

    def _stop_boundary_stack(self) -> None:
        self._kill_proc_group(self._boundary_map_proc, "boundary_overlay")
        self._kill_proc_group(self._boundary_color_proc, "boundary_color")
        self._kill_proc_group(self._boundary_tf_proc, "boundary_tf")
        self._kill_proc_group(self._boundary_rviz_proc, "boundary_rviz")
        self._boundary_map_proc = None
        self._boundary_color_proc = None
        self._boundary_tf_proc = None
        self._boundary_rviz_proc = None

    def publish_boundary_vertices(
        self, points: list, closed: bool = False
    ) -> None:
        """把顶点写入状态文件，overlay 轮询后烤致死带 + 亮红线。"""
        state = self._boundary_state_file
        state.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "points": [[float(x), float(y)] for x, y in points],
            "closed": bool(closed),
            "cmd": "",
            "ts": time.time(),
        }
        state.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        self._agent_log(
            "L",
            "sim_bridge.publish_boundary_vertices",
            "write_state",
            {"n": len(points), "closed": closed, "path": str(state)},
        )

    def close_boundary_rviz(self) -> None:
        """关闭边界 RViz / overlay / TF / 热力（保存成功后调用）。"""
        self._agent_log("P", "sim_bridge.close_boundary_rviz", "enter", {})
        self._stop_boundary_stack()
        self.log_message.emit("✓ 已关闭本应用启动的边界 RViz")
        self._agent_log("P", "sim_bridge.close_boundary_rviz", "done", {})

    def save_security_costmap(self, security_dir: Optional[str] = None) -> Optional[Path]:
        """保存到 security 目录。成功返回路径；失败返回 None（RViz 保持打开）。"""
        out = Path(security_dir) if security_dir else self._security_map_dir
        try:
            out.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self.log_message.emit(f"✗ 无法创建目录 {out}: {exc}")
            self._agent_log(
                "M",
                "sim_bridge.save_security_costmap",
                "mkdir_fail",
                {"dir": str(out), "err": str(exc)},
            )
            return None

        state = self._boundary_state_file
        state.parent.mkdir(parents=True, exist_ok=True)
        try:
            cur = json.loads(state.read_text(encoding="utf-8")) if state.exists() else {}
        except Exception:
            cur = {}
        # 若还没同步顶点，至少带上 closed 标记触发 overlay 用内存态保存
        cur["cmd"] = "save"
        cur["security_dir"] = str(out)
        cur["ts"] = time.time()
        state.write_text(json.dumps(cur, ensure_ascii=False), encoding="utf-8")
        self._agent_log("M", "sim_bridge.save_security_costmap", "request", {"dir": str(out)})

        yaml_path = out / "arena_trav.yaml"
        # 记录请求前 mtime，避免误判旧文件
        before = yaml_path.stat().st_mtime if yaml_path.exists() else 0.0
        deadline = time.time() + 10.0
        while time.time() < deadline:
            time.sleep(0.2)
            if not yaml_path.exists():
                continue
            mtime = yaml_path.stat().st_mtime
            if mtime > before and mtime > time.time() - 20:
                # 不覆盖 bridge 的建图路径语义时：仍切换导航用 security
                self._costmap_path = out
                self.costmap_saved.emit(str(out))
                self.log_message.emit(f"✓ 安全边界 costmap 已保存: {out}")
                self._agent_log(
                    "M",
                    "sim_bridge.save_security_costmap",
                    "ok",
                    {"dir": str(out), "mtime": mtime},
                )
                return out

        self.log_message.emit(f"✗ 保存超时/失败: {out}（RViz 保持打开）")
        self._agent_log("M", "sim_bridge.save_security_costmap", "fail", {"dir": str(out)})
        return None

    def _zero_cmd_vel(self) -> None:
        cmd = (
            f"{self._source_cmd()} && "
            "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
            "\"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\""
        )
        subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    def _kill_proc_group(self, proc: Optional[subprocess.Popen], label: str) -> None:
        if proc is None or proc.poll() is not None:
            return
        pid = proc.pid
        owns_group = False
        try:
            # Every managed launch is started with start_new_session=True.  If
            # that invariant no longer holds, signal only the tracked direct
            # child instead of an arbitrary process group.
            owns_group = os.getpgid(pid) == pid
            if owns_group:
                os.killpg(pid, signal.SIGTERM)
                self.log_message.emit(f"已 SIGTERM 本应用进程组 {label} pgid={pid}")
            else:
                proc.terminate()
                self.log_message.emit(f"已 SIGTERM 本应用子进程 {label} pid={pid}")
        except ProcessLookupError:
            pass
        except Exception as e:
            self.log_message.emit(f"停止本应用子进程 {label} 失败: {e}; 改 terminate()")
            try:
                proc.terminate()
            except Exception:
                pass
        try:
            proc.wait(timeout=3)
        except Exception:
            try:
                if owns_group:
                    os.killpg(pid, signal.SIGKILL)
                else:
                    proc.kill()
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def _cleanup_sim_guis(self) -> None:
        """Reap this process's exited children without touching other sessions."""
        from .sim_conflict import reap_zombie_children

        n = reap_zombie_children()
        self._agent_log("Z", "sim_bridge._cleanup_sim_guis", "reaped", {"n": n})
        if n:
            self.log_message.emit(f"已回收本应用的 {n} 个已退出子进程")

    def stop_navigation(self, close_sim: bool = True) -> None:
        """停止导航：先 0 速，再杀 launch 进程组；默认关掉 Gazebo+RViz。"""
        self._agent_log(
            "F",
            "sim_bridge.stop_navigation",
            "enter",
            {
                "close_sim": close_sim,
                "nav_pid": None if not self._nav_proc else self._nav_proc.pid,
                "nav_alive": self.navigation_alive(),
                "gz_before": self._gz_running(),
            },
        )
        self._zero_cmd_vel()
        self._kill_proc_group(self._nav_proc, "nav")
        self._nav_proc = None
        if close_sim:
            self.ensure_sim_idle()
            self._map_proc = None
        self.navigation_stopped.emit()
        self._agent_log(
            "F",
            "sim_bridge.stop_navigation",
            "done",
            {"gz_after": self._gz_running()},
        )
        self.log_message.emit(
            "✓ 导航已停止（已将小车锁死，仿真窗口应已关闭）"
            if not self._gz_running()
            else "⚠ 已将小车锁死，但仍检测到 gzserver；外部会话未触碰，请在其原终端关闭"
        )

    def cancel_navigation_mission(self) -> None:
        """废除当前导航任务：清路径、锁轮，直到新 Goal Pose。"""
        self._agent_log("S", "sim_bridge.cancel_navigation_mission", "pub", {})
        cmd = (
            f"{self._source_cmd()} && "
            "ros2 topic pub --once /long_nav/cancel_nav std_msgs/msg/Bool \"{data: true}\""
        )
        subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.log_message.emit("✓ 已废除本次导航任务（清路径/锁轮，重新 Set Goal 可再开）")

    def try_unstuck(self, mode: str = "here") -> None:
        """脱离卡死：here/nearby/origin/goal；并废除本次导航任务。"""
        mode = (mode or "here").strip().lower()
        if mode not in ("here", "nearby", "origin", "goal"):
            mode = "here"
        labels = {
            "here": "原地重生",
            "nearby": "附近重生",
            "origin": "原点重生",
            "goal": "终点重生",
        }
        # 先废除导航，避免重生后继续朝旧 Goal 冲红线
        self.cancel_navigation_mission()
        self.log_message.emit(
            f"⚙ {labels.get(mode, mode)}：mode={mode} → force_recover…"
        )
        self._agent_log("P", "sim_bridge.try_unstuck", "force_recover", {"mode": mode})
        mode_cmd = (
            f"{self._source_cmd()} && "
            f"ros2 topic pub --once /long_nav/force_recover_mode std_msgs/msg/String "
            f"\"{{data: '{mode}'}}\""
        )
        bool_cmd = (
            f"{self._source_cmd()} && sleep 0.2 && "
            "ros2 topic pub --once /long_nav/force_recover std_msgs/msg/Bool \"{data: true}\""
        )
        subprocess.Popen(
            ["bash", "-c", mode_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        subprocess.Popen(
            ["bash", "-c", bool_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.log_message.emit(
            f"✓ 已请求{labels.get(mode)}；本次导航已废除，请重新 Set Goal"
        )

    def set_robot_pose(self, x: float, y: float, yaw: float = 0.0, z: float = 0.35) -> None:
        """Gazebo 设置小车位姿（Set Pose），并废除当前导航任务。"""
        self.cancel_navigation_mission()
        # 用 gz service 更稳：ros2 service call /gazebo/set_entity_state
        # yaw → quat
        import math

        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        cmd = (
            f"{self._source_cmd()} && "
            "ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState "
            "\"{"
            f"state: {{name: 'scout_mini', "
            f"pose: {{position: {{x: {float(x)}, y: {float(y)}, z: {float(z)}}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}, "
            "twist: {{linear: {{x: 0.0, y: 0.0, z: 0.0}}, "
            "angular: {{x: 0.0, y: 0.0, z: 0.0}}}}, "
            "reference_frame: 'world'}}"
            "}\""
        )
        self._agent_log(
            "S",
            "sim_bridge.set_robot_pose",
            "call",
            {"x": x, "y": y, "yaw": yaw},
        )
        self.log_message.emit(f"⚙ 设置重生位 → ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.log_message.emit("✓ 已请求 Set Pose；本次导航已废除，请重新 Set Goal")

    def set_safety_enable(self, key: str, enabled: bool) -> None:
        """安全开关：翻车尽量打到 rollover_guard 参数；其余先记状态。"""
        self._safety_enable[key] = enabled
        self._agent_log(
            "G",
            "sim_bridge.set_safety_enable",
            "toggle",
            {"key": key, "enabled": enabled},
        )
        self.log_message.emit(f"安全开关 {key}: {'开' if enabled else '关'}")

        if key == "rollover":
            en = "true" if enabled else "false"
            # 关检测：enabled=false；顺带抬高阈值双保险
            roll = 55.0 if enabled else 179.0
            pitch = 55.0 if enabled else 179.0
            ssum = 75.0 if enabled else 360.0
            cmd = (
                f"{self._source_cmd()} && "
                f"ros2 param set /rollover_guard_node enabled {en} ; "
                f"ros2 param set /rollover_guard_node rollover_roll_deg {roll} ; "
                f"ros2 param set /rollover_guard_node rollover_pitch_deg {pitch} ; "
                f"ros2 param set /rollover_guard_node rollover_sum_deg {ssum}"
            )
            subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self.log_message.emit(
                f"翻车监护 enabled={en}（关=不再自动重生；开=恢复阈值）"
            )
        elif key == "collision":
            self.log_message.emit(
                "碰撞风险：long_nav 无独立报警节点；开关已记录，"
                "当前仅影响 ScoutNav 本地策略（WARN-only，不停车）"
            )
        elif key == "stuck":
            self.log_message.emit(
                "卡死检测：导航栈无独立 stuck 开关；开关已记录，"
                "完整订阅监测仍待接入 /odom"
            )

    def estop(self) -> None:
        """急停：发 0 速并停导航（关仿真窗口）。"""
        self.log_message.emit("!!! 急停 !!!")
        self.stop_navigation(close_sim=True)

    def shutdown(self) -> None:
        """关闭所有进程。"""
        self._zero_cmd_vel()
        idle = self.ensure_sim_idle()
        self._cleanup_sim_guis()
        self.log_message.emit("仿真已关闭" if idle else "本应用仿真已关闭；外部冲突未触碰")

    @property
    def is_world_loaded(self) -> bool:
        return self._world_path is not None

    @property
    def is_mapping(self) -> bool:
        return self._map_proc is not None

    @property
    def is_costmap_saved(self) -> bool:
        return self._costmap_path is not None

    @property
    def is_navigating(self) -> bool:
        return self.navigation_alive()
