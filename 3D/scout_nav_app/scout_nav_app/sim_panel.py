"""仿真模式面板。"""

from __future__ import annotations

from enum import Enum
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from .model_loader import ModelLoader
from .safety_boundary import SafetyBoundary
from .sim_bridge import SimBridge
from .sim_conflict import format_conflicts
from .boundary_click import ClickedPointWatcher
from .coverage_utils import coverage_inside_boundary
from .nav_abort_watch import NavAbortWatcher
from .workspace_helper import runtime_debug_log_path, workspace_file


class SimState(Enum):
    IDLE = "idle"
    WORLD_LOADED = "world_loaded"
    MAPPING = "mapping"
    COSTMAP_SAVED = "costmap_saved"
    NAVIGATING = "navigating"


class SimPanel(QWidget):
    def __init__(self, model_loader: ModelLoader) -> None:
        super().__init__()
        self._model_loader = model_loader
        self._bridge = SimBridge()
        self._bridge.log_message.connect(self._on_log)
        self._boundary = SafetyBoundary()
        self._monitor = None
        self._click_watch: ClickedPointWatcher | None = None
        self._abort_watch: NavAbortWatcher | None = None
        self._state = SimState.IDLE
        self._coverage_ok = False
        self._last_coverage_ratio: float | None = None
        self._base_costmap_path: Path | None = None
        self._safety_checks = {
            "stuck": True,
            "rollover": True,
            "collision": True,
        }
        self._build_ui()

    def bind_monitor(self, monitor) -> None:
        """主窗口注入 EventMonitor。"""
        self._monitor = monitor

    def _ensure_slot_free(self, action: str) -> bool:
        """建图/导航/安全边界前：若 Gazebo/RViz 占用则弹窗询问是否关闭。"""
        import json
        import time

        conflicts = self._bridge.list_sim_conflicts()
        # #region agent log
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(
                    json.dumps(
                        {
                            "sessionId": "8873ab",
                            "hypothesisId": "Q",
                            "location": "sim_panel._ensure_slot_free",
                            "message": "check",
                            "data": {
                                "action": action,
                                "n": len(conflicts),
                                "kinds": [c.kind for c in conflicts[:15]],
                            },
                            "timestamp": int(time.time() * 1000),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion
        if not conflicts:
            return True

        detail = format_conflicts(conflicts)
        self._on_log(f"⚠ 「{action}」前检测到仿真占用:\n{detail}")
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Gazebo / RViz 正在使用")
        box.setText(
            f"无法直接执行「{action}」。\n"
            "检测到 Gazebo Server / RViz / 训练或 long_nav 相关进程仍在运行。"
        )
        box.setInformativeText(
            detail
            + "\n\n是否关闭上述进程后再继续？\n"
            "• 是 = 关闭占用进程，然后继续\n"
            "• 否 = 取消本次操作，保留现有仿真"
        )
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        box.setDefaultButton(QMessageBox.No)
        if box.exec() != QMessageBox.Yes:
            self._on_log(f"✗ 用户选择「否」：取消「{action}」，未关闭现有进程")
            return False

        self._on_log("用户选择「是」：正在关闭占用的 Gazebo/RViz/相关节点…")
        self._bridge.ensure_sim_idle()
        self._bridge._map_proc = None
        self._bridge._nav_proc = None
        left = self._bridge.list_sim_conflicts()
        # #region agent log
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(
                    json.dumps(
                        {
                            "sessionId": "8873ab",
                            "hypothesisId": "Z",
                            "location": "sim_panel._ensure_slot_free",
                            "message": "after_yes",
                            "data": {
                                "left_n": len(left),
                                "left": [
                                    {"kind": c.kind, "pid": c.pid} for c in left[:15]
                                ],
                            },
                            "timestamp": int(time.time() * 1000),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion
        if left:
            self._show_error(
                "清理未完成",
                "仍有进程占用，无法继续。请手动结束后重试：\n\n"
                + format_conflicts(left),
            )
            return False
        self._on_log("✓ 占用已清除，继续执行「" + action + "」")
        return True

    def _build_ui(self) -> None:
        # 主布局
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # 滚动区域
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # 内容容器
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(10)
        layout.setContentsMargins(10, 10, 10, 10)

        # 世界选择（默认 arena）
        world_group = QGroupBox("Gazebo 世界")
        world_layout = QVBoxLayout(world_group)

        # 默认世界由共享工作区路径工具解析。
        default_world = str(
            workspace_file(
                "ros2_ws",
                "src",
                "navigation_3d",
                "scout_offroad_3d",
                "arena",
                "bumpy_offroad_full.world",
            )
        )
        self.world_edit = QLineEdit(default_world)
        self.world_edit.setPlaceholderText("输入世界文件绝对路径")
        world_layout.addWidget(QLabel("世界路径（绝对路径）:"))
        world_layout.addWidget(self.world_edit)

        # 提示
        world_hint = QLabel(
            "提示: 默认世界位于 ros2_ws/src/navigation_3d/"
            "scout_offroad_3d/arena/（由 SCOUT_WS_ROOT 自动解析）"
        )
        world_hint.setStyleSheet("color: #666; font-size: 11px;")
        world_layout.addWidget(world_hint)

        # 世界状态
        self.world_status = QLabel("未加载")
        self.world_status.setStyleSheet("color: #666;")
        world_layout.addWidget(self.world_status)

        layout.addWidget(world_group)

        # 建图
        map_group = QGroupBox("建图 / Costmap")
        map_layout = QVBoxLayout(map_group)

        self.start_map_btn = QPushButton("开始建图（打开 Gazebo + RViz）")
        self.start_map_btn.clicked.connect(self._on_start_map)
        map_layout.addWidget(self.start_map_btn)

        self.save_map_btn = QPushButton("结束建图并保存 costmap")
        self.save_map_btn.clicked.connect(self._on_save_map)
        map_layout.addWidget(self.save_map_btn)

        # Costmap 保存路径（绝对路径）
        default_costmap = str(workspace_file("3D", "maps_long"))
        self.costmap_path_edit = QLineEdit(default_costmap)
        self.costmap_path_edit.setPlaceholderText("costmap 保存路径（绝对路径）")
        map_layout.addWidget(QLabel("保存路径（绝对路径）:"))
        map_layout.addWidget(self.costmap_path_edit)
        
        # 提示
        costmap_hint = QLabel("提示: 默认保存到工作区的 3D/maps_long/")
        costmap_hint.setStyleSheet("color: #666; font-size: 11px;")
        map_layout.addWidget(costmap_hint)

        layout.addWidget(map_group)

        # 安全边界
        boundary_group = QGroupBox("安全边界")
        boundary_layout = QVBoxLayout(boundary_group)

        boundary_layout.addWidget(
            QLabel(
                "流程：加载 maps_long → 开始设置 → 加点 → 闭合 → 检查覆盖率\n"
                "检测成功后再「保存安全 costmap」（写入下方路径；成功则关 RViz）"
            )
        )

        self.load_costmap_btn = QPushButton("加载已有 costmap")
        self.load_costmap_btn.clicked.connect(self._on_load_costmap)
        boundary_layout.addWidget(self.load_costmap_btn)

        boundary_layout.addWidget(QLabel("安全 costmap 保存路径（开始设置前可改）:"))
        default_security = str(workspace_file("3D", "maps_security"))
        self.security_path_edit = QLineEdit(default_security)
        self.security_path_edit.setPlaceholderText("例如 …/3D/maps_security")
        boundary_layout.addWidget(self.security_path_edit)

        self.security_save_status = QLabel("保存状态: 未保存")
        self.security_save_status.setStyleSheet("color: #666;")
        self.security_save_status.setWordWrap(True)
        boundary_layout.addWidget(self.security_save_status)

        self.boundary_btn = QPushButton("开始设置安全边界")
        self.boundary_btn.clicked.connect(self._on_boundary)
        boundary_layout.addWidget(self.boundary_btn)

        self.undo_point_btn = QPushButton("撤销上一个点")
        self.undo_point_btn.clicked.connect(self._on_undo_point)
        boundary_layout.addWidget(self.undo_point_btn)

        self.reset_boundary_btn = QPushButton("重设安全边界")
        self.reset_boundary_btn.clicked.connect(self._on_reset_boundary)
        boundary_layout.addWidget(self.reset_boundary_btn)

        self.close_boundary_btn = QPushButton("闭合安全边界")
        self.close_boundary_btn.clicked.connect(self._on_close_boundary)
        boundary_layout.addWidget(self.close_boundary_btn)

        self.coverage_btn = QPushButton("检查安全边界内覆盖率")
        self.coverage_btn.clicked.connect(self._on_coverage)
        boundary_layout.addWidget(self.coverage_btn)

        self.save_security_btn = QPushButton("保存安全 costmap")
        self.save_security_btn.clicked.connect(self._on_save_security)
        boundary_layout.addWidget(self.save_security_btn)

        self.boundary_status = QLabel("边界状态: 未开始")
        self.boundary_status.setStyleSheet("color: #666;")
        boundary_layout.addWidget(self.boundary_status)

        layout.addWidget(boundary_group)

        # 导航
        nav_group = QGroupBox("导航")
        nav_layout = QVBoxLayout(nav_group)

        # 导航配置
        nav_layout.addWidget(QLabel("Gazebo 世界（绝对路径）:"))
        self.nav_world_edit = QLineEdit(default_world)
        nav_layout.addWidget(self.nav_world_edit)

        nav_layout.addWidget(QLabel("Costmap 路径（绝对路径，留空用默认）:"))
        self.nav_costmap_edit = QLineEdit()
        self.nav_costmap_edit.setPlaceholderText("留空则使用默认 costmap")
        nav_layout.addWidget(self.nav_costmap_edit)

        self.start_nav_btn = QPushButton("开始导航")
        self.start_nav_btn.clicked.connect(self._on_start_nav)
        nav_layout.addWidget(self.start_nav_btn)

        self.stop_nav_btn = QPushButton("停止导航")
        self.stop_nav_btn.clicked.connect(self._on_stop_nav)
        nav_layout.addWidget(self.stop_nav_btn)

        layout.addWidget(nav_group)

        # 安全监测
        safety_group = QGroupBox("安全监测")
        safety_layout = QVBoxLayout(safety_group)

        self.stuck_check_btn = QPushButton("卡死检测: 开")
        self.stuck_check_btn.setCheckable(True)
        self.stuck_check_btn.setChecked(True)
        self.stuck_check_btn.clicked.connect(lambda: self._on_safety_toggle("stuck"))
        safety_layout.addWidget(self.stuck_check_btn)

        self.rollover_check_btn = QPushButton("翻车检测: 开")
        self.rollover_check_btn.setCheckable(True)
        self.rollover_check_btn.setChecked(True)
        self.rollover_check_btn.clicked.connect(lambda: self._on_safety_toggle("rollover"))
        safety_layout.addWidget(self.rollover_check_btn)

        self.collision_check_btn = QPushButton("碰撞风险报警: 开")
        self.collision_check_btn.setCheckable(True)
        self.collision_check_btn.setChecked(True)
        self.collision_check_btn.clicked.connect(lambda: self._on_safety_toggle("collision"))
        safety_layout.addWidget(self.collision_check_btn)

        self.unstuck_btn = QPushButton("原地重生")
        self.unstuck_btn.clicked.connect(lambda: self._on_unstuck("here"))
        safety_layout.addWidget(self.unstuck_btn)

        self.unstuck_near_btn = QPushButton("附近重生")
        self.unstuck_near_btn.clicked.connect(lambda: self._on_unstuck("nearby"))
        safety_layout.addWidget(self.unstuck_near_btn)

        self.unstuck_origin_btn = QPushButton("原点重生")
        self.unstuck_origin_btn.clicked.connect(lambda: self._on_unstuck("origin"))
        safety_layout.addWidget(self.unstuck_origin_btn)

        self.unstuck_goal_btn = QPushButton("终点重生")
        self.unstuck_goal_btn.clicked.connect(lambda: self._on_unstuck("goal"))
        safety_layout.addWidget(self.unstuck_goal_btn)

        safety_layout.addWidget(QLabel("设置重生位 (x, y, yaw°)："))
        pose_row = QHBoxLayout()
        self.respawn_x = QLineEdit("0.0")
        self.respawn_y = QLineEdit("0.0")
        self.respawn_yaw = QLineEdit("0.0")
        for w, tip in (
            (self.respawn_x, "x"),
            (self.respawn_y, "y"),
            (self.respawn_yaw, "yaw°"),
        ):
            w.setPlaceholderText(tip)
            w.setMaximumWidth(80)
            pose_row.addWidget(w)
        safety_layout.addLayout(pose_row)
        self.set_pose_btn = QPushButton("设置重生位（Set Pose）")
        self.set_pose_btn.clicked.connect(self._on_set_pose)
        safety_layout.addWidget(self.set_pose_btn)
        safety_layout.addWidget(
            QLabel(
                "RViz 重生：工具栏选「2D Pose Estimate」，点地图拉箭头\n"
                "→ 从 z=0.23 起 Z 阶梯落到该点（并废除本次导航）\n"
                "也可点下方按钮；手动坐标为备用"
            )
        )

        layout.addWidget(safety_group)

        # 设置滚动区域内容
        scroll.setWidget(content)
        main_layout.addWidget(scroll)

    def _on_log(self, msg: str) -> None:
        # 转发到主窗口（parent 是 QStackedWidget，没有 log）
        win = self.window()
        if win is not None and hasattr(win, "log"):
            win.log(msg)

    def _show_error(self, title: str, msg: str) -> None:
        """显示错误提示。"""
        QMessageBox.warning(self, title, msg)
        self._on_log(f"✗ {msg}")

    def _show_info(self, title: str, msg: str) -> None:
        """显示信息提示。"""
        QMessageBox.information(self, title, msg)
        self._on_log(f"✓ {msg}")

    def _on_safety_toggle(self, key: str) -> None:
        """安全监测开关切换（以 checkable 按钮状态为准）。"""
        btn = getattr(self, f"{key}_check_btn")
        enabled = btn.isChecked()
        self._safety_checks[key] = enabled
        labels = {
            "stuck": "卡死检测",
            "rollover": "翻车检测",
            "collision": "碰撞风险报警",
        }
        state = "开" if enabled else "关"
        btn.setText(f"{labels.get(key, key)}: {state}")
        self._bridge.set_safety_enable(key, enabled)
        if self._monitor is not None:
            self._monitor.set_enabled(key, enabled)
        self._on_log(f"{key} 检测: {state}")

    def _on_unstuck(self, mode: str = "here") -> None:
        """手动重生：here / nearby / origin / goal（均废除本次导航）。"""
        labels = {
            "here": "原地重生",
            "nearby": "附近重生",
            "origin": "原点重生",
            "goal": "终点重生",
        }
        self._on_log(f"用户请求：{labels.get(mode, mode)}（将废除本次导航）")
        self._bridge.try_unstuck(mode)

    def _on_set_pose(self) -> None:
        try:
            x = float(self.respawn_x.text().strip())
            y = float(self.respawn_y.text().strip())
            yaw_deg = float(self.respawn_yaw.text().strip() or "0")
        except ValueError:
            self._show_error("错误", "请输入有效的 x / y / yaw° 数字")
            return
        import math

        self._on_log(f"用户请求：Set Pose → ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)")
        self._bridge.set_robot_pose(x, y, yaw=math.radians(yaw_deg))
        self._show_info(
            "已设置重生位",
            f"已请求放到 ({x:.2f}, {y:.2f})。\n"
            "本次导航已废除。\n"
            "请在 RViz 用 Set Goal 重新开始。",
        )

    def _on_start_map(self) -> None:
        """开始建图：复用 long_nav_map（自带 Gazebo+RViz，不需要单独 start_rviz）。"""
        # #region agent log
        import json, time
        _dbg = runtime_debug_log_path()
        def _agent_log(hid, msg, data=None):
            try:
                with open(_dbg, "a") as f:
                    f.write(json.dumps({"sessionId":"8873ab","hypothesisId":hid,"location":"sim_panel._on_start_map","message":msg,"data":data or {},"timestamp":int(time.time()*1000)}, ensure_ascii=False) + "\n")
            except Exception:
                pass
        _agent_log("A", "enter_start_map", {"has_start_rviz": hasattr(self._bridge, "start_rviz"), "model_loaded": self._model_loader.loaded, "state": self._state.value})
        # #endregion
        try:
            # 建图不需要 TD3 权重（README4 §0）
            if not self._ensure_slot_free("开始建图"):
                return

            world = self.world_edit.text().strip()
            if not world:
                self._show_error("错误", "请输入世界路径")
                return

            world_path = Path(world)
            if not world_path.exists():
                self._show_error("错误", f"世界文件不存在: {world}")
                self.world_status.setText("✗ 世界不存在")
                self.world_status.setStyleSheet("color: red;")
                return

            if not self._bridge.load_world(world):
                self._show_error("错误", "加载世界失败")
                return

            self.world_status.setText(f"✓ 世界已加载: {world_path.name}")
            self.world_status.setStyleSheet("color: green;")

            # #region agent log
            _agent_log("A", "before_start_mapping", {"world": world})
            # #endregion
            if not self._bridge.start_mapping():
                self._show_error("错误", "启动建图失败（见终端日志）")
                return

            self._state = SimState.MAPPING
            # #region agent log
            _agent_log("A", "start_mapping_ok", {"state": self._state.value})
            # #endregion
            if self._monitor is not None:
                self._monitor.start()
            self._on_log("✓ 已请求 long_nav_map；2s 后检查进程是否存活")
            QTimer.singleShot(2000, self._check_mapping_alive)
        except Exception as e:
            # #region agent log
            _agent_log("A", "start_map_exception", {"error": repr(e)})
            # #endregion
            self._show_error("错误", f"开始建图异常: {e}")

    def _check_mapping_alive(self) -> None:
        """建图 launch 拉起后非阻塞体检（隔离失败会立刻退出）。"""
        # #region agent log
        import json, time
        alive = self._bridge.mapping_alive()
        hint = self._bridge.mapping_exit_hint()
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(json.dumps({"sessionId":"8873ab","hypothesisId":"C","location":"sim_panel._check_mapping_alive","message":"poll","data":{"alive": alive, "hint_head": hint[:200]},"timestamp":int(time.time()*1000)}, ensure_ascii=False) + "\n")
        except Exception:
            pass
        # #endregion
        if alive:
            self._on_log("✓ long_nav_map 仍在运行（应已弹出 Gazebo/RViz）")
            return
        self._state = SimState.IDLE
        self._show_error(
            "建图启动失败",
            "long_nav_map 已退出。常见原因：已有 gzserver/训练占用。"
            "请先结束现有仿真再点开始建图。\n\n" + hint[-500:],
        )
    def _on_save_map(self) -> None:
        """结束建图并保存 costmap，同时关闭 Gazebo/RViz。"""
        if self._state != SimState.MAPPING and not self._bridge.mapping_alive():
            self._show_error("错误", "请先开始建图")
            return

        path = self.costmap_path_edit.text().strip()
        output = Path(path) if path else None

        if not self._bridge.end_mapping(output):
            self._show_error("错误", "结束建图失败（见终端日志）")
            return

        self._state = SimState.COSTMAP_SAVED
        self._show_info("成功", f"Costmap 已保存，建图窗口已关闭\n路径: {output}")

    def _on_coverage(self) -> None:
        """检测：已闭合、无交叉、边界内覆盖率合格 → 检测成功。"""
        import json
        import time

        if not self._boundary.closed:
            self._coverage_ok = False
            self._show_error("检测失败", "安全边界未闭合，请先「闭合安全边界」")
            return
        if self._boundary._is_self_intersecting():  # noqa: SLF001 — 显式再检一次交叉
            self._coverage_ok = False
            self._show_error("检测失败", "安全边界存在交叉，请重设后再检")
            return
        if not self._boundary.is_valid():
            self._coverage_ok = False
            self._show_error("检测失败", "安全边界无效（需闭合且至少 3 顶点）")
            return
        if not self._bridge.is_costmap_saved:
            self._coverage_ok = False
            self._show_error("错误", "请先加载或保存建图 costmap（maps_long）")
            return

        map_dir = self._base_costmap_path or Path(self._bridge._costmap_path)  # type: ignore[arg-type]
        ratio, info = coverage_inside_boundary(map_dir, self._boundary.points())
        self._last_coverage_ratio = ratio
        self._on_log(
            f"边界内覆盖率: {ratio:.1%}（已知 {info.get('known')}/"
            f"{info.get('total')} 采样格）"
        )
        # #region agent log
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(
                    json.dumps(
                        {
                            "sessionId": "8873ab",
                            "hypothesisId": "P",
                            "location": "sim_panel._on_coverage",
                            "message": "coverage_result",
                            "data": {
                                "ratio": ratio,
                                "closed": self._boundary.closed,
                                "n": self._boundary.point_count(),
                                "map_dir": str(map_dir),
                            },
                            "timestamp": int(time.time() * 1000),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion

        if ratio < 0.7:
            self._coverage_ok = False
            self._show_error(
                "检测失败",
                f"覆盖率不合格：{ratio:.1%} < 70%\n请继续建图补全后再检",
            )
            return

        self._coverage_ok = True
        self._show_info(
            "检测成功",
            f"检测成功\n"
            f"· 已闭合（{self._boundary.point_count()} 顶点）\n"
            f"· 无自交\n"
            f"· 边界内覆盖率 {ratio:.1%} ≥ 70%\n"
            f"可以点「保存安全 costmap」",
        )

    def _update_boundary_status(self) -> None:
        if self._boundary.closed:
            text = f"边界状态: 已闭合（{self._boundary.point_count()} 顶点）"
            color = "green"
        elif self._boundary.editing:
            text = f"边界状态: 设置中（{self._boundary.point_count()} 顶点）"
            color = "#c45c26"
        else:
            text = "边界状态: 未开始"
            color = "#666"
        self.boundary_status.setText(text)
        self.boundary_status.setStyleSheet(f"color: {color};")

    def _on_boundary(self) -> None:
        """开始设置安全边界。"""
        if self._state == SimState.MAPPING or self._bridge.mapping_alive():
            self._show_error("错误", "请先结束建图，再设置安全边界")
            return
        if not self._has_costmap():
            self._show_error("错误", "请先保存或加载 costmap")
            return

        if not self._ensure_slot_free("设置安全边界"):
            return

        self._coverage_ok = False
        self._last_coverage_ratio = None
        # 记住建图底图目录，覆盖率始终相对它（不被 security 保存路径冲掉）
        if self._bridge._costmap_path is not None:
            self._base_costmap_path = Path(self._bridge._costmap_path)
        self.security_save_status.setText("保存状态: 未保存")
        self.security_save_status.setStyleSheet("color: #666;")
        self._boundary.begin_edit()
        self._update_boundary_status()
        self._on_log("开始设置安全边界：打开 RViz，请用 Publish Point 依次点顶点")
        # 清空上一次顶点状态
        self._bridge.publish_boundary_vertices([], closed=False)
        sec = self.security_path_edit.text().strip()
        if not self._bridge.open_boundary_rviz(security_dir=sec or None):
            self._boundary.end_edit()
            self._update_boundary_status()
            self._show_error("错误", "打开 RViz 失败（见终端）")
            return

        if self._click_watch is not None:
            self._click_watch.stop()
        self._click_watch = ClickedPointWatcher(self._bridge._source_cmd(), self)
        self._click_watch.point.connect(self._on_clicked_point)
        self._click_watch.status.connect(self._on_log)
        self._click_watch.start()

        self._show_info(
            "提示",
            "RViz 已打开（无需小车）。\n"
            "1) Publish Point 加点（可点透明/未扫区域）\n"
            "2) 「闭合安全边界」（检交叉）\n"
            "3) 「检查安全边界内覆盖率」→ 检测成功\n"
            "4) 「保存安全 costmap」成功后自动关 RViz",
        )

    def _sync_boundary_viz(self) -> None:
        self._bridge.publish_boundary_vertices(
            self._boundary.points(), closed=self._boundary.closed
        )

    def _on_clicked_point(self, x: float, y: float) -> None:
        ok, msg = self._boundary.add_point(x, y)
        if not ok:
            self._show_error("提示", msg)
            return
        self._coverage_ok = False
        self._on_log(f"✓ {msg}")
        self._update_boundary_status()
        self._sync_boundary_viz()

    def _on_undo_point(self) -> None:
        ok, msg = self._boundary.undo_last()
        if not ok:
            self._show_error("提示", msg)
            return
        self._coverage_ok = False
        self._on_log(f"✓ {msg}")
        self._update_boundary_status()
        self._sync_boundary_viz()

    def _on_reset_boundary(self) -> None:
        ok, msg = self._boundary.reset()
        if not ok:
            self._show_error("提示", msg)
            return
        self._coverage_ok = False
        self._on_log(f"✓ {msg}")
        self._update_boundary_status()
        self._sync_boundary_viz()

    def _on_close_boundary(self) -> None:
        """仅闭合并检交叉；不保存。下一步去检查覆盖率。"""
        self._coverage_ok = False
        ok, msg = self._boundary.close()
        if not ok:
            self._show_error("安全边界", msg)
            self._on_log(f"✗ {msg}")
            self._update_boundary_status()
            return
        self._on_log(f"✓ {msg}")
        self._update_boundary_status()
        self._sync_boundary_viz()
        self._show_info(
            "闭合成功",
            msg + "\n请点「检查安全边界内覆盖率」；检测成功后再保存。",
        )

    def _on_save_security(self) -> None:
        """检测成功后保存到 security 路径；成功关 RViz，失败保持 RViz。"""
        import json
        import time

        if not self._boundary.is_valid():
            self._show_error("无法保存", "请先成功「闭合安全边界」（无交叉）")
            return
        if not self._coverage_ok:
            self._show_error(
                "无法保存",
                "请先「检查安全边界内覆盖率」并得到「检测成功」后再保存",
            )
            return

        out = self.security_path_edit.text().strip()
        if not out:
            self._show_error("无法保存", "请填写安全 costmap 保存路径")
            return

        # 确保最新闭合围栏已同步到 overlay
        self._sync_boundary_viz()
        saved = self._bridge.save_security_costmap(out)
        # #region agent log
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(
                    json.dumps(
                        {
                            "sessionId": "8873ab",
                            "hypothesisId": "P",
                            "location": "sim_panel._on_save_security",
                            "message": "save_result",
                            "data": {
                                "ok": saved is not None,
                                "out": out,
                                "coverage_ok": self._coverage_ok,
                                "ratio": self._last_coverage_ratio,
                            },
                            "timestamp": int(time.time() * 1000),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion

        if saved is None:
            self.security_save_status.setText(f"保存状态: 失败（RViz 仍打开）→ {out}")
            self.security_save_status.setStyleSheet("color: #c62828;")
            self._show_error(
                "保存失败",
                f"写入失败，RViz 保持打开，请重试。\n路径: {out}",
            )
            return

        # 成功：关 RViz + 展示路径
        if self._click_watch is not None:
            self._click_watch.stop()
            self._click_watch = None
        self._bridge.close_boundary_rviz()
        self._boundary.end_edit()
        self.nav_costmap_edit.setText(str(saved))
        self.security_path_edit.setText(str(saved))
        self.security_save_status.setText(f"保存成功: {saved}\n已关闭 RViz")
        self.security_save_status.setStyleSheet("color: green;")
        self._state = SimState.COSTMAP_SAVED
        self._show_info(
            "保存成功",
            f"安全 costmap 已保存\n路径: {saved}\n已关闭 RViz\n"
            "（maps_long 未改动；导航请用此路径）",
        )

    def _on_load_costmap(self) -> None:
        """加载已有 costmap。"""
        path = self.costmap_path_edit.text().strip()
        if not path:
            self._show_error("错误", "请输入 costmap 路径")
            return
        if not self._bridge.load_costmap(path):
            self._show_error("错误", f"Costmap 不存在或无效: {path}")
            return
        self._base_costmap_path = Path(self._bridge._costmap_path)  # type: ignore[arg-type]
        self._state = SimState.COSTMAP_SAVED
        self._show_info("成功", f"Costmap 已加载: {path}\n可开始设置安全边界")

    def _has_costmap(self) -> bool:
        return self._state == SimState.COSTMAP_SAVED or self._bridge.is_costmap_saved

    def _on_start_nav(self) -> None:
        """开始导航。"""
        if not self._model_loader.loaded:
            self._show_error("错误", "请先点击顶部「加载模型」成功后再开始导航")
            return

        world = self.nav_world_edit.text().strip() or self.world_edit.text().strip()
        if world and not self._bridge.is_world_loaded:
            if not self._bridge.load_world(world):
                self._show_error("错误", "世界路径无效")
                return

        if not self._bridge.is_world_loaded:
            self._show_error("错误", "请先确认世界路径")
            return

        costmap = self.nav_costmap_edit.text().strip() or self.costmap_path_edit.text().strip()
        if not self._bridge.is_costmap_saved:
            if costmap and self._bridge.load_costmap(costmap):
                self._state = SimState.COSTMAP_SAVED
            else:
                self._show_error("错误", "请先保存或加载 costmap")
                return

        model = self._model_loader.loaded
        self._on_log(f"开始导航，世界: {world}, costmap: {costmap or self._bridge._costmap_path}")

        if not self._ensure_slot_free("开始导航"):
            return

        if not self._bridge.start_navigation(model["path"]):
            self._show_error("错误", "启动导航失败")
            return

        self._state = SimState.NAVIGATING
        self._start_nav_abort_watch()
        self._on_log("已请求导航；3s 后检查 long_nav_nav 是否真正存活")
        QTimer.singleShot(12000, self._check_nav_alive)

    def _start_nav_abort_watch(self) -> None:
        if self._abort_watch is not None:
            self._abort_watch.stop()
        self._abort_watch = NavAbortWatcher(self._bridge._source_cmd(), self)
        self._abort_watch.aborted.connect(self._on_nav_aborted)
        self._abort_watch.rejected.connect(self._on_goal_rejected)
        self._abort_watch.status.connect(self._on_log)
        self._abort_watch.start()

    def _stop_nav_abort_watch(self) -> None:
        if self._abort_watch is not None:
            self._abort_watch.stop()
            self._abort_watch = None

    def _security_popup_ok(self, reason: str, kind: str) -> bool:
        """True = 应弹穿红线相关窗；取消/重生/无关文案不弹。"""
        import time

        if "用户取消" in reason or "非穿红线" in reason or "cancel" in reason.lower():
            self._on_log(f"· 导航已取消（重生/手动）：{reason}")
            return False
        if "SUCCESS" in reason or "任务完成" in reason:
            self._on_log(f"· {reason}")
            return False
        if (
            "红线" not in reason
            and "无限大" not in reason
            and "security" not in reason.lower()
            and "隔开" not in reason
        ):
            self._on_log(f"· 导航提示：{reason}")
            return False
        attr = "_last_abort_popup_t" if kind == "abort" else "_last_reject_popup_t"
        now = time.time()
        last = getattr(self, attr, 0.0)
        if now - last < 4.0:
            self._on_log(f"· （已忽略重复弹窗）{reason[:80]}")
            return False
        setattr(self, attr, now)
        return True

    def _on_nav_aborted(self, reason: str) -> None:
        """任务因穿红线已停止 → 弹窗。"""
        import json
        import time

        if not self._security_popup_ok(reason, "abort"):
            return

        # #region agent log
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(
                    json.dumps(
                        {
                            "sessionId": "8873ab",
                            "hypothesisId": "S",
                            "location": "sim_panel._on_nav_aborted",
                            "message": "popup_abort",
                            "data": {"reason": reason[:200]},
                            "timestamp": int(time.time() * 1000),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion
        self._on_log(f"✗ 导航中止(穿红线): {reason}")
        win = self.window()
        if win is not None:
            win.raise_()
            win.activateWindow()
        QMessageBox.warning(
            self,
            "安全边界：导航已停止",
            "检测到小车规划路径会穿过安全红线（代价无限大）。\n"
            "本次导航已停止，红色全局路径已清除，本地跟随已锁轮。\n\n"
            "请重新设置导航终点（2D Goal Pose），\n"
            "或用「2D Pose Estimate」/ 重生改起点后再导航。\n\n"
            f"详情：{reason}",
        )

    def _on_goal_rejected(self, reason: str) -> None:
        """新终点会穿红线：拒绝切换，当前任务/待机不变 → 弹窗提醒。"""
        import json
        import time

        if not self._security_popup_ok(reason, "reject"):
            return

        # #region agent log
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(
                    json.dumps(
                        {
                            "sessionId": "8873ab",
                            "hypothesisId": "S",
                            "location": "sim_panel._on_goal_rejected",
                            "message": "popup_reject",
                            "data": {"reason": reason[:200]},
                            "timestamp": int(time.time() * 1000),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
        except Exception:
            pass
        # #endregion
        self._on_log(f"⚠ 新终点已拒绝(穿红线): {reason}")
        win = self.window()
        if win is not None:
            win.raise_()
            win.activateWindow()
        QMessageBox.warning(
            self,
            "安全边界：新终点已拒绝",
            "新终点会被安全红线隔开（不过红线到不了）。\n"
            "已拒绝切换：\n"
            "· 若当前有导航 → 继续执行原终点\n"
            "· 若当前无导航 → 保持待机\n\n"
            "请在红线围成的可达区域内重新 2D Goal Pose。\n\n"
            f"详情：{reason}",
        )

    def _check_nav_alive(self) -> None:
        """导航 launch 拉起后体检（gz 残留会导致隔离立刻退出）。"""
        # #region agent log
        import json, time
        alive = self._bridge.navigation_alive()
        hint = self._bridge.navigation_exit_hint()
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(json.dumps({"sessionId":"8873ab","runId":"post-fix","hypothesisId":"I","location":"sim_panel._check_nav_alive","message":"poll","data":{"alive": alive, "hint_head": hint[:300]},"timestamp":int(time.time()*1000)}, ensure_ascii=False) + "\n")
        except Exception:
            pass
        # #endregion
        if alive:
            # 再确认 spawn
            hint = self._bridge.navigation_exit_hint()
            logf = getattr(self._bridge, "_nav_log", None)
            spawn_ok = False
            if logf and Path(logf).exists():
                text = Path(logf).read_text(errors="replace")
                spawn_ok = "spawn OK" in text
                if "entity [scout_mini] does not exist" in text and not spawn_ok:
                    self._on_log("⚠ 仍未见 scout_mini，请再等几秒或看 launch 日志")
            if spawn_ok:
                self._on_log("✓ long_nav_nav 运行中且 spawn OK（应能看到小车）")
            else:
                self._on_log("✓ long_nav_nav 仍在运行（应已弹出 Gazebo/RViz）")
            self._show_info("成功", "导航已开始：在 RViz 用 2D Goal Pose 点终点")
            return
        self._state = SimState.COSTMAP_SAVED
        self._show_error(
            "导航启动失败",
            "long_nav_nav 已退出。常见原因：建图 Gazebo 未关干净。\n"
            "请先点「结束建图并保存」或「停止导航」清残留后再开。\n\n"
            + hint[-600:],
        )

    def _on_stop_nav(self) -> None:
        """停止导航：停车 + 关掉 Gazebo/RViz。"""
        # #region agent log
        import json, time
        try:
            with runtime_debug_log_path().open("a", encoding="utf-8") as f:
                f.write(json.dumps({"sessionId":"8873ab","hypothesisId":"F","location":"sim_panel._on_stop_nav","message":"enter","data":{"state": self._state.value},"timestamp":int(time.time()*1000)}, ensure_ascii=False) + "\n")
        except Exception:
            pass
        # #endregion
        if self._state != SimState.NAVIGATING and not self._bridge.is_navigating:
            self._show_error("错误", "当前未在导航")
            return

        self._bridge.stop_navigation(close_sim=True)
        self._stop_nav_abort_watch()
        self._state = SimState.COSTMAP_SAVED
        self._show_info("成功", "导航已停止：已将小车锁死，并关闭仿真窗口")

    def estop(self) -> None:
        """急停。"""
        self._bridge.estop()
        self._state = SimState.IDLE
        self._on_log("!!! 急停：已将小车锁死并关闭仿真 !!!")
