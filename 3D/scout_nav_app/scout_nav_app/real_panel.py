"""实车模式面板。"""

from __future__ import annotations

from enum import Enum
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from .coverage_utils import coverage_inside_boundary
from .dialogs import apply_dialog_style, ask_password, styled_info, styled_warning
from .model_loader import ModelLoader
from .real_bridge import lan_hint_for_host, RealBridge
from .safety_boundary import SafetyBoundary
from .workspace_helper import workspace_file


class RealState(Enum):
    IDLE = "idle"
    CONNECTED = "connected"
    MAPPING = "mapping"
    COSTMAP_SAVED = "costmap_saved"
    NAVIGATING = "navigating"


class RealPanel(QWidget):
    def __init__(self, model_loader: ModelLoader) -> None:
        super().__init__()
        self._model_loader = model_loader
        self._bridge = RealBridge()
        self._bridge.log_message.connect(self._on_log)
        self._bridge.connected.connect(self._on_connected)
        self._bridge.connect_finished.connect(self._on_connect_finished)
        self._bridge.check_result.connect(self._on_check_result)
        self._bridge.locked_changed.connect(self._on_locked_changed)
        self._state = RealState.IDLE
        self._connecting = False
        self._coverage_ok = False
        self._boundary = SafetyBoundary()
        self._monitor = None
        self._safety_checks = {
            "stuck": True,
            "rollover": True,
            "collision": True,
        }
        self._bridge.safety_alarm.connect(self._on_safety_alarm)
        self._build_ui()
        self._set_lock_style(False)

    @property
    def bridge(self) -> RealBridge:
        return self._bridge

    def bind_monitor(self, monitor) -> None:
        """主窗口注入 EventMonitor（开关与仿真同一套）。"""
        self._monitor = monitor

    def _build_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        lock_group = QGroupBox("锁车")
        lock_layout = QVBoxLayout(lock_group)
        lock_layout.setContentsMargins(8, 8, 8, 8)
        self.lock_btn = QPushButton("锁车")
        self.lock_btn.setCheckable(True)
        self.lock_btn.setMinimumHeight(36)
        self.lock_btn.clicked.connect(self._on_lock_clicked)
        lock_layout.addWidget(self.lock_btn)
        self.lock_status = QLabel("未锁")
        lock_layout.addWidget(self.lock_status)
        main_layout.addWidget(lock_group)

        wifi = QLabel(
            "实车功能前，请确认工作站与车辆接入同一现场局域网。\n"
            "具体网络信息只应保存在本机私有配置中，不随源码发布。"
        )
        wifi.setWordWrap(True)
        wifi.setStyleSheet(
            "QLabel { background: #fff3cd; color: #7a3e00; padding: 6px 8px; font-weight: bold; }"
        )
        main_layout.addWidget(wifi)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(10)
        layout.setContentsMargins(2, 2, 8, 10)

        conn_group = QGroupBox("连接")
        conn_layout = QVBoxLayout(conn_group)
        conn_layout.addWidget(QLabel("车辆地址:"))
        self.host_edit = QLineEdit()
        self.host_edit.setPlaceholderText("用户名@车辆主机（从本机私有配置或现场信息填写）")
        conn_layout.addWidget(self.host_edit)
        self.connect_btn = QPushButton("连接车辆")
        self.connect_btn.clicked.connect(self._on_connect)
        conn_layout.addWidget(self.connect_btn)
        self.conn_status = QLabel("未连接")
        self.conn_status.setStyleSheet("color: #666;")
        conn_layout.addWidget(self.conn_status)
        layout.addWidget(conn_group)

        check_group = QGroupBox("自检")
        check_layout = QVBoxLayout(check_group)
        self.check_nodes_btn = QPushButton("检查节点")
        self.check_nodes_btn.clicked.connect(self._on_check_nodes)
        self.check_topics_btn = QPushButton("检查话题")
        self.check_topics_btn.clicked.connect(self._on_check_topics)
        self.check_tf_btn = QPushButton("检查 TF")
        self.check_tf_btn.clicked.connect(self._on_check_tf)
        self.check_storage_btn = QPushButton("检查存储")
        self.check_storage_btn.clicked.connect(self._on_check_storage)
        self._add_button_grid(
            check_layout,
            [
                self.check_nodes_btn,
                self.check_topics_btn,
                self.check_tf_btn,
                self.check_storage_btn,
            ],
            columns=2,
        )
        self.check_status = QLabel("未检查")
        self.check_status.setWordWrap(True)
        check_layout.addWidget(self.check_status)
        layout.addWidget(check_group)

        factory_group = QGroupBox("厂方雷达 / 建图 / 导航 / 相机")
        factory_layout = QVBoxLayout(factory_group)
        factory_layout.setContentsMargins(8, 8, 8, 8)
        hint_f = QLabel(
            "功能：原厂 2D 建图与导航（gmapping、move_base）。\n"
            "显示：车辆端 RViz。\n"
            "建图层：Map、LaserScan。\n"
            "导航工具：2D Nav Goal。\n"
            "相机显示：车辆端 rqt_image_view。\n"
            "相机话题：/camera/color/image_raw。"
        )
        hint_f.setWordWrap(True)
        hint_f.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        factory_layout.addWidget(hint_f)
        self.lidar_btn = QPushButton("开雷达")
        self.lidar_btn.setToolTip("roslaunch scout_bringup open_rslidar.launch")
        self.lidar_btn.clicked.connect(self._on_start_lidar)
        self.gmap_btn = QPushButton("开建图")
        self.gmap_btn.setToolTip("roslaunch scout_bringup gmapping.launch")
        self.gmap_btn.clicked.connect(self._on_factory_map)
        self.factory_nav_btn = QPushButton("开导航")
        self.factory_nav_btn.setToolTip("roslaunch scout_bringup navigation_4wd.launch")
        self.factory_nav_btn.clicked.connect(self._on_factory_nav_start)
        self.camera_btn = QPushButton("开相机")
        self.camera_btn.setToolTip("RealSense D435；话题 /camera/color/image_raw")
        self.camera_btn.clicked.connect(self._on_start_camera)
        self.factory_nav_stop_btn = QPushButton("停止")
        self.factory_nav_stop_btn.clicked.connect(self._on_factory_nav_stop)
        self._add_button_grid(
            factory_layout,
            [
                self.lidar_btn,
                self.gmap_btn,
                self.factory_nav_btn,
                self.camera_btn,
            ],
            columns=2,
        )
        self.factory_nav_stop_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        factory_layout.addWidget(self.factory_nav_stop_btn)
        layout.addWidget(factory_group)

        map_group = QGroupBox("建图 / Costmap")
        map_layout = QVBoxLayout(map_group)
        map_layout.setContentsMargins(8, 8, 8, 8)
        hint_m = QLabel(
            "功能：由激光点云生成可通行性代价地图（scout_long_nav）。\n"
            "输入：/rslidar_points，/odom。\n"
            "输出：maps_long/arena_trav.yaml。\n"
            "前提：雷达已启动；遥控扫场。"
        )
        hint_m.setWordWrap(True)
        hint_m.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        map_layout.addWidget(hint_m)
        default_costmap = str(workspace_file("3D", "maps_long"))
        map_layout.addWidget(QLabel("保存路径:"))
        self.costmap_path_edit = QLineEdit(default_costmap)
        map_layout.addWidget(self.costmap_path_edit)
        self.start_trav_map_btn = QPushButton("开始建图")
        self.start_trav_map_btn.clicked.connect(self._on_start_trav_map)
        self.end_trav_map_btn = QPushButton("结束建图并保存")
        self.end_trav_map_btn.clicked.connect(self._on_end_trav_map)
        self._add_button_grid(
            map_layout,
            [self.start_trav_map_btn, self.end_trav_map_btn],
            columns=2,
        )
        self.map_status = QLabel("状态: 未建图")
        self.map_status.setStyleSheet("color: #666;")
        map_layout.addWidget(self.map_status)
        layout.addWidget(map_group)

        boundary_group = QGroupBox("安全边界")
        boundary_layout = QVBoxLayout(boundary_group)
        boundary_layout.setContentsMargins(8, 8, 8, 8)
        hint_b = QLabel(
            "功能：在可通行性地图上标注禁行多边形。\n"
            "输入：maps_long/arena_trav.yaml。\n"
            "输出：maps_security。\n"
            "顶点工具：仿真 RViz Publish Point。"
        )
        hint_b.setWordWrap(True)
        hint_b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        boundary_layout.addWidget(hint_b)
        default_local = str(workspace_file("3D", "maps_long"))
        self.local_map_edit = QLineEdit(default_local)
        boundary_layout.addWidget(QLabel("输入：本机 costmap 目录（maps_long）"))
        boundary_layout.addWidget(self.local_map_edit)
        default_security = str(workspace_file("3D", "maps_security"))
        self.security_path_edit = QLineEdit(default_security)
        boundary_layout.addWidget(QLabel("输出：安全 costmap 目录（maps_security）"))
        boundary_layout.addWidget(self.security_path_edit)
        self.load_costmap_btn = QPushButton("加载已有 costmap")
        self.load_costmap_btn.clicked.connect(self._on_load_costmap)
        self.boundary_btn = QPushButton("开始设置安全边界")
        self.boundary_btn.clicked.connect(self._on_boundary)
        self.undo_point_btn = QPushButton("撤销上一个点")
        self.undo_point_btn.clicked.connect(self._on_undo_point)
        self.reset_boundary_btn = QPushButton("重设安全边界")
        self.reset_boundary_btn.clicked.connect(self._on_reset_boundary)
        self.close_boundary_btn = QPushButton("闭合安全边界")
        self.close_boundary_btn.clicked.connect(self._on_close_boundary)
        self.coverage_btn = QPushButton("检查安全边界内覆盖率")
        self.coverage_btn.clicked.connect(self._on_coverage)
        self._add_button_grid(
            boundary_layout,
            [
                self.load_costmap_btn,
                self.boundary_btn,
                self.undo_point_btn,
                self.reset_boundary_btn,
                self.close_boundary_btn,
                self.coverage_btn,
            ],
            columns=2,
        )
        self.save_security_btn = QPushButton("保存安全 costmap")
        self.save_security_btn.clicked.connect(self._on_save_security)
        self.save_security_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        boundary_layout.addWidget(self.save_security_btn)
        self.boundary_status = QLabel("边界状态: 未开始")
        self.boundary_status.setStyleSheet("color: #666;")
        boundary_layout.addWidget(self.boundary_status)
        layout.addWidget(boundary_group)

        nav_group = QGroupBox("TD3")
        nav_layout = QVBoxLayout(nav_group)
        td3_hint = QLabel(
            "功能：TD3 局部策略接口。\n"
            "速度输出：关闭。本模式不向 /cmd_vel 发布。\n"
            "可视化：仿真模式 RViz。\n"
            "平面点到点：厂方导航。"
        )
        td3_hint.setWordWrap(True)
        td3_hint.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        nav_layout.addWidget(td3_hint)
        self.enable_cmd_label = QLabel("速度输出: 关")
        nav_layout.addWidget(self.enable_cmd_label)
        self.enable_cmd_btn = QPushButton("使能速度输出")
        self.enable_cmd_btn.setCheckable(True)
        self.enable_cmd_btn.clicked.connect(self._on_enable_cmd)
        self.start_nav_btn = QPushButton("开始 TD3 导航")
        self.start_nav_btn.clicked.connect(self._on_start_nav)
        self._add_button_grid(
            nav_layout,
            [self.enable_cmd_btn, self.start_nav_btn],
            columns=2,
        )
        self.stop_nav_btn = QPushButton("停止 TD3 导航")
        self.stop_nav_btn.clicked.connect(self._on_stop_nav)
        self.stop_nav_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        nav_layout.addWidget(self.stop_nav_btn)
        layout.addWidget(nav_group)

        safety_group = QGroupBox("安全监测")
        safety_layout = QVBoxLayout(safety_group)
        self.stuck_check_btn = QPushButton("卡死检测: 开")
        self.stuck_check_btn.setCheckable(True)
        self.stuck_check_btn.setChecked(True)
        self.stuck_check_btn.clicked.connect(lambda: self._on_safety_toggle("stuck"))
        self.rollover_check_btn = QPushButton("翻车检测: 开")
        self.rollover_check_btn.setCheckable(True)
        self.rollover_check_btn.setChecked(True)
        self.rollover_check_btn.clicked.connect(lambda: self._on_safety_toggle("rollover"))
        self.collision_check_btn = QPushButton("碰撞风险报警: 开")
        self.collision_check_btn.setCheckable(True)
        self.collision_check_btn.setChecked(True)
        self.collision_check_btn.clicked.connect(lambda: self._on_safety_toggle("collision"))
        self._add_button_grid(
            safety_layout,
            [
                self.stuck_check_btn,
                self.rollover_check_btn,
                self.collision_check_btn,
            ],
            columns=3,
        )
        layout.addWidget(safety_group)

        rec_group = QGroupBox("数据记录")
        rec_layout = QVBoxLayout(rec_group)
        rec_layout.setContentsMargins(8, 8, 8, 8)
        rec_line = QLabel(
            "记录话题：/cmd_vel，/odom。可选 /rslidar_points。\n"
            "存储：车辆端本机数据目录（由私有部署配置决定）。"
        )
        rec_line.setWordWrap(True)
        rec_line.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        rec_layout.addWidget(rec_line)
        self.record_lidar_chk = QCheckBox("同时录 /rslidar_points")
        rec_layout.addWidget(self.record_lidar_chk)
        self.record_btn = QPushButton("开始记录")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record)
        rec_layout.addWidget(self.record_btn)
        layout.addWidget(rec_group)

        layout.addStretch(1)

        self._tighten_groups(content)
        self._tighten_groups(lock_group)

        scroll.setWidget(content)
        main_layout.addWidget(scroll)

    def _add_button_grid(self, parent_layout, buttons, columns: int = 3) -> None:
        grid = QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        for i, btn in enumerate(buttons):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setMinimumWidth(0)
            row, col = divmod(i, columns)
            grid.addWidget(btn, row, col)
        for col in range(columns):
            grid.setColumnStretch(col, 1)
            grid.setColumnMinimumWidth(col, 96)
        parent_layout.addLayout(grid)

    def _tighten_groups(self, root: QWidget) -> None:
        for btn in root.findChildren(QPushButton):
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setMinimumWidth(0)
            btn.setMinimumHeight(36)
            btn.setMaximumHeight(42)
        for box in root.findChildren(QGroupBox):
            box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        root.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)

    def _on_log(self, msg: str) -> None:
        win = self.window()
        if win is not None and hasattr(win, "log"):
            win.log(msg)

    def _show_error(self, title: str, msg: str) -> None:
        styled_warning(self, title, msg)
        self._on_log(f"✗ {msg}")

    def _show_info(self, title: str, msg: str) -> None:
        styled_info(self, title, msg)
        self._on_log(f"✓ {msg}")

    def _require_connected(self) -> bool:
        if not self._bridge.connected_ok:
            self._show_error("未连接", "须先建立 SSH 连接。")
            return False
        return True

    def _require_unlocked(self, action: str) -> bool:
        if self._bridge.locked:
            self._show_error("车辆已锁定", f"{action}：须先解锁。")
            return False
        return True

    def _set_lock_style(self, locked: bool) -> None:
        self.lock_btn.blockSignals(True)
        self.lock_btn.setChecked(locked)
        self.lock_btn.blockSignals(False)
        if locked:
            self.lock_btn.setText("解锁")
            self.lock_btn.setStyleSheet(
                "QPushButton { background: #7a1f1f; color: white; font-weight: bold; }"
            )
            self.lock_status.setText("已锁")
            self.lock_status.setStyleSheet("color: #c0392b; font-weight: bold;")
        else:
            self.lock_btn.setText("锁车")
            self.lock_btn.setStyleSheet("")
            self.lock_status.setText("未锁")
            self.lock_status.setStyleSheet("color: #666;")

    def _on_locked_changed(self, locked: bool) -> None:
        self._set_lock_style(locked)

    def _on_lock_clicked(self, checked: bool) -> None:
        if not self._require_connected():
            self._set_lock_style(self._bridge.locked)
            return
        ok = self._bridge.set_locked(checked)
        if not ok:
            self._set_lock_style(self._bridge.locked)
            return
        if checked:
            self._on_log("✓ 已锁，本软件不再下发运动")
        else:
            self._on_log("✓ 已解锁")

    def _on_connected(self, ok: bool) -> None:
        if ok:
            self.conn_status.setText("已连接")
            self.conn_status.setStyleSheet("color: green;")
            self._state = RealState.CONNECTED
        else:
            self.conn_status.setText("连接失败")
            self.conn_status.setStyleSheet("color: red;")

    def _on_connect_finished(self, ok: bool, kind: str, message: str) -> None:
        self._connecting = False
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("连接车辆")
        titles = {
            "net": "连不上：网络不通",
            "auth": "需要登录",
            "timeout": "连不上：超时",
            "other": "连接失败",
            "ok": "连接成功",
        }
        if ok:
            self._show_info("连接成功", message)
            return
        if kind == "auth":
            self._ask_password_and_retry()
            return
        self._show_error(titles.get(kind, "连接失败"), message)

    def _ask_password_and_retry(self) -> None:
        pw = ask_password(
            self,
            "SSH 认证",
            "网络已连通。输入车辆系统密码。\n该密码仅用于本次会话，不写入磁盘。",
        )
        if not pw:
            self.conn_status.setText("未认证")
            self._on_log("已取消 SSH 认证。")
            return
        self._connecting = True
        self.connect_btn.setEnabled(False)
        self.connect_btn.setText("登录中…")
        self.conn_status.setText("正在登录…")
        self.conn_status.setStyleSheet("color: #d35400; font-weight: bold;")
        self._bridge.connect_async(password=pw, skip_ping=True)

    def _on_connect(self) -> None:
        if self._connecting:
            return
        host = self.host_edit.text().strip()
        if not host:
            self._show_error("错误", "请从本机私有配置或现场信息填写车辆地址。")
            return
        if "@" not in host:
            self._show_error(
                "错误", "请以“用户名@车辆主机”的形式填写本机私有连接目标。"
            )
            return
        user, host_addr = host.split("@", 1)
        if not user or not host_addr:
            self._show_error(
                "错误", "请以“用户名@车辆主机”的形式填写本机私有连接目标。"
            )
            return
        same, _hint = lan_hint_for_host(host_addr)
        if not same:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Warning)
            box.setWindowTitle("局域网未就绪")
            box.setText("工作站与车辆须接入同一现场局域网后才能访问车辆。")
            box.setInformativeText(
                "请检查本机私有网络配置或向现场管理员确认网络信息；接入完成后选择继续。"
            )
            apply_dialog_style(box)
            box.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            ok_btn = box.button(QMessageBox.Ok)
            cancel_btn = box.button(QMessageBox.Cancel)
            if ok_btn is not None:
                ok_btn.setText("继续")
            if cancel_btn is not None:
                cancel_btn.setText("取消")
            if box.exec() != QMessageBox.Ok:
                return
        self._bridge.set_target(user, host_addr)
        self._connecting = True
        self.connect_btn.setEnabled(False)
        self.connect_btn.setText("连接中…")
        self.conn_status.setText("正在连接…")
        self.conn_status.setStyleSheet("color: #d35400; font-weight: bold;")
        self._on_log("开始连接车辆")
        self._bridge.connect_async()

    def _on_check_result(self, name: str, ok: bool, detail: str) -> None:
        mark = "✓" if ok else "✗"
        self.check_status.setText(f"{mark} {name}：{detail}")
        self.check_status.setStyleSheet("color: green;" if ok else "color: #c0392b;")
        self._on_log(f"{mark} {name}：{detail}")

    def _run_factory(self, fn) -> None:
        if not self._require_connected() or not self._require_unlocked("厂方功能"):
            return
        ok, msg = fn()
        if not ok:
            self._show_error("厂方", msg)
            return
        self._on_log("✓ " + msg)

    def _on_start_lidar(self) -> None:
        self._run_factory(self._bridge.start_lidar)

    def _on_start_trav_map(self) -> None:
        if not self._require_connected():
            return
        ok, msg = self._bridge.start_trav_mapping()
        if not ok:
            self._show_error("建图", msg)
            return
        self._state = RealState.MAPPING
        self.map_status.setText("状态: 建图中")
        self._on_log("✓ " + msg)
        self._show_info("建图", msg)

    def _on_end_trav_map(self) -> None:
        if not self._require_connected():
            return
        dest = self.costmap_path_edit.text().strip() or self.local_map_edit.text().strip()
        ok, msg = self._bridge.end_trav_mapping(dest)
        if not ok:
            self._show_error("建图", msg)
            return
        self.local_map_edit.setText(dest)
        self._state = RealState.COSTMAP_SAVED
        self.map_status.setText("状态: 已保存")
        self._show_info("建图", msg)

    def _on_start_camera(self) -> None:
        if not self._require_connected():
            return
        ok, msg = self._bridge.start_camera()
        if not ok:
            self._show_error("相机", msg)
            return
        self._on_log("✓ " + msg)

    def _on_factory_map(self) -> None:
        self._run_factory(self._bridge.start_factory_mapping)
        if self._bridge.connected_ok:
            self._state = RealState.MAPPING

    def _on_factory_nav_start(self) -> None:
        self._run_factory(self._bridge.start_factory_nav)

    def _on_factory_nav_stop(self) -> None:
        if not self._require_connected():
            return
        ok, msg = self._bridge.stop_factory_stack()
        self._on_log(("✓ " if ok else "✗ ") + msg)

    def _on_check_nodes(self) -> None:
        if not self._require_connected():
            return
        self._bridge.check_nodes()

    def _on_check_topics(self) -> None:
        if not self._require_connected():
            return
        self._bridge.check_topics()

    def _on_check_tf(self) -> None:
        if not self._require_connected():
            return
        self._bridge.check_tf()

    def _on_check_storage(self) -> None:
        if not self._require_connected():
            return
        self._bridge.check_storage()

    def _on_load_costmap(self) -> None:
        path = Path(self.local_map_edit.text().strip())
        trav = path / "arena_trav.yaml"
        factory_like = path / "map.yaml"
        if trav.exists():
            self._state = RealState.COSTMAP_SAVED
            self._show_info("成功", f"已加载 costmap。\n路径：{trav}")
            return
        if factory_like.exists():
            self._show_error(
                "地图类型不匹配",
                "当前文件：厂方 gmapping map.yaml（2D 占用图）。\n"
                "本流程输入：maps_long/arena_trav.yaml（可通行性代价地图）。",
            )
            return
        if not path.exists():
            self._show_error("错误", f"本机 Costmap 目录不存在:\n{path}")
            return
        self._show_error(
            "未找到 arena_trav.yaml",
            f"输入路径：{path}\n"
            "所需文件：arena_trav.yaml",
        )

    def _on_boundary(self) -> None:
        path = Path(self.local_map_edit.text().strip())
        if not (path / "arena_trav.yaml").exists():
            self._show_error(
                "错误",
                "须先加载 maps_long/arena_trav.yaml。\n"
                "厂方 gmapping 的 map.yaml 不可用于本流程。",
            )
            return
        self._state = RealState.COSTMAP_SAVED
        self._boundary.begin_edit()
        self._coverage_ok = False
        self._update_boundary_status()
        self._on_log("已进入安全边界设置。顶点工具：仿真 RViz Publish Point。")
        self._show_info(
            "安全边界",
            "状态：设置中。\n"
            "顶点工具：仿真 RViz Publish Point。\n"
            "本面板：撤销、重设、闭合。",
        )

    def _update_boundary_status(self) -> None:
        if self._boundary.closed:
            text = f"边界状态: 已闭合（{self._boundary.point_count()} 顶点）"
        elif self._boundary.editing:
            text = f"边界状态: 设置中（{self._boundary.point_count()} 顶点）"
        else:
            text = "边界状态: 未开始"
        self.boundary_status.setText(text)

    def _on_undo_point(self) -> None:
        ok, msg = self._boundary.undo_last()
        if not ok:
            self._show_error("提示", msg)
            return
        self._coverage_ok = False
        self._update_boundary_status()
        self._on_log(msg)

    def _on_reset_boundary(self) -> None:
        ok, msg = self._boundary.reset()
        if not ok:
            self._show_error("提示", msg)
            return
        self._coverage_ok = False
        self._update_boundary_status()
        self._on_log(msg)

    def _on_close_boundary(self) -> None:
        ok, msg = self._boundary.close()
        if not ok:
            self._show_error("安全边界", msg)
            return
        self._update_boundary_status()
        self._on_log(msg)

    def _on_coverage(self) -> None:
        if not self._boundary.closed:
            self._show_error("检测失败", "安全边界未闭合。须先执行「闭合安全边界」。")
            return
        map_dir = Path(self.local_map_edit.text().strip())
        if not map_dir.exists():
            self._show_error("错误", "须先加载 maps_long 目录。")
            return
        ratio, info = coverage_inside_boundary(map_dir, self._boundary.points())
        pct = ratio * 100.0
        self._on_log(f"边界内覆盖率: {pct:.1f}%（{info}）")
        if ratio < 0.70:
            self._coverage_ok = False
            self._show_error("检测失败", f"覆盖率 {pct:.1f}% 低于 70%。须扩大边界或补扫。")
            return
        self._coverage_ok = True
        self._show_info("检测成功", f"边界内覆盖率 {pct:.1f}% ≥ 70%。")

    def _on_save_security(self) -> None:
        if not self._boundary.closed:
            self._show_error("无法保存", "须先闭合安全边界（无交叉）。")
            return
        if not self._coverage_ok:
            self._show_error("无法保存", "须先完成覆盖率检查。")
            return
        out = Path(self.security_path_edit.text().strip())
        if not out:
            self._show_error("无法保存", "须填写安全 costmap 保存路径。")
            return
        out.mkdir(parents=True, exist_ok=True)
        note = out / "boundary_vertices.txt"
        pts = self._boundary.points()
        note.write_text("\n".join(f"{x:.4f} {y:.4f}" for x, y in pts), encoding="utf-8")
        self._show_info("成功", f"已写入顶点。\n路径：{note}\n栅格烘焙：仿真流程。")

    def _on_safety_toggle(self, key: str) -> None:
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
        self._on_log(f"{key} 检测: {state}（实车 SSH 抽样，阈值与仿真 SafetyMonitor 相同）")

    def _on_safety_alarm(self, level: str, message: str) -> None:
        if level == "critical":
            self.enable_cmd_btn.setChecked(False)
            self.enable_cmd_label.setText("速度输出: 关")
            self._show_error("安全监测", message)
        else:
            self._on_log("⚠ " + message)

    def _on_enable_cmd(self, checked: bool) -> None:
        if not self._require_connected() or not self._require_unlocked("使能速度输出"):
            self.enable_cmd_btn.setChecked(False)
            return
        if not self._bridge.set_enable_cmd_vel(checked):
            self.enable_cmd_btn.setChecked(False)
            if checked:
                self._show_error(
                    "速度输出",
                    "本模式速度输出为关闭。不向 /cmd_vel 发布。\n"
                    "平面点到点：厂方导航，车辆端 RViz 工具 2D Nav Goal。\n"
                    "TD3 可视化：仿真模式 RViz。",
                )
            return
        self.enable_cmd_label.setText("速度输出: 关")

    def _on_start_nav(self) -> None:
        if not self._require_connected() or not self._require_unlocked("开始导航"):
            return
        model = self._model_loader.loaded
        if not model:
            self._show_error("错误", "须先加载模型。")
            return
        if model.get("simulation_only", False):
            self._on_log(
                f"已阻止实车导航：{model.get('name', '当前模型')} 仅允许用于仿真。"
            )
            self._show_error(
                "不允许实车导航",
                "当前加载的模型仅用于仿真，已阻止向实车发起 TD3 导航请求。\n"
                "请改用经过实车验证并明确允许实车控制的模型。",
            )
            return
        ok, msg = self._bridge.start_navigation(model["path"])
        if not ok:
            self._show_error("无法导航", msg)
            return
        self._state = RealState.NAVIGATING
        self._show_info("导航", msg)

    def _on_stop_nav(self) -> None:
        if self._state != RealState.NAVIGATING:
            self._show_error("错误", "当前未在导航")
            return
        self._bridge.stop_navigation()
        self._state = RealState.COSTMAP_SAVED
        self._show_info("成功", "已停止 TD3 导航请求。\n速度输出：关闭。")

    def _on_record(self, checked: bool) -> None:
        if checked:
            if not self._require_connected():
                self.record_btn.setChecked(False)
                return
            ok, msg = self._bridge.start_recording(include_lidar=self.record_lidar_chk.isChecked())
            if not ok:
                self.record_btn.setChecked(False)
                self._show_error("无法记录", msg)
                return
            self.record_btn.setText("停止记录")
            self._show_info("数据记录", msg)
        else:
            ok, msg = self._bridge.stop_recording()
            self.record_btn.setText("开始记录")
            self._on_log(msg)

    def estop(self) -> None:
        self._bridge.estop()
        self.enable_cmd_btn.setChecked(False)
        self.enable_cmd_label.setText("速度输出: 关")
        self._state = RealState.CONNECTED
        self._show_info(
            "急停已执行",
            "本软件控车：禁止。\n遥控：SWB 拨至安全档。",
        )
