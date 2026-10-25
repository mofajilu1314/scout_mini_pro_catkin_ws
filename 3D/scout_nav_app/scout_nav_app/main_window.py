"""主窗口：模式选择、模型加载、终端日志、安全监测。"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, Signal, QTimer, QRect, QSize
from PySide6.QtGui import QAction, QColor, QFont, QIcon, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QDockWidget,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStackedWidget,
    QStatusBar,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .dialogs import apply_dialog_style
from .model_loader import ModelLoader
from .sim_panel import SimPanel
from .real_panel import RealPanel
from .terminal_widget import TerminalWidget
from .utils import apply_shadow
from .dep_check import run_dep_checks, format_report, agent_log_deps
from .event_monitor import EventMonitor


APP_NAME = "ScoutNav"
APP_TITLE = "ScoutNav · 越野智能导航"
APP_TITLE_SHORT = "SN"


class BatteryChip(QWidget):
    """Quiet phone-like battery mark for the header."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._text = "未连接"
        self._pct = -1
        self.setFixedHeight(28)
        self.setMinimumWidth(96)
        self.setToolTip("小车电量。连上之后读取车上电压。")

    def set_battery(self, text: str, pct: int, tip: str = "") -> None:
        self._text = text or "未连接"
        self._pct = int(pct)
        self.setToolTip(tip or "小车电量")
        self.update()
        self.setFixedWidth(max(96, 40 + len(self._text) * 12))

    def sizeHint(self) -> QSize:
        return QSize(self.width() or 96, 28)

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if self._pct < 0:
            fill = QColor("#b7ccc7")
            textc = QColor("#6a8a85")
            body = 0
        elif self._pct <= 15:
            fill = QColor("#e07a5f")
            textc = QColor("#c0392b")
            body = self._pct
        elif self._pct <= 30:
            fill = QColor("#e9a825")
            textc = QColor("#b36b00")
            body = self._pct
        else:
            fill = QColor("#5bc0a8")
            textc = QColor("#2c5f5a")
            body = self._pct
        x, y, w, h = 4, 8, 22, 12
        p.setPen(QPen(QColor("#7aa39c"), 1.2))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(x, y, w, h, 2, 2)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#7aa39c"))
        p.drawRoundedRect(x + w, y + 3, 3, h - 6, 1, 1)
        if body > 0:
            inner = max(1, int((w - 4) * body / 100.0))
            p.setBrush(fill)
            p.drawRoundedRect(x + 2, y + 2, inner, h - 4, 1, 1)
        p.setPen(textc)
        font = QFont(self.font())
        font.setPointSize(11)
        p.setFont(font)
        p.drawText(
            QRect(x + w + 8, 0, self.width() - w - 12, self.height()),
            Qt.AlignVCenter | Qt.AlignLeft,
            self._text,
        )


class MainWindow(QMainWindow):
    mode_changed = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1600, 1000)
        self.setMinimumSize(320, 240)  # 允许缩到很小

        # 设置图标
        icon_path = Path(__file__).parent.parent / "resources" / "icons" / "scout_icon.png"
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))

        self._model_loader = ModelLoader()
        self._current_mode = "sim"
        self._compact_mode = False

        self._build_ui()
        self._apply_theme()

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(0)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # 左侧：主内容区
        self.left_widget = QWidget()
        left_layout = QVBoxLayout(self.left_widget)
        left_layout.setSpacing(12)
        left_layout.setContentsMargins(16, 16, 16, 16)

        # 顶部：Logo + 标题 + 工具栏
        self.header_widget = QWidget()
        header = QHBoxLayout(self.header_widget)
        header.setContentsMargins(0, 0, 0, 0)

        # Logo
        self.logo_label = QLabel()
        icon_path = Path(__file__).parent.parent / "resources" / "icons" / "scout_icon.png"
        if icon_path.exists():
            pixmap = QPixmap(str(icon_path)).scaled(56, 56, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.logo_label.setPixmap(pixmap)
        header.addWidget(self.logo_label)

        # 标题
        self.title_label = QLabel(APP_TITLE)
        self.title_label.setObjectName("title")
        title_font = QFont()
        title_font.setPointSize(20)
        title_font.setBold(True)
        self.title_label.setFont(title_font)
        header.addWidget(self.title_label)

        self.battery_chip = BatteryChip()
        header.addWidget(self.battery_chip)

        header.addStretch()

        # 置顶按钮（无阴影）
        self.always_on_top_btn = QPushButton("置顶")
        self.always_on_top_btn.setCheckable(True)
        self.always_on_top_btn.setObjectName("top")
        self.always_on_top_btn.clicked.connect(self._on_always_on_top)
        header.addWidget(self.always_on_top_btn)

        # 模式选择
        header.addWidget(QLabel("模式:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["仿真", "实车"])
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        header.addWidget(self.mode_combo)

        # 模型选择
        header.addWidget(QLabel("模型:"))
        self.model_combo = QComboBox()
        self.model_combo.setMinimumWidth(220)
        self.model_combo.currentTextChanged.connect(self._on_model_changed)
        header.addWidget(self.model_combo)

        self.load_model_btn = QPushButton("加载模型")
        self.load_model_btn.setObjectName("primary")
        self.load_model_btn.clicked.connect(self._on_load_model)
        header.addWidget(self.load_model_btn)

        # 急停按钮（大 + 阴影）
        self.estop_btn = QPushButton("急停")
        self.estop_btn.setObjectName("estop")
        self.estop_btn.setMinimumSize(140, 72)
        self.estop_btn.clicked.connect(self._on_estop)
        header.addWidget(self.estop_btn)

        left_layout.addWidget(self.header_widget)

        # 中央：模式面板
        self.stack = QStackedWidget()
        left_layout.addWidget(self.stack, stretch=1)

        self.sim_panel = SimPanel(self._model_loader)
        self.real_panel = RealPanel(self._model_loader)
        self.stack.addWidget(self.sim_panel)
        self.stack.addWidget(self.real_panel)
        self.real_panel.bridge.battery_changed.connect(self._on_battery)
        self.real_panel.bridge.connected.connect(self._on_real_link)

        # 右侧：终端日志侧边栏（只读展示）
        self.terminal = TerminalWidget()
        self.terminal.setMinimumWidth(380)
        self.terminal.setMaximumWidth(480)
        self.terminal.setObjectName("terminal")

        # 用 Splitter 让用户可调宽度
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.addWidget(self.left_widget)
        self.splitter.addWidget(self.terminal)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 0)
        self.splitter.setSizes([1200, 400])

        main_layout.addWidget(self.splitter)

        # 状态栏
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("就绪")

        # 给主要按钮加阴影（已按用户要求去掉）
        # apply_shadow(self.load_model_btn, "#5bc0a8", 20, 4)
        # apply_shadow(self.estop_btn, "#ff6b4a", 28, 6)

        self._refresh_models()

        # 启动依赖自检 + 安全事件监视
        self._monitor = EventMonitor(self)
        self._monitor.event.connect(self.log)
        self.sim_panel.bind_monitor(self._monitor)
        self.real_panel.bind_monitor(self._monitor)
        QTimer.singleShot(300, self._run_startup_checks)

    def _run_startup_checks(self) -> None:
        items = run_dep_checks()
        agent_log_deps(items)
        ok, report = format_report(items)
        self.log(report)
        if not ok:
            self.log("✗ 关键依赖缺失：仿真功能可能无法启动，请先安装/ source ROS")
            QMessageBox.warning(self, "依赖自检", "关键依赖不完整，详见右侧终端。")
        else:
            self.log("✓ 关键依赖齐全")
        self._monitor.start()
        joy_line = next((i for i in items if i.name == "遥控器 USB"), None)
        if joy_line and not joy_line.ok:
            self.log("⚠ 建图前请插入 Flysky USB 并打开遥控器")

    def _apply_theme(self) -> None:
        # 薄荷蓝/绿 + 鲜橙主题，带阴影
        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #f0faf7, stop:1 #e0f2ee);
            }
            QLabel#title {
                color: #2c5f5a;
                padding: 4px 12px;
            }
            QLabel {
                color: #1a3a35;
            }
            QMessageBox, QInputDialog, QDialog {
                background-color: #f4faf8;
                color: #1a3a35;
            }
            QMessageBox QLabel, QInputDialog QLabel, QDialog QLabel {
                color: #1a3a35;
                background-color: transparent;
            }
            QInputDialog QLineEdit, QMessageBox QLineEdit {
                background-color: #ffffff;
                color: #1a3a35;
            }
            QCheckBox {
                color: #1a3a35;
                spacing: 8px;
            }
            QPushButton {
                background: #4eae98;
                border: none;
                border-radius: 10px;
                padding: 8px 14px;
                color: white;
                font-weight: 600;
                font-size: 13px;
                min-height: 34px;
                max-height: 42px;
            }
            QPushButton:hover {
                background: #429987;
            }
            QPushButton:pressed {
                background: #378675;
            }
            QPushButton:disabled {
                background: #c5ddd7;
                color: #8aa39e;
            }
            QPushButton#primary {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #6ed0b8, stop:1 #5bc0a8);
                color: white;
                max-height: 48px;
            }
            QPushButton#primary:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #5ec0a8, stop:1 #4aaf97);
            }
            QPushButton#estop {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #ffb380, stop:1 #ff9f68);
                color: white;
                font-weight: bold;
                font-size: 20px;
                border-radius: 14px;
                min-height: 72px;
                max-height: 16777215;
                padding: 12px 24px;
            }
            QPushButton#estop:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #ffa366, stop:1 #ff8f4f);
            }
            QPushButton#estop:pressed {
                background: #e87f3f;
            }
            QPushButton#top {
                background: #a8e0d0;
                color: #1a3a35;
                font-size: 12px;
                padding: 8px 16px;
            }
            QPushButton#top:checked {
                background: #5bc0a8;
                color: white;
            }
            QLineEdit {
                padding: 10px 14px;
                border: 2px solid #bfe3da;
                border-radius: 8px;
                background: white;
                color: #1a3a35;
                font-size: 13px;
            }
            QLineEdit:focus {
                border-color: #7fd8c0;
            }
            QComboBox {
                padding: 10px 14px;
                border: 2px solid #bfe3da;
                border-radius: 8px;
                background: white;
                color: #1a3a35;
                min-width: 120px;
                font-size: 13px;
            }
            QComboBox:hover {
                border-color: #7fd8c0;
            }
            QComboBox::drop-down {
                border: none;
                width: 28px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 6px solid transparent;
                border-right: 6px solid transparent;
                border-top: 7px solid #5bc0a8;
            }
            QComboBox QAbstractItemView {
                background: white;
                color: #1a3a35;
                selection-background-color: #7fd8c0;
                selection-color: white;
                border: 1px solid #bfe3da;
                border-radius: 6px;
            }
            QGroupBox {
                border: 2px solid #d0ebe5;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 12px;
                font-weight: 500;
                color: #2c5f5a;
                background: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 10px;
                background: white;
                border-radius: 4px;
            }
            QScrollArea {
                background: #f0faf7;
                border: none;
            }
            QScrollArea > QWidget > QWidget {
                background: #f0faf7;
            }
            QStatusBar {
                background: #e0f2ee;
                color: #2c5f5a;
                padding: 4px;
            }
            QSplitter::handle {
                background: #d0ebe5;
                width: 3px;
            }
            QSplitter::handle:hover {
                background: #7fd8c0;
            }
            QWidget#terminal {
                background: #f0faf7;
            }
        """)

    def _refresh_models(self) -> None:
        self.model_combo.clear()
        models = self._model_loader.list_models()
        for m in models:
            self.model_combo.addItem(m["name"], m)
        if models:
            self._on_model_changed(models[0]["name"])

    def _on_mode_changed(self, text: str) -> None:
        self._current_mode = "sim" if text == "仿真" else "real"
        self.stack.setCurrentIndex(0 if self._current_mode == "sim" else 1)
        self.mode_changed.emit(self._current_mode)
        self.log(f"切换到 {text} 模式")
        if self._current_mode == "sim" and not self.real_panel.bridge.connected_ok:
            self.battery_chip.set_battery("未连接", -1, "未连接小车，没有电量")

    def _on_battery(self, text: str, pct: int, tip: str) -> None:
        self.battery_chip.set_battery(text, pct, tip)

    def _on_real_link(self, ok: bool) -> None:
        if not ok:
            self.battery_chip.set_battery("未连接", -1, "未连接小车，没有电量")

    def _on_model_changed(self, name: str) -> None:
        model = self.model_combo.currentData()
        if model:
            self.status.showMessage(f"选中模型: {name} (state_dim={model.get('state_dim')})")

    def _on_load_model(self) -> None:
        model = self.model_combo.currentData()
        if not model:
            QMessageBox.warning(self, "模型", "请先选择模型")
            return
        self.log(f"正在加载模型: {model['name']} ...")
        ok, msg = self._model_loader.load(model["path"])
        if ok:
            self._show_message("模型", f"读取模型成功:\n{model['name']}\n\nstate_dim={self._model_loader.loaded.get('state_dim')}\nhidden_dim={self._model_loader.loaded.get('hidden_dim')}", QMessageBox.Information)
            self.log(f"✓ 模型加载成功: {model['name']} (state_dim={self._model_loader.loaded.get('state_dim')})")
        else:
            self._show_message("模型", f"读取模型失败:\n{msg}", QMessageBox.Critical)
            self.log(f"✗ 模型加载失败: {msg}")

    def _show_message(self, title: str, text: str, icon: QMessageBox.Icon) -> None:
        """显示消息框，确保文字完整显示。"""
        box = QMessageBox(self)
        box.setWindowTitle(title)
        box.setText(text)
        box.setIcon(icon)
        box.setStandardButtons(QMessageBox.Ok)
        box.setMinimumWidth(400)
        box.setMinimumHeight(150)
        box.setTextFormat(Qt.PlainText)
        apply_dialog_style(box)
        box.exec()

    def _on_estop(self) -> None:
        self.log("!!! 急停触发 !!!")
        panel = self.sim_panel if self._current_mode == "sim" else self.real_panel
        panel.estop()

    def _on_always_on_top(self, checked: bool) -> None:
        """置顶窗口：用 wmctrl 调用系统级置顶。"""
        import subprocess
        try:
            # 获取窗口 ID
            win_id = int(self.winId())
            if checked:
                # 使用 wmctrl 设置置顶
                subprocess.run(["wmctrl", "-i", "-r", str(win_id), "-b", "add,above"], check=False)
            else:
                subprocess.run(["wmctrl", "-i", "-r", str(win_id), "-b", "remove,above"], check=False)
            self.log(f"置顶: {'开' if checked else '关'}")
        except Exception as e:
            self.log(f"置顶失败: {e}")

    def log(self, text: str) -> None:
        """输出到终端侧边栏。"""
        self.terminal.append(text)
        self.status.showMessage(text, 3000)

    def resizeEvent(self, event) -> None:
        """根据窗口大小切换紧凑模式。"""
        super().resizeEvent(event)
        width = event.size().width()
        height = event.size().height()

        # 紧凑模式：宽度 < 800 或高度 < 600
        compact = width < 800 or height < 600
        if compact != self._compact_mode:
            self._compact_mode = compact
            self._apply_compact_mode(compact)

    def _apply_compact_mode(self, compact: bool) -> None:
        """紧凑模式：隐藏标题、终端，只留图标和急停。"""
        if compact:
            self.title_label.hide()
            self.terminal.hide()
            self.mode_combo.hide()
            self.model_combo.hide()
            self.load_model_btn.hide()
            self.always_on_top_btn.hide()
            # 急停保持可见但缩小，并去掉阴影防止被截断
            self.estop_btn.setMinimumSize(80, 48)
            self.estop_btn.setMaximumSize(100, 56)
            self.estop_btn.setGraphicsEffect(None)  # 紧凑模式去掉阴影
            self.battery_chip.show()
        else:
            self.title_label.show()
            self.terminal.show()
            self.mode_combo.show()
            self.model_combo.show()
            self.load_model_btn.show()
            self.always_on_top_btn.show()
            self.estop_btn.setMinimumSize(140, 72)
            self.estop_btn.setMaximumSize(16777215, 16777215)
            # 恢复阴影（已按用户要求去掉）
            # apply_shadow(self.estop_btn, "#ff6b4a", 28, 6)

    def closeEvent(self, event) -> None:
        """关闭窗口：先急停，再关闭。"""
        try:
            self._on_estop()
        except Exception:
            pass
        try:
            self.real_panel.bridge.shutdown()
        except Exception:
            pass
        event.accept()
