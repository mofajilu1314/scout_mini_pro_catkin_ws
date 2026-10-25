"""RViz 内嵌视图（占位：先以子进程方式启动 rviz）。"""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Optional

from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget


class RVizWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._process: Optional[subprocess.Popen] = None
        layout = QVBoxLayout(self)
        self._placeholder = QLabel("RViz 视图\n（点击「加载世界」后启动）")
        self._placeholder.setStyleSheet("color: #7a9; font-size: 14px;")
        layout.addWidget(self._placeholder)

    def start(self, config_path: Optional[Path] = None) -> None:
        if self._process is not None:
            return
        cmd = ["rviz2"]
        if config_path and config_path.exists():
            cmd += ["-d", str(config_path)]
        self._process = subprocess.Popen(cmd)
        self._placeholder.setText("RViz 已启动（外部窗口）")

    def stop(self) -> None:
        if self._process:
            self._process.terminate()
            self._process = None
        self._placeholder.setText("RViz 视图\n（点击「加载世界」后启动）")
