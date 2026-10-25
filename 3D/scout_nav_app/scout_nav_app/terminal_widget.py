"""终端日志展示：把 launch/SSH 输出实时显示（只读）。"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QProcess, Qt, Signal
from PySide6.QtGui import QFont, QTextCursor, QTextOption
from PySide6.QtWidgets import QLabel, QTextEdit, QVBoxLayout, QWidget


class TerminalWidget(QWidget):
    """只读终端：展示命令流程，不可编辑。"""

    output_received = Signal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._build_ui()
        self._process: Optional[QProcess] = None

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # 标题
        title = QLabel("终端流程")
        title.setObjectName("terminalTitle")
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(11)
        title.setFont(title_font)
        layout.addWidget(title)

        # 只读终端
        self._view = QTextEdit()
        self._view.setReadOnly(True)
        self._view.setFont(QFont("Monospace", 9))
        self._view.setLineWrapMode(QTextEdit.WidgetWidth)  # 自动换行
        self._view.setWordWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)  # 在单词边界或任意处换行
        layout.addWidget(self._view)

        # 样式
        self.setStyleSheet("""
            TerminalWidget {
                background: #f0faf7;
            }
            QLabel#terminalTitle {
                color: #2c5f5a;
                padding: 6px 10px;
                background: #d0ebe5;
                border-radius: 6px;
                font-weight: bold;
            }
            QTextEdit {
                background: white;
                color: #1a3a35;
                border: 1px solid #bfe3da;
                border-radius: 8px;
                padding: 10px;
                selection-background-color: #7fd8c0;
                selection-color: white;
            }
            QScrollBar:vertical {
                background: #f0faf7;
                width: 10px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background: #bfe3da;
                border-radius: 5px;
                min-height: 20px;
            }
            QScrollBar::handle:vertical:hover {
                background: #7fd8c0;
            }
        """)

    def run(self, cmd: list[str]) -> None:
        """运行命令并展示输出。"""
        self.append(f"$ {' '.join(cmd)}")
        self._process = QProcess(self)
        self._process.readyReadStandardOutput.connect(self._on_stdout)
        self._process.readyReadStandardError.connect(self._on_stderr)
        self._process.start(cmd[0], cmd[1:])

    def _on_stdout(self) -> None:
        if self._process:
            text = self._process.readAllStandardOutput().data().decode()
            self.append(text.rstrip())

    def _on_stderr(self) -> None:
        if self._process:
            text = self._process.readAllStandardError().data().decode()
            self.append(f"[stderr] {text.rstrip()}")

    def append(self, text: str) -> None:
        """追加文本并滚动到底部。"""
        self._view.append(text)
        cursor = self._view.textCursor()
        cursor.movePosition(QTextCursor.End)
        self._view.setTextCursor(cursor)

    def clear(self) -> None:
        self._view.clear()
