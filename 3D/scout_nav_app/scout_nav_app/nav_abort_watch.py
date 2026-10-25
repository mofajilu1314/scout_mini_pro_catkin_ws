"""订阅 /long_nav/nav_abort 与 /long_nav/goal_rejected → GUI 弹窗。"""

from __future__ import annotations

import re
from typing import Optional

from PySide6.QtCore import QObject, QProcess, Signal


def _parse_string_blocks(buf: str) -> tuple[str, list[str]]:
    """从 ros2 topic echo 缓冲中拆出 String.data 文本。"""
    texts: list[str] = []
    while "---" in buf:
        block, buf = buf.split("---", 1)
        m = re.search(r"data:\s*[\"']?(.*?)[\"']?\s*$", block, re.M | re.S)
        if not m:
            m = re.search(r"^data:\s*(.+)$", block, re.M)
        if m:
            text = m.group(1).strip().strip("\"'")
            if text:
                texts.append(text)
    return buf, texts


class NavAbortWatcher(QObject):
    """nav_abort → aborted；goal_rejected → rejected。"""

    aborted = Signal(str)
    rejected = Signal(str)
    status = Signal(str)

    def __init__(self, source_cmd: str, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._source = source_cmd
        self._abort_proc: Optional[QProcess] = None
        self._reject_proc: Optional[QProcess] = None
        self._abort_buf = ""
        self._reject_buf = ""

    def start(self) -> None:
        self.stop()
        self._abort_proc = self._spawn(
            "/long_nav/nav_abort", self._on_abort_out
        )
        self._reject_proc = self._spawn(
            "/long_nav/goal_rejected", self._on_reject_out
        )
        self.status.emit("正在监听安全边界越线中止 / 新终点拒绝…")

    def _spawn(self, topic: str, on_out) -> QProcess:
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.readyReadStandardOutput.connect(on_out)
        cmd = f"{self._source} && ros2 topic echo {topic} std_msgs/msg/String"
        proc.start("bash", ["-c", cmd])
        return proc

    def stop(self) -> None:
        for attr in ("_abort_proc", "_reject_proc"):
            proc = getattr(self, attr)
            if proc is not None:
                proc.kill()
                proc.waitForFinished(1000)
                setattr(self, attr, None)
        self._abort_buf = ""
        self._reject_buf = ""

    def _on_abort_out(self) -> None:
        if self._abort_proc is None:
            return
        chunk = bytes(self._abort_proc.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        self._abort_buf += chunk
        self._abort_buf, texts = _parse_string_blocks(self._abort_buf)
        for text in texts:
            self.aborted.emit(text)

    def _on_reject_out(self) -> None:
        if self._reject_proc is None:
            return
        chunk = bytes(self._reject_proc.readAllStandardOutput()).decode(
            "utf-8", errors="replace"
        )
        self._reject_buf += chunk
        self._reject_buf, texts = _parse_string_blocks(self._reject_buf)
        for text in texts:
            self.rejected.emit(text)
