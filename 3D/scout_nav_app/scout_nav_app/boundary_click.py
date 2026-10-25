"""订阅 RViz Publish Point (/clicked_point) → 信号。"""

from __future__ import annotations

import re
import subprocess
from typing import Optional

from PySide6.QtCore import QObject, QProcess, Signal


class ClickedPointWatcher(QObject):
    point = Signal(float, float)
    status = Signal(str)

    def __init__(self, source_cmd: str, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._source = source_cmd
        self._proc: Optional[QProcess] = None
        self._buf = ""

    def start(self) -> None:
        self.stop()
        self._proc = QProcess(self)
        self._proc.setProcessChannelMode(QProcess.MergedChannels)
        self._proc.readyReadStandardOutput.connect(self._on_out)
        cmd = (
            f"{self._source} && ros2 topic echo /clicked_point geometry_msgs/msg/PointStamped"
        )
        self._proc.start("bash", ["-c", cmd])
        self.status.emit("正在监听 /clicked_point（RViz 用 Publish Point 加点）…")

    def stop(self) -> None:
        if self._proc is not None:
            self._proc.kill()
            self._proc.waitForFinished(1000)
            self._proc = None

    def _on_out(self) -> None:
        if self._proc is None:
            return
        chunk = bytes(self._proc.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._buf += chunk
        # parse blocks ending with ---
        while "---" in self._buf:
            block, self._buf = self._buf.split("---", 1)
            xs = re.findall(r"^\s*x:\s*([-\d.eE]+)", block, re.M)
            ys = re.findall(r"^\s*y:\s*([-\d.eE]+)", block, re.M)
            if xs and ys:
                # PointStamped has point: x/y under point
                # take last x/y in block (point section)
                try:
                    x = float(xs[-1])
                    y = float(ys[-1])
                    self.point.emit(x, y)
                except ValueError:
                    pass
