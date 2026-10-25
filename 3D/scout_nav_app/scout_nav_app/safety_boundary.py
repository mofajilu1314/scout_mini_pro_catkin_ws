"""安全边界：多边形顶点、自交检测、闭合。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class BoundaryPoint:
    x: float
    y: float


def _orient(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def _on_segment(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> bool:
    return (
        min(ax, bx) - 1e-9 <= cx <= max(ax, bx) + 1e-9
        and min(ay, by) - 1e-9 <= cy <= max(ay, by) + 1e-9
    )


def segments_intersect(
    a1: Tuple[float, float],
    a2: Tuple[float, float],
    b1: Tuple[float, float],
    b2: Tuple[float, float],
    *,
    allow_shared_endpoint: bool = True,
) -> bool:
    """Proper intersection (not mere shared endpoint when allow_shared_endpoint)."""
    (x1, y1), (x2, y2) = a1, a2
    (x3, y3), (x4, y4) = b1, b2
    if allow_shared_endpoint:
        for p in (a1, a2):
            for q in (b1, b2):
                if abs(p[0] - q[0]) < 1e-9 and abs(p[1] - q[1]) < 1e-9:
                    # shared vertex — only count if they also cross otherwise
                    pass
    o1 = _orient(x1, y1, x2, y2, x3, y3)
    o2 = _orient(x1, y1, x2, y2, x4, y4)
    o3 = _orient(x3, y3, x4, y4, x1, y1)
    o4 = _orient(x3, y3, x4, y4, x2, y2)

    def _nz(v: float) -> int:
        if v > 1e-12:
            return 1
        if v < -1e-12:
            return -1
        return 0

    s1, s2, s3, s4 = _nz(o1), _nz(o2), _nz(o3), _nz(o4)
    if s1 != s2 and s3 != s4:
        # exclude pure shared-endpoint cases
        if allow_shared_endpoint:
            shared = (
                (abs(x1 - x3) < 1e-9 and abs(y1 - y3) < 1e-9)
                or (abs(x1 - x4) < 1e-9 and abs(y1 - y4) < 1e-9)
                or (abs(x2 - x3) < 1e-9 and abs(y2 - y3) < 1e-9)
                or (abs(x2 - x4) < 1e-9 and abs(y2 - y4) < 1e-9)
            )
            if shared:
                return False
        return True
    # colinear overlap
    if s1 == 0 and _on_segment(x1, y1, x2, y2, x3, y3):
        if not (allow_shared_endpoint and (
            (abs(x3 - x1) < 1e-9 and abs(y3 - y1) < 1e-9)
            or (abs(x3 - x2) < 1e-9 and abs(y3 - y2) < 1e-9)
        )):
            return True
    if s2 == 0 and _on_segment(x1, y1, x2, y2, x4, y4):
        if not (allow_shared_endpoint and (
            (abs(x4 - x1) < 1e-9 and abs(y4 - y1) < 1e-9)
            or (abs(x4 - x2) < 1e-9 and abs(y4 - y2) < 1e-9)
        )):
            return True
    if s3 == 0 and _on_segment(x3, y3, x4, y4, x1, y1):
        if not (allow_shared_endpoint and (
            (abs(x1 - x3) < 1e-9 and abs(y1 - y3) < 1e-9)
            or (abs(x1 - x4) < 1e-9 and abs(y1 - y4) < 1e-9)
        )):
            return True
    if s4 == 0 and _on_segment(x3, y3, x4, y4, x2, y2):
        if not (allow_shared_endpoint and (
            (abs(x2 - x3) < 1e-9 and abs(y2 - y3) < 1e-9)
            or (abs(x2 - x4) < 1e-9 and abs(y2 - y4) < 1e-9)
        )):
            return True
    return False


class SafetyBoundary:
    def __init__(self) -> None:
        self._points: List[BoundaryPoint] = []
        self._closed = False
        self._editing = False

    def begin_edit(self) -> None:
        self._points.clear()
        self._closed = False
        self._editing = True

    def end_edit(self) -> None:
        self._editing = False

    @property
    def editing(self) -> bool:
        return self._editing

    @property
    def closed(self) -> bool:
        return self._closed

    def add_point(self, x: float, y: float) -> Tuple[bool, str]:
        if not self._editing:
            return False, "当前未在设置安全边界。须先执行「开始设置安全边界」。"
        if self._closed:
            return False, "边界已闭合。须先执行「重设安全边界」后再加点。"
        self._points.append(BoundaryPoint(x, y))
        return True, f"已添加顶点 #{len(self._points)} ({x:.2f}, {y:.2f})"

    def undo_last(self) -> Tuple[bool, str]:
        if not self._editing:
            return False, "当前未在设置安全边界，无法撤销"
        if not self._points:
            return False, "没有可撤销的顶点"
        self._points.pop()
        self._closed = False
        return True, f"已撤销，剩余 {len(self._points)} 个顶点"

    def reset(self) -> Tuple[bool, str]:
        if not self._editing and not self._points:
            return False, "当前未在设置安全边界，无需重设"
        self._points.clear()
        self._closed = False
        # 保持 editing，便于继续画
        return True, "已清空顶点，可重新点选"

    def close(self) -> Tuple[bool, str]:
        if not self._editing:
            return False, "当前未在设置安全边界，无法闭合"
        if len(self._points) < 3:
            return False, f"至少需要 3 个顶点才能闭合（当前 {len(self._points)}）"
        if self._is_self_intersecting():
            return False, "安全边界存在交叉。须重新设置。"
        self._closed = True
        return True, f"安全边界已闭合（{len(self._points)} 顶点）"

    def _edges(self, closed: bool) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        pts = self.points()
        if len(pts) < 2:
            return []
        edges = [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        if closed and len(pts) >= 3:
            edges.append((pts[-1], pts[0]))
        return edges

    def _is_self_intersecting(self) -> bool:
        edges = self._edges(closed=True)
        n = len(edges)
        for i in range(n):
            for j in range(i + 1, n):
                # skip adjacent edges (share vertex)
                if abs(i - j) <= 1:
                    continue
                if i == 0 and j == n - 1:
                    continue
                if segments_intersect(edges[i][0], edges[i][1], edges[j][0], edges[j][1]):
                    return True
        return False

    def is_valid(self) -> bool:
        return self._closed and len(self._points) >= 3

    def points(self) -> List[Tuple[float, float]]:
        return [(p.x, p.y) for p in self._points]

    def point_count(self) -> int:
        return len(self._points)
