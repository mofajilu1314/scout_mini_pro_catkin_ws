"""工具函数。"""

from __future__ import annotations

from PySide6.QtGui import QColor
from PySide6.QtWidgets import QGraphicsDropShadowEffect, QWidget


def apply_shadow(widget: QWidget, color: str = "#7fd8c0", blur: int = 20, offset: int = 4) -> None:
    """给控件加柔和阴影（浮空效果）。注意：必须传 parent 防止被 GC。"""
    effect = QGraphicsDropShadowEffect(widget)
    effect.setBlurRadius(blur)
    effect.setColor(QColor(color))
    effect.setOffset(0, offset)
    widget.setGraphicsEffect(effect)
