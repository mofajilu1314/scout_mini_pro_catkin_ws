"""Light-theme dialogs: dark text on light background."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QLabel,
    QLineEdit,
    QMessageBox,
    QVBoxLayout,
    QWidget,
)

DIALOG_QSS = """
QDialog, QMessageBox {
    background-color: #f4faf8;
    color: #1a3a35;
}
QLabel {
    color: #1a3a35;
    background-color: transparent;
}
QLineEdit {
    background-color: #ffffff;
    color: #1a3a35;
    border: 2px solid #bfe3da;
    border-radius: 8px;
    padding: 8px 12px;
    selection-background-color: #4eae98;
    selection-color: #ffffff;
}
QPushButton {
    background-color: #4eae98;
    color: #ffffff;
    border: none;
    border-radius: 8px;
    padding: 8px 16px;
    min-height: 32px;
    max-height: 40px;
    font-weight: 600;
}
QPushButton:hover { background-color: #429987; }
QPushButton:pressed { background-color: #378675; }
"""


def apply_dialog_style(widget: QWidget) -> None:
    widget.setStyleSheet(DIALOG_QSS)


def styled_message(parent: Optional[QWidget], title: str, text: str, icon: QMessageBox.Icon) -> None:
    box = QMessageBox(parent)
    box.setWindowTitle(title)
    box.setText(text)
    box.setIcon(icon)
    box.setStandardButtons(QMessageBox.Ok)
    box.setTextFormat(Qt.PlainText)
    apply_dialog_style(box)
    box.exec()


def styled_warning(parent: Optional[QWidget], title: str, text: str) -> None:
    styled_message(parent, title, text, QMessageBox.Warning)


def styled_info(parent: Optional[QWidget], title: str, text: str) -> None:
    styled_message(parent, title, text, QMessageBox.Information)


def ask_password(parent: Optional[QWidget], title: str, prompt: str) -> Optional[str]:
    dlg = QDialog(parent)
    dlg.setWindowTitle(title)
    apply_dialog_style(dlg)
    dlg.setMinimumWidth(420)
    layout = QVBoxLayout(dlg)
    lab = QLabel(prompt)
    lab.setWordWrap(True)
    layout.addWidget(lab)
    edit = QLineEdit()
    edit.setEchoMode(QLineEdit.Password)
    layout.addWidget(edit)
    buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
    layout.addWidget(buttons)
    buttons.accepted.connect(dlg.accept)
    buttons.rejected.connect(dlg.reject)
    if dlg.exec() != QDialog.Accepted:
        return None
    text = edit.text()
    return text if text else None
