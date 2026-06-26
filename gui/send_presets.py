# 说明：常用报文（独立窗口）
# 用途：保存多组常用发送配置（名称/通道/ID/扩展/帧类型/数据），方便快速切换并发送；
#       支持新增/编辑/删除、保存到 JSON、从 JSON 加载、填入主发送窗口。
import json
from typing import Optional, List, Dict, Callable

from PySide6.QtWidgets import (
    QWidget, QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QComboBox,
    QPushButton, QCheckBox, QTableWidget, QTableWidgetItem, QFormLayout,
    QDialogButtonBox, QMessageBox, QFileDialog, QHeaderView,
)
from PySide6.QtCore import Qt

FRAME_TYPES = ["CAN", "CAN FD", "CAN FD+BRS"]


def _parse_data(text: str) -> List[int]:
    """把十六进制字符串（空格分隔）解析为字节列表，并校验范围。"""
    out = []
    for p in str(text).split():
        v = int(p, 16)
        if not (0 <= v <= 0xFF):
            raise ValueError(f"数据字节超出范围: {p}")
        out.append(v)
    return out


class PresetEditDialog(QDialog):
    """新增/编辑一条发送预设。"""

    def __init__(self, parent=None, preset: Optional[dict] = None, channels: Optional[List[int]] = None):
        super().__init__(parent)
        self.setWindowTitle("发送预设")
        form = QFormLayout(self)

        self.name_edit = QLineEdit(preset.get("name", "") if preset else "")
        self.channel_combo = QComboBox(); self.channel_combo.setEditable(True)
        ch_items = [str(c) for c in (channels or [])]
        self.channel_combo.addItems(ch_items)
        self.id_edit = QLineEdit(preset.get("id", "123") if preset else "123")
        self.ext_check = QCheckBox("扩展帧")
        self.frame_combo = QComboBox(); self.frame_combo.addItems(FRAME_TYPES)
        self.data_edit = QLineEdit(preset.get("data", "DE AD BE EF") if preset else "DE AD BE EF")

        if preset:
            self.channel_combo.setCurrentText(str(preset.get("channel", 0)))
            self.ext_check.setChecked(bool(preset.get("is_extended", False)))
            ft = preset.get("frame_type", "CAN")
            i = self.frame_combo.findText(ft)
            if i >= 0:
                self.frame_combo.setCurrentIndex(i)
        elif ch_items:
            self.channel_combo.setCurrentText(ch_items[0])
        else:
            self.channel_combo.setCurrentText("0")

        form.addRow("名称:", self.name_edit)
        form.addRow("通道:", self.channel_combo)
        form.addRow("ID (Hex):", self.id_edit)
        form.addRow("", self.ext_check)
        form.addRow("帧类型:", self.frame_combo)
        form.addRow("数据 (Hex):", self.data_edit)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        form.addRow(btns)

    def accept(self):
        try:
            self.get_preset()
        except ValueError as e:
            QMessageBox.warning(self, "无效输入", str(e))
            return
        super().accept()

    def get_preset(self) -> dict:
        int(self.id_edit.text().strip(), 16)        # 校验 ID
        _parse_data(self.data_edit.text())           # 校验数据
        try:
            channel = int(self.channel_combo.currentText().strip())
        except ValueError:
            raise ValueError("通道必须是整数")
        name = self.name_edit.text().strip()
        return {
            "name": name or f"ID {self.id_edit.text().strip()}",
            "channel": channel,
            "id": self.id_edit.text().strip(),
            "is_extended": self.ext_check.isChecked(),
            "frame_type": self.frame_combo.currentText(),
            "data": self.data_edit.text().strip(),
        }


class SendPresetWindow(QWidget):
    """常用报文：预设列表 + 快速发送 + 保存/加载。"""

    def __init__(self, communicators: dict, parent=None,
                 load_to_main: Optional[Callable[[dict], None]] = None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("常用报文")
        self.resize(760, 420)
        self.communicators = communicators      # 与主窗口共享的引用，通道列表实时
        self.load_to_main = load_to_main
        self.presets: List[dict] = []

        layout = QVBoxLayout(self)

        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["名称", "通道", "ID (Hex)", "扩展", "帧类型", "数据 (Hex)", "操作"])
        self.table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.cellDoubleClicked.connect(lambda r, c: self._edit(r))
        layout.addWidget(self.table)

        row1 = QHBoxLayout()
        for text, fn in [("新增", self._add), ("编辑", lambda: self._edit(self.table.currentRow())),
                         ("删除", self._delete), ("填入主发送", self._load_to_main)]:
            b = QPushButton(text); b.clicked.connect(fn); row1.addWidget(b)
        row1.addStretch()
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        save_btn = QPushButton("保存到文件"); save_btn.clicked.connect(self._save); row2.addWidget(save_btn)
        load_btn = QPushButton("从文件加载"); load_btn.clicked.connect(self._load); row2.addWidget(load_btn)
        row2.addStretch()
        layout.addLayout(row2)

    # --- 列表渲染 ---
    def _rebuild(self):
        self.table.setRowCount(0)
        for p in self.presets:
            row = self.table.rowCount()
            self.table.insertRow(row)
            ext = "是" if p.get("is_extended") else ""
            cells = [p.get("name", ""), str(p.get("channel", 0)), p.get("id", ""),
                     ext, p.get("frame_type", "CAN"), p.get("data", "")]
            for col, text in enumerate(cells):
                self.table.setItem(row, col, QTableWidgetItem(text))
            btn = QPushButton("发送")
            btn.clicked.connect(lambda _=False, pr=p: self.send_preset(pr))
            self.table.setCellWidget(row, 6, btn)

    def _connected_channels(self) -> List[int]:
        return sorted(self.communicators.keys())

    # --- 增删改 ---
    def _add(self):
        dlg = PresetEditDialog(self, channels=self._connected_channels())
        if dlg.exec() == QDialog.Accepted:
            self.presets.append(dlg.get_preset())
            self._rebuild()

    def _edit(self, row: int):
        if not (0 <= row < len(self.presets)):
            return
        dlg = PresetEditDialog(self, preset=self.presets[row], channels=self._connected_channels())
        if dlg.exec() == QDialog.Accepted:
            self.presets[row] = dlg.get_preset()
            self._rebuild()

    def _delete(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.presets):
            del self.presets[row]
            self._rebuild()

    def _load_to_main(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.presets) and self.load_to_main:
            self.load_to_main(self.presets[row])

    # --- 发送 ---
    def send_preset(self, p: dict):
        ch = int(p.get("channel", 0))
        comm = self.communicators.get(ch)
        if comm is None:
            QMessageBox.warning(self, "未连接", f"通道 {ch} 未连接，无法发送")
            return
        try:
            msg_id = int(p["id"], 16)
            data = _parse_data(p["data"])
            ft = p.get("frame_type", "CAN")
            comm.send_one(msg_id, data, bool(p.get("is_extended", False)),
                          "FD" in ft, "+BRS" in ft, channel=ch)
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))

    # --- 保存 / 加载 ---
    def _save(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存发送预设", "send_presets.json",
                                              "JSON Files (*.json);;All Files (*)",
                                              options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"version": 1, "presets": self.presets}, f, ensure_ascii=False, indent=2)
        except Exception as e:
            QMessageBox.warning(self, "保存失败", str(e))

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(self, "加载发送预设", "",
                                              "JSON Files (*.json);;All Files (*)",
                                              options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.presets = list(data.get("presets", []))
        except Exception as e:
            QMessageBox.warning(self, "加载失败", str(e))
            return
        self._rebuild()
