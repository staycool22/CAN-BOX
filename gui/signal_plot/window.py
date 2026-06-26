# 说明：独立的「信号解析与绘图」窗口（PySide6 + pyqtgraph）
# 用途：让没有 Python 基础的人用可视化表格按协议定义报文/信号，把实时接收的 CAN
#       报文解码成命名信号，并自由组合到多路实时曲线图里绘制。
# 该窗口只接收数据（由主 GUI 通过信号喂入），不做发送/编码。
import os
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")  # 强制 pyqtgraph 绑定 PySide6

import time
from collections import deque
from typing import Dict, List, Optional, Tuple

from PySide6.QtWidgets import (
    QWidget, QDialog, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QLineEdit,
    QComboBox, QPushButton, QCheckBox, QSpinBox, QTreeWidget, QTreeWidgetItem,
    QTableWidget, QTableWidgetItem, QFormLayout, QDialogButtonBox, QMessageBox,
    QFileDialog, QSplitter, QScrollArea, QDoubleSpinBox, QHeaderView, QListWidget,
    QListWidgetItem, QColorDialog,
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor

import pyqtgraph as pg

try:
    from .codec import CodecDatabase, MessageDef, SignalDef, Segment
except ImportError:
    import sys
    _repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    sys.path.insert(0, _repo_root)
    from gui.signal_plot.codec import CodecDatabase, MessageDef, SignalDef, Segment


def parse_number(text: str, default: float = 0.0) -> float:
    """解析数值，支持 '0.02' / '1/50' / '1e-6' 等写法，方便填写协议缩放系数。"""
    s = str(text).strip()
    if not s:
        return default
    if "/" in s:
        a, b = s.split("/", 1)
        return float(a) / float(b)
    return float(s)


_BYTE_ORDER_LABELS = ["大端 (Motorola)", "小端 (Intel)"]
_BYTE_ORDER_VALUES = ["big", "little"]


# ---------------------------------------------------------------------------
# 编辑对话框
# ---------------------------------------------------------------------------

class MessageEditDialog(QDialog):
    """编辑报文头（名称 / ID / 帧类型 / DLC）。"""

    def __init__(self, parent=None, msg: Optional[MessageDef] = None):
        super().__init__(parent)
        self.setWindowTitle("报文定义")
        form = QFormLayout(self)
        self.name_edit = QLineEdit(msg.name if msg else "")
        self.id_edit = QLineEdit(f"{msg.can_id:X}" if msg else "123")
        self.ext_check = QCheckBox("扩展帧")
        if msg:
            self.ext_check.setChecked(msg.is_extended)
        self.dlc_spin = QSpinBox()
        self.dlc_spin.setRange(0, 64)
        self.dlc_spin.setValue(msg.dlc if msg else 8)

        form.addRow("报文名称:", self.name_edit)
        form.addRow("仲裁 ID (Hex):", self.id_edit)
        form.addRow("", self.ext_check)
        form.addRow("DLC 长度:", self.dlc_spin)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        form.addRow(btns)

    def accept(self):
        if not self.name_edit.text().strip():
            QMessageBox.warning(self, "无效输入", "报文名称不能为空")
            return
        try:
            int(self.id_edit.text().strip(), 16)
        except ValueError:
            QMessageBox.warning(self, "无效输入", "仲裁 ID 必须是十六进制，如 123 或 0x18FF50E5")
            return
        super().accept()

    def get_values(self):
        return (
            self.name_edit.text().strip(),
            int(self.id_edit.text().strip(), 16),
            self.ext_check.isChecked(),
            self.dlc_spin.value(),
        )


class SignalEditDialog(QDialog):
    """编辑单个信号的位域定义与工程量换算。"""

    def __init__(self, parent=None, sig: Optional[SignalDef] = None):
        super().__init__(parent)
        self.setWindowTitle("信号定义")
        form = QFormLayout(self)

        self.name_edit = QLineEdit(sig.name if sig else "")
        self.start_byte_spin = QSpinBox(); self.start_byte_spin.setRange(0, 63)
        self.start_bit_spin = QSpinBox(); self.start_bit_spin.setRange(0, 7)
        self.bit_len_spin = QSpinBox(); self.bit_len_spin.setRange(1, 64)
        self.bit_len_spin.setValue(8)
        self.order_combo = QComboBox(); self.order_combo.addItems(_BYTE_ORDER_LABELS)
        self.type_combo = QComboBox(); self.type_combo.addItems(["整数", "IEEE-754 浮点"])
        self.signed_check = QCheckBox("有符号")
        self.scale_edit = QLineEdit("1")
        self.offset_edit = QLineEdit("0")
        self.unit_edit = QLineEdit("")

        if sig:
            self.start_byte_spin.setValue(sig.start_byte)
            self.start_bit_spin.setValue(sig.start_bit)
            self.bit_len_spin.setValue(sig.bit_length)
            try:
                self.order_combo.setCurrentIndex(_BYTE_ORDER_VALUES.index(sig.byte_order))
            except ValueError:
                pass
            self.type_combo.setCurrentIndex(1 if sig.is_float else 0)
            self.signed_check.setChecked(sig.signed)
            self.scale_edit.setText(repr(sig.scale))
            self.offset_edit.setText(repr(sig.offset))
            self.unit_edit.setText(sig.unit)

        form.addRow("信号名称:", self.name_edit)
        form.addRow("起始字节:", self.start_byte_spin)
        form.addRow("起始位 (0-7):", self.start_bit_spin)
        form.addRow("位长度:", self.bit_len_spin)
        form.addRow("字节序:", self.order_combo)
        form.addRow("数据类型:", self.type_combo)
        form.addRow("", self.signed_check)
        form.addRow("缩放系数:", self.scale_edit)
        form.addRow("偏移量:", self.offset_edit)
        form.addRow("单位:", self.unit_edit)

        # 多段拼接（可选）：把分散在帧里不连续的字节/位段按顺序拼成一个字段
        seg_group = QGroupBox("多段拼接（可选）")
        seg_v = QVBoxLayout(seg_group)
        self.seg_hint = QLabel()
        self.seg_hint.setWordWrap(True)
        seg_v.addWidget(self.seg_hint)
        self.seg_table = QTableWidget(0, 3)
        self.seg_table.setHorizontalHeaderLabels(["起始字节", "起始位", "位长"])
        self.seg_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.seg_table.verticalHeader().setVisible(False)
        self.seg_table.setMaximumHeight(150)
        seg_v.addWidget(self.seg_table)
        seg_btns = QHBoxLayout()
        add_seg = QPushButton("添加段"); add_seg.clicked.connect(lambda: self._add_seg_row())
        del_seg = QPushButton("删除选中段"); del_seg.clicked.connect(self._del_seg_row)
        seg_btns.addWidget(add_seg); seg_btns.addWidget(del_seg); seg_btns.addStretch()
        seg_v.addLayout(seg_btns)
        form.addRow(seg_group)

        if sig and sig.segments:
            for seg in sig.segments:
                self._add_seg_row(seg.start_byte, seg.start_bit, seg.bit_length)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        form.addRow(btns)

        # IEEE-754 浮点自带符号，「有符号」对浮点无意义——选浮点时置灰该项以免误解
        self.type_combo.currentIndexChanged.connect(self._sync_signed_enabled)
        self._sync_signed_enabled()
        self._update_seg_hint()

    def _sync_signed_enabled(self):
        is_float = self.type_combo.currentIndex() == 1
        self.signed_check.setEnabled(not is_float)
        self.signed_check.setToolTip("IEEE-754 浮点自带符号，无需设置" if is_float else "")

    def _add_seg_row(self, start_byte: int = 0, start_bit: int = 0, bit_length: int = 8):
        row = self.seg_table.rowCount()
        self.seg_table.insertRow(row)
        for col, (val, lo, hi) in enumerate(
            [(start_byte, 0, 63), (start_bit, 0, 7), (bit_length, 1, 64)]
        ):
            sb = QSpinBox(); sb.setRange(lo, hi); sb.setValue(val)
            if col == 2:
                sb.valueChanged.connect(self._update_seg_hint)
            self.seg_table.setCellWidget(row, col, sb)
        self._update_seg_hint()

    def _del_seg_row(self):
        row = self.seg_table.currentRow()
        if row < 0:
            row = self.seg_table.rowCount() - 1
        if row >= 0:
            self.seg_table.removeRow(row)
        self._update_seg_hint()

    def _read_segments(self) -> List[Segment]:
        segs = []
        for row in range(self.seg_table.rowCount()):
            segs.append(Segment(
                start_byte=self.seg_table.cellWidget(row, 0).value(),
                start_bit=self.seg_table.cellWidget(row, 1).value(),
                bit_length=self.seg_table.cellWidget(row, 2).value(),
            ))
        return segs

    def _update_seg_hint(self):
        n = self.seg_table.rowCount()
        if n == 0:
            self.seg_hint.setText("未定义段 → 使用上方单段字段（普通信号）")
        else:
            total = sum(self.seg_table.cellWidget(r, 2).value() for r in range(n))
            self.seg_hint.setText(
                f"已定义 {n} 段（从上到下 = 高位→低位），总位宽 {total} 位；"
                "多段模式下忽略上方的单段字段")

    def accept(self):
        try:
            self.get_signal()
        except (ValueError, ZeroDivisionError) as e:
            QMessageBox.warning(self, "无效输入", str(e))
            return
        super().accept()

    def get_signal(self) -> SignalDef:
        segs = self._read_segments()
        bit_length = sum(s.bit_length for s in segs) if segs else self.bit_len_spin.value()
        sig = SignalDef(
            name=self.name_edit.text().strip(),
            start_byte=self.start_byte_spin.value(),
            start_bit=self.start_bit_spin.value(),
            bit_length=bit_length,
            byte_order=_BYTE_ORDER_VALUES[self.order_combo.currentIndex()],
            signed=self.signed_check.isChecked(),
            is_float=(self.type_combo.currentIndex() == 1),
            scale=parse_number(self.scale_edit.text(), 1.0),
            offset=parse_number(self.offset_edit.text(), 0.0),
            unit=self.unit_edit.text().strip(),
            segments=segs,
        )
        sig.validate()
        return sig


def _auto_color_hex(i: int) -> str:
    """按索引生成一个默认曲线颜色（十六进制）。"""
    return pg.intColor(i, hues=9).name()


class SignalSelectDialog(QDialog):
    """勾选某个曲线图要绘制的信号，并可为每条曲线自定义颜色。"""

    def __init__(self, parent, available: List[str], selected: List[str],
                 colors: Optional[Dict[str, str]] = None):
        super().__init__(parent)
        self.setWindowTitle("选择要绘制的信号")
        self.resize(360, 420)
        colors = colors or {}
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("勾选要绘制的信号，点右侧色块可自定义颜色:"))

        self.table = QTableWidget(len(available), 2)
        self.table.setHorizontalHeaderLabels(["信号", "颜色"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._colors: Dict[str, str] = {}
        self._buttons: Dict[str, QPushButton] = {}
        for row, name in enumerate(available):
            item = QTableWidgetItem(name)
            item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            item.setCheckState(Qt.Checked if name in selected else Qt.Unchecked)
            self.table.setItem(row, 0, item)
            hexc = colors.get(name) or _auto_color_hex(row)
            self._colors[name] = hexc
            btn = QPushButton()
            btn.setFixedWidth(64)
            self._apply_btn_color(btn, hexc)
            btn.clicked.connect(lambda _=False, n=name: self._pick(n))
            self._buttons[name] = btn
            self.table.setCellWidget(row, 1, btn)
        layout.addWidget(self.table)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    @staticmethod
    def _apply_btn_color(btn: QPushButton, hexc: str):
        btn.setStyleSheet(f"background-color: {hexc}; border: 1px solid #888;")

    def _pick(self, name: str):
        # 非原生颜色对话框，规避 Linux/WSL 原生对话框卡死问题
        c = QColorDialog.getColor(QColor(self._colors[name]), self, "选择曲线颜色",
                                  QColorDialog.DontUseNativeDialog)
        if c.isValid():
            self._colors[name] = c.name()
            self._apply_btn_color(self._buttons[name], c.name())

    def get_result(self) -> Tuple[List[str], Dict[str, str]]:
        names: List[str] = []
        out_colors: Dict[str, str] = {}
        for row in range(self.table.rowCount()):
            item = self.table.item(row, 0)
            if item.checkState() == Qt.Checked:
                n = item.text()
                names.append(n)
                out_colors[n] = self._colors[n]
        return names, out_colors


# ---------------------------------------------------------------------------
# 单个曲线图窗格
# ---------------------------------------------------------------------------

class PlotPane(QWidget):
    """一个曲线图窗格：选择若干信号，实时绘制时间序列。"""

    def __init__(self, host: "SignalPlotWindow", index: int):
        super().__init__()
        self.host = host
        self.selected: List[str] = []
        self.colors: Dict[str, str] = {}  # 信号名 -> 十六进制颜色
        self.curves: Dict[str, pg.PlotDataItem] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        bar = QHBoxLayout()
        self.title_label = QLabel(f"曲线图 {index}")
        bar.addWidget(self.title_label)
        bar.addStretch()
        self.select_btn = QPushButton("选择信号...")
        self.select_btn.clicked.connect(self._choose_signals)
        bar.addWidget(self.select_btn)
        self.remove_btn = QPushButton("删除此图")
        self.remove_btn.clicked.connect(lambda: self.host.remove_pane(self))
        bar.addWidget(self.remove_btn)
        layout.addLayout(bar)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("w")
        self.plot_item = self.plot_widget.getPlotItem()
        self.plot_item.showGrid(x=True, y=True, alpha=0.3)
        self.plot_item.setLabel("bottom", "时间", units="s")
        self.plot_item.setClipToView(True)
        self.plot_item.setDownsampling(auto=True, mode="peak")
        self.legend = self.plot_item.addLegend()
        layout.addWidget(self.plot_widget)

    def _choose_signals(self):
        dlg = SignalSelectDialog(self, self.host.db.all_signal_names(), self.selected, self.colors)
        if dlg.exec() == QDialog.Accepted:
            names, colors = dlg.get_result()
            self.set_signals(names, colors)

    def set_signals(self, names: List[str], colors: Optional[Dict[str, str]] = None):
        self.selected = list(names)
        self.colors = dict(colors) if colors else {}
        self.plot_item.clear()
        if self.legend is not None:
            self.legend.clear()
        self.curves = {}
        for i, name in enumerate(self.selected):
            hexc = self.colors.get(name) or _auto_color_hex(i)
            self.colors[name] = hexc  # 确保每条选中曲线都有明确颜色（便于导出布局）
            pen = pg.mkPen(color=hexc, width=2)
            self.curves[name] = self.plot_item.plot([], [], pen=pen, name=name)

    def redraw(self, now_rel: float):
        buffers = self.host.buffers
        for name, curve in self.curves.items():
            buf = buffers.get(name)
            if buf:
                curve.setData([p[0] for p in buf], [p[1] for p in buf])
            else:
                curve.setData([], [])
        if self.host.auto_scroll and self.curves:
            self.plot_item.setXRange(max(0.0, now_rel - self.host.window_sec), now_rel, padding=0)


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------

class SignalPlotWindow(QWidget):
    """独立的信号解析与绘图窗口。"""

    closed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("信号解析与绘图")
        self.resize(1200, 760)

        self.db = CodecDatabase()
        self.latest: Dict[str, float] = {}
        self.buffers: Dict[str, deque] = {}
        self.value_rows: Dict[str, int] = {}
        self.panes: List[PlotPane] = []
        self.pane_counter = 0
        self._known_channels: set = set()  # 从收到的帧里自动学习到的通道名

        self.paused = False
        self.auto_scroll = True
        self.window_sec = 10.0
        self.t0: Optional[float] = None

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_left_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([400, 800])

        root = QVBoxLayout(self)
        root.addWidget(splitter)

        # 绘图刷新定时器（与逐帧解码解耦，约 25Hz）
        self.redraw_timer = QTimer(self)
        self.redraw_timer.setInterval(40)
        self.redraw_timer.timeout.connect(self._refresh)
        self.redraw_timer.start()

        self._rebuild_tree()
        self._rebuild_value_table()

    # --- 界面构建 ---

    def _build_left_panel(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        proto_group = QGroupBox("协议定义")
        pg_layout = QVBoxLayout(proto_group)
        self.tree = QTreeWidget()
        self.tree.setColumnCount(2)
        self.tree.setHeaderLabels(["名称", "详情"])
        self.tree.setColumnWidth(0, 160)
        pg_layout.addWidget(self.tree)

        row1 = QHBoxLayout()
        for text, fn in [("新建报文", self._add_message), ("新建信号", self._add_signal),
                         ("编辑", self._edit_item), ("删除", self._delete_item)]:
            b = QPushButton(text); b.clicked.connect(fn); row1.addWidget(b)
        pg_layout.addLayout(row1)

        row2 = QHBoxLayout()
        imp = QPushButton("导入协议"); imp.clicked.connect(self._import_protocol); row2.addWidget(imp)
        exp = QPushButton("导出协议"); exp.clicked.connect(self._export_protocol); row2.addWidget(exp)
        pg_layout.addLayout(row2)
        layout.addWidget(proto_group, 3)

        val_group = QGroupBox("实时信号值")
        vg_layout = QVBoxLayout(val_group)
        self.value_table = QTableWidget()
        self.value_table.setColumnCount(3)
        self.value_table.setHorizontalHeaderLabels(["信号", "值", "单位"])
        self.value_table.horizontalHeader().setStretchLastSection(True)
        self.value_table.setEditTriggers(QTableWidget.NoEditTriggers)
        vg_layout.addWidget(self.value_table)
        layout.addWidget(val_group, 2)
        return w

    def _build_right_panel(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        ctrl = QHBoxLayout()
        add_plot_btn = QPushButton("添加曲线图")
        add_plot_btn.clicked.connect(self.add_pane)
        ctrl.addWidget(add_plot_btn)

        self.pause_btn = QPushButton("暂停")
        self.pause_btn.clicked.connect(self._toggle_pause)
        ctrl.addWidget(self.pause_btn)

        clear_btn = QPushButton("清空数据")
        clear_btn.clicked.connect(self._clear_data)
        ctrl.addWidget(clear_btn)

        ctrl.addWidget(QLabel("数据源:"))
        self.source_combo = QComboBox()
        self.source_combo.addItem("全部通道", None)  # userData=None 表示不筛选
        ctrl.addWidget(self.source_combo)
        self.rx_only_check = QCheckBox("只看接收(RX)")
        ctrl.addWidget(self.rx_only_check)

        ctrl.addWidget(QLabel("时间窗口(s):"))
        self.window_spin = QDoubleSpinBox()
        self.window_spin.setRange(1.0, 600.0)
        self.window_spin.setValue(self.window_sec)
        self.window_spin.valueChanged.connect(self._on_window_changed)
        ctrl.addWidget(self.window_spin)

        self.autoscroll_check = QCheckBox("自动滚动")
        self.autoscroll_check.setChecked(True)
        self.autoscroll_check.toggled.connect(self._on_autoscroll_toggled)
        ctrl.addWidget(self.autoscroll_check)

        ctrl.addStretch()
        imp = QPushButton("导入布局"); imp.clicked.connect(self._import_layout); ctrl.addWidget(imp)
        exp = QPushButton("导出布局"); exp.clicked.connect(self._export_layout); ctrl.addWidget(exp)
        layout.addLayout(ctrl)

        self.plot_scroll = QScrollArea()
        self.plot_scroll.setWidgetResizable(True)
        self.plot_container = QWidget()
        self.plot_layout = QVBoxLayout(self.plot_container)
        self.plot_scroll.setWidget(self.plot_container)
        layout.addWidget(self.plot_scroll)

        self.add_pane()  # 默认给一个空曲线图
        return w

    # --- 协议编辑 ---

    def _selected_message(self) -> Optional[MessageDef]:
        item = self.tree.currentItem()
        if item is None:
            return None
        data = item.data(0, Qt.UserRole)
        if isinstance(data, MessageDef):
            return data
        if isinstance(data, tuple) and isinstance(data[0], MessageDef):
            return data[0]
        return None

    def _add_message(self):
        dlg = MessageEditDialog(self)
        if dlg.exec() == QDialog.Accepted:
            name, can_id, ext, dlc = dlg.get_values()
            self.db.add_message(MessageDef(name=name, can_id=can_id, is_extended=ext, dlc=dlc))
            self._on_db_changed()

    def _add_signal(self):
        msg = self._selected_message()
        if msg is None:
            QMessageBox.information(self, "提示", "请先在上方选中一条报文")
            return
        dlg = SignalEditDialog(self)
        if dlg.exec() == QDialog.Accepted:
            msg.signals.append(dlg.get_signal())
            self._on_db_changed()

    def _edit_item(self):
        item = self.tree.currentItem()
        if item is None:
            return
        data = item.data(0, Qt.UserRole)
        if isinstance(data, MessageDef):
            dlg = MessageEditDialog(self, data)
            if dlg.exec() == QDialog.Accepted:
                data.name, data.can_id, data.is_extended, data.dlc = dlg.get_values()
                self._on_db_changed()
        elif isinstance(data, tuple):
            msg, sig = data
            dlg = SignalEditDialog(self, sig)
            if dlg.exec() == QDialog.Accepted:
                new_sig = dlg.get_signal()
                idx = msg.signals.index(sig)
                msg.signals[idx] = new_sig
                self._on_db_changed()

    def _delete_item(self):
        item = self.tree.currentItem()
        if item is None:
            return
        data = item.data(0, Qt.UserRole)
        if isinstance(data, MessageDef):
            self.db.remove_message(data)
        elif isinstance(data, tuple):
            msg, sig = data
            if sig in msg.signals:
                msg.signals.remove(sig)
        self._on_db_changed()

    def _import_protocol(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入协议定义", "", "JSON Files (*.json);;All Files (*)",
                                              options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        try:
            self.db = CodecDatabase.load_json(path)
        except Exception as e:
            QMessageBox.warning(self, "导入失败", str(e))
            return
        self._clear_data()
        self._on_db_changed()

    def _export_protocol(self):
        path, _ = QFileDialog.getSaveFileName(self, "导出协议定义", "protocol.json", "JSON Files (*.json);;All Files (*)",
                                              options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        try:
            self.db.save_json(path)
        except Exception as e:
            QMessageBox.warning(self, "导出失败", str(e))

    def _on_db_changed(self):
        self.db.reindex()
        self._rebuild_tree()
        self._rebuild_value_table()
        # 清理已不存在的信号缓存
        valid = set(self.db.all_signal_names())
        for name in list(self.buffers.keys()):
            if name not in valid:
                del self.buffers[name]
        for name in list(self.latest.keys()):
            if name not in valid:
                del self.latest[name]

    def _rebuild_tree(self):
        self.tree.clear()
        for m in self.db.messages:
            frame = "扩展" if m.is_extended else "标准"
            top = QTreeWidgetItem([m.name, f"ID 0x{m.can_id:X} | {frame} | DLC {m.dlc}"])
            top.setData(0, Qt.UserRole, m)
            for s in m.signals:
                order = "大端" if s.byte_order == "big" else "小端"
                nbits = s.total_bits()
                if s.is_float:
                    type_str = f"float{nbits}"
                else:
                    type_str = "有符号" if s.signed else "无符号"
                if s.segments:
                    loc = f"[{len(s.segments)}段拼接] len{nbits}"
                else:
                    loc = f"byte{s.start_byte}.{s.start_bit} len{nbits}"
                detail = f"{loc} {order} {type_str} x{s.scale}+{s.offset} {s.unit}"
                child = QTreeWidgetItem([s.name, detail])
                child.setData(0, Qt.UserRole, (m, s))
                top.addChild(child)
            self.tree.addTopLevelItem(top)
        self.tree.expandAll()

    def _rebuild_value_table(self):
        names = self.db.all_signal_names()
        self.value_rows = {}
        self.value_table.setRowCount(len(names))
        for row, name in enumerate(names):
            msg = next((m for m in self.db.messages if name.startswith(m.name + ".")), None)
            unit = ""
            if msg:
                sig_name = name.split(".", 1)[1]
                sig = next((s for s in msg.signals if s.name == sig_name), None)
                if sig:
                    unit = sig.unit
            self.value_table.setItem(row, 0, QTableWidgetItem(name))
            self.value_table.setItem(row, 1, QTableWidgetItem("-"))
            self.value_table.setItem(row, 2, QTableWidgetItem(unit))
            self.value_rows[name] = row

    # --- 曲线图管理 ---

    def add_pane(self) -> PlotPane:
        self.pane_counter += 1
        pane = PlotPane(self, self.pane_counter)
        self.panes.append(pane)
        self.plot_layout.addWidget(pane)
        return pane

    def remove_pane(self, pane: PlotPane):
        if pane in self.panes:
            self.panes.remove(pane)
            self.plot_layout.removeWidget(pane)
            pane.setParent(None)
            pane.deleteLater()

    def _toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.setText("继续" if self.paused else "暂停")

    def _clear_data(self):
        self.buffers.clear()
        self.latest.clear()
        self.t0 = None
        for row in range(self.value_table.rowCount()):
            it = self.value_table.item(row, 1)
            if it:
                it.setText("-")

    def _on_window_changed(self, v):
        self.window_sec = float(v)

    def _on_autoscroll_toggled(self, checked):
        self.auto_scroll = bool(checked)

    def _export_layout(self):
        path, _ = QFileDialog.getSaveFileName(self, "导出绘图布局", "layout.json", "JSON Files (*.json);;All Files (*)",
                                              options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        data = {
            "window_sec": self.window_sec,
            "auto_scroll": self.auto_scroll,
            "panes": [{"signals": pane.selected, "colors": pane.colors} for pane in self.panes],
        }
        try:
            import json
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            QMessageBox.warning(self, "导出失败", str(e))

    def _import_layout(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入绘图布局", "", "JSON Files (*.json);;All Files (*)",
                                              options=QFileDialog.DontUseNativeDialog)
        if not path:
            return
        try:
            import json
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.warning(self, "导入失败", str(e))
            return
        for pane in list(self.panes):
            self.remove_pane(pane)
        self.window_sec = float(data.get("window_sec", 10.0))
        self.window_spin.setValue(self.window_sec)
        self.auto_scroll = bool(data.get("auto_scroll", True))
        self.autoscroll_check.setChecked(self.auto_scroll)
        panes = data.get("panes", []) or [{"signals": []}]
        for pd in panes:
            pane = self.add_pane()
            pane.set_signals(pd.get("signals", []), pd.get("colors"))

    # --- 数据接收（由主 GUI 的信号触发，运行在 GUI 线程） ---

    def on_frames(self, msgs: list):
        if self.paused or not msgs:
            return
        now = time.perf_counter()
        if self.t0 is None:
            self.t0 = now
        t_rel = now - self.t0
        sel_channel = self.source_combo.currentData()  # None=全部通道
        rx_only = self.rx_only_check.isChecked()
        for msg in msgs:
            # 解析通道与方向：RX 帧 channel 形如 "can0"；TX 回显形如 "TX:can0"
            ch_str = str(getattr(msg, "channel", "") or "")
            is_tx = ch_str.startswith("TX:")
            base_ch = ch_str[3:] if is_tx else ch_str
            # 自动学习通道，填充数据源下拉
            if base_ch and base_ch not in self._known_channels:
                self._known_channels.add(base_ch)
                self.source_combo.addItem(base_ch, base_ch)
            # 数据源筛选
            if rx_only and is_tx:
                continue
            if sel_channel is not None and base_ch != sel_channel:
                continue
            try:
                decoded = self.db.decode_frame(msg.arbitration_id, bool(msg.is_extended_id), msg.data)
            except Exception:
                continue
            for name, value in decoded.items():
                self.latest[name] = value
                buf = self.buffers.get(name)
                if buf is None:
                    buf = deque()
                    self.buffers[name] = buf
                buf.append((t_rel, value))

    def _refresh(self):
        if self.paused or self.t0 is None:
            return
        now_rel = time.perf_counter() - self.t0
        # 按时间窗口裁剪缓存，限制内存
        cutoff = now_rel - self.window_sec - 1.0
        for buf in self.buffers.values():
            while buf and buf[0][0] < cutoff:
                buf.popleft()
        # 刷新数值表
        for name, value in self.latest.items():
            row = self.value_rows.get(name)
            if row is not None:
                it = self.value_table.item(row, 1)
                if it:
                    it.setText(f"{value:.4g}")
        # 刷新曲线
        for pane in self.panes:
            pane.redraw(now_rel)

    def closeEvent(self, event):
        self.redraw_timer.stop()
        self.closed.emit()
        event.accept()
