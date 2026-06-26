# 说明：可复用的发送面板（主窗口与独立发送窗口共用）
# 用途：把"通道选择 + ID/数据/DLC/帧类型 + 单次/突发/周期发送"的 UI 与逻辑收敛到一处，
#       消除 main_gui 中主窗口与 SendWindow 的重复实现。
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QComboBox, QPushButton,
    QCheckBox, QSpinBox, QDoubleSpinBox, QMessageBox,
)
from PySide6.QtCore import Signal


class SendPanel(QWidget):
    """发送面板：依赖共享的 communicators 字典（通道号 -> CANCommunicator）。"""

    burst_finished_signal = Signal()  # 跨线程：把突发完成回调投递回 GUI 线程

    def __init__(self, communicators: dict, parent=None):
        super().__init__(parent)
        self.communicators = communicators
        self.stop_burst_func = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # 通道选择
        chan_layout = QHBoxLayout()
        chan_layout.addWidget(QLabel("选择通道:"))
        self.channel_combo = QComboBox()
        chan_layout.addWidget(self.channel_combo)
        layout.addLayout(chan_layout)

        # ID + 扩展帧
        id_layout = QHBoxLayout()
        id_layout.addWidget(QLabel("ID (Hex):"))
        self.send_id_input = QLineEdit("123")
        id_layout.addWidget(self.send_id_input)
        self.send_ext_checkbox = QCheckBox("扩展帧")
        id_layout.addWidget(self.send_ext_checkbox)
        layout.addLayout(id_layout)

        # 数据
        data_layout = QHBoxLayout()
        data_layout.addWidget(QLabel("数据 (Hex):"))
        self.send_dat_input = QLineEdit("DE AD BE EF")
        data_layout.addWidget(self.send_dat_input)
        layout.addLayout(data_layout)

        # DLC
        dlc_layout = QHBoxLayout()
        dlc_layout.addWidget(QLabel("DLC 长度"))
        self.dlc_spin = QComboBox()
        dlc_layout.addWidget(self.dlc_spin)
        layout.addLayout(dlc_layout)

        # 帧类型
        frame_type_layout = QHBoxLayout()
        frame_type_layout.addWidget(QLabel("帧类型:"))
        self.frame_type_combo = QComboBox()
        self.frame_type_combo.addItems(["CAN", "CAN FD", "CAN FD+BRS"])
        frame_type_layout.addWidget(self.frame_type_combo)
        layout.addLayout(frame_type_layout)

        # 单次发送
        self.send_once_button = QPushButton("单次发送")
        layout.addWidget(self.send_once_button)

        # 突发
        burst_layout = QHBoxLayout()
        self.burst_send_button = QPushButton("发送")
        burst_layout.addWidget(self.burst_send_button)
        burst_layout.addWidget(QLabel("数量(N):"))
        self.burst_count_input = QSpinBox(); self.burst_count_input.setRange(1, 1000000); self.burst_count_input.setValue(10)
        burst_layout.addWidget(self.burst_count_input)
        burst_layout.addWidget(QLabel("间隔(ms):"))
        self.burst_interval_input = QDoubleSpinBox(); self.burst_interval_input.setRange(0, 10000); self.burst_interval_input.setValue(100.0)
        self.burst_interval_input.setSingleStep(0.1)
        burst_layout.addWidget(self.burst_interval_input)
        layout.addLayout(burst_layout)

        # 周期
        periodic_layout = QHBoxLayout()
        self.periodic_send_button = QPushButton("开始周期发送")
        periodic_layout.addWidget(self.periodic_send_button)
        periodic_layout.addWidget(QLabel("频率(Hz):"))
        self.send_freq_input = QDoubleSpinBox(); self.send_freq_input.setRange(0.1, 10000); self.send_freq_input.setValue(10.0)
        self.send_freq_input.setSingleStep(1.0)
        periodic_layout.addWidget(self.send_freq_input)
        layout.addLayout(periodic_layout)
        layout.addStretch()

        # 信号连接（面板自管，不依赖上层）
        self.send_once_button.clicked.connect(self.do_send_once)
        self.burst_send_button.clicked.connect(self.do_send_burst)
        self.periodic_send_button.clicked.connect(self.toggle_periodic_send)
        self.frame_type_combo.currentTextChanged.connect(self.update_dlc_options)
        self.channel_combo.currentTextChanged.connect(self.update_periodic_btn_state)
        self.burst_interval_input.valueChanged.connect(self.update_burst_freq_from_interval)
        self.send_freq_input.valueChanged.connect(self.update_burst_interval_from_freq)
        self.burst_finished_signal.connect(self.on_burst_finished)

        self.update_dlc_options()
        self.refresh_channels()

    # --- 通道 ---
    def refresh_channels(self):
        current = self.channel_combo.currentText()
        self.channel_combo.blockSignals(True)
        self.channel_combo.clear()
        channels = sorted(self.communicators.keys())
        if channels:
            items = [str(c) for c in channels]
            self.channel_combo.addItems(items)
            if current in items:
                self.channel_combo.setCurrentText(current)
            self.setEnabled(True)
        else:
            self.channel_combo.addItem("无连接")
            self.setEnabled(False)
        self.channel_combo.blockSignals(False)
        self.update_periodic_btn_state()

    def current_channel(self) -> Optional[int]:
        try:
            return int(self.channel_combo.currentText())
        except (ValueError, TypeError):
            return None

    def current_communicator(self):
        return self.communicators.get(self.current_channel())

    # --- 频率/间隔联动 ---
    def update_burst_freq_from_interval(self):
        interval_ms = self.burst_interval_input.value()
        if interval_ms > 0:
            freq = 1000.0 / interval_ms
            self.send_freq_input.blockSignals(True)
            self.send_freq_input.setValue(freq)
            self.send_freq_input.blockSignals(False)

    def update_burst_interval_from_freq(self):
        freq = self.send_freq_input.value()
        if freq > 0:
            interval_ms = 1000.0 / freq
            self.burst_interval_input.blockSignals(True)
            self.burst_interval_input.setValue(interval_ms)
            self.burst_interval_input.blockSignals(False)

    def update_dlc_options(self):
        frame_type = self.frame_type_combo.currentText()
        self.dlc_spin.clear()
        if frame_type == "CAN":
            self.dlc_spin.addItems([str(i) for i in range(9)])
            self.dlc_spin.setCurrentText("8")
        else:
            dlc_options = [str(i) for i in range(9)] + ["12", "16", "20", "24", "32", "48", "64"]
            self.dlc_spin.addItems(dlc_options)
            self.dlc_spin.setCurrentText("16")

    # --- 数据 ---
    def prepare_data_for_send(self):
        dlc = int(self.dlc_spin.currentText())
        parts = [p for p in self.send_dat_input.text().split() if p]
        data = [int(p, 16) for p in parts]
        if len(data) < dlc:
            data.extend([0] * (dlc - len(data)))
        elif len(data) > dlc:
            data = data[:dlc]
        return data

    # --- 发送 ---
    def do_send_once(self):
        comm = self.current_communicator()
        if not comm:
            return
        try:
            ch = self.current_channel()
            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()
            comm.send_one(msg_id, data, self.send_ext_checkbox.isChecked(),
                          "FD" in frame_type, "+BRS" in frame_type, channel=ch)
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))

    def do_send_burst(self):
        if self.stop_burst_func:
            try:
                self.stop_burst_func()
            except Exception:
                pass
            self.stop_burst_func = None
            self.burst_send_button.setText("发送")
            return

        comm = self.current_communicator()
        if not comm:
            return
        try:
            count = self.burst_count_input.value()
            interval_ms = self.burst_interval_input.value()
            if interval_ms <= 0:
                interval_ms = 0.001
            freq = 1000.0 / interval_ms
            ch = self.current_channel()
            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()

            self.burst_send_button.setText("停止发送")
            self.stop_burst_func = comm.start_burst_send(
                msg_id, data, freq, count,
                self.send_ext_checkbox.isChecked(),
                "FD" in frame_type, "+BRS" in frame_type,
                channel=ch,
                on_finish=lambda: self.burst_finished_signal.emit(),
            )
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))
            self.burst_send_button.setText("发送")

    def on_burst_finished(self):
        self.stop_burst_func = None
        self.burst_send_button.setText("发送")

    def update_periodic_btn_state(self):
        comm = self.current_communicator()
        if not comm:
            self.periodic_send_button.setText("开始周期发送")
            return
        if comm.is_periodic_sending(self.current_channel()):
            self.periodic_send_button.setText("停止周期发送")
        else:
            self.periodic_send_button.setText("开始周期发送")

    def toggle_periodic_send(self):
        comm = self.current_communicator()
        if not comm:
            return
        ch = self.current_channel()
        if comm.is_periodic_sending(ch):
            comm.stop_periodic_send(channel=ch)
            self.periodic_send_button.setText("开始周期发送")
        else:
            try:
                msg_id = int(self.send_id_input.text(), 16)
                data = self.prepare_data_for_send()
                frame_type = self.frame_type_combo.currentText()
                comm.start_periodic_send(
                    msg_id, data, self.send_freq_input.value(),
                    self.send_ext_checkbox.isChecked(),
                    "FD" in frame_type, "+BRS" in frame_type,
                    channel=ch,
                )
                self.periodic_send_button.setText("停止周期发送")
            except Exception as e:
                QMessageBox.warning(self, "发送错误", str(e))

    # --- 预设回填（供主窗口「填入主发送」复用） ---
    def load_preset(self, preset: dict):
        try:
            self.send_id_input.setText(str(preset.get("id", "")))
            self.send_ext_checkbox.setChecked(bool(preset.get("is_extended", False)))
            self.send_dat_input.setText(str(preset.get("data", "")))
            idx = self.frame_type_combo.findText(preset.get("frame_type", "CAN"))
            if idx >= 0:
                self.frame_type_combo.setCurrentIndex(idx)
            ch = preset.get("channel")
            if ch is not None:
                i = self.channel_combo.findText(str(ch))
                if i >= 0:
                    self.channel_combo.setCurrentIndex(i)
        except Exception:
            pass
