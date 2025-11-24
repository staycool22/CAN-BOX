# 说明：CAN 工具主 GUI 界面
# 技术：使用 PySide6 构建界面，与 can_communicator.py 后端分离
import sys
import platform
from datetime import datetime
from collections import deque
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QLineEdit, QComboBox, QPushButton, QTableWidget, QTableWidgetItem,
    QStatusBar, QCheckBox, QDoubleSpinBox, QSpinBox, QMessageBox
)
from PySide6.QtCore import Qt, Signal, Slot, QTimer
from PySide6.QtGui import QFont, QColor

from can_communicator import CANCommunicator, parse_bitrate_token
from can import Message as CANMessage
from can.bus import BusState

class CANToolGUI(QMainWindow):
    # 信号：用于从非 GUI 线程安全地更新 UI
    message_received_signal = Signal(CANMessage)
    status_changed_signal = Signal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("跨平台 CAN/CAN FD 工具")
        self.setGeometry(100, 100, 1200, 800)

        # --- 初始化后端通信器 ---
        self.communicator = CANCommunicator(
            on_message_received=self.handle_incoming_message,
            on_status_changed=self.handle_status_update
        )
        self.is_paused = False
        self.message_index = 0

        # --- 创建主布局 ---
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        top_layout = QHBoxLayout()
        main_layout.addLayout(top_layout)

        # --- UI 组件创建 ---
        self.connection_group = QGroupBox("连接设置")
        top_layout.addWidget(self.connection_group, 1)
        self.create_connection_widgets(QVBoxLayout(self.connection_group))

        self.send_group = QGroupBox("报文发送")
        top_layout.addWidget(self.send_group, 2)
        self.create_send_widgets(QVBoxLayout(self.send_group))

        receive_group = QGroupBox("报文接收")
        main_layout.addWidget(receive_group, 5)
        self.create_receive_widgets(QVBoxLayout(receive_group))

        # --- 状态栏 ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("状态: 未连接 | RX FPS: 0.0 | TX FPS: 0.0 | 总线负载: 0.00%")
        self.status_bar.addPermanentWidget(self.status_label)

        # --- 连接信号与槽 ---
        self.connect_signals_and_slots()
        self.set_controls_enabled(True) # 初始状态，连接设置可用

        # 高速接收显示缓冲与限速
        self.rx_buffer = deque(maxlen=20000)
        self.ui_timer = QTimer(self)
        self.ui_timer.setInterval(50)
        self.ui_max_rows_per_flush = 20
        self.ui_timer.timeout.connect(self.flush_rx_buffer)
        self.ui_timer.start()

        # --- 突发发送相关 ---
        self.burst_timer = QTimer(self)
        self.burst_timer.timeout.connect(self._send_one_burst_message)
        self.burst_messages_left = 0

        self.update_dlc_options()

    def create_connection_widgets(self, layout):
        os_type = platform.system().lower()
        layout.addWidget(QLabel("后端:"))
        self.backend_combo = QComboBox()
        if os_type == 'windows': self.backend_combo.addItems(["candle", "gs_usb"])
        else: self.backend_combo.addItems(["socketcan"])
        layout.addWidget(self.backend_combo)

        chan_layout = QHBoxLayout()
        iface_label = "接口:" if os_type == 'linux' else "通道:"
        chan_layout.addWidget(QLabel(iface_label))
        self.channel_select = QComboBox()
        if os_type == 'windows':
            self.channel_select.addItems(["0", "1"]) 
        else:
            self.channel_select.addItems(["can0", "can1"]) 
        chan_layout.addWidget(self.channel_select)
        layout.addLayout(chan_layout)



        layout.addWidget(QLabel("仲裁域速率:"))
        self.arb_rate_input = QComboBox()
        self.arb_rate_input.addItems(["5k", "10k", "20k", "50k", "100k", "125k", "200k", "250k", "500k", "1m"])
        self.arb_rate_input.setCurrentText("500k")
        layout.addWidget(self.arb_rate_input)

        layout.addWidget(QLabel("数据域速率:"))
        self.data_rate_input = QComboBox()
        self.data_rate_input.addItems(["1m", "2m", "4m", "5m", "8m", "10m"])
        self.data_rate_input.setCurrentText("2m")
        layout.addWidget(self.data_rate_input)

        layout.addWidget(QLabel("采样点 (SP, 0.5-0.9):"))
        self.sp_input = QDoubleSpinBox(); self.sp_input.setRange(0.5, 0.9); self.sp_input.setSingleStep(0.05); self.sp_input.setValue(0.8)
        layout.addWidget(self.sp_input)

        layout.addWidget(QLabel("数据采样点 (DSP, 0.5-0.9):"))
        self.dsp_input = QDoubleSpinBox(); self.dsp_input.setRange(0.5, 0.9); self.dsp_input.setSingleStep(0.05); self.dsp_input.setValue(0.75)
        layout.addWidget(self.dsp_input)

        self.connect_button = QPushButton("连接")
        layout.addWidget(self.connect_button)
        layout.addStretch()

    def create_send_widgets(self, layout):
        id_layout = QHBoxLayout()
        id_layout.addWidget(QLabel("ID (Hex):")); self.send_id_input = QLineEdit("123"); id_layout.addWidget(self.send_id_input)
        self.send_ext_checkbox = QCheckBox("扩展帧"); id_layout.addWidget(self.send_ext_checkbox)
        layout.addLayout(id_layout)

        data_layout = QHBoxLayout()
        data_layout.addWidget(QLabel("数据 (Hex):")); self.send_dat_input = QLineEdit("DE AD BE EF"); data_layout.addWidget(self.send_dat_input)
        layout.addLayout(data_layout)
        
        dlc_layout = QHBoxLayout()
        dlc_layout.addWidget(QLabel("DLC 长度"))
        self.dlc_spin = QComboBox()
        dlc_layout.addWidget(self.dlc_spin)
        layout.addLayout(dlc_layout)
        


        frame_type_layout = QHBoxLayout()
        frame_type_layout.addWidget(QLabel("帧类型:"))
        self.frame_type_combo = QComboBox()
        self.frame_type_combo.addItems(["CAN", "CAN FD", "CAN FD+BRS"])
        frame_type_layout.addWidget(self.frame_type_combo)
        layout.addLayout(frame_type_layout)

        self.send_once_button = QPushButton("单次发送")
        layout.addWidget(self.send_once_button)

        burst_layout = QHBoxLayout()
        self.burst_send_button = QPushButton("发送")
        burst_layout.addWidget(self.burst_send_button)
        burst_layout.addWidget(QLabel("数量(N):"))
        self.burst_count_input = QSpinBox()
        self.burst_count_input.setRange(1, 1000000)
        self.burst_count_input.setValue(10)
        burst_layout.addWidget(self.burst_count_input)
        burst_layout.addWidget(QLabel("间隔(ms):"))
        self.burst_interval_input = QSpinBox()
        self.burst_interval_input.setRange(0, 10000)
        self.burst_interval_input.setValue(100)
        burst_layout.addWidget(self.burst_interval_input)
        layout.addLayout(burst_layout)

        periodic_layout = QHBoxLayout()
        self.periodic_send_button = QPushButton("开始周期发送")
        periodic_layout.addWidget(self.periodic_send_button)
        periodic_layout.addWidget(QLabel("频率(Hz):"))
        self.send_freq_input = QSpinBox(); self.send_freq_input.setRange(1, 10000); self.send_freq_input.setValue(10)
        periodic_layout.addWidget(self.send_freq_input)
        layout.addLayout(periodic_layout)
        layout.addStretch()

    def create_receive_widgets(self, layout):
        rx_controls_layout = QHBoxLayout()
        self.pause_rx_button = QPushButton("暂停显示"); self.clear_rx_button = QPushButton("清空")
        rx_controls_layout.addWidget(self.pause_rx_button); rx_controls_layout.addWidget(self.clear_rx_button)
        rx_controls_layout.addStretch()
        layout.addLayout(rx_controls_layout)

        self.rx_table = QTableWidget()
        self.rx_table.setColumnCount(8)
        self.rx_table.setHorizontalHeaderLabels(["序号", "时间戳", "方向", "ID (Hex)", "类型", "DLC", "数据 (Hex)", "总线"])
        self.rx_table.setColumnWidth(0, 80); self.rx_table.setColumnWidth(1, 150); self.rx_table.setColumnWidth(6, 300)
        self.rx_table.horizontalHeader().setStretchLastSection(True)
        self.rx_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.rx_table.setFont(QFont("Consolas", 10))
        layout.addWidget(self.rx_table)

    def connect_signals_and_slots(self):
        self.connect_button.clicked.connect(self.toggle_connection)
        self.send_once_button.clicked.connect(self.do_send_once)
        self.burst_send_button.clicked.connect(self.do_send_burst)
        self.periodic_send_button.clicked.connect(self.toggle_periodic_send)
        self.clear_rx_button.clicked.connect(self.clear_table)
        self.pause_rx_button.clicked.connect(self.toggle_pause)
        self.message_received_signal.connect(self.enqueue_message)
        self.status_changed_signal.connect(self.update_status_bar)
        self.frame_type_combo.currentTextChanged.connect(self.update_dlc_options)


    # --- 回调与槽函数 ---
    def handle_incoming_message(self, msg: CANMessage):
        self.message_received_signal.emit(msg)

    def handle_status_update(self, status: dict):
        self.status_changed_signal.emit(status)

    def update_dlc_options(self):
        frame_type = self.frame_type_combo.currentText()
        self.dlc_spin.clear()
        if frame_type == "CAN":
            self.dlc_spin.addItems([str(i) for i in range(9)])
            self.dlc_spin.setCurrentText("8")
        else: # CAN FD or CAN FD+BRS
            dlc_options = [str(i) for i in range(9)] + ["12", "16", "20", "24", "32", "48", "64"]
            self.dlc_spin.addItems(dlc_options)
            self.dlc_spin.setCurrentText("16")

    @Slot(CANMessage)
    def enqueue_message(self, msg: CANMessage):
        # 将消息和当前时间戳作为一个元组放入缓冲区
        self.rx_buffer.append((msg, datetime.now()))

    def flush_rx_buffer(self):
        if self.is_paused:
            return
        n = min(len(self.rx_buffer), self.ui_max_rows_per_flush)
        for _ in range(n):
            msg, reception_time = self.rx_buffer.popleft()
            self.add_message_to_table(msg, reception_time)

    @Slot(CANMessage, datetime)
    def add_message_to_table(self, msg: CANMessage, reception_time: datetime):
        if self.is_paused: return
        row_count = self.rx_table.rowCount()
        self.rx_table.insertRow(row_count)
        self.message_index += 1
        
        # 格式化数据
        ts = reception_time.strftime("%H:%M:%S.%f")[:-1]
        direction = "TX" if getattr(msg, "is_tx", False) else "RX"
        msg_id = f"{msg.arbitration_id:X}"
        msg_type = ("扩展" if msg.is_extended_id else "标准") + (" FD" if msg.is_fd else "")
        data = ' '.join(f'{b:02X}' for b in msg.data)
        bus_name = str(getattr(msg, 'channel', self.channel_select.currentText()))

        # 填充表格
        self.rx_table.setItem(row_count, 0, QTableWidgetItem(str(self.message_index)))
        self.rx_table.setItem(row_count, 1, QTableWidgetItem(ts))
        self.rx_table.setItem(row_count, 2, QTableWidgetItem(direction))
        self.rx_table.setItem(row_count, 3, QTableWidgetItem(msg_id))
        self.rx_table.setItem(row_count, 4, QTableWidgetItem(msg_type))
        self.rx_table.setItem(row_count, 5, QTableWidgetItem(str(msg.dlc)))
        self.rx_table.setItem(row_count, 6, QTableWidgetItem(data))
        self.rx_table.setItem(row_count, 7, QTableWidgetItem(bus_name))
        
        # 颜色区分
        color = QColor("blue") if getattr(msg, "is_tx", False) else QColor("black")
        for i in range(self.rx_table.columnCount()):
            self.rx_table.item(row_count, i).setForeground(color)

        self.rx_table.scrollToBottom()

    @Slot(dict)
    def update_status_bar(self, status: dict):
        state_str = status['bus_state'].name if isinstance(status['bus_state'], BusState) else str(status['bus_state'])
        text = f"状态: {state_str} | RX FPS: {status['rx_fps']:.1f} | TX FPS: {status['tx_fps']:.1f} | 总线负载: {status['bus_load']:.2f}%"
        self.status_label.setText(text)

    # --- 按钮操作 ---
    def toggle_connection(self):
        if self.communicator.is_connected:
            self.communicator.disconnect()
            self.connect_button.setText("连接")
            self.set_controls_enabled(True)
            self.periodic_send_button.setText("开始周期发送")
        else:
            try:
                config = {
                    'interface': self.channel_select.currentText(),
                    'backend': self.backend_combo.currentText(),
                    'is_fd': True,
                    'arb_rate': parse_bitrate_token(self.arb_rate_input.currentText()),
                    'data_rate': parse_bitrate_token(self.data_rate_input.currentText()),
                    'sp': self.sp_input.value(),
                    'dsp': self.dsp_input.value()
                }
                self.communicator.connect(**config)
                self.connect_button.setText("断开")
                self.set_controls_enabled(False)
            except Exception as e:
                QMessageBox.critical(self, "连接错误", str(e))

    def do_send_once(self):
        try:
            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()
            is_fd = "FD" in frame_type
            brs = "+BRS" in frame_type

            self.communicator.send_one(
                arbitration_id=msg_id,
                data=data,
                is_extended_id=self.send_ext_checkbox.isChecked(),
                is_fd=is_fd,
                brs=brs
            )
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))

    def do_send_burst(self):
        if self.burst_timer.isActive():
            self.burst_timer.stop()
            self.burst_send_button.setText("发送")
            return

        try:
            self.burst_messages_left = self.burst_count_input.value()
            interval = self.burst_interval_input.value()
            
            # 立即发送第一条
            self._send_one_burst_message()

            # 如果需要发送更多，则启动定时器
            if self.burst_messages_left > 0:
                self.burst_timer.start(interval)
                self.burst_send_button.setText("停止发送")

        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))

    def _send_one_burst_message(self):
        if self.burst_messages_left <= 0:
            self.burst_timer.stop()
            self.burst_send_button.setText("发送")
            return

        try:
            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()
            is_fd = "FD" in frame_type
            brs = "+BRS" in frame_type

            self.communicator.send_one(
                arbitration_id=msg_id,
                data=data,
                is_extended_id=self.send_ext_checkbox.isChecked(),
                is_fd=is_fd,
                brs=brs
            )
            self.burst_messages_left -= 1

            if self.burst_messages_left == 0:
                self.burst_timer.stop()
                self.burst_send_button.setText("发送")

        except Exception as e:
            self.burst_timer.stop()
            self.burst_send_button.setText("发送")
            QMessageBox.warning(self, "发送错误", f"发送中止: {e}")

    def toggle_periodic_send(self):
        running = False
        try:
            running = (
                (self.communicator.periodic_send_thread and self.communicator.periodic_send_thread.is_alive()) or
                getattr(self.communicator, 'periodic_task', None) is not None
            )
        except Exception:
            running = False

        if running:
            self.communicator.stop_periodic_send()
            self.periodic_send_button.setText("开始周期发送")
        else:
            try:
                msg_id = int(self.send_id_input.text(), 16)
                data = self.prepare_data_for_send()
                freq = self.send_freq_input.value()
                frame_type = self.frame_type_combo.currentText()
                is_fd = "FD" in frame_type
                brs = "+BRS" in frame_type

                self.communicator.start_periodic_send(
                    arbitration_id=msg_id, data=data, frequency=freq,
                    is_extended_id=self.send_ext_checkbox.isChecked(),
                    is_fd=is_fd,
                    brs=brs
                )
                self.periodic_send_button.setText("停止周期发送")
            except Exception as e:
                QMessageBox.warning(self, "发送错误", str(e))

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        self.pause_rx_button.setText("继续显示" if self.is_paused else "暂停显示")

    def clear_table(self):
        self.rx_table.setRowCount(0)
        self.message_index = 0
        try:
            self.rx_buffer.clear()
        except Exception:
            pass

    def prepare_data_for_send(self):
        dlc = int(self.dlc_spin.currentText())
        parts = [p for p in self.send_dat_input.text().split() if p]
        data = []
        for p in parts:
            data.append(int(p, 16))
        if len(data) < dlc:
            data.extend([0] * (dlc - len(data)))
        elif len(data) > dlc:
            data = data[:dlc]

        return data

    def set_controls_enabled(self, is_enabled: bool):
        """启用或禁用连接设置区域的控件（保留连接/断开按钮始终可用）"""
        try:
            self.backend_combo.setEnabled(is_enabled)
            self.channel_select.setEnabled(is_enabled)

            self.arb_rate_input.setEnabled(is_enabled)
            self.data_rate_input.setEnabled(is_enabled)
            self.sp_input.setEnabled(is_enabled)
            self.dsp_input.setEnabled(is_enabled)
        except Exception:
            pass
        self.send_group.setEnabled(not is_enabled)
        self.connect_button.setEnabled(True)

    def closeEvent(self, event):
        """重写关闭事件，确保资源被释放"""
        if self.communicator.is_connected:
            self.communicator.disconnect()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    try:
        from PySide6 import QtCore
    except ImportError:
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Critical)
        msg_box.setText("依赖缺失")
        msg_box.setInformativeText("PySide6 未安装，请运行: pip install pyside6")
        msg_box.setWindowTitle("错误")
        msg_box.exec()
        sys.exit(1)

    window = CANToolGUI()
    window.show()
    sys.exit(app.exec())
