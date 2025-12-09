# 说明：CAN 工具主 GUI 界面
# 技术：使用 PySide6 构建界面，与 can_communicator.py 后端分离
import sys
import csv
import platform
from datetime import datetime
from collections import deque
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QLineEdit, QComboBox, QPushButton, QTableWidget, QTableWidgetItem,
    QStatusBar, QCheckBox, QDoubleSpinBox, QSpinBox, QMessageBox, QFileDialog,
    QHeaderView, QDialog
)
from PySide6.QtCore import Qt, Signal, Slot, QTimer
from PySide6.QtGui import QFont, QColor, QAction

from can_communicator import CANCommunicator, parse_bitrate_token
from can import Message as CANMessage
from can.bus import BusState

class SendWindow(QWidget):
    """独立的发送窗口"""
    burst_finished_signal = Signal()

    def __init__(self, communicators: dict, parent=None):
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("独立发送窗口")
        self.resize(500, 300)
        self.communicators = communicators
        self.layout = QVBoxLayout(self)
        
        self.burst_finished_signal.connect(self.on_burst_finished)
        self.stop_burst_func = None
        
        # Channel Selection
        chan_layout = QHBoxLayout()
        chan_layout.addWidget(QLabel("选择通道:"))
        self.channel_combo = QComboBox()
        self.channel_combo.currentTextChanged.connect(self.update_periodic_btn_state)
        chan_layout.addWidget(self.channel_combo)
        self.layout.addLayout(chan_layout)
        
        # Send Widgets
        self.create_send_widgets(self.layout)
        self.refresh_channels()
        
        # Burst Timer
        self.burst_timer = QTimer(self)
        self.burst_timer.timeout.connect(self._send_one_burst_message)
        self.burst_messages_left = 0

    def create_send_widgets(self, layout):
        # ID & Ext
        id_layout = QHBoxLayout()
        id_layout.addWidget(QLabel("ID (Hex):")); self.send_id_input = QLineEdit("123"); id_layout.addWidget(self.send_id_input)
        self.send_ext_checkbox = QCheckBox("扩展帧"); id_layout.addWidget(self.send_ext_checkbox)
        layout.addLayout(id_layout)

        # Data
        data_layout = QHBoxLayout()
        data_layout.addWidget(QLabel("数据 (Hex):")); self.send_dat_input = QLineEdit("DE AD BE EF"); data_layout.addWidget(self.send_dat_input)
        layout.addLayout(data_layout)
        
        # DLC
        dlc_layout = QHBoxLayout()
        dlc_layout.addWidget(QLabel("DLC 长度"))
        self.dlc_spin = QComboBox()
        dlc_layout.addWidget(self.dlc_spin)
        layout.addLayout(dlc_layout)

        # Frame Type
        frame_type_layout = QHBoxLayout()
        frame_type_layout.addWidget(QLabel("帧类型:"))
        self.frame_type_combo = QComboBox()
        self.frame_type_combo.addItems(["CAN", "CAN FD", "CAN FD+BRS"])
        frame_type_layout.addWidget(self.frame_type_combo)
        self.frame_type_combo.currentTextChanged.connect(self.update_dlc_options)
        layout.addLayout(frame_type_layout)
        self.update_dlc_options()

        # Send Buttons
        self.send_once_button = QPushButton("单次发送")
        self.send_once_button.clicked.connect(self.do_send_once)
        layout.addWidget(self.send_once_button)

        # Burst
        burst_layout = QHBoxLayout()
        self.burst_send_button = QPushButton("发送")
        self.burst_send_button.clicked.connect(self.do_send_burst)
        burst_layout.addWidget(self.burst_send_button)
        burst_layout.addWidget(QLabel("数量(N):"))
        self.burst_count_input = QSpinBox(); self.burst_count_input.setRange(1, 1000000); self.burst_count_input.setValue(10)
        burst_layout.addWidget(self.burst_count_input)
        burst_layout.addWidget(QLabel("间隔(ms):"))
        self.burst_interval_input = QDoubleSpinBox(); self.burst_interval_input.setRange(0, 10000); self.burst_interval_input.setValue(100.0)
        self.burst_interval_input.setSingleStep(0.1)
        self.burst_interval_input.valueChanged.connect(self.update_burst_freq_from_interval)
        burst_layout.addWidget(self.burst_interval_input)
        layout.addLayout(burst_layout)

        # Periodic
        periodic_layout = QHBoxLayout()
        self.periodic_send_button = QPushButton("开始周期发送")
        self.periodic_send_button.clicked.connect(self.toggle_periodic_send)
        periodic_layout.addWidget(self.periodic_send_button)
        periodic_layout.addWidget(QLabel("频率(Hz):"))
        self.send_freq_input = QDoubleSpinBox(); self.send_freq_input.setRange(0.1, 10000); self.send_freq_input.setValue(10.0)
        self.send_freq_input.setSingleStep(1.0)
        self.send_freq_input.valueChanged.connect(self.update_burst_interval_from_freq)
        periodic_layout.addWidget(self.send_freq_input)
        layout.addLayout(periodic_layout)
        layout.addStretch()
        
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

    def refresh_channels(self):
        current = self.channel_combo.currentText()
        self.channel_combo.clear()
        channels = sorted(self.communicators.keys())
        if not channels:
            self.channel_combo.addItem("无可用通道")
            self.setEnabled(False)
        else:
            self.channel_combo.addItems([str(ch) for ch in channels])
            self.setEnabled(True)
            if current in [str(ch) for ch in channels]:
                self.channel_combo.setCurrentText(current)

    def get_current_communicator(self):
        try:
            ch_str = self.channel_combo.currentText()
            if not ch_str or ch_str == "无可用通道": return None
            ch = int(ch_str)
            return self.communicators.get(ch)
        except:
            return None

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

    def do_send_once(self):
        comm = self.get_current_communicator()
        if not comm: return
        try:
            ch_str = self.channel_combo.currentText()
            ch = int(ch_str) if ch_str and ch_str != "无可用通道" else None
            
            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()
            comm.send_one(msg_id, data, self.send_ext_checkbox.isChecked(), "FD" in frame_type, "+BRS" in frame_type, channel=ch)
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))

    def do_send_burst(self):
        if self.stop_burst_func:
            try:
                self.stop_burst_func()
            except:
                pass
            self.stop_burst_func = None
            self.burst_send_button.setText("发送")
            return
            
        comm = self.get_current_communicator()
        if not comm: return
        try:
            count = self.burst_count_input.value()
            interval_ms = self.burst_interval_input.value()
            if interval_ms <= 0: interval_ms = 0.001 # limit max freq
            freq = 1000.0 / interval_ms
            
            ch_str = self.channel_combo.currentText()
            ch = int(ch_str) if ch_str and ch_str != "无可用通道" else None

            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()
            
            self.burst_send_button.setText("停止发送")
            
            def on_finish():
                self.burst_finished_signal.emit()

            self.stop_burst_func = comm.start_burst_send(
                msg_id, data, freq, count, 
                self.send_ext_checkbox.isChecked(), 
                "FD" in frame_type, "+BRS" in frame_type, 
                channel=ch,
                on_finish=on_finish
            )
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))
            self.burst_send_button.setText("发送")

    def on_burst_finished(self):
        self.stop_burst_func = None
        self.burst_send_button.setText("发送")

    def _send_one_burst_message(self):
        pass # Deprecated


    def update_periodic_btn_state(self):
        comm = self.get_current_communicator()
        if not comm:
             self.periodic_send_button.setText("开始周期发送")
             return
        
        try:
            ch_str = self.channel_combo.currentText()
            ch = int(ch_str) if ch_str and ch_str != "无可用通道" else None
        except:
            ch = None

        if comm.is_periodic_sending(ch):
            self.periodic_send_button.setText("停止周期发送")
        else:
            self.periodic_send_button.setText("开始周期发送")

    def toggle_periodic_send(self):
        comm = self.get_current_communicator()
        if not comm: return
        
        try:
            ch_str = self.channel_combo.currentText()
            ch = int(ch_str) if ch_str and ch_str != "无可用通道" else None
        except:
            ch = None

        running = comm.is_periodic_sending(ch)

        if running:
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
                    channel=ch
                )
                self.periodic_send_button.setText("停止周期发送")
            except Exception as e:
                QMessageBox.warning(self, "发送错误", str(e))


class CANToolGUI(QMainWindow):
    # 信号：用于从非 GUI 线程安全地更新 UI
    message_received_signal = Signal(CANMessage)
    status_changed_signal = Signal(dict, int) # status, channel
    burst_finished_signal = Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN/CAN FD 工具 (多通道版)")
        self.setGeometry(100, 100, 1300, 850)

        # --- 多通道管理器 ---
        # 键: 通道ID (int), 值: CANCommunicator 实例
        self.communicators = {} 
        self.channel_statuses = {} # 保存每个通道的最新状态
        self.send_windows = []

        self.is_paused = False
        self.message_index = 0

        # --- 创建主布局 ---
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        top_layout = QHBoxLayout()
        main_layout.addLayout(top_layout)

        # --- UI 组件创建 ---
        self.connection_group = QGroupBox("通道配置与连接")
        top_layout.addWidget(self.connection_group, 2)
        self.create_connection_widgets(QVBoxLayout(self.connection_group))

        self.send_group = QGroupBox("主发送窗口")
        top_layout.addWidget(self.send_group, 1)
        self.create_send_widgets(QVBoxLayout(self.send_group))

        receive_group = QGroupBox("报文接收")
        main_layout.addWidget(receive_group, 5)
        self.create_receive_widgets(QVBoxLayout(receive_group))

        # --- 状态栏 ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("就绪")
        self.status_bar.addPermanentWidget(self.status_label)

        # --- 信号连接 ---
        self.connect_signals_and_slots()

        # 高速接收显示缓冲与限速
        self.rx_buffer = deque(maxlen=100000)
        self.ui_timer = QTimer(self)
        try:
            self.ui_timer.setTimerType(Qt.PreciseTimer)
        except Exception:
            pass
        self.ui_timer.setInterval(50)
        self.ui_max_rows_per_flush = 20
        self.ui_timer.timeout.connect(self.flush_rx_buffer)
        self.ui_timer.start()
        self.adaptive_flush = True

        # 主窗口突发发送
        self.burst_timer = QTimer(self)
        self.burst_timer.timeout.connect(self._send_one_burst_message)
        self.burst_messages_left = 0
        
        self.stop_burst_func = None
        self.burst_finished_signal.connect(self.on_burst_finished)
        
        self.update_dlc_options()

    def create_connection_widgets(self, layout):
        # 使用表格管理多通道配置
        self.conn_table = QTableWidget()
        self.conn_table.setColumnCount(10)
        self.conn_table.setHorizontalHeaderLabels(["后端", "通道", "仲裁波特率", "数据波特率", "SP", "DSP", "RX FPS", "TX FPS", "负载 %", "操作"])
        self.conn_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        
        # 预置 2 行 (通道 0 和 1)
        for i in range(2):
            self.add_connection_row(i)
            
        layout.addWidget(self.conn_table)
        
        btn_layout = QHBoxLayout()
        add_btn = QPushButton("添加通道行")
        add_btn.clicked.connect(lambda: self.add_connection_row(self.conn_table.rowCount()))
        btn_layout.addWidget(add_btn)
        
        self.btn_connect_all = QPushButton("全部连接 (多通道)")
        self.btn_connect_all.clicked.connect(self.connect_all_devices)
        btn_layout.addWidget(self.btn_connect_all)

        self.btn_disconnect_all = QPushButton("全部断开")
        self.btn_disconnect_all.clicked.connect(self.disconnect_all_devices)
        self.btn_disconnect_all.setEnabled(False)
        btn_layout.addWidget(self.btn_disconnect_all)

        refresh_btn = QPushButton("刷新设备列表")
        refresh_btn.clicked.connect(self.refresh_device_list)
        btn_layout.addWidget(refresh_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def refresh_device_list(self):
        row_count = self.conn_table.rowCount()
        for row in range(row_count):
            self.update_channel_combo(row)

    def add_connection_row(self, default_ch=0):
        row = self.conn_table.rowCount()
        self.conn_table.insertRow(row)
        
        # 1. Backend
        backend_combo = QComboBox()
        os_type = platform.system().lower()
        if os_type == 'windows': backend_combo.addItems(["candle", "gs_usb"])
        else: backend_combo.addItems(["socketcan"])
        self.conn_table.setCellWidget(row, 0, backend_combo)
        
        # 2. Channel
        ch_combo = QComboBox()
        self.conn_table.setCellWidget(row, 1, ch_combo)
        
        # Update channels based on backend
        # 安全捕获 row 和 combo
        backend_combo.currentTextChanged.connect(lambda text, r=row, c=backend_combo: self.on_backend_changed(r, c))
        self.update_channel_combo(row)
        
        if default_ch < ch_combo.count():
            ch_combo.setCurrentIndex(default_ch)
        
        # 3. Arb Rate
        arb_combo = QComboBox()
        arb_combo.addItems(["50k", "100k", "125k", "250k", "500k", "800k", "1m"])
        arb_combo.setCurrentText("500k")
        self.conn_table.setCellWidget(row, 2, arb_combo)
        
        # 4. Data Rate
        data_combo = QComboBox()
        data_combo.addItems(["1m", "2m", "4m", "5m", "8m"])
        data_combo.setCurrentText("2m")
        self.conn_table.setCellWidget(row, 3, data_combo)
        
        # 5. SP
        sp_spin = QDoubleSpinBox(); sp_spin.setRange(0.5, 0.9); sp_spin.setSingleStep(0.05); sp_spin.setValue(0.8)
        self.conn_table.setCellWidget(row, 4, sp_spin)
        
        # 6. DSP
        dsp_spin = QDoubleSpinBox(); dsp_spin.setRange(0.5, 0.9); dsp_spin.setSingleStep(0.05); dsp_spin.setValue(0.75)
        self.conn_table.setCellWidget(row, 5, dsp_spin)
        
        # 7. RX FPS
        rx_label = QLabel("0.0")
        rx_label.setAlignment(Qt.AlignCenter)
        self.conn_table.setCellWidget(row, 6, rx_label)

        # 8. TX FPS
        tx_label = QLabel("0.0")
        tx_label.setAlignment(Qt.AlignCenter)
        self.conn_table.setCellWidget(row, 7, tx_label)

        # 9. Load %
        load_label = QLabel("0.0%")
        load_label.setAlignment(Qt.AlignCenter)
        self.conn_table.setCellWidget(row, 8, load_label)
        
        # 10. Action Button
        btn = QPushButton("连接")
        # 使用闭包捕获 row 索引? 注意 row 可能会变，所以这里绑定 item 或者重新获取
        btn.clicked.connect(lambda checked=False, b=btn: self.toggle_connection_row(b))
        self.conn_table.setCellWidget(row, 9, btn)

    def on_backend_changed(self, row, combo):
        # 验证 sender 是否仍然有效（虽然传递了 combo 对象，但检查 row 对应的是否一致更安全）
        current_widget = self.conn_table.cellWidget(row, 0)
        if current_widget != combo:
             return
        self.update_channel_combo(row)

    def update_channel_combo(self, row):
        backend_combo = self.conn_table.cellWidget(row, 0)
        ch_combo = self.conn_table.cellWidget(row, 1)
        if not backend_combo or not ch_combo: return
        
        backend = backend_combo.currentText()
        ch_combo.blockSignals(True)
        ch_combo.clear()
        
        if backend == 'candle':
            try:
                # 确保 CANCommunicator.list_devices 存在
                if hasattr(CANCommunicator, 'list_devices'):
                    devices = CANCommunicator.list_devices('candle')
                    ch_combo.addItems(devices)
                else:
                    ch_combo.addItems([str(i) for i in range(16)])
            except Exception:
                ch_combo.addItems([str(i) for i in range(16)])
        elif backend == 'socketcan':
            ch_combo.addItems([f"can{i}" for i in range(4)])
        else:
            ch_combo.addItems([str(i) for i in range(16)])
            
        ch_combo.blockSignals(False)

    def create_send_widgets(self, layout):
        # Channel Selection
        chan_layout = QHBoxLayout()
        chan_layout.addWidget(QLabel("选择通道:"))
        self.main_send_channel_combo = QComboBox()
        self.main_send_channel_combo.currentTextChanged.connect(self.update_periodic_btn_state)
        chan_layout.addWidget(self.main_send_channel_combo)
        layout.addLayout(chan_layout)

        # ID & Data
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
        self.burst_count_input = QSpinBox(); self.burst_count_input.setRange(1, 1000000); self.burst_count_input.setValue(10)
        burst_layout.addWidget(self.burst_count_input)
        burst_layout.addWidget(QLabel("间隔(ms):"))
        self.burst_interval_input = QDoubleSpinBox(); self.burst_interval_input.setRange(0, 10000); self.burst_interval_input.setValue(100.0)
        self.burst_interval_input.setSingleStep(0.1)
        self.burst_interval_input.valueChanged.connect(self.update_burst_freq_from_interval)
        burst_layout.addWidget(self.burst_interval_input)
        layout.addLayout(burst_layout)

        # Periodic
        periodic_layout = QHBoxLayout()
        self.periodic_send_button = QPushButton("开始周期发送")
        periodic_layout.addWidget(self.periodic_send_button)
        periodic_layout.addWidget(QLabel("频率(Hz):"))
        self.send_freq_input = QDoubleSpinBox(); self.send_freq_input.setRange(0.1, 10000); self.send_freq_input.setValue(10.0)
        self.send_freq_input.setSingleStep(1.0)
        self.send_freq_input.valueChanged.connect(self.update_burst_interval_from_freq)
        periodic_layout.addWidget(self.send_freq_input)
        layout.addLayout(periodic_layout)
        
        # New Window Button
        new_win_btn = QPushButton("打开新发送窗口")
        new_win_btn.clicked.connect(self.open_new_send_window)
        layout.addWidget(new_win_btn)
        
        layout.addStretch()

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

    def create_receive_widgets(self, layout):
        rx_controls_layout = QHBoxLayout()
        self.pause_rx_button = QPushButton("暂停显示"); self.clear_rx_button = QPushButton("清空"); self.save_rx_button = QPushButton("保存数据")
        rx_controls_layout.addWidget(self.pause_rx_button); rx_controls_layout.addWidget(self.clear_rx_button); rx_controls_layout.addWidget(self.save_rx_button)
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
        self.send_once_button.clicked.connect(self.do_send_once)
        self.burst_send_button.clicked.connect(self.do_send_burst)
        self.periodic_send_button.clicked.connect(self.toggle_periodic_send)
        self.clear_rx_button.clicked.connect(self.clear_table)
        self.save_rx_button.clicked.connect(self.save_table_data)
        self.pause_rx_button.clicked.connect(self.toggle_pause)
        self.message_received_signal.connect(self.enqueue_message)
        self.status_changed_signal.connect(self.update_status_bar)
        self.frame_type_combo.currentTextChanged.connect(self.update_dlc_options)

    # --- 逻辑处理 ---
    
    def toggle_connection_row(self, btn):
        # 查找按钮所在的行
        index = self.conn_table.indexAt(btn.pos())
        if not index.isValid(): return
        row = index.row()
        
        # 获取配置
        # 使用 item(row, col) 可能会返回 None，如果使用的是 setCellWidget，应该用 cellWidget
        backend_widget = self.conn_table.cellWidget(row, 0)
        ch_widget = self.conn_table.cellWidget(row, 1)
        arb_widget = self.conn_table.cellWidget(row, 2)
        data_widget = self.conn_table.cellWidget(row, 3)
        sp_widget = self.conn_table.cellWidget(row, 4)
        dsp_widget = self.conn_table.cellWidget(row, 5)

        if not (backend_widget and ch_widget and arb_widget and data_widget and sp_widget and dsp_widget):
             QMessageBox.warning(self, "错误", "无法获取行配置组件")
             return

        backend = backend_widget.currentText()
        ch_str = ch_widget.currentText()
        
        # 处理 Linux can0 -> 0
        if ch_str.startswith("can"):
            try: channel_id = int(ch_str.replace("can", ""))
            except: channel_id = 0
        elif " (SN:" in ch_str:
            try: channel_id = int(ch_str.split()[0])
            except: channel_id = 0
        else:
            try:
                channel_id = int(ch_str)
            except:
                channel_id = 0 # default or error
            
        arb_rate = parse_bitrate_token(arb_widget.currentText())
        data_rate = parse_bitrate_token(data_widget.currentText())
        sp = sp_widget.value()
        dsp = dsp_widget.value()
        
        # 检查是否已连接
        if channel_id in self.communicators:
            # 断开逻辑
            comm = self.communicators.pop(channel_id)
            try:
                comm.disconnect()
            except: pass
            btn.setText("连接")
            self.set_row_enabled(row, True)
            self.status_label.setText(f"通道 {channel_id} 已断开")
        else:
            # 连接逻辑
            try:
                comm = CANCommunicator(
                    on_message_received=self.handle_incoming_message,
                    on_status_changed=lambda s, c=channel_id: self.handle_status_update(s, c),
                    on_message_batch_received=self.enqueue_messages_batch
                )
                
                # 配置推送
                try:
                    comm.configure_receive_push(push_every_message=True, echo_filter_enabled=True)
                except: pass
                
                comm.connect(
                    interface=channel_id,
                    backend=backend,
                    is_fd=True,
                    arb_rate=arb_rate,
                    data_rate=data_rate,
                    sp=sp,
                    dsp=dsp
                )
                
                self.communicators[channel_id] = comm
                btn.setText("断开")
                self.set_row_enabled(row, False)
                self.status_label.setText(f"通道 {channel_id} 连接成功")
                
            except Exception as e:
                QMessageBox.critical(self, "连接失败", f"无法打开通道 {channel_id}: {str(e)}")
                if channel_id in self.communicators:
                    del self.communicators[channel_id]
        
        self.refresh_all_channel_combos()

    def connect_all_devices(self):
        # 收集所有行配置
        row_count = self.conn_table.rowCount()
        configs = {}
        channels_to_open = []
        backend = "candle" # 默认或检查第一个
        
        if row_count == 0:
            return

        # 检查是否已有连接
        if self.communicators:
            QMessageBox.warning(self, "警告", "请先断开现有连接")
            return

        for row in range(row_count):
            # Safe widget access
            b_widget = self.conn_table.cellWidget(row, 0)
            ch_widget = self.conn_table.cellWidget(row, 1)
            arb_widget = self.conn_table.cellWidget(row, 2)
            dat_widget = self.conn_table.cellWidget(row, 3)
            sp_widget = self.conn_table.cellWidget(row, 4)
            dsp_widget = self.conn_table.cellWidget(row, 5)
            
            if not (b_widget and ch_widget and arb_widget and dat_widget and sp_widget and dsp_widget):
                 continue

            b = b_widget.currentText()
            ch_str = ch_widget.currentText()
            
            if ch_str.startswith("can"):
                try: ch_id = int(ch_str.replace("can", ""))
                except: ch_id = 0
            elif " (SN:" in ch_str:
                try: ch_id = int(ch_str.split()[0])
                except: ch_id = 0
            else:
                try:
                    ch_id = int(ch_str)
                except:
                    ch_id = 0
            
            arb = parse_bitrate_token(arb_widget.currentText())
            dat = parse_bitrate_token(dat_widget.currentText())
            sp = sp_widget.value()
            dsp = dsp_widget.value()
            
            # 假设所有行使用相同后端 (Candle多通道场景)
            if row == 0: backend = b
            
            channels_to_open.append(ch_id)
            configs[ch_id] = {
                "arb_rate": arb,
                "data_rate": dat,
                "sp": sp,
                "dsp": dsp
            }
        
        # 创建一个共享 Communicator
        try:
            comm = CANCommunicator(
                on_message_received=self.handle_incoming_message,
                on_status_changed=lambda s, c=-1: self.handle_status_update(s, c), # 使用 -1 代表多通道
                on_message_batch_received=self.enqueue_messages_batch
            )
            comm.configure_receive_push(push_every_message=True, echo_filter_enabled=True)
            
            # 使用列表作为 interface，传递 channel_configs
            
            comm.connect(
                interface=channels_to_open,
                backend=backend,
                is_fd=True,
                arb_rate=configs[channels_to_open[0]]['arb_rate'], # 主波特率，用于fallback
                data_rate=configs[channels_to_open[0]]['data_rate'],
                channel_configs=configs
            )
            
            # 注册到 self.communicators
            for ch in channels_to_open:
                self.communicators[ch] = comm
                
            # 更新 GUI
            for row in range(row_count):
                btn = self.conn_table.cellWidget(row, 9)
                btn.setText("已连接(Multi)")
                btn.setEnabled(False) # 禁用单个断开
                self.set_row_enabled(row, False)
                
            self.btn_connect_all.setEnabled(False)
            self.btn_disconnect_all.setEnabled(True)
            self.refresh_all_channel_combos()
            self.status_label.setText(f"已连接通道: {channels_to_open}")
            
        except Exception as e:
            QMessageBox.critical(self, "连接失败", str(e))

    def disconnect_all_devices(self):
        if not self.communicators:
            return
            
        # 获取任意一个 comm 实例 (它们可能都是同一个)
        comm = next(iter(self.communicators.values()))
        try:
            comm.disconnect()
        except: pass
        
        self.communicators.clear()
        
        # 恢复 GUI
        row_count = self.conn_table.rowCount()
        for row in range(row_count):
            btn = self.conn_table.cellWidget(row, 9)
            btn.setText("连接")
            btn.setEnabled(True)
            self.set_row_enabled(row, True)
            
        self.btn_connect_all.setEnabled(True)
        self.btn_disconnect_all.setEnabled(False)
        self.refresh_all_channel_combos()
        self.status_label.setText("已断开所有连接")

    def set_row_enabled(self, row, enabled):
        for col in range(6): # 前6列是配置
            w = self.conn_table.cellWidget(row, col)
            if w: w.setEnabled(enabled)

    def refresh_all_channel_combos(self):
        # 刷新主窗口发送通道列表
        current = self.main_send_channel_combo.currentText()
        self.main_send_channel_combo.clear()
        channels = sorted(self.communicators.keys())
        if channels:
            self.main_send_channel_combo.addItems([str(c) for c in channels])
            if current in [str(c) for c in channels]:
                self.main_send_channel_combo.setCurrentText(current)
            self.send_group.setEnabled(True)
        else:
            self.main_send_channel_combo.addItem("无连接")
            self.send_group.setEnabled(False)
            
        # 刷新子窗口
        for win in self.send_windows:
            win.refresh_channels()

    def get_main_communicator(self):
        try:
            ch = int(self.main_send_channel_combo.currentText())
            return self.communicators.get(ch)
        except:
            return None

    def open_new_send_window(self):
        win = SendWindow(self.communicators, self)
        win.show()
        self.send_windows.append(win)

    # --- 接收处理 ---
    def handle_incoming_message(self, msg: CANMessage):
        self.message_received_signal.emit(msg)

    def handle_status_update(self, status: dict, channel: int):
        self.status_changed_signal.emit(status, channel)

    def enqueue_messages_batch(self, msgs: list):
        if not msgs: return
        now = datetime.now()
        for m in msgs:
            self.rx_buffer.append((m, now))

    @Slot(CANMessage)
    def enqueue_message(self, msg: CANMessage):
        self.rx_buffer.append((msg, datetime.now()))

    def flush_rx_buffer(self):
        if self.is_paused: return
        total = 0
        try: self.rx_table.setUpdatesEnabled(False)
        except: pass
        
        while self.rx_buffer and total < self.ui_max_rows_per_flush:
            msg, reception_time = self.rx_buffer.popleft()
            self.add_message_to_table(msg, reception_time)
            total += 1
            
        try: self.rx_table.setUpdatesEnabled(True)
        except: pass
        if total > 0:
            self.rx_table.scrollToBottom()

    def add_message_to_table(self, msg: CANMessage, reception_time: datetime):
        row_count = self.rx_table.rowCount()
        self.rx_table.insertRow(row_count)
        self.message_index += 1
        
        ts = f"{reception_time:%H:%M:%S}.{reception_time.microsecond//1000:03d}"
        
        # Determine Channel Name
        chan_attr = getattr(msg, 'channel', None)
        if isinstance(chan_attr, str) and chan_attr.startswith('TX:'):
            bus_name = chan_attr.split(':', 1)[1]
            is_tx = True
        elif isinstance(chan_attr, int):
            bus_name = str(chan_attr)
            is_tx = False
        else:
            bus_name = str(chan_attr) if chan_attr is not None else "?"
            is_tx = False
            
        direction = "TX" if is_tx else "RX"
        msg_id = f"{msg.arbitration_id:X}"
        msg_type = ("扩展" if msg.is_extended_id else "标准") + (" FD" if msg.is_fd else "")
        data = ' '.join(f'{b:02X}' for b in msg.data)

        self.rx_table.setItem(row_count, 0, QTableWidgetItem(str(self.message_index)))
        self.rx_table.setItem(row_count, 1, QTableWidgetItem(ts))
        self.rx_table.setItem(row_count, 2, QTableWidgetItem(direction))
        self.rx_table.setItem(row_count, 3, QTableWidgetItem(msg_id))
        self.rx_table.setItem(row_count, 4, QTableWidgetItem(msg_type))
        self.rx_table.setItem(row_count, 5, QTableWidgetItem(str(msg.dlc)))
        self.rx_table.setItem(row_count, 6, QTableWidgetItem(data))
        self.rx_table.setItem(row_count, 7, QTableWidgetItem(bus_name))
        
        color = QColor("blue") if is_tx else QColor("black")
        for i in range(8):
            item = self.rx_table.item(row_count, i)
            if item: item.setForeground(color)

    @Slot(dict, int)
    def update_status_bar(self, status: dict, channel: int):
        self.channel_statuses[channel] = status
        
        # 1. Update Connection Table Rows
        channels_data = status.get('channels', {})
        if not channels_data and channel >= 0:
             channels_data = {channel: status}
             
        row_count = self.conn_table.rowCount()
        for row in range(row_count):
            ch_widget = self.conn_table.cellWidget(row, 1)
            if not ch_widget:
                continue
            ch_str = ch_widget.currentText()
            try:
                if ch_str.startswith("can"):
                    ch_id = int(ch_str.replace("can", ""))
                elif " (SN:" in ch_str:
                    ch_id = int(ch_str.split()[0])
                else:
                    ch_id = int(ch_str)
            except:
                ch_id = -999
            
            if ch_id in channels_data:
                ch_stat = channels_data[ch_id]
                rx_fps = ch_stat.get('rx_fps', 0.0)
                tx_fps = ch_stat.get('tx_fps', 0.0)
                load = ch_stat.get('bus_load', 0.0)
                
                lbl_rx = self.conn_table.cellWidget(row, 6)
                lbl_tx = self.conn_table.cellWidget(row, 7)
                lbl_load = self.conn_table.cellWidget(row, 8)
                
                if lbl_rx: lbl_rx.setText(f"{rx_fps:.1f}")
                if lbl_tx: lbl_tx.setText(f"{tx_fps:.1f}")
                if lbl_load: lbl_load.setText(f"{load:.1f}%")

        # 2. Update Status Bar
        state_str = str(status.get('bus_state', 'Unknown'))
        rx_fps = status.get('rx_fps', 0.0)
        tx_fps = status.get('tx_fps', 0.0)
        load = status.get('bus_load', 0.0)
        
        ch_disp = "Multi" if channel == -1 else f"CH {channel}"
        text = f"[{ch_disp}] 状态: {state_str} | RX: {rx_fps:.1f} | TX: {tx_fps:.1f} | 负载: {load:.2f}%"
        self.status_label.setText(text)
        
        # Adaptive flush (using highest RX FPS)
        max_fps = 0.0
        for s in self.channel_statuses.values():
            max_fps = max(max_fps, s.get('rx_fps', 0.0))
            
        if self.adaptive_flush:
            if max_fps > 0.0:
                tick_ms = 1 if max_fps >= 1000.0 else max(1, int(1000.0 / max_fps))
                self.ui_timer.setInterval(tick_ms)
                rows_per_tick = max(1, int(max_fps * (tick_ms / 1000.0)))
                self.ui_max_rows_per_flush = max(1, min(1000, rows_per_tick))
            else:
                self.ui_timer.setInterval(50)
                self.ui_max_rows_per_flush = 20

    # --- 发送逻辑 (主窗口) ---
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

    def do_send_once(self):
        comm = self.get_main_communicator()
        if not comm: return
        try:
            try:
                ch = int(self.main_send_channel_combo.currentText())
            except:
                ch = None

            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()
            comm.send_one(msg_id, data, self.send_ext_checkbox.isChecked(), "FD" in frame_type, "+BRS" in frame_type, channel=ch)
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))

    def do_send_burst(self):
        if self.stop_burst_func:
            try:
                self.stop_burst_func()
            except:
                pass
            self.stop_burst_func = None
            self.burst_send_button.setText("发送")
            return
            
        comm = self.get_main_communicator()
        if not comm: return
        try:
            count = self.burst_count_input.value()
            interval_ms = self.burst_interval_input.value()
            if interval_ms <= 0: interval_ms = 0.001
            freq = 1000.0 / interval_ms
            
            try:
                ch_str = self.main_send_channel_combo.currentText()
                ch = int(ch_str)
            except:
                ch = None

            msg_id = int(self.send_id_input.text(), 16)
            data = self.prepare_data_for_send()
            frame_type = self.frame_type_combo.currentText()
            
            self.burst_send_button.setText("停止发送")
            
            def on_finish():
                self.burst_finished_signal.emit()

            self.stop_burst_func = comm.start_burst_send(
                msg_id, data, freq, count,
                self.send_ext_checkbox.isChecked(),
                "FD" in frame_type, "+BRS" in frame_type,
                channel=ch,
                on_finish=on_finish
            )
        except Exception as e:
            QMessageBox.warning(self, "发送错误", str(e))
            self.burst_send_button.setText("发送")

    def on_burst_finished(self):
        self.stop_burst_func = None
        self.burst_send_button.setText("发送")

    def _send_one_burst_message(self):
        pass

    def update_periodic_btn_state(self):
        comm = self.get_main_communicator()
        if not comm:
             self.periodic_send_button.setText("开始周期发送")
             return
        
        try:
            ch_str = self.main_send_channel_combo.currentText()
            ch = int(ch_str)
        except:
            ch = None

        if comm.is_periodic_sending(ch):
            self.periodic_send_button.setText("停止周期发送")
        else:
            self.periodic_send_button.setText("开始周期发送")

    def toggle_periodic_send(self):
        comm = self.get_main_communicator()
        if not comm: return
        
        try:
            ch_str = self.main_send_channel_combo.currentText()
            ch = int(ch_str)
        except:
            ch = None
            
        running = comm.is_periodic_sending(ch)

        if running:
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
                    channel=ch
                )
                self.periodic_send_button.setText("停止周期发送")
            except Exception as e:
                QMessageBox.warning(self, "发送错误", str(e))

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

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        self.pause_rx_button.setText("继续显示" if self.is_paused else "暂停显示")

    def clear_table(self):
        self.rx_table.setRowCount(0)
        self.message_index = 0
        self.rx_buffer.clear()

    def save_table_data(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存数据", "", "CSV Files (*.csv);;All Files (*)")
        if not path: return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                headers = [self.rx_table.horizontalHeaderItem(i).text() for i in range(self.rx_table.columnCount())]
                w.writerow(headers)
                for r in range(self.rx_table.rowCount()):
                    row = [self.rx_table.item(r, c).text() if self.rx_table.item(r, c) else "" for c in range(self.rx_table.columnCount())]
                    w.writerow(row)
        except Exception as e:
            QMessageBox.warning(self, "保存错误", str(e))

    def closeEvent(self, event):
        for comm in self.communicators.values():
            try: comm.disconnect()
            except: pass
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CANToolGUI()
    window.show()
    sys.exit(app.exec())
