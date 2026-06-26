# 说明：信号解析与绘图「离线假数据」演示（无需 CAN 硬件）
# 用途：用正弦/三角/锯齿/方波合成 CAN 帧，喂入 SignalPlotWindow，离线即可看到曲线滚动。
#       覆盖「不分段」信号（sine/ramp/temp_le/flag）与「分段拼接」信号（rpm_split）。
# 运行：python3 tests/signal_plot_offline_demo.py
import os
import sys
import math
import time
import json

try:
    from gui.signal_plot import SignalPlotWindow, CodecDatabase
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from gui.signal_plot import SignalPlotWindow, CodecDatabase

import can
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

_PROTO_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "gui", "signal_plot", "protocols",
)
PROTO_PATH = os.path.join(_PROTO_DIR, "demo_offline.json")
LAYOUT_PATH = os.path.join(_PROTO_DIR, "demo_offline_layout.json")
DEMO_ID = 0x100


def fake_values(t: float) -> dict:
    """按时间 t（秒）生成各信号的工程量。"""
    ph = (t % 6.0) / 6.0
    triangle = (4 * ph - 1) if ph < 0.5 else (3 - 4 * ph)  # -1→1→-1
    return {
        "sine": 50.0 * math.sin(2 * math.pi * t / 5.0),     # ±50 A 正弦（不分段 int16 大端）
        "ramp": (t * 20.0) % 100.0,                          # 0~100 % 锯齿（不分段 uint8）
        "temp_le": 25.0 + 10.0 * math.sin(2 * math.pi * t / 8.0),  # 温度（不分段 int16 小端）
        "flag": 1 if math.sin(2 * math.pi * t / 2.0) >= 0 else 0,  # 方波（不分段 1 位）
        "rpm_split": round(2000 * triangle),                 # ±2000 rpm 三角（分段：byte5 高 + byte7 低）
    }


def make_frame(db: CodecDatabase, t: float) -> can.Message:
    """把 t 时刻的工程量编码成一帧 CAN 报文（复用编解码引擎的 encode）。"""
    data = db.encode_message("DEMO", fake_values(t))
    msg = can.Message(arbitration_id=DEMO_ID, is_extended_id=False, data=data)
    msg.channel = "demo0"  # 假通道名，便于在「数据源」下拉里看到
    return msg


def main():
    app = QApplication(sys.argv)
    db = CodecDatabase.load_json(PROTO_PATH)

    win = SignalPlotWindow()
    win.db = db
    win._on_db_changed()

    # 应用演示布局（两个窗格）
    layout = json.load(open(LAYOUT_PATH, encoding="utf-8"))
    for pane in list(win.panes):
        win.remove_pane(pane)
    win.window_sec = float(layout.get("window_sec", 10.0))
    win.window_spin.setValue(win.window_sec)
    win.auto_scroll = bool(layout.get("auto_scroll", True))
    win.autoscroll_check.setChecked(win.auto_scroll)
    for pd in layout.get("panes", []):
        pane = win.add_pane()
        pane.set_signals(pd.get("signals", []), pd.get("colors"))

    win.setWindowTitle("信号解析与绘图 — 离线假数据演示")
    win.show()

    t0 = time.perf_counter()

    def tick():
        win.on_frames([make_frame(db, time.perf_counter() - t0)])

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(20)  # 50 Hz 喂帧

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
