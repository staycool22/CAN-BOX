"""信号解析与绘图特性包。

- codec.py  : 纯 Python 编解码引擎（无 Qt 依赖），可单独使用/测试。
- window.py : PySide6 + pyqtgraph 的独立窗口 UI（协议编辑器 + 实时数值表 + 多路曲线图）。

注意：导入本包会一并加载 window.py，从而引入 pyqtgraph。
只需要编解码引擎、不想引入 GUI 依赖时，请直接 `from gui.signal_plot.codec import ...`。
"""
from .codec import CodecDatabase, MessageDef, SignalDef, Segment
from .window import SignalPlotWindow

__all__ = ["CodecDatabase", "MessageDef", "SignalDef", "Segment", "SignalPlotWindow"]
