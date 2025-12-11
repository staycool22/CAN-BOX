from abc import ABC, abstractmethod

class CANMessageTransmitter(ABC):
    """通用 CAN/CANFD 消息发送接收与设备管理的抽象基类。

    具体设备实现（如 ZCANTransmitter）需实现以下抽象方法。
    """

    def __init__(self, channel_handle):
        self.channel_handle = channel_handle

    @abstractmethod
    def _send_can_data(self, send_id, data_list, is_ext_frame=False, canfd_mode=False, brs=0, esi=0):
        """发送 CAN/CANFD 数据。"""
        pass

    @abstractmethod
    def _receive_can_data(self, target_id=None, timeout=5, is_ext_frame=None, canfd_mode=False):
        """接收 CAN/CANFD 数据。"""
        pass

    @staticmethod
    @abstractmethod
    def init_can_device(baud_rate=500000, dbit_baud_rate=2000000, channels=[0], can_type=0, canfd_standard=0, channel_count=None):
        """初始化 CAN 设备，返回 (m_dev, ch0, ch1)。"""
        pass

    @staticmethod
    @abstractmethod
    def close_can_device(m_dev, channel_handle0=None, channel_handle1=None):
        """关闭 CAN 设备与通道。"""
        pass

    @staticmethod
    def choose_can_device(backend="ZCAN"):
        """
        仅进行设备选择并返回具体实现类。
        - "ZCAN": 返回 ZCANTransmitter 类
        - "TZCAN": 返回 TZCANTransmitter 类（基于 python-can/socketcan）
        其他设备可按需扩展并在此分派。
        """
        backend = (backend or "").upper()
        if backend == "ZCAN":
            # 延迟导入以避免循环依赖
            try:
                from .ZCANTransmitter import ZCANTransmitter
                return ZCANTransmitter
            except ImportError:
                 # 尝试从同级目录直接导入（如果不作为包使用）
                try:
                    from ZCANTransmitter import ZCANTransmitter
                    return ZCANTransmitter
                except ImportError:
                     raise ImportError("无法导入 ZCANTransmitter")
        if backend == "TZCAN":
            # 延迟导入以避免循环依赖
            try:
                from .TZCANTransmitter import TZCANTransmitter
                return TZCANTransmitter
            except ImportError:
                # 尝试从同级目录直接导入（如果不作为包使用）
                try:
                    from TZCANTransmitter import TZCANTransmitter
                    return TZCANTransmitter
                except ImportError:
                    raise ImportError("无法导入 TZCANTransmitter")
        raise ValueError(f"不支持的设备类型: {backend}")
