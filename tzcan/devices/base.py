"""CANMessageTransmitter: CAN/CAN FD 设备抽象基类（设备注册表插件化）"""
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple


class CANMessageTransmitter(ABC):
    """通用 CAN/CANFD 消息发送接收与设备管理的抽象基类。

    具体设备实现（如 TZCANTransmitter）需实现以下抽象方法，
    并用 @CANMessageTransmitter.register("KEY") 自行注册。

    示例::
        @CANMessageTransmitter.register("FOO")
        class FooTransmitter(CANMessageTransmitter):
            ...
    """

    _registry: Dict[str, type] = {}

    def __init__(self, channel_handle):
        self.channel_handle = channel_handle

    @classmethod
    def register(cls, key: str):
        """设备注册装饰器，供各实现类在定义时自行注册。

        用法::
            @CANMessageTransmitter.register("TZCAN")
            class TZCANTransmitter(CANMessageTransmitter): ...
        """
        def _decorator(device_cls):
            cls._registry[key.upper()] = device_cls
            return device_cls
        return _decorator

    @staticmethod
    def choose_can_device(backend="TZCAN"):
        """根据名称从注册表中返回对应的设备实现类。

        Args:
            backend: 已注册的设备名称，不区分大小写（如 "TZCAN"、"TZETHCAN"）。

        Returns:
            对应的设备类（未实例化）。

        Raises:
            ValueError: 若名称未在注册表中。
        """
        key = (backend or "").upper()
        device_cls = CANMessageTransmitter._registry.get(key)
        if device_cls is None:
            raise ValueError(
                f"不支持的设备类型: {backend}，已注册: {list(CANMessageTransmitter._registry)}"
            )
        return device_cls

    @classmethod
    def open(cls, device="TZCAN", **kwargs):
        """选择设备并初始化，一步完成。

        等价于::
            TX = CANMessageTransmitter.choose_can_device(device)
            m_dev, ch0, ch1 = TX.init_can_device(**kwargs)

        Args:
            device: 已注册的设备名称（"TZCAN"、"TZETHCAN"），不区分大小写。
                    注意：不要与 kwargs 中的 backend（CAN 物理后端，如 "socketcan"）
                    混淆，两者含义不同。
            **kwargs: 透传给 init_can_device 的所有参数
                     （baud_rate, dbit_baud_rate, channels, fd, backend, sp, dsp, ...）

        Returns:
            (device_cls, m_dev, ch0, ch1)
            device_cls 保留供后续 TX(bus) 实例化和 TX.close_can_device() 调用使用。

        示例::
            TX, m_dev, ch0, _ = CANMessageTransmitter.open("TZCAN",
                baud_rate=500000, channels=[0], backend="socketcan")
            tx = TX(ch0)
            ...
            TX.close_can_device(m_dev)
        """
        device_cls = cls.choose_can_device(device)
        m_dev, ch0, ch1 = device_cls.init_can_device(**kwargs)
        return device_cls, m_dev, ch0, ch1

    @abstractmethod
    def _send_can_data(self, send_id, data_list, is_ext_frame=False,
                       canfd_mode=False, brs=0, esi=0):
        """发送 CAN/CANFD 数据。"""
        pass

    @abstractmethod
    def _receive_can_data(self, target_id=None, timeout=5,
                          is_ext_frame=None, canfd_mode=False,
                          return_msg=False):
        """接收 CAN/CANFD 数据。"""
        pass

    @staticmethod
    @abstractmethod
    def init_can_device(baud_rate=500000, dbit_baud_rate=2000000,
                        channels=None, fd=False):
        """初始化 CAN 设备，返回 (m_dev, ch0, ch1)。"""
        pass

    @staticmethod
    @abstractmethod
    def close_can_device(m_dev, channel_handle0=None, channel_handle1=None):
        """关闭 CAN 设备与通道。"""
        pass
