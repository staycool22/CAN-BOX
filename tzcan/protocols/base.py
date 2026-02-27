"""CANProtocolBase: 基于 CANMessageTransmitter 的薄封装适配层
将底层 CAN 接口适配为更简洁的 send/receive 接口，供上层协议继承。
移自 tz_arm/tzcan/can_py.py
"""
from typing import Optional, Tuple


class CANProtocolBase:
    """协议适配层：将 TZUSB2CANTransmitter 的接口适配为
    更简洁的 send / receive / receiveFD，供上层协议类（如 VESC_CAN）继承。

    一个实例对应一条 CAN 总线。多条总线场景在应用层创建多个实例，
    不在协议层做通道路由。

    Args:
        transmitter: 已初始化的 TZUSB2CANTransmitter 实例。
    """

    def __init__(self, transmitter):
        self.transmitter = transmitter
        self._hw_handle = None  # 预留：子类可在此存放硬件句柄

    def close(self):
        """显式释放硬件资源（推荐优先于依赖 __del__）。"""
        if self._hw_handle is not None:
            hw = self._hw_handle
            try:
                hw['TX'].close_can_device(hw['m_dev'])
            except Exception:
                pass
            self._hw_handle = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def send(self, _id: int, _data: list) -> bool:
        """发送 CAN 帧（扩展帧）。"""
        return self.transmitter._send_can_data(_id, _data, is_ext_frame=True)

    def receive(self, timeout: float) -> Tuple[Optional[int], Optional[list]]:
        """接收 CAN 帧，返回 (arbitration_id, data) 或 (None, None)。"""
        result = self.transmitter._receive_can_data(
            timeout=timeout, is_ext_frame=True, return_msg=True
        )
        if result[0] and len(result) == 3 and result[2] is not None:
            msg = result[2]
            return msg.arbitration_id, list(msg.data)
        return None, None

    def receiveFD(self, timeout: float) -> Tuple[Optional[int], Optional[list]]:
        """接收 CAN FD 帧，返回 (arbitration_id, data) 或 (None, None)。"""
        result = self.transmitter._receive_can_data(
            timeout=timeout, canfd_mode=True, return_msg=True
        )
        if result[0] and len(result) == 3 and result[2] is not None:
            msg = result[2]
            return msg.arbitration_id, list(msg.data)
        return None, None


# 向后兼容别名
TZCanInterface = CANProtocolBase
