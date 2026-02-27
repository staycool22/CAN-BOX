"""CANProtocolBase: 基于 CANMessageTransmitter 的薄封装适配层
将底层 CAN 接口适配为更简洁的 send/receive 接口，供上层协议继承。
移自 tz_arm/can_bridge/can_py.py
"""
from typing import Optional, List, Tuple


class CANProtocolBase:
    """
    协议适配层：将 TZCANTransmitter 的接口适配为
    更简洁的 send(id, data, ch) / receive(timeout, ch) 接口，
    供上层协议类（如 VESC_CAN）继承使用。

    __init__ 接受已初始化的 TZCANTransmitter 实例，不负责硬件初始化。
    """

    def __init__(self, *transmitters, channels=None):
        """
        接受已初始化的 TZCANTransmitter 实例。

        Args:
            *transmitters: TZCANTransmitter 实例，每个对应一个 CAN 通道。
            channels: 可选整数元组，指定每个 transmitter 对应的逻辑通道号。
                      若为 None，默认按顺序使用 0, 1, ...。
        """
        self.can_transmitter: List[Optional[object]] = list(transmitters)
        self.channels = tuple(channels) if channels is not None else tuple(range(len(transmitters)))
        self._hw_handle = None  # 预留：子类可在此存放硬件句柄

    def close(self):
        """显式释放硬件资源（推荐优先于依赖 __del__）。"""
        if self._hw_handle is not None:
            hw = self._hw_handle
            try:
                hw['TX'].close_can_device(hw['m_dev'], hw['ch0'], hw['ch1'])
            except Exception:
                pass
            self._hw_handle = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def send(self, _id: int, _data: list, can_channel: int = 0) -> bool:
        """发送 CAN 帧（扩展帧）"""
        idx = self.channels.index(can_channel)
        return self.can_transmitter[idx]._send_can_data(_id, _data, is_ext_frame=True)

    def receive(self, timeout: float, can_channel: int = 0) -> Tuple[Optional[int], Optional[list]]:
        """接收 CAN 帧，返回 (arbitration_id, data) 或 (None, None)"""
        idx = self.channels.index(can_channel)
        result = self.can_transmitter[idx]._receive_can_data(
            timeout=timeout, is_ext_frame=True, return_msg=True
        )
        if result[0] and len(result) == 3 and result[2] is not None:
            msg = result[2]
            return msg.arbitration_id, list(msg.data)
        return None, None

    def receiveFD(self, timeout: float, can_channel: int = 0) -> Tuple[Optional[int], Optional[list]]:
        """接收 CAN FD 帧，返回 (arbitration_id, data) 或 (None, None)"""
        idx = self.channels.index(can_channel)
        result = self.can_transmitter[idx]._receive_can_data(
            timeout=timeout, canfd_mode=True, return_msg=True
        )
        if result[0] and len(result) == 3 and result[2] is not None:
            msg = result[2]
            return msg.arbitration_id, list(msg.data)
        return None, None


# 向后兼容别名
TZCanInterface = CANProtocolBase
