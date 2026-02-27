"""VESC_CAN: VESC 无刷电机驱动的 CAN 协议编解码器
移自 tz_arm/can_bridge/can_py.py
"""
import time
from ctypes import Structure, c_int, c_float
from typing import Optional, Tuple

import numpy as np

from .base import CANProtocolBase


# ---------------------------------------------------------------------------
# 缓冲区辅助函数
# ---------------------------------------------------------------------------

def buffer_get_int16(buffer, index):
    value = buffer[index] << 8 | buffer[index + 1]
    return np.int16(value)


def buffer_get_int32(buffer, index):
    value = (buffer[index] << 24 | buffer[index + 1] << 16
             | buffer[index + 2] << 8 | buffer[index + 3])
    return np.int32(value)


def buffer_get_float16(buffer, scale, index):
    value = buffer_get_int16(buffer, index)
    return float(value) / scale


def buffer_get_float32(buffer, scale, index):
    value = buffer_get_int32(buffer, index)
    return float(value) / scale


# ---------------------------------------------------------------------------
# VESC 协议常量与数据结构
# ---------------------------------------------------------------------------

class VESC_CAN_STATUS:
    VESC_ID_A = 20
    VESC_ID_B = 25
    VESC_CAN_PACKET_STATUS_1 = 0x09
    VESC_CAN_PACKET_STATUS_2 = 0x0E
    VESC_CAN_PACKET_STATUS_3 = 0x0F
    VESC_CAN_PACKET_STATUS_4 = 0x10
    VESC_CAN_PACKET_STATUS_5 = 0x1B


class VESC_PACK(Structure):
    _fields_ = [
        ("id", c_int),
        ("rpm", c_int),
        ("current", c_float),
        ("pid_pos_now", c_float),
        ("amp_hours", c_float),
        ("amp_hours_charged", c_float),
        ("watt_hours", c_float),
        ("watt_hours_charged", c_float),
        ("temp_fet", c_float),
        ("temp_motor", c_float),
        ("tot_current_in", c_float),
        ("duty", c_float),
        ("tachometer_value", c_float),
        ("input_voltage", c_float),
    ]


# ---------------------------------------------------------------------------
# VESC CAN 协议类
# ---------------------------------------------------------------------------

class VESC_CAN(CANProtocolBase):
    """VESC 无刷电机驱动的 CAN 协议编解码器。

    推荐通过工厂方法创建（继承自 TZCanInterface.open）：
        vesc = VESC_CAN.open(0, 1, abaudrate=500000)

    也可传入已初始化的 TZCANTransmitter 实例：
        vesc = VESC_CAN(tx0, tx1, channels=(0, 1))
    """

    def __init__(self, *transmitters, channels=None):
        super().__init__(*transmitters, channels=channels)
        self.can_packet = VESC_PACK()

    # open() 继承自 TZCanInterface，cls=VESC_CAN 时自动调用本类 __init__，无需重写。

    def send_pass_through(self, _id: np.uint8, _pos: float, _rpm: float, _cur: float, can_channel=0):
        id_ = _id + 0x3F00
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        pos_int = np.uint16(int(_pos * 100))
        rpm_int = np.uint16(int(_rpm))
        cur_int = np.uint16(int(_cur * 1000))
        data[0] = (pos_int >> 8) & 0xff
        data[1] = pos_int & 0xff
        data[2] = (rpm_int >> 8) & 0xff
        data[3] = rpm_int & 0xff
        data[4] = (cur_int >> 8) & 0xff
        data[5] = cur_int & 0xff
        ret = self.send(id_, data, can_channel=can_channel)
        if not ret:
            print(f"❌ SEND vesc id: {id_ & 0xff} failed")
            print(f"time: {time.time():.4f}")

    def send_pos(self, _id: np.uint8, _pos: float, can_channel=0):
        id_ = _id + 0x400
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        pos_int = np.uint32(int(_pos * 1e6))
        data[0] = (pos_int >> 24) & 0xff
        data[1] = (pos_int >> 16) & 0xff
        data[2] = (pos_int >> 8) & 0xff
        data[3] = pos_int & 0xff
        print(f"SEND vesc id: {id_ & 0xff}, pos: {pos_int}, data: {data}")
        self.send(id_, data, can_channel=can_channel)

    def send_rpm(self, _id: np.uint8, _rpm: float, can_channel=0):
        id_ = _id + 0x300
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        rpm_int = np.uint32(int(_rpm))
        data[0] = (rpm_int >> 24) & 0xff
        data[1] = (rpm_int >> 16) & 0xff
        data[2] = (rpm_int >> 8) & 0xff
        data[3] = rpm_int & 0xff
        self.send(id_, data, can_channel=can_channel)

    def send_current(self, _id: np.uint8, _cur: float, can_channel=0):
        id_ = _id + 0x100
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        off_delay_int = np.uint16(0)
        cur_int = np.uint32(int(_cur * 1000))
        data[0] = (off_delay_int >> 8) & 0xff
        data[1] = off_delay_int & 0xff
        data[2] = (cur_int >> 24) & 0xff
        data[3] = (cur_int >> 16) & 0xff
        data[4] = (cur_int >> 8) & 0xff
        data[5] = cur_int & 0xff
        ret = self.send(id_, data, can_channel=can_channel)
        if not ret:
            print(f"❌ SEND vesc id: {id_ & 0xff} failed")

    def receive_decode(self, timeout=1, can_channel=0) -> Tuple[Optional[int], Optional[VESC_PACK]]:
        id_, data = self.receive(timeout, can_channel=can_channel)
        if id_ is None:
            return None, None

        self.can_packet.id = id_ & 0xff
        status_id = (id_ >> 8) & 0xff

        if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
            self.can_packet.rpm = int(buffer_get_float32(data, 1, 0))
            self.can_packet.current = buffer_get_float16(data, 1e2, 4)
            self.can_packet.pid_pos_now = buffer_get_float16(data, 50.0, 6)
        else:
            return None, None

        if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_2:
            self.can_packet.amp_hours = buffer_get_float32(data, 1e4, 0)
            self.can_packet.amp_hours_charged = buffer_get_float32(data, 1e4, 4)
        if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_3:
            self.can_packet.watt_hours = buffer_get_float32(data, 1e4, 0)
            self.can_packet.watt_hours_charged = buffer_get_float32(data, 1e4, 4)
        if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_4:
            self.can_packet.temp_fet = buffer_get_float16(data, 1e1, 0)
            self.can_packet.temp_motor = buffer_get_float16(data, 1e1, 2)
            self.can_packet.tot_current_in = buffer_get_float16(data, 1e1, 4)
            self.can_packet.duty = buffer_get_float16(data, 1e3, 6)
        if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_5:
            self.can_packet.tachometer_value = buffer_get_float32(data, 1, 0)
            self.can_packet.input_voltage = buffer_get_float16(data, 1e1, 4)

        return id_, self.can_packet
