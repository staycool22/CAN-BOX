"""VESC_CAN: VESC 无刷电机驱动的 CAN 协议编解码器
移自 tz_arm/tzcan/can_py.py
"""
import time
from ctypes import Structure, c_int, c_float
from typing import Dict, Optional, Tuple

import numpy as np

from .base import CANProtocolBase


# ---------------------------------------------------------------------------
# 缓冲区辅助函数
# ---------------------------------------------------------------------------

def buffer_get_int(buffer, index, size):
    return int.from_bytes(bytes(buffer[index:index + size]), byteorder="big", signed=True)

def buffer_get_int16(buffer, index):
    return buffer_get_int(buffer, index, 2)


def buffer_get_int32(buffer, index):
    return buffer_get_int(buffer, index, 4)


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
        ("enc1_deg", c_float),
        ("enc2_deg", c_float),
        ("encoder_laps", c_int),
        ("last_rx_status_id", c_int),
    ]


# ---------------------------------------------------------------------------
# VESC CAN 协议类
# ---------------------------------------------------------------------------

class VESC_CAN(CANProtocolBase):
    """VESC 无刷电机驱动的 CAN 协议编解码器。

    一个实例对应一条 CAN 总线。同一总线上的多个 VESC 电机控制器
    通过 vesc_id（编码在仲裁 ID 中）区分；不同总线上的电机在应用层
    创建多个 VESC_CAN 实例。

    用法::
        TX, m_dev, _, _ = CANMessageTransmitter.open("TZUSB2CAN",
            baud_rate=500000, channels=[0])
        vesc = VESC_CAN(TX(m_dev["buses"][0]))

        vesc.send_rpm(vesc_id=1, rpm=2000)
        _, pack = vesc.receive_decode(timeout=0.1)
    """

    def __init__(self, transmitter):
        super().__init__(transmitter)
        self.can_packet = VESC_PACK()
        self._pack_by_dev_id: dict = {}

    @staticmethod
    def _clamp_int32(value):
        value = int(value)
        if value < -2147483648:
            return -2147483648
        if value > 2147483647:
            return 2147483647
        return value

    def send_pass_through(self, _id: np.uint8, _pos: float, _rpm: float, _cur: float):
        id_ = _id + 0x3F00
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        pos_int = int(round(_pos * 100))
        rpm_int = int(round(_rpm))
        cur_int = int(round(_cur * 1000))
        pos_u = pos_int & 0xfffff
        rpm_u = rpm_int & 0xfffff
        cur_u = cur_int & 0xfffff
        data[0] = (pos_u >> 8) & 0xff
        data[1] = pos_u & 0xff
        data[2] = (rpm_u >> 8) & 0xff
        data[3] = rpm_u & 0xff
        data[4] = (cur_u >> 8) & 0xff
        data[5] = cur_u & 0xff
        ret = self.send(id_, data)
        if not ret:
            print(f"❌ SEND vesc id: {id_ & 0xff} failed")
            print(f"time: {time.time():.4f}")

    def send_pos(self, _id: np.uint8, _pos: float):
        id_ = _id + 0x400
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        pos_int = self._clamp_int32(round(float(_pos) * 1e6))
        pos_bytes = int(pos_int).to_bytes(4, byteorder="big", signed=True)
        data[0] = pos_bytes[0]
        data[1] = pos_bytes[1]
        data[2] = pos_bytes[2]
        data[3] = pos_bytes[3]
        print(f"SEND vesc id: {id_ & 0xff}, pos: {pos_int}, data: {data}")
        self.send(id_, data)

    def send_vel_cur(self, _id:np.uint8, _cur:float, _vel:float):
        id = _id + 0x200
        data = [0, 0, 0, 0, 0, 0, 0, 0]

        # 处理速度值，将其转换为有符号32位整数 (Byte 0-3)
        vel_int = int(round(_vel))
        # 确保速度值在32位有符号整数范围内
        if vel_int < -(2**31):
            vel_int = -(2**31)
        elif vel_int > (2**31 - 1):
            vel_int = (2**31 - 1)
            
        data[0] = (vel_int >> 24) & 0xff 
        data[1] = (vel_int >> 16) & 0xff
        data[2] = (vel_int >> 8) & 0xff
        data[3] = vel_int & 0xff
        
        # 处理电流值，将其转换为有符号32位整数 (Byte 4-7)
        cur_int = int(round(_cur * 1000.0))
        # 确保电流值在32位有符号整数范围内
        if cur_int < -(2**31):
            cur_int = -(2**31)
        elif cur_int > (2**31 - 1):
            cur_int = (2**31 - 1)
            
        data[4] = (cur_int >> 24) & 0xff
        data[5] = (cur_int >> 16) & 0xff
        data[6] = (cur_int >> 8) & 0xff
        data[7] = cur_int & 0xff
        
        self.send(id, data)

    def send_rpm(self, _id: np.uint8, _rpm: float):
        id_ = _id + 0x300
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        rpm_int = self._clamp_int32(round(float(_rpm)))
        rpm_bytes = int(rpm_int).to_bytes(4, byteorder="big", signed=True)
        data[0] = rpm_bytes[0]
        data[1] = rpm_bytes[1]
        data[2] = rpm_bytes[2]
        data[3] = rpm_bytes[3]
        self.send(id_, data)

    def send_current(self, _id: np.uint8, _cur: float):
        id_ = _id + 0x100
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        off_delay_int = np.uint16(0)
        cur_int = self._clamp_int32(round(float(_cur) * 1000.0))
        cur_bytes = int(cur_int).to_bytes(4, byteorder="big", signed=True)
        data[0] = (off_delay_int >> 8) & 0xff
        data[1] = off_delay_int & 0xff
        data[2] = cur_bytes[0]
        data[3] = cur_bytes[1]
        data[4] = cur_bytes[2]
        data[5] = cur_bytes[3]
        ret = self.send(id_, data)
        if not ret:
            print(f"❌ SEND vesc id: {id_ & 0xff} failed")

    def send_pid_parameter(self, _id: np.uint8, param_type: int | str, value: float, save: bool = False):
        id_ = _id + 0x4400
        data = [0, 0, 0, 0, 0, 0]
        param_map: Dict[str, int] = {
            "speed_kp": 0x0,
            "speed_ki": 0x1,
            "speed_kd": 0x2,
            "position_kp": 0x3,
            "position_ki": 0x4,
            "position_kd": 0x5,
        }
        if isinstance(param_type, str):
            key = param_type.strip().lower()
            if key not in param_map:
                raise ValueError(
                    "param_type 必须是 0x0~0x5，或以下字符串之一: "
                    "speed_kp, speed_ki, speed_kd, position_kp, position_ki, position_kd"
                )
            param_code = param_map[key]
        else:
            param_code = int(param_type)
            if param_code < 0x0 or param_code > 0x5:
                raise ValueError("param_type 超出范围，必须在 0x0 ~ 0x5 之间")

        if value < 0.0:
            raise ValueError("PID 参数值必须为非负数")

        scaled_value = int(round(float(value) * 1000000.0))
        if scaled_value < 0:
            scaled_value = 0
        elif scaled_value > 0xFFFFFFFF:
            scaled_value = 0xFFFFFFFF

        data[0] = param_code & 0xFF
        data[1] = (scaled_value >> 24) & 0xFF
        data[2] = (scaled_value >> 16) & 0xFF
        data[3] = (scaled_value >> 8) & 0xFF
        data[4] = scaled_value & 0xFF
        data[5] = 0x01 if bool(save) else 0x00
        ret = self.send(id_, data)
        if not ret:
            print(f"❌ SEND vesc id: {id_ & 0xff} failed")

    def receive_pid_parameter(
        self,
        _id: np.uint8,
        param_type: int | str,
    ) -> Tuple[Optional[int], Optional[Dict[str, float | int | bool]]]:
        expected_id = int(_id) + 0x4400
        param_map: Dict[str, int] = {
            "speed_kp": 0x0,
            "speed_ki": 0x1,
            "speed_kd": 0x2,
            "position_kp": 0x3,
            "position_ki": 0x4,
            "position_kd": 0x5,
        }
        if isinstance(param_type, str):
            key = param_type.strip().lower()
            if key not in param_map:
                raise ValueError(
                    "param_type 必须是 0x0~0x5，或以下字符串之一: "
                    "speed_kp, speed_ki, speed_kd, position_kp, position_ki, position_kd"
                )
            expected_param_code = param_map[key]
        else:
            expected_param_code = int(param_type)
            if expected_param_code < 0x0 or expected_param_code > 0x5:
                raise ValueError("param_type 超出范围，必须在 0x0 ~ 0x5 之间")
        deadline = time.time() + max(0.0, float(timeout))

        while True:
            remain = max(0.0, deadline - time.time())
            arb_id, data = self.receive(timeout=remain)
            if arb_id is None or data is None:
                return None, None
            if int(arb_id) != expected_id:
                if time.time() >= deadline:
                    return None, None
                continue
            if len(data) < 6:
                if time.time() >= deadline:
                    return None, None
                continue
            param_code = int(data[0]) & 0xFF
            if param_code != expected_param_code:
                if time.time() >= deadline:
                    return None, None
                continue

            raw_value = (
                ((int(data[1]) & 0xFF) << 24)
                | ((int(data[2]) & 0xFF) << 16)
                | ((int(data[3]) & 0xFF) << 8)
                | (int(data[4]) & 0xFF)
            )
            value = float(raw_value) / 1000000.0
            save = bool(int(data[5]) & 0x01)
            payload: Dict[str, float | int | bool] = {
                "param_type": param_code,
                "value": value,
                "save": save,
                "raw_value": raw_value,
            }
            return arb_id, payload


    def receive_decode(self, timeout=0) -> Tuple[Optional[int], Optional[VESC_PACK]]:
        id_, data = self.receive(timeout=timeout)
        if id_ is None:
            return None, None

        dev_id = int(id_ & 0xff)
        pack = self._pack_by_dev_id.get(dev_id)
        if pack is None:
            pack = VESC_PACK()
            self._pack_by_dev_id[dev_id] = pack
        pack.id = dev_id
        status_id = (id_ >> 8) & 0xff

        if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
            pack.last_rx_status_id = status_id
            pack.rpm = int(buffer_get_float32(data, 1, 0))
            pack.current = buffer_get_float16(data, 1e2, 4)
            pack.pid_pos_now = buffer_get_float16(data, 50.0, 6)
        elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_2:
            pack.last_rx_status_id = status_id
            pack.enc1_deg = buffer_get_float16(data, 50.0, 0)
            pack.enc2_deg = buffer_get_float16(data, 50.0, 2)
            pack.encoder_laps = int(buffer_get_int32(data, 4))
        elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_3:
            pack.last_rx_status_id = status_id
            pack.watt_hours = buffer_get_float32(data, 1e4, 0)
            pack.watt_hours_charged = buffer_get_float32(data, 1e4, 4)
        elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_4:
            pack.last_rx_status_id = status_id
            pack.temp_fet = buffer_get_float16(data, 1e1, 0)
            pack.temp_motor = buffer_get_float16(data, 1e1, 2)
            pack.tot_current_in = buffer_get_float16(data, 1e1, 4)
            pack.duty = buffer_get_float16(data, 1e3, 6)
        elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_5:
            pack.last_rx_status_id = status_id
            pack.tachometer_value = buffer_get_float32(data, 1, 0)
            pack.input_voltage = buffer_get_float16(data, 1e1, 4)
        else:
            return None, None

        self.can_packet = pack
        return id_, pack
