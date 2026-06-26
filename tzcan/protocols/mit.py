import time
from ctypes import Structure, c_int, c_float
from typing import Optional, Tuple

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

P_MIN, P_MAX   = -12.5, 12.5   # rad
V_MIN, V_MAX   = -30.0, 30.0   # rad/s
KP_MIN, KP_MAX =   0.0, 500.0
KD_MIN, KD_MAX =   0.0,   5.0
T_MIN, T_MAX   = -10.0,  10.0  # Nm

def _float_to_uint(x, x_min, x_max, bits) -> int:
    return int((np.clip(x, x_min, x_max) - x_min) / (x_max - x_min) * ((1 << bits) - 1))

def _uint_to_float(x, x_min, x_max, bits) -> float:
    return x / ((1 << bits) - 1) * (x_max - x_min) + x_min

class MITMotorPack(Structure):
    _fields_ = [
        ("motor_id", c_int),
        ("position", c_float),
        ("velocity", c_float),
        ("torque", c_float),
        ("temperature", c_float),
        ("error_code", c_int),
    ]

class MIT_CAN(CANProtocolBase):   # from tzcan.protocols.base
    CMD_ENABLE   = [0xFF]*7 + [0xFC]
    CMD_DISABLE  = [0xFF]*7 + [0xFD]
    CMD_SET_ZERO = [0xFF]*7 + [0xFE]

    def __init__(self, transmitter):
        super().__init__(transmitter)
        self.can_packet = MITMotorPack()
        self._pack_by_dev_id: dict = {}
        # per-motor 量程覆盖表，键为 motor_id（整数）
        # 每条只需包含要覆盖的字段，缺省回退到模块级全局量程
        self._motor_limits: dict = {}

    def set_motor_limits(self, motor_id: int, **limits) -> None:
        """为指定 motor_id 设置量程覆盖。
        可传入的关键字：p_min, p_max, v_min, v_max,
                        kp_min, kp_max, kd_min, kd_max, t_min, t_max
        """
        self._motor_limits[int(motor_id)] = {k: float(v) for k, v in limits.items()}

    def _limits(self, motor_id: int) -> dict:
        """返回指定 motor_id 的量程，缺失字段回退到全局默认值。"""
        ov = self._motor_limits.get(int(motor_id), {})
        return {
            "p_min":  ov.get("p_min",  P_MIN),
            "p_max":  ov.get("p_max",  P_MAX),
            "v_min":  ov.get("v_min",  V_MIN),
            "v_max":  ov.get("v_max",  V_MAX),
            "kp_min": ov.get("kp_min", KP_MIN),
            "kp_max": ov.get("kp_max", KP_MAX),
            "kd_min": ov.get("kd_min", KD_MIN),
            "kd_max": ov.get("kd_max", KD_MAX),
            "t_min":  ov.get("t_min",  T_MIN),
            "t_max":  ov.get("t_max",  T_MAX),
        }

    def send_control(self, motor_id, p_des, v_des, kp, kd, tau_ff) -> bool:
        lim = self._limits(motor_id)
        p_des_int = _float_to_uint(p_des, lim["p_min"], lim["p_max"], 16)
        v_des_int = _float_to_uint(v_des, lim["v_min"], lim["v_max"], 12)
        kp_int = _float_to_uint(kp, lim["kp_min"], lim["kp_max"], 12)
        kd_int = _float_to_uint(kd, lim["kd_min"], lim["kd_max"], 12)
        tau_ff_int = _float_to_uint(tau_ff, lim["t_min"], lim["t_max"], 12)

        data = [
            (p_des_int >> 8) & 0xFF,
            p_des_int & 0xFF,
            (v_des_int >> 4) & 0xFF,
            ((v_des_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0x0F) << 4) | ((tau_ff_int >> 8) & 0x0F),
            tau_ff_int & 0xFF,
        ]
        return self.send(motor_id, data, _is_ext_frame=False, _is_fd=False)

    def send_enable(self, motor_id) -> bool:
        # self.send(motor_id, CMD_ENABLE, _is_ext_frame=False, _is_fd=False)
        return self.send(motor_id, self.CMD_ENABLE, _is_ext_frame=False, _is_fd=False)

    def send_disable(self, motor_id) -> bool:
        return self.send(motor_id, self.CMD_DISABLE, _is_ext_frame=False, _is_fd=False)

    def send_set_zero(self, motor_id) -> bool:
        return self.send(motor_id, self.CMD_SET_ZERO, _is_ext_frame=False, _is_fd=False)
    
    def receive_decode(self, timeout=0) -> Tuple[Optional[int], Optional[MITMotorPack]]:
        id_, data = self.receive(_is_ext_frame=False, _is_fd=False, timeout=timeout)
        if id_ is None:
            return None, None
        dev_id = data[0] & 0x0F
        pack = self._pack_by_dev_id.get(dev_id)
        if pack is None:
            pack = MITMotorPack()
            self._pack_by_dev_id[dev_id] = pack
        pack.motor_id = dev_id
        lim = self._limits(dev_id)
        d0, d1, d2, d3, d4, d5, d6 = data[0], data[1], data[2], data[3], data[4], data[5], data[6]
        pos_u16 = (d1 << 8) | d2
        vel_u12 = (d3 << 4) | (d4 >> 4)
        torque_u12 = ((d4 & 0x0F) << 8) | d5
        pack.position = _uint_to_float(pos_u16, lim["p_min"], lim["p_max"], 16)
        pack.velocity = _uint_to_float(vel_u12, lim["v_min"], lim["v_max"], 12)
        pack.torque = _uint_to_float(torque_u12, lim["t_min"], lim["t_max"], 12)
        pack.temperature = _uint_to_float(d6 & 0xFF, 0, 100, 8)
        pack.error_code = (d0 >> 4) & 0x0F
        return id_, pack