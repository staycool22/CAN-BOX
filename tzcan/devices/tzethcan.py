import socket
import struct
import time
import can
from typing import List, Optional

from .base import CANMessageTransmitter

# Cannelloni Protocol Constants for Config
class ETHCANConstants:
    CANNELLONI_VERSION = 2
    CANNELLONI_OP_CONFIG = 1
    TARGET_IP = "192.168.1.10"
    TARGET_PORT_BASE = 20000
    PROTOCOL = "UDP" # Options: "UDP", "TCP"

@CANMessageTransmitter.register("TZETHCAN")
class TZETHCANTransmitter(CANMessageTransmitter):
    """
    Hybrid Implementation:
    1. Control Plane: Sends raw UDP/TCP OpCode 1 packets to HPM for baud rate config.
    2. Data Plane: Uses standard python-can (SocketCAN) via vcan interfaces bridged by cannelloni.
    """

    def __init__(self, channel_handle: can.BusABC, channel_id: int = 0, is_canfd: bool = False):
        super().__init__(channel_handle)
        self.bus = channel_handle
        self.channel_id = channel_id
        self.is_canfd = is_canfd

    def _send_can_data(self, send_id, data_list, is_ext_frame=False, canfd_mode=False, brs=0, esi=0):
        # Delegate to python-can (same as TZUSB2CANTransmitter)
        try:
            # Enforce CAN 2.0 mode if configured as such
            if not self.is_canfd:
                canfd_mode = False
                brs = 0

            msg = can.Message(
                arbitration_id=send_id,
                data=bytes(bytearray(data_list)),
                is_extended_id=bool(is_ext_frame),
                is_fd=bool(canfd_mode),
                bitrate_switch=bool(brs),
                error_state_indicator=bool(esi),
                check=True
            )
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"Tx Error: {e}")
            return False
        except ValueError as e:
            print(f"Value Error: {e}")
            return False

    def _receive_can_data(self, target_id=None, timeout=5, is_ext_frame=None, canfd_mode=False, return_msg=False):
        # Delegate to python-can
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg:
                result = (
                    msg.arbitration_id,
                    list(msg.data),
                    msg.is_extended_id,
                    msg.is_fd,
                    msg.bitrate_switch,
                    msg.error_state_indicator
                )
                if return_msg:
                    return result + (msg,)
                return result
            return None
        except can.CanError:
            return None

    @staticmethod
    def _send_config(ch_index, baud_rate, dbit_baud_rate, target_ip=None):
        """Sends the custom Cannelloni Config packet to HPM board via UDP or TCP"""
        ip = target_ip if target_ip is not None else ETHCANConstants.TARGET_IP
        try:
            payload = bytearray()
            payload.extend(struct.pack('!BBBH', ETHCANConstants.CANNELLONI_VERSION, ETHCANConstants.CANNELLONI_OP_CONFIG, 0, 0))
            payload.extend(struct.pack('!II', int(baud_rate), int(dbit_baud_rate)))
            target_port = ETHCANConstants.TARGET_PORT_BASE + ch_index

            if ETHCANConstants.PROTOCOL == "UDP":
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(1.0)
                sock.sendto(payload, (ip, target_port))
                sock.close()
                print(f"Sent UDP BaudConfig to {ip}:{target_port} (Nom={baud_rate}, Data={dbit_baud_rate})")

            elif ETHCANConstants.PROTOCOL == "TCP":
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                try:
                    sock.connect((ip, target_port))
                    sock.sendall(payload)
                    print(f"Sent TCP BaudConfig to {ip}:{target_port} (Nom={baud_rate}, Data={dbit_baud_rate})")
                finally:
                    sock.close()

            else:
                print(f"Unknown Protocol: {ETHCANConstants.PROTOCOL}")

        except Exception as e:
            print(f"Failed to send baud config ({ETHCANConstants.PROTOCOL}): {e}")

    @staticmethod
    def init_can_device(baud_rate=500000, dbit_baud_rate=2000000, channels=None,
                        fd=False, can_type=0, canfd_standard=0, channel_count=None,
                        target_ip=None):
        """
        1. Sends Config packets to HPM board (Protocol defined in ETHCANConstants).
        2. Initializes python-can SocketCAN interfaces (vcan0, vcan1...)

        Args:
            fd: True for CAN-FD, False for CAN 2.0 (can_type=1 also accepted for compat)
        """
        if channels is None:
            channels = [0]

        buses = {}
        is_canfd = fd or (can_type == 1)

        if not is_canfd:
            dbit_baud_rate = baud_rate

        # 阶段 1：向所有通道发送波特率配置包
        for ch in channels:
            # 1. Send Config Packet
            TZETHCANTransmitter._send_config(ch, baud_rate, dbit_baud_rate, target_ip=target_ip)

        # 等待 HPM 完成波特率切换，再打开 vcan 接口
        if channels:
            time.sleep(0.1)

        # 阶段 2：打开对应 vcan 接口
        for ch in channels:
            interface_name = f"vcan{ch}"
            try:
                bus = can.Bus(interface='socketcan', channel=interface_name, fd=True)
                buses[ch] = bus  # 存原始 can.Bus，供 TX(m_dev["buses"][ch]) 使用
            except Exception as e:
                print(f"Failed to open {interface_name}: {e}")
                raise ImportError(f"Could not open {interface_name}. Ensure 'setup_cannelloni.sh' is running.")

        m_dev = {
            "backend": "tzethcan",
            "buses": buses,
            "config": {
                "baud_rate": baud_rate,
                "dbit_baud_rate": dbit_baud_rate,
                "channels": channels,
                "fd": is_canfd,
            },
        }
        ch0 = buses.get(channels[0]) if channels else None
        ch1 = buses.get(channels[1]) if len(channels) > 1 else None
        return m_dev, ch0, ch1

    @staticmethod
    def close_can_device(m_dev, channel_handle0=None, channel_handle1=None):
        if isinstance(m_dev, dict) and "buses" in m_dev:
            seen = set()
            for bus in m_dev["buses"].values():
                if id(bus) not in seen:
                    seen.add(id(bus))
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
        else:
            for handle in (channel_handle0, channel_handle1):
                if handle is not None:
                    try:
                        handle.shutdown()
                    except Exception:
                        pass
