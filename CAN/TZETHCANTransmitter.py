import socket
import struct
import time
import can
from typing import List, Optional, Tuple, Any

try:
    from .CANMessageTransmitter import CANMessageTransmitter
except ImportError:
    try:
        from CANMessageTransmitter import CANMessageTransmitter
    except ImportError:
        class CANMessageTransmitter:
            def __init__(self, channel_handle): pass

# Cannelloni Protocol Constants for Config
class ETHCANConstants:
    CANNELLONI_VERSION = 2
    CANNELLONI_OP_CONFIG = 1
    TARGET_IP = "192.168.1.10"
    TARGET_PORT_BASE = 20000
    PROTOCOL = "UDP" # Options: "UDP", "TCP"

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
        # Delegate to python-can (same as TZCANTransmitter)
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

    def _receive_can_data(self, target_id=None, timeout=5, is_ext_frame=None, canfd_mode=False):
        # Delegate to python-can
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg:
                # Return format: (id, data, is_ext, is_fd, brs, esi) ??
                # The base class doc is vague, but usually we return the message object or tuple.
                # Let's match the previous TZETHCANTransmitter behavior (tuple).
                # (can_id, data_list, is_ext, is_fd, brs, esi)
                return (
                    msg.arbitration_id,
                    list(msg.data),
                    msg.is_extended_id,
                    msg.is_fd,
                    msg.bitrate_switch,
                    msg.error_state_indicator
                )
            return None
        except can.CanError:
            return None

    @staticmethod
    def _send_config(ch_index, baud_rate, dbit_baud_rate):
        """Sends the custom Cannelloni Config packet to HPM board via UDP or TCP"""
        try:
            payload = bytearray()
            payload.extend(struct.pack('!BBBH', ETHCANConstants.CANNELLONI_VERSION, ETHCANConstants.CANNELLONI_OP_CONFIG, 0, 0))
            payload.extend(struct.pack('!II', int(baud_rate), int(dbit_baud_rate)))
            target_port = ETHCANConstants.TARGET_PORT_BASE + ch_index

            if ETHCANConstants.PROTOCOL == "UDP":
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(1.0)            
                sock.sendto(payload, (ETHCANConstants.TARGET_IP, target_port))
                sock.close()
                print(f"Sent UDP BaudConfig to {ETHCANConstants.TARGET_IP}:{target_port} (Nom={baud_rate}, Data={dbit_baud_rate})")
            
            elif ETHCANConstants.PROTOCOL == "TCP":
                # TCP Implementation Placeholder
                # Note: TCP requires connection and might need a persistent socket or one-time connect
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                sock.connect((ETHCANConstants.TARGET_IP, target_port))
                sock.sendall(payload)
                sock.close()
                print(f"Sent TCP BaudConfig to {ETHCANConstants.TARGET_IP}:{target_port} (Nom={baud_rate}, Data={dbit_baud_rate})")
            
            else:
                print(f"Unknown Protocol: {ETHCANConstants.PROTOCOL}")

        except Exception as e:
            print(f"Failed to send baud config ({ETHCANConstants.PROTOCOL}): {e}")

    @staticmethod
    def init_can_device(baud_rate=500000, dbit_baud_rate=2000000, channels=[0], can_type=0, canfd_standard=0, channel_count=None):
        """
        1. Sends Config packets to HPM board (Protocol defined in ETHCANConstants).
        2. Initializes python-can SocketCAN interfaces (vcan0, vcan1...)
        
        Args:
            can_type: 0 for CAN 2.0, 1 for CAN-FD
        """
        
        dev_instances = []
        is_canfd = (can_type == 1)
        
        # Adjust config for CAN 2.0 to avoid confusion in firmware logs (optional)
        if not is_canfd:
            dbit_baud_rate = baud_rate

        # We don't really have a single "m_dev" object for SocketCAN, 
        # usually we return a dummy or the first bus.
        # But consistent with previous pattern: (m_dev, ch0, ch1...)
        
        for ch in channels:
            # 1. Send Config Packet
            TZETHCANTransmitter._send_config(ch, baud_rate, dbit_baud_rate)
            
            # 2. Open SocketCAN interface
            # Assuming vcan0, vcan1, etc.
            interface_name = f"vcan{ch}"
            try:
                # Note: 'fd=True' is important for CAN-FD support in python-can
                # We enable it at socket level even for CAN 2.0 to support mixed usage if needed,
                # but valid flags are enforced by _send_can_data
                bus = can.Bus(interface='socketcan', channel=interface_name, fd=True)
                dev_instances.append(TZETHCANTransmitter(bus, ch, is_canfd=is_canfd))
            except Exception as e:
                print(f"Failed to open {interface_name}: {e}")
                # Should we raise or continue?
                raise ImportError(f"Could not open {interface_name}. Ensure 'setup_cannelloni.sh' is running.")

        # Return format: (m_dev, ch0, ch1...)
        # We'll use the first bus as the "device handle" if needed, or just None
        return (None,) + tuple(dev_instances)

    @staticmethod
    def close_can_device(m_dev, channel_handle0=None, channel_handle1=None):
        if channel_handle0:
            channel_handle0.bus.shutdown()
        if channel_handle1:
            channel_handle1.bus.shutdown()
