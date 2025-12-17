import os
import sys
import time
import argparse
from typing import Optional

import can

def configure_can_interface(
    interface: str,
    bitrate: int,
    dbitrate: Optional[int] = None,
    fd: bool = False,
):
    """使用 ip link 命令配置 CAN 接口。"""
    print(f"Configuring {interface}...")
    os.system(f"sudo ip link set {interface} down")
    
    if fd:
        if dbitrate is None:
            raise ValueError("CAN-FD mode requires a data bitrate (dbitrate).")
        cmd = f"sudo ip link set {interface} type can bitrate {bitrate} dbitrate {dbitrate} fd on"
    else:
        cmd = f"sudo ip link set {interface} type can bitrate {bitrate}"
        
    print(f"Executing: {cmd}")
    os.system(cmd)
    os.system(f"sudo ip link set {interface} up")
    print(f"{interface} configured and up.")

def send_messages(
    interface: str,
    arbitration_id: int,
    data: bytes,
    is_fd: bool,
    frequency: float,
    count: int,
):
    """在指定接口上发送报文。"""
    try:
        bus = can.interface.Bus(channel=interface, interface='socketcan', fd=is_fd)
    except OSError as e:
        print(f"Error opening bus on {interface}: {e}")
        return

    print(f"Sender started on {interface}. Sending {count} messages at {frequency} Hz.")
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False,
        is_fd=is_fd,
        bitrate_switch=is_fd,
    )
    
    interval = 1.0 / frequency if frequency > 0 else 0
    for i in range(count):
        try:
            bus.send(message)
            if interval > 0:
                time.sleep(interval)
        except can.CanError as e:
            print(f"Error sending message: {e}")
            break
    
    bus.shutdown()
    print("Sender finished.")

def receive_messages(interface: str, is_fd: bool, timeout: float):
    """在指定接口上接收报文。"""
    try:
        bus = can.interface.Bus(channel=interface, interface='socketcan', fd=is_fd)
    except OSError as e:
        print(f"Error opening bus on {interface}: {e}")
        return

    print(f"Receiver started on {interface}. Press Ctrl+C to stop.")
    count = 0
    start_time = None
    try:
        for msg in bus:
            count += 1
            if start_time is None:
                start_time = msg.timestamp
            relative_time = msg.timestamp - start_time
            print(f"[{count}] Received at +{relative_time:.6f}s: ID={hex(msg.arbitration_id)}, Data={msg.data.hex()}")
    except KeyboardInterrupt:
        print("\nReceiver stopped by user.")
    finally:
        bus.shutdown()
        print(f"Receiver finished. Total messages received: {count}")

def main():
    parser = argparse.ArgumentParser(
        description="Independent CAN/CAN-FD sender/receiver for socketcan.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    
    # 通用参数
    parser.add_argument("interface", type=str, choices=['can0', 'can1'], help="CAN interface to use.")
    parser.add_argument("mode", type=str, choices=['send', 'receive'], help="Operating mode.")
    parser.add_argument("--bitrate", type=int, default=500000, help="Arbitration phase bitrate.")
    parser.add_argument("--dbitrate", type=int, help="Data phase bitrate for CAN-FD.")
    parser.add_argument("--fd", action="store_true", help="Enable CAN-FD mode.")

    # 发送模式参数
    parser.add_argument("--id", type=str, default="0x123", help="Arbitration ID for sending.")
    parser.add_argument("--data", type=str, help="Data to send (hex string). Defaults to 64 bytes for FD.")
    parser.add_argument("--freq", type=float, default=1.0, help="Sending frequency in Hz.")
    parser.add_argument("--count", type=int, default=10, help="Number of messages to send.")
    parser.add_argument("--data-len", type=int, help="Length of data to send (1-64 bytes). Used if --data is not given.")

    args = parser.parse_args()

    if not sys.platform.startswith('linux'):
        print("This script is for Linux with socketcan.")
        sys.exit(1)

    if args.fd and not args.dbitrate:
        parser.error("--dbitrate is required for --fd mode.")

    try:
        configure_can_interface(args.interface, args.bitrate, args.dbitrate, args.fd)
    except Exception as e:
        print(f"Failed to configure {args.interface}: {e}")
        sys.exit(1)

    if args.mode == 'send':
        arbitration_id = int(args.id, 16)
        
        # 如果是 FD 模式且用户未提供数据，则根据 data-len 或默认64字节生成数据
        if args.fd and args.data is None:
            data_len = args.data_len if args.data_len is not None else 64
            data = bytes(range(min(data_len, 64)))
        elif args.data is not None:
            data = bytes.fromhex(args.data)
        else:
            # 默认的标准CAN数据，可由 data-len覆盖
            data_len = args.data_len if args.data_len is not None else 8
            data = bytes(range(min(data_len, 8)))

        send_messages(args.interface, arbitration_id, data, args.fd, args.freq, args.count)
    elif args.mode == 'receive':
        receive_messages(args.interface, args.fd, timeout=10.0)

if __name__ == "__main__":
    main()