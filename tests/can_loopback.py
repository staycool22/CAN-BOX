import os
import sys
import time
import argparse
import threading
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
    
    # 禁用接口以应用更改
    os.system(f"sudo ip link set {interface} down")
    
    # 根据是否启用 FD 配置不同的命令
    if fd:
        if dbitrate is None:
            raise ValueError("CAN-FD mode requires a data bitrate (dbitrate).")
        cmd = (
            f"sudo ip link set {interface} type can "
            f"bitrate {bitrate} dbitrate {dbitrate} fd on"
        )
    else:
        cmd = f"sudo ip link set {interface} type can bitrate {bitrate}"
        
    print(f"Executing: {cmd}")
    os.system(cmd)
    
    # 重新启用接口
    os.system(f"sudo ip link set {interface} up")
    print(f"{interface} configured and up.")

def can_sender(
    interface: str,
    arbitration_id: int,
    data: bytes,
    is_fd: bool,
    frequency: float,
    count: int,
):
    """在指定的 CAN 接口上以固定频率发送报文。"""
    try:
        bus = can.interface.Bus(channel=interface, interface='socketcan', fd=is_fd)
    except OSError as e:
        print(f"Error opening bus on {interface}: {e}")
        print("Please ensure the interface is correctly configured and up.")
        return

    print(f"Sender started on {interface}. Sending {count} messages at {frequency} Hz.")

    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False,
        is_fd=is_fd,
        bitrate_switch=is_fd,  # Enable BRS in FD mode
    )

    period = 1.0 / frequency

    # 使用 send_periodic 以更精确的频率发送
    task = bus.send_periodic(message, period)

    # 等待足够的时间以发送所有报文
    time.sleep(count * period)

    task.stop()

    bus.shutdown()
    print(f"Sender on {interface} finished.")

def can_receiver(interface: str, is_fd: bool, expected_messages: int):
    """在指定的 CAN 接口上接收并打印报文。"""
    try:
        bus = can.interface.Bus(channel=interface, interface='socketcan', fd=is_fd)
    except OSError as e:
        print(f"Error opening bus on {interface}: {e}")
        return

    print(f"Receiver started on {interface}. Waiting for {expected_messages} messages.")
    
    start_time = None
    received_count = 0
    while received_count < expected_messages:
        msg = bus.recv(timeout=0.5)  # 5秒超时
        if msg:
            received_count += 1
            if start_time is None:
                start_time = msg.timestamp
            relative_time = msg.timestamp - start_time
            print(f"[{received_count}] Received on {interface} at +{relative_time:.6f}s: ID={hex(msg.arbitration_id)}, Data={msg.data.hex()}")
        else:
            print(f"Receiver on {interface} timed out.")
            break
            
    bus.shutdown()
    print(f"Receiver on {interface} finished after receiving {received_count} messages.")

def main():
    parser = argparse.ArgumentParser(
        description="CAN/CAN-FD loopback test using socketcan.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    
    parser.add_argument(
        "--bitrate", type=int, default=500000,
        help="CAN arbitration phase bitrate (e.g., 500000 for 500k)."
    )
    parser.add_argument(
        "--dbitrate", type=int,
        help="CAN-FD data phase bitrate (required for --fd)."
    )
    parser.add_argument(
        "--fd", action="store_true",
        help="Enable CAN-FD mode."
    )
    parser.add_argument(
        "--freq", type=float, default=10.0,
        help="Sending frequency in Hz."
    )
    parser.add_argument(
        "--count", type=int, default=10,
        help="Number of messages to send."
    )
    parser.add_argument(
        "--data-len", type=int, default=8,
        help="Length of data to send (1-64 bytes). Only used if --data is not provided."
    )
    
    args = parser.parse_args()

    # 检查平台
    if not sys.platform.startswith('linux'):
        print("This script is designed for Linux with socketcan.")
        sys.exit(1)

    # 检查 FD 模式的参数
    if args.fd and not args.dbitrate:
        print("Error: --dbitrate is required when --fd is enabled.")
        sys.exit(1)

    # 配置 CAN 接口
    try:
        configure_can_interface("can0", args.bitrate, args.dbitrate, args.fd)
        configure_can_interface("can1", args.bitrate, args.dbitrate, args.fd)
    except Exception as e:
        print(f"Failed to configure CAN interfaces: {e}")
        sys.exit(1)

    # 定义报文内容
    arbitration_id = 0x1F334455
    if args.fd:
        # CAN-FD 支持更长的数据
        data = bytes(range(min(args.data_len, 64)))
    else:
        data = bytes.fromhex("1122334455667788")

    # 创建并启动收发线程
    # can1 发送, can0 接收
    sender_thread = threading.Thread(
        target=can_sender,
        args=("can1", arbitration_id, data, args.fd, args.freq, args.count),
    )
    receiver_thread = threading.Thread(
        target=can_receiver,
        args=("can0", args.fd, args.count),
    )
    
    print("\nStarting loopback test: can1 -> can0")
    receiver_thread.start()
    time.sleep(0.1)  # 确保接收方先启动
    sender_thread.start()
    
    # 等待线程结束
    sender_thread.join()
    receiver_thread.join()
    
    print("\nLoopback test finished.")

if __name__ == "__main__":
    main()