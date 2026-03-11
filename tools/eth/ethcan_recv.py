# 说明：TZETHCANTransmitter 接收测试脚本，通过 cannelloni/vcan 接收 CAN/CAN FD 报文
# 用途：初始化 ETH-CAN 通道（自动通过 UDP 配置 HPM 硬件波特率），接收并统计报文
# 注意：需先运行 setup_cannelloni.sh，确保 vcan 接口已建立；仅适用于 Linux/WSL
import argparse
import time
import os
import sys
import select
import termios
import tty
import threading
from typing import Optional, List

try:
    import can
except ImportError:
    can = None

try:
    from tzcan import CANMessageTransmitter
except ImportError:
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
    from tzcan import CANMessageTransmitter


def ensure_python_can():
    if can is None:
        raise RuntimeError("python-can 未安装。请先运行: pip install python-can")


def parse_bitrate_token(token: str) -> int:
    s = str(token).strip().lower()
    try:
        if s.isdigit():
            return int(s)
        if s.endswith('k'):
            return int(float(s[:-1]) * 1000)
        if s.endswith('m'):
            return int(float(s[:-1]) * 1000000)
    except Exception:
        pass
    raise ValueError(f"无法解析速率值: {token}")


class ReceiverThread(threading.Thread):
    """单通道接收线程，直接操作 raw can.Bus（vcan 接口）。"""

    def __init__(
        self,
        ch: int,
        bus: can.BusABC,
        duration_s: float,
        max_count: Optional[int],
        filter_id: Optional[int],
        fd_only: bool,
        can_only: bool,
        print_each: bool,
        time_mode: str,
        stop_event: threading.Event,
    ):
        super().__init__(daemon=True)
        self.ch = ch
        self.bus = bus
        self.duration_s = duration_s
        self.max_count = max_count
        self.filter_id = filter_id
        self.fd_only = fd_only
        self.can_only = can_only
        self.print_each = print_each
        self.time_mode = time_mode
        self.stop_event = stop_event
        self.stats = {'total': 0, 'can': 0, 'fd': 0}
        self.first_msg_ts: Optional[float] = None

    def run(self):
        deadline = (time.perf_counter() + self.duration_s) if self.duration_s > 0 else None
        while not self.stop_event.is_set():
            if deadline and time.perf_counter() >= deadline:
                break
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            if not self._accept(msg):
                continue
            self.stats['total'] += 1
            if msg.is_fd:
                self.stats['fd'] += 1
            else:
                self.stats['can'] += 1
            if self.print_each:
                self._print_msg(msg)
            if self.max_count and self.stats['total'] >= self.max_count:
                break
        print(f"[CH{self.ch}/vcan{self.ch}] 总计接收: {self.stats['total']} 条 "
              f"(CAN: {self.stats['can']}, FD: {self.stats['fd']})")

    def _accept(self, msg: can.Message) -> bool:
        if self.filter_id is not None and msg.arbitration_id != self.filter_id:
            return False
        if self.fd_only and not msg.is_fd:
            return False
        if self.can_only and msg.is_fd:
            return False
        return True

    def _print_msg(self, msg: can.Message):
        ts = msg.timestamp
        if self.first_msg_ts is None:
            self.first_msg_ts = ts

        if self.time_mode == 'abs':
            ms = int((ts - int(ts)) * 1000)
            t_str = time.strftime('%H:%M:%S', time.localtime(ts)) + f'.{ms:03d}'
        else:
            t_str = f"{ts - self.first_msg_ts:.6f}s"

        id_hex = (
            f"0x{msg.arbitration_id:08X}"
            if msg.is_extended_id
            else f"0x{msg.arbitration_id:03X}"
        )
        data_hex = ' '.join(f"{b:02X}" for b in msg.data)
        type_str = 'FD ' if msg.is_fd else 'CAN'
        print(f"RX {t_str} CH{self.ch} [{type_str}] ID={id_hex} LEN={len(msg.data)} DATA={data_hex}")


def main():
    parser = argparse.ArgumentParser(
        description="通过 cannelloni/vcan 接收 CAN/CAN FD 报文（TZETHCANTransmitter）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python ethcan_recv.py --channels 0                               # CAN 2.0，vcan0，500kbps
  python ethcan_recv.py --channels 0 1 --baud-rate 1m              # 双通道
  python ethcan_recv.py --channels 0 --mode fd --dbit-baud-rate 5m # CAN FD
  python ethcan_recv.py --channels 0 --mode all --dbit-baud-rate 2m # FD 总线，接收全部帧
  python ethcan_recv.py --channels 0 --filter-id 0x123 --duration 10
前置条件: 运行 ./setup_cannelloni.sh 建立 vcan 与 cannelloni 转发
        """,
    )
    parser.add_argument("--channels", nargs='+', type=int, default=[0],help="vcan 通道索引列表，例如: 0 1 2（默认: 0）")
    parser.add_argument("--baud-rate", default="500k",help="仲裁域波特率，例如 500k、1m（默认: 500k）")
    parser.add_argument("--dbit-baud-rate", default="2m",help="FD 数据域波特率，例如 2m、5m（默认: 2m，mode=fd/all 时生效）")
    parser.add_argument("--mode", choices=["all", "can", "fd"], default="all",help="接收模式：all=FD总线接收全部, can=仅 CAN 2.0, fd=仅 CAN FD（默认: all）")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="接收时长（秒）；0 表示持续接收直至按 ESC 退出（默认: 0）")
    parser.add_argument("--count", type=int, default=0,
                        help="达到该帧数后自动停止（默认: 0=不限）")
    parser.add_argument("--filter-id", type=lambda x: int(x, 0),
                        help="仅接收指定 ID，例如 0x123")
    parser.add_argument("--time-mode", choices=["rel", "abs"], default="rel",
                        help="时间戳格式：rel=相对首帧, abs=绝对时间（默认: rel）")
    parser.add_argument("--quiet", action="store_true",
                        help="不逐条打印消息，仅输出最终统计")
    args = parser.parse_args()

    ensure_python_can()

    baud_rate = parse_bitrate_token(args.baud_rate)
    dbit_baud_rate = parse_bitrate_token(args.dbit_baud_rate)
    # mode=can 时以 CAN 2.0 初始化；fd 或 all 时以 FD 初始化（FD 总线可同时接收 CAN 2.0）
    is_fd = args.mode != "can"

    fd_info = f", dbit_baud_rate={dbit_baud_rate}" if is_fd else ""
    print(f"初始化 TZETHCANTransmitter: channels={args.channels}, baud_rate={baud_rate}{fd_info}, fd={is_fd}")
    print("（将通过 UDP 向 HPM 硬件发送波特率配置包）")

    TX, m_dev, _, _ = CANMessageTransmitter.open(
        "TZETHCAN",
        baud_rate=baud_rate,
        dbit_baud_rate=dbit_baud_rate,
        channels=args.channels,
        fd=is_fd,
    )

    # 共享停止事件，ESC 可同时停止所有通道
    stop_event = threading.Event()
    receivers: List[ReceiverThread] = []

    for ch in args.channels:
        raw_bus = m_dev["buses"].get(ch)
        if raw_bus is None:
            print(f"❌ 通道 {ch} (vcan{ch}) 初始化失败，跳过")
            continue
        label = "CAN FD" if is_fd else "CAN 2.0"
        print(f"✅ 已打开 vcan{ch} ({label})")
        r = ReceiverThread(
            ch=ch,
            bus=raw_bus,
            duration_s=args.duration,
            max_count=args.count if args.count > 0 else None,
            filter_id=args.filter_id,
            fd_only=(args.mode == "fd"),
            can_only=(args.mode == "can"),
            print_each=not args.quiet,
            time_mode=args.time_mode,
            stop_event=stop_event,
        )
        receivers.append(r)

    if not receivers:
        print("没有可用的接收通道，退出。")
        TX.close_can_device(m_dev)
        return

    print(f"\n开始接收（共 {len(receivers)} 个通道）... 按 ESC 退出。\n")
    for r in receivers:
        r.start()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while any(r.is_alive() for r in receivers):
            if select.select([sys.stdin], [], [], 0.1)[0]:
                if sys.stdin.read(1) == '\x1b':
                    print("\nESC 按下，正在停止...")
                    stop_event.set()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    for r in receivers:
        r.join()

    TX.close_can_device(m_dev)
    print("接收结束。")


if __name__ == "__main__":
    main()
