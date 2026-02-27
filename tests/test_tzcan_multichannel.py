"""多通道 CAN 测试（socketcan）：同时监听多个通道，定时在指定通道发送

适用后端：socketcan（Linux/WSL）
每个通道拥有独立的 can.Bus（独立 socket），接收线程之间互不干扰，
直接在每个线程里调用 bus.recv() 即可，无需 dispatcher 或队列。

用法：
  python3 tests/test_tzcan_multichannel.py \\
      --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k --freq 10

  python3 tests/test_tzcan_multichannel.py \\
      --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k --freq 10 --duration 30
"""
import argparse
import sys
import os
import threading
import time
from collections import defaultdict

try:
    from tzcan import CANMessageTransmitter
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from tzcan import CANMessageTransmitter


def parse_bitrate_token(s):
    s = s.lower().strip()
    if s.endswith('k'): return int(float(s[:-1]) * 1_000)
    if s.endswith('m'): return int(float(s[:-1]) * 1_000_000)
    return int(s)


def _recv_worker(txer, ch, rx_counters, stop_event):
    """单通道接收线程。

    socketcan 每个通道是独立的 can.Bus（独立 socket），
    直接调用 bus.recv() 即可，不会抢占其他通道的消息。
    """
    t0 = time.time()
    while not stop_event.is_set():
        msg = txer.bus.recv(timeout=0.1)
        if msg is None:
            continue
        rx_counters[ch] += 1
        elapsed = time.time() - t0
        fd_tag = "FD " if msg.is_fd else ""
        print(f"[{elapsed:7.2f}s] RX ch{ch} | "
              f"ID=0x{msg.arbitration_id:08X} "
              f"DLC={len(msg.data):2d} {fd_tag}"
              f"data={msg.data.hex()}")


def _send_worker(txers_tx, tx_channels, send_id, freq, tx_counters, stop_event):
    """定时发送线程，数据包含序号和通道号便于调试。"""
    interval = 1.0 / freq
    seq = 0
    while not stop_event.is_set():
        t0 = time.time()
        for ch in tx_channels:
            if ch not in txers_tx:
                continue
            data = [
                (seq >> 24) & 0xFF, (seq >> 16) & 0xFF,
                (seq >> 8) & 0xFF,  seq & 0xFF,
                ch & 0xFF, 0, 0, 0,
            ]
            if txers_tx[ch]._send_can_data(send_id=send_id, data_list=data):
                tx_counters[ch] += 1
        seq = (seq + 1) & 0xFFFFFFFF
        time.sleep(max(0.0, interval - (time.time() - t0)))


def main():
    parser = argparse.ArgumentParser(description="多通道 CAN 收发测试（socketcan）")
    parser.add_argument('--channels',    type=int, nargs='+', default=[0, 1, 2, 3],
                        help='要同时打开并监听的通道列表，默认 0 1 2 3')
    parser.add_argument('--tx-channels', type=int, nargs='+', default=[0, 1],
                        help='定时发送的通道子集（必须在 --channels 中）')
    parser.add_argument('--can-br',      default='500k',    help='CAN 波特率，默认 500k')
    parser.add_argument('--send-id',     type=lambda x: int(x, 0), default=0x123,
                        help='发送帧 ID（支持 0x 前缀），默认 0x123')
    parser.add_argument('--freq',        type=float, default=10.0,  help='发送频率 Hz，默认 10')
    parser.add_argument('--duration',    type=float, default=0.0,
                        help='运行时长 s，0=持续直到 Ctrl+C')
    args = parser.parse_args()

    baud = parse_bitrate_token(args.can_br)

    bad = set(args.tx_channels) - set(args.channels)
    if bad:
        print(f"❌ --tx-channels 包含不在 --channels 中的通道: {sorted(bad)}")
        sys.exit(1)

    # ── 1. 一次调用打开所有通道 ──────────────────────────────────────────────
    print(f"正在初始化通道 {args.channels}（backend=socketcan，baud={baud}）...")
    TX, m_dev, _, _ = CANMessageTransmitter.open(
        "TZUSB2CAN", baud_rate=baud, channels=args.channels, backend="socketcan"
    )

    # ── 2. 按通道号从 m_dev["buses"] 取各自独立的 Bus ───────────────────────
    #
    #   socketcan：每个通道一个独立的 can.Bus（独立 socket）
    #   不设 channel_id，各 Bus 已绑定到具体的 canX 接口
    #
    txers: dict[int, object] = {}
    for ch in args.channels:
        bus = m_dev["buses"].get(ch)
        if bus is None:
            print(f"⚠️  通道 {ch} 未能成功打开，跳过")
            continue
        txers[ch] = TX(bus)

    if not txers:
        print("❌ 没有成功打开任何通道，退出")
        TX.close_can_device(m_dev)
        sys.exit(1)

    print(f"已打开通道: {sorted(txers.keys())}")
    print(f"发送通道  : {args.tx_channels}，频率 {args.freq} Hz，ID=0x{args.send_id:X}")
    if args.duration > 0:
        print(f"运行时长  : {args.duration} s")
    else:
        print("持续运行，按 Ctrl+C 退出")

    stop_event = threading.Event()
    rx_counters: dict[int, int] = defaultdict(int)
    tx_counters: dict[int, int] = defaultdict(int)
    threads = []

    # ── 3. 每通道一个接收线程，直接 recv ────────────────────────────────────
    #
    #   socketcan 独立 socket：各线程调用各自 Bus.recv()，互不干扰，无竞争。
    #
    for ch, txer in txers.items():
        t = threading.Thread(
            target=_recv_worker,
            args=(txer, ch, rx_counters, stop_event),
            daemon=True,
            name=f"recv-ch{ch}",
        )
        t.start()
        threads.append(t)

    # ── 4. 发送线程 ──────────────────────────────────────────────────────────
    txers_tx = {ch: txers[ch] for ch in args.tx_channels if ch in txers}
    if txers_tx:
        t = threading.Thread(
            target=_send_worker,
            args=(txers_tx, args.tx_channels, args.send_id,
                  args.freq, tx_counters, stop_event),
            daemon=True,
            name="sender",
        )
        t.start()
        threads.append(t)
    else:
        print("⚠️  发送通道全部未成功打开，跳过发送")

    # ── 5. 主循环：定时打印统计 ──────────────────────────────────────────────
    t_start = time.time()
    try:
        while True:
            elapsed = time.time() - t_start
            if args.duration > 0 and elapsed >= args.duration:
                break
            time.sleep(1.0)
            rx_str = "  ".join(f"ch{ch}:{rx_counters[ch]}" for ch in sorted(txers.keys()))
            tx_str = "  ".join(f"ch{ch}:{tx_counters[ch]}" for ch in sorted(txers_tx.keys()))
            print(f"── [{elapsed:5.0f}s] RX [{rx_str}]  TX [{tx_str}]")
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=1.0)
        TX.close_can_device(m_dev)

    total = time.time() - t_start
    print(f"\n── 最终统计（{total:.1f}s）──")
    for ch in sorted(txers.keys()):
        tx_info = f"  TX {tx_counters[ch]:5d} ({tx_counters[ch]/total:4.1f} fps)" \
                  if ch in txers_tx else ""
        print(f"  ch{ch}: RX {rx_counters[ch]:5d} ({rx_counters[ch]/total:4.1f} fps){tx_info}")


if __name__ == '__main__':
    main()
