"""多通道 CAN 测试（candle）：同时监听多个通道，定时在指定通道发送

适用后端：candle（Windows，需 python-can-candle）

candle 多通道设计：同一物理设备的所有通道共享同一个 can.Bus 对象。
若多个线程同时调用 bus.recv()，每条消息只会被其中一个线程取走，
导致通道间消息互相"偷走"。因此必须用单 dispatcher 线程读取后按
msg.channel 派发到各通道的独立队列，消费线程从队列取消息。

用法：
  python tests/test_tzcan_multichannel_candle.py \\
      --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k --freq 10

  python tests/test_tzcan_multichannel_candle.py \\
      --channels 0 1 --tx-channels 0 --can-br 500k --fd-dbr 2m --fd --freq 5
"""
import argparse
import queue
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


def _dispatch_worker(bus, chs, rx_queues, stop_event):
    """从共享 Bus 接收消息，按 msg.channel 派发到各通道队列。

    candle 驱动在 msg.channel 上写入整数型的物理通道索引。
    若无法匹配，消息归入 chs[0] 并打印一次警告。
    """
    ch_set = set(chs)
    unmatched_warned = False
    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.1)
        except Exception:
            break
        if msg is None:
            continue
        raw_ch = getattr(msg, 'channel', None)
        target = None
        if raw_ch is not None:
            try:
                candidate = int(raw_ch)
                if candidate in ch_set:
                    target = candidate
            except (TypeError, ValueError):
                pass
        if target is None:
            if not unmatched_warned:
                print(f"⚠️  dispatcher: msg.channel={raw_ch!r} 无法匹配 {chs}，"
                      f"归入 ch{chs[0]}")
                unmatched_warned = True
            target = chs[0]
        rx_queues[target].put_nowait(msg)


def _consume_worker(ch, q, rx_counters, stop_event):
    """从通道队列取消息并打印。"""
    t0 = time.time()
    while not stop_event.is_set():
        try:
            msg = q.get(timeout=0.1)
        except queue.Empty:
            continue
        rx_counters[ch] += 1
        elapsed = time.time() - t0
        fd_tag = "FD " if msg.is_fd else ""
        print(f"[{elapsed:7.2f}s] RX ch{ch} | "
              f"ID=0x{msg.arbitration_id:08X} "
              f"DLC={len(msg.data):2d} {fd_tag}"
              f"data={msg.data.hex()}")


def _send_worker(txers_tx, tx_channels, send_id, freq, fd, tx_counters, stop_event):
    """定时发送线程。"""
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
            if txers_tx[ch]._send_can_data(
                send_id=send_id, data_list=data,
                canfd_mode=fd, brs=1 if fd else 0,
            ):
                tx_counters[ch] += 1
        seq = (seq + 1) & 0xFFFFFFFF
        time.sleep(max(0.0, interval - (time.time() - t0)))


def main():
    parser = argparse.ArgumentParser(description="多通道 CAN 收发测试（candle）")
    parser.add_argument('--channels',    type=int, nargs='+', default=[0, 1, 2, 3],
                        help='要同时打开并监听的通道列表，默认 0 1 2 3')
    parser.add_argument('--tx-channels', type=int, nargs='+', default=[0, 1],
                        help='定时发送的通道子集（必须在 --channels 中）')
    parser.add_argument('--can-br',      default='500k',    help='CAN 仲裁域波特率，默认 500k')
    parser.add_argument('--fd-dbr',      default=None,      help='CAN FD 数据域波特率（启用 FD 时使用）')
    parser.add_argument('--fd',          action='store_true', help='启用 CAN FD 模式')
    parser.add_argument('--send-id',     type=lambda x: int(x, 0), default=0x123,
                        help='发送帧 ID（支持 0x 前缀），默认 0x123')
    parser.add_argument('--freq',        type=float, default=10.0,  help='发送频率 Hz，默认 10')
    parser.add_argument('--duration',    type=float, default=0.0,
                        help='运行时长 s，0=持续直到 Ctrl+C')
    args = parser.parse_args()

    baud = parse_bitrate_token(args.can_br)
    dbitrate = parse_bitrate_token(args.fd_dbr) if args.fd_dbr else baud

    bad = set(args.tx_channels) - set(args.channels)
    if bad:
        print(f"❌ --tx-channels 包含不在 --channels 中的通道: {sorted(bad)}")
        sys.exit(1)

    # ── 1. 一次调用打开所有通道（candle 模式，共享 Bus） ─────────────────────
    print(f"正在初始化通道 {args.channels}（backend=candle，baud={baud}）...")
    TX, m_dev, _, _ = CANMessageTransmitter.open(
        "TZUSB2CAN",
        baud_rate=baud,
        dbit_baud_rate=dbitrate,
        channels=args.channels,
        backend="candle",
        fd=args.fd,
    )

    # ── 2. 实例化各通道的 Transmitter，设置 channel_id ──────────────────────
    #
    #   candle：多通道共享同一个 Bus 对象。
    #   channel_id 有两个作用：
    #     发送：_send_can_data 将其写入 msg.channel，candle 驱动据此路由到正确物理通道
    #     接收：_receive_can_data 用其过滤（但本脚本用 dispatcher 接收，不依赖此过滤）
    #
    txers: dict[int, object] = {}
    for ch in args.channels:
        bus = m_dev["buses"].get(ch)
        if bus is None:
            print(f"⚠️  通道 {ch} 未能成功打开，跳过")
            continue
        txers[ch] = TX(bus, channel_id=ch)

    if not txers:
        print("❌ 没有成功打开任何通道，退出")
        TX.close_can_device(m_dev)
        sys.exit(1)

    print(f"已打开通道: {sorted(txers.keys())}")
    print(f"发送通道  : {args.tx_channels}，频率 {args.freq} Hz，ID=0x{args.send_id:X}")

    stop_event = threading.Event()
    rx_counters: dict[int, int] = defaultdict(int)
    tx_counters: dict[int, int] = defaultdict(int)
    threads = []

    # ── 3. 按唯一 Bus 对象分组（同一物理设备的通道共享同一 Bus） ────────────
    bus_groups: dict[int, list[int]] = defaultdict(list)
    for ch, txer in txers.items():
        bus_groups[id(txer.bus)].append(ch)

    # ── 4. 每通道一个接收队列 ────────────────────────────────────────────────
    rx_queues: dict[int, queue.Queue] = {ch: queue.Queue() for ch in txers}

    # ── 5. 每个唯一 Bus 启动一个 dispatcher 线程 ────────────────────────────
    #
    #   candle 共享 Bus：若多线程同时 recv()，每条消息只被一个线程取走。
    #   必须用单 dispatcher 线程统一 recv，再按 msg.channel 派发到各队列。
    #
    for chs in bus_groups.values():
        actual_bus = txers[chs[0]].bus
        t = threading.Thread(
            target=_dispatch_worker,
            args=(actual_bus, chs, rx_queues, stop_event),
            daemon=True,
            name=f"dispatch-{chs}",
        )
        t.start()
        threads.append(t)

    # ── 6. 消费线程（每通道一个，从队列读取并打印） ──────────────────────────
    for ch in txers:
        t = threading.Thread(
            target=_consume_worker,
            args=(ch, rx_queues[ch], rx_counters, stop_event),
            daemon=True,
            name=f"consume-ch{ch}",
        )
        t.start()
        threads.append(t)

    # ── 7. 发送线程 ──────────────────────────────────────────────────────────
    txers_tx = {ch: txers[ch] for ch in args.tx_channels if ch in txers}
    if txers_tx:
        t = threading.Thread(
            target=_send_worker,
            args=(txers_tx, args.tx_channels, args.send_id,
                  args.freq, args.fd, tx_counters, stop_event),
            daemon=True,
            name="sender",
        )
        t.start()
        threads.append(t)
    else:
        print("⚠️  发送通道全部未成功打开，跳过发送")

    # ── 8. 主循环 ────────────────────────────────────────────────────────────
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
