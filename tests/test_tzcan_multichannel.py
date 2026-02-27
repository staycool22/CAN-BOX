"""多通道 CAN 测试：同时监听多个通道，定时在指定通道发送

架构说明
--------
socketcan：每个通道拥有独立的 can.Bus，各通道接收互不干扰，每通道一个
           dispatcher 线程直接从 Bus 读取并入队。

candle   ：同一物理设备的多个通道共享同一个 can.Bus 对象。若多个线程同时
           调用 bus.recv()，每条消息只会被其中一个线程取走，导致通道间消息
           互相"偷走"。因此对共享 Bus 使用单 dispatcher 线程读取，再按
           msg.channel 派发到各通道独立队列，消费线程从队列读取。

用法（socketcan - Linux）：
  python3 tests/test_tzcan_multichannel.py \\
      --channels 0 1 2 3 --tx-channels 0 1 \\
      --can-br 500k --freq 10 --duration 30

用法（candle - Windows，需 python-can-candle）：
  python3 tests/test_tzcan_multichannel.py \\
      --backend candle --channels 0 1 2 3 --tx-channels 0 1 \\
      --can-br 500k --freq 10
"""
import argparse
import queue
import sys
import os
import threading
import time
from collections import defaultdict

try:
    from can_bridge import CANMessageTransmitter
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from can_bridge import CANMessageTransmitter


# ── 工具函数 ─────────────────────────────────────────────────────────────────

def parse_bitrate_token(s):
    s = s.lower().strip()
    if s.endswith('k'): return int(float(s[:-1]) * 1_000)
    if s.endswith('m'): return int(float(s[:-1]) * 1_000_000)
    return int(s)


# ── 线程工作函数 ──────────────────────────────────────────────────────────────

def _dispatch_worker(bus, chs, rx_queues, stop_event):
    """从 bus 接收消息，派发到各通道队列。

    socketcan：每个 bus 只对应一个通道（chs 长度为 1），直接入队。
    candle    ：多通道共享 bus，按 msg.channel（整数）派发；无法匹配时
                降级到 chs[0]，并打印警告（仅首次）。
    """
    single_ch = chs[0] if len(chs) == 1 else None
    unmatched_warned = False

    while not stop_event.is_set():
        try:
            msg = bus.recv(timeout=0.1)
        except Exception:
            break
        if msg is None:
            continue

        if single_ch is not None:
            # socketcan：独立 bus，不需要按 msg.channel 区分
            rx_queues[single_ch].put_nowait(msg)
        else:
            # candle：共享 bus，按 msg.channel 整数值派发
            raw_ch = getattr(msg, 'channel', None)
            target = None
            if raw_ch is not None:
                try:
                    candidate = int(raw_ch)
                    if candidate in rx_queues:
                        target = candidate
                except (TypeError, ValueError):
                    pass
            if target is None:
                if not unmatched_warned:
                    print(f"⚠️  dispatcher: 无法从 msg.channel={raw_ch!r} 匹配通道 "
                          f"{chs}，消息将归入 ch{chs[0]}")
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


def _send_worker(txers_tx, tx_channels, send_id, freq, tx_counters, stop_event):
    """定时在 tx_channels 上循环发送，数据包含序号和通道号便于调试。"""
    interval = 1.0 / freq
    seq = 0
    while not stop_event.is_set():
        t0 = time.time()
        for ch in tx_channels:
            if ch not in txers_tx:
                continue
            # 数据格式：[seq(4B BE), ch, 0, 0, 0]
            data = [
                (seq >> 24) & 0xFF, (seq >> 16) & 0xFF,
                (seq >> 8) & 0xFF,  seq & 0xFF,
                ch & 0xFF, 0, 0, 0,
            ]
            ok = txers_tx[ch]._send_can_data(
                send_id=send_id, data_list=data,
                is_ext_frame=False, canfd_mode=False,
            )
            if ok:
                tx_counters[ch] += 1
        seq = (seq + 1) & 0xFFFFFFFF
        elapsed = time.time() - t0
        time.sleep(max(0.0, interval - elapsed))


# ── 主函数 ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="多通道 CAN 收发测试")
    parser.add_argument('--channels',    type=int, nargs='+', default=[0, 1, 2, 3],
                        help='要同时打开并监听的通道列表')
    parser.add_argument('--tx-channels', type=int, nargs='+', default=[0, 1],
                        help='定时发送的通道子集（必须在 --channels 中）')
    parser.add_argument('--can-br',      default='500k',      help='CAN 仲裁域波特率')
    parser.add_argument('--backend',     default='socketcan',
                        help='后端：socketcan / candle / gs_usb')
    parser.add_argument('--send-id',     type=lambda x: int(x, 0), default=0x123,
                        help='发送帧 ID（支持 0x 前缀），默认 0x123')
    parser.add_argument('--freq',        type=float, default=10.0,
                        help='发送频率 Hz，默认 10')
    parser.add_argument('--duration',    type=float, default=0.0,
                        help='运行时长 s，0=持续直到 Ctrl+C')
    args = parser.parse_args()

    baud = parse_bitrate_token(args.can_br)

    # 校验 tx-channels ⊆ channels
    bad = set(args.tx_channels) - set(args.channels)
    if bad:
        print(f"❌ --tx-channels 包含不在 --channels 中的通道: {sorted(bad)}")
        sys.exit(1)

    # ── 1. 一次调用打开所有通道 ──────────────────────────────────────────────
    print(f"正在初始化通道 {args.channels}（backend={args.backend}，baud={baud}）...")
    TX, m_dev, _, _ = CANMessageTransmitter.open(
        "TZCAN",
        baud_rate=baud,
        channels=args.channels,
        backend=args.backend,
    )

    # ── 2. 按通道号从 m_dev["buses"] 取 Bus，实例化各通道的 Transmitter ─────
    #
    #   关键：不用 ch0/ch1 快捷引用，直接按 key 查字典。
    #   ch0/ch1 只保证 key=0 或 key=1 的通道有值；key=2,3,... 时它们是 None。
    #
    #   channel_id 仅对 candle 共享 Bus 有意义（发送时标记 msg.channel 路由）。
    #   socketcan 每 Bus 独立，不需要 channel_id（若设置会导致接收过滤器失配）。
    #
    txers: dict[int, object] = {}
    for ch in args.channels:
        bus = m_dev["buses"].get(ch)
        if bus is None:
            print(f"⚠️  通道 {ch} 未能成功打开，跳过")
            continue
        cid = ch if args.backend != "socketcan" else None
        txers[ch] = TX(bus, channel_id=cid)

    if not txers:
        print("❌ 没有成功打开任何通道，退出")
        TX.close_can_device(m_dev)
        sys.exit(1)

    print(f"已打开通道: {sorted(txers.keys())}")
    print(f"监听通道  : {sorted(txers.keys())}")
    print(f"发送通道  : {args.tx_channels}，"
          f"频率 {args.freq} Hz，ID=0x{args.send_id:X}")
    if args.duration > 0:
        print(f"运行时长  : {args.duration} s")
    else:
        print("持续运行，按 Ctrl+C 退出")

    # ── 3. 按唯一 Bus 对象分组，决定 dispatcher 线程数 ──────────────────────
    #
    #   socketcan：每通道一个独立 Bus → 每 Bus 一个 dispatcher（即每通道一个）
    #   candle   ：同设备多通道共享一个 Bus → 该 Bus 对应一个 dispatcher
    #
    bus_id_to_chs: dict[int, list[int]] = defaultdict(list)
    for ch, txer in txers.items():
        bus_id_to_chs[id(txer.bus)].append(ch)

    # ── 4. 每通道一个接收队列 ────────────────────────────────────────────────
    rx_queues: dict[int, queue.Queue] = {ch: queue.Queue() for ch in txers}

    stop_event = threading.Event()
    rx_counters: dict[int, int] = defaultdict(int)
    tx_counters: dict[int, int] = defaultdict(int)
    threads = []

    # ── 5. 启动 dispatcher 线程（每个唯一 Bus 一个）─────────────────────────
    for bus_id, chs in bus_id_to_chs.items():
        actual_bus = txers[chs[0]].bus
        t = threading.Thread(
            target=_dispatch_worker,
            args=(actual_bus, chs, rx_queues, stop_event),
            daemon=True,
            name=f"dispatch-bus{bus_id & 0xFFFF:04X}",
        )
        t.start()
        threads.append(t)

    # ── 6. 启动 consumer 线程（每通道一个，从队列读取并打印）───────────────
    for ch in txers:
        t = threading.Thread(
            target=_consume_worker,
            args=(ch, rx_queues[ch], rx_counters, stop_event),
            daemon=True,
            name=f"consume-ch{ch}",
        )
        t.start()
        threads.append(t)

    # ── 7. 启动发送线程 ──────────────────────────────────────────────────────
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
        print("⚠️  发送通道全部未成功打开，跳过发送线程")

    # ── 8. 主循环：定时打印统计，到时间或 Ctrl+C 退出 ───────────────────────
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

    # ── 9. 最终统计 ──────────────────────────────────────────────────────────
    total_elapsed = time.time() - t_start
    print(f"\n── 最终统计（运行 {total_elapsed:.1f}s）──")
    for ch in sorted(txers.keys()):
        rx_fps = rx_counters[ch] / total_elapsed if total_elapsed > 0 else 0
        tx_fps = tx_counters[ch] / total_elapsed if total_elapsed > 0 else 0
        tx_info = f"  TX {tx_counters[ch]:6d} 帧 ({tx_fps:5.1f} fps)" if ch in txers_tx else ""
        print(f"  ch{ch}: RX {rx_counters[ch]:6d} 帧 ({rx_fps:5.1f} fps){tx_info}")


if __name__ == '__main__':
    main()
