# 说明：TZETHCANTransmitter 发送测试脚本，通过 cannelloni/vcan 发送 CAN/CAN FD 报文
# 用途：初始化 ETH-CAN 通道（自动通过 UDP 配置 HPM 硬件波特率），按目标频率持续发帧并统计
# 注意：需先运行 setup_cannelloni.sh，确保 vcan 接口已建立；仅适用于 Linux/WSL
import argparse
import time
import os
import sys
import threading
from typing import List

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


def run_send_channel(
    ch: int,
    tx: 'TZETHCANTransmitter',
    send_id: int,
    count: int,
    freq: float,
    is_fd: bool,
    fd_len: int,
    brs: bool,
):
    """单通道发送逻辑（阻塞，在独立线程中运行）。"""
    period = 1.0 / freq
    payload_len = fd_len if is_fd else 8

    fd_info = f", fd_len={fd_len}, brs={brs}" if is_fd else ""
    print(f"[CH{ch}/vcan{ch}] 开始发送: ID=0x{send_id:X}, count={count}, freq={freq}Hz, fd={is_fd}{fd_info}")

    start = time.perf_counter()
    next_t = start
    sent = 0
    errors = 0

    for i in range(count):
        # 构造数据：低位在前的 4 字节递增计数 + 固定填充
        prefix = bytes((i >> (8 * b)) & 0xFF for b in range(4))
        data = list(prefix) + [((ch * 16 + j) & 0xFF) for j in range(4, payload_len)]

        ok = tx._send_can_data(
            send_id=send_id,
            data_list=data,
            is_ext_frame=False,
            canfd_mode=is_fd,
            brs=1 if (is_fd and brs) else 0,
            esi=0,
        )
        if not ok:
            errors += 1

        sent += 1
        next_t += period
        delay = next_t - time.perf_counter()
        if delay > 0:
            time.sleep(delay)

    elapsed = time.perf_counter() - start
    fps = sent / elapsed if elapsed > 0 else 0
    print(f"[CH{ch}/vcan{ch}] 完成: 发送 {sent} 帧，错误 {errors}，"
          f"用时 {elapsed:.3f}s，平均 {fps:.1f} FPS")


def main():
    parser = argparse.ArgumentParser(
        description="通过 cannelloni/vcan 发送 CAN/CAN FD 报文（TZETHCANTransmitter）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python ethcan_send.py --channels 0                                    # CAN 2.0，vcan0，500kbps，1000Hz
  python ethcan_send.py --channels 0 1 2 3 --baud-rate 1m               # 4 通道并发发送
  python ethcan_send.py --channels 0 --mode fd --dbit-baud-rate 5m --fd-len 64 --freq 500
  python ethcan_send.py --channels 0 --send-id 0x321 --count 50000 --freq 2000
前置条件: 运行 ./setup_cannelloni.sh 建立 vcan 与 cannelloni 转发
        """,
    )
    parser.add_argument("--channels", nargs='+', type=int, default=[0],
                        help="vcan 通道索引列表，例如: 0 1 2 3（默认: 0）")
    parser.add_argument("--baud-rate", default="500k",
                        help="仲裁域波特率，例如 500k、1m（默认: 500k）")
    parser.add_argument("--mode", choices=["can", "fd"], default="can",
                        help="发送模式：can=CAN 2.0, fd=CAN FD（默认: can）")
    parser.add_argument("--dbit-baud-rate", default="2m",
                        help="FD 数据域波特率，例如 2m、5m（默认: 2m，mode=fd 时生效）")
    parser.add_argument("--freq", type=float, default=1000.0,
                        help="发送频率 Hz（默认: 1000）")
    parser.add_argument("--count", type=int, default=10000,
                        help="每通道发送帧数（默认: 10000）")
    parser.add_argument("--send-id", type=lambda x: int(x, 0), default=0x123,
                        help="发送报文 ID（默认: 0x123）")
    parser.add_argument("--fd-len", type=int, default=64,
                        help="FD 帧数据长度（字节），有效值 1-64（默认: 64，mode=fd 时生效）")
    parser.add_argument("--no-brs", action="store_true",
                        help="FD 模式关闭 BRS（数据域不切换到高速）")
    parser.add_argument("--ip", default=None,
                        help="HPM 硬件目标 IP（默认使用 ETHCANConstants.TARGET_IP=192.168.1.10）")
    args = parser.parse_args()

    ensure_python_can()

    baud_rate = parse_bitrate_token(args.baud_rate)
    dbit_baud_rate = parse_bitrate_token(args.dbit_baud_rate)
    is_fd = args.mode == "fd"

    # fd_len 校验：CAN FD 合法 DLC 对应的数据长度
    valid_fd_lens = [0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64]
    fd_len = args.fd_len
    if is_fd and fd_len not in valid_fd_lens:
        # 向上取最近的合法值
        fd_len = next((v for v in valid_fd_lens if v >= fd_len), 64)
        print(f"⚠️ fd-len 调整为最近的合法 DLC 长度: {fd_len}")

    fd_info = f", dbit_baud_rate={dbit_baud_rate}" if is_fd else ""
    print(f"初始化 TZETHCANTransmitter: channels={args.channels}, baud_rate={baud_rate}{fd_info}, fd={is_fd}")
    print("（将通过 UDP 向 HPM 硬件发送波特率配置包）")

    TX, m_dev, _, _ = CANMessageTransmitter.open(
        "TZETHCAN",
        baud_rate=baud_rate,
        dbit_baud_rate=dbit_baud_rate,
        channels=args.channels,
        fd=is_fd,
        target_ip=args.ip,
    )

    threads: List[threading.Thread] = []

    for ch in args.channels:
        raw_bus = m_dev["buses"].get(ch)
        if raw_bus is None:
            print(f"❌ 通道 {ch} (vcan{ch}) 初始化失败，跳过")
            continue
        # 构造 TZETHCANTransmitter 实例用于发送
        tx = TX(raw_bus, channel_id=ch, is_canfd=is_fd)
        print(f"✅ 已打开 vcan{ch}")

        t = threading.Thread(
            target=run_send_channel,
            args=(ch, tx, args.send_id, args.count, args.freq,
                  is_fd, fd_len, not args.no_brs),
            daemon=True,
        )
        threads.append(t)

    if not threads:
        print("没有可用的发送通道，退出。")
        TX.close_can_device(m_dev)
        return

    print(f"\n开始发送（共 {len(threads)} 个通道并发）...\n")
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    TX.close_can_device(m_dev)
    print("发送结束。")


if __name__ == "__main__":
    main()
