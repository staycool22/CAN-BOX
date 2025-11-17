import argparse
import time
from typing import Optional

try:
    import can
except ImportError:
    can = None

from CANMessageTransmitter import CANMessageTransmitter


def ensure_python_can():
    if can is None:
        raise RuntimeError("python-can 未安装。请先运行: pip install python-can")


def parse_bitrate_token(token: str) -> int:
    s = str(token).strip().lower()
    try:
        if s.isdigit():
            return int(s)
        if s.endswith('k'):
            base = float(s[:-1])
            return int(base * 1000)
        if s.endswith('m'):
            base = float(s[:-1])
            return int(base * 1000000)
    except Exception:
        pass
    raise ValueError(f"无法解析速率值: {token}")


def main():
    parser = argparse.ArgumentParser(description="在 Windows 上使用 TZCANTransmitter 进行 CAN 发送测试（candle/gs_usb）")
    parser.add_argument("--backend", choices=["candle", "gs_usb"], default="candle", help="选择后端：candle 或 gs_usb")
    parser.add_argument("--index", type=int, default=0, help="设备索引，从 0 起")
    parser.add_argument("--mode", choices=["all", "can", "fd"], default="all", help="测试模式：全部/仅CAN/仅FD")
    parser.add_argument("--count", type=int, default=100000, help="每种配置的发送帧数")
    parser.add_argument("--freq", type=float, default=1000.0, help="发送频率 Hz")
    parser.add_argument("--fd_len", type=int, default=16, help="FD 帧数据长度（字节），例如 16/32/64")
    parser.add_argument("--no-brs", action="store_true", help="FD 模式下关闭 BRS（数据域不切到高速）")
    parser.add_argument("--sp", type=float, help="仲裁段采样点 (0-1.0)")
    parser.add_argument("--dsp", type=float, help="数据段采样点 (0-1.0)")
    parser.add_argument("--can-br", nargs='+', help="选择 CAN2.0 速率，例如: 500k 或 1m，可多选")
    parser.add_argument("--fd-dbr", nargs='+', help="选择 CAN FD 数据域速率，例如: 1m 5m 8m 10m，可多选")
    parser.add_argument("--fd-arb", default=None, help="CAN FD 仲裁域速率（必填），例如 500k/1m 或精确数值")
    parser.add_argument("--rx-index", type=int, help="可选：指定另一台设备用于接收统计（例如 0 或 1）")
    args = parser.parse_args()

    print(parser.format_help())
    print("指南：")
    print("- CAN2.0：建议设置 `--can-br` 如 500k 1m；不设置时默认 500k、1m")
    print("- CAN FD：建议设置 `--fd-arb` 与 `--fd-dbr`，例如 `--fd-arb 500k --fd-dbr 5m 8m`；不设置时默认仲裁=500k，数据=1m/5m/8m/10m")
    print("- 示例：")
    print("  python test_tzcan_send_win.py --mode can --can-br 500k 1m")
    print("  python test_tzcan_send_win.py --mode fd --fd-arb 500k --fd-dbr 5m 8m --fd-len 64")

    need_can = args.mode in ("all", "can")
    need_fd = args.mode in ("all", "fd")
    missing = []
    if need_can and args.can_br is None:
        missing.append("--can-br")
    if need_fd:
        if args.fd_dbr is None:
            missing.append("--fd-dbr")
        if args.fd_arb is None:
            missing.append("--fd-arb")
        if args.backend != 'candle':
            missing.append("--backend candle")
    if missing:
        print("未设置必要参数：" + ", ".join(missing))
        print("请设置参数后重新运行脚本进行发送")
        return

    ensure_python_can()

    can_bitrates = [parse_bitrate_token(v) for v in args.can_br] if args.can_br else []
    fd_data_bitrates = [parse_bitrate_token(v) for v in args.fd_dbr] if args.fd_dbr else []
    fd_arb_bitrate = parse_bitrate_token(args.fd_arb) if args.fd_arb else None

    if need_can:
        for bitrate in can_bitrates:
            print(f"[CAN2.0] 后端 {args.backend}，设备 index={args.index}，bitrate={bitrate}")
            TX = CANMessageTransmitter.choose_can_device("TZCAN")
            channels = [args.index] + ([args.rx_index] if args.rx_index is not None else [])
            m_dev, _, _ = TX.init_can_device(baud_rate=bitrate, channels=channels, backend=args.backend, fd=False, sp=args.sp)
            try:
                bus_tx = m_dev['buses'][args.index]
                tx = TX(bus_tx)
                period = 1.0 / args.freq
                payload = bytes((i % 256 for i in range(8)))
                start = time.perf_counter()
                next_t = start
                for i in range(args.count):
                    data_bytes = bytes(((i >> 0) & 0xFF, (i >> 8) & 0xFF, (i >> 16) & 0xFF, (i >> 24) & 0xFF)) + payload[4:]
                    tx._send_can_data(send_id=0x321, data_list=list(data_bytes), is_ext_frame=False, canfd_mode=False, brs=0, esi=0)
                    next_t += period
                    delay = next_t - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
                elapsed = time.perf_counter() - start
                print(f"完成发送 {args.count} 帧，目标频率 {args.freq}Hz，用时 {elapsed:.3f}s，平均 {args.count/elapsed:.1f} FPS")
            finally:
                TX.close_can_device(m_dev)

    if need_fd:
        for dbitrate in fd_data_bitrates:
            print(f"[CAN FD] 后端 {args.backend}，设备 index={args.index}，arb={fd_arb_bitrate}，data={dbitrate}")
            TX = CANMessageTransmitter.choose_can_device("TZCAN")
            channels = [args.index] + ([args.rx_index] if args.rx_index is not None else [])
            m_dev, _, _ = TX.init_can_device(baud_rate=fd_arb_bitrate, dbit_baud_rate=dbitrate, channels=channels, backend=args.backend, fd=True, sp=args.sp, dsp=args.dsp)
            try:
                bus_tx = m_dev['buses'][args.index]
                tx = TX(bus_tx)
                period = 1.0 / args.freq
                payload = bytes((i % 256 for i in range(args.fd_len)))
                brs_flag = 0 if args.no_brs else 1
                start = time.perf_counter()
                next_t = start
                for i in range(args.count):
                    data_bytes = bytes(((i >> 0) & 0xFF, (i >> 8) & 0xFF, (i >> 16) & 0xFF, (i >> 24) & 0xFF)) + payload[4:]
                    tx._send_can_data(send_id=0x456, data_list=list(data_bytes), is_ext_frame=False, canfd_mode=True, brs=brs_flag, esi=0)
                    next_t += period
                    delay = next_t - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
                elapsed = time.perf_counter() - start
                print(f"完成发送 {args.count} 帧，目标频率 {args.freq}Hz，用时 {elapsed:.3f}s，平均 {args.count/elapsed:.1f} FPS")
            finally:
                TX.close_can_device(m_dev)


if __name__ == "__main__":
    main()

