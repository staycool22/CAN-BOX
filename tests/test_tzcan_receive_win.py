# 说明：Windows 端 CAN/CAN FD 接收测试脚本（candle/gs_usb 后端）
# 用途：在 Windows 控制台下接收并统计帧，可按任意键退出；支持按 ID 与模式过滤
# 注意：FD 模式仅在 candle 后端支持；键盘退出依赖 msvcrt，仅适用于 Windows 控制台
import argparse
import time
from typing import Optional, Tuple

# 依赖 python-can；未安装时在运行时给出明确提示
try:
    import can
except ImportError:
    can = None


try:
    # Windows 控制台键盘处理库：用于检测按键退出
    import msvcrt  # type: ignore
except Exception:
    msvcrt = None

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

try:
    from can_bridge.CANMessageTransmitter import CANMessageTransmitter
    # 动态加载 TZCAN 后端
    TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
except ImportError:
    # 尝试兼容同级目录直接运行
    try:
        from TZCANTransmitter import TZCANTransmitter
    except ImportError:
        raise ImportError("无法加载 TZCANTransmitter，请检查路径。")



def ensure_python_can():
    # 运行前校验第三方库是否已安装
    if can is None:
        raise RuntimeError("python-can 未安装。请先运行: pip install python-can")


def parse_bitrate_token(token: str) -> int:
    # 将 '500k' / '1m' 这类速率字符串解析为整数位速率（bit/s）
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




def _any_key_pressed() -> bool:
    """检测是否有任意键按下（仅 Windows 控制台）。"""
    # msvcrt.kbhit() 返回是否有键盘输入待处理
    return (msvcrt is not None) and bool(msvcrt.kbhit())


def _drain_key_buffer():
    """清空键盘缓冲，避免后续重复触发。"""
    # 轮询读取缓冲区直至为空
    if msvcrt is None:
        return
    try:
        while msvcrt.kbhit():
            msvcrt.getch()
    except Exception:
        pass




class ReceiverStats:
    # 接收统计：总数/CAN/FD 计数、首尾时间戳、过滤设置
    def __init__(self, report_every: int = 1000, filter_id: Optional[int] = None, fd_only: bool = False, can_only: bool = False):
        self.total = 0
        self.can_count = 0
        self.fd_count = 0
        self.report_every = report_every
        self.filter_id = filter_id
        self.fd_only = fd_only
        self.can_only = can_only
        self.first_ts: Optional[float] = None
        self.last_ts: Optional[float] = None

    def accept(self, msg: can.Message) -> bool:
        # 按 ID 与 CAN/FD 模式过滤
        if self.filter_id is not None and msg.arbitration_id != self.filter_id:
            return False
        if self.fd_only and not getattr(msg, "is_fd", False):
            return False
        if self.can_only and getattr(msg, "is_fd", False):
            return False
        return True

    def update(self, msg: can.Message):
        # 更新计数与时间戳；可选按间隔打印进度（默认关闭）
        if not self.accept(msg):
            return False
        now = time.perf_counter()
        self.total += 1
        if getattr(msg, "is_fd", False):
            self.fd_count += 1
        else:
            self.can_count += 1
        if self.first_ts is None:
            self.first_ts = now
        self.last_ts = now
        if self.report_every and (self.total % self.report_every == 0):
            # 默认不输出，按需可扩展进度输出
            pass
        return True

    def summary(self) -> Tuple[int, float]:
        # 返回总条数与平均速率（FPS）
        elapsed = (self.last_ts - self.first_ts) if (self.last_ts and self.first_ts and self.last_ts > self.first_ts) else 0.0
        rate = self.total / elapsed if elapsed > 0 else 0.0
        return self.total, rate


def _format_time(ts: float, base_ts: Optional[float], mode: str) -> str:
    # 支持绝对时间或相对首条消息的时间
    if mode == 'abs':
        ms = int((ts - int(ts)) * 1000)
        return time.strftime('%H:%M:%S', time.localtime(ts)) + f'.{ms:03d}'
    if base_ts is None:
        base_ts = ts
    delta = ts - base_ts
    return f"{delta:.6f}s"


def receive_frames(
    tx,
    duration_s: float,
    max_count: Optional[int] = None,
    report_every: int = 0,
    filter_id: Optional[int] = None,
    fd_only: bool = False,
    can_only: bool = False,
    print_each: bool = True,
    time_mode: str = 'rel',
    bus_type_label: Optional[str] = None,
    key_exit: bool = True,
):
    # 通用接收循环：支持时长/条数限制、键盘退出、逐条打印与统计
    stats = ReceiverStats(report_every=report_every, filter_id=filter_id, fd_only=fd_only, can_only=can_only)
    deadline = (time.perf_counter() + duration_s) if (duration_s and duration_s > 0) else None
    first_msg_ts: Optional[float] = None

    try:
        while True:
            # 任意键退出
            if key_exit and _any_key_pressed():
                _drain_key_buffer()
                print("检测到按键，退出接收...")
                break

            if deadline is not None and time.perf_counter() >= deadline:
                break

            # 从总线拉取一条消息；遇到错误帧时由底层抛异常
            ok, data, msg = tx._receive_can_data(target_id=filter_id, timeout=0.1, is_ext_frame=None, canfd_mode=fd_only, stop_on_error=True, return_msg=True)
            if ok and msg is not None:
                if stats.update(msg) and print_each:
                    # 格式化时间戳/ID/数据并输出一行
                    ts = getattr(msg, 'timestamp', None)
                    if ts is None:
                        ts = time.time()
                    if first_msg_ts is None:
                        first_msg_ts = ts
                    t_str = _format_time(ts, first_msg_ts if time_mode == 'rel' else None, time_mode)
                    id_hex = f"0x{msg.arbitration_id:03X}" if not getattr(msg, 'is_extended_id', False) else f"0x{msg.arbitration_id:08X}"
                    data_bytes = getattr(msg, 'data', b'') or b''
                    data_hex = ' '.join(f"{b:02X}" for b in data_bytes)
                    dlc = getattr(msg, 'dlc', len(data_bytes))
                    is_err = bool(getattr(msg, 'is_error_frame', False))
                    is_rtr = bool(getattr(msg, 'is_remote_frame', False))
                    type_str = 'ERROR' if is_err else ('REMOTE' if is_rtr else 'DATA')
                    bus_label = bus_type_label if bus_type_label else ('CAN FD' if getattr(msg, 'is_fd', False) else 'CAN')
                    print(f"RX {t_str} BUS={bus_label} TYPE={type_str} ID={id_hex} LEN={dlc} DATA={data_hex}")

                if max_count and stats.total >= max_count:
                    break
    finally:
        # 结束后输出总计与平均速率（若设置了时长）
        total, rate = stats.summary()
        if duration_s and duration_s > 0:
            print(f"总计接收: {total} 条，平均速率: {rate:.1f} FPS")
        else:
            print(f"总计接收: {total} 条")


def run_can_receive_win(backend: str, index: int, bitrate: int, duration_s: float, max_count: Optional[int], filter_id: Optional[int], report_every: int, print_each: bool, time_mode: str, sample_point: Optional[float]):
    # 打开指定后端与设备索引，以 CAN2.0 模式接收
    # 使用已加载的 TZCANTransmitter 类
    TX = TZCANTransmitter
    m_dev, _, _ = TX.init_can_device(baud_rate=bitrate, channels=[index], backend=backend, fd=False, sp=sample_point)
    try:
        tx = TX(m_dev['buses'][index])
        receive_frames(tx, duration_s=duration_s, max_count=max_count, report_every=report_every, filter_id=filter_id, fd_only=False, can_only=True, print_each=print_each, time_mode=time_mode, bus_type_label="CAN 2.0", key_exit=True)
    finally:
        TX.close_can_device(m_dev)


def run_fd_receive_win(backend: str, index: int, arb_bitrate: int, data_bitrate: int, duration_s: float, max_count: Optional[int], filter_id: Optional[int], report_every: int, print_each: bool, time_mode: str, sample_point: Optional[float], data_sample_point: Optional[float]):
    # FD 接收：仅 candle 后端支持 FD；需安装 python-can-candle
    if backend != 'candle':
        raise RuntimeError("FD 模式需要使用 candle 后端。请安装 python-can-candle 并指定 --backend candle")
    # 使用已加载的 TZCANTransmitter 类
    TX = TZCANTransmitter
    m_dev, _, _ = TX.init_can_device(baud_rate=arb_bitrate, dbit_baud_rate=data_bitrate, channels=[index], backend=backend, fd=True, sp=sample_point, dsp=data_sample_point)
    try:
        tx = TX(m_dev['buses'][index])
        receive_frames(tx, duration_s=duration_s, max_count=max_count, report_every=report_every, filter_id=filter_id, fd_only=True, can_only=False, print_each=print_each, time_mode=time_mode, bus_type_label="CAN FD", key_exit=True)
    finally:
        TX.close_can_device(m_dev)


def main():
    # 命令行入口：选择后端与速率，分 CAN/FD 模式进行接收
    parser = argparse.ArgumentParser(description="在 Windows 上使用 candle/gs_usb 进行 CAN/CAN FD 接收测试")
    parser.add_argument("--backend", choices=["candle", "gs_usb"], default="candle", help="选择后端：candle 或 gs_usb")
    parser.add_argument("--index", type=int, default=0, help="设备索引，从 0 起")
    parser.add_argument("--mode", choices=["all", "can", "fd"], default="all", help="接收模式：全部/仅CAN/仅FD")
    parser.add_argument("--duration", type=float, default=0.0, help="每种配置的接收时长（秒）；0 表示持续接收直至按键退出")
    parser.add_argument("--count", type=int, default=0, help="可选：达到指定接收条数后提前结束")
    parser.add_argument("--report-every", type=int, default=0, help="默认不输出进度，仅打印每条时间与总数；>0 时按间隔输出进度")
    parser.add_argument("--filter-id", type=lambda x: int(x, 0), default=None, help="可选：仅接收统计指定ID，如 0x456 或 1110")
    parser.add_argument("--quiet", action="store_true", help="不逐条打印时间，仅在退出时输出总数")
    parser.add_argument("--time-mode", choices=["rel", "abs"], default="rel", help="逐条打印时间格式：rel 相对首条，abs 绝对时刻")
    parser.add_argument("--sp", type=float, help="仲裁段采样点：0-1（如 0.8=80%%）或 50-90（直接百分比）")
    parser.add_argument("--dsp", type=float, help="数据段采样点：0-1（如 0.8=80%%）或 50-90（直接百分比）")
    # 速率选择：可多值，分别用于 CAN2.0 和 CAN FD 数据域
    parser.add_argument("--can-br", nargs='+', help="选择 CAN2.0 速率，例如: 500k 或 1m，可多选")
    parser.add_argument("--fd-dbr", nargs='+', help="选择 CAN FD 数据域速率，例如: 1m 5m 8m，可多选")
    parser.add_argument("--fd-arb", default="500k", help="CAN FD 仲裁域速率（默认 500k），可填如 500k/1m 或精确数值")

    args = parser.parse_args()

    print(parser.format_help())
    print("指南：")
    print("- CAN2.0：建议设置 `--can-br` 如 500k 1m")
    print("- CAN FD：建议设置 `--fd-arb` 与 `--fd-dbr`，例如 `--fd-arb 500k --fd-dbr 5m 8m`")
    print("- 默认接收时长为 0，持续接收，按任意键退出")
    print("- 示例：")
    print("  python test_tzcan_receive_win.py --mode can --can-br 500k 1m --filter-id 0x321")
    print("  python test_tzcan_receive_win.py --mode fd --fd-arb 500k --fd-dbr 5m 8m --filter-id 0x456")

    # 先校验依赖
    ensure_python_can()

    # 解析速率列表与接收上限
    can_bitrates = [500000, 1000000] if not args.can_br else [parse_bitrate_token(v) for v in args.can_br]
    fd_data_bitrates = [1000000, 5000000, 8000000] if not args.fd_dbr else [parse_bitrate_token(v) for v in args.fd_dbr]
    fd_arb_bitrate = parse_bitrate_token(args.fd_arb)
    max_count = args.count if args.count and args.count > 0 else None

    print_each = not args.quiet

    if args.mode in ("all", "can"):
        # 逐个 CAN2.0 速率进行接收
        for bitrate in can_bitrates:
            print(f"[CAN2.0 RX] 后端 {args.backend}，设备 index={args.index}，bitrate={bitrate}")
            run_can_receive_win(args.backend, index=args.index, bitrate=bitrate, duration_s=args.duration, max_count=max_count, filter_id=args.filter_id, report_every=args.report_every, print_each=print_each, time_mode=args.time_mode, sample_point=args.sp)

    if args.mode in ("all", "fd"):
        # FD 模式仅支持 candle 后端
        if args.backend != 'candle':
            raise RuntimeError("FD 模式需要使用 candle 后端。请安装 python-can-candle 并指定 --backend candle")
        for dbitrate in fd_data_bitrates:
            print(f"[CAN FD RX] 后端 {args.backend}，设备 index={args.index}，arb={fd_arb_bitrate}，data={dbitrate}")
            run_fd_receive_win(args.backend, index=args.index, arb_bitrate=fd_arb_bitrate, data_bitrate=dbitrate, duration_s=args.duration, max_count=max_count, filter_id=args.filter_id, report_every=args.report_every, print_each=print_each, time_mode=args.time_mode, sample_point=args.sp, data_sample_point=args.dsp)


if __name__ == "__main__":
    main()