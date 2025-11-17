# 说明：TZCAN 接收测试脚本，支持 CAN/CAN FD，基于 python-can/socketcan
# 用途：批量配置速率，接收并统计报文，可检测总线错误，按 ESC 退出
# 注意：socketcan 配置命令仅适用于 Linux；在 Windows 下请使用对应驱动或在 WSL 运行
import argparse
import time
import os
import sys
import select
import termios
import tty
import threading
from typing import Optional, List, Tuple

# 依赖 python-can；未安装时在运行时给出明确提示
try:
    import can
except ImportError:
    can = None

# 动态添加项目根目录到 sys.path，便于从包或同目录导入
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    from CAN.CANMessageTransmitter import CANMessageTransmitter
except Exception:
    # 兼容直接运行脚本的场景（同目录下存在 CANMessageTransmitter.py）
    from CANMessageTransmitter import CANMessageTransmitter


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
            return int(float(s[:-1]) * 1000)
        if s.endswith('m'):
            return int(float(s[:-1]) * 1000000)
    except Exception:
        pass
    raise ValueError(f"无法解析速率值: {token}")


def configure_socketcan(
    interface: str,
    bitrate: Optional[int] = None,
    fd: bool = False,
    data_bitrate: Optional[int] = None,
    sample_point: Optional[float] = None,
    dsample_point: Optional[float] = None,
):
    # 利用 ip link 配置 socketcan 接口速率；优先尝试 sudo
    sp_opts = ""
    if sample_point is not None:
        # 仲裁段采样点（百分比或小数）
        sp_opts += f" sample-point {sample_point}"
    if dsample_point is not None and fd:
        # 数据段采样点（仅 FD 模式生效）
        sp_opts += f" dsample-point {dsample_point}"

    if fd:
        if bitrate is None or data_bitrate is None:
            raise ValueError("FD 模式需要同时提供仲裁域 bitrate 和数据域 data_bitrate")
        cmd_base = f"ip link set {interface} type can bitrate {bitrate}{sp_opts} dbitrate {data_bitrate} fd on"
    else:
        if bitrate is None:
            raise ValueError("标准 CAN 模式需要提供 bitrate")
        cmd_base = f"ip link set {interface} type can bitrate {bitrate}{sp_opts}"

    cmd_sudo = f"sudo ip link set {interface} down; sudo {cmd_base}; sudo ip link set {interface} up"
    cmd_nosudo = f"ip link set {interface} down; {cmd_base}; ip link set {interface} up"

    # 优先 sudo；失败则尝试无 sudo
    rc = os.system(cmd_sudo)
    if rc != 0:
        os.system(cmd_nosudo)


class ReceiverThread(threading.Thread):
    # 后台接收线程：拉取报文、做过滤、统计并可逐条打印
    def __init__(
        self,
        transmitter,
        duration_s: float,
        max_count: Optional[int],
        filter_id: Optional[int],
        fd_only: bool,
        can_only: bool,
        print_each: bool,
        time_mode: str,
        bus_type_label: str,
    ):
        super().__init__(daemon=True)
        self.transmitter = transmitter
        self.duration_s = duration_s
        self.max_count = max_count
        self.filter_id = filter_id
        self.fd_only = fd_only
        self.can_only = can_only
        self.print_each = print_each
        self.time_mode = time_mode
        self.bus_type_label = bus_type_label
        
        # 接收统计：总数/CAN/FD/错误计数
        self.stats = {'total': 0, 'can': 0, 'fd': 0, 'error': 0}
        self.first_msg_ts: Optional[float] = None
        # 用于外部请求线程停止
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def run(self):
        # 根据 duration_s 生成截止时间；0 表示不设定截止
        deadline = (time.perf_counter() + self.duration_s) if self.duration_s > 0 else None
        
        while not self.stop_event.is_set():
            if deadline and time.perf_counter() >= deadline:
                break
            
            try:
                # 从总线拉取一条消息；遇到错误帧时抛异常并停止
                ok, data, msg = self.transmitter._receive_can_data(
                    target_id=self.filter_id,
                    timeout=0.1,
                    is_ext_frame=None,
                    canfd_mode=self.fd_only,
                    stop_on_error=True,
                    return_msg=True,
                )
            except RuntimeError as e:
                self.stats['error'] += 1
                print(f"‼️ 检测到总线错误帧，停止接收: {e}")
                self.stop()
                break

            if ok and msg:
                # 成功接收后做模式/ID 过滤并统计
                if self._accept_msg(msg):
                    self.stats['total'] += 1
                    if msg.is_fd:
                        self.stats['fd'] += 1
                    else:
                        self.stats['can'] += 1
                    if self.print_each:
                        self._print_msg(msg)

                # 达到指定条数提前结束
                if self.max_count and self.stats['total'] >= self.max_count:
                    break
        
        # 结束后打印统计与错误提示
        print(f"总计接收: {self.stats['total']} 条 (CAN: {self.stats['can']}, FD: {self.stats['fd']})")
        if self.stats['error'] > 0:
            print(f"检测到 {self.stats['error']} 个总线错误，已停止。")

    def _accept_msg(self, msg: can.Message) -> bool:
        # 按 ID 与 CAN/FD 模式过滤
        if self.filter_id is not None and msg.arbitration_id != self.filter_id:
            return False
        if self.fd_only and not msg.is_fd:
            return False
        if self.can_only and msg.is_fd:
            return False
        return True

    def _print_msg(self, msg: can.Message):
        # 格式化时间戳/ID/数据并输出一行
        ts = msg.timestamp
        if self.first_msg_ts is None:
            self.first_msg_ts = ts
        
        t_str = self._format_time(ts, self.first_msg_ts if self.time_mode == 'rel' else None, self.time_mode)
        id_hex = f"0x{msg.arbitration_id:03X}" if not msg.is_extended_id else f"0x{msg.arbitration_id:08X}"
        data_hex = ' '.join(f"{b:02X}" for b in msg.data)
        type_str = 'REMOTE' if msg.is_remote_frame else 'DATA'
        
        print(f"RX {t_str} BUS={self.bus_type_label} TYPE={type_str} ID={id_hex} LEN={msg.dlc} DATA={data_hex}")

    def _format_time(self, ts: float, base_ts: Optional[float], mode: str) -> str:
        # 支持绝对时间或相对首条消息的时间
        if mode == 'abs':
            ms = int((ts - int(ts)) * 1000)
            return time.strftime('%H:%M:%S', time.localtime(ts)) + f'.{ms:03d}'
        delta = ts - (base_ts if base_ts is not None else ts)
        return f"{delta:.6f}s"


def main():
    # 命令行入口：解析参数、按模式/速率组合依次测试
    parser = argparse.ArgumentParser(description="使用 TZCANTransmitter 接收 CAN/CAN FD 数据，并能检测总线错误")
    parser.add_argument("--iface", default="can0", help="socketcan 接口名")
    parser.add_argument("--mode", choices=["all", "can", "fd"], default="all", help="接收模式")
    parser.add_argument("--duration", type=float, default=0.0, help="接收时长（秒）；0 表示持续接收直至按键退出")
    parser.add_argument("--count", type=int, default=0, help="达到指定数量后提前结束")
    parser.add_argument("--filter-id", type=lambda x: int(x, 0), help="仅接收指定 ID")
    parser.add_argument("--time-mode", choices=["rel", "abs"], default="rel", help="时间戳打印格式")
    parser.add_argument("--quiet", action="store_true", help="不逐条打印消息")
    parser.add_argument("--can-br", nargs='+', help="CAN 2.0 速率 (e.g., 500k, 1m)")
    parser.add_argument("--fd-dbr", nargs='+', help="CAN FD 数据域速率 (e.g., 2m, 5m)")
    parser.add_argument("--fd-arb", default="500k", help="CAN FD 仲裁域速率")
    parser.add_argument("--sp", type=float, help="仲裁段采样点 (例如 0.8=80%% 或 80 表示 80%%)")
    parser.add_argument("--dsp", type=float, help="数据段采样点 (例如 0.8=80%% 或 80 表示 80%%)")
    args = parser.parse_args()

    print(parser.format_help())
    print("指南：")
    print("- CAN2.0：建议设置 `--can-br` 如 500k 1m")
    print("- CAN FD：建议设置 `--fd-arb` 与 `--fd-dbr`，例如 `--fd-arb 500k --fd-dbr 2m 5m`")
    print("- 默认接收时长为 0，持续接收，按 ESC 退出")

    # 先校验依赖
    ensure_python_can()
    
    # 解析接口通道号与默认速率列表
    iface_channel = int(args.iface.replace("can", ""))
    can_bitrates = [500000, 1000000] if not args.can_br else [parse_bitrate_token(v) for v in args.can_br]
    fd_data_bitrates = [2000000, 5000000] if not args.fd_dbr else [parse_bitrate_token(v) for v in args.fd_dbr]
    fd_arb_bitrate = parse_bitrate_token(args.fd_arb)

    def run_test(is_fd: bool, bitrate: int, dbitrate: Optional[int] = None):
        # 单次测试：配置接口、打开设备、启动接收线程并监听 ESC
        label = "CAN FD" if is_fd else "CAN 2.0"
        print(f"\n[{label}] 配置 {args.iface} @ " + (f"arb={bitrate}, data={dbitrate}" if is_fd else f"br={bitrate}"))
        
        configure_socketcan(
            args.iface,
            bitrate=bitrate,
            fd=is_fd,
            data_bitrate=dbitrate,
            sample_point=args.sp,
            dsample_point=args.dsp
        )

        # 选择并初始化 TZCAN 设备
        TX = CANMessageTransmitter.choose_can_device("TZCAN")
        # 通过 socketcan 打开指定通道；fd 指示是否进入 CAN FD 模式
        m_dev, _, _ = TX.init_can_device(channels=[iface_channel], backend='socketcan', fd=is_fd)
        # 获取总线句柄
        bus_handle = m_dev['buses'].get(iface_channel)
        
        if not bus_handle:
            print(f"❌ 初始化 {args.iface} 失败，请检查配置。")
            TX.close_can_device(m_dev)
            return

        transmitter = TX(bus_handle)
        
        # 清空过滤器，接收所有消息（驱动支持时也会收到错误帧）
        transmitter.bus.set_filters([])

        receiver = ReceiverThread(
            transmitter,
            duration_s=args.duration,
            max_count=args.count if args.count > 0 else None,
            filter_id=args.filter_id,
            fd_only=is_fd,
            can_only=not is_fd,
            print_each=not args.quiet,
            time_mode=args.time_mode,
            bus_type_label=label,
        )

        print(f"开始接收... 按 ESC 退出。")
        receiver.start()
        
        # 终端按键监听：按 ESC 立即停止接收（POSIX 终端）；Windows 可能不支持 termios
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while receiver.is_alive():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    if sys.stdin.read(1) == '\x1b':
                        print("ESC 按下，正在停止...")
                        receiver.stop()
                        break
                receiver.join(0.1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        receiver.join()
        TX.close_can_device(m_dev)

    if args.mode in ("all", "can"):
        for br in can_bitrates:
            run_test(is_fd=False, bitrate=br)

    if args.mode in ("all", "fd"):
        for dbr in fd_data_bitrates:
            run_test(is_fd=True, bitrate=fd_arb_bitrate, dbitrate=dbr)

if __name__ == "__main__":
    main()