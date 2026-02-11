# 说明：TZCAN 发送速率测试脚本，基于 python-can/socketcan
# 用途：配置 CAN/CAN FD 速率与频率，在真实总线或内核 loopback 下发送并统计
# 注意：socketcan 的 ip link 命令仅适用于 Linux/WSL；Windows 下请使用设备驱动提供的配置方式
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import argparse
import time
from typing import Optional, List

try:
    import can
except ImportError:
    can = None

try:
    from can_bridge.CANMessageTransmitter import CANMessageTransmitter
    # 动态加载 TZCAN 后端
    TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
except Exception:
    # 兼容直接运行脚本的场景（同目录下存在 CANMessageTransmitter.py）
    try:
        from CANMessageTransmitter import CANMessageTransmitter
        TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
    except ImportError:
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


def configure_socketcan(interface: str, bitrate: Optional[int] = None, sample_point: Optional[float] = None, fd: bool = False, data_bitrate: Optional[int] = None, dsample_point: Optional[float] = None, loopback: Optional[bool] = None, txqueuelen: Optional[int] = None):
    # 利用 ip link 配置 socketcan 接口；优先 sudo，失败回退不带 sudo
    # 参数说明：
    # - bitrate: 仲裁域速率（CAN2.0/FD）
    # - data_bitrate: 数据域速率（仅 FD）
    # - sample_point/dsample_point: 采样点（0-1 或百分比），dsample_point 仅 FD
    # - loopback: 是否开启内核 loopback，用于本机吞吐测试
    # - txqueuelen: 发送队列长度（提高高频率发送的吞吐）
    if fd:
        if bitrate is None or data_bitrate is None:
            raise ValueError("FD 模式需要同时提供仲裁域 bitrate 和数据域 data_bitrate")
        sp_cmd = f"sample-point {sample_point}" if sample_point else ""
        dsp_cmd = f"dsample-point {dsample_point}" if dsample_point else ""
        lb_cmd = "loopback on" if (loopback is None or loopback) else "loopback off"
        cmd_sudo = (
            f"sudo ip link set {interface} down; "
            f"sudo ip link set {interface} type can bitrate {bitrate} {sp_cmd} dbitrate {data_bitrate} {dsp_cmd} fd on {lb_cmd}; "
            f"sudo ip link set {interface} up" + (f"; sudo ip link set {interface} txqueuelen {txqueuelen}" if txqueuelen else "")
        )
        cmd_nosudo = (
            f"ip link set {interface} down; "
            f"ip link set {interface} type can bitrate {bitrate} {sp_cmd} dbitrate {data_bitrate} {dsp_cmd} fd on {lb_cmd}; "
            f"ip link set {interface} up" + (f"; ip link set {interface} txqueuelen {txqueuelen}" if txqueuelen else "")
        )
    else:
        if bitrate is None:
            raise ValueError("标准 CAN 模式需要提供 bitrate")
        sp_cmd = f"sample-point {sample_point}" if sample_point else ""
        lb_cmd = "loopback on" if (loopback is None or loopback) else "loopback off"
        cmd_sudo = (
            f"sudo ip link set {interface} down; "
            f"sudo ip link set {interface} type can bitrate {bitrate} {sp_cmd} {lb_cmd}; "
            f"sudo ip link set {interface} up" + (f"; sudo ip link set {interface} txqueuelen {txqueuelen}" if txqueuelen else "")
        )
        cmd_nosudo = (
            f"ip link set {interface} down; "
            f"ip link set {interface} type can bitrate {bitrate} {sp_cmd} {lb_cmd}; "
            f"ip link set {interface} up" + (f"; ip link set {interface} txqueuelen {txqueuelen}" if txqueuelen else "")
        )

    rc = os.system(cmd_sudo)
    if rc != 0:
        _ = os.system(cmd_nosudo)


def main():
    # 命令行入口：配置速率与发送参数，分 CAN/FD 模式进行测试
    parser = argparse.ArgumentParser(description="使用 TZCANTransmitter 在 socketcan 上进行发送速率测试")
    parser.add_argument("--iface", default="can0", help="socketcan 接口名，例如 can0")
    parser.add_argument("--mode", choices=["all", "can", "fd"], default="all", help="测试模式：全部/仅CAN/仅FD")
    parser.add_argument("--count", type=int, default=100000, help="每种配置的发送帧数")
    parser.add_argument("--freq", type=float, default=1000.0, help="发送频率 Hz")
    parser.add_argument("--fd_len", type=int, default=16, help="FD 帧数据长度（字节），例如 16/32/64")
    parser.add_argument("--no-brs", action="store_true", help="FD 模式下关闭 BRS（数据域不切到高速）")
    parser.add_argument("--sp", type=float, help="仲裁段采样点 (0-1.0)")
    parser.add_argument("--dsp", type=float, help="数据段采样点 (0-1.0)，仅 FD 模式有效")
    parser.add_argument("--can-br", nargs='+', help="选择 CAN2.0 速率，例如: 500k 或 1m，可多选")
    parser.add_argument("--fd-dbr", nargs='+', help="选择 CAN FD 数据域速率，例如: 1m 5m 8m 10m，可多选")
    parser.add_argument("--fd-arb", default=None, help="CAN FD 仲裁域速率（必填），例如 500k/1m 或精确数值")
    parser.add_argument("--no-loopback", action="store_true", help="关闭内核 loopback，仅做真实总线测量")
    parser.add_argument("--txqlen", type=int, help="设置 socketcan 接口的 txqueuelen")
    args = parser.parse_args()

    print(parser.format_help())
    print("指南：")
    print("- CAN2.0：必须设置 `--can-br` 如 500k 1m")
    print("- CAN FD：必须设置 `--fd-arb` 与 `--fd-dbr`，例如 `--fd-arb 500k --fd-dbr 5m 8m`")
    print("- 示例：")
    print("  python new_test_tzcan_send.py --mode can --iface can0 --can-br 500k 1m")
    print("  python new_test_tzcan_send.py --mode fd --iface can0 --fd-arb 500k --fd-dbr 5m 8m --fd-len 64")

    # 根据模式校验必需参数是否给出
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
    if missing:
        print("未设置必要参数：" + ", ".join(missing))
        print("请设置参数后重新运行脚本进行发送")
        return

    # 先校验依赖
    ensure_python_can()

    # 解析接口通道号并选择设备
    iface_channel = int(str(args.iface).replace("can", ""))
    # 使用已加载的 TZCANTransmitter 类
    TX = TZCANTransmitter

    if need_can:
        # 按给定的 CAN2.0 速率逐项进行发送测试
        for bitrate_str in args.can_br:
            bitrate = parse_bitrate_token(bitrate_str)
            configure_socketcan(
                args.iface,
                bitrate=bitrate,
                sample_point=args.sp,
                fd=False,
                loopback=(not args.no_loopback),
                txqueuelen=args.txqlen,
            )
            m_dev, _, _ = TX.init_can_device(channels=[iface_channel], backend='socketcan', fd=False)
            try:
                bus = m_dev['buses'][iface_channel]
                tx = TX(bus)
                # 通过 period 做简单的时间调度以达到目标频率
                period = 1.0 / args.freq
                # 基础负载 8 字节；实际发送数据首 4 字节包含递增计数 i
                payload = bytes((i % 256 for i in range(8)))
                start = time.perf_counter()
                next_t = start
                for i in range(args.count):
                    # 构造数据：低位在前的 4 字节计数 + 固定负载后 4 字节
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
        # 按给定的 FD 数据域速率逐项进行发送测试
        fd_arb_bitrate = parse_bitrate_token(args.fd_arb)
        for dbitrate_str in args.fd_dbr:
            dbitrate = parse_bitrate_token(dbitrate_str)
            configure_socketcan(
                args.iface,
                bitrate=fd_arb_bitrate,
                sample_point=args.sp,
                fd=True,
                data_bitrate=dbitrate,
                dsample_point=args.dsp,
                loopback=(not args.no_loopback),
                txqueuelen=args.txqlen,
            )
            m_dev, _, _ = TX.init_can_device(channels=[iface_channel], backend='socketcan', fd=True)
            try:
                bus = m_dev['buses'][iface_channel]
                tx = TX(bus)
                # 通过 period 做简单的时间调度以达到目标频率
                period = 1.0 / args.freq
                # FD 负载长度可配置；默认把 i 计数前 4 字节放到首部
                payload = bytes((i % 256 for i in range(args.fd_len)))
                # BRS（Bit Rate Switching）：数据域是否切到高速
                brs_flag = 0 if args.no_brs else 1
                start = time.perf_counter()
                next_t = start
                for i in range(args.count):
                    # 构造数据：低位在前的 4 字节计数 + 固定负载剩余部分
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