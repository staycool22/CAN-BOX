"""VESC ESC 集成测试：显式选择硬件层 + 协议层
用法：
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode receive
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode rpm --rpm 2000 --duration 3
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode vel_cur --vel 2000 --current 3.0 --duration 3
  python3 tests/test_tzcan_vesc.py --iface 2 --can-br 500k --vesc-id 1 --backend socketcan --mode receive

CAN FD（仲裁段与数据段速率；socketcan 需先用 ip link 配置 fd on，参见 CLAUDE.md）：
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --fd --fd-dbr 2m --vesc-id 1 --mode receive

多电机（同一总线，靠 vesc_id 区分）：
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode rpm --rpm 2000
  # 另开一个终端，--vesc-id 2，同一 VESC_CAN 实例，不同 id

多总线（不同通道，各自创建 VESC_CAN 实例）：
  # 见本脚本 main() 注释

完整参数说明与 CAN FD 前置条件见：docs/test_tzcan_vesc.md
"""
import argparse
import sys
import os
import time

# ── 物理连接层 ──────────────────────────────────────────────────────────────
try:
    from tzcan import CANMessageTransmitter
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from tzcan import CANMessageTransmitter

# ── 协议层 ──────────────────────────────────────────────────────────────────
from tzcan.protocols.vesc import VESC_CAN, VESC_CAN_STATUS


def parse_bitrate_token(s):
    s = s.lower().strip()
    if s.endswith('k'): return int(float(s[:-1]) * 1000)
    if s.endswith('m'): return int(float(s[:-1]) * 1_000_000)
    return int(s)


def _run_receive(vesc, args):
    t0 = time.time()
    while args.duration == 0 or time.time() - t0 < args.duration:
        arb_id, pack = vesc.receive_decode(timeout=0.1)
        if pack is not None:
            print(f"[{time.time()-t0:5.2f}s] VESC#{pack.id:2d} "
                  f"RPM={pack.rpm:7.0f}  I={pack.current:6.2f}A  "
                  f"POS={pack.pid_pos_now:7.1f}°  VIN={pack.input_voltage:5.1f}V")


def _run_rpm(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_rpm(args.vesc_id, args.rpm)
        _, pack = vesc.receive_decode(timeout=0.05)
        fb = f"RPM={pack.rpm:.0f}" if pack else "no feedback"
        print(f"[{time.time()-t0:5.2f}s] send RPM={args.rpm:.0f}  {fb}")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_current(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_current(args.vesc_id, args.current)
        _, pack = vesc.receive_decode(timeout=0.05)
        fb = f"I={pack.current:.2f}A" if pack else "no feedback"
        print(f"[{time.time()-t0:5.2f}s] send current={args.current:.1f}A  {fb}")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_pos(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_pos(args.vesc_id, args.pos)
        _, pack = vesc.receive_decode(timeout=0.05)
        fb = f"POS={pack.pid_pos_now:.1f}°" if pack else "no feedback"
        print(f"[{time.time()-t0:5.2f}s] send pos={args.pos:.1f}°  {fb}")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_vel_cur(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_vel_cur(args.vesc_id, args.current, args.vel)
        _, pack = vesc.receive_decode(timeout=0.05)
        fb = f"RPM={pack.rpm:.0f} I={pack.current:.2f}A" if pack else "no feedback"
        print(
            f"[{time.time()-t0:5.2f}s] send vel={args.vel:.0f} current={args.current:.2f}A  {fb}"
        )
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_pass_through(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_pass_through(args.vesc_id, args.pos, args.rpm, args.current)
        _, pack = vesc.receive_decode(timeout=0.05)
        if pack.id == args.vesc_id:
            print(f"[{time.time()-t0:5.2f}s] receive pack: {pack.id}, {pack.pid_pos_now:.1f}°, {pack.rpm:.0f}, {pack.current:.2f}A")
        time.sleep(max(0, interval - (time.time() - loop_t)))

def main():
    parser = argparse.ArgumentParser(description="VESC CAN 集成测试")
    parser.add_argument('--iface',    type=int,   default=0,           help='CAN 通道号')
    parser.add_argument('--can-br',              default='500k',      help='仲裁段波特率（经典 CAN 或 CAN FD 仲裁段）')
    parser.add_argument('--fd', action='store_true', help='启用 CAN FD（需接口与后端支持；gs_usb 不支持 FD）')
    parser.add_argument('--fd-dbr',              default='2m',        help='CAN FD 数据段波特率，如 2m / 4m（仅 --fd 时生效）')
    parser.add_argument('--backend',             default='socketcan', help='CAN 后端（socketcan/candle/gs_usb）')
    parser.add_argument('--vesc-id',  type=int,   default=1,           help='VESC 设备 CAN ID')
    parser.add_argument('--mode', choices=['receive', 'rpm', 'current', 'pos', 'vel_cur', 'pass_through'], default='receive')
    parser.add_argument('--rpm',      type=float, default=1000.0,     help='目标转速 (ERPM)')
    parser.add_argument('--vel',      type=float, default=1000.0,     help='速度电流模式目标速度 (ERPM)')
    parser.add_argument('--current',  type=float, default=0.0,        help='目标电流 (A)')
    parser.add_argument('--pos',      type=float, default=0.0,        help='目标位置 (deg)')
    parser.add_argument('--duration', type=float, default=5.0,        help='运行时长 s，0=持续')
    parser.add_argument('--freq',     type=float, default=10.0,       help='发送频率 Hz')
    args = parser.parse_args()

    baud = parse_bitrate_token(args.can_br)
    if args.fd and args.backend.lower() == 'gs_usb':
        print('❌ gs_usb 后端不支持 CAN FD，请改用 socketcan 或 candle')
        return

    open_kw = dict(
        baud_rate=baud,
        channels=[args.iface],
        backend=args.backend,
        fd=bool(args.fd),
    )
    if args.fd:
        open_kw['dbit_baud_rate'] = parse_bitrate_token(args.fd_dbr)

    # ── 步骤 1：初始化物理连接 ───────────────────────────────────────────────
    TX, m_dev, _, _ = CANMessageTransmitter.open("TZUSB2CAN", **open_kw)
    bus = m_dev["buses"].get(args.iface)
    if bus is None:
        print(f"❌ 通道 {args.iface} 未能成功打开")
        TX.close_can_device(m_dev)
        return

    # ── 步骤 2：创建协议实例（一个实例 = 一条总线）──────────────────────────
    #
    #   同一总线上多个 VESC：用同一个 vesc 实例，调用时传不同 vesc_id。
    #   不同总线上的 VESC：各自创建独立的 VESC_CAN 实例，例如：
    #       TX, m_dev, _, _ = CANMessageTransmitter.open("TZUSB2CAN",
    #           baud_rate=baud, channels=[0, 1])
    #       vesc0 = VESC_CAN(TX(m_dev["buses"][0]))
    #       vesc1 = VESC_CAN(TX(m_dev["buses"][1]))
    #
    vesc = VESC_CAN(TX(bus))

    # ── 步骤 3：运行 ─────────────────────────────────────────────────────────
    try:
        {'receive': _run_receive, 'rpm': _run_rpm,
         'current': _run_current, 'pos': _run_pos, 'vel_cur': _run_vel_cur,
         'pass_through': _run_pass_through}[args.mode](vesc, args)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        TX.close_can_device(m_dev)


if __name__ == '__main__':
    main()
