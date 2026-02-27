"""VESC ESC 集成测试：显式选择硬件层 + 协议层
用法：
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode receive
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode rpm --rpm 2000 --duration 3

  # 指定 socketcan 后端（Linux）
  python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --backend socketcan --mode receive
"""
import argparse
import sys
import os
import time

# ── 物理连接层 ──────────────────────────────────────────────────────────────
try:
    from can_bridge import CANMessageTransmitter
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from can_bridge import CANMessageTransmitter

# ── 协议层 ──────────────────────────────────────────────────────────────────
from can_bridge.protocols.vesc import VESC_CAN, VESC_CAN_STATUS


def parse_bitrate_token(s):
    s = s.lower().strip()
    if s.endswith('k'): return int(float(s[:-1]) * 1000)
    if s.endswith('m'): return int(float(s[:-1]) * 1_000_000)
    return int(s)


def _run_receive(vesc, args):
    t0 = time.time()
    while args.duration == 0 or time.time() - t0 < args.duration:
        arb_id, pack = vesc.receive_decode(timeout=0.1, can_channel=args.iface)
        if pack is not None:
            print(f"[{time.time()-t0:5.2f}s] VESC#{pack.id:2d} "
                  f"RPM={pack.rpm:7.0f}  I={pack.current:6.2f}A  "
                  f"POS={pack.pid_pos_now:7.1f}°  VIN={pack.input_voltage:5.1f}V")


def _run_rpm(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_rpm(args.vesc_id, args.rpm, can_channel=args.iface)
        _, pack = vesc.receive_decode(timeout=0.05, can_channel=args.iface)
        fb = f"RPM={pack.rpm:.0f}" if pack else "no feedback"
        print(f"[{time.time()-t0:5.2f}s] send RPM={args.rpm:.0f}  {fb}")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_current(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_current(args.vesc_id, args.current, can_channel=args.iface)
        _, pack = vesc.receive_decode(timeout=0.05, can_channel=args.iface)
        fb = f"I={pack.current:.2f}A" if pack else "no feedback"
        print(f"[{time.time()-t0:5.2f}s] send current={args.current:.1f}A  {fb}")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_pos(vesc, args):
    t0, interval = time.time(), 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        vesc.send_pos(args.vesc_id, args.pos, can_channel=args.iface)
        _, pack = vesc.receive_decode(timeout=0.05, can_channel=args.iface)
        fb = f"POS={pack.pid_pos_now:.1f}°" if pack else "no feedback"
        print(f"[{time.time()-t0:5.2f}s] send pos={args.pos:.1f}°  {fb}")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def main():
    parser = argparse.ArgumentParser(description="VESC CAN 集成测试")
    parser.add_argument('--iface',    type=int,   default=0,        help='CAN 通道号')
    parser.add_argument('--can-br',              default='500k',   help='CAN 波特率')
    parser.add_argument('--backend',             default='socketcan', help='CAN 后端（socketcan/candle/gs_usb）')
    parser.add_argument('--vesc-id',  type=int,   default=1,        help='VESC 设备 CAN ID')
    parser.add_argument('--mode', choices=['receive', 'rpm', 'current', 'pos'], default='receive')
    parser.add_argument('--rpm',      type=float, default=1000.0)
    parser.add_argument('--current',  type=float, default=5.0,     help='目标电流 (A)')
    parser.add_argument('--pos',      type=float, default=0.0,     help='目标位置 (deg)')
    parser.add_argument('--duration', type=float, default=5.0,     help='运行时长 s，0=持续')
    parser.add_argument('--freq',     type=float, default=10.0,    help='发送频率 Hz')
    args = parser.parse_args()

    baud = parse_bitrate_token(args.can_br)

    # ── 步骤 1：一步完成设备选择 + 物理连接初始化 ───────────────────────
    TX, m_dev, _, _ = CANMessageTransmitter.open(
        "TZCAN", baud_rate=baud, channels=[args.iface], backend=args.backend
    )
    # 按实际通道号从 buses 字典取 Bus，避免 iface>1 时 ch0/ch1 为 None 的问题
    bus = m_dev["buses"].get(args.iface)
    if bus is None:
        print(f"❌ 通道 {args.iface} 未能成功打开")
        TX.close_can_device(m_dev)
        return

    # ── 步骤 2：注入协议层（硬件与协议解耦）────────────────────────────
    vesc = VESC_CAN(TX(bus), channels=(args.iface,))

    # ── 步骤 3：运行 ─────────────────────────────────────────────────────
    try:
        {'receive': _run_receive, 'rpm': _run_rpm,
         'current': _run_current, 'pos': _run_pos}[args.mode](vesc, args)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        TX.close_can_device(m_dev)


if __name__ == '__main__':
    main()
