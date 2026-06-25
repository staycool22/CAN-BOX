"""DM 电机集成测试：显式选择硬件层 + 协议层
用法：
  python3 tests/test_dmmotor.py --device TZUSB2CAN --iface 0 --can-br 500k --backend socketcan --mode refresh
  python3 tests/test_dmmotor.py --device TZUSB2CAN --iface 0 --can-br 500k --backend candle --mode enable
  python3 tests/test_dmmotor.py --device TZUSB2CAN --iface 0 --can-br 500k --backend socketcan --mode mit --kp 50 --kd 0.3 --q 0 --dq 0 --tau 0 --duration 3
  python3 tests/test_dmmotor.py --device TZUSB2CAN --iface 0 --can-br 500k --backend socketcan --mode pos_vel --pos 1.0 --vel 2.0 --duration 3
  python3 tests/test_dmmotor.py --device TZETHCAN --iface 0 --can-br 500k --mode refresh --target-ip 192.168.100.11
  python3 tests/test_dmmotor.py --device TZETHCAN --iface 0 --can-br 500k --mode read_param --rid PMAX --target-ip 192.168.100.11
"""
import argparse
import os
import sys
import time

try:
    from tzcan import CANMessageTransmitter
except ImportError:
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from tzcan import CANMessageTransmitter

from tzcan.protocols.dm_motor_protocol import (
    Control_Type,
    DM_MOTOR,
    DM_Motor_Type,
    DM_variable,
    Motor,
)


def parse_bitrate_token(s):
    s = str(s).lower().strip()
    if s.endswith("k"):
        return int(float(s[:-1]) * 1000)
    if s.endswith("m"):
        return int(float(s[:-1]) * 1_000_000)
    return int(s)


def parse_motor_type(name):
    key = str(name).strip()
    try:
        return DM_Motor_Type[key]
    except KeyError as exc:
        raise argparse.ArgumentTypeError(f"不支持的电机类型: {name}") from exc


def parse_control_mode(name):
    key = str(name).strip()
    try:
        return Control_Type[key]
    except KeyError as exc:
        raise argparse.ArgumentTypeError(f"不支持的控制模式: {name}") from exc


def parse_rid(name):
    key = str(name).strip()
    try:
        return DM_variable[key]
    except KeyError as exc:
        raise argparse.ArgumentTypeError(f"不支持的寄存器名: {name}") from exc


def parse_device(name):
    key = str(name).strip().upper()
    if key not in {"TZUSB2CAN", "TZETHCAN"}:
        raise argparse.ArgumentTypeError(f"不支持的设备类型: {name}")
    return key


def _print_motor_state(motor, prefix="state"):
    print(
        f"{prefix}: POS={motor.getPosition():.6f} "
        f"VEL={motor.getVelocity():.6f} "
        f"TOR={motor.getTorque():.6f} "
        f"ERR={motor.getError()}"
    )


def _run_refresh(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.refresh_motor_status(motor)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] refresh")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_mit(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.controlMIT(motor, args.kp, args.kd, args.q, args.dq, args.tau)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] mit")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_pos_vel(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.control_Pos_Vel(motor, args.pos, args.vel)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] pos_vel")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_vel(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.control_Vel(motor, args.vel)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] vel")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_pos_force(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.control_pos_force(motor, args.pos, args.vel_u16, args.i_des)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] pos_force")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_pos_vel_csp(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.control_Pos_Vel_CSP(motor, args.pos, args.vel)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] pos_vel_csp")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_vel_csp(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.control_Vel_CSP(motor, args.vel)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] vel_csp")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _run_tor_csp(dm, motor, args):
    t0 = time.time()
    interval = 1.0 / args.freq
    while args.duration == 0 or time.time() - t0 < args.duration:
        loop_t = time.time()
        dm.control_Tor_CSP(motor, args.tor)
        _print_motor_state(motor, prefix=f"[{time.time()-t0:5.2f}s] tor_csp")
        time.sleep(max(0, interval - (time.time() - loop_t)))


def _create_protocol_transmitter(device, tx_cls, bus, iface, fd_enabled):
    if device == "TZETHCAN":
        return tx_cls(bus, channel_id=iface, is_canfd=bool(fd_enabled))
    return tx_cls(bus)


def _mode_to_control_type(mode):
    mapping = {
        "mit": Control_Type.MIT,
        "pos_vel": Control_Type.POS_VEL,
        "vel": Control_Type.VEL,
        "pos_force": Control_Type.Torque_Pos,
        "pos_vel_csp": Control_Type.POS_VEL_CSP,
        "vel_csp": Control_Type.VEL_CSP,
        "tor_csp": Control_Type.Torque_CSP,
    }
    return mapping.get(mode)


def _prepare_motor_for_mode(dm, motor, mode, prepare_delay):
    control_mode = _mode_to_control_type(mode)
    if control_mode is None:
        return

    print(f"prepare: disable -> switch_mode({control_mode.name}) -> enable")
    dm.disable(motor)
    time.sleep(prepare_delay)
    ok = dm.switchControlMode(motor, control_mode)
    print(f"prepare: switchControlMode({control_mode.name}) -> {ok}")
    if not ok:
        print("⚠️ 模式切换返回 False，请确认电机固件和总线反馈是否正常")
    time.sleep(prepare_delay)
    dm.enable(motor)
    print(f"prepare: enable sent, slave_id=0x{motor.SlaveID:X}")
    time.sleep(prepare_delay)


def main():
    parser = argparse.ArgumentParser(description="DM 电机 CAN 集成测试")
    parser.add_argument("--device", type=parse_device, default="TZUSB2CAN", help="设备类型：TZUSB2CAN 或 TZETHCAN")
    parser.add_argument("--iface", type=int, default=0, help="CAN 通道号")
    parser.add_argument("--can-br", default="500k", help="仲裁段波特率")
    parser.add_argument("--backend", default="socketcan", help="TZUSB2CAN 后端（socketcan/candle/gs_usb）")
    parser.add_argument("--fd", action="store_true", help="启用 CAN FD 打开总线")
    parser.add_argument("--fd-dbr", default="2m", help="CAN FD 数据段波特率（仅 --fd 时生效）")
    parser.add_argument("--target-ip", default=None, help="TZETHCAN 目标 IP，例如 192.168.100.11")
    parser.add_argument("--target-port", type=int, default=None, help="TZETHCAN 当前通道配置端口，可选")
    parser.add_argument("--motor-type", type=parse_motor_type, default=DM_Motor_Type.DM4310, help="电机类型，如 DM4310")
    parser.add_argument("--slave-id", type=lambda x: int(x, 0), default=0x01, help="电机 SlaveID，如 0x01")
    parser.add_argument("--master-id", type=lambda x: int(x, 0), default=0x11, help="电机 MasterID，如 0x11")
    parser.add_argument("--mode", choices=[
        "enable", "enable_old", "disable", "zero", "refresh",
        "mit", "pos_vel", "vel", "pos_force",
        "pos_vel_csp", "vel_csp", "tor_csp",
        "switch_mode", "save_param", "read_param", "write_param",
    ], default="refresh")
    parser.add_argument("--duration", type=float, default=3.0, help="循环模式运行时长 s，0=持续")
    parser.add_argument("--freq", type=float, default=10.0, help="循环模式发送频率 Hz")
    parser.add_argument("--kp", type=float, default=50.0, help="MIT 模式 kp")
    parser.add_argument("--kd", type=float, default=0.3, help="MIT 模式 kd")
    parser.add_argument("--q", type=float, default=0.0, help="MIT 模式目标位置")
    parser.add_argument("--dq", type=float, default=0.0, help="MIT 模式目标速度")
    parser.add_argument("--tau", type=float, default=0.0, help="MIT 模式目标力矩")
    parser.add_argument("--pos", type=float, default=0.0, help="位置命令")
    parser.add_argument("--vel", type=float, default=0.0, help="速度命令")
    parser.add_argument("--tor", type=float, default=0.0, help="力矩 CSP 命令")
    parser.add_argument("--vel-u16", type=int, default=1000, help="力位混合模式速度整型值")
    parser.add_argument("--i-des", type=int, default=100, help="力位混合模式电流整型值")
    parser.add_argument("--control-mode", type=parse_control_mode, default=Control_Type.MIT, help="模式切换/旧固件使能模式")
    parser.add_argument("--prepare-delay", type=float, default=0.1, help="自动前置流程中各步骤之间的延迟秒数")
    parser.add_argument("--rid", type=parse_rid, default=DM_variable.PMAX, help="寄存器名，如 PMAX")
    parser.add_argument("--value", type=float, default=0.0, help="写寄存器时的目标值")
    args = parser.parse_args()

    if args.device == "TZUSB2CAN" and args.fd and args.backend.lower() == "gs_usb":
        print("❌ gs_usb 后端不支持 CAN FD，请改用 socketcan 或 candle")
        return

    open_kw = dict(
        baud_rate=parse_bitrate_token(args.can_br),
        channels=[args.iface],
        fd=bool(args.fd),
    )
    if args.fd:
        open_kw["dbit_baud_rate"] = parse_bitrate_token(args.fd_dbr)
    if args.device == "TZUSB2CAN":
        open_kw["backend"] = args.backend
    else:
        if args.target_ip is not None:
            open_kw["target_ip"] = args.target_ip
        if args.target_port is not None:
            open_kw["channel_configs"] = {
                args.iface: {"target_ip": args.target_ip, "target_port": args.target_port}
            }

    TX, m_dev, _, _ = CANMessageTransmitter.open(args.device, **open_kw)
    bus = m_dev["buses"].get(args.iface)
    if bus is None:
        print(f"❌ 通道 {args.iface} 未能成功打开")
        TX.close_can_device(m_dev)
        return

    dm = DM_MOTOR(_create_protocol_transmitter(args.device, TX, bus, args.iface, args.fd))
    motor = Motor(args.motor_type, args.slave_id, args.master_id)
    dm.addMotor(motor)

    try:
        if args.mode == "enable":
            dm.enable(motor)
            print(f"enable sent: slave_id=0x{motor.SlaveID:X}")
            _print_motor_state(motor)
        elif args.mode == "enable_old":
            dm.enable_old(motor, args.control_mode)
            print(f"enable_old sent: slave_id=0x{motor.SlaveID:X}, mode={args.control_mode.name}")
            _print_motor_state(motor)
        elif args.mode == "disable":
            dm.disable(motor)
            print(f"disable sent: slave_id=0x{motor.SlaveID:X}")
        elif args.mode == "zero":
            dm.set_zero_position(motor)
            print(f"zero command sent: slave_id=0x{motor.SlaveID:X}")
            _print_motor_state(motor)
        elif args.mode == "refresh":
            _run_refresh(dm, motor, args)
        elif args.mode == "mit":
            _prepare_motor_for_mode(dm, motor, args.mode, args.prepare_delay)
            _run_mit(dm, motor, args)
        elif args.mode == "pos_vel":
            _prepare_motor_for_mode(dm, motor, args.mode, args.prepare_delay)
            _run_pos_vel(dm, motor, args)
        elif args.mode == "vel":
            _prepare_motor_for_mode(dm, motor, args.mode, args.prepare_delay)
            _run_vel(dm, motor, args)
        elif args.mode == "pos_force":
            _prepare_motor_for_mode(dm, motor, args.mode, args.prepare_delay)
            _run_pos_force(dm, motor, args)
        elif args.mode == "pos_vel_csp":
            _prepare_motor_for_mode(dm, motor, args.mode, args.prepare_delay)
            _run_pos_vel_csp(dm, motor, args)
        elif args.mode == "vel_csp":
            _prepare_motor_for_mode(dm, motor, args.mode, args.prepare_delay)
            _run_vel_csp(dm, motor, args)
        elif args.mode == "tor_csp":
            _prepare_motor_for_mode(dm, motor, args.mode, args.prepare_delay)
            _run_tor_csp(dm, motor, args)
        elif args.mode == "switch_mode":
            ok = dm.switchControlMode(motor, args.control_mode)
            print(f"switchControlMode({args.control_mode.name}) -> {ok}")
        elif args.mode == "save_param":
            dm.save_motor_param(motor)
            print("save_motor_param sent")
        elif args.mode == "read_param":
            value = dm.read_motor_param(motor, args.rid)
            print(f"read_motor_param({args.rid.name}) -> {value}")
        elif args.mode == "write_param":
            ok = dm.change_motor_param(motor, args.rid, args.value)
            print(f"change_motor_param({args.rid.name}, {args.value}) -> {ok}")
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        try:
            dm.disable(motor)
            print(f"cleanup: disable sent, slave_id=0x{motor.SlaveID:X}")
        except Exception as exc:
            print(f"cleanup: disable failed: {exc}")
        TX.close_can_device(m_dev)


if __name__ == "__main__":
    main()
