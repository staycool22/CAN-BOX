import argparse
import glob
import os
import platform
import shutil
import socket
import subprocess
import sys
from typing import List, Sequence, Tuple


DEFAULT_REMOTE_IP = "192.168.100.10"
DEFAULT_PORT_BASE = 20000
DEFAULT_TIMEOUT_US = 20000
DEFAULT_CHANNELS = [0, 1, 2, 3]
EXIT_OK = 0
EXIT_INVALID_USAGE = 1
EXIT_LINK_FAILURE = 2
DEFAULT_PING_TIMEOUT_S = 1


class LinkCheckError(RuntimeError):
    pass


def ensure_linux() -> None:
    if platform.system() != "Linux":
        raise RuntimeError(
            f"ethcan_tool.py 仅支持 Linux/WSL；当前平台为 {platform.system()}。"
        )


def ensure_cannelloni_available(binary: str) -> None:
    if shutil.which(binary) is None:
        raise RuntimeError(
            f"未找到 '{binary}'，请先安装 cannelloni 并确保其在 PATH 中。"
        )


def ping_remote_ip(
    remote_ip: str,
    timeout_s: int = DEFAULT_PING_TIMEOUT_S,
    dry_run: bool = False,
) -> Tuple[bool, str]:
    ping_binary = shutil.which("ping")
    if ping_binary is None:
        return False, "未找到 ping 命令"

    command = [ping_binary, "-c", "1", "-W", str(int(timeout_s)), remote_ip]
    print(f"Checking remote device reachability: {' '.join(command)}")
    if dry_run:
        return True, "dry-run"

    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        return True, "reachable"

    detail = (result.stderr or result.stdout).strip()
    if not detail:
        detail = f"ping exited with code {result.returncode}"
    return False, detail


def run_command(
    command: Sequence[str],
    check: bool = True,
    dry_run: bool = False,
) -> subprocess.CompletedProcess:
    print(f"Executing: {' '.join(command)}")
    if dry_run:
        return subprocess.CompletedProcess(command, 0, "", "")
    return subprocess.run(command, check=check, capture_output=True, text=True)


def interface_exists(iface: str) -> bool:
    return os.path.exists(f"/sys/class/net/{iface}")


def discover_vcan_interfaces() -> List[str]:
    interfaces = [os.path.basename(path) for path in glob.glob("/sys/class/net/vcan*")]
    interfaces.sort(key=lambda name: int(name.replace("vcan", "")))
    return interfaces


def setup_vcan_interfaces(channels: Sequence[int], dry_run: bool = False) -> None:
    print("\n--- Setting up vcan interfaces ---")
    run_command(["sudo", "modprobe", "vcan"], dry_run=dry_run)
    for ch in channels:
        iface = f"vcan{ch}"
        if not interface_exists(iface):
            print(f"Creating {iface}...")
            run_command(["sudo", "ip", "link", "add", "dev", iface, "type", "vcan"], dry_run=dry_run)
        run_command(["sudo", "ip", "link", "set", iface, "up"], dry_run=dry_run)
        print(f"{iface} is UP")
    print("--- vcan setup complete ---\n")


def shutdown_vcan_interfaces(channels: Sequence[int], dry_run: bool = False) -> None:
    print("\n--- Shutting down vcan interfaces ---")
    for ch in channels:
        iface = f"vcan{ch}"
        if not interface_exists(iface):
            print(f"Skipping {iface}: interface not found")
            continue
        run_command(["sudo", "ip", "link", "set", iface, "down"], check=False, dry_run=dry_run)
        print(f"{iface} is DOWN")
    print("--- vcan shutdown complete ---\n")


def stop_existing_cannelloni(binary: str, dry_run: bool = False) -> None:
    print("\n--- Stopping existing cannelloni processes ---")
    process_name = os.path.basename(binary) or "cannelloni"
    result = run_command(["sudo", "killall", process_name], check=False, dry_run=dry_run)
    if result.returncode == 0:
        print("Stopped existing cannelloni processes.")
    else:
        print("No existing cannelloni process found.")
    print("--- cannelloni cleanup complete ---\n")


def build_cannelloni_command(
    iface: str,
    remote_ip: str,
    local_port: int,
    remote_port: int,
    timeout_us: int,
    binary: str,
    sort_by_sequence: bool,
) -> List[str]:
    command = [
        binary,
        "-I",
        iface,
        "-R",
        remote_ip,
        "-r",
        str(remote_port),
        "-l",
        str(local_port),
        "-t",
        str(timeout_us),
    ]
    if sort_by_sequence:
        command.append("-S")
    return command


def start_cannelloni_instances(
    channels: Sequence[int],
    remote_ip: str,
    local_port_base: int,
    remote_port_base: int,
    timeout_us: int,
    binary: str,
    sort_by_sequence: bool,
    log_dir: str = "",
    dry_run: bool = False,
) -> None:
    print("\n--- Starting cannelloni bridge instances ---")
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
    for ch in channels:
        iface = f"vcan{ch}"
        local_port = int(local_port_base) + int(ch)
        remote_port = int(remote_port_base) + int(ch)
        command = build_cannelloni_command(
            iface=iface,
            remote_ip=remote_ip,
            local_port=local_port,
            remote_port=remote_port,
            timeout_us=timeout_us,
            binary=binary,
            sort_by_sequence=sort_by_sequence,
        )
        print(f"Running: {' '.join(command)}")
        if dry_run:
            print(
                f"Dry-run bridge: {iface} <-> {remote_ip}:{remote_port} "
                f"(local={local_port})"
            )
            continue
        stdout_target = subprocess.DEVNULL
        stderr_target = subprocess.DEVNULL
        log_fd = None
        log_path = ""
        if log_dir:
            log_path = os.path.join(log_dir, f"cannelloni_ch{ch}.log")
            log_fd = os.open(log_path, os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o644)
            stdout_target = log_fd
            stderr_target = log_fd
        try:
            proc = subprocess.Popen(
                command,
                stdout=stdout_target,
                stderr=stderr_target,
                start_new_session=True,
            )
        finally:
            if log_fd is not None:
                os.close(log_fd)
        log_hint = f", log={log_path}" if log_path else ""
        print(
            f"Started bridge: {iface} <-> {remote_ip}:{remote_port} "
            f"(local={local_port}, pid={proc.pid}{log_hint})"
        )
    print("--- cannelloni startup complete ---\n")


def is_udp_port_available(port: int) -> Tuple[bool, str]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", int(port)))
        return True, "available"
    except OSError as exc:
        return False, str(exc)
    finally:
        sock.close()


def has_running_cannelloni(binary: str) -> Tuple[bool, List[str]]:
    process_name = os.path.basename(binary) or "cannelloni"
    result = subprocess.run(
        ["pgrep", "-af", process_name],
        check=False,
        capture_output=True,
        text=True,
    )
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    return bool(lines), lines


def health_check_ethcan_environment(
    channels: Sequence[int],
    remote_ip: str,
    local_port_base: int,
    binary: str,
    check_vcan: bool,
    check_process: bool,
    check_ports: bool,
    check_remote_ip: bool,
    ping_timeout_s: int,
) -> bool:
    print("\n--- ETHCAN health check ---")
    checks_ok = True

    print(f"Platform: {platform.system()}")
    if platform.system() != "Linux":
        print("FAIL: 当前平台不是 Linux/WSL")
        checks_ok = False
    else:
        print("PASS: Linux/WSL 环境可用")

    binary_path = shutil.which(binary)
    if binary_path is None:
        print(f"FAIL: 未找到 cannelloni binary: {binary}")
        checks_ok = False
    else:
        print(f"PASS: cannelloni binary = {binary_path}")

    if check_remote_ip:
        reachable, detail = ping_remote_ip(remote_ip, timeout_s=ping_timeout_s)
        if reachable:
            print(f"PASS: remote ip {remote_ip} 可达")
        else:
            print(f"FAIL: remote ip {remote_ip} 不可达 ({detail})")
            checks_ok = False

    if check_vcan:
        for ch in channels:
            iface = f"vcan{ch}"
            if interface_exists(iface):
                print(f"PASS: {iface} 存在")
            else:
                print(f"FAIL: {iface} 不存在")
                checks_ok = False

    if check_ports:
        for ch in channels:
            port = int(local_port_base) + int(ch)
            available, detail = is_udp_port_available(port)
            if available:
                print(f"PASS: UDP 端口 {port} 可用")
            else:
                print(f"FAIL: UDP 端口 {port} 不可用 ({detail})")
                checks_ok = False

    if check_process:
        running, lines = has_running_cannelloni(binary)
        if running:
            print("PASS: 检测到 cannelloni 进程")
            for line in lines:
                print(f"  - {line}")
        else:
            print("INFO: 未检测到 cannelloni 进程")

    print(f"--- health check {'PASS' if checks_ok else 'FAIL'} ---\n")
    return checks_ok


def discover_ethcan_environment(cannelloni_binary: str) -> None:
    print("\n--- Discovering ETHCAN Environment ---")
    print(f"Platform: {platform.system()}")
    print(f"cannelloni binary: {shutil.which(cannelloni_binary) or 'not found'}")

    interfaces = discover_vcan_interfaces()
    if interfaces:
        print("\nFound vcan interfaces:")
        for iface in interfaces:
            print(f"  - {iface}")
    else:
        print("\nNo vcan interfaces found.")

    result = subprocess.run(
        ["pgrep", "-af", cannelloni_binary],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0 and result.stdout.strip():
        print("\nRunning cannelloni processes:")
        for line in result.stdout.strip().splitlines():
            print(f"  - {line}")
    else:
        print("\nNo running cannelloni process found.")
    print("\nDefault parameters:")
    print(f"  - remote_ip={DEFAULT_REMOTE_IP}")
    print(f"  - port_base={DEFAULT_PORT_BASE}")
    print(f"  - timeout_us={DEFAULT_TIMEOUT_US}")
    print(f"  - channels={DEFAULT_CHANNELS}")
    print("--- ETHCAN discovery complete ---\n")


def validate_channels(channels: Sequence[int]) -> List[int]:
    cleaned = sorted({int(ch) for ch in channels})
    if any(ch < 0 for ch in cleaned):
        raise ValueError(f"通道号必须 >= 0，当前输入: {channels}")
    return cleaned


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ETHCAN 桥接工具：创建 vcan 接口并管理 cannelloni 进程",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python ethcan_tool.py --discover
  python ethcan_tool.py --setup
  python ethcan_tool.py --setup --remote-ip 192.168.100.11 --timeout-us 20000
  python ethcan_tool.py --setup --channels 0 1 --port-base 21000
  python ethcan_tool.py --health-check --check-remote-ip --remote-ip 192.168.100.10
  python ethcan_tool.py --shutdown
  python ethcan_tool.py --shutdown --bring-down-vcan --channels 0 1
        """,
    )
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--discover", action="store_true", help="显示当前 vcan/cannelloni 状态")
    actions.add_argument("--setup", action="store_true", help="创建 vcan 并启动 cannelloni")
    actions.add_argument("--shutdown", action="store_true", help="停止 cannelloni，可选关闭 vcan")
    actions.add_argument("--health-check", action="store_true", help="执行环境健康检查")

    parser.add_argument(
        "--channels",
        nargs="+",
        type=int,
        default=list(DEFAULT_CHANNELS),
        help="通道索引列表，例如: 0 1 2 3（默认: 0 1 2 3）",
    )
    parser.add_argument(
        "--remote-ip",
        default=DEFAULT_REMOTE_IP,
        help=f"HPM 板卡 IP（默认: {DEFAULT_REMOTE_IP}）",
    )
    parser.add_argument(
        "--port-base",
        type=int,
        default=DEFAULT_PORT_BASE,
        help=f"本地/远端端口基址，实际端口=base+channel（默认: {DEFAULT_PORT_BASE}）",
    )
    parser.add_argument(
        "--local-port-base",
        type=int,
        default=None,
        help="本地监听端口基址；默认跟随 --port-base",
    )
    parser.add_argument(
        "--remote-port-base",
        type=int,
        default=None,
        help="远端目标端口基址；默认跟随 --port-base",
    )
    parser.add_argument(
        "--timeout-us",
        type=int,
        default=DEFAULT_TIMEOUT_US,
        help=f"cannelloni flush 超时（微秒，默认: {DEFAULT_TIMEOUT_US}）",
    )
    parser.add_argument(
        "--ping-timeout-s",
        type=int,
        default=DEFAULT_PING_TIMEOUT_S,
        help=f"ping 检测超时（秒，默认: {DEFAULT_PING_TIMEOUT_S}）",
    )
    parser.add_argument(
        "--sort",
        action="store_true",
        help="启动 cannelloni 时加 -S，按序列号排序 UDP 接收帧",
    )
    parser.add_argument(
        "--bring-down-vcan",
        action="store_true",
        help="在 --shutdown 时同时执行 ip link set vcanX down",
    )
    parser.add_argument(
        "--cannelloni-bin",
        default="cannelloni",
        help="cannelloni 可执行文件名或绝对路径（默认: cannelloni）",
    )
    parser.add_argument(
        "--log-dir",
        default="",
        help="setup 时为每个通道输出 cannelloni 日志目录",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="仅打印将要执行的动作，不实际修改系统环境",
    )
    parser.add_argument(
        "--check-ports",
        action="store_true",
        help="health-check 时检查本地 UDP 端口占用",
    )
    parser.add_argument(
        "--check-process",
        action="store_true",
        help="health-check 时检查 cannelloni 进程状态",
    )
    parser.add_argument(
        "--check-vcan",
        action="store_true",
        help="health-check 时检查 vcan 接口是否存在",
    )
    parser.add_argument(
        "--check-remote-ip",
        action="store_true",
        help="health-check 时检查 remote ip 是否可达",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    try:
        ensure_linux()
        channels = validate_channels(args.channels)

        local_port_base = (
            int(args.local_port_base)
            if args.local_port_base is not None
            else int(args.port_base)
        )
        remote_port_base = (
            int(args.remote_port_base)
            if args.remote_port_base is not None
            else int(args.port_base)
        )

        if args.timeout_us <= 0:
            raise ValueError("--timeout-us 必须 > 0")
        if args.ping_timeout_s <= 0:
            raise ValueError("--ping-timeout-s 必须 > 0")

        if args.discover:
            discover_ethcan_environment(args.cannelloni_bin)
            return

        if args.health_check:
            requested_checks = (
                args.check_ports
                or args.check_process
                or args.check_vcan
                or args.check_remote_ip
            )
            checks_ok = health_check_ethcan_environment(
                channels=channels,
                remote_ip=args.remote_ip,
                local_port_base=local_port_base,
                binary=args.cannelloni_bin,
                check_vcan=args.check_vcan or not requested_checks,
                check_process=args.check_process or not requested_checks,
                check_ports=args.check_ports or not requested_checks,
                check_remote_ip=args.check_remote_ip or not requested_checks,
                ping_timeout_s=args.ping_timeout_s,
            )
            sys.exit(EXIT_OK if checks_ok else EXIT_LINK_FAILURE)

        if args.setup:
            ensure_cannelloni_available(args.cannelloni_bin)
            reachable, detail = ping_remote_ip(
                args.remote_ip,
                timeout_s=args.ping_timeout_s,
                dry_run=args.dry_run,
            )
            if not reachable:
                raise LinkCheckError(
                    f"remote ip {args.remote_ip} 不可达，硬件设备检测失败 ({detail})"
                )
            print(f"PASS: remote ip {args.remote_ip} 连通，继续启动 ETHCAN 桥接")
            setup_vcan_interfaces(channels, dry_run=args.dry_run)
            stop_existing_cannelloni(args.cannelloni_bin, dry_run=args.dry_run)
            start_cannelloni_instances(
                channels=channels,
                remote_ip=args.remote_ip,
                local_port_base=local_port_base,
                remote_port_base=remote_port_base,
                timeout_us=args.timeout_us,
                binary=args.cannelloni_bin,
                sort_by_sequence=args.sort,
                log_dir=args.log_dir,
                dry_run=args.dry_run,
            )
            return

        if args.shutdown:
            stop_existing_cannelloni(args.cannelloni_bin, dry_run=args.dry_run)
            if args.bring_down_vcan:
                shutdown_vcan_interfaces(channels, dry_run=args.dry_run)
            return

        parser.print_help()
        print("\nRunning a default discovery...")
        discover_ethcan_environment(args.cannelloni_bin)
    except LinkCheckError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(EXIT_LINK_FAILURE)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(EXIT_INVALID_USAGE)


if __name__ == "__main__":
    main()
