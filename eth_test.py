#!/usr/bin/env python3
import argparse
import socket
import time
import struct
import threading
import selectors
import math
from typing import Optional


DEST_IP = "192.168.100.189"
DEST_PORT = 8080
SRC_IP = "192.168.100.189"


def parse_hex_payload(hex_str: str) -> bytes:
    s = "".join(hex_str.split()).lower()
    if s.startswith("0x"):
        s = s[2:]
    if len(s) % 2 != 0:
        raise ValueError("hex 字符串长度必须为偶数（每字节 2 个 hex 字符）")
    data = bytes.fromhex(s)
    if len(data) != 64:
        raise ValueError(f"payload 必须刚好 64 字节，当前 {len(data)} 字节")
    return data


def default_payload() -> bytes:
    return bytes(range(64))


def busy_wait_until(t_deadline: float) -> None:
    while True:
        now = time.perf_counter()
        if now >= t_deadline:
            return


def receiver_loop(bind_ip: str, port: int, stop_event: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    except OSError:
        print("Warning: Failed to set SO_RCVBUF")

    try:
        sock.bind((bind_ip, port))
        sock.settimeout(1.0)
    except OSError as e:
        print(f"[Receiver] Error binding to {bind_ip}:{port}: {e}")
        return

    print(f"[Receiver] Listening on {bind_ip}:{port} (Buffer: 4MB)")

    count = 0
    last_seq = -1
    lost = 0
    
    start_time = time.time()
    last_stats_time = start_time

    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(2048)
            count += 1
            
            if len(data) >= 8:
                seq = struct.unpack('!Q', data[:8])[0]
                if last_seq != -1:
                    diff = seq - last_seq
                    if diff > 1:
                        lost += (diff - 1)
                last_seq = seq
            
            now = time.time()
            if now - last_stats_time >= 1.0:
                print(f"[Receiver] Received={count} Lost={lost} (Loss Rate: {lost/(count+lost)*100:.2f}%)")
                last_stats_time = now

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Receiver] Error: {e}")
            break
    
    sock.close()
    print(f"[Receiver] Finished. Total Received={count}, Total Lost={lost}")


def parse_ports_csv(ports_csv: str) -> list[int]:
    ports: list[int] = []
    for part in ports_csv.split(","):
        s = part.strip()
        if not s:
            continue
        port = int(s, 10)
        if port < 1 or port > 65535:
            raise ValueError(f"端口不合法: {port}")
        ports.append(port)
    if not ports:
        raise ValueError("端口列表不能为空")
    return ports


def receiver_multi_loop(
    bind_ip: str,
    ports: list[int],
    stats_interval: float,
    stop_event: Optional[threading.Event] = None,
    recv_state: Optional[dict[str, int]] = None,
    recv_lock: Optional[threading.Lock] = None,
) -> None:
    sel = selectors.DefaultSelector()
    socks: dict[int, socket.socket] = {}
    stats: dict[int, dict[str, int]] = {}
    for port in ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        except OSError:
            pass
        sock.bind((bind_ip, port))
        sock.setblocking(False)
        sel.register(sock, selectors.EVENT_READ, port)
        socks[port] = sock
        stats[port] = {"count": 0, "lost": 0, "last_seq": -1}

    ports_str = ",".join(str(p) for p in ports)
    print(f"[Receiver] Listening on {bind_ip} ports: {ports_str} (Ctrl+C stop)")

    t0 = time.time()
    t_last = t0
    try:
        while True:
            if stop_event is not None and stop_event.is_set():
                break
            events = sel.select(timeout=0.5)
            for key, _ in events:
                port = key.data
                sock = key.fileobj
                try:
                    data, _ = sock.recvfrom(65535)
                except BlockingIOError:
                    continue

                st = stats[port]
                st["count"] += 1
                if len(data) >= 8:
                    seq = struct.unpack("!Q", data[:8])[0]
                    last_seq = st["last_seq"]
                    if last_seq != -1:
                        diff = seq - last_seq
                        if diff > 1:
                            st["lost"] += diff - 1
                    st["last_seq"] = seq
                if recv_state is not None and recv_lock is not None:
                    with recv_lock:
                        recv_state["total"] += 1

            now = time.time()
            if stats_interval > 0 and now - t_last >= stats_interval:
                total_recv = sum(stats[p]["count"] for p in ports)
                total_lost = sum(stats[p]["lost"] for p in ports)
                elapsed = now - t0
                rate = total_recv / elapsed if elapsed > 0 else 0.0
                loss_rate = (total_lost / (total_recv + total_lost) * 100.0) if (total_recv + total_lost) > 0 else 0.0
                per_port = " ".join(f"{p}:{stats[p]['count']}" for p in ports)
                print(f"[Receiver] total={total_recv} lost={total_lost} loss={loss_rate:.2f}% rate={rate:.1f}pps | {per_port}")
                t_last = now
    except KeyboardInterrupt:
        pass
    finally:
        sel.close()
        for sock in socks.values():
            try:
                sock.close()
            except OSError:
                pass

    total_recv = sum(stats[p]["count"] for p in ports)
    total_lost = sum(stats[p]["lost"] for p in ports)
    loss_rate = (total_lost / (total_recv + total_lost) * 100.0) if (total_recv + total_lost) > 0 else 0.0
    print("[Receiver] Summary:")
    for p in ports:
        print(f"  port={p} received={stats[p]['count']} lost={stats[p]['lost']}")
    print(f"  total_received={total_recv} total_lost={total_lost} loss_rate={loss_rate:.2f}%")


def main() -> int:
    ap = argparse.ArgumentParser(description="Send 64-byte UDP payload at specific rate.")
    ap.add_argument("--dst-ip", default=DEST_IP)
    ap.add_argument("--dst-port", type=int, default=DEST_PORT)
    ap.add_argument("--src-ip", default=SRC_IP)
    ap.add_argument("--src-port", type=int, default=0, help="发送端绑定端口；0 表示随机端口")
    ap.add_argument("--rate", type=float, default=3000.0, help="Hz (default: 3000)")
    ap.add_argument("--count", type=int, default=0, help="发送包的总数量，0 表示不限制")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0 means run forever")
    ap.add_argument("--hex", dest="hex_payload", default=None, help="64字节 payload 的 hex 字符串")
    ap.add_argument("--stats-interval", type=float, default=1.0, help="print stats every N seconds")
    ap.add_argument("--loopback", action="store_true", help="同机回环：后台启动接收端，并向 dst-ip:dst-port 发送")
    ap.add_argument("--server", action="store_true", help="Run in server (receiver) mode only")
    ap.add_argument("--multi-server", action="store_true", help="多端口接收模式（仅接收）")
    ap.add_argument("--recv-while-send", action="store_true", help="发送时后台接收")
    ap.add_argument("--recv-ip", default=SRC_IP, help="接收端绑定 IP（默认本机地址）")
    ap.add_argument("--recv-ports", default="8000,8001,8002,8003", help="接收端口列表，用逗号分隔")
    ap.add_argument("--recv-count", type=int, default=0, help="接收达到指定数量后退出（仅发送时后台接收）")
    ap.add_argument("--recv-timeout", type=float, default=0.0, help="等待接收数量的超时秒数，0 表示不超时")
    
    args = ap.parse_args()

    if args.rate <= 0:
        raise ValueError("--rate 必须 > 0")

    if args.multi_server:
        ports = parse_ports_csv(args.recv_ports)
        receiver_multi_loop(args.recv_ip, ports, args.stats_interval)
        return 0

    if args.server:
        stop_event = threading.Event()
        try:
            receiver_loop("0.0.0.0", args.dst_port, stop_event)
        except KeyboardInterrupt:
            stop_event.set()
        return 0

    receiver_thread = None
    stop_event = threading.Event()
    recv_stop_event = None
    recv_state = None
    recv_lock = None

    if args.loopback:
        print(f"Starting Loopback Test on {args.dst_ip}:{args.dst_port} (sender bind {args.src_ip}:{args.src_port or 0})")
        
        receiver_thread = threading.Thread(
            target=receiver_loop,
            args=(args.dst_ip, args.dst_port, stop_event),
            daemon=True
        )
        receiver_thread.start()
        time.sleep(0.5)

    if args.recv_while_send:
        recv_ports = parse_ports_csv(args.recv_ports)
        recv_stop_event = threading.Event()
        recv_state = {"total": 0}
        recv_lock = threading.Lock()
        receiver_thread = threading.Thread(
            target=receiver_multi_loop,
            args=(args.recv_ip, recv_ports, args.stats_interval, recv_stop_event, recv_state, recv_lock),
            daemon=True,
        )
        receiver_thread.start()
        time.sleep(0.5)

    # Prepare payload base
    base_payload = parse_hex_payload(args.hex_payload) if args.hex_payload else default_payload()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.src_ip, args.src_port))
    except OSError as e:
        print(f"Error binding sender to {args.src_ip}: {e}")
        if receiver_thread:
            stop_event.set()
        return 1

    period = 1.0 / args.rate
    t0 = time.perf_counter()
    next_t = t0
    end_t: Optional[float] = None if args.duration <= 0 else (t0 + args.duration)

    sent = 0
    sent_last = 0
    t_stats = t0 + args.stats_interval

    # Stats for verification
    jitter_min = float('inf')
    jitter_max = float('-inf')
    jitter_sum = 0.0
    jitter_sq_sum = 0.0

    try:
        while True:
            now = time.perf_counter()
            if end_t is not None and now >= end_t:
                break
            if args.count > 0 and sent >= args.count:
                break

            # Verify schedule (Jitter calculation)
            ideal_time = t0 + sent * period
            # now is current time, but we want time at send moment.
            # Using 'now' from top of loop is close enough for iteration start
            jitter = now - ideal_time
            if jitter < jitter_min: jitter_min = jitter
            if jitter > jitter_max: jitter_max = jitter
            jitter_sum += jitter
            jitter_sq_sum += jitter * jitter

            next_t += period
            
            payload = struct.pack('!Q', sent) + base_payload[8:]
            
            try:
                bytes_sent = sock.sendto(payload, (args.dst_ip, args.dst_port))
                if bytes_sent != len(payload):
                     print(f"[Sender] Warning: Partial send! Sent {bytes_sent} bytes, expected {len(payload)}")
                sent += 1
            except OSError as e:
                if hasattr(e, 'winerror') and e.winerror == 10054:
                    pass
                else:
                    print(f"Send error: {e}")

            now = time.perf_counter()
            if now < next_t:
                busy_wait_until(next_t)

            if args.stats_interval > 0 and now >= t_stats:
                elapsed = now - (t_stats - args.stats_interval)
                inst_rate = (sent - sent_last) / elapsed if elapsed > 0 else 0.0
                total_elapsed = now - t0
                avg_rate = sent / total_elapsed if total_elapsed > 0 else 0.0
                print(f"[Sender] sent={sent} inst_rate={inst_rate:.1f}Hz avg_rate={avg_rate:.1f}Hz")
                sent_last = sent
                t_stats = now + args.stats_interval
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if receiver_thread:
            if args.recv_while_send and recv_state is not None and recv_lock is not None:
                t_wait_start = time.time()
                while True:
                    with recv_lock:
                        total_recv = recv_state["total"]
                    if args.recv_count > 0 and total_recv >= args.recv_count:
                        break
                    if args.recv_timeout > 0 and time.time() - t_wait_start >= args.recv_timeout:
                        break
                    if args.recv_count <= 0 and args.recv_timeout <= 0:
                        break
                    time.sleep(0.05)
            time.sleep(0.2)
            if recv_stop_event is not None:
                recv_stop_event.set()
            else:
                stop_event.set()
            receiver_thread.join(timeout=1.0)

    total_elapsed = time.perf_counter() - t0
    avg_rate = sent / total_elapsed if total_elapsed > 0 else 0.0
    
    avg_jitter = jitter_sum / sent if sent > 0 else 0.0
    variance = (jitter_sq_sum / sent) - (avg_jitter * avg_jitter) if sent > 0 else 0.0
    std_dev = math.sqrt(max(0.0, variance))

    print(f"[Sender] Summary:")
    print(f"  Sent={sent} (Target: {args.count if args.count > 0 else 'Unlimited'})")
    print(f"  Rate: Target={args.rate}Hz, Actual_Avg={avg_rate:.1f}Hz")
    print(f"  Time: Elapsed={total_elapsed:.3f}s")
    print(f"  Jitter: min={jitter_min*1000:.3f}ms max={jitter_max*1000:.3f}ms avg={avg_jitter*1000:.3f}ms std={std_dev*1000:.3f}ms")
    
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
