import time
import threading
import argparse
from typing import Optional, Tuple, List

from CAN.TZCANTransmitter import TZCANTransmitter
from .can_vesc import VESC


class TransmitterAdapter:
    def __init__(self, transmitter: TZCANTransmitter, use_canfd: bool = True):
        self.tx = transmitter
        self.use_canfd = use_canfd

    def send(self, id: int, data: List[int]) -> None:
        self.tx._send_can_data(
            send_id=id,
            data_list=data,
            is_ext_frame=True,
            canfd_mode=self.use_canfd,
            brs=1 if self.use_canfd else 0,
            esi=0,
        )

    def receive(self, timeout: float) -> Tuple[Optional[int], Optional[List[int]]]:
        result = self.tx._receive_can_data(
            target_id=None,
            timeout=timeout,
            is_ext_frame=None,
            canfd_mode=self.use_canfd,
            stop_on_error=False,
            return_msg=True,
        )
        if isinstance(result, tuple) and len(result) == 3:
            ok, data, msg = result
        elif isinstance(result, tuple) and len(result) == 2:
            ok, data = result
            msg = None
        else:
            return None, None
        if not ok or msg is None:
            return None, None
        return msg.arbitration_id, data


def build(
    baud_rate: int = 500000,
    data_bitrate: int = 2000000,
    channels: List[int] = [0],
    backend: Optional[str] = None,
    sp: Optional[float] = None,
    dsp: Optional[float] = None,
    use_canfd: bool = True,
):
    m_dev, ch0, ch1 = TZCANTransmitter.init_can_device(
        baud_rate=baud_rate,
        dbit_baud_rate=data_bitrate,
        channels=channels,
        can_type=0,
        canfd_standard=0,
        backend=backend,
        fd=use_canfd,
        sp=sp,
        dsp=dsp,
    )
    bus = ch0 if ch0 is not None else ch1
    if bus is None and isinstance(m_dev, dict) and "buses" in m_dev and len(m_dev["buses"]) > 0:
        bus = list(m_dev["buses"].values())[0]
    tx = TZCANTransmitter(bus)
    adapter = TransmitterAdapter(tx, use_canfd=use_canfd)
    vesc = VESC(adapter)
    return m_dev, bus, vesc


def parse_id_token(value: str) -> int:
    s = str(value).strip().lower()
    if s.startswith("0x"):
        return int(s, 16)
    return int(s)


def main():
    parser = argparse.ArgumentParser(description="VESC CAN/CAN-FD 控制与监控")
    parser.add_argument("--baud-rate", type=int, default=500000)
    parser.add_argument("--data-bitrate", type=int, default=1000000)
    parser.add_argument("--channels", type=int, nargs="+", default=[1])
    parser.add_argument("--backend", type=str, default="candle")
    parser.add_argument("--sp", type=float, default=75.0)
    parser.add_argument("--dsp", type=float, default=80.0)
    parser.add_argument("--use-canfd", dest="use_canfd", action="store_true", default=True)
    parser.add_argument("--no-canfd", dest="use_canfd", action="store_false")
    parser.add_argument("--vesc-id", type=parse_id_token, default=0x2D)
    parser.add_argument("--target-rpm", type=int, default=5000)
    parser.add_argument("--target-duty", type=float, default=0.1)
    parser.add_argument("--mode", type=str, choices=['rpm', 'duty'], default='rpm')
    parser.add_argument("--target-freq", type=float, default=200.0)
    parser.add_argument("--stats-ids", type=parse_id_token, nargs="+", default=[0x92D, 0x922])
    parser.add_argument("--duration", type=int, default=10)
    args = parser.parse_args()

    m_dev, bus, vesc = build(
        baud_rate=args.baud_rate,
        data_bitrate=args.data_bitrate,
        channels=args.channels,
        backend=args.backend,
        sp=args.sp,
        dsp=args.dsp,
        use_canfd=args.use_canfd,
    )
    vesc_id = args.vesc_id
    target_rpm = args.target_rpm
    target_duty = args.target_duty
    target_freq = args.target_freq

    stats = {
        "sent_total": 0,
        "received_total": 0,
        "send_times": [],
    }
    for can_id in args.stats_ids:
        stats[can_id] = {"count": 0, "times": []}

    stop_event = threading.Event()

    def receiver_loop():
        reached = False
        while not stop_event.is_set():
            id_val, packet = vesc.receive_decode(timeout=0.01)
            if id_val is not None:
                stats["received_total"] += 1
                if id_val in stats:
                    stats[id_val]["count"] += 1
                    stats[id_val]["times"].append(time.time())

                if packet is not None:
                    rpm_now = int(packet.rpm)
                    if not reached and abs(rpm_now - target_rpm) <= 50:
                        reached = True
                        print("RPM 达标", rpm_now)
                    else:
                        print("RPM", rpm_now)
            # time.sleep(0.01)

    t = threading.Thread(target=receiver_loop, daemon=True)
    t.start()

    actual_duration = 0
    try:
        print(f"--- Starting CAN sender with target frequency: {target_freq} Hz ---")
        start_time = time.perf_counter()
        next_send_time = start_time
        send_interval = 1 / target_freq

        while time.perf_counter() - start_time < args.duration:
            # 使用忙等待确保发送时间点
            while time.perf_counter() < next_send_time:
                time.sleep(0)

            # 记录发送函数耗时
            send_start_ts = time.perf_counter()
            if args.mode == 'rpm':
                vesc.send_rpm(vesc_id, target_rpm)
            elif args.mode == 'duty':
                vesc.send_duty(vesc_id, target_duty)
            send_end_ts = time.perf_counter()

            stats["sent_total"] += 1
            stats["send_times"].append(send_end_ts - send_start_ts)

            # 计算下一次发送的时间
            next_send_time += send_interval

    except KeyboardInterrupt:
        pass
    finally:
        # 确保在计算最终统计数据之前停止循环
        if "start_time" in locals():
            actual_duration = time.perf_counter() - start_time

        stop_event.set()
        t.join(timeout=1.0)
        TZCANTransmitter.close_can_device(m_dev, bus)

        print("\n--- CAN Message Statistics ---")
        print(f"Sent Total: {stats['sent_total']}")
        print(f"Received Total: {stats['received_total']}")
        for can_id, data in stats.items():
            if isinstance(data, dict):
                print(f"ID: {hex(can_id)}")
                print(f"  Count: {data['count']}")
                if data["times"]:
                    print(f"  Last 5 timestamps: {data['times'][-5:]}")
        print("--------------------------")

        print("\n--- Performance Statistics ---")
        if stats["sent_total"] > 0 and actual_duration > 0:
            actual_freq = stats["sent_total"] / actual_duration
            avg_send_time_ms = (sum(stats["send_times"]) / stats["sent_total"]) * 1000
            print(f"Control Mode: {args.mode.upper()}")
            if args.mode == 'rpm':
                print(f"Target RPM: {target_rpm}")
            elif args.mode == 'duty':
                print(f"Target Duty: {target_duty}")
            print(f"Target Send Frequency: {target_freq} Hz")
            print(f"Actual Average Frequency: {actual_freq:.2f} Hz")
            print(f"Average send time: {avg_send_time_ms:.4f} ms")
        print(f"Total Run Duration: {actual_duration:.2f} seconds")
        print("----------------------------")


if __name__ == "__main__":
    main()