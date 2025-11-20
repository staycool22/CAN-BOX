import time
import threading
from typing import Optional, Tuple, List

from TZCANTransmitter import TZCANTransmitter
from can_vesc import VESC


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


def main(run_duration: int = 10):
    m_dev, bus, vesc = build(
        baud_rate=500000,
        data_bitrate=5000000,
        channels=[1],
        backend="candle",
        sp=75.0,
        dsp=50.0,
        use_canfd=True,
    )
    vesc_id = 0x2D
    target_rpm = 5000
    target_freq = 200  # <--- 设置目标频率 (Hz)

    stats = {
        "sent_total": 0,
        "received_total": 0,
        0x92D: {"count": 0, "times": []},
        0x922: {"count": 0, "times": []},
        "send_times": [],
    }

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

        while time.perf_counter() - start_time < run_duration:
            # 使用忙等待确保发送时间点
            while time.perf_counter() < next_send_time:
                pass

            # 记录发送函数耗时
            send_start_ts = time.perf_counter()
            vesc.send_rpm(vesc_id, target_rpm)
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
            print(f"Target Frequency: {target_freq} Hz")
            print(f"Actual Average Frequency: {actual_freq:.2f} Hz")
            print(f"Average send_rpm() time: {avg_send_time_ms:.4f} ms")
        print(f"Total Run Duration: {actual_duration:.2f} seconds")
        print("----------------------------")


if __name__ == "__main__":
    main(run_duration=10)