import time
import sys
import os

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from typing import List, Optional, Tuple

from CAN.TZCANTransmitter import TZCANTransmitter
from VESC_test.can_vesc import VESC


class TransmitterAdapter:
    """
    将 TZCANTransmitter 适配为 can_vesc.py 中 VESC 类所需的接口。
    """
    def __init__(self, transmitter: TZCANTransmitter, use_canfd: bool = True):
        self.tx = transmitter
        self.use_canfd = use_canfd

    def send(self, id: int, data: List[int]) -> None:
        self.tx._send_can_data(
            send_id=id,
            data_list=data,
            is_ext_frame=True,  # VESC 使用扩展帧
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


def main():
    # --- 固定参数 ---
    baud_rate = 500000
    data_bitrate = 1000000
    channels = [0]
    backend = "candle"
    sp = 75.0
    dsp = 80.0
    use_canfd = False  # 使用 CAN 2.0
    vesc_id = 0x22     # VESC ID

    # --- 初始化CAN总线 ---
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

    if bus is None:
        print("错误：无法初始化CAN总线。请检查设备连接和驱动。")
        return

    # --- 创建VESC控制器实例 ---
    tx = TZCANTransmitter(bus)
    adapter = TransmitterAdapter(tx, use_canfd=use_canfd)
    vesc = VESC(adapter)

    try:
        print("--- 开始占空比正反转测试 ---")
        print(f"使用 VESC ID: {vesc_id} (十进制), {hex(vesc_id)} (十六进制)")
        print(f"使用 CAN 2.0, 波特率: {baud_rate}")
        while True:
            print(f"发送占空比: 0.2")
            t0 = time.time()
            while time.time() - t0 < 10:
                vesc.send_duty(vesc_id, 0.2)
                time.sleep(0.05)

            print(f"发送占空比: -0.2")
            t1 = time.time()
            while time.time() - t1 < 10:
                vesc.send_duty(vesc_id, -0.2)
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n--- 测试结束 ---")
    finally:
        # 将占空比归零
        print("占空比归零")
        vesc.send_duty(vesc_id, 0)
        time.sleep(0.1)
        if 'm_dev' in locals() and 'bus' in locals():
            TZCANTransmitter.close_can_device(m_dev, bus)

if __name__ == "__main__":
    main()
