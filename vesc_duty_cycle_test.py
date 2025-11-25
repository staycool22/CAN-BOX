
import time
import argparse
from typing import List, Optional

from vesc_canfd import build, parse_id_token
from TZCANTransmitter import TZCANTransmitter

def main():
    parser = argparse.ArgumentParser(description="VESC CAN/CAN-FD 占空比正反转循环测试")
    parser.add_argument("--baud-rate", type=int, default=500000)
    parser.add_argument("--data-bitrate", type=int, default=1000000)
    parser.add_argument("--channels", type=int, nargs="+", default=[1])
    parser.add_argument("--backend", type=str, default="candle")
    parser.add_argument("--sp", type=float, default=75.0)
    parser.add_argument("--dsp", type=float, default=80.0)
    parser.add_argument("--use-canfd", dest="use_canfd", action="store_true", default=False)
    parser.add_argument("--no-canfd", dest="use_canfd", action="store_false")
    parser.add_argument("--vesc-id", type=parse_id_token, default=0x30)
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

    try:
        print("--- 开始占空比正反转测试 ---")
        while True:
            print(f"发送占空比: 0.2")
            vesc.send_duty(vesc_id, 0.2)
            time.sleep(5)

            print(f"发送占空比: -0.2")
            vesc.send_duty(vesc_id, -0.2)
            time.sleep(5)

    except KeyboardInterrupt:
        print("\n--- 测试结束 ---")
    finally:
        # 将占空比归零
        print("占空比归零")
        vesc.send_duty(vesc_id, 0)
        time.sleep(0.1)
        TZCANTransmitter.close_can_device(m_dev, bus)

if __name__ == "__main__":
    main()
