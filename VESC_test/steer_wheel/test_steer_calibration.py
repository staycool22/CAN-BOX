
import sys
import os
import time
import signal

# Add project root to path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

from Steering_wheel_chassis_new import VESCMonitor, BasicConfig

def main():
    print("=== 转向电机零位标定辅助测试工具 ===")
    print("功能：实时读取并打印 Status 2 消息中的 enc2 数据。")
    print("当前逻辑：以 enc2 为绝对位置反馈。")
    print(f"当前配置的零点参数 (STEER_ZERO_PARAMS): {BasicConfig.STEER_ZERO_PARAMS}")
    print("========================================")
    
    monitor = VESCMonitor()
    monitor.perform_zero_calibration = lambda: None
    monitor._control_steer_motor = lambda *args, **kwargs: None
    monitor.start()
    
    # Wait for connection
    time.sleep(1.0)
    
    if not monitor.vesc:
        print("错误：无法连接到 VESC (转向电机 CAN 通道未就绪)。请检查连接。")
        return

    print("\n开始监控电机状态 (按 Ctrl+C 退出)...\n")
    
    try:
        while True:
            # Clear screen (optional, simple print for now)
            # os.system('cls' if os.name == 'nt' else 'clear')
            
            print(f"--- 时间: {time.strftime('%H:%M:%S')} ---")
            
            steer_ids = BasicConfig.get_steer_ids()
            
            for mid in steer_ids:
                if mid in monitor.motor_states:
                    state = monitor.motor_states[mid]
                    
                    turns_vesc = state.get("turns", 0)
                    enc1 = state.get("enc1", 0.0) # Fine
                    enc2 = state.get("enc2", 0.0) # Coarse
                    total_angle = state.get("total_angle", 0.0)
                    
                    # 使用新函数计算圈数
                    turns_calc = monitor.calculate_motor_turns_from_dual(enc1, enc2)
                    
                    print(f"电机 ID [{mid}]:")
                    print(f"  > Enc1 (Fine): {enc1:.2f}")
                    print(f"  > Enc2 (Coarse): {enc2:.2f}")
                    print(f"  > 圈数对比:")
                    print(f"     - VESC输出: {turns_vesc}")
                    print(f"     - 双编计算: {turns_calc} (基于 45:910 比例)")
                    
                    print(f"  > 当前计算角度 (Angle): {total_angle:.4f}°")
                    print(f"  > 零位参数: {monitor.runtime_zero_params.get(mid, 'Not Set')}")
                    
                else:
                    print(f"电机 ID [{mid}]: 无数据 (等待 Status 2 消息...)")
            
            print("-" * 40)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n测试结束。")
    finally:
        # Cleanup if needed (VESCMonitor doesn't have explicit close yet, but thread is daemon)
        monitor.stop()

if __name__ == "__main__":
    main()
