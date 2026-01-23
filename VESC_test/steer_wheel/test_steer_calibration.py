
import sys
import os
import time
import signal

# Add project root to path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

from VESC_test.steer_wheel.Steering_wheel_chassis import VESCMonitor, BasicConfig

def main():
    print("=== 转向电机零位标定辅助测试工具 ===")
    print("功能：实时读取并打印 Status 2 消息中的 enc2 数据。")
    print("当前逻辑：以 enc2 为绝对位置反馈，ENC2_ZERO_OFFSET 为零点。")
    print(f"当前配置的零点偏置 (ENC2_ZERO_OFFSET): {BasicConfig.ENC2_ZERO_OFFSET}")
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
                    
                    turns = state.get("turns", 0)
                    enc1 = state.get("last_pos")
                    if enc1 is None:
                        enc1 = 0.0
                        
                    enc2 = state.get("enc2", 0.0)
                    total_angle = state.get("total_angle", 0.0)
                    
                    print(f"电机 ID [{mid}]:")
                    # 打印原始 enc2 值 (从 VESC 接收并除以50后的值)
                    print(f"  > 原始 Enc2 (Raw): {enc2:.4f} (from packet)")
                    print(f"  > 零点偏置 (Offset): {BasicConfig.ENC2_ZERO_OFFSET}")
                    
                    # 打印计算后的角度
                    print(f"  > 当前计算角度 (Angle): {total_angle:.4f}° (Enc2 - Offset)")
                    
                    # 打印其他辅助信息
                    print(f"  > 原始 Enc1: {enc1:.2f}")
                    print(f"  > 圈数: {turns}")
                    
                    # 验证计算逻辑
                    expected_angle = enc2 - BasicConfig.ENC2_ZERO_OFFSET
                    if expected_angle > 180: expected_angle -= 360
                    elif expected_angle < -180: expected_angle += 360
                    
                    if abs(total_angle - expected_angle) > 0.1:
                        print(f"  ⚠️ 警告: 计算不一致! State={total_angle:.4f}, Expected={expected_angle:.4f}")
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
