import time
import threading
import sys
import os

# 确保可以从当前目录导入
# 假设脚本位于 /home/dhx/CAN-BOX/VESC_test/steer_wheel/
# 添加父目录以支持 VESC_test.steer_wheel 导入
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
grandparent_dir = os.path.dirname(parent_dir)

if current_dir not in sys.path:
    sys.path.append(current_dir)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)
if grandparent_dir not in sys.path:
    sys.path.append(grandparent_dir)

try:
    from VESC_test.steer_wheel.Steering_wheel_chassis import VESCMonitor, SteerController
except ImportError:
    # 尝试相对导入
    from Steering_wheel_chassis import VESCMonitor, SteerController

# --- 配置 ---
# 设置为 True 以启用手动推车测试模式（不发送控制指令，仅记录数据）
# 设置为 False 以启用自动运动测试模式（发送旋转指令）
MANUAL_TEST_MODE = False

def main():
    # 1. 启动监控器（自动启动日志记录线程）
    # VESCMonitor 现在会同时初始化 can0 (Motor_CTL 驱动) 和 can1 (VESC 转向)
    print("正在初始化底盘监控系统 (Dual CAN)...")
    monitor = VESCMonitor()
    monitor.start()
    
    # 2. 创建控制器
    controller = SteerController(monitor)
    
    try:
        print("--- 开始舵轮底盘测试 (Motor_CTL + VESC) ---")
        
        # 无论何种模式，首先执行归位校准 (转向电机)
        # controller.calibrate_home()
        
        if MANUAL_TEST_MODE:
            print(">>> 当前为【手动测试模式】")
            print(">>> 已禁用所有主动控制指令。")
            print(">>> 请手动推动底盘，程序将持续记录电机角度和状态到 motor.log。")
            print(">>> 按 Ctrl+C 停止测试。")
            
            # 保持程序运行，以便后台监控线程继续工作
            while True:
                time.sleep(1)
                
        else:
            print(">>> 当前为【自动运动模式】")
            
            # 测试左旋
            # 使用 Motor_CTL 控制驱动电机，VESC 控制转向电机
            print(">>> 测试左旋（5秒）")
            # 测试左旋
            print(">>> 测试左旋（5秒）")
            # 注意：spin_left 现在内部使用 self.drive_ctl.set_speed()
            controller.spin_left(rpm=20, duration=5.0) # 降低一点速度进行测试
            # time.sleep(5) 
            
            # 短暂停止以保护机械结构
            print(">>> 停止")
            controller.stop()
            time.sleep(2)
            
            # 测试右旋
            print(">>> 测试右旋（5秒）")
            controller.spin_right(rpm=20, duration=5.0)
            # time.sleep(5)
            
            # 停止
            print(">>> 测试结束，停止")
            controller.stop()
            print(">>> 测试完成")
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试期间发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保彻底关闭
        try:
            controller.stop()
            monitor.stop()
        except Exception as e:
            print(f"关闭资源时出错: {e}")

if __name__ == "__main__":
    main()
