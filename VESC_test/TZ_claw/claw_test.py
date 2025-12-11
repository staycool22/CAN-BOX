import time
import sys
import os
import msvcrt

# 确保可以导入 claw_base
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from claw_base import Claw_config, TZ_Claw, claw_controller, BasicConfig

def print_controls():
    print("\n=== Claw 键盘控制测试 ===")
    print("控制按键:")
    print("  u : 开启夹爪 (RPM 控制)")
    print("  i : 关闭夹爪 (RPM 控制 + 电流限制)")
    print("  j : 开启夹爪 (开环 Duty)")
    print("  k : 关闭夹爪 (开环 Duty)")
    print("  s : 停止夹爪")
    print("  o : 增加限制电流 (+0.5A)")
    print("  p : 减少限制电流 (-0.5A)")
    print("  z : 退出测试")
    print("=========================")

def main():
    print("正在初始化 Claw 系统...")
    
    # 1. 创建配置
    config = Claw_config()
    config.can_type = BasicConfig.TYPE_CAN
    config.baud_rate = 500000
    config.vesc_id = 32
    
    # 初始化电流限制
    # config.max_current 已经在 claw_base 中默认为 3.5A
    
    tz_claw = None
    controller = None
    try:
        # 2. 初始化硬件
        tz_claw = TZ_Claw(config)
        controller = claw_controller(tz_claw)
        
        print_controls()
        print(f"当前限制电流: {config.max_current:.1f} A")
        
        while True:
            # 检测按键
            if msvcrt.kbhit():
                # 读取按键 (Windows)
                key_char = msvcrt.getch()
                try:
                    key = key_char.decode('utf-8').lower()
                except UnicodeDecodeError:
                    continue
                
                if key == 'z':
                    print("退出测试...")
                    break
                
                elif key == 'u':
                    print(f"指令: 开启夹爪 (RPM: {config.open_rpm})")
                    controller.open()
                    
                elif key == 'i':
                    print(f"指令: 关闭夹爪 (RPM: {config.close_rpm}, MaxCur: {config.max_current}A)")
                    controller.close()
                    
                elif key == 'j':
                    print(f"指令: 开环开启 (Duty: {config.open_loop_duty})")
                    controller.open_loop_open()
                    
                elif key == 'k':
                    print(f"指令: 开环关闭 (Duty: {config.close_loop_duty})")
                    controller.open_loop_close()

                elif key == 's':
                    print(f"指令: 停止")
                    controller.stop()
                    
                elif key == 'o':
                    config.max_current += 0.5
                    print(f"调整: 限制电流增加了 0.5A -> 当前: {config.max_current:.1f} A")
                    
                elif key == 'p':
                    config.max_current -= 0.5
                    if config.max_current < 0.5:
                        config.max_current = 0.5
                    print(f"调整: 限制电流减少了 0.5A -> 当前: {config.max_current:.1f} A")
                    
            # 这里的循环速度很快，主要依赖 claw_base 内部线程维持 VESC 状态 (在 close 模式下)
            # 在非 close 模式下 (open/duty)，如果没有心跳维持，VESC 可能会超时停止
            # 但用户只要求了 close 时的电流监控，暂不修改 open 的行为
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        print("\n正在清理资源...")
        if tz_claw:
            try:
                if controller:
                    controller.stop() # 先停止动作
                    controller.stop_thread() # 停止后台线程
            except Exception as e:
                print(f"清理过程出错: {e}")
            
            # 关闭 CAN 设备
            try:
                tz_claw.close()
            except Exception as e:
                print(f"关闭 CAN 设备出错: {e}")
                
        print("=== 测试结束 ===")
        sys.stdout.flush()
        # 强制退出，以防有其他库留下的残留线程
        print("Calling os._exit(0)...")
        print("\n\n") # 打印额外的换行，确保提示符能显示
        sys.stdout.flush()
        sys.exit(0)

if __name__ == "__main__":
    main()
