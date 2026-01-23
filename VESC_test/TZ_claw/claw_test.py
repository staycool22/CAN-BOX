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
    print("  u : 开启夹爪 (RPM + 电流限制)")
    print("  i : 关闭夹爪 (RPM + 电流限制)")
    print("  j : 开启夹爪 (开环 Duty)")
    print("  k : 关闭夹爪 (开环 Duty)")
    print("  g : 开启夹爪 (位置模式)")
    print("  h : 关闭夹爪 (位置模式)")
    print("  b : 位置模式移动到 0 度")
    print("  s : 停止夹爪")
    print("  o : 增加限制电流 (+0.5A)")
    print("  p : 减少限制电流 (-0.5A)")
    print("  t : 增加转速 (+500 RPM)")
    print("  y : 减少转速 (-500 RPM)")
    print("  n : 位置模式角度 +100 度")
    print("  m : 位置模式角度 -100 度")
    print("  z : 退出测试")
    print("=========================")

def main():
    print("正在初始化 Claw 系统...")
    
    # 1. 创建配置
    config = Claw_config()
    # config.can_type 已移除，由 use_canfd 自动决定
    config.baud_rate = 500000
    config.vesc_id = 47
    
    # === 控制参数 ===
    open_rpm = 5000.0 # 打开时的转速 (RPM)
    close_rpm = -5000.0 # 关闭时的转速 (RPM)
    open_loop_duty = 0.2 # 开环打开时的占空比 (0.0 - 1.0)
    close_loop_duty = -0.2 # 开环关闭时的占空比 (-1.0 - 0.0)
    max_current = 1.5 # 最大电流限制 (A)
    open_pos_deg = -360.0
    close_pos_deg = 4800.0
    
    tz_claw = None
    controller = None
    try:
        # 2. 初始化硬件
        tz_claw = TZ_Claw(config)
        controller = claw_controller(tz_claw)
        
        print_controls()
        print(f"当前限制电流: {max_current:.1f} A")
        
        # 状态变量
        running = True
        mode = "IDLE" # IDLE, OPEN, CLOSE, OPEN_LOOP_OPEN, OPEN_LOOP_CLOSE, POS_OPEN, POS_CLOSE
        limit_triggered = False # 是否触发了电流限制
        
        # 监控数据
        current_val = 0.0
        rpm_val = 0
        duty_val = 0.0
        
        while running:
            # === 1. 键盘输入处理 ===
            if msvcrt.kbhit():
                # 读取按键 (Windows)
                key_char = msvcrt.getch()
                try:
                    key = key_char.decode('utf-8').lower()
                except UnicodeDecodeError:
                    continue
                
                if key == 'z':
                    print("退出测试...")
                    running = False
                    break
                
                elif key == 'u':
                    print(f"指令: 开启夹爪 (RPM: {open_rpm})")
                    mode = "OPEN"
                    limit_triggered = False
                    
                elif key == 'i':
                    print(f"指令: 关闭夹爪 (RPM: {close_rpm}, MaxCur: {max_current}A)")
                    mode = "CLOSE"
                    limit_triggered = False
                    
                elif key == 'j':
                    print(f"指令: 开环开启 (Duty: {open_loop_duty})")
                    mode = "OPEN_LOOP_OPEN"
                    limit_triggered = False
                    
                elif key == 'k':
                    print(f"指令: 开环关闭 (Duty: {close_loop_duty})")
                    mode = "OPEN_LOOP_CLOSE"
                    limit_triggered = False

                elif key == 'g':
                    print(f"指令: 位置开启 (Pos: {open_pos_deg} deg)")
                    mode = "POS_OPEN"
                    limit_triggered = False

                elif key == 'h':
                    print(f"指令: 位置关闭 (Pos: {close_pos_deg} deg)")
                    mode = "POS_CLOSE"
                    limit_triggered = False

                elif key == 'b':
                    print("指令: 位置归零 (Pos: 0 deg)")
                    controller.open_position(0.0)
                    mode = "IDLE"
                    limit_triggered = False

                elif key == 's':
                    print(f"指令: 停止")
                    mode = "IDLE"
                    limit_triggered = False
                    
                elif key == 'o':
                    max_current += 0.5
                    print(f"调整: 限制电流增加了 0.5A -> 当前: {max_current:.1f} A")
                    # 如果在 Close 模式下触发了限流，更新当前限流值可能需要重置触发状态或者动态调整，这里暂不复杂化
                    
                elif key == 'p':
                    max_current -= 0.5
                    if max_current < 0.5:
                        max_current = 0.5
                    print(f"调整: 限制电流减少了 0.5A -> 当前: {max_current:.1f} A")

                elif key == 't':
                    open_rpm += 500.0
                    close_rpm = -open_rpm
                    print(f"调整: 转速增加 (+500) -> Open: {open_rpm}, Close: {close_rpm}")

                elif key == 'y':
                    open_rpm -= 500.0
                    if open_rpm < 500.0:
                        open_rpm = 500.0
                    close_rpm = -open_rpm
                    print(f"调整: 转速减少 (-500) -> Open: {open_rpm}, Close: {close_rpm}")

                elif key == 'n':
                    close_pos_deg += 100.0
                    print(f"调整: 位置角度增加 (+100) -> Open: {open_pos_deg} deg, Close: {close_pos_deg} deg")

                elif key == 'm':
                    open_pos_deg -= 100.0
                    print(f"调整: 位置角度减少 (-100) -> Open: {open_pos_deg} deg, Close: {close_pos_deg} deg")

            # === 2. 执行控制逻辑 ===
            display_mode = mode
            
            try:
                if mode == "IDLE":
                    controller.stop()
                    
                elif mode == "OPEN":
                    controller.open(open_rpm, max_current)
                    
                elif mode == "CLOSE":
                    controller.close(close_rpm, max_current)
                        
                elif mode == "OPEN_LOOP_OPEN":
                    controller.open_loop_open(open_loop_duty)
                    
                elif mode == "OPEN_LOOP_CLOSE":
                    controller.open_loop_close(close_loop_duty)

                elif mode == "POS_OPEN":
                    controller.open_position(open_pos_deg)

                elif mode == "POS_CLOSE":
                    controller.close_position(close_pos_deg)
                    
            except Exception as e:
                print(f"控制发送错误: {e}")

            # === 3. 读取状态 ===
            try:
                msg_id, packet = controller.get_status(timeout=0.005) # 短超时，避免阻塞循环
                if msg_id and (msg_id & 0xFF) == config.vesc_id:
                    status_id = (msg_id >> 8) & 0xFF
                    if status_id == 0x09: # Status 1
                        current_val = packet.current
                        rpm_val = packet.rpm
                        duty_val = packet.duty
                        
                        # 打印状态 (每行刷新)
                        sys.stdout.write(f"\r[状态] Cur: {current_val:6.2f} A | RPM: {rpm_val:6d} | Duty: {duty_val:5.2f} | 模式: {display_mode:10s}")
                        sys.stdout.flush()
            except Exception as e:
                pass # 忽略读取错误，避免刷屏

            # 循环延时
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
                    controller.stop() # 停止动作
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
