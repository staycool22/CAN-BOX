import time
import sys
import os

# 确保可以导入 claw_base
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from claw_base import Claw_config, TZ_Claw, claw_controller, BasicConfig

def run_action_for_duration(action_func, duration_sec, interval_sec=0.02):
    """
    持续调用 action_func 持续 duration_sec 秒。
    VESC 通常需要持续发送指令以保持激活状态（心跳超时机制）。
    """
    start_time = time.time()
    while (time.time() - start_time) < duration_sec:
        action_func()
        time.sleep(interval_sec)

def main():
    print("=== 开始 Claw 测试 ===")
    
    # 1. 创建配置
    print("正在配置参数...")
    config = Claw_config()
    # 在这里可以覆盖默认配置
    config.can_type = BasicConfig.TYPE_CAN
    config.baud_rate = 500000
    config.vesc_id = 32 # 确保这与实际设备 ID 匹配
    
    # 测试参数设置
    config.open_rpm = 5000.0
    config.close_rpm = -5000.0
    config.open_loop_duty = 0.2
    config.close_loop_duty = -0.2
    
    tz_claw = None
    try:
        # 2. 初始化硬件
        print("正在初始化硬件...")
        tz_claw = TZ_Claw(config)
        controller = claw_controller(tz_claw)
        
        # 3. 执行测试序列
        print("\n--- 测试序列开始 ---")
        
        # 定义测试持续时间（秒）
        TEST_DURATION = 1.0
        STOP_DURATION = 1.0
        
        # 测试 1: 转速控制打开
        print("\n[测试 1] 转速控制: 打开")
        run_action_for_duration(controller.open, TEST_DURATION)
        
        # 停止 (发送 0 RPM)
        print("停止电机 (0 RPM)")
        def stop_rpm(): controller.vesc.send_rpm(config.vesc_id, 0)
        run_action_for_duration(stop_rpm, STOP_DURATION)

        # 测试 2: 转速控制关闭
        print("\n[测试 2] 转速控制: 关闭")
        run_action_for_duration(controller.close, TEST_DURATION)
        
        # 停止
        print("停止电机 (0 RPM)")
        run_action_for_duration(stop_rpm, STOP_DURATION)

        # 测试 3: 开环控制 (占空比) 打开
        print("\n[测试 3] 开环控制: 打开")
        run_action_for_duration(controller.open_loop_open, TEST_DURATION)
        
        # 停止 (发送 0 占空比)
        print("停止电机 (0 Duty)")
        def stop_duty(): controller.vesc.send_duty(config.vesc_id, 0)
        run_action_for_duration(stop_duty, STOP_DURATION)

        # 测试 4: 开环控制 (占空比) 关闭
        print("\n[测试 4] 开环控制: 关闭")
        run_action_for_duration(controller.open_loop_close, TEST_DURATION)
        
        # 停止
        print("停止电机 (0 Duty)")
        run_action_for_duration(stop_duty, STOP_DURATION)
        
        print("\n--- 测试序列完成 ---")

    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
    finally:
        # 4. 清理资源
        print("\n正在清理资源...")
        if tz_claw:
            # 尝试停止电机
            try:
                if tz_claw.vesc:
                    # 尝试发送几次停止指令确保收到
                    for _ in range(5):
                        tz_claw.vesc.send_duty(config.vesc_id, 0)
                        time.sleep(0.01)
            except:
                pass
            tz_claw.close()
        print("=== 测试结束 ===")

if __name__ == "__main__":
    main()
