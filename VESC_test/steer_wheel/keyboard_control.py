import sys
import os
import time
import threading

# 跨平台按键输入检测
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty
    from select import select

# 添加路径以导入模块
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
    from Steering_wheel_chassis import VESCMonitor, SteerController, BasicConfig
except ImportError:
    print("Error: Could not import Steering_wheel_chassis modules.")
    sys.exit(1)

# 配置参数
DRIVE_RPM = 10  # 用户请求 RPM
STEER_ZERO = 0.0

class KeyboardController:
    def __init__(self):
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        self.monitor = VESCMonitor()
        self.controller = SteerController(self.monitor)
        self.running = True
        self.current_speed_l = 0
        self.current_speed_r = 0
        self.current_steer_angle_fl = 0.0 # 左前
        self.current_steer_angle_fr = 0.0 # 右前
        
        # 错误处理机制
        self.drive_error_count = 0
        self.drive_error_threshold = 10
        self.drive_disabled = False

    def getKey(self):
        if os.name == 'nt':
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8').lower()
            return ''
        else:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key

    def stop_robot(self):
        self.current_speed_l = 0
        self.current_speed_r = 0
        self.send_drive_command(0, 0)
        print("\r\n🛑 停止")

    def send_drive_command(self, left_rpm, right_rpm):
        """
        发送驱动电机指令 (PDO)
        """
        if self.drive_disabled:
            return

        if self.controller.drive_ctl:
            left_speed_int = int(left_rpm)
            right_speed_int = int(right_rpm)
            
            try:
                left_bytes = left_speed_int.to_bytes(2, byteorder='little', signed=True)
                right_bytes = right_speed_int.to_bytes(2, byteorder='little', signed=True)
                pdo_data = list(left_bytes) + list(right_bytes)
                
                if not self.controller.drive_ctl.send_pdo('rpdo1', pdo_data):
                     self.drive_error_count += 1
                else:
                     self.drive_error_count = 0 # 成功则重置计数
                
                if self.drive_error_count > self.drive_error_threshold:
                    print(f"\r\n⚠️ 驱动电机通信连续失败 {self.drive_error_count} 次，已禁用驱动指令发送。")
                    self.drive_disabled = True
                    
            except Exception as e:
                self.drive_error_count += 1
                if self.drive_error_count > self.drive_error_threshold:
                    print(f"\r\n⚠️ 驱动电机通信异常: {e}。已禁用驱动指令发送。")
                    self.drive_disabled = True
                # print(f"\r\n❌ PDO构建错误: {e}")

    def run(self):
        print("正在初始化底盘监控系统...")
        self.monitor.start()
        
        # 确保转向电机归零 (软件位置控制目标设为0，即锁定在当前位置)
        # 用户要求：“原地旋转时无需改变舵角直接使用电机正反转”
        # 所以我们这里显式设置转向目标为0
        self.controller._send_steer_pos(BasicConfig.FL_STEER_ID, 0.0)
        self.controller._send_steer_pos(BasicConfig.FR_STEER_ID, 0.0)
        
        print("\r\n=== 键盘控制测试工具 ===")
        print(f"设定 RPM: {DRIVE_RPM}")
        print("控制键:")
        print("  w: 前进 (Left +, Right -)")
        print("  s: 后退 (Left -, Right +)")
        print("  a: 原地左旋 (Left -, Right -) [差速旋转]")
        print("  d: 原地右旋 (Left +, Right +) [差速旋转]")
        print("  j: 增加左前舵角 (FL +1度)")
        print("  k: 减小左前舵角 (FL -1度)")
        print("  u: 增加右前舵角 (FR +1度)")
        print("  i: 减小右前舵角 (FR -1度)")
        print("  空格: 停止")
        print("  q: 退出")
        print("=========================")

        try:
            while self.running:
                key = self.getKey()
                
                if key == 'w':
                    
                    self.current_speed_l = DRIVE_RPM
                    self.current_speed_r = -DRIVE_RPM
                    print(f"\r\n⬆️ 前进: L={self.current_speed_l}, R={self.current_speed_r}")

                elif key == 's':
                    # 后退
                    self.current_speed_l = -DRIVE_RPM
                    self.current_speed_r = DRIVE_RPM
                    print(f"\r\n⬇️ 后退: L={self.current_speed_l}, R={self.current_speed_r}")

                elif key == 'a':
                    # 左旋 (Left -, Right -)
                    self.current_speed_l = -DRIVE_RPM
                    self.current_speed_r = -DRIVE_RPM
                    print(f"\r\n⬅️ 左旋: L={self.current_speed_l}, R={self.current_speed_r}")

                elif key == 'd':
                    # 右旋 (Left +, Right +)
                    self.current_speed_l = DRIVE_RPM
                    self.current_speed_r = DRIVE_RPM
                    print(f"\r\n➡️ 右旋: L={self.current_speed_l}, R={self.current_speed_r}")

                elif key == 'j':
                    self.current_steer_angle_fl += 1.0
                    print(f"\r\n📐 增加左前舵角: {self.current_steer_angle_fl}")
                    self.controller._send_steer_pos(BasicConfig.FL_STEER_ID, self.current_steer_angle_fl)

                elif key == 'k':
                    self.current_steer_angle_fl -= 1.0
                    print(f"\r\n📐 减小左前舵角: {self.current_steer_angle_fl}")
                    self.controller._send_steer_pos(BasicConfig.FL_STEER_ID, self.current_steer_angle_fl)

                elif key == 'u':
                    self.current_steer_angle_fr += 1.0
                    print(f"\r\n📐 增加右前舵角: {self.current_steer_angle_fr}")
                    self.controller._send_steer_pos(BasicConfig.FR_STEER_ID, self.current_steer_angle_fr)

                elif key == 'i':
                    self.current_steer_angle_fr -= 1.0
                    print(f"\r\n📐 减小右前舵角: {self.current_steer_angle_fr}")
                    self.controller._send_steer_pos(BasicConfig.FR_STEER_ID, self.current_steer_angle_fr)

                elif key == ' ':
                    self.stop_robot()

                elif key == 'q':
                    self.stop_robot()
                    self.running = False
                    print("\r\n退出...")
                    break
                
                # 持续发送指令 (心跳)
                # if self.current_speed_l != 0 or self.current_speed_r != 0:
                self.send_drive_command(self.current_speed_l, self.current_speed_r)
                
                # 稍微延时避免 CPU 占用过高，同时保持发送频率
                time.sleep(0.05) 

        except Exception as e:
            print(f"\r\nError: {e}")
        finally:
            self.stop_robot()
            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.monitor.stop()

if __name__ == "__main__":
    kb = KeyboardController()
    kb.run()
