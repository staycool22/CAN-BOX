import time
import sys
import os
import threading
import msvcrt

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from Steering_wheel_chassis import VESCMonitor, SteerController, BasicConfig
from chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics

def main():
    print("Initializing VESC Monitor...")
    monitor = VESCMonitor()
    monitor.start()
    
    controller = SteerController(monitor)
    
    # Initialize Kinematics
    # width: 左右轮距 (m) -> 354mm = 0.354m
    geometry = ChassisGeometry(length=0.6, width=0.354, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
    kinematics = FourWheelSteeringKinematics(geometry)
    
    print("Steering Control Test Script (Keyboard Mode)")
    print("Controls:")
    print("  w : Forward (0°)")
    print("  s : Backward (0°)")
    print("  a : Left Strafe (90°)")
    print("  d : Right Strafe (90°)")
    print("  q : Rotate Left")
    print("  e : Rotate Right")
    print("  SPACE : Stop")
    print("  j : Left Wheel +15° (Manual)")
    print("  k : Left Wheel -15° (Manual)")
    print("  u : Right Wheel +15° (Manual)")
    print("  i : Right Wheel -15° (Manual)")
    print("  z : Quit")
    
    # Initialize logical angles to 0
    current_left_angle = 0.0
    current_right_angle = 0.0
    
    # 临时屏蔽驱动电机
    # 只要将 controller.drive_ctl 置为 None，就不会发送驱动指令
    # controller.drive_ctl = None
    print("⚠️ 驱动电机已在测试脚本中被屏蔽，仅测试转向。")

    # Set initial position to 0
    controller._send_steer_pos(BasicConfig.FL_STEER_ID, current_left_angle)
    controller._send_steer_pos(BasicConfig.FR_STEER_ID, current_right_angle)
    
    # Speed params
    MAX_SPEED = 0.1 # m/s
    MAX_OMEGA = 0.15 # rad/s
    
    try:
        while True:
            if msvcrt.kbhit():
                # Read key
                char = msvcrt.getch()
                try:
                    key = char.decode('utf-8').lower()
                except UnicodeDecodeError:
                    continue
                
                if key == 'z':
                    break
                
                # Kinematics Controls
                vx, vy, omega = 0.0, 0.0, 0.0
                active_kinematics = False
                
                if key == 'w':
                    vx = MAX_SPEED
                    active_kinematics = True
                    print(f" -> Forward (v={vx})")
                elif key == 's':
                    vx = -MAX_SPEED
                    active_kinematics = True
                    print(f" -> Backward (v={vx})")
                elif key == 'a':
                    vy = MAX_SPEED # Left
                    active_kinematics = True
                    print(f" -> Left Strafe (vy={vy})")
                elif key == 'd':
                    vy = -MAX_SPEED # Right
                    active_kinematics = True
                    print(f" -> Right Strafe (vy={vy})")
                elif key == 'q':
                    omega = MAX_OMEGA
                    active_kinematics = True
                    print(f" -> Rotate Left (w={omega})")
                elif key == 'e':
                    omega = -MAX_OMEGA
                    active_kinematics = True
                    print(f" -> Rotate Right (w={omega})")
                elif key == ' ':
                    vx, vy, omega = 0.0, 0.0, 0.0
                    active_kinematics = True
                    print(" -> Stop")
                
                if active_kinematics:
                    # Calculate wheel states
                    wheel_states = kinematics.inverse_kinematics(vx, vy, omega)
                    # Apply to motors
                    controller.apply_kinematics(wheel_states)
                    continue

                # Manual Controls (Legacy)
                if key == 'j':
                    current_left_angle += 15.0
                    controller._send_steer_pos(BasicConfig.FL_STEER_ID, current_left_angle)
                    print(f" -> Left Target: {current_left_angle}°")
                    
                elif key == 'k':
                    current_left_angle -= 15.0
                    controller._send_steer_pos(BasicConfig.FL_STEER_ID, current_left_angle)
                    print(f" -> Left Target: {current_left_angle}°")
                    
                elif key == 'u':
                    current_right_angle += 15.0
                    controller._send_steer_pos(BasicConfig.FR_STEER_ID, current_right_angle)
                    print(f" -> Right Target: {current_right_angle}°")
                    
                elif key == 'i':
                    current_right_angle -= 15.0
                    controller._send_steer_pos(BasicConfig.FR_STEER_ID, current_right_angle)
                    print(f" -> Right Target: {current_right_angle}°")

            time.sleep(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping...")
        monitor.stop()

if __name__ == "__main__":
    main()
