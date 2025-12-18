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
    # Initialize Kinematics
    # width: 左右轮距 (m) -> 354mm = 0.354m
    geometry = ChassisGeometry(length=0.25, width=0.354, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
    kinematics = FourWheelSteeringKinematics(geometry)
    
    print("Steering Control Test Script (Keyboard Mode)")
    print("Controls:")
    print("  w : Forward (0°)")
    print("  s : Backward (0°)")
    print("  a : Left Strafe (90°)")
    print("  d : Right Strafe (90°)")
    print("  q : Rotate Left")
    print("  e : Rotate Right")
    print("  o : Diagonal Out (L:45, R:-45)")
    print("  p : Diagonal In (L:-45, R:45)")
    print("  SPACE : Stop")
    print("  j : Left Wheel +15° (Manual)")
    print("  k : Left Wheel -15° (Manual)")
    print("  u : Increase Max Speed (+0.1 m/s)")
    print("  i : Decrease Max Speed (-0.1 m/s)")
    print("  y : Increase Accel (+0.1 m/s^2)")
    print("  h : Decrease Accel (-0.1 m/s^2)")
    print("  t : Increase Decel (+0.1 m/s^2)")
    print("  g : Decrease Decel (-0.1 m/s^2)")
    print("  z : Quit")
    
    # Initialize logical angles to 0
    current_left_angle = 0.0
    current_right_angle = 0.0

    # Speed params
    MAX_SPEED = 0.2 # m/s
    MAX_OMEGA = 0.15 # rad/s
    ACCEL = 0.5 # m/s^2 (平均加速度)
    DECEL = 5.5 # m/s^2 (平均减速度)

    # Calculate Accel/Decel time (ms) using the shared logic in BasicConfig
    accel_time_ms = BasicConfig.calc_accel_time_ms(ACCEL)
    decel_time_ms = BasicConfig.calc_accel_time_ms(DECEL)
    
    print(f"Initializing VESC Monitor with Accel: {ACCEL} m/s^2 ({accel_time_ms}ms), Decel: {DECEL} m/s^2 ({decel_time_ms}ms)")
    monitor = VESCMonitor(accel_time_ms=accel_time_ms, decel_time_ms=decel_time_ms)
    monitor.start()
    
    controller = SteerController(monitor)
    
    # 临时屏蔽驱动电机
    # 只要将 controller.drive_ctl 置为 None，就不会发送驱动指令
    # controller.drive_ctl = None

    # Set initial position to 0
    controller._send_steer_pos(BasicConfig.FL_STEER_ID, current_left_angle)
    controller._send_steer_pos(BasicConfig.FR_STEER_ID, current_right_angle)

    # Safety: Auto-stop timestamp
    last_cmd_time = time.time()
    is_moving = False
    
    # WATCHDOG_TIMEOUT: Timeout for auto-stop when key is released
    WATCHDOG_TIMEOUT = 0.2

    try:
        while True:
            key = None
            # Drain buffer to get the latest key
            while msvcrt.kbhit():
                char = msvcrt.getch()
                try:
                    key = char.decode('utf-8').lower()
                except UnicodeDecodeError:
                    continue
            
            # Current loop time
            now = time.time()

            if key:
                last_cmd_time = now # Update watchdog
                
                if key == 'z':
                    break
                
                # Kinematics Controls
                vx, vy, omega = 0.0, 0.0, 0.0
                active_kinematics = False
                
                if key == 'w':
                    vx = MAX_SPEED
                    active_kinematics = True
                    # print(f" -> Forward (v={vx})") # Reduce print spam
                elif key == 's':
                    vx = -MAX_SPEED
                    active_kinematics = True
                    # print(f" -> Backward (v={vx})")
                elif key == 'a':
                    vy = MAX_SPEED # Left
                    active_kinematics = True
                    # print(f" -> Left Strafe (vy={vy})")
                elif key == 'd':
                    vy = -MAX_SPEED # Right
                    active_kinematics = True
                    # print(f" -> Right Strafe (vy={vy})")
                elif key == 'q':
                    omega = MAX_OMEGA
                    active_kinematics = True
                    # print(f" -> Rotate Left (w={omega})")
                elif key == 'e':
                    omega = -MAX_OMEGA
                    active_kinematics = True
                    # print(f" -> Rotate Right (w={omega})")
                elif key == 'o':
                    # 左轮逆时针 45 (45度), 右轮顺时针 45 (-45度), MAX_SPEED
                    import math
                    wheel_states = {
                        "FL": (MAX_SPEED, math.radians(45)),
                        "FR": (MAX_SPEED, math.radians(45))
                    }
                    controller.apply_kinematics(wheel_states)
                    last_wheel_states = wheel_states
                    is_moving = True
                    print(f" -> Diagonal O (L:45°, R:45°, v={MAX_SPEED})")
                    continue
                elif key == 'p':
                    # 相反方向: 左轮顺时针 45 (-45度), 右轮逆时针 45 (45度), MAX_SPEED
                    import math
                    wheel_states = {
                        "FL": (-MAX_SPEED, math.radians(45)),
                        "FR": (-MAX_SPEED, math.radians(45))
                    }
                    controller.apply_kinematics(wheel_states)
                    last_wheel_states = wheel_states
                    is_moving = True
                    print(f" -> Diagonal P (L:45°, R:45°, v={MAX_SPEED})")
                    continue
                elif key == ' ':
                    vx, vy, omega = 0.0, 0.0, 0.0
                    active_kinematics = True
                    print(" -> Stop")
                
                if active_kinematics:
                    # Calculate wheel states
                    wheel_states = kinematics.inverse_kinematics(vx, vy, omega)
                    # Apply to motors
                    controller.apply_kinematics(wheel_states)
                    last_wheel_states = wheel_states
                    is_moving = True
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
                    MAX_SPEED += 0.1
                    print(f" -> Max Speed Increased: {MAX_SPEED:.1f} m/s")
                    
                elif key == 'i':
                    MAX_SPEED = max(0.0, MAX_SPEED - 0.1)
                    print(f" -> Max Speed Decreased: {MAX_SPEED:.1f} m/s")

                elif key == 'y':
                    ACCEL += 0.1
                    controller.set_accel_decel(ACCEL, DECEL)
                    print(f" -> Accel Increased: {ACCEL:.1f} m/s^2")
                    
                elif key == 'h':
                    ACCEL = max(0.1, ACCEL - 0.1)
                    controller.set_accel_decel(ACCEL, DECEL)
                    print(f" -> Accel Decreased: {ACCEL:.1f} m/s^2")

                elif key == 't':
                    DECEL += 0.1
                    controller.set_accel_decel(ACCEL, DECEL)
                    print(f" -> Decel Increased: {DECEL:.1f} m/s^2")
                    
                elif key == 'g':
                    DECEL = max(0.1, DECEL - 0.1)
                    controller.set_accel_decel(ACCEL, DECEL)
                    print(f" -> Decel Decreased: {DECEL:.1f} m/s^2")
            
            # Watchdog check: If no key for > WATCHDOG_TIMEOUT, Stop
            elif is_moving and (now - last_cmd_time > WATCHDOG_TIMEOUT):
                print(" -> Auto Stop (Key Released)")
                vx, vy, omega = 0.0, 0.0, 0.0
                wheel_states = kinematics.inverse_kinematics(vx, vy, omega)
                controller.apply_kinematics(wheel_states)
                is_moving = False

            time.sleep(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping...")
        monitor.stop()

if __name__ == "__main__":
    main()
