import time
import sys
import os
import threading
import msvcrt

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from Steering_wheel_chassis_new import VESCMonitor, SteerController, BasicConfig
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
    print("  ↑ : Increase Accel (+0.1 m/s^2)")
    print("  ↓ : Decrease Accel (-0.1 m/s^2)")
    print("  → : Increase Decel (+0.1 m/s^2)")
    print("  ← : Decrease Decel (-0.1 m/s^2)")
    print("  z : Quit")
    
    # Initialize logical angles to 0
    current_left_angle = 0.0
    current_right_angle = 0.0

    # Speed params
    MAX_SPEED = 8.0 # m/s
    MAX_OMEGA = 0.45 # rad/s

    print(f"Initializing VESC Monitor...")
    monitor = VESCMonitor()
    monitor.perform_zero_calibration = lambda: None
    monitor.start()

    print("Waiting for VESC data...")
    # Wait for valid data from all steering motors
    start_wait = time.time()
    while time.time() - start_wait < 3.0:
        ready = True
        active_ids = []
        for mid in BasicConfig.get_steer_ids():
            state = monitor.get_state(mid)
            if state.get("last_pos") is None:
                ready = False
            else:
                active_ids.append(mid)
        
        if ready:
            print(f"VESC data received for all motors: {active_ids}")
            break
        
        # If some are ready but not all, we wait a bit more, but eventually proceed
        if active_ids and time.time() - start_wait > 1.0:
             # If we have at least some data after 1s, maybe that's all we'll get
             pass
             
        time.sleep(0.1)
    
    # Allow testing drive motors even if steering motors are missing
    if not active_ids and monitor.vesc_drive:
        print("⚠️ Warning: No steering motors detected, but Drive VESC is connected. Enabling Drive-Only test mode.")
    
    print("VESC Monitor Active.")

    # Display status
    for mid in BasicConfig.get_steer_ids():
        state = monitor.get_state(mid)
        if state.get("last_pos") is not None:
             print(f"Steer VESC {mid} OK: Angle={state.get('total_angle', 0.0):.1f}")
        else:
             print(f"Steer VESC {mid} NO DATA")

    if monitor.vesc_drive:
         print("Drive VESC Connected")
    else:
         print("Drive VESC Not Connected")

    controller = SteerController(monitor)
    
    # 设置软件加减速 (m/s^2)
    controller.set_accel_decel(3.5, 4.0)

    def reset_steer_to_zero():
        print(" -> Resetting Steer to 0°")
        for mid in BasicConfig.get_steer_ids():
            controller._send_steer_pos(mid, 0.0)

    # 临时屏蔽驱动电机
    # 只要将 controller.drive_ctl 置为 None，就不会发送驱动指令
    # controller.drive_ctl = None

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
                if char == b'\xe0': # Special key prefix (Arrow keys)
                    if msvcrt.kbhit():
                        ext_char = msvcrt.getch()
                        # Handle arrow keys
                        if ext_char == b'H': # Up Arrow -> Accel +
                            BasicConfig.DRIVE_ACCEL += 0.1
                            print(f" -> Accel Increased: {BasicConfig.DRIVE_ACCEL:.1f} m/s^2")
                        elif ext_char == b'P': # Down Arrow -> Accel -
                            BasicConfig.DRIVE_ACCEL = max(0.1, BasicConfig.DRIVE_ACCEL - 0.1)
                            print(f" -> Accel Decreased: {BasicConfig.DRIVE_ACCEL:.1f} m/s^2")
                        elif ext_char == b'M': # Right Arrow -> Decel +
                            BasicConfig.DRIVE_DECEL += 0.1
                            print(f" -> Decel Increased: {BasicConfig.DRIVE_DECEL:.1f} m/s^2")
                        elif ext_char == b'K': # Left Arrow -> Decel -
                            BasicConfig.DRIVE_DECEL = max(0.1, BasicConfig.DRIVE_DECEL - 0.1)
                            print(f" -> Decel Decreased: {BasicConfig.DRIVE_DECEL:.1f} m/s^2")
                    continue
                
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
                    wheel_states = kinematics.inverse_kinematics(vx, vy, omega)
                    controller.apply_kinematics(wheel_states)
                    reset_steer_to_zero()
                    is_moving = False
                    print(" -> Stop")
                    continue
                
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
            
            # Watchdog check: If no key for > WATCHDOG_TIMEOUT, Stop
            elif is_moving and (now - last_cmd_time > WATCHDOG_TIMEOUT):
                print(" -> Auto Stop (Key Released)")
                vx, vy, omega = 0.0, 0.0, 0.0
                
                # 持续调用 apply_kinematics 直到速度真正降为 0
                # 注意：apply_kinematics 内部有斜坡，我们需要给它时间去执行
                # 在这里我们不阻塞主循环，而是将 is_moving 保持为 True，但在没有按键时发送 0 速度
                # 直到 controller 内部的 RPM 也降为 0
                
                wheel_states = kinematics.inverse_kinematics(0.0, 0.0, 0.0)
                controller.apply_kinematics(wheel_states)
                
                # 检查是否已经完全停止
                all_stopped = True
                for drive_id in BasicConfig.get_drive_ids():
                    if abs(controller.current_drive_rpms.get(drive_id, 0.0)) > 10.0:
                        all_stopped = False
                        break
                
                if all_stopped:
                    is_moving = False
                    reset_steer_to_zero() # 改为回正
                    print(" -> Fully Stopped.")
                else:
                    # 只要还没停稳，就继续标记为 moving，以便下一次循环继续进来发送 0 速度
                    # 更新 watchdog 时间以避免重复打印 "Auto Stop" (或者我们可以允许重复进入这个分支)
                    # 为了避免刷屏 "Auto Stop"，我们可以加个标志位或者只在第一次打印
                    pass

            time.sleep(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping...")
        monitor.stop()

if __name__ == "__main__":
    main()
