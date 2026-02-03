import time
import sys
import os
import threading
import math
import argparse

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# Cross-platform non-blocking keyboard input
if os.name == 'nt':
    import msvcrt
    def get_key():
        while msvcrt.kbhit():
            try:
                char = msvcrt.getch()
                return char.decode('utf-8').lower()
            except UnicodeDecodeError:
                continue
        return None
else:
    import sys
    import select
    import tty
    import termios
    
    class NonBlockingConsole(object):
        def __enter__(self):
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            return self

        def __exit__(self, type, value, traceback):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

        def get_data(self):
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
            return None
    
    # Global instance management for simplicity in this script structure
    console_input = None
    
    def get_key():
        global console_input
        if console_input is None:
            # Note: This requires the context manager to be active. 
            # We will handle this by checking/initializing or just using direct calls if context is tricky to wrap around the loop.
            # A simpler approach for this script:
            pass
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
             return sys.stdin.read(1).lower()
        return None

    # Setup terminal settings for Linux
    def setup_linux_term():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        return old_settings

    def restore_linux_term(old_settings):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)

from Steering_wheel_chassis_0131 import VESCMonitor, SteerController
from steer_wheel_config import BasicConfig
from chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics
from joystick_controller import JoystickController
from dashboard_client import DashboardClient, SimulatedSteerController, start_dashboard_server

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["real", "sim"], default="real")
    parser.add_argument("--input", choices=["keyboard", "joystick", "both"], default="keyboard")
    parser.add_argument("--joystick", default="/dev/input/js0")
    parser.add_argument("--dashboard", default=None)
    parser.add_argument("--dashboard-host", default="0.0.0.0")
    parser.add_argument("--dashboard-port", type=int, default=8080)
    parser.add_argument("--dashboard-interval", type=float, default=0.2)
    parser.add_argument("--dashboard-enable", action="store_true", default=False)
    args = parser.parse_args()

    print("Initializing VESC Monitor...")
    # Initialize Kinematics
    # width: 左右轮距 (m) -> 354mm = 0.354m
    geometry = ChassisGeometry(length=BasicConfig.CHASSIS_LENGTH, width=BasicConfig.CHASSIS_WIDTH, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
    kinematics = FourWheelSteeringKinematics(geometry)
    
    print("Steering Control Test Script")
    print("Controls:")
    print("  w : Forward (0°)")
    print("  s : Backward (0°)")
    print("  a : Left Strafe (90°)")
    print("  d : Right Strafe (90°)")
    print("  q : Rotate Left")
    print("  e : Rotate Right")
    print("  o : Increase Max Omega (+0.05 rad/s)")
    print("  p : Decrease Max Omega (-0.05 rad/s)")
    print("  SPACE : Stop")
    print("  j : Left Wheel +15° (Manual)")
    print("  k : Left Wheel -15° (Manual)")
    print("  u : Increase Desired Speed (+0.1 m/s)")
    print("  i : Decrease Desired Speed (-0.1 m/s)")
    print("  z : Quit")
    print(f"Input Mode: {args.input}")
    print(f"Run Mode: {args.mode}")
    
    # Initialize logical angles to 0
    current_left_angle = 0.0
    current_right_angle = 0.0
    
    # 标志位: 开启新版手柄控制模式 (360度 + 倒车逻辑)
    ENABLE_NEW_JOYSTICK_MODE = True

    # Speed params
    MAX_SPEED = 2.0 # m/s
    MAX_OMEGA = 1.0 # rad/s
    ACCEL_LIMIT = 0.5 # m/s^2 (Smooth braking/acceleration for translation)
    SPIN_ACCEL_LIMIT = 0.1 # m/s^2 (Smooth braking/acceleration for spin)
    enable_smooth_braking = True # Default to smooth mode
    last_physics_time = time.time()
    
    monitor = None
    controller = None
    if args.mode == "real":
        print(f"Initializing VESC Monitor...")
        monitor = VESCMonitor()
        monitor.start()
        controller = SteerController(monitor)
        if controller.vesc_drive is None:
            print("✅ 驱动电机已禁用 (纯转向调试模式)")
        else:
            print("⚠️ 注意: 驱动电机已启用 (VESC)")
    else:
        print("✅ 仿真模式已启用")
        controller = SimulatedSteerController()

    if args.dashboard_enable:
        server, server_thread, server_store = start_dashboard_server(
            host=args.dashboard_host,
            port=args.dashboard_port
        )
        if not args.dashboard:
            args.dashboard = f"http://127.0.0.1:{args.dashboard_port}"

    # Set initial position to 0
    controller._send_steer_pos(BasicConfig.FL_STEER_ID, current_left_angle)
    controller._send_steer_pos(BasicConfig.FR_STEER_ID, current_right_angle)

    # Safety: Auto-stop timestamp
    last_cmd_time = time.time()
    is_moving = False
    
    # WATCHDOG_TIMEOUT: Timeout for auto-stop when key is released
    WATCHDOG_TIMEOUT = 0.5
    
    last_print_time_38 = time.time()

    joystick = None
    if args.input in ("joystick", "both"):
        try:
            # 传入标志位
            joystick = JoystickController(args.joystick, control_mode_360=ENABLE_NEW_JOYSTICK_MODE)
            print(f"✅ 手柄已连接: {args.joystick}")
            if ENABLE_NEW_JOYSTICK_MODE:
                print("   [模式] 启用新版 360° 控制模式 (左摇杆耦合转向+油门, 下半圆倒车)")
        except Exception as e:
            print(f"⚠️ 手柄初始化失败: {e}")

    dashboard_client = None
    if args.dashboard:
        dashboard_client = DashboardClient(args.dashboard)
        print(f"✅ 仪表盘客户端已启用: {args.dashboard}")
    
    # Linux terminal settings
    linux_old_settings = None
    if os.name != 'nt':
        linux_old_settings = setup_linux_term()

    # Initialize control states
    steer_angle_deg = 0.0
    current_speed_cmd = 0.0
    desired_speed = min(0.1, MAX_SPEED)
    joy_last_angle = 0.0
    joy_last_direction = 1.0
    last_dashboard_time = 0.0
    
    try:
        while True:
            # --- 批量读取键盘输入，解决组合键卡顿问题 ---
            keys_batch = []
            if args.input in ("keyboard", "both"):
                # 最多读取10个积压按键，防止死循环
                for _ in range(10):
                    k = get_key()
                    if k is None:
                        break
                    keys_batch.append(k)
            
            # 如果没有按键，设为None以便进入看门狗逻辑
            if not keys_batch:
                keys_batch = [None]
                
            # Current loop time
            now = time.time()
            dt = now - last_physics_time
            last_physics_time = now
            
            handled_input = False
            is_wsad = False  # 标记本帧是否有运动指令
            
            # 优先处理手柄 (逻辑不变)
            if joystick:
                joystick.poll()
                joy_cmd = joystick.get_command()
                # ... (手柄逻辑代码保持原样，这里只处理键盘逻辑的合并)
                if joy_cmd["emergency_stop"]:
                    current_speed_cmd = 0.0
                    steer_angle_deg = 0.0
                    controller.stop()
                    is_moving = False
                    handled_input = True
                    last_cmd_time = now
                elif joy_cmd.get("switch_mode"):
                    # Toggle Mode via Joystick Button A
                    # Debounce needed? JoystickController sends continuous button press
                    # We should check if it was already pressed last frame
                    # Simple debounce: check if now - last_mode_switch_time > 0.5
                    if not hasattr(main, "last_joy_mode_time"):
                        main.last_joy_mode_time = 0
                    
                    if now - main.last_joy_mode_time > 0.5:
                        BasicConfig.ENABLE_ACKERMANN_MODE = not BasicConfig.ENABLE_ACKERMANN_MODE
                        controller.switch_kinematics_mode(BasicConfig.ENABLE_ACKERMANN_MODE)
                        m_str = "Ackermann" if BasicConfig.ENABLE_ACKERMANN_MODE else "Holonomic"
                        print(f"🎮 Joystick Toggled Mode to: {m_str}")
                        main.last_joy_mode_time = now
                    
                    handled_input = True
                    last_cmd_time = now
                elif joy_cmd["active_stick"] or abs(joy_cmd["speed"]) > 0.01 or abs(joy_cmd.get("spin", 0.0)) > 0.01:
                    # Spin Mode (Right Stick Horizontal)
                    spin_cmd = joy_cmd.get("spin", 0.0)
                    if abs(spin_cmd) > 0.01:
                        # Calculate chassis radius (distance from center to wheel)
                        chassis_radius = math.hypot(geometry.L/2, geometry.W/2)
                        # Max linear speed at wheels for desired MAX_OMEGA
                        max_spin_linear_speed = MAX_OMEGA * chassis_radius
                        
                        target_speed = max_spin_linear_speed * abs(spin_cmd) 
                        
                        # Apply smooth acceleration for spin mode if enabled
                        if enable_smooth_braking:
                            max_delta = SPIN_ACCEL_LIMIT * dt
                            diff = target_speed - current_speed_cmd
                            if abs(diff) > max_delta:
                                current_speed_cmd += math.copysign(max_delta, diff)
                            else:
                                current_speed_cmd = target_speed
                        else:
                            current_speed_cmd = target_speed
                        
                        if spin_cmd > 0:
                            controller.spin_clockwise(current_speed_cmd)
                        else:
                            controller.spin_counter_clockwise(current_speed_cmd)
                        
                    else:
                        # Normal Mode (Explicit Angle + Speed)
                        if joy_cmd["active_stick"] and joy_cmd["angle_deg"] is not None:
                            # New Mapping: Joystick 0(Left)->180(Right)
                            target_steer = 90.0 - joy_cmd["angle_deg"]
                            joy_last_angle = target_steer
                        else:
                            joy_last_angle = 0.0
    
                        steer_angle_deg = joy_last_angle
                        
                        # Speed from Right Stick (Up=Positive, Down=Negative)
                        target_speed = MAX_SPEED * joy_cmd["speed"]
                        
                        if enable_smooth_braking:
                            # Recalculate dt for this block if needed, but using main loop dt is better
                            # dt is calculated at start of loop (line 207)
                            max_delta = ACCEL_LIMIT * dt
                            diff = target_speed - current_speed_cmd
                            if abs(diff) > max_delta:
                                current_speed_cmd += math.copysign(max_delta, diff)
                            else:
                                current_speed_cmd = target_speed
                        else:
                            current_speed_cmd = target_speed
                        
                        # Use high-level command
                        controller.move_diagonal(steer_angle_deg, current_speed_cmd)
                        
                    is_moving = True
                    handled_input = True
                    last_cmd_time = now
                    
                # Handle deceleration when joystick is released (but loop still running)
                elif is_moving and enable_smooth_braking and abs(current_speed_cmd) > 0.01:
                     # Decelerate to 0
                     target_speed = 0.0
                     max_delta = ACCEL_LIMIT * dt
                     diff = target_speed - current_speed_cmd
                     
                     if abs(diff) > max_delta:
                         current_speed_cmd += math.copysign(max_delta, diff)
                     else:
                         current_speed_cmd = target_speed
                         
                     # Maintain steering angle but update speed
                     controller.move_diagonal(steer_angle_deg, current_speed_cmd)
                     
                     handled_input = True
                     last_cmd_time = now # Keep alive while decelerating
                     
                     if abs(current_speed_cmd) < 0.01:
                         is_moving = False
                         current_speed_cmd = 0.0
                         controller.stop()

            # --- 键盘处理逻辑 (重构后) ---
            if not handled_input:
                for key in keys_batch:
                    if not key:
                        continue
                        
                    last_cmd_time = now # Update watchdog
                    
                    if key == 'z':
                        raise KeyboardInterrupt # 退出
                    
                    # --- Runtime Mode Switching ---
                    if key == 'm':
                        # Toggle motion mode
                        current_mode = BasicConfig.ENABLE_ACKERMANN_MODE
                        BasicConfig.ENABLE_ACKERMANN_MODE = not current_mode
                        controller.switch_kinematics_mode(BasicConfig.ENABLE_ACKERMANN_MODE)
                        mode_str = "Ackermann (4WS)" if (BasicConfig.ENABLE_ACKERMANN_MODE and BasicConfig.ACKERMANN_4WS) else \
                                   "Ackermann (2WS)" if BasicConfig.ENABLE_ACKERMANN_MODE else "Holonomic"
                        print(f"🔄 Motion mode toggled to: {mode_str}")
                        continue

                    # State-based control for W/S/A/D
                    if key == 'w':
                        current_speed_cmd = desired_speed
                        is_wsad = True
                    elif key == 's':
                        current_speed_cmd = -desired_speed
                        is_wsad = True
                    elif key == 'a':
                        steer_angle_deg = min(90.0, steer_angle_deg + 1.0) # Adjust angle Left (Step 1.0)
                        is_wsad = True
                        print(f" -> Steer Angle: {steer_angle_deg:.1f}°")
                    elif key == 'd':
                        steer_angle_deg = max(-90.0, steer_angle_deg - 1.0) # Adjust angle Right (Step 1.0)
                        is_wsad = True
                        print(f" -> Steer Angle: {steer_angle_deg:.1f}°")
                    elif key == ' ':
                        current_speed_cmd = 0.0
                        steer_angle_deg = 0.0
                        is_wsad = True
                        print(" -> Reset/Stop")

                    # Other controls (Immediate execution)
                    elif key == 'q':
                        radius = math.hypot(geometry.L/2, geometry.W/2)
                        spin_speed = MAX_OMEGA * radius
                        controller.spin_counter_clockwise(spin_speed)
                        is_moving = True
                        is_wsad = False # Spin overrides WSAD
                    elif key == 'e':
                        radius = math.hypot(geometry.L/2, geometry.W/2)
                        spin_speed = MAX_OMEGA * radius
                        controller.spin_clockwise(spin_speed)
                        is_moving = True
                        is_wsad = False
                    elif key == 'o':
                        MAX_OMEGA += 0.05
                        print(f" -> Max Omega Increased: {MAX_OMEGA:.2f} rad/s")
                    elif key == 'p':
                        MAX_OMEGA = max(0.0, MAX_OMEGA - 0.05)
                        print(f" -> Max Omega Decreased: {MAX_OMEGA:.2f} rad/s")
                    elif key == 'j':
                        current_left_angle += 15.0
                        controller._send_steer_pos(BasicConfig.FL_STEER_ID, current_left_angle)
                        print(f" -> Left Target: {current_left_angle}°")
                    elif key == 'k':
                        current_left_angle -= 15.0
                        controller._send_steer_pos(BasicConfig.FL_STEER_ID, current_left_angle)
                        print(f" -> Left Target: {current_left_angle}°")
                    elif key == 'u':
                        desired_speed += 0.1
                        if desired_speed > MAX_SPEED: desired_speed = MAX_SPEED
                        if desired_speed < 0.0: desired_speed = 0.0
                        print(f" -> Desired Speed Increased: {desired_speed:.1f} m/s")
                    elif key == 'i':
                        desired_speed -= 0.1
                        if desired_speed < 0.0: desired_speed = 0.0
                        print(f" -> Desired Speed Decreased: {desired_speed:.1f} m/s")

                # End of batch processing
                
                # Apply WSAD logic if triggered in this batch
                if is_wsad:
                    # Use core function instead of manual kinematics
                    if BasicConfig.ENABLE_ACKERMANN_MODE:
                         # In Ackermann mode, A/D acts as Steering Wheel
                         if abs(current_speed_cmd) > 0.01:
                             L = geometry.L
                             delta_rad = math.radians(steer_angle_deg)
                             omega = current_speed_cmd * math.tan(delta_rad) / L
                             # Limit Omega
                             omega = max(min(omega, MAX_OMEGA), -MAX_OMEGA)
                             if args.mode == "sim":
                                 # Pass explicit steer angle for simulation display updates
                                 # Calculate wheel angles for visualization
                                 vis_angles = {
                                     "FL": steer_angle_deg,
                                     "FR": steer_angle_deg,
                                     "RL": -steer_angle_deg if BasicConfig.ACKERMANN_4WS else 0.0,
                                     "RR": -steer_angle_deg if BasicConfig.ACKERMANN_4WS else 0.0
                                 }
                                 controller.chassis_move(steer_angle_deg, current_speed_cmd, omega, wheel_angles=vis_angles)
                             else:
                                 # Real robot uses omega for Ackermann, angle ignored (set to 0)
                                 controller.chassis_move(0.0, current_speed_cmd, omega)
                         else:
                             # Static Steering
                             wheel_states = {}
                             for name in ["FL", "FR", "RL", "RR"]:
                                 wheel_states[name] = (0.0, steer_angle_deg if "F" in name else (0.0 if not BasicConfig.ACKERMANN_4WS else -steer_angle_deg))
                             controller.apply_kinematics(wheel_states)
                    else:
                         # Holonomic Mode
                         if BasicConfig.ACKERMANN_4WS:
                             # 4WS: Crab Walk (All wheels steer)
                             controller.move_diagonal(steer_angle_deg, current_speed_cmd)
                         else:
                             # 2WS: Front wheels steer, Rear wheels straight
                             w_angles = {
                                 "FL": steer_angle_deg, "FR": steer_angle_deg,
                                 "RL": 0.0, "RR": 0.0
                             }
                             controller.chassis_move(steer_angle_deg, current_speed_cmd, 0.0, wheel_angles=w_angles)
                         
                    is_moving = True

                # Watchdog check: If no key for > WATCHDOG_TIMEOUT, Stop
                # Only check if joystick not active and no keys pressed in this batch
                elif is_moving and keys_batch == [None] and (now - last_cmd_time > WATCHDOG_TIMEOUT):
                    print(" -> Auto Stop (Key Released)")
                    vx, vy, omega = 0.0, 0.0, 0.0
                    # Stop but keep steering angle? Or reset?
                    # User said "adjust angle", maybe they want to keep it.
                    # But speed should be 0.
                    current_speed_cmd = 0.0
                    # steer_angle_deg = 0.0 # Keep angle for next move?
                    
                    wheel_states = {
                        "FL": (0.0, steer_angle_deg),
                        "FR": (0.0, steer_angle_deg)
                    }
                    # For Holonomic or 4WS, we should also update rear wheels to keep them aligned
                    if not BasicConfig.ENABLE_ACKERMANN_MODE:
                         # Holonomic
                         if BasicConfig.ACKERMANN_4WS:
                             # 4WS: All wheels aligned
                             wheel_states["RL"] = (0.0, steer_angle_deg)
                             wheel_states["RR"] = (0.0, steer_angle_deg)
                         else:
                             # 2WS: Rear wheels straight
                             wheel_states["RL"] = (0.0, 0.0)
                             wheel_states["RR"] = (0.0, 0.0)
                    elif BasicConfig.ACKERMANN_4WS:
                         # 4WS: Rear wheels opposed
                         wheel_states["RL"] = (0.0, -steer_angle_deg)
                         wheel_states["RR"] = (0.0, -steer_angle_deg)
                    else:
                         # 2WS: Rear wheels straight
                         wheel_states["RL"] = (0.0, 0.0)
                         wheel_states["RR"] = (0.0, 0.0)

                    controller.apply_kinematics(wheel_states)
                    is_moving = False

            # Debug print for ID 38 (FR_STEER_ID)
            if monitor and now - last_print_time_38 > 0.1:
                state_38 = monitor.get_state(BasicConfig.FR_STEER_ID)
                if state_38:
                    last_pos = state_38.get('last_pos', 0)
                    enc2 = state_38.get('enc2', None)
                    enc2_str = f"{enc2:.2f}" if enc2 is not None else "N/A"
                    if last_pos is None:
                        last_pos = 0.0
                    current_info = f"Current 38: Angle={state_38.get('total_angle', 0):.2f}, Turns={state_38.get('turns', 0)}, Raw={last_pos:.2f}, Enc2={enc2_str}"
                    print(f"--> [DEBUG ID 38] {current_info}")
                last_print_time_38 = now

            if dashboard_client and (now - last_dashboard_time) >= args.dashboard_interval:
                # 1. Check for commands from Dashboard (e.g. Mode Switch)
                dash_cmd = dashboard_client.get_command()
                if dash_cmd and "set_mode" in dash_cmd:
                    target_mode = dash_cmd["set_mode"]
                    should_be_ackermann = (target_mode == "ackermann")
                    if BasicConfig.ENABLE_ACKERMANN_MODE != should_be_ackermann:
                        BasicConfig.ENABLE_ACKERMANN_MODE = should_be_ackermann
                        controller.switch_kinematics_mode(BasicConfig.ENABLE_ACKERMANN_MODE)
                        print(f"🖥️ Dashboard set mode to: {target_mode}")

                if dash_cmd and "set_ackermann_type" in dash_cmd:
                    atype = dash_cmd["set_ackermann_type"]
                    is_4ws = (atype == "4ws")
                    if BasicConfig.ACKERMANN_4WS != is_4ws:
                        BasicConfig.ACKERMANN_4WS = is_4ws
                        if BasicConfig.ENABLE_ACKERMANN_MODE:
                            controller.switch_kinematics_mode(True)
                        print(f"🖥️ Dashboard set Ackermann type to: {atype}")

                # 2. Collect RPMs
                rpms = {}
                if args.mode == "real" and monitor:
                    # Helper to get RPM and Enc2
                    def get_val(mid, key):
                        s = monitor.get_state(mid)
                        return s.get(key, 0) if s else 0
                    
                    rpms = {
                        "FL_drive": get_val(BasicConfig.FL_DRIVE_ID, "rpm"),
                        "FL_steer": get_val(BasicConfig.FL_STEER_ID, "rpm"),
                        "FL_enc2": get_val(BasicConfig.FL_STEER_ID, "enc2"),
                        "FR_drive": get_val(BasicConfig.FR_DRIVE_ID, "rpm"),
                        "FR_steer": get_val(BasicConfig.FR_STEER_ID, "rpm"),
                        "FR_enc2": get_val(BasicConfig.FR_STEER_ID, "enc2"),
                        "RL_drive": get_val(BasicConfig.RL_DRIVE_ID, "rpm"),
                        "RL_steer": get_val(BasicConfig.RL_STEER_ID, "rpm"),
                        "RL_enc2": get_val(BasicConfig.RL_STEER_ID, "enc2"),
                        "RR_drive": get_val(BasicConfig.RR_DRIVE_ID, "rpm"),
                        "RR_steer": get_val(BasicConfig.RR_STEER_ID, "rpm"),
                        "RR_enc2": get_val(BasicConfig.RR_STEER_ID, "enc2"),
                    }
                else:
                    # Sim mode: Approximate RPM from speed (v = omega * r * 0.105 -> rpm = v / (2pi*r) * 60)
                    # For visualization, just show something relative to speed
                    sim_rpm = int(current_speed_cmd * 1000)
                    
                    # Get simulated wheel angles if available
                    w_angles = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
                    if hasattr(controller, "state") and hasattr(controller.state, "wheel_angles"):
                        w_angles = controller.state.wheel_angles

                    rpms = {
                        "FL_drive": sim_rpm, "FL_steer": 0, "FL_enc2": w_angles.get("FL", 0.0),
                        "FR_drive": sim_rpm, "FR_steer": 0, "FR_enc2": w_angles.get("FR", 0.0),
                        "RL_drive": sim_rpm, "RL_steer": 0, "RL_enc2": w_angles.get("RL", 0.0),
                        "RR_drive": sim_rpm, "RR_steer": 0, "RR_enc2": w_angles.get("RR", 0.0)
                    }

                # 3. Send State
                if args.mode == "sim" and hasattr(controller, "get_state"):
                    state = controller.get_state()
                    # Inject extra data
                    state["rpms"] = rpms
                    state["mode"] = "ackermann" if BasicConfig.ENABLE_ACKERMANN_MODE else "holonomic"
                    state["ackermann_type"] = "4ws" if BasicConfig.ACKERMANN_4WS else "2ws"
                else:
                    state = {
                        "x": 0.0,
                        "y": 0.0,
                        "heading_deg": steer_angle_deg,
                        "steer_angle_deg": steer_angle_deg,
                        "speed_mps": current_speed_cmd,
                        "throttle_pct": (abs(current_speed_cmd) / MAX_SPEED * 100.0) if MAX_SPEED > 0 else 0.0,
                        "timestamp": now,
                        "rpms": rpms,
                        "mode": "ackermann" if BasicConfig.ENABLE_ACKERMANN_MODE else "holonomic",
                        "ackermann_type": "4ws" if BasicConfig.ACKERMANN_4WS else "2ws"
                    }
                dashboard_client.send_state(state)
                last_dashboard_time = now

            time.sleep(0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping...")
        if monitor:
            monitor.stop()
        if os.name != 'nt' and linux_old_settings:
            restore_linux_term(linux_old_settings)

if __name__ == "__main__":
    main()
