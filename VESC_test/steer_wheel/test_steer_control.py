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

from Steering_wheel_chassis_0130 import VESCMonitor, SteerController
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
    geometry = ChassisGeometry(length=0.30, width=0.44, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
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
    MAX_SPEED = 5.0 # m/s
    MAX_OMEGA = 1 # rad/s
    
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
            key = get_key() if args.input in ("keyboard", "both") else None
            
            # Current loop time
            now = time.time()

            handled_input = False
            if joystick:
                joystick.poll()
                joy_cmd = joystick.get_command()
                if joy_cmd["emergency_stop"]:
                    current_speed_cmd = 0.0
                    steer_angle_deg = 0.0
                    wheel_states = {
                        "FL": (0.0, 0.0),
                        "FR": (0.0, 0.0)
                    }
                    controller.apply_kinematics(wheel_states)
                    last_wheel_states = wheel_states
                    is_moving = False
                    handled_input = True
                    last_cmd_time = now
                elif joy_cmd["active_stick"] or abs(joy_cmd["speed"]) > 0.01 or abs(joy_cmd.get("spin", 0.0)) > 0.01:
                    # Spin Mode (Right Stick Horizontal)
                    spin_cmd = joy_cmd.get("spin", 0.0)
                    if abs(spin_cmd) > 0.01:
                        # Use Kinematics for Spin (and optional forward/back)
                        vx = MAX_SPEED * joy_cmd["speed"]
                        vy = 0.0
                        # RX > 0 (Right) -> Turn Right -> Omega < 0
                        omega = -spin_cmd * MAX_OMEGA
                        
                        wheel_states = kinematics.inverse_kinematics(vx, vy, omega)
                        
                        # [DEBUG JOY SPIN]
                        radius = BasicConfig.DRIVE_WHEEL_RADIUS
                        debug_msg = f" [DEBUG JOY SPIN] SpinCmd={spin_cmd:.2f}"
                        for name in ['FL', 'FR']:
                            if name in wheel_states:
                                v_target, angle_target = wheel_states[name]
                                rpm_target = (v_target / (2 * math.pi * radius)) * 60 * BasicConfig.DRIVE_POLE_PAIRS
                                debug_msg += f" {name}={v_target:.2f}m/s({rpm_target:.1f}ERPM)@{angle_target:.1f}°"
                        print(debug_msg)

                        controller.apply_kinematics(wheel_states)
                        
                        current_speed_cmd = vx
                        # Note: In spin mode, individual wheel angles vary, so 'steer_angle_deg' is ambiguous.
                        # We don't update 'joy_last_angle' here to preserve the last explicit steering setting.
                        
                    else:
                        # Normal Mode (Explicit Angle + Speed)
                        if joy_cmd["active_stick"] and joy_cmd["angle_deg"] is not None:
                            # New Mapping: Joystick 0(Left)->180(Right)
                            # Steering: +90(Left) -> -90(Right)
                            # Formula: 90 - Input
                            target_steer = 90.0 - joy_cmd["angle_deg"]
                            joy_last_angle = target_steer
                        else:
                            # Auto-center steering when stick is released
                            joy_last_angle = 0.0
    
                        steer_angle_deg = joy_last_angle
                        
                        # Speed from Right Stick (Up=Positive, Down=Negative)
                        current_speed_cmd = MAX_SPEED * joy_cmd["speed"]
                        
                        wheel_states = {
                            "FL": (current_speed_cmd, steer_angle_deg),
                            "FR": (current_speed_cmd, steer_angle_deg)
                        }
                        controller.apply_kinematics(wheel_states)
                        
                    last_wheel_states = wheel_states
                    is_moving = True
                    handled_input = True
                    last_cmd_time = now

            if key and not handled_input:
                last_cmd_time = now # Update watchdog
                
                if key == 'z':
                    break
                
                # Kinematics Controls
                vx, vy, omega = 0.0, 0.0, 0.0
                active_kinematics = False
                
                # State-based control for W/S/A/D
                is_wsad = False
                
                if key == 'w':
                    current_speed_cmd = desired_speed
                    is_wsad = True
                elif key == 's':
                    current_speed_cmd = -desired_speed
                    is_wsad = True
                elif key == 'a':
                    steer_angle_deg = min(90.0, steer_angle_deg + 1.0) # Adjust angle Left
                    is_wsad = True
                    print(f" -> Steer Angle: {steer_angle_deg:.1f}°")
                elif key == 'd':
                    steer_angle_deg = max(-90.0, steer_angle_deg - 1.0) # Adjust angle Right
                    is_wsad = True
                    print(f" -> Steer Angle: {steer_angle_deg:.1f}°")
                elif key == ' ':
                    current_speed_cmd = 0.0
                    steer_angle_deg = 0.0
                    is_wsad = True
                    print(" -> Reset/Stop")

                if is_wsad:
                    # apply_kinematics expects degrees, not radians
                    wheel_states = {
                        "FL": (current_speed_cmd, steer_angle_deg),
                        "FR": (current_speed_cmd, steer_angle_deg)
                    }
                    controller.apply_kinematics(wheel_states)
                    last_wheel_states = wheel_states
                    is_moving = True
                    continue

                # Other controls (override state)
                if key == 'q':
                    omega = MAX_OMEGA
                    active_kinematics = True
                    # print(f" -> Rotate Left (w={omega})")
                elif key == 'e':
                    omega = -MAX_OMEGA
                    active_kinematics = True
                    # print(f" -> Rotate Right (w={omega})")
                elif key == 'o':
                    MAX_OMEGA += 0.05
                    print(f" -> Max Omega Increased: {MAX_OMEGA:.2f} rad/s")
                    continue
                elif key == 'p':
                    MAX_OMEGA = max(0.0, MAX_OMEGA - 0.05)
                    print(f" -> Max Omega Decreased: {MAX_OMEGA:.2f} rad/s")
                    continue
                elif key == ' ':
                    vx, vy, omega = 0.0, 0.0, 0.0
                    active_kinematics = True
                    print(" -> Stop")
                
                if active_kinematics:
                    # Calculate wheel states
                    # Note: inverse_kinematics returns DEGREES
                    wheel_states = kinematics.inverse_kinematics(vx, vy, omega)
                    
                    # [调试] 打印驱动电机的期望速度和 RPM
                    # RPM = (v / (2*pi*r)) * 60 * PolePairs (ERPM)
                    radius = BasicConfig.DRIVE_WHEEL_RADIUS
                    
                    debug_msg = " [DEBUG EXP]"
                    for name in ['FL', 'FR']:
                        if name in wheel_states:
                            v_target, _ = wheel_states[name]
                            rpm_target = (v_target / (2 * math.pi * radius)) * 60 * BasicConfig.DRIVE_POLE_PAIRS
                            debug_msg += f" {name}={v_target:.2f}m/s({rpm_target:.1f}ERPM)"
                    print(debug_msg)

                    # Apply to motors
                    # 这里是发送指令的核心入口
                    # apply_kinematics 内部会调用 self.vesc_drive.send_rpm 向 FL_DRIVE_ID/FR_DRIVE_ID 发送速度
                    controller.apply_kinematics(wheel_states)
                    
                    # [用户自定义修改区域]
                    # 如果您想绕过运动学计算，直接向驱动电机发送指定转速，可以注释掉上面的 apply_kinematics，
                    # 并使用以下代码 (注意 ID 号需导入或硬编码):
                    # FL_ID = 0x32F # BasicConfig.FL_DRIVE_ID
                    # FR_ID = 0x320 # BasicConfig.FR_DRIVE_ID
                    # if controller.vesc_drive:
                    #     controller.vesc_drive.send_rpm(FL_ID, 1000)  # 发送 1000 RPM
                    #     controller.vesc_drive.send_rpm(FR_ID, -1000) # 右轮通常反向
                    
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
                    desired_speed = min(MAX_SPEED, desired_speed + 0.1)
                    print(f" -> Desired Speed Increased: {desired_speed:.1f} m/s (Max {MAX_SPEED:.1f})")
                    
                elif key == 'i':
                    desired_speed = max(0.0, desired_speed - 0.1)
                    print(f" -> Desired Speed Decreased: {desired_speed:.1f} m/s (Max {MAX_SPEED:.1f})")

            # Watchdog check: If no key for > WATCHDOG_TIMEOUT, Stop
            elif is_moving and (now - last_cmd_time > WATCHDOG_TIMEOUT):
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
                controller.apply_kinematics(wheel_states)
                is_moving = False

            # Debug print for ID 38 (FR_STEER_ID)
            if monitor and now - last_print_time_38 > 0.1:
                state_38 = monitor.get_state(BasicConfig.FR_STEER_ID)
                if state_38:
                    last_pos = state_38.get('last_pos', 0)
                    if last_pos is None:
                        last_pos = 0.0
                    current_info = f"Current 38: Angle={state_38.get('total_angle', 0):.2f}, Turns={state_38.get('turns', 0)}, Raw={last_pos:.2f}"
                    print(f"--> [DEBUG ID 38] {current_info}")
                last_print_time_38 = now

            if dashboard_client and (now - last_dashboard_time) >= args.dashboard_interval:
                if args.mode == "sim" and hasattr(controller, "get_state"):
                    state = controller.get_state()
                else:
                    state = {
                        "x": 0.0,
                        "y": 0.0,
                        "heading_deg": steer_angle_deg,
                        "steer_angle_deg": steer_angle_deg,
                        "speed_mps": current_speed_cmd,
                        "throttle_pct": (abs(current_speed_cmd) / MAX_SPEED * 100.0) if MAX_SPEED > 0 else 0.0,
                        "timestamp": now
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
