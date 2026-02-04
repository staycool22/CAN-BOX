import time
import sys
import os
import argparse

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

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
    import select
    import tty
    import termios
    
    def get_key():
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
             return sys.stdin.read(1).lower()
        return None
    
    def setup_linux_term():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        return old_settings

    def restore_linux_term(old_settings):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)

from actuators.chassis_system import ChassisSystem
from config.steer_wheel_config import config

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["real", "sim"], default="real")
    parser.add_argument("--input", choices=["keyboard", "joystick", "both"], default="keyboard")
    parser.add_argument("--joystick", default="/dev/input/js0")
    parser.add_argument("--dashboard", default=None)
    parser.add_argument("--dashboard-host", default="0.0.0.0")
    parser.add_argument("--dashboard-port", type=int, default=8080)
    parser.add_argument("--dashboard-enable", action="store_true", default=False)
    args = parser.parse_args()
    
    # ---------------------------------------------------------
    # 1. INITIALIZE ROBOT SYSTEM (Central Orchestrator)
    # ---------------------------------------------------------
    # Note: RobotSystem handles VESC, Joystick, Dashboard, and Control Loop internally.
    chassis_system = ChassisSystem(
        mode=args.mode,
        dashboard_host=args.dashboard_host,
        dashboard_port=args.dashboard_port,
        enable_dashboard=args.dashboard_enable,
        joystick_dev=args.joystick,
        enable_joystick=(args.input in ["joystick", "both"])
    )
    
    chassis_system.start()
    
    # ---------------------------------------------------------
    # 2. TEST SCRIPT LOGIC (Client / Producer)
    # ---------------------------------------------------------
    print("Steering Control Test Script (Client Mode)")
    print("Controls:")
    print("  w : Forward")
    print("  s : Backward")
    print("  a : Left Strafe / Steer Left")
    print("  d : Right Strafe / Steer Right")
    print("  q : Rotate Left")
    print("  e : Rotate Right")
    print("  SPACE : Emergency Stop")
    print("  u/i : Adjust Desired Speed")
    print("  o/p : Adjust Max Omega")
    print("  m : Toggle Mode (Ackermann/Holonomic)")
    print("  z : Quit")
    
    # Linux terminal settings
    linux_old_settings = None
    if os.name != 'nt':
        linux_old_settings = setup_linux_term()
        
    # Local State for Test Script
    input_desired_speed = 0.1
    steer_angle_deg = 0.0
    last_key_time = time.time()
    
    # Get direct access to robot state to send commands
    # In a real networked scenario, this would use an API client.
    # Here we use shared memory reference.
    chassis_state = chassis_system.state
    
    try:
        while True:
            now = time.time()
            # --- Batch Read Keyboard ---
            keys_batch = []
            if args.input in ("keyboard", "both"):
                for _ in range(10):
                    k = get_key()
                    if k is None: break
                    keys_batch.append(k)
            
            # --- Process Keys ---
            input_updated = False
            target_speed = 0.0
            target_omega = 0.0
            
            if keys_batch:
                last_key_time = now
            
            current_params = chassis_state.get_params()
            MAX_SPEED = current_params.max_speed
            MAX_OMEGA = current_params.max_omega
            
            for key in keys_batch:
                if not key: continue
                
                if key == 'z': raise KeyboardInterrupt
                
                # Mode Switch
                if key == 'm':
                    config.ENABLE_ACKERMANN_MODE = not config.ENABLE_ACKERMANN_MODE
                    # We need to notify system about mode change? 
                    # Actually RobotSystem reads config in its loop, or we can use dashboard client command
                    # But config is global shared memory here.
                    # Ideally we should send a command. 
                    # Let's assume shared config works for now as it's same process.
                    mode_str = "Ackermann" if config.ENABLE_ACKERMANN_MODE else "Holonomic"
                    print(f"🔄 Mode: {mode_str}")
                    continue
                    
                # WSAD
                if key == 'w':
                    target_speed = input_desired_speed
                    input_updated = True
                elif key == 's':
                    target_speed = -input_desired_speed
                    input_updated = True
                elif key == 'a':
                    steer_angle_deg = min(90.0, steer_angle_deg + 1.0)
                    input_updated = True
                    # If stopped, apply steering immediately without speed
                    if abs(target_speed) < 0.001:
                         chassis_state.set_control_command(0.0, steer_angle_deg, 0.0)
                    print(f" -> Steer: {steer_angle_deg:.1f}°")
                elif key == 'd':
                    steer_angle_deg = max(-90.0, steer_angle_deg - 1.0)
                    input_updated = True
                    # If stopped, apply steering immediately without speed
                    if abs(target_speed) < 0.001:
                         chassis_state.set_control_command(0.0, steer_angle_deg, 0.0)
                    print(f" -> Steer: {steer_angle_deg:.1f}°")
                elif key == ' ':
                    target_speed = 0.0
                    steer_angle_deg = 0.0
                    input_updated = True
                    chassis_state.set_control_command(0.0, 0.0, 0.0, emergency_stop=True)
                    print(" -> Reset/Stop")
                    
                # Spin
                elif key == 'q':
                    target_omega = MAX_OMEGA
                    input_updated = True
                elif key == 'e':
                    target_omega = -MAX_OMEGA
                    input_updated = True
                    
                # Params
                elif key == 'o':
                    chassis_state.update_params(max_omega=MAX_OMEGA + 0.05)
                    print(f" -> Max Omega: {MAX_OMEGA + 0.05:.2f} rad/s")
                elif key == 'p':
                    chassis_state.update_params(max_omega=max(0.0, MAX_OMEGA - 0.05))
                    print(f" -> Max Omega: {MAX_OMEGA - 0.05:.2f} rad/s")
                elif key == 'u':
                    input_desired_speed = min(MAX_SPEED, input_desired_speed + 0.1)
                    print(f" -> Desired Speed: {input_desired_speed:.1f} m/s")
                elif key == 'i':
                    input_desired_speed = max(0.0, input_desired_speed - 0.1)
                    print(f" -> Desired Speed: {input_desired_speed:.1f} m/s")

            # --- Send Command ---
            if input_updated:
                if abs(target_omega) > 0.001:
                    chassis_state.set_control_command(0.0, 0.0, target_omega)
                elif not any(k == ' ' for k in keys_batch if k):
                    chassis_state.set_control_command(target_speed, steer_angle_deg, 0.0)
            
            # Watchdog for Keyboard (Auto-Stop on Key Release)
            # If no input for 0.2s, and we are not already stopped, send stop command.
            if args.input in ("keyboard", "both") and not input_updated and (now - last_key_time > 0.2):
                current_cmd = chassis_state.get_control_command()
                # Check if we are moving
                if abs(current_cmd.speed_mps) > 0.001 or abs(current_cmd.omega_rad) > 0.001:
                    print("🛑 Keyboard Idle - Stopping Chassis")
                    chassis_state.set_control_command(0.0, 0.0, 0.0)
            
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping Test Script...")
        chassis_system.stop()
        if os.name != 'nt' and linux_old_settings:
            restore_linux_term(linux_old_settings)

if __name__ == "__main__":
    main()
