import time
import sys
import os
import threading
import json
import socket
import argparse
import math

# Platform specific imports for keyboard input
if os.name == 'nt':
    import msvcrt
else:
    import select
    import termios
    import tty

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from Steering_wheel_chassis_new import VESCMonitor, SteerController, BasicConfig
from chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics

# UDP Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 8080
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_telemetry(monitor, target_speed, steer_angle, vx=0.0, vy=0.0, omega=0.0):
    # Collect data
    data = {
        "speed": 0.0, # Estimated actual speed
        "target_speed": target_speed,
        "vx": vx,
        "vy": vy,
        "omega": omega,
        "steer_angle": steer_angle,
        "accel": BasicConfig.DRIVE_ACCEL,
        "decel": BasicConfig.DRIVE_DECEL,
        "motors": {}
    }
    
    # Calculate average speed from drive motors
    drive_speeds = []
    for mid in BasicConfig.get_drive_ids():
        state = monitor.get_state(mid)
        rpm = state.get("rpm", 0.0)
        # RPM to m/s
        speed = (rpm / 60.0) * (2 * 3.14159 * BasicConfig.DRIVE_WHEEL_RADIUS) / BasicConfig.DRIVE_REDUCTION_RATIO
        drive_speeds.append(speed)
        
        data["motors"][mid] = {
            "rpm": rpm,
            "current": state.get("current", 0.0),
            "temp": state.get("temp_motor", 0.0)
        }
        
    for mid in BasicConfig.get_steer_ids():
        state = monitor.get_state(mid)
        data["motors"][mid] = {
            "angle": state.get("total_angle", 0.0),
            "rpm": state.get("rpm", 0.0),
            "current": state.get("current", 0.0)
        }
        
    if drive_speeds:
        data["speed"] = sum(drive_speeds) / len(drive_speeds)
        
    try:
        sock.sendto(json.dumps(data).encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception:
        pass

def main():
    parser = argparse.ArgumentParser(description='Vehicle Control Script')
    parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
    args = parser.parse_args()

    print(f"Initializing VESC Monitor (Simulation: {args.sim})...")
    
    # Initialize Kinematics
    geometry = ChassisGeometry(length=0.44, width=0.30, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
    kinematics = FourWheelSteeringKinematics(geometry)
    
    print("Controls:")
    print("  w : Forward")
    print("  s : Backward")
    print("  a : Left Strafe")
    print("  d : Right Strafe")
    print("  q : Rotate Left")
    print("  e : Rotate Right")
    print("  t : Diagonal Front-Left")
    print("  y : Diagonal Front-Right")
    print("  g : Diagonal Back-Left")
    print("  h : Diagonal Back-Right")
    print("  o : Diagonal Out")
    print("  p : Diagonal In")
    print("  SPACE : Stop")
    print("  u : Increase Max Speed (+0.1 m/s)")
    print("  i : Decrease Max Speed (-0.1 m/s)")
    print("  v/b : +/- Accel")
    print("  n/m : +/- Decel")
    print("  z : Quit")
    
    # Initialize logical angles
    current_left_angle = 0.0
    
    # Speed params
    MAX_SPEED = 8.0 # m/s
    MAX_OMEGA = 5 # rad/s
    
    # Diagonal Angle
    DIAG_ANGLE_DEG = 45.0
    
    monitor = VESCMonitor(simulation=args.sim)
    monitor.perform_zero_calibration = lambda: None
    monitor.start()

    if not args.sim:
        print("Waiting for VESC data...")
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
                print(f"VESC data received: {active_ids}")
                break
            time.sleep(0.1)
    else:
        print("Simulation Mode: Skipping VESC wait.")

    controller = SteerController(monitor)
    controller.set_accel_decel(3.5, 4.0)

    # Keyboard input setup
    if os.name != 'nt':
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

    def read_key():
        key = None
        if os.name == 'nt':
            if msvcrt.kbhit():
                char = msvcrt.getch()
                try:
                    key = char.decode('utf-8').lower()
                except UnicodeDecodeError:
                    pass
        else:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                ch = sys.stdin.read(1)
                key = ch.lower()
        return key

    def reset_steer_to_zero():
        for mid in BasicConfig.get_steer_ids():
            current_angle = monitor.get_state(mid).get("total_angle", 0.0)
            controller._send_steer_pos(mid, current_angle)

    last_cmd_time = time.time()
    is_moving = False
    WATCHDOG_TIMEOUT = 0.2
    
    target_vx = 0.0
    
    # Current kinematics state for telemetry
    curr_vx, curr_vy, curr_omega = 0.0, 0.0, 0.0
    
    # Key history for detecting simultaneous presses (diagonal movement)
    key_history = {} # {key: timestamp}
    # Increased retention to allow easier diagonal triggering in terminal
    KEY_retention = 0.5 # Seconds to keep a key "active" (increased from 0.25s)
    
    diag_latch = None
    
    try:
        while True:
            # Drain all keys from buffer to handle rapid typing (simulated simultaneous press)
            while True:
                key = read_key()
                if key:
                    key_history[key] = time.time()
                    if key == 'z':
                        raise KeyboardInterrupt
                else:
                    break
            
            now = time.time()
            
            # Prune old keys
            active_keys = [k for k, t in key_history.items() if now - t < KEY_retention]
            # Update history to only keep active ones
            key_history = {k: key_history[k] for k in active_keys}

            if active_keys:
                last_cmd_time = now
                
                vx, vy, omega = 0.0, 0.0, 0.0
                active_kinematics = False
                
                # Check Dual Key Combinations for Diagonal Movement (High Priority)
                # WA: Forward-Left
                is_wa = ('w' in active_keys and 'a' in active_keys)
                is_wd = ('w' in active_keys and 'd' in active_keys)
                is_sa = ('s' in active_keys and 'a' in active_keys)
                is_sd = ('s' in active_keys and 'd' in active_keys)

                # Update Latch
                if is_wa: diag_latch = 'wa'
                elif is_wd: diag_latch = 'wd'
                elif is_sa: diag_latch = 'sa'
                elif is_sd: diag_latch = 'sd'
                
                # Check Latch Validity (Sticky Logic)
                # If latch is set, we keep it as long as AT LEAST ONE of the constituent keys is active.
                # This handles the case where one key stops repeating in the terminal stream.
                if diag_latch:
                    keep_latch = False
                    if diag_latch == 'wa':
                        if 'w' in active_keys or 'a' in active_keys: keep_latch = True
                    elif diag_latch == 'wd':
                        if 'w' in active_keys or 'd' in active_keys: keep_latch = True
                    elif diag_latch == 'sa':
                        if 's' in active_keys or 'a' in active_keys: keep_latch = True
                    elif diag_latch == 'sd':
                        if 's' in active_keys or 'd' in active_keys: keep_latch = True
                    
                    if not keep_latch:
                        diag_latch = None

                # Execute based on Latch or Direct Combinations
                if diag_latch == 'wa':
                    angle_rad = math.radians(DIAG_ANGLE_DEG)
                    vx = MAX_SPEED * math.cos(angle_rad)
                    vy = MAX_SPEED * math.sin(angle_rad)
                    active_kinematics = True
                
                elif diag_latch == 'wd':
                    angle_rad = math.radians(DIAG_ANGLE_DEG)
                    vx = MAX_SPEED * math.cos(angle_rad)
                    vy = -MAX_SPEED * math.sin(angle_rad)
                    active_kinematics = True
                    
                elif diag_latch == 'sa':
                    angle_rad = math.radians(DIAG_ANGLE_DEG)
                    vx = -MAX_SPEED * math.cos(angle_rad)
                    vy = MAX_SPEED * math.sin(angle_rad)
                    active_kinematics = True
                    
                elif diag_latch == 'sd':
                    angle_rad = math.radians(DIAG_ANGLE_DEG)
                    vx = -MAX_SPEED * math.cos(angle_rad)
                    vy = -MAX_SPEED * math.sin(angle_rad)
                    active_kinematics = True
                
                # Check single keys if no diagonal combination
                if not active_kinematics:
                    # Forward/Backward
                    if 'w' in active_keys and 's' not in active_keys:
                        vx = MAX_SPEED
                        active_kinematics = True
                    elif 's' in active_keys and 'w' not in active_keys:
                        vx = -MAX_SPEED
                        active_kinematics = True
                    
                    # Left/Right (Strafe)
                    if 'a' in active_keys and 'd' not in active_keys:
                        vy = MAX_SPEED
                        active_kinematics = True
                    elif 'd' in active_keys and 'a' not in active_keys:
                        vy = -MAX_SPEED
                        active_kinematics = True
                    
                    # Rotation
                    if 'q' in active_keys:
                        omega = MAX_OMEGA
                        active_kinematics = True
                    elif 'e' in active_keys:
                        omega = -MAX_OMEGA
                        active_kinematics = True

                # Diagonal Movement (Independent Keys)
                if not active_kinematics:
                    # Normalize speed so total velocity is MAX_SPEED (not sqrt(2)*MAX_SPEED)
                    diag_speed = MAX_SPEED / math.sqrt(2)
                    
                    if 't' in active_keys: # Front-Left
                        vx = diag_speed
                        vy = diag_speed
                        active_kinematics = True
                    elif 'y' in active_keys: # Front-Right
                        vx = diag_speed
                        vy = -diag_speed
                        active_kinematics = True
                    elif 'g' in active_keys: # Back-Left
                        vx = -diag_speed
                        vy = diag_speed
                        active_kinematics = True
                    elif 'h' in active_keys: # Back-Right
                        vx = -diag_speed
                        vy = -diag_speed
                        active_kinematics = True

                # Special discrete modes (override continuous logic if pressed alone or explicitly)
                if ' ' in active_keys:
                    vx, vy, omega = 0.0, 0.0, 0.0
                    active_kinematics = True # Will result in stop
                    key_history.clear() # Clear all keys on stop
                
                # Param adjustments
                if 'u' in active_keys:
                    MAX_SPEED += 0.1
                    print(f" -> Max Speed Increased: {MAX_SPEED:.1f} m/s")
                    key_history.pop('u', None)
                elif 'i' in active_keys:
                    MAX_SPEED = max(0.0, MAX_SPEED - 0.1)
                    print(f" -> Max Speed Decreased: {MAX_SPEED:.1f} m/s")
                    key_history.pop('i', None)

                elif 'v' in active_keys:
                    BasicConfig.DRIVE_ACCEL += 0.1
                    print(f"Accel: {BasicConfig.DRIVE_ACCEL:.1f}")
                    key_history.pop('v', None) # Consume key
                elif 'b' in active_keys:
                    BasicConfig.DRIVE_ACCEL = max(0.1, BasicConfig.DRIVE_ACCEL - 0.1)
                    print(f"Accel: {BasicConfig.DRIVE_ACCEL:.1f}")
                    key_history.pop('b', None)
                elif 'n' in active_keys:
                    BasicConfig.DRIVE_DECEL += 0.1
                    print(f"Decel: {BasicConfig.DRIVE_DECEL:.1f}")
                    key_history.pop('n', None)
                elif 'm' in active_keys:
                    BasicConfig.DRIVE_DECEL = max(0.1, BasicConfig.DRIVE_DECEL - 0.1)
                    print(f"Decel: {BasicConfig.DRIVE_DECEL:.1f}")
                    key_history.pop('m', None)
                    
                # Diagonal Angle Adjustment
                elif 'j' in active_keys:
                    DIAG_ANGLE_DEG = min(85.0, DIAG_ANGLE_DEG + 5.0)
                    print(f" -> Diag Angle Increased: {DIAG_ANGLE_DEG:.1f}°")
                    key_history.pop('j', None)
                elif 'k' in active_keys:
                    DIAG_ANGLE_DEG = max(5.0, DIAG_ANGLE_DEG - 5.0)
                    print(f" -> Diag Angle Decreased: {DIAG_ANGLE_DEG:.1f}°")
                    key_history.pop('k', None)


                if active_kinematics:
                    # Apply kinematics
                    # Note: 4WS kinematics handles Vx and Vy combination automatically
                    wheel_states = kinematics.inverse_kinematics(vx, vy, omega)
                    controller.apply_kinematics(wheel_states)
                    is_moving = True
                    curr_vx, curr_vy, curr_omega = vx, vy, omega
                    target_vx = vx # Approximate target for display

            elif is_moving and (now - last_cmd_time > WATCHDOG_TIMEOUT):
                # Auto stop
                wheel_states = kinematics.inverse_kinematics(0.0, 0.0, 0.0)
                controller.apply_kinematics(wheel_states)
                
                curr_vx, curr_vy, curr_omega = 0.0, 0.0, 0.0
                
                # Check if stopped (Simulated or Real)
                all_stopped = True
                for drive_id in BasicConfig.get_drive_ids():
                    rpm = monitor.get_state(drive_id).get("rpm", 0.0)
                    if abs(rpm) > 10.0:
                        all_stopped = False
                        break
                
                if all_stopped:
                    is_moving = False
                    target_vx = 0.0
            
            # Send Telemetry
            # Get steering angle from FL wheel for display
            fl_state = monitor.get_state(BasicConfig.FL_STEER_ID)
            current_steer = fl_state.get("total_angle", 0.0)
            
            # Pass kinematics state
            send_telemetry(monitor, target_vx if is_moving else 0.0, current_steer, curr_vx, curr_vy, curr_omega)
            
            time.sleep(0.02) # 50Hz

    except KeyboardInterrupt:
        pass
    finally:
        if os.name != 'nt':
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("Stopping...")
        monitor.stop()

if __name__ == "__main__":
    main()
