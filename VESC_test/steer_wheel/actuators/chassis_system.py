import threading
import time
import math
import os
import sys

# Ensure path is correct
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

from core.chassis_state import ChassisState
from config.steer_wheel_config import config
from actuators.Steering_wheel_chassis_0203 import VESCMonitor, VESCControlLoop, SteerController
from ui.dashboard_client import DashboardClient, SimulatedSteerController, start_dashboard_server
from algorithm.chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics
from driver.joystick_controller import JoystickController

class ChassisSystem:
    def __init__(self, mode="real", dashboard_host="0.0.0.0", dashboard_port=8080, enable_dashboard=False, joystick_dev="/dev/input/js0", enable_joystick=True):
        self.mode = mode
        self.enable_dashboard = enable_dashboard
        self.enable_joystick = enable_joystick
        self.running = False
        
        # Initialize ChassisState (Singleton)
        self.state = ChassisState()
        
        # Initialize Hardware/Sim
        print(f"Initializing Robot System (Mode: {self.mode})...")
        self.geometry = ChassisGeometry(length=config.CHASSIS_LENGTH, width=config.CHASSIS_WIDTH, wheel_radius=config.DRIVE_WHEEL_RADIUS)
        
        if self.mode == "real":
            self.monitor = VESCMonitor()
            self.monitor.start()
            self.control_loop = VESCControlLoop(self.monitor)
            self.control_loop.start()
            self.controller = SteerController(self.monitor)
        else:
            self.monitor = None
            self.control_loop = None
            self.controller = SimulatedSteerController()
            print("✅ Simulation Mode Enabled")

        # Initialize Dashboard
        self.dashboard_client = None
        if self.enable_dashboard:
            # Start local server if needed, or just connect
            # For simplicity, we assume server is started here if local
            self.dash_server, self.dash_thread, self.dash_store = start_dashboard_server(host=dashboard_host, port=dashboard_port)
            self.dashboard_client = DashboardClient(f"http://127.0.0.1:{dashboard_port}")
            print(f"✅ Dashboard Server Started at http://{dashboard_host}:{dashboard_port}")

        # Initialize Joystick
        self.joystick = None
        self.last_joy_active = False  # Track joystick state for auto-stop
        if self.enable_joystick:
            try:
                self.joystick = JoystickController(joystick_dev, control_mode_360=True)
                print(f"✅ Joystick Connected: {joystick_dev}")
            except Exception as e:
                print(f"⚠️ Joystick Init Failed: {e}")

        # Control Loop State
        self.current_physical_speed = 0.0
        self.last_loop_time = time.time()
        self.accel_limit = 0.5 # m/s^2
        self.enable_smooth_braking = True
        
        self.thread = threading.Thread(target=self._control_loop, daemon=True)

    def start(self):
        if not self.running:
            self.running = True
            self.thread.start()
            print("🚀 Chassis System Started")

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        
        if self.controller:
            self.controller.stop()
        if self.control_loop:
            self.control_loop.stop()
        if self.monitor:
            self.monitor.stop()
        if self.joystick:
            self.joystick.close()
        print("🛑 Chassis System Stopped")

    def _control_loop(self):
        while self.running:
            now = time.time()
            dt = now - self.last_loop_time
            self.last_loop_time = now
            
            # ---------------------------------------------------------
            # 1. INPUT PHASE (Joystick Only - Keyboard comes from external via RobotState)
            # ---------------------------------------------------------
            if self.joystick:
                self.joystick.poll()
                joy_cmd = self.joystick.get_command()
                
                # Priority: Joystick E-Stop overrides everything
                if joy_cmd["emergency_stop"]:
                    self.state.set_control_command(0.0, 0.0, 0.0, emergency_stop=True)
                
                elif joy_cmd.get("switch_mode"):
                    # Mode switching logic (Debounced)
                    if not hasattr(self, "_last_joy_mode_time"): self._last_joy_mode_time = 0
                    if now - self._last_joy_mode_time > 0.5:
                        config.ENABLE_ACKERMANN_MODE = not config.ENABLE_ACKERMANN_MODE
                        self.controller.switch_kinematics_mode(config.ENABLE_ACKERMANN_MODE)
                        print(f"🎮 Joystick Toggled Mode to: {'Ackermann' if config.ENABLE_ACKERMANN_MODE else 'Holonomic'}")
                        self._last_joy_mode_time = now
                
                elif joy_cmd["active_stick"] or abs(joy_cmd["speed"]) > 0.01 or abs(joy_cmd.get("spin", 0.0)) > 0.01:
                    # Joystick has activity, update RobotState
                    self.last_joy_active = True
                    # Note: This might conflict with Keyboard if both active. 
                    # Assuming User doesn't use both simultaneously or Joystick takes precedence if moved.
                    
                    params = self.state.get_params()
                    
                    if abs(joy_cmd.get("spin", 0.0)) > 0.01:
                        target_omega = params.max_omega * joy_cmd["spin"]
                        self.state.set_control_command(0.0, 0.0, target_omega)
                    else:
                        # Normal Move
                        target_speed = params.max_speed * joy_cmd["speed"]
                        target_steer = 0.0
                        if joy_cmd["active_stick"] and joy_cmd["angle_deg"] is not None:
                            target_steer = 90.0 - joy_cmd["angle_deg"]
                        
                        self.state.set_control_command(target_speed, target_steer, 0.0)
                
                elif self.last_joy_active:
                    # Joystick just became inactive - Stop Chassis
                    self.last_joy_active = False
                    print("🛑 Joystick Idle - Stopping Chassis")
                    self.state.set_control_command(0.0, 0.0, 0.0)

            # ---------------------------------------------------------
            # 2. CONTROL EXECUTION PHASE (Consumer)
            # ---------------------------------------------------------
            cmd = self.state.get_control_command()
            params = self.state.get_params()
            
            if cmd.emergency_stop:
                self.controller.stop()
                self.current_physical_speed = 0.0
            else:
                # Smoothing
                target_v = cmd.speed_mps
                if self.enable_smooth_braking:
                    max_delta = self.accel_limit * dt
                    diff = target_v - self.current_physical_speed
                    if abs(diff) > max_delta:
                        self.current_physical_speed += math.copysign(max_delta, diff)
                    else:
                        self.current_physical_speed = target_v
                else:
                    self.current_physical_speed = target_v
                
                # Execute
                if abs(cmd.omega_rad) > 0.001:
                    # Spin
                    radius = math.hypot(self.geometry.L/2, self.geometry.W/2)
                    spin_linear = cmd.omega_rad * radius
                    if cmd.omega_rad > 0:
                        self.controller.spin_counter_clockwise(abs(spin_linear))
                    else:
                        self.controller.spin_clockwise(abs(spin_linear))
                else:
                    # Translation
                    if config.ENABLE_ACKERMANN_MODE:
                        # 修复：即使速度为0，只要有转向角输入，也要执行原地转向
                        # 之前逻辑: if abs(self.current_physical_speed) > 0.01
                        # 现在逻辑: 只要有速度 或者 显式要求转向（通过非零steer_angle且速度为0的情况）
                        # 但这里需要区分：如果是从 test_script 发来的纯转向指令，speed=0, angle!=0
                        
                        # 始终计算阿克曼几何
                        L = self.geometry.L
                        delta_rad = math.radians(cmd.steer_angle_deg)
                        
                        # 如果有速度，计算对应的角速度 omega
                        if abs(self.current_physical_speed) > 0.01:
                             omega = self.current_physical_speed * math.tan(delta_rad) / L
                             omega = max(min(omega, params.max_omega), -params.max_omega)
                        else:
                             omega = 0.0
                        
                        # 如果速度极小，但有转向指令，我们希望轮子转动到位，但不驱动
                        # chassis_move 内部会处理 speed=0 的情况
                        
                        if self.mode == "sim":
                             vis_angles = {
                                 "FL": cmd.steer_angle_deg, "FR": cmd.steer_angle_deg,
                                 "RL": -cmd.steer_angle_deg if config.ACKERMANN_4WS else 0.0,
                                 "RR": -cmd.steer_angle_deg if config.ACKERMANN_4WS else 0.0
                             }
                             # Sim mode doesn't really care about physics correctness as much as visual
                             self.controller.chassis_move(cmd.steer_angle_deg, self.current_physical_speed, omega, wheel_angles=vis_angles)
                        else:
                             # 关键修改：直接计算轮子状态，绕过 inverse_kinematics 的 omega 反算
                             # 这样可以保证转向角与手柄输入一致，避免 omega 限幅导致的比例失调
                             
                             wheel_states = {}
                             s_ang = cmd.steer_angle_deg
                             
                             # Ackermann Angle Limit (Default 45 degrees)
                             MAX_ACKERMANN_ANGLE = 45.0
                             if s_ang > MAX_ACKERMANN_ANGLE:
                                 s_ang = MAX_ACKERMANN_ANGLE
                             elif s_ang < -MAX_ACKERMANN_ANGLE:
                                 s_ang = -MAX_ACKERMANN_ANGLE
                             
                             # FL/FR
                             wheel_states["FL"] = (self.current_physical_speed, s_ang)
                             wheel_states["FR"] = (self.current_physical_speed, s_ang)
                             
                             # RL/RR
                             rear_ang = -s_ang if config.ACKERMANN_4WS else 0.0
                             wheel_states["RL"] = (self.current_physical_speed, rear_ang)
                             wheel_states["RR"] = (self.current_physical_speed, rear_ang)
                             
                             self.controller.apply_kinematics(wheel_states)

                    else:
                        # Holonomic
                        if config.ACKERMANN_4WS:
                             if abs(self.current_physical_speed) > 0.01:
                                 self.controller.move_diagonal(cmd.steer_angle_deg, self.current_physical_speed)
                             else:
                                 # 静态转向：强制覆盖 (4WS 全向)
                                 # 4WS 模式下，所有轮子同向
                                 wheel_states = {}
                                 s_ang = cmd.steer_angle_deg
                                 wheel_states["FL"] = (0.0, s_ang)
                                 wheel_states["FR"] = (0.0, s_ang)
                                 wheel_states["RL"] = (0.0, s_ang)
                                 wheel_states["RR"] = (0.0, s_ang)
                                 self.controller.apply_kinematics(wheel_states)
                        else:
                             # 同样逻辑，支持原地转向
                             w_angles = {"FL": cmd.steer_angle_deg, "FR": cmd.steer_angle_deg, "RL": 0.0, "RR": 0.0}                             
                             if abs(self.current_physical_speed) > 0.01:
                                 w_angles = {"FL": cmd.steer_angle_deg, "FR": cmd.steer_angle_deg, "RL": 0.0, "RR": 0.0}
                                 self.controller.chassis_move(cmd.steer_angle_deg, self.current_physical_speed, 0.0)
                             else:
                                 # 静态转向：强制覆盖
                                 wheel_states = {}
                                 s_ang = cmd.steer_angle_deg
                                 wheel_states["FL"] = (0.0, s_ang)
                                 wheel_states["FR"] = (0.0, s_ang)
                                 wheel_states["RL"] = (0.0, 0.0)
                                 wheel_states["RR"] = (0.0, 0.0)
                                 self.controller.apply_kinematics(wheel_states)

            # ---------------------------------------------------------
            # 3. FEEDBACK PHASE
            # ---------------------------------------------------------
            if self.dashboard_client:
                # Dashboard Commands
                dash_cmd = self.dashboard_client.get_command()
                if dash_cmd and "set_mode" in dash_cmd:
                    target_mode = dash_cmd["set_mode"]
                    should_be_ackermann = (target_mode == "ackermann")
                    if config.ENABLE_ACKERMANN_MODE != should_be_ackermann:
                        config.ENABLE_ACKERMANN_MODE = should_be_ackermann
                        self.controller.switch_kinematics_mode(config.ENABLE_ACKERMANN_MODE)
                        print(f"🖥️ Dashboard set mode to: {target_mode}")
                
                if dash_cmd and "set_ackermann_type" in dash_cmd:
                    atype = dash_cmd["set_ackermann_type"]
                    is_4ws = (atype == "4ws")
                    if config.ACKERMANN_4WS != is_4ws:
                        config.ACKERMANN_4WS = is_4ws
                        if config.ENABLE_ACKERMANN_MODE:
                            self.controller.switch_kinematics_mode(True)
                        print(f"🖥️ Dashboard set Ackermann type to: {atype}")

                # Update State
                self.state.update_chassis_command(
                    speed_mps=self.current_physical_speed,
                    steer_angle_deg=cmd.steer_angle_deg,
                    throttle_pct=(abs(self.current_physical_speed) / params.max_speed * 100.0) if params.max_speed > 0 else 0.0
                )
                self.state.set_mode(
                    is_ackermann=config.ENABLE_ACKERMANN_MODE,
                    is_4ws=config.ACKERMANN_4WS
                )
                
                if self.mode == "real" and self.monitor:
                    def safe_get(mid, key):
                        s = self.monitor.get_state(mid)
                        return s.get(key, 0) if s else 0
                    
                    batch_updates = {}
                    for name, steer_id, drive_id in [
                        ("FL", config.FL_STEER_ID, config.FL_DRIVE_ID),
                        ("FR", config.FR_STEER_ID, config.FR_DRIVE_ID),
                        ("RL", config.RL_STEER_ID, config.RL_DRIVE_ID),
                        ("RR", config.RR_STEER_ID, config.RR_DRIVE_ID)
                    ]:
                        batch_updates[name] = {
                            "drive_rpm": safe_get(drive_id, "rpm"),
                            "steer_rpm": safe_get(steer_id, "rpm"),
                            "display_angle": safe_get(steer_id, "display_angle")
                        }
                    
                    # One lock for all 4 modules
                    self.state.update_batch_module_states(batch_updates)
                    
                elif self.mode == "sim":
                    sim_state = self.controller.get_state()
                    self.state.update_pose(
                        x=sim_state.get('x', 0.0),
                        y=sim_state.get('y', 0.0),
                        heading_deg=sim_state.get('heading_deg', 0.0)
                    )
                    
                    sim_rpm = int(self.current_physical_speed * 1000)
                    w_angles = self.controller.state.wheel_angles
                    
                    batch_updates = {}
                    for name in ["FL", "FR", "RL", "RR"]:
                        batch_updates[name] = {
                            "drive_rpm": sim_rpm,
                            "steer_rpm": 0,
                            "display_angle": w_angles.get(name, 0.0)
                        }
                    self.state.update_batch_module_states(batch_updates)
                
                # Send Snapshot
                self.dashboard_client.send_state(self.state.get_snapshot())

            time.sleep(0.01)
