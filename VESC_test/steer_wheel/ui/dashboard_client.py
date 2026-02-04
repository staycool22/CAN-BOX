import time
import math
import threading
from typing import Dict, Tuple
import requests


class SimulationState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading_deg = 0.0
        self.steer_angle_deg = 0.0
        self.wheel_angles = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
        self.speed_mps = 0.0
        self.omega_rad = 0.0
        self.throttle_pct = 0.0
        self.last_update = time.time()

    def update(self, speed_mps, steer_angle_deg, omega_rad=0.0, throttle_pct=None, wheel_angles=None):
        now = time.time()
        dt = now - self.last_update
        if dt < 0:
            dt = 0.0
        self.last_update = now
        
        # Fix: Use previous state (self.speed_mps, self.omega_rad) to integrate over dt
        # instead of the new commanded values. This prevents jumps when speed changes from 0.
        
        # 1. Integrate Position using old velocity and old heading
        global_move_angle_rad = math.radians(self.heading_deg + self.steer_angle_deg)
        self.x += self.speed_mps * math.cos(global_move_angle_rad) * dt
        self.y += self.speed_mps * math.sin(global_move_angle_rad) * dt
        
        # 2. Integrate Heading using old omega
        self.heading_deg += math.degrees(self.omega_rad * dt)
        self.heading_deg %= 360.0
        
        # 3. Update State to new values
        self.speed_mps = speed_mps
        self.steer_angle_deg = steer_angle_deg
        self.omega_rad = omega_rad
        
        if wheel_angles:
            self.wheel_angles = wheel_angles.copy()
        else:
            # Default fallback if no specific wheel angles provided
            self.wheel_angles = {
                "FL": steer_angle_deg, "FR": steer_angle_deg,
                "RL": 0.0, "RR": 0.0
            }
        
        if throttle_pct is not None:
            self.throttle_pct = throttle_pct

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "heading_deg": self.heading_deg,
            "steer_angle_deg": self.steer_angle_deg,
            "wheel_angles": self.wheel_angles,
            "speed_mps": self.speed_mps,
            "throttle_pct": self.throttle_pct,
            "timestamp": self.last_update
        }


class SimulatedSteerController:
    def __init__(self):
        self.state = SimulationState()
        self.vesc_drive = None
        self.is_ackermann = False
        self.radius = 0.5 # 模拟旋转半径 (m)

    def _send_steer_pos(self, motor_id: int, target_angle: float):
        self.state.update(self.state.speed_mps, target_angle, self.state.omega_rad, self.state.throttle_pct)

    def set_accel_decel(self, accel_mps2: float, decel_mps2: float = None):
        return

    def apply_kinematics(self, wheel_states: Dict[str, Tuple[float, float]]):
        speeds = []
        angles = []
        current_wheel_angles = self.state.wheel_angles.copy()
        
        for name, (speed, angle) in wheel_states.items():
            speeds.append(speed)
            angles.append(angle)
            if name in current_wheel_angles:
                current_wheel_angles[name] = angle
        
        if not speeds:
            return
        avg_speed = sum(speeds) / len(speeds)
        avg_angle = sum(angles) / len(angles) if angles else self.state.steer_angle_deg
        self.state.update(avg_speed, avg_angle, 0.0, self.state.throttle_pct, wheel_angles=current_wheel_angles)

    def emergency_stop(self):
        self.stop()

    def stop(self):
        self.state.update(0.0, self.state.steer_angle_deg, 0.0, 0.0, self.state.wheel_angles)

    def switch_kinematics_mode(self, enable_ackermann: bool):
        self.is_ackermann = enable_ackermann
        print(f"[Sim] Switched to {'Ackermann' if enable_ackermann else 'Holonomic'} mode")

    def chassis_move(self, angle_deg: float, speed_mps: float, omega_rad: float = 0.0, wheel_angles=None):
        # 估算油门百分比
        throttle = (abs(speed_mps) / 5.0) * 100.0 
        self.state.update(speed_mps, angle_deg, omega_rad, throttle, wheel_angles)

    def move_straight(self, speed_mps: float):
        w_angles = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
        self.chassis_move(0.0, speed_mps, 0.0, w_angles)

    def move_diagonal(self, angle_deg: float, speed_mps: float):
        w_angles = {"FL": angle_deg, "FR": angle_deg, "RL": angle_deg, "RR": angle_deg}
        self.chassis_move(angle_deg, speed_mps, 0.0, w_angles)

    def spin_clockwise(self, speed_mps: float):
        # omega = v / r
        omega = -abs(speed_mps) / self.radius
        w_angles = {"FL": -45.0, "FR": 45.0, "RL": 45.0, "RR": -45.0}
        self.chassis_move(0.0, 0.0, omega, w_angles)

    def spin_counter_clockwise(self, speed_mps: float):
        omega = abs(speed_mps) / self.radius
        w_angles = {"FL": 45.0, "FR": -45.0, "RL": -45.0, "RR": 45.0}
        self.chassis_move(0.0, 0.0, omega, w_angles)

    def get_state(self):
        # Perform a tick update to integrate position over the elapsed time
        # since the last command, making the motion appear smooth.
        self.state.update(
            self.state.speed_mps,
            self.state.steer_angle_deg,
            self.state.omega_rad,
            self.state.throttle_pct,
            wheel_angles=self.state.wheel_angles
        )
        return self.state.to_dict()


class DashboardStateStore:
    def __init__(self):
        self.state = {}
        self.command = {}
        self.lock = threading.Lock()

    def set_state(self, state):
        with self.lock:
            self.state = state

    def get_state(self):
        with self.lock:
            return dict(self.state)

    def set_command(self, command):
        with self.lock:
            self.command = command

    def get_command(self):
        with self.lock:
            return dict(self.command)


def create_dashboard_app(state_store=None):
    from fastapi import FastAPI, Request
    from fastapi.responses import HTMLResponse
    import os

    store = state_store or DashboardStateStore()
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    async def index():
        # Read HTML from external file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        html_path = os.path.join(current_dir, "dashboard_ui.html")
        try:
            with open(html_path, "r", encoding="utf-8") as f:
                html_content = f.read()
            return html_content
        except FileNotFoundError:
            return "<h1>Error: dashboard_ui.html not found</h1>"

    @app.get("/api/state")
    async def get_state():
        return store.get_state()

    @app.post("/api/state")
    async def post_state(request: Request):
        data = await request.json()
        store.set_state(data)
        return {"ok": True}

    @app.get("/api/command")
    async def get_command():
        return store.get_command()

    @app.post("/api/command")
    async def post_command(request: Request):
        data = await request.json()
        store.set_command(data)
        return {"ok": True}

    return app, store


def start_dashboard_server(state_store=None, host="0.0.0.0", port=8080):
    import uvicorn
    app, store = create_dashboard_app(state_store)
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    return server, thread, store


class DashboardClient:
    def __init__(self, base_url="http://127.0.0.1:8080", timeout=0.2):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        
        # Async State Sending
        self._latest_state = None
        self._state_lock = threading.Lock()
        self._new_state_event = threading.Event()
        self._running = True
        
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        
        # Async Command Polling
        self._latest_command = {}
        self._command_lock = threading.Lock()
        self._poller_thread = threading.Thread(target=self._poller_loop, daemon=True)
        self._poller_thread.start()

    def _sender_loop(self):
        while self._running:
            # Wait for new data
            if self._new_state_event.wait(timeout=0.1):
                self._new_state_event.clear()
                
                state_to_send = None
                with self._state_lock:
                    state_to_send = self._latest_state
                
                if state_to_send:
                    try:
                        requests.post(f"{self.base_url}/api/state", json=state_to_send, timeout=self.timeout)
                    except Exception:
                        pass # Ignore errors in background

    def _poller_loop(self):
        while self._running:
            try:
                resp = requests.get(f"{self.base_url}/api/command", timeout=self.timeout)
                if resp.status_code == 200:
                    cmd = resp.json()
                    with self._command_lock:
                        self._latest_command = cmd
            except Exception:
                pass
            time.sleep(0.1) # Poll every 100ms (10Hz)

    def get_command(self):
        with self._command_lock:
            return self._latest_command.copy()

    def send_state(self, state):
        with self._state_lock:
            self._latest_state = state
        self._new_state_event.set()
        return True
        
    def stop(self):
        self._running = False
