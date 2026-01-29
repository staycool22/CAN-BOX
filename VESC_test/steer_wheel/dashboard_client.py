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
        self.speed_mps = 0.0
        self.throttle_pct = 0.0
        self.last_update = time.time()

    def update(self, speed_mps, steer_angle_deg, throttle_pct=None):
        now = time.time()
        dt = now - self.last_update
        if dt < 0:
            dt = 0.0
        self.last_update = now
        angle_rad = math.radians(steer_angle_deg)
        self.x += speed_mps * math.cos(angle_rad) * dt
        self.y += speed_mps * math.sin(angle_rad) * dt
        self.speed_mps = speed_mps
        self.steer_angle_deg = steer_angle_deg
        self.heading_deg = steer_angle_deg
        if throttle_pct is not None:
            self.throttle_pct = throttle_pct

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "heading_deg": self.heading_deg,
            "steer_angle_deg": self.steer_angle_deg,
            "speed_mps": self.speed_mps,
            "throttle_pct": self.throttle_pct,
            "timestamp": self.last_update
        }


class SimulatedSteerController:
    def __init__(self):
        self.state = SimulationState()
        self.vesc_drive = None

    def _send_steer_pos(self, motor_id: int, target_angle: float):
        self.state.update(self.state.speed_mps, target_angle, self.state.throttle_pct)

    def set_accel_decel(self, accel_mps2: float, decel_mps2: float = None):
        return

    def apply_kinematics(self, wheel_states: Dict[str, Tuple[float, float]]):
        speeds = []
        angles = []
        for name in ["FL", "FR"]:
            if name in wheel_states:
                speed, angle = wheel_states[name]
                speeds.append(speed)
                angles.append(angle)
        if not speeds:
            return
        avg_speed = sum(speeds) / len(speeds)
        avg_angle = sum(angles) / len(angles) if angles else self.state.steer_angle_deg
        self.state.update(avg_speed, avg_angle, self.state.throttle_pct)

    def emergency_stop(self):
        self.state.update(0.0, self.state.steer_angle_deg, self.state.throttle_pct)

    def get_state(self):
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

    store = state_store or DashboardStateStore()
    app = FastAPI()

    html = """
    <!doctype html>
    <html>
    <head>
      <meta charset="utf-8">
      <title>Steer Wheel Dashboard</title>
      <style>
        body { font-family: sans-serif; margin: 20px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
        .card { border: 1px solid #ddd; padding: 12px; border-radius: 8px; }
        .row { display: flex; justify-content: space-between; margin: 4px 0; }
        input { width: 160px; }
        button { padding: 6px 10px; }
      </style>
    </head>
    <body>
      <h2>Steer Wheel Dashboard</h2>
      <div class="grid">
        <div class="card">
          <div class="row"><span>x 坐标</span><span id="x">0</span></div>
          <div class="row"><span>y 坐标</span><span id="y">0</span></div>
          <div class="row"><span>航向角 heading_deg</span><span id="heading">0</span></div>
          <div class="row"><span>舵角 steer_angle_deg</span><span id="steer">0</span></div>
          <div class="row"><span>速度 speed_mps</span><span id="speed">0</span></div>
          <div class="row"><span>油门 throttle_pct</span><span id="throttle">0</span></div>
          <div class="row"><span>时间戳 timestamp</span><span id="ts">0</span></div>
        </div>
        <div class="card">
          <div class="row"><span>目标舵角 steer_angle_deg</span><input id="cmd_steer" value="0"></div>
          <div class="row"><span>目标油门 throttle_pct</span><input id="cmd_throttle" value="0"></div>
          <div class="row"><span>方向 direction</span><input id="cmd_dir" value="1"></div>
          <div class="row"><button onclick="sendCommand()">Send Command</button></div>
        </div>
      </div>
      <script>
        async function refresh() {
          try {
            const res = await fetch('/api/state');
            const data = await res.json();
            document.getElementById('x').textContent = (data.x || 0).toFixed(3);
            document.getElementById('y').textContent = (data.y || 0).toFixed(3);
            document.getElementById('heading').textContent = (data.heading_deg || 0).toFixed(2);
            document.getElementById('steer').textContent = (data.steer_angle_deg || 0).toFixed(2);
            document.getElementById('speed').textContent = (data.speed_mps || 0).toFixed(3);
            document.getElementById('throttle').textContent = (data.throttle_pct || 0).toFixed(1);
            document.getElementById('ts').textContent = data.timestamp || 0;
          } catch (e) {}
        }
        async function sendCommand() {
          const payload = {
            steer_angle_deg: parseFloat(document.getElementById('cmd_steer').value || 0),
            throttle_pct: parseFloat(document.getElementById('cmd_throttle').value || 0),
            direction: parseFloat(document.getElementById('cmd_dir').value || 1)
          };
          try {
            await fetch('/api/command', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
          } catch (e) {}
        }
        setInterval(refresh, 200);
        refresh();
      </script>
    </body>
    </html>
    """

    @app.get("/", response_class=HTMLResponse)
    async def index():
        return html

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
    config = uvicorn.Config(app, host=host, port=port, log_level="info")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    return server, thread, store


class DashboardClient:
    def __init__(self, base_url="http://127.0.0.1:8080", timeout=0.2):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def send_state(self, state):
        try:
            requests.post(f"{self.base_url}/api/state", json=state, timeout=self.timeout)
        except Exception:
            return False
        return True
