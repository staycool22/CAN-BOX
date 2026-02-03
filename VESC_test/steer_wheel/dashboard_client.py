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

    store = state_store or DashboardStateStore()
    app = FastAPI()

    html = """
    <!doctype html>
    <html>
    <head>
      <meta charset="utf-8">
      <title>Steer Wheel Dashboard</title>
      <style>
        :root {
          --bg-color: #121212;
          --card-bg: #1e1e1e;
          --text-primary: #e0e0e0;
          --text-secondary: #a0a0a0;
          --accent: #bb86fc;
          --success: #03dac6;
          --border: #333;
        }
        body {
          font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
          margin: 0;
          padding: 20px;
          background-color: var(--bg-color);
          color: var(--text-primary);
          height: 100vh;
          box-sizing: border-box;
          overflow: hidden;
        }
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
            padding-bottom: 10px;
            border-bottom: 1px solid var(--border);
        }
        h2 { margin: 0; font-weight: 300; letter-spacing: 1px; color: var(--accent); }
        
        .dashboard-container {
            display: grid;
            grid-template-columns: 320px 1fr;
            gap: 20px;
            height: calc(100% - 70px);
        }
        
        .metrics-panel {
            display: flex;
            flex-direction: column;
            gap: 15px;
            overflow-y: auto;
        }
        
        .card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        
        .metric-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }
        .metric-row:last-child { margin-bottom: 0; }
        
        .label {
            font-size: 0.85rem;
            color: var(--text-secondary);
            text-transform: uppercase;
        }
        
        .value {
            font-family: 'Consolas', monospace;
            font-size: 1.1rem;
            font-weight: bold;
            color: var(--success);
        }
        
        .control-panel {
            margin-top: 10px;
            padding-top: 10px;
            border-top: 1px solid var(--border);
        }
        
        select {
            background-color: #2a2a2a;
            color: var(--text-primary);
            border: 1px solid var(--border);
            padding: 5px 10px;
            border-radius: 4px;
            width: 100%;
            font-size: 0.9rem;
        }

        .viz-panel {
            background-color: var(--card-bg);
            border-radius: 12px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
            position: relative;
            overflow: hidden;
        }
        
        canvas {
            width: 100%;
            height: 100%;
            display: block;
        }
        
        .status-badge {
            font-size: 0.8rem;
            background: rgba(3, 218, 198, 0.1);
            color: var(--success);
            padding: 4px 8px;
            border-radius: 4px;
            border: 1px solid rgba(3, 218, 198, 0.3);
        }
      </style>
    </head>
    <body>
      <div class="header">
        <h2>Steer Wheel Monitor</h2>
        <div class="status-badge">● Live System</div>
      </div>
      
      <div class="dashboard-container">
        <!-- Metrics Column -->
        <div class="metrics-panel">
            <div class="card">
                <div class="metric-row">
                    <span class="label">X 坐标 (m)</span>
                    <span class="value" id="x">0.000</span>
                </div>
                <div class="metric-row">
                    <span class="label">Y 坐标 (m)</span>
                    <span class="value" id="y">0.000</span>
                </div>
            </div>
            
            <div class="card">
                <div class="metric-row">
                    <span class="label">速度 (m/s)</span>
                    <span class="value" id="speed">0.000</span>
                </div>
                <div class="metric-row">
                    <span class="label">油门 (%)</span>
                    <span class="value" id="throttle">0.0</span>
                </div>
            </div>
            
            <div class="card">
                <div class="metric-row">
                    <span class="label">航向角 (deg)</span>
                    <span class="value" id="heading">0.00</span>
                </div>
                <div class="metric-row">
                    <span class="label">转向角 (deg)</span>
                    <span class="value" id="steer">0.00</span>
                </div>
            </div>

            <!-- RPM Monitor -->
            <div class="card">
                 <div class="metric-row" style="margin-bottom:5px; border-bottom:1px solid #333; padding-bottom:5px;">
                    <span class="label" style="color:var(--accent)">电机状态 (RPM / Enc2)</span>
                </div>
                <!-- FL -->
                <div class="metric-row" style="display:block; margin-bottom:8px">
                    <div style="display:flex; justify-content:space-between; margin-bottom:2px">
                        <span class="label">左前 (FL)</span>
                        <span class="value" style="font-size:0.8rem; color:var(--success)" id="enc2-fl">N/A</span>
                    </div>
                    <div style="display:flex; justify-content:flex-end; font-size:0.8rem; color:var(--text-secondary)">
                        <span style="margin-right:8px">D: <span id="rpm-fl-drive">0</span></span>
                        <span>S: <span id="rpm-fl-steer">0</span></span>
                    </div>
                </div>
                <!-- FR -->
                <div class="metric-row" style="display:block; margin-bottom:8px">
                    <div style="display:flex; justify-content:space-between; margin-bottom:2px">
                        <span class="label">右前 (FR)</span>
                        <span class="value" style="font-size:0.8rem; color:var(--success)" id="enc2-fr">N/A</span>
                    </div>
                    <div style="display:flex; justify-content:flex-end; font-size:0.8rem; color:var(--text-secondary)">
                        <span style="margin-right:8px">D: <span id="rpm-fr-drive">0</span></span>
                        <span>S: <span id="rpm-fr-steer">0</span></span>
                    </div>
                </div>
                <!-- RL -->
                <div class="metric-row" style="display:block; margin-bottom:8px">
                    <div style="display:flex; justify-content:space-between; margin-bottom:2px">
                        <span class="label">左后 (RL)</span>
                        <span class="value" style="font-size:0.8rem; color:var(--success)" id="enc2-rl">N/A</span>
                    </div>
                    <div style="display:flex; justify-content:flex-end; font-size:0.8rem; color:var(--text-secondary)">
                        <span style="margin-right:8px">D: <span id="rpm-rl-drive">0</span></span>
                        <span>S: <span id="rpm-rl-steer">0</span></span>
                    </div>
                </div>
                <!-- RR -->
                <div class="metric-row" style="display:block; margin-bottom:8px">
                    <div style="display:flex; justify-content:space-between; margin-bottom:2px">
                        <span class="label">右后 (RR)</span>
                        <span class="value" style="font-size:0.8rem; color:var(--success)" id="enc2-rr">N/A</span>
                    </div>
                    <div style="display:flex; justify-content:flex-end; font-size:0.8rem; color:var(--text-secondary)">
                        <span style="margin-right:8px">D: <span id="rpm-rr-drive">0</span></span>
                        <span>S: <span id="rpm-rr-steer">0</span></span>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <div class="metric-row">
                    <span class="label">时间戳</span>
                    <span class="value" id="ts" style="font-size: 0.9rem; color: var(--text-secondary)">0</span>
                </div>
                
                <div class="control-panel">
                    <div class="metric-row">
                <span class="label">底盘模式</span>
            </div>
            <select id="mode-selector" onchange="changeMode()">
                <option value="holonomic">全向移动 (Holonomic)</option>
                <option value="ackermann">阿克曼转向 (Ackermann)</option>
            </select>
            
            <div class="metric-row" id="steering-mode-container" style="display: none; margin-top: 10px;">
                <span class="label">转向方式</span>
                <select id="steer-mode-selector" onchange="changeSteerMode()">
                    <option value="2ws">前轮转向 (2WS)</option>
                    <option value="4ws">四轮转向 (4WS)</option>
                </select>
            </div>
        </div>
            </div>
        </div>
        
        <!-- Visualization Column -->
        <div class="viz-panel" id="viz-container">
            <canvas id="viz"></canvas>
        </div>
      </div>

      <script>
        const canvas = document.getElementById('viz');
        const ctx = canvas.getContext('2d');
        const container = document.getElementById('viz-container');
        
        let carState = { x: 0, y: 0, heading: 0, steer: 0, speed: 0 };
        let lastMode = "holonomic";

        function resize() {
            canvas.width = container.clientWidth;
            canvas.height = container.clientHeight;
        }
        window.addEventListener('resize', resize);
        resize();
        
        async function changeMode() {
        const selector = document.getElementById('mode-selector');
        const mode = selector.value;
        
        // Show/Hide 2WS/4WS selector based on Ackermann mode
        // User requested 2WS/4WS selection in Holonomic mode too, so we keep it visible always
        const steerContainer = document.getElementById('steering-mode-container');
        steerContainer.style.display = 'block';

        try {
            await fetch('/api/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ set_mode: mode })
            });
            console.log("Sent mode:", mode);
        } catch (e) {
            console.error("Failed to set mode", e);
        }
    }

    async function changeSteerMode() {
        const selector = document.getElementById('steer-mode-selector');
        const mode = selector.value;
        try {
            await fetch('/api/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ set_ackermann_type: mode })
            });
            console.log("Sent steer mode:", mode);
        } catch (e) {
            console.error("Failed to set steer mode", e);
        }
    }

    function updateCard(prefix, driveRpm, steerRpm, enc2) {
        const driveEl = document.getElementById(`rpm-${prefix}-drive`);
        const steerEl = document.getElementById(`rpm-${prefix}-steer`);
        const enc2El = document.getElementById(`enc2-${prefix}`);
        
        if (driveEl) driveEl.innerText = Math.round(driveRpm || 0);
        if (steerEl) steerEl.innerText = Math.round(steerRpm || 0);
        if (enc2El && enc2 !== undefined && enc2 !== null) {
            enc2El.innerText = enc2.toFixed(2) + "°";
        } else if (enc2El) {
             enc2El.innerText = "N/A";
        }
    }

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
            document.getElementById('ts').textContent = (data.timestamp || 0).toFixed(3);
            
            // Update Mode Selector if changed from server side
            if (data.mode) {
                const selector = document.getElementById('mode-selector');
                if (document.activeElement !== selector) {
                    selector.value = data.mode;
                    // Also update visibility of 2WS/4WS selector - Always visible now
                    const steerContainer = document.getElementById('steering-mode-container');
                    steerContainer.style.display = 'block';
                }
            }
            
            // Update 2WS/4WS Selector if changed from server side
            if (data.ackermann_type) {
                const selector = document.getElementById('steer-mode-selector');
                if (document.activeElement !== selector) {
                    selector.value = data.ackermann_type;
                }
            }

            // Update RPMs and Enc2
            if (data.rpms) {
                updateCard('fl', data.rpms.FL_drive, data.rpms.FL_steer, data.rpms.FL_enc2);
                updateCard('fr', data.rpms.FR_drive, data.rpms.FR_steer, data.rpms.FR_enc2);
                updateCard('rl', data.rpms.RL_drive, data.rpms.RL_steer, data.rpms.RL_enc2);
                updateCard('rr', data.rpms.RR_drive, data.rpms.RR_steer, data.rpms.RR_enc2);
            }
            
            carState = {
                x: data.x || 0,
                y: data.y || 0,
                heading: data.heading_deg || 0,
                steer: data.steer_angle_deg || 0,
                wheels: data.wheel_angles || {
                    "FL": data.steer_angle_deg || 0, 
                    "FR": data.steer_angle_deg || 0, 
                    "RL": 0, 
                    "RR": 0
                }
            };
            
            updatePath(carState.x, carState.y);
            drawScene();
            
          } catch (e) {}
        }
        
        // Trajectory Path Storage
        const path = [];
        const MAX_PATH_POINTS = 1000;
        const MIN_DIST_THRESHOLD = 0.1; // meters
        
        function updatePath(x, y) {
            if (path.length === 0) {
                path.push({x, y});
                return;
            }
            
            const last = path[path.length - 1];
            const dist = Math.hypot(x - last.x, y - last.y);
            
            if (dist > MIN_DIST_THRESHOLD) {
                path.push({x, y});
                if (path.length > MAX_PATH_POINTS) {
                    path.shift(); // Remove oldest
                }
            }
        }

        function drawScene() {
            const w = canvas.width;
            const h = canvas.height;
            const cx = w / 2;
            const cy = h / 2;
            
            // Clear
            ctx.fillStyle = '#1e1e1e';
            ctx.fillRect(0, 0, w, h);
            
            const scale = 100; // 1m = 100px
            
            // Camera Transform: Keep Car at Center (cx, cy)
            // World Coordinates (wx, wy) -> Screen Coordinates (sx, sy)
            // sx = (wx - carX) * scale + cx
            // sy = -(wy - carY) * scale + cy  (Note: -Y for canvas coords)
            
            // 1. Draw Grid (Fixed in World)
            ctx.strokeStyle = '#2a2a2a';
            ctx.lineWidth = 1;
            const gridSize = 1.0; // 1 meter grid
            
            // Calculate visible grid range roughly
            const leftWorld = carState.x - (cx / scale);
            const rightWorld = carState.x + (cx / scale);
            const bottomWorld = carState.y - (cy / scale);
            const topWorld = carState.y + (cy / scale);
            
            const startX = Math.floor(leftWorld / gridSize) * gridSize;
            const startY = Math.floor(bottomWorld / gridSize) * gridSize;
            
            ctx.beginPath();
            // Vertical lines
            for (let x = startX; x < rightWorld; x += gridSize) {
                const sx = (x - carState.x) * scale + cx;
                ctx.moveTo(sx, 0);
                ctx.lineTo(sx, h);
            }
            // Horizontal lines
            for (let y = startY; y < topWorld; y += gridSize) {
                const sy = -(y - carState.y) * scale + cy;
                ctx.moveTo(0, sy);
                ctx.lineTo(w, sy);
            }
            ctx.stroke();
            
            // 2. Draw Trajectory
            if (path.length > 1) {
                ctx.strokeStyle = 'rgba(3, 218, 198, 0.5)';
                ctx.lineWidth = 2;
                ctx.beginPath();
                // Move to first point
                let p0 = path[0];
                let s0x = (p0.x - carState.x) * scale + cx;
                let s0y = -(p0.y - carState.y) * scale + cy;
                ctx.moveTo(s0x, s0y);
                
                for (let i = 1; i < path.length; i++) {
                    let p = path[i];
                    let sx = (p.x - carState.x) * scale + cx;
                    let sy = -(p.y - carState.y) * scale + cy;
                    ctx.lineTo(sx, sy);
                }
                ctx.stroke();
            }

            // 3. Draw Car (Centered)
            ctx.save();
            ctx.translate(cx, cy);
            
            // Rotate Car: -Heading (North Up View -> Car rotates)
            // If heading is 0 (East), car points East (Right). 
            // In Canvas 0 is Right. 
            // If Heading increases (CCW), car rotates CCW.
            // Canvas rotation is CW for positive. So we use -heading.
            ctx.rotate(-carState.heading * Math.PI / 180);
            
            // Dimensions
            const carLen = 0.60 * scale; 
            const carWid = 0.44 * scale;
            
            // Chassis
            ctx.fillStyle = '#333';
            ctx.strokeStyle = '#bb86fc';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.rect(-carLen/2, -carWid/2, carLen, carWid);
            ctx.fill();
            ctx.stroke();
            
            // Forward Arrow on Chassis
            ctx.fillStyle = '#bb86fc';
            ctx.beginPath();
            ctx.moveTo(carLen/2 - 10, -10);
            ctx.lineTo(carLen/2 + 15, 0);
            ctx.lineTo(carLen/2 - 10, 10);
            ctx.fill();
            
            // Wheels
            const wheelLen = 0.15 * scale;
            const wheelWid = 0.06 * scale;
            const wheelX = carLen/2 - 10;
            const wheelY = carWid/2 + 5;
            
            function drawWheel(x, y, angle) {
                ctx.save();
                ctx.translate(x, y);
                ctx.rotate(-angle * Math.PI / 180); // -angle for CCW steer
                
                ctx.fillStyle = '#03dac6';
                ctx.beginPath();
                ctx.rect(-wheelLen/2, -wheelWid/2, wheelLen, wheelWid);
                ctx.fill();
                
                // Wheel direction line
                ctx.strokeStyle = '#000';
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(-wheelLen/4, 0);
                ctx.lineTo(wheelLen/4, 0);
                ctx.stroke();
                
                ctx.restore();
            }
            
            // Draw 4 Wheels
            // Note: wheel_angles keys depend on backend. 
            // Assuming: FL, FR, RL, RR
            const wheels = carState.wheels || {};
            
            // Front Left (Top in diagram if pointing Right? No. 
            // Standard car coords: X forward, Y left. 
            // Here drawing: X is Right (Forward), Y is Down (Right side of car).
            // Wait, Canvas Y is down.
            // If car points Right (X+), then Y- is Left Side, Y+ is Right Side.
            
            drawWheel(wheelX, -wheelY, wheels.FL || 0); // Front Left
            drawWheel(wheelX, wheelY, wheels.FR || 0);  // Front Right
            drawWheel(-wheelX, -wheelY, wheels.RL || 0); // Rear Left
            drawWheel(-wheelX, wheelY, wheels.RR || 0);  // Rear Right
            
            ctx.restore();
            
            // Info Overlay
            ctx.fillStyle = '#666';
            ctx.font = '12px monospace';
            ctx.fillText('SCALE: 1m = 100px | TRAIL: ' + path.length + ' pts', 10, h - 10);
        }

        setInterval(refresh, 100);
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

    def get_command(self):
        try:
            resp = requests.get(f"{self.base_url}/api/command", timeout=self.timeout)
            if resp.status_code == 200:
                return resp.json()
        except:
            pass
        return {}
    def send_state(self, state):
        try:
            requests.post(f"{self.base_url}/api/state", json=state, timeout=self.timeout)
        except Exception:
            return False
        return True
