import threading
import time
from dataclasses import dataclass
from typing import Dict, Any, Optional

@dataclass
class ChassisStateData:
    speed_mps: float = 0.0
    steer_angle_deg: float = 0.0
    heading_deg: float = 0.0
    x: float = 0.0
    y: float = 0.0
    throttle_pct: float = 0.0
    timestamp: float = 0.0

@dataclass
class MotorState:
    rpm_drive: float = 0.0
    rpm_steer: float = 0.0
    angle_deg: float = 0.0 # Display angle
    # enc2_raw: float = 0.0  # Optional if needed

@dataclass
class SystemMode:
    is_ackermann: bool = False
    is_4ws: bool = True

@dataclass
class ControlCommand:
    speed_mps: float = 0.0
    steer_angle_deg: float = 0.0
    omega_rad: float = 0.0
    emergency_stop: bool = False
    timestamp: float = 0.0

@dataclass
class ControlParams:
    max_speed: float = 2.0
    max_omega: float = 1.0
    # Add other params as needed

class ChassisState:
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ChassisState, cls).__new__(cls)
            cls._instance._initialize()
        return cls._instance

    def _initialize(self):
        self._lock = threading.Lock()
        self.chassis = ChassisStateData()
        self.command = ControlCommand()
        self.params = ControlParams()
        self.motors: Dict[str, MotorState] = {
            "FL": MotorState(), "FR": MotorState(),
            "RL": MotorState(), "RR": MotorState()
        }
        self.mode = SystemMode()
    
    def set_control_command(self, speed_mps: float, steer_angle_deg: float, omega_rad: float = 0.0, emergency_stop: bool = False):
        """Set the active control command from input devices."""
        with self._lock:
            self.command.speed_mps = speed_mps
            self.command.steer_angle_deg = steer_angle_deg
            self.command.omega_rad = omega_rad
            self.command.emergency_stop = emergency_stop
            self.command.timestamp = time.time()
            
    def get_control_command(self) -> ControlCommand:
        """Get the latest control command."""
        import copy
        with self._lock:
            return copy.copy(self.command)

    def update_params(self, max_speed: Optional[float] = None, max_omega: Optional[float] = None):
        with self._lock:
            if max_speed is not None: self.params.max_speed = max_speed
            if max_omega is not None: self.params.max_omega = max_omega
            
    def get_params(self) -> ControlParams:
        import copy
        with self._lock:
            return copy.copy(self.params)

    def update_chassis_command(self, speed_mps: float, steer_angle_deg: float, throttle_pct: float = 0.0):
        """Update the commanded chassis state."""
        with self._lock:
            self.chassis.speed_mps = speed_mps
            self.chassis.steer_angle_deg = steer_angle_deg
            self.chassis.throttle_pct = throttle_pct
            self.chassis.timestamp = time.time()
            
    def update_pose(self, x: float, y: float, heading_deg: float):
        """Update the estimated/simulated position and heading."""
        with self._lock:
            self.chassis.x = x
            self.chassis.y = y
            self.chassis.heading_deg = heading_deg
            
    def update_motor_state(self, name: str, drive_rpm: float, steer_rpm: float, display_angle: float):
        """Update the feedback state of a specific motor module."""
        with self._lock:
            if name in self.motors:
                m = self.motors[name]
                m.rpm_drive = drive_rpm
                m.rpm_steer = steer_rpm
                m.angle_deg = display_angle

    def update_batch_module_states(self, updates: Dict[str, Dict[str, float]]):
        """
        Atomically update multiple modules at once to reduce lock contention.
        Format: {"FL": {"drive_rpm": ..., "steer_rpm": ..., "display_angle": ...}, ...}
        """
        with self._lock:
            for name, data in updates.items():
                if name in self.motors:
                    m = self.motors[name]
                    if "drive_rpm" in data: m.rpm_drive = data["drive_rpm"]
                    if "steer_rpm" in data: m.rpm_steer = data["steer_rpm"]
                    if "display_angle" in data: m.angle_deg = data["display_angle"]
                
    def set_mode(self, is_ackermann: bool, is_4ws: bool):
        """Update the current operating mode."""
        with self._lock:
            self.mode.is_ackermann = is_ackermann
            self.mode.is_4ws = is_4ws

    def get_snapshot(self) -> Dict[str, Any]:
        """Get a thread-safe snapshot of the full state for dashboard/logging."""
        with self._lock:
            # Construct the dictionary structure expected by dashboard_client
            rpms = {}
            for name, m in self.motors.items():
                rpms[f"{name}_drive"] = m.rpm_drive
                rpms[f"{name}_steer"] = m.rpm_steer
                rpms[f"{name}_enc2"] = m.angle_deg 
            
            return {
                "x": self.chassis.x,
                "y": self.chassis.y,
                "heading_deg": self.chassis.heading_deg,
                "steer_angle_deg": self.chassis.steer_angle_deg,
                "speed_mps": self.chassis.speed_mps,
                "throttle_pct": self.chassis.throttle_pct,
                "timestamp": self.chassis.timestamp,
                "rpms": rpms,
                "mode": "ackermann" if self.mode.is_ackermann else "holonomic",
                "ackermann_type": "4ws" if self.mode.is_4ws else "2ws"
            }
