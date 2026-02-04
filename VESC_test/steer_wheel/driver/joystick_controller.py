import math
import os
import select
import struct
import time

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


class JoystickMapping:
    def __init__(
        self,
        axis_lx=0,
        axis_ly=1,
        axis_rx=3,
        axis_ry=4,
        axis_lt=2,
        axis_rt=5,
        button_a=0,
        button_b=1,
        button_x=2,
        button_y=3
    ):
        self.axis_lx = axis_lx
        self.axis_ly = axis_ly
        self.axis_rx = axis_rx
        self.axis_ry = axis_ry
        self.axis_lt = axis_lt
        self.axis_rt = axis_rt
        self.button_a = button_a
        self.button_b = button_b
        self.button_x = button_x
        self.button_y = button_y


class JoystickController:
    def __init__(self, device_path="/dev/input/js0", deadzone=0.2, mapping=None, control_mode_360=False):
        self.device_path = device_path
        self.deadzone = deadzone
        self.mapping = mapping or JoystickMapping()
        self.control_mode_360 = control_mode_360
        self.axis_values = {}
        self.button_values = {}
        self.trigger_signed = {}
        self.last_event_time = None
        self._open_device()

    def _open_device(self):
        if not os.path.exists(self.device_path):
            raise FileNotFoundError(self.device_path)
        self.fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)

    def close(self):
        if hasattr(self, "fd") and self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def poll(self):
        updated = False
        if self.fd is None:
            return updated
        while True:
            rlist, _, _ = select.select([self.fd], [], [], 0)
            if not rlist:
                break
            try:
                data = os.read(self.fd, 8)
            except BlockingIOError:
                break
            if not data or len(data) < 8:
                break
            time_ms, value, event_type, number = struct.unpack("IhBB", data)
            event_type = event_type & ~JS_EVENT_INIT
            self.last_event_time = time.time()
            if event_type == JS_EVENT_AXIS:
                self.axis_values[number] = value
                updated = True
            elif event_type == JS_EVENT_BUTTON:
                self.button_values[number] = value
                updated = True
        return updated

    def _axis_normalized(self, axis_index):
        raw = self.axis_values.get(axis_index, 0)
        if raw > 32767:
            raw = 32767
        if raw < -32767:
            raw = -32767
        return raw / 32767.0

    def _trigger_normalized(self, axis_index):
        raw = self.axis_values.get(axis_index, 0)
        signed = self.trigger_signed.get(axis_index)
        if signed is None:
            signed = raw < 0
            self.trigger_signed[axis_index] = signed
        if signed:
            value = (raw + 32767.0) / 65534.0
        else:
            value = raw / 32767.0
        if value < 0.0:
            value = 0.0
        if value > 1.0:
            value = 1.0
        return value

    def _button_pressed(self, button_index):
        return bool(self.button_values.get(button_index, 0))

    def get_command(self):
        lx = self._axis_normalized(self.mapping.axis_lx)
        ly = self._axis_normalized(self.mapping.axis_ly)
        
        # Shared Variables
        active_stick = False
        angle_deg = None
        speed_raw = 0.0
        
        # Calculate Magnitude for Deadzone check
        magnitude = math.hypot(lx, ly)
        
        # --- NEW MODE: 360 Degree + Backward Logic ---
        if self.control_mode_360:
            if magnitude >= self.deadzone:
                active_stick = True
                
                # Calculate full 360 angle (-180 to 180 -> 0 to 360)
                # math.atan2(y, x). 
                # Use same coordinate system as legacy: dx=-lx, dy=-ly (Up is positive Y)
                raw_angle = math.degrees(math.atan2(-lx, -ly)) + 90.0
                
                # Normalize to 0-360
                if raw_angle < 0:
                    raw_angle += 360.0
                elif raw_angle >= 360.0:
                    raw_angle -= 360.0
                
                # Logic: 0-180 is Forward (Upper Half), 180-360 is Backward (Lower Half)
                # If Angle is in [180, 360), reverse speed and map angle to "Forward" equivalent
                if 180.0 <= raw_angle < 360.0:
                    # Backward Motion
                    speed_raw = -magnitude
                    # Map to opposite angle in upper half
                    # Example: 270 (Down) -> 90 (Up) -> Steer Straight
                    # Formula: angle - 180
                    angle_deg = raw_angle - 180.0
                else:
                    # Forward Motion (0-180)
                    speed_raw = magnitude
                    angle_deg = raw_angle
            
            # Note: In this mode, Right Stick RY (speed) is ignored/overridden by Left Stick Magnitude
        
        # --- LEGACY MODE: Upper Half Only ---
        else:
            # Left Stick Logic: Only active in upper half (ly < 0)
            if magnitude >= self.deadzone:
                if ly <= self.deadzone: # Upper half (ly is negative or near 0)
                    active_stick = True
                    angle_deg = math.degrees(math.atan2(-lx, -ly)) + 90.0
                    angle_deg = max(0.0, min(180.0, angle_deg))

            # Right Stick Logic for Speed (Forward/Backward)
            ry = self._axis_normalized(self.mapping.axis_ry)
            speed_raw = -ry # Up(-1) becomes 1.0
            if abs(speed_raw) < self.deadzone:
                speed_raw = 0.0
        
        # Right Stick Horizontal for Spin (Shared)
        rx = self._axis_normalized(self.mapping.axis_rx)
        spin_raw = rx
        if abs(spin_raw) < self.deadzone:
            spin_raw = 0.0
            
        # Throttle (Trigger) - kept for compatibility or hybrid use
        throttle = self._trigger_normalized(self.mapping.axis_rt)
        emergency_stop = self._button_pressed(self.mapping.button_b)
        switch_mode = self._button_pressed(self.mapping.button_a)
        toggle_braking = self._button_pressed(self.mapping.button_y)
        
        return {
            "active_stick": active_stick,
            "angle_deg": angle_deg, # 0-180 (mapped)
            "speed": speed_raw,     # -1.0 to 1.0
            "spin": spin_raw,       # -1.0 to 1.0 (Left/Right)
            "throttle": throttle,
            "throttle_pct": throttle * 100.0,
            "emergency_stop": emergency_stop,
            "switch_mode": switch_mode,
            "toggle_braking": toggle_braking
        }
