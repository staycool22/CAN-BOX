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
        button_b=1
    ):
        self.axis_lx = axis_lx
        self.axis_ly = axis_ly
        self.axis_rx = axis_rx
        self.axis_ry = axis_ry
        self.axis_lt = axis_lt
        self.axis_rt = axis_rt
        self.button_b = button_b


class JoystickController:
    def __init__(self, device_path="/dev/input/js0", deadzone=0.1, mapping=None):
        self.device_path = device_path
        self.deadzone = deadzone
        self.mapping = mapping or JoystickMapping()
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
        
        # Left Stick Logic: Only active in upper half (ly < 0)
        # Map Left(-1,0)->0°, Up(0,-1)->90°, Right(1,0)->180°
        # ly is -1 for Up, 1 for Down.
        
        active_stick = False
        angle_deg = None
        
        # Check if stick is in upper half (allowing for small deadzone around 0)
        # We treat "Up" as valid. "Down" (ly > deadzone) is invalid/ignored.
        magnitude = math.hypot(lx, ly)
        
        if magnitude >= self.deadzone:
            # If ly is positive (Down), we ignore the rotation or clamp?
            # User said "Remove lower half function", so we treat it as inactive.
            if ly <= self.deadzone: # Upper half (ly is negative or near 0)
                active_stick = True
                # Calculate angle: 180 (Left) -> 90 (Up) -> 0 (Right)
                # Reversed X axis mapping as requested.
                # math.atan2(y, x). 
                # standard atan2(dy, dx):
                #   dx=-lx (Flip Left/Right), dy=-ly (Up is positive Y)
                
                angle_deg = math.degrees(math.atan2(-lx, -ly)) + 90.0
                
                # Clamp to 0-180 just in case
                angle_deg = max(0.0, min(180.0, angle_deg))

        # Right Stick Logic for Speed (Forward/Backward)
        # ry: -1 (Up) -> 1 (Down)
        # We want Up -> Forward (Positive Speed), Down -> Backward (Negative Speed)
        ry = self._axis_normalized(self.mapping.axis_ry)
        speed_raw = -ry # Up(-1) becomes 1.0
        
        if abs(speed_raw) < self.deadzone:
            speed_raw = 0.0
            
        # Throttle (Trigger) - kept for compatibility or hybrid use
        throttle = self._trigger_normalized(self.mapping.axis_rt)
        emergency_stop = self._button_pressed(self.mapping.button_b)
        
        return {
            "active_stick": active_stick,
            "angle_deg": angle_deg, # 0-180
            "speed": speed_raw,     # -1.0 to 1.0
            "throttle": throttle,
            "throttle_pct": throttle * 100.0,
            "emergency_stop": emergency_stop
        }
