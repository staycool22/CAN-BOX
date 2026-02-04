import sys
import os
import time

# Ensure path is correct for importing joystick_controller
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

from driver.joystick_controller import JoystickController

def main():
    device_path = "/dev/input/js0"
    if len(sys.argv) > 1:
        device_path = sys.argv[1]
        
    print(f"Opening joystick device: {device_path}")
    print("Press Ctrl+C to exit.")
    print("Move sticks and buttons to see raw axis/button indices.")
    
    try:
        js = JoystickController(device_path=device_path)
    except FileNotFoundError:
        print(f"Error: Device {device_path} not found.")
        return

    try:
        while True:
            if js.poll():
                # Build a status string of all active axes and buttons
                status = []
                
                # Show first 8 axes
                axes_str = []
                for i in range(8):
                    val = js.axis_values.get(i, 0)
                    # Normalize for display roughly
                    if val != 0:
                        axes_str.append(f"Ax{i}:{val:6d}")
                
                if axes_str:
                    status.append(" | ".join(axes_str))
                
                # Show pressed buttons
                btns_str = []
                for k, v in js.button_values.items():
                    if v:
                        btns_str.append(f"Btn{k}")
                
                if btns_str:
                    status.append("Btns:" + ",".join(btns_str))
                
                if status:
                    # Clear line and print
                    print("\r" + " ".join(status) + " " * 20, end="", flush=True)
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
