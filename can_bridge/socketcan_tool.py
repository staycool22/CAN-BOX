import glob
import os
import platform
import subprocess
import sys
import time

# --- Linux Specific Configuration ---
# --- Single Source of Truth for CAN Device Configuration ---
# To add or remove a device, simply add or remove a dictionary from this list.
DEVICE_CONFIG = [
    {
        "name": "can_handle_arm_hole",
        "path_hint": "1-2",
        "channel": 1,
        "bitrate": 500000,
        "default_iface": "can3",
    },
    {
        "name": "can_handle_arm_tree",
        "path_hint": "1-5",
        "channel": 0,
        "bitrate": 500000,
        "default_iface": "can1",
    },
    {
        "name": "can_handle_6d_force_sensor",
        "path_hint": "1-5",
        "fd": True,
        "channel": 1,
        "bitrate": 1000000,
        "dbitrate": 5000000,
        "default_iface": "can0",
    },

    # {
    #     "name": "xxx",
    #     "bitrate": 500000,
    #     "dbitrate": 2000000,
    #     "fd": True,
    #     "sample_point": 0.875,       # 仲裁段采样点
    #     "dsample_point": 0.80,       # 数据段采样点（CAN‑FD）
    #     "default_iface": "can0",
    # }

    # {
    #     "name": "can_handle_dexhand",
    #     "path_hint": "1-1:1.0",
    #     "fd": True,
    #     "channel": 0,
    #     "bitrate": 1000000,
    #     "dbitrate": 5000000,
    #     "default_iface": "can0",
    # }
]

def _can_idx(x):
    return int(x[3:]) if isinstance(x, str) and x.startswith('can') else 0

def get_device_physical_path(interface_name):
    """
    Resolves the physical device path for a given network interface (e.g., 'can0').
    Returns a partial, stable path relative to the /sys/devices/ tree.
    """
    try:
        symlink_path = f"/sys/class/net/{interface_name}/device"
        if os.path.islink(symlink_path):
            # Resolve the symlink to the actual physical device directory
            physical_dir = os.path.realpath(symlink_path)
            
            # The goal is to get a path that is stable across reboots.
            # The full path is often long and includes PCI bus details that might change.
            # A common stable portion is the USB port path.
            # e.g., /sys/devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4:1.0
            # We want to extract something like "1-4/1-4:1.0"
            
            base_path = "/sys/devices/"
            if physical_dir.startswith(base_path):
                return physical_dir[len(base_path):]
        return None
    except Exception:
        return None

def identify_can_devices_linux():
    """
    Scans for 'can*' interfaces and matches them based on the DEVICE_CONFIG.
    This version handles multi-channel devices by grouping interfaces by a shared path
    and then assigning them based on a sorted order (e.g., can0, can1).
    """
    can_interfaces = glob.glob('/sys/class/net/can*')
    if not can_interfaces:
        print("Warning: No SocketCAN interfaces (can*) found.")
        return {}

    # 1. Group available interfaces by their physical path hint
    grouped_by_path = {}
    path_hints = {device["path_hint"] for device in DEVICE_CONFIG}
    for iface_path in can_interfaces:
        iface_name = os.path.basename(iface_path)
        physical_path = get_device_physical_path(iface_name)
        if not physical_path:
            continue

        for hint in path_hints:
            if hint in physical_path:
                if hint not in grouped_by_path:
                    grouped_by_path[hint] = []
                grouped_by_path[hint].append(iface_name)
                break

    # 2. Sort interfaces within each group to ensure stable ordering (can0 before can1)
    for path_hint, ifaces in grouped_by_path.items():
        ifaces.sort(key=lambda name: int(name.replace("can", "")))

    # 3. Map logical names to the sorted interfaces based on DEVICE_CONFIG
    found_devices = {}
    for device_config in DEVICE_CONFIG:
        path_hint = device_config["path_hint"]
        logical_name = device_config["name"]
        channel_index = device_config["channel"]

        if path_hint in grouped_by_path:
            sorted_ifaces = grouped_by_path[path_hint]
            if channel_index < len(sorted_ifaces):
                iface = sorted_ifaces[channel_index]
                found_devices[logical_name] = iface
                print(f"Mapped {logical_name} to {iface} (path: {path_hint}, channel: {channel_index})")
            else:
                print(f"Warning: Channel index {channel_index} for {logical_name} is out of bounds for device {path_hint}. Only {len(sorted_ifaces)} interfaces found.")
        else:
            print(f"Warning: Path hint '{path_hint}' for {logical_name} not found among discovered devices.")

    # 4. Check for any configured logical names that were not found
    for device_config in DEVICE_CONFIG:
        if device_config["name"] not in found_devices:
            print(f"Warning: Logical name '{device_config['name']}' could not be mapped. Check config and connections.")

    return found_devices

def identify_can_devices():
    """
    Identifies CAN interfaces based on the operating system and physical device paths.
    Returns a dictionary mapping logical names to interface names (e.g., 'can_handle_chassis_A': 'can0').
    This function is the main entry point for external callers.
    """
    # Generate the default mapping dynamically from the single source of truth
    default_interfaces = {device["name"]: device["default_iface"] for device in DEVICE_CONFIG}

    if platform.system() == "Linux":
        print("Running on Linux, attempting to identify devices by physical path...")
        try:
            identified_devices = identify_can_devices_linux()
            
            # If any devices were not found by path, fall back to defaults for the missing ones.
            # This ensures the returned dictionary is always complete.
            if len(identified_devices) < len(default_interfaces):
                 print("Warning: Not all devices were identified by physical path. Falling back to defaults for missing ones.")
                 # Create a combined dictionary, giving precedence to identified devices
                 full_device_map = default_interfaces.copy()
                 full_device_map.update(identified_devices) # Overwrite defaults with identified ones
                 return full_device_map

            return identified_devices
        except Exception as e:
            print(f"Error during physical device identification: {e}. Falling back to default interface names.")
            return default_interfaces
    else:
        print(f"Running on {platform.system()}, using default interface names.")
        return default_interfaces

def run_command(command):
    """Executes a shell command with sudo and handles errors."""
    try:
        print(f"Executing: {' '.join(command)}")
        # Using subprocess.run for better error handling
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print("Error:", result.stderr)
    except FileNotFoundError:
        print(f"Error: The command '{command[0]}' was not found. Is 'ip' or 'sudo' installed and in your PATH?")
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {' '.join(command)}")
        print(f"Return code: {e.returncode}")
        print(f"Output:\n{e.stdout}")
        print(f"Error Output:\n{e.stderr}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

def setup_can_interfaces(identified_devices, default_bitrate=1000000):
    """
    Sets up the identified CAN interfaces using bitrate information from DEVICE_CONFIG.
    """
    if platform.system() != "Linux":
        print("CAN interface setup is only supported on Linux.")
        return

    # Create a config lookup map from the single source of truth
    config_map = {device["name"]: device for device in DEVICE_CONFIG}

    print("\n--- Setting up CAN interfaces ---")
    for logical_name, iface in identified_devices.items():
        device_conf = config_map.get(logical_name)
        bitrate = device_conf.get("bitrate", default_bitrate) if device_conf else default_bitrate
        
        print(f"Setting up {logical_name} ({iface}) with bitrate {bitrate}...")
        # 1. Bring the link down
        run_command(["sudo", "ip", "link", "set", iface, "down"])
        
        # 2. Set the type to CAN with the specified bitrate
        cmd = ["sudo", "ip", "link", "set", iface, "type", "can", "bitrate", str(bitrate)]
        
        # Add FD support if configured
        if device_conf and device_conf.get("fd"):
            dbitrate = device_conf.get("dbitrate")
            if dbitrate:
                cmd.extend(["dbitrate", str(dbitrate), "fd", "on"])
                print(f"  - Enabled CAN FD with dbitrate {dbitrate}")
            else:
                 print(f"  - Warning: FD enabled for {logical_name} but no dbitrate specified. Ignoring FD.")

        sp = None
        dsp = None
        if device_conf:
            sp = device_conf.get("sample_point", device_conf.get("sp"))
            dsp = device_conf.get("data_sample_point", device_conf.get("dsp"))
        if sp is not None:
            cmd.extend(["sample-point", str(sp)])
        if dsp is not None and device_conf and device_conf.get("fd"):
            cmd.extend(["dsample-point", str(dsp)])

        run_command(cmd)
        # 3. Bring the link up
        run_command(["sudo", "ip", "link", "set", iface, "up"])
    print("--- CAN interfaces setup complete ---\n")

def shutdown_can_interfaces(interfaces):
    """
    Brings down the specified CAN interfaces.
    """
    if platform.system() != "Linux":
        return
        
    print("\n--- Shutting down CAN interfaces ---")
    for iface in interfaces:
        print(f"Bringing down {iface}...")
        run_command(["sudo", "ip", "link", "set", iface, "down"])
    print("--- CAN interfaces shut down complete ---\n")


def discover_can_devices():
    """
    Scans and prints all found CAN interfaces, grouped by their physical USB device.
    This helps the user to easily find the correct `path_hint` for DEVICE_MAPPING.
    """
    print("\n--- Discovering CAN Interfaces ---")
    can_interfaces = glob.glob('/sys/class/net/can*')
    if not can_interfaces:
        print("No CAN interfaces (can*) found.")
        return

    # Group interfaces by their base physical path to identify multi-channel devices
    grouped_by_base_path = {}
    for iface_path in can_interfaces:
        iface_name = os.path.basename(iface_path)
        physical_path = get_device_physical_path(iface_name)
        if not physical_path:
            print(f"- Could not resolve physical path for {iface_name}")
            continue
        
        # Heuristic to find the USB port path (e.g., '.../usb1/1-8/1-8.1:1.0/...' -> '1-8' or '1-8.1:1.0')
        # This is a bit of a guess, but usually works for USB-CAN adapters.
        parts = physical_path.split('/')
        base_path_hint = None
        
        # Check if the last part contains ':' (e.g. 1-1:1.0) - use it for more specific matching if needed
        # This allows distinguishing between different interfaces on the same USB composite device
        if parts and ':' in parts[-1]:
            base_path_hint = parts[-1]
        else:
            for i, part in enumerate(parts):
                if part.startswith("usb") and i + 1 < len(parts):
                    # The next part is usually the port identifier, e.g., '1-8'
                    base_path_hint = parts[i+1]
                    break
        
        if base_path_hint is None:
            # Fallback for non-standard paths
            base_path_hint = physical_path

        if base_path_hint not in grouped_by_base_path:
            grouped_by_base_path[base_path_hint] = []
        grouped_by_base_path[base_path_hint].append(iface_name)

    print("\nFound the following CAN devices grouped by physical USB port:")
    print("============================================================")

    for base_path, ifaces in grouped_by_base_path.items():
        # Sort interfaces to provide a stable order for channel indexing (can0, can1, ...)
        ifaces.sort(key=lambda name: int(name.replace("can", "")))
        
        print(f"\nUSB Device Path Hint: \"{base_path}\"")
        print(f"  - This device has {len(ifaces)} CAN channel(s).")
        print( "  - Use this hint in your DEVICE_MAPPING.")
        print( "  - Channels (sorted by name):")
        for i, iface in enumerate(ifaces):
            print(f"    - Channel Index {i}: {iface}")

    print("\n============================================================")
    print("\nTo configure, update the DEVICE_MAPPING in the script with the path hints and channel mappings.")


def main():
    """
    Main function for standalone script execution.
    Can be used to discover devices or to setup/shutdown all configured interfaces.
    """
    if '--discover' in sys.argv:
        discover_can_devices()
        return

    if '--setup' in sys.argv:
        devices = identify_can_devices()
        if devices:
            setup_can_interfaces(devices)
        return

    if '--shutdown' in sys.argv:
        devices = identify_can_devices()
        if devices:
            shutdown_can_interfaces(devices.values())
        return

    print("Usage:")
    print("  python socketcan_tool.py --discover   - Scan and show available CAN devices and paths.")
    print("  python socketcan_tool.py --setup      - Identify and configure all CAN devices.")
    print("  python socketcan_tool.py --shutdown   - Shut down all configured CAN devices.")
    print("\nRunning a default discovery...")
    discover_can_devices()

if __name__ == "__main__":
    main()
