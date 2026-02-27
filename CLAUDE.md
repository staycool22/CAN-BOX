# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

CAN-BOX (also called TZCAN) is a cross-platform CAN/CAN FD communication toolkit built on `python-can`. It supports Linux/WSL (SocketCAN) and Windows (candle/gs_usb) backends, and provides both a CLI test suite and a PySide6 GUI.

## Environment Setup

```bash
pip install -r requirements.txt          # python-can==4.6.1, numpy, pyside6
pip install python-can-candle            # Windows FD support only
```

**Linux SocketCAN interface setup** (required before using socketcan backend):
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
# For CAN FD:
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can0 up
```

**Automated device setup** (uses `DEVICE_CONFIG` in `socketcan_tool.py`):
```bash
python3 tools/socketcan_tool.py --discover   # Find devices and path hints
python3 tools/socketcan_tool.py --setup      # Auto-configure all interfaces
python3 tools/socketcan_tool.py --shutdown   # Bring down all interfaces
```

**Cannelloni bridge** (required for `TZETHCANTransmitter`):
```bash
./setup_cannelloni.sh
```

## Running Tests

Tests are standalone scripts, not a test framework. They also serve as **usage reference patterns** — each script demonstrates the recommended API for its scenario (single-channel, multi-channel socketcan, multi-channel candle, VESC protocol, etc.). Run from project root:

```bash
# Linux/WSL - Receive
python3 tests/new_test_tzcan_receive.py --mode can --iface can0 --can-br 500k
python3 tests/new_test_tzcan_receive.py --mode fd --iface can0 --fd-arb 500k --fd-dbr 2m --duration 10

# Linux/WSL - Send
python3 tests/new_test_tzcan_send.py --mode can --iface can0 --can-br 500k 1m --freq 1000
python3 tests/new_test_tzcan_send.py --mode fd --iface can0 --fd-arb 500k --fd-dbr 5m --fd-len 64 --freq 500

# Multi-channel (socketcan, Linux) — each channel is an independent socket, direct recv threads
python3 tests/test_tzcan_multichannel.py --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k --freq 10

# Multi-channel (candle, Windows) — shared Bus, requires dispatcher pattern
python tests/test_tzcan_multichannel_candle.py --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k

# VESC protocol
python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode receive
python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode rpm --rpm 2000

# Windows - Receive / Send
python tests/test_tzcan_receive_win.py --mode can --backend candle --index 0 --can-br 500k --filter-id 0x321
python tests/test_tzcan_send_win.py --mode can --backend gs_usb --index 0 --can-br 500k --freq 1000
python tests/test_tzcan_send_win.py --mode fd --backend candle --index 0 --fd-arb 500k --fd-dbr 5m --fd-len 64 --freq 500
```

## Launch GUI

```bash
python gui/main_gui.py          # Windows
python3 gui/main_gui.py         # Linux (may need: sudo apt-get install libxcb-cursor0)
```

## Architecture

### Core Abstraction Layer (`tzcan/`)

```
CANMessageTransmitter (abstract base + registry, tzcan/devices/base.py)
    ├── TZUSB2CANTransmitter  — python-can backend (socketcan / candle / gs_usb)
    └── TZETHCANTransmitter   — Hybrid UDP config + SocketCAN via cannelloni (vcan0-3)
```

**Factory pattern — preferred one-step API:**
```python
from tzcan import CANMessageTransmitter

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN", baud_rate=500000, channels=[0], backend="socketcan"
)
tx = TX(m_dev["buses"][0])   # always use m_dev["buses"][ch], works for any channel number
tx._send_can_data(send_id=0x123, data_list=[1,2,3,4])
TX.close_can_device(m_dev)
```

All test scripts use a **dual-import fallback** pattern to support both package import and direct script execution:
```python
try:
    from tzcan import CANMessageTransmitter
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from tzcan import CANMessageTransmitter
```

### `TZUSB2CANTransmitter` Internals

- **socketcan**: Opens `canX` interfaces directly; bitrate must be pre-configured via `ip link`.
- **candle**: Supports multi-channel shared Bus. Detects devices via `can.detect_available_configs("candle")` or `candle_api`, groups by serial number, and creates one `can.Bus` per physical device (shared across channels).
- **gs_usb**: CAN 2.0 only; no FD support.
- `init_can_device()` returns `(m_dev_dict, ch0_bus, ch1_bus)`. `m_dev` contains `{"backend": ..., "buses": {ch_idx: bus}, "config": {...}}`.
- In multi-channel candle mode, multiple `TZUSB2CANTransmitter` instances share the same `can.Bus`. `_receive_can_data` filters by `channel_id`; direct use in multi-instance setups is not recommended — use `CANCommunicator` instead.

### `CANCommunicator` (`gui/can_communicator.py`)

High-level wrapper for the GUI and multi-channel scenarios. Provides:
- Background receiver thread with per-channel message queuing
- Periodic/burst send scheduling
- Bus state monitoring (error passive, bus-off detection)
- FPS and bus load calculation

### `socketcan_tool.py` — Device Identity Management

`DEVICE_CONFIG` list is the single source of truth for mapping logical names (e.g., `can_handle_chassis`) to physical USB port hints and bitrate configs. The tool resolves stable physical paths via `/sys/class/net/<iface>/device` symlinks. External code uses:
```python
from tzcan.socketcan_tool import identify_can_devices, _can_idx
can_handles = identify_can_devices()  # {'can_handle_chassis': 'can5', ...}
channel_idx = _can_idx(can_handles.get('can_handle_chassis', 'can0'))
```

### `TZETHCANTransmitter` — Ethernet-CAN Bridge

- **Control plane**: Sends a custom UDP packet (Cannelloni OpCode 1) to `192.168.1.10`, port `20000 + channel_index` to configure baud rate on the HPM hardware board.
- **Data plane**: Uses SocketCAN on `vcan0`–`vcan3` interfaces bridged by `cannelloni`.
- Requires `setup_cannelloni.sh` to be running before use.

## Key Configuration Points

- **`socketcan_tool.py:DEVICE_CONFIG`**: Add/remove CAN devices here. Each entry maps a `path_hint` (USB port path substring from `--discover`) to a logical name, channel index, bitrate, and optional FD settings.
- **`TZETHCANTransmitter.py:ETHCANConstants`**: Change `TARGET_IP`, `TARGET_PORT_BASE`, and `PROTOCOL` for the Ethernet-CAN bridge.
- Bitrate strings accept `'500k'`, `'1m'`, `'2m'` format in CLI scripts (parsed by `parse_bitrate_token()`).
