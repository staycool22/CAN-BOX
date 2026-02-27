# CAN-BOX / TZCAN

**TZCAN** 是一款 USB 转 CAN 适配器（USB-to-CAN），提供一或多路 CAN/CAN FD 物理接口，通过 USB 连接至 PC。本项目是其跨平台 Python 驱动与上位机工具包，底层封装 `python-can`，支持 Linux/WSL 和 Windows。

### 硬件与驱动对应关系

| 操作系统 | `backend` 参数 | 驱动原理 | FD 支持 |
|---|---|---|---|
| Linux / WSL | `socketcan` | 内核将 USB 设备枚举为网络接口（`can0`、`can1` …），需 `ip link` 配置波特率 | ✓ |
| Windows | `candle` | Candle 协议 USB 驱动，波特率由 `init_can_device` 直接下发 | ✓ |
| Windows / Linux | `gs_usb` | GS\_USB 协议驱动（兼容 CANable 等），CAN 2.0 only | ✗ |

同一块硬件在 Linux 上用 `backend="socketcan"`，在 Windows 上用 `backend="candle"`。
多路设备（如双通道 TZCAN）在 candle 模式下共享一个 `can.Bus` 对象；在 socketcan 模式下每路独立为一个网络接口。

## 安装

```bash
pip install -r requirements.txt          # python-can==4.6.1, numpy, pyside6
pip install python-can-candle            # Windows FD 支持（可选）
```

**Linux/WSL — SocketCAN 接口配置**（使用前需执行一次）：

```bash
# CAN 2.0
sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up

# CAN FD
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on && sudo ip link set can0 up

# 自动发现并配置所有已注册设备（推荐）
python3 can_bridge/socketcan_tool.py --setup
```

## 快速开始

### 单通道（最小调用）

```python
from can_bridge import CANMessageTransmitter

TX, m_dev, _, _ = CANMessageTransmitter.open("TZCAN", baud_rate=500000, channels=[0])
tx = TX(m_dev["buses"][0])   # 始终用 m_dev["buses"][ch]，对任意通道号都正确

try:
    tx._send_can_data(0x123, [1, 2, 3, 4])
    ok, data, _ = tx._receive_can_data(target_id=0x123, timeout=0.5, return_msg=True)
finally:
    TX.close_can_device(m_dev)
```

### 多通道（最小调用）

```python
from can_bridge import CANMessageTransmitter

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZCAN", baud_rate=500000, channels=[0, 1, 2, 3]
)
txers = {ch: TX(m_dev["buses"][ch]) for ch in [0, 1, 2, 3]}

try:
    txers[0]._send_can_data(0x100, [0xAA])
    txers[2]._send_can_data(0x200, [0xBB])
finally:
    TX.close_can_device(m_dev)   # 一次关闭所有通道
```

> 详细用法（多通道并发收发、CAN FD、VESC 协议、ETHCAN）→ **[docs/usage.md](docs/usage.md)**

## 测试脚本

```bash
# Linux/WSL — 接收
python3 tests/new_test_tzcan_receive.py --mode can --iface can0 --can-br 500k
python3 tests/new_test_tzcan_receive.py --mode fd  --iface can0 --fd-arb 500k --fd-dbr 2m

# Linux/WSL — 发送
python3 tests/new_test_tzcan_send.py --mode can --iface can0 --can-br 500k --freq 1000
python3 tests/new_test_tzcan_send.py --mode fd  --iface can0 --fd-arb 500k --fd-dbr 5m --fd-len 64

# 多通道并发收发
python3 tests/test_tzcan_multichannel.py --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k

# VESC 电机
python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode receive

# Windows — 接收 / 发送（candle/gs_usb）
python tests/test_tzcan_receive_win.py --mode can --backend candle --index 0 --can-br 500k
python tests/test_tzcan_send_win.py    --mode fd  --backend candle --index 0 --fd-arb 500k --fd-dbr 5m
```

## GUI

```bash
python3 gui/main_gui.py          # Linux（若缺少组件：sudo apt-get install libxcb-cursor0）
python  gui/main_gui.py          # Windows
```

功能：设备/速率配置、实时报文收发（单次/周期/突发）、CAN/CAN FD/BRS、总线状态与负载、CSV 导出。

## 项目结构

```
can_bridge/
  devices/
    base.py       CANMessageTransmitter 抽象基类 + 注册表（open / choose_can_device）
    tzcan.py      TZCANTransmitter：python-can 后端（socketcan / candle / gs_usb）
    tzethcan.py   TZETHCANTransmitter：UDP 配置 + Cannelloni 数据面
  protocols/
    base.py       CANProtocolBase（别名 TZCanInterface）：send/receive/receiveFD 适配层
    vesc.py       VESC_CAN：VESC 无刷电机 CAN 协议编解码
  socketcan_tool.py   SocketCAN 设备发现与自动配置（DEVICE_CONFIG 为身份映射单一来源）
gui/
  can_communicator.py  后台收发/调度器，供 GUI 使用
  main_gui.py          PySide6 上位机
tests/
  new_test_tzcan_receive.py    Linux 接收
  new_test_tzcan_send.py       Linux 发送
  test_tzcan_multichannel.py   多通道并发收发
  test_tzcan_vesc.py           VESC 协议集成测试
  test_tzcan_receive_win.py    Windows 接收
  test_tzcan_send_win.py       Windows 发送
docs/
  usage.md    完整用法指南（单/多通道、FD、VESC、ETHCAN、API 速查）
```

## ETHCAN（以太网转 CAN）

需先运行 `./setup_cannelloni.sh`。详见 [docs/usage.md § ETHCAN](docs/usage.md#ethcan以太网转-can)。
