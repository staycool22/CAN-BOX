# CAN-BOX 用法指南

## 目录

1. [硬件说明](#硬件说明)
2. [核心概念](#核心概念)
3. [单通道](#单通道)
4. [多通道](#多通道)
5. [多通道并发收发](#多通道并发收发)
6. [协议层 — VESC](#协议层--vesc)
7. [ETHCAN（以太网转 CAN）](#ethcan以太网转-can)
8. [API 速查](#api-速查)

---

## 硬件说明

**TZCAN** 是一款 USB 转 CAN 适配器（USB-to-CAN），提供一或多路 CAN/CAN FD 物理接口，通过 USB 连接至 PC。`TZUSB2CANTransmitter` 是其 Python 驱动，底层封装 `python-can`。

### backend 与硬件驱动的对应关系

| `backend` | 适用 OS | 驱动原理 | FD | 波特率配置方 |
|---|---|---|---|---|
| `socketcan` | Linux / WSL | 内核将 USB 设备枚举为网络接口（`can0`、`can1` …） | ✓ | `ip link set canX type can bitrate N` |
| `candle` | Windows | Candle 协议 USB 驱动 | ✓ | `init_can_device` 直接下发 |
| `gs_usb` | Win / Linux | GS\_USB 协议（兼容 CANable 等），CAN 2.0 only | ✗ | `init_can_device` 直接下发 |

同一块硬件在不同 OS 上切换 backend 即可，代码其余部分不变：

```python
# Linux
TX, m_dev, _, _ = CANMessageTransmitter.open("TZUSB2CAN", baud_rate=500000,
                                              channels=[0], backend="socketcan")
# Windows（同一台设备）
TX, m_dev, _, _ = CANMessageTransmitter.open("TZUSB2CAN", baud_rate=500000,
                                              channels=[0], backend="candle")
```

### 多路通道的物理含义

- **socketcan**：每路通道 = 独立的 Linux 网络接口（`can0`、`can1` …），有各自的 socket，接收互不干扰。
- **candle**：同一物理设备的多路通道共享一个 `can.Bus` 对象，需用 dispatcher 模式分发消息（详见[多通道并发收发](#多通道并发收发)）。

### `channels` 参数与物理接口的映射

```
socketcan：channels=[0, 1, 2, 3]  →  can0, can1, can2, can3
candle   ：channels=[0, 1]         →  设备扁平索引 0, 1（按序列号排序）
```

`m_dev["buses"]` 始终以 `channels` 中的值为 key，无论哪种 backend：

```python
m_dev["buses"][0]   # 第一路
m_dev["buses"][2]   # 第三路（socketcan: can2；candle: 扁平索引2的通道）
```

---

## 核心概念

调用 `open()` 后会得到三样东西，用途各不相同：

```
CANMessageTransmitter.open("TZUSB2CAN", baud_rate=500000, channels=[0, 1])
  ↓
TX      — TZUSB2CANTransmitter 类（未实例化）
            用于：TX(bus) 实例化 / TX.close_can_device(m_dev) 关闭
m_dev   — 字典，记录本次初始化状态
            {
              "backend": "socketcan",
              "buses": {0: Bus(can0), 1: Bus(can1)},   ← 按通道号取 Bus
              "config": {...}
            }
_, _    — ch0/ch1 快捷引用（仅保证 key=0 和 key=1 有值，其他通道为 None）
```

**取 Bus 的正确方式**：始终用 `m_dev["buses"][ch]`，对任意通道号都正确。
`ch0`/`ch1` 只是 `buses[0]`/`buses[1]` 的快捷引用，通道号 ≥ 2 时它们是 `None`。

---

## 单通道

### 最小调用

```python
from tzcan import CANMessageTransmitter

TX, m_dev, _, _ = CANMessageTransmitter.open("TZUSB2CAN", baud_rate=500000, channels=[0])
tx = TX(m_dev["buses"][0])

try:
    tx._send_can_data(0x123, [1, 2, 3, 4])
    ok, data, _ = tx._receive_can_data(target_id=0x123, timeout=0.5, return_msg=True)
    if ok:
        print("收到:", data)
finally:
    TX.close_can_device(m_dev)
```

### 常用参数

| 参数 | 说明 | 示例值 |
|---|---|---|
| `baud_rate` | 仲裁域波特率 | `500000` / `1000000` |
| `dbit_baud_rate` | 数据域波特率（FD） | `2000000` |
| `channels` | 要打开的通道列表 | `[0]` / `[2]` |
| `backend` | CAN 后端 | `"socketcan"` / `"candle"` / `"gs_usb"` |
| `fd` | 是否启用 CAN FD | `True` / `False` |

### CAN FD（Windows candle 后端）

```python
TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN", baud_rate=500000, dbit_baud_rate=2000000,
    channels=[0], backend="candle", fd=True
)
tx = TX(m_dev["buses"][0])
tx._send_can_data(0x456, [i % 256 for i in range(16)], canfd_mode=True, brs=1)
TX.close_can_device(m_dev)
```

### 任意通道号（can2、can3 等）

```python
# 打开 can2（注意：不能用 ch0，ch0 此时为 None）
TX, m_dev, _, _ = CANMessageTransmitter.open("TZUSB2CAN", baud_rate=500000, channels=[2])
tx = TX(m_dev["buses"][2])   # 正确：按实际通道号取
```

---

## 多通道

### 最小调用

```python
from tzcan import CANMessageTransmitter

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN", baud_rate=500000, channels=[0, 1, 2, 3]
)
# 按通道号批量实例化
txers = {ch: TX(m_dev["buses"][ch]) for ch in [0, 1, 2, 3]}

try:
    txers[0]._send_can_data(0x100, [0xAA])
    txers[2]._send_can_data(0x200, [0xBB])
finally:
    TX.close_can_device(m_dev)   # 一次关闭所有通道
```

一次 `open()` 打开所有通道，一次 `close_can_device()` 关闭所有通道。

### socketcan vs candle 的区别

| 后端 | `m_dev["buses"]` | 共享 Bus？ | 接收线程是否互干扰？ |
|---|---|---|---|
| socketcan | `{0: Bus(can0), 1: Bus(can1), ...}` | 否 | 否（各自独立 socket） |
| candle | `{0: shared_bus, 1: shared_bus, ...}` | **是（同一对象）** | **是（需 dispatcher）** |

candle 多通道共享一个 Bus：若多个线程同时调 `bus.recv()`，每条消息只会被
其中一个线程取走。详见下一节的 dispatcher 模式。

---

## 多通道并发收发

`tests/test_tzcan_multichannel.py` 是完整的可运行示例，这里展示核心结构：

```python
import queue, threading
from collections import defaultdict
from tzcan import CANMessageTransmitter

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN", baud_rate=500000, channels=[0, 1, 2, 3], backend="socketcan"
)

# channel_id：socketcan 不需要（各 Bus 独立），candle 需要（路由发送帧）
is_socketcan = (m_dev["backend"] == "socketcan")
txers = {
    ch: TX(m_dev["buses"][ch], channel_id=None if is_socketcan else ch)
    for ch in [0, 1, 2, 3]
}

# ── 接收：dispatcher + 队列 ──────────────────────────────────────────────
# 按 id(bus) 分组：socketcan 每通道独立，candle 多通道同一对象
bus_groups = defaultdict(list)
for ch, t in txers.items():
    bus_groups[id(t.bus)].append(ch)

rx_queues = {ch: queue.Queue() for ch in txers}
stop = threading.Event()

def dispatcher(bus, chs):
    single = chs[0] if len(chs) == 1 else None
    while not stop.is_set():
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        if single is not None:
            rx_queues[single].put_nowait(msg)       # socketcan：直接入队
        else:
            ch = getattr(msg, 'channel', None)      # candle：按 msg.channel 派发
            rx_queues.get(ch, rx_queues[chs[0]]).put_nowait(msg)

for chs in bus_groups.values():
    threading.Thread(target=dispatcher, args=(txers[chs[0]].bus, chs),
                     daemon=True).start()

# ── 发送：定时在 ch0、ch1 发送 ───────────────────────────────────────────
def sender():
    seq = 0
    while not stop.is_set():
        for ch in [0, 1]:
            txers[ch]._send_can_data(0x123, [seq & 0xFF, ch])
        seq += 1
        threading.Event().wait(0.1)   # 10 Hz

threading.Thread(target=sender, daemon=True).start()

# ── 消费：打印各通道收到的帧 ─────────────────────────────────────────────
try:
    while True:
        for ch, q in rx_queues.items():
            if not q.empty():
                msg = q.get_nowait()
                print(f"ch{ch} RX: 0x{msg.arbitration_id:X} {msg.data.hex()}")
        threading.Event().wait(0.01)
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    TX.close_can_device(m_dev)
```

也可直接运行内置测试脚本：

```bash
# socketcan（Linux）：监听 4 路，在 can0/can1 发送
python3 tests/test_tzcan_multichannel.py \
    --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k --freq 10

# candle（Windows）
python3 tests/test_tzcan_multichannel.py \
    --backend candle --channels 0 1 2 3 --tx-channels 0 1 --can-br 500k
```

---

## 协议层 — VESC

协议层与硬件层**完全解耦**：先初始化硬件，再把 `TZUSB2CANTransmitter` 实例注入协议层。
**一个 `VESC_CAN` 实例对应一条 CAN 总线**，协议层不做通道路由。

### 最小调用

```python
from tzcan import CANMessageTransmitter
from tzcan.protocols.vesc import VESC_CAN

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN", baud_rate=500000, channels=[0], backend="socketcan"
)
vesc = VESC_CAN(TX(m_dev["buses"][0]))

try:
    vesc.send_rpm(vesc_id=1, rpm=2000)
    _, pack = vesc.receive_decode(timeout=0.1)
    if pack:
        print(f"RPM={pack.rpm:.0f}  I={pack.current:.2f}A")
finally:
    TX.close_can_device(m_dev)
```

### 多电机场景

**同一条总线上的多个 VESC**：用同一实例，靠 `vesc_id` 区分（已编码在仲裁 ID 中）：

```python
vesc = VESC_CAN(TX(m_dev["buses"][0]))
vesc.send_rpm(vesc_id=1, rpm=2000)   # 电机 #1
vesc.send_rpm(vesc_id=2, rpm=1500)   # 电机 #2，同一总线
```

**不同总线上的 VESC**：各自创建独立实例，通道管理在应用层：

```python
TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN", baud_rate=500000, channels=[0, 1]
)
vesc0 = VESC_CAN(TX(m_dev["buses"][0]))   # can0 上的电机
vesc1 = VESC_CAN(TX(m_dev["buses"][1]))   # can1 上的电机

vesc0.send_rpm(vesc_id=1, rpm=2000)
vesc1.send_rpm(vesc_id=1, rpm=3000)
```

### 测试脚本

```bash
python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode receive
python3 tests/test_tzcan_vesc.py --iface 2 --can-br 500k --vesc-id 1 --mode rpm --rpm 2000
```

---

## ETHCAN（以太网转 CAN）

`TZETHCANTransmitter` 面向以太网转 CAN 硬件（HPM 系列），采用**混合双平面架构**：

- **控制平面**：Python 通过 UDP/TCP 向硬件板下发波特率配置包（OpCode 1）
- **数据平面**：报文通过 cannelloni 桥接，在 `vcan0`–`vcan3` 上以 SocketCAN 收发

```
Python ──_send_can_data()──▶ vcanX ──cannelloni(UDP)──▶ HPM CAN 总线
Python ◀──_receive_can_data()── vcanX ◀──cannelloni(UDP)── HPM CAN 总线
```

### 前置条件

仅适用于 **Linux / WSL**，需要 `cannelloni` 已安装且在 `$PATH`：

```bash
cd tools/eth
./setup_cannelloni.sh   # 加载 vcan 内核模块，创建 vcan0–vcan3，启动 4 个 cannelloni 进程
```

脚本所建立的映射关系：

| vcan 接口 | cannelloni 端口（本地 & 远端） | 对应 CAN 通道 |
|---|---|---|
| `vcan0` | `20000` | CH0 |
| `vcan1` | `20001` | CH1 |
| `vcan2` | `20002` | CH2 |
| `vcan3` | `20003` | CH3 |

默认远端 IP `192.168.1.10`，有三种修改方式（优先级从高到低）：

1. **Python API `target_ip` 参数**：调用 `CANMessageTransmitter.open()` 时传入，仅对本次调用生效。
2. **修改源码常量**（永久）：编辑 `tzcan/devices/tzethcan.py` 中的 `ETHCANConstants.TARGET_IP`，或编辑 `tools/eth/setup_cannelloni.sh` 顶部的 `REMOTE_IP`。

### 初始化流程说明

`init_can_device` 内部分两个阶段，确保硬件完成波特率切换后再打开 vcan 接口：

1. **阶段 1**：向所有请求通道依次发送 UDP 配置包（波特率生效）
2. **等待 100 ms**：HPM 硬件重新配置 CAN 控制器
3. **阶段 2**：逐一打开对应的 `vcanX` SocketCAN 接口

### 最小调用（Python API）

```python
from tzcan import CANMessageTransmitter

# CAN 2.0，单通道
TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZETHCAN", baud_rate=500000, channels=[0], fd=False
    # target_ip="192.168.10.5"  # 可选：覆盖默认目标 IP
)
tx = TX(m_dev["buses"][0])
tx._send_can_data(0x123, [1, 2, 3, 4])
ok, data = tx._receive_can_data(timeout=1.0)[:2]
TX.close_can_device(m_dev)

# CAN FD，4 通道并发
TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZETHCAN", baud_rate=500000, dbit_baud_rate=2000000,
    channels=[0, 1, 2, 3], fd=True
)
txers = {ch: TX(m_dev["buses"][ch], channel_id=ch, is_canfd=True)
         for ch in [0, 1, 2, 3]}
txers[0]._send_can_data(0x100, [i % 256 for i in range(64)], canfd_mode=True, brs=1)
TX.close_can_device(m_dev)
```

### 命令行工具

提供开箱即用的收发测试脚本（`tools/eth/`），参数风格与 USB-CAN 工具一致：

#### ethcan_recv.py — 接收测试

```bash
# CAN 2.0，vcan0，500kbps，持续接收直至 ESC
python3 tools/eth/ethcan_recv.py --channels 0

# CAN FD，vcan0，数据域 5Mbps，仅显示 FD 帧
python3 tools/eth/ethcan_recv.py --channels 0 --mode fd --dbit-baud-rate 5m

# 4 通道同时接收，FD 总线，显示全部帧，持续 30 秒
python3 tools/eth/ethcan_recv.py --channels 0 1 2 3 --mode all \
    --dbit-baud-rate 2m --duration 30

# 指定非默认 HPM 硬件 IP
python3 tools/eth/ethcan_recv.py --channels 0 --ip 192.168.10.5

# 仅接收指定 ID，安静模式（只输出统计）
python3 tools/eth/ethcan_recv.py --channels 0 --filter-id 0x123 --quiet
```

`--mode` 同时控制**硬件初始化模式**和**接收过滤行为**：

| `--mode` | 总线初始化 | 接收过滤 |
|---|---|---|
| `all`（默认）| FD 模式（可收 CAN 2.0 + FD） | 不过滤，全部显示 |
| `fd` | FD 模式 | 仅显示 FD 帧 |
| `can` | CAN 2.0 模式 | 仅显示 CAN 2.0 帧 |

#### ethcan_send.py — 发送测试

```bash
# CAN 2.0，vcan0，500kbps，1000 Hz，发送 10000 帧
python3 tools/eth/ethcan_send.py --channels 0

# CAN FD，4 通道并发，64 字节帧，500 Hz
python3 tools/eth/ethcan_send.py --channels 0 1 2 3 --mode fd \
    --dbit-baud-rate 2m --fd-len 64 --freq 500

# 指定非默认 HPM 硬件 IP
python3 tools/eth/ethcan_send.py --channels 0 --ip 192.168.10.5

# 指定 ID，大批量发送
python3 tools/eth/ethcan_send.py --channels 0 --send-id 0x321 \
    --count 100000 --freq 2000
```

`--mode can`（默认）发 CAN 2.0；`--mode fd` 发 CAN FD（`--dbit-baud-rate`、`--fd-len`、`--no-brs` 此时生效）。

### 配置常量

```python
# tzcan/devices/tzethcan.py
class ETHCANConstants:
    TARGET_IP        = "192.168.1.10"  # HPM 硬件板 IP（全局默认值）
    TARGET_PORT_BASE = 20000           # 端口 = BASE + channel_index
    PROTOCOL         = "UDP"           # "UDP" 或 "TCP"
```

> **运行时覆盖 IP**：无需修改源码，可通过以下两种方式临时指定目标 IP：
> - CLI：`--ip <IP地址>`（例如 `--ip 192.168.10.5`）
> - Python API：`CANMessageTransmitter.open("TZETHCAN", ..., target_ip="192.168.10.5")`

---

## API 速查

### `CANMessageTransmitter`（`tzcan/devices/base.py`）

| 方法 | 说明 |
|---|---|
| `open(device, **kwargs)` | 选择设备 + 初始化，返回 `(TX, m_dev, ch0, ch1)`。`device` 为注册表键（`"TZUSB2CAN"`/`"TZETHCAN"`），`**kwargs` 透传给 `init_can_device` |
| `choose_can_device(device)` | 仅返回设备类（不初始化） |
| `register(key)` | 装饰器，注册新设备实现 |

### `TZUSB2CANTransmitter` 实例方法

| 方法 | 说明 |
|---|---|
| `_send_can_data(send_id, data_list, is_ext_frame, canfd_mode, brs, esi)` | 发送一帧 |
| `_receive_can_data(target_id, timeout, is_ext_frame, canfd_mode, return_msg)` | 接收一帧，返回 `(ok, data[, msg])` |

### `TZUSB2CANTransmitter.init_can_device()` 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `baud_rate` | `500000` | 仲裁域波特率 |
| `dbit_baud_rate` | `500000` | 数据域波特率（FD 模式使用） |
| `channels` | `[0]` | 通道列表，结果在 `m_dev["buses"]` 中按 key 访问 |
| `fd` | `False` | 是否启用 CAN FD |
| `backend` | 平台自动 | `"socketcan"` / `"candle"` / `"gs_usb"` |
| `sp` | `None` | 仲裁域采样点（0–1 或 0–100） |
| `dsp` | `None` | 数据域采样点 |
