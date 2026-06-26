# CAN-BOX Backend 快速开始

如果你想看完整 API、单通道/多通道、VESC、ETHCAN 等内容，请转到 [usage.md](./usage.md)。

## 1. backend 选型

| backend | 适用系统 | 典型硬件/驱动前提 | CAN FD | 适合场景 |
|---|---|---|---|---|
| `socketcan` | Linux / WSL | 设备已被系统枚举为 `can0`、`can1` 等网络接口 | ✓ | Linux 原生或 WSL 下调试 TZCAN |
| `candle` | Windows | 设备使用 Candle 协议驱动，Python 侧可用 `python-can-candle` | ✓ | Windows 上使用 TZCAN，且需要 CAN FD |
| `gs_usb` | Windows / Linux | 设备可通过 GS_USB 协议访问 | ✗ | 兼容模式，本文不展开 |

同一块 TZCAN 硬件，通常：

- Linux / WSL 选 `backend="socketcan"`
- Windows 选 `backend="candle"`
- 若确需兼容 GS_USB 设备，再选 `backend="gs_usb"`

## 2. 通用依赖

项目根目录执行：

```bash
pip install -r requirements.txt
```

当前 `requirements.txt` 已包含：

- `python-can==4.6.1`
- `numpy`
- `pyside6`
- `python-can-candle==1.2.3`

如果你只做命令行收发验证，最少需要保证 `python-can` 已安装；使用 `candle` 时建议直接安装完整 `requirements.txt`。

## 3. 通用最小自检

下面这段代码用于确认：

- Python 依赖已安装
- 目标 `backend` 能正常打开设备
- `m_dev["buses"]` 中已经拿到总线对象

把其中的 `BACKEND` 和 `CHANNEL` 按你的平台改掉即可：

```python
from tzcan import CANMessageTransmitter

BACKEND = "socketcan"   # 常见可改为 "candle"
CHANNEL = 0

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN",
    baud_rate=500000,
    channels=[CHANNEL],
    backend=BACKEND,
    fd=False,
)

try:
    print("backend =", m_dev["backend"])
    print("opened channels =", list(m_dev["buses"].keys()))
    print("bus object =", m_dev["buses"][CHANNEL])
finally:
    TX.close_can_device(m_dev)
```

判断成功的标准：

- 没有抛出异常
- 能打印出 `backend = ...`
- 能打印出 `opened channels = [0]` 一类结果

如果这一步都打不开，先不要继续发包，优先检查系统驱动、接口名和速率配置。

## 4. socketcan 快速开始

### 4.1 系统与依赖

- 适用系统：Linux / WSL
- 系统前提：设备已被内核识别成 `canX` 网络接口
- Python 依赖：`python-can`
- FD 支持：支持

`socketcan` 模式下，波特率不是在 `open()` 时由 USB 驱动下发，而是先由系统把接口配置好，再由本项目直接打开 `can0`、`can1` 等接口。

### 4.2 拉起接口

标准 CAN：

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

CAN FD：

```bash
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can0 up
```

如果你已经在 `tools/socketcan_tool.py` 的 `DEVICE_CONFIG` 中配置过设备，也可以直接：

```bash
python3 tools/socketcan_tool.py --discover
python3 tools/socketcan_tool.py --setup
```

### 4.3 快速验证

接收验证：

```bash
python3 tests/new_test_tzcan_receive.py --mode can --iface can0 --can-br 500k --duration 3
```

发送验证：

```bash
python3 tests/new_test_tzcan_send.py --mode can --iface can0 --can-br 500k --freq 10 --count 50
```

CAN FD 验证：

```bash
python3 tests/new_test_tzcan_receive.py --mode fd --iface can0 --fd-arb 500k --fd-dbr 2m --duration 3
python3 tests/new_test_tzcan_send.py    --mode fd --iface can0 --fd-arb 500k --fd-dbr 2m --fd-len 16 --freq 10 --count 50
```

### 4.4 看到什么算成功

- 接收脚本打印 `RX ...` 行，或最后输出 `总计接收: ...`
- 发送脚本打印 `完成发送 ... 帧`
- 若接口未拉起，常见现象是 `can0` 打不开，或出现 `Network is down`

## 5. candle 快速开始

### 5.1 系统与依赖

- 适用系统：Windows
- 系统前提：设备在 Windows 下使用 Candle 协议驱动
- Python 依赖：`python-can`、`python-can-candle`
- FD 支持：支持

`candle` 模式下，波特率会在初始化时直接下发，不需要像 `socketcan` 一样先手动执行 `ip link`。

### 5.2 先做枚举检查

如果你想先确认 Python 侧能看到 Candle 设备，可执行：

```bash
python -c "import can; print(can.detect_available_configs('candle'))"
```

有返回列表通常说明驱动和 Python 插件已经通了。

### 5.3 快速验证

标准 CAN 接收：

```bash
python tests/test_tzcan_receive_win.py --mode can --backend candle --index 0 --can-br 500k --duration 3
```

标准 CAN 发送：

```bash
python tests/test_tzcan_send_win.py --mode can --backend candle --index 0 --can-br 500k --freq 10 --count 50
```

CAN FD 接收 / 发送：

```bash
python tests/test_tzcan_receive_win.py --mode fd --backend candle --index 0 --fd-arb 500k --fd-dbr 2m --duration 3
python tests/test_tzcan_send_win.py    --mode fd --backend candle --index 0 --fd-arb 500k --fd-dbr 2m --fd-len 16 --freq 10 --count 50
```

### 5.4 看到什么算成功

- 接收端能打印 `RX ... BUS=CAN 2.0` 或 `RX ... BUS=CAN FD`
- 发送端输出 `完成发送 ... 帧`
- 如果 FD 模式失败，先确认当前后端确实是 `candle`，并且 `python-can-candle` 已安装

## 6. ETHCAN 快速开始

### 6.1 适用范围

- 适用系统：Linux / WSL
- 适用设备：`TZETHCAN` / HPM 系列以太网转 CAN 硬件
- 数据平面依赖：`cannelloni`
- Python 依赖：`python-can`
- FD 支持：支持

`TZETHCANTransmitter` 不是直接操作本地 USB-CAN，而是：

- Python 先通过 UDP/TCP 给硬件下发速率配置
- 再通过 `cannelloni` 把 `vcan0`、`vcan1` 等虚拟接口桥接到远端 ETHCAN 硬件

也就是说，ETHCAN 能否工作，除了 Python 依赖外，还取决于本机是否已经安装并启动了 `cannelloni` 桥接环境。

### 6.2 系统依赖安装

建议准备以下系统环境：

- `cmake`
- C++ 编译器（如 `g++`）
- `iproute2`
- Linux `vcan` 内核模块
- 可选：`can-utils`，用于 `candump` / `cangen` 辅助排查

如果系统里还没有 `cannelloni`，可以按其官方仓库说明从源码安装：

```bash
git clone https://github.com/mguentner/cannelloni.git
cd cannelloni
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

如果你不需要 SCTP，也可在编译时关闭：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DSCTP_SUPPORT=OFF
cmake --build build
sudo cmake --install build
```

官方项目地址：<https://github.com/mguentner/cannelloni>

安装完成后建议先确认：

```bash
cannelloni --help
```

### 6.3 创建桥接环境

本项目已经提供了两种方式：

方式一：直接使用旧脚本

```bash
cd tools/eth
./setup_cannelloni.sh
```

这个脚本会：

- 加载 `vcan` 模块
- 创建并拉起 `vcan0` 到 `vcan3`
- 启动 4 个 `cannelloni` 进程

方式二：使用更完整的管理工具，推荐：

```bash
python3 tools/eth/ethcan_tool.py --discover
python3 tools/eth/ethcan_tool.py --setup --remote-ip 192.168.100.10
python3 tools/eth/ethcan_tool.py --health-check --check-vcan --check-process --check-remote-ip
```

常用说明：

- `--discover`：查看当前 `vcan` / `cannelloni` 状态
- `--setup`：创建 `vcan` 并启动 `cannelloni`
- `--health-check`：检查接口、进程、远端 IP 是否可达
- `--remote-ip`：指定 ETHCAN 设备 IP

### 6.4 快速验证

先做桥接健康检查：

```bash
python3 tools/eth/ethcan_tool.py --health-check --check-vcan --check-process --check-remote-ip --remote-ip 192.168.100.10
```

CAN 2.0 接收 / 发送验证：

```bash
python3 tools/eth/ethcan_recv.py --channels 0 --mode can --duration 3 --ip 192.168.100.10
python3 tools/eth/ethcan_send.py --channels 0 --mode can --count 50 --freq 10 --ip 192.168.100.10
```

CAN FD 接收 / 发送验证：

```bash
python3 tools/eth/ethcan_recv.py --channels 0 --mode fd --dbit-baud-rate 2m --duration 3 --ip 192.168.100.10
python3 tools/eth/ethcan_send.py --channels 0 --mode fd --dbit-baud-rate 2m --fd-len 16 --count 50 --freq 10 --ip 192.168.100.10
```

### 6.5 看到什么算成功

- `ethcan_tool.py --health-check` 输出 `PASS`
- `ethcan_recv.py` 能打印接收统计或逐帧 `RX ...`
- `ethcan_send.py` 能输出发送完成统计
- Python API 打开 `TZETHCAN` 时不再报 `Ensure 'setup_cannelloni.sh' is running.`

## 7. 常见判断顺序

如果“设备打不开”或“脚本一跑就报错”，建议按下面顺序排查：

1. 先确认系统选对了 `backend`
2. 再确认驱动是否正确安装，设备是否已被系统识别
3. `socketcan` 先检查 `can0` 是否已经 `up`
4. `candle` 先检查 `python-can-candle` 是否可导入
5. `ETHCAN` 先检查 `cannelloni` 是否已安装、`vcan` 是否已创建、远端 IP 是否可达
6. 最后再跑本文档里的最小自检和发送/接收脚本

## 8. 进一步阅读

- 完整 API 与多通道说明：[usage.md](./usage.md)
- GUI 信号解析与绘图：[signal_plot.md](./signal_plot.md)
- GUI 常用报文发送配置：[send_presets.md](./send_presets.md)
- SocketCAN 设备识别与自动配置：[../tools/README_CAN_DEBUG_CN.md](../tools/README_CAN_DEBUG_CN.md)
