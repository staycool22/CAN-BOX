# CAN-BOX / TZCAN 使用说明

本项目基于 `python-can`，提供跨平台的 CAN/CAN FD 发送与接收示例，以及一个通用的 `TZCANTransmitter` 实现。包含 Linux/WSL（socketcan）与 Windows（candle/gs_usb）两套测试脚本。

## 环境准备

- python安装
  - 推荐使用python 3.10版本
  - 安装python 3.10版本：[python 3.10.10下载](https://www.python.org/downloads/windows/)（Windows下载地址）
  - 安装教程连接：https://blog.csdn.net/qq_47574956/article/details/130901233?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522cc0f198bce57e3aef0d0054b5ec09a1e%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=cc0f198bce57e3aef0d0054b5ec09a1e&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-1-130901233-null-null.142^v102^pc_search_result_base2&utm_term=%E5%AE%89%E8%A3%85python3.10&spm=1018.2226.3001.4187
  
- 安装依赖：
  - 必需：`pip install -r requirements.txt`（`python-can==4.6.1`）
  - Windows 使用 FD（candle 后端）：`pip install python-can-candle`
- Linux/WSL：
  - 需启用并配置 `socketcan` 接口（如 `can0`），示例：
    - `sudo ip link set can0 down`
    - `sudo ip link set can0 type can bitrate 500000`
    - `sudo ip link set can0 up`
- Windows：无需 `socketcan`；通过 `candle` 或 `gs_usb` 后端选择设备并配置速率。

## 文件说明

- `requirements.txt`：Python 依赖版本（`python-can==4.6.1`）。
- `can_communicator.py`：跨平台的 CAN/CAN FD 通信器，封装了 `python-can` 的后端差异，为上层应用（如 GUI）提供统一的连接、收发和监控接口。
- `main_gui.py`：基于 PySide6 的图形化上位机界面，提供设备选择、速率配置、实时报文收发、数据监控与保存等功能。
- `CANMessageTransmitter.py`：抽象基类，定义统一的发送/接收与设备管理接口，提供 `choose_can_device` 工厂方法选择具体实现。
- `TZCANTransmitter.py`：基于 `python-can` 的具体实现，支持：
  - Linux/WSL：`socketcan`，返回 `Bus` 句柄并按接口名（`canX`）打开
  - Windows：`candle`/`gs_usb`，FD 仅支持 `candle`
  - 方法：`_send_can_data`、`_receive_can_data`、`init_can_device`、`close_can_device`
- `new_test_tzcan_receive.py`：Linux/WSL 接收测试（socketcan）。支持 CAN/CAN FD，按 ESC 退出，支持采样点和 ID 过滤，并能检测错误帧。
- `new_test_tzcan_send.py`：Linux/WSL 发送测试（socketcan）。支持 loopback 与 `txqueuelen`，CAN/FD 速率批量测试、BRS 控制与频率调度。
- `test_tzcan_receive_win.py`：Windows 接收测试。支持 `candle`/`gs_usb`；按任意键退出；FD 模式仅 `candle`。
- `test_tzcan_send_win.py`：Windows 发送测试。支持 `candle`/`gs_usb`；FD 模式仅 `candle`；可选同时打开另一设备用于接收统计。

## 图形化上位机

本项目提供一个基于 PySide6 的图形化上位机界面 (`main_gui.py`)，支持 CAN/CAN FD 通信。

### Windows启动 GUI

```bash
python main_gui.py
```
### Linux启动 GUI

```bash
#如果显示组件不完全，还需安装显示组件(xcb)
# sudo apt-get update 
# sudo apt-get install libxcb-cursor0
sudo chmod +x main_gui.py
python3 main_gui.py
```

### 主要功能

- **连接设置**:
  - 自动检测操作系统并选择可用后端 (`socketcan` for Linux, `candle`/`gs_usb` for Windows)。
  - 支持自定义仲裁域/数据域波特率、采样点。
- **报文收发**:
  - 实时显示 RX/TX 报文，包含时间戳、方向、ID、类型、DLC 和数据。
  - TX/RX 报文以不同颜色区分。
  - 支持单次发送、周期发送和突发发送。
  - 支持标准帧/扩展帧、CAN/CAN FD/CAN FD+BRS 格式。
- **数据监控与操作**:
  - 暂停/继续报文显示。
  - 清空接收列表。
  - 将接收到的数据保存为 CSV 文件。
- **状态显示**:
  - 实时显示总线状态、RX/TX 帧率 (FPS) 和总线负载。


## 快速使用

以下命令均在项目根目录执行（`e:\CAN-BOX`）。

- Linux/WSL 接收（socketcan）：
  - `python new_test_tzcan_receive.py --mode can --iface can0 --can-br 500k `
  - `python new_test_tzcan_receive.py --mode fd --iface can0 --fd-arb 500k --fd-dbr 2m --duration 10`

- Linux/WSL 发送（socketcan）：
  - `python new_test_tzcan_send.py --mode can --iface can0 --can-br 500k 1m --freq 1000`
  - `python new_test_tzcan_send.py --mode fd --iface can0 --fd-arb 500k --fd-dbr 5m --fd-len 64 --freq 500`

- Windows 接收：
  - CAN2.0（candle 或 gs_usb）：`python test_tzcan_receive_win.py --mode can --backend candle --index 0 --can-br 500k  --filter-id 0x321`
  - FD（仅 candle）：`python test_tzcan_receive_win.py --mode fd --backend candle --index 0 --fd-arb 500k --fd-dbr 5m `

- Windows 发送：
  - CAN2.0：`python test_tzcan_send_win.py --mode can --backend gs_usb --index 0 --can-br 500k --freq 1000`
  - FD（仅 candle）：`python test_tzcan_send_win.py --mode fd --backend candle --index 0 --fd-arb 500k --fd-dbr 5m --fd-len 64 --freq 500`

提示：
- Linux 接收脚本按 ESC 退出；Windows 接收脚本按任意键退出。
- FD 模式的 BRS（Bit Rate Switching）由脚本参数控制，打开后数据域速率切到高速。

## TZCANTransmitter 简单示例

示例 1：Linux/WSL 在 `socketcan` 上发送与接收一帧 CAN 报文

```python
from CANMessageTransmitter import CANMessageTransmitter

# 选择并初始化 TZCAN（Linux：socketcan）
# 注意：使用 choose_can_device 动态加载后端，更稳健
TX = CANMessageTransmitter.choose_can_device("TZCAN")
m_dev, ch0, _ = TX.init_can_device(channels=[0], backend="socketcan", fd=False)

try:
    tx = TX(ch0 or m_dev["buses"][0])
    # 发送一帧 CAN 报文（标准帧）
    tx._send_can_data(send_id=0x123, data_list=[1, 2, 3, 4], is_ext_frame=False, canfd_mode=False, brs=0, esi=0)

    # 尝试接收同 ID 报文（若有回环或对端）
    ok, data, msg = tx._receive_can_data(target_id=0x123, timeout=0.5, is_ext_frame=False, canfd_mode=False, stop_on_error=True, return_msg=True)
    print("RX ok:", ok, "data:", data)
finally:
    TX.close_can_device(m_dev)
```

示例 2：Windows 使用 `candle` 发送一帧 CAN FD 报文

```python
from CANMessageTransmitter import CANMessageTransmitter

# 动态加载 TZCAN
TX = CANMessageTransmitter.choose_can_device("TZCAN")
# FD 仅支持 candle；需安装 python-can-candle
m_dev, ch0, _ = TX.init_can_device(baud_rate=500000, dbit_baud_rate=2000000, channels=[0], backend="candle", fd=True)

try:
    tx = TX(ch0 or m_dev["buses"][0])
    # 发送一帧 CAN FD 报文，打开 BRS
    payload = [i % 256 for i in range(16)]
    tx._send_can_data(send_id=0x456, data_list=payload, is_ext_frame=False, canfd_mode=True, brs=1, esi=0)
finally:
    TX.close_can_device(m_dev)
```
V2.0版本补充说明

### 核心架构重构
- **工厂模式加载**：全面重构了设备加载逻辑，统一使用 `CANMessageTransmitter.choose_can_device("TZCAN")`。这消除了对具体实现类的硬编码依赖，并支持在运行时动态选择后端。
- **稳健的导入机制**：所有测试脚本和主程序现在都包含智能导入回退机制，能够适应不同的目录结构（无论作为包安装还是直接运行脚本）。
- **配置解耦**：`TZCANTransmitter` 现在直接携带其配置类 (`Config`)，无需外部通过脆弱的反射机制去寻找 `BasicConfig`。

### 测试工具更新
- 所有测试脚本 (`test_tzcan_*.py`, `new_test_tzcan_*.py`) 均已升级适配新的架构，支持更灵活的设备选择和错误处理。

## TODO

- [x] **已完成**：增加图形化上位机界面 (`main_gui.py`)
  - 功能：设备/速率配置、实时报文监控、多种发送模式、状态显示、数据保存。
- [x] **已完成**：重构底层架构，统一设备加载接口，增强代码稳健性。
- [ ] **未完成**：优化UI更新，同步消息接收与刷新

