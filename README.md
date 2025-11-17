# CAN-BOX / TZCAN 使用说明

本项目基于 `python-can`，提供跨平台的 CAN/CAN FD 发送与接收示例，以及一个通用的 `TZCANTransmitter` 实现。包含 Linux/WSL（socketcan）与 Windows（candle/gs_usb）两套测试脚本。

## 环境准备

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
- `CANMessageTransmitter.py`：抽象基类，定义统一的发送/接收与设备管理接口，提供 `choose_can_device` 工厂方法选择具体实现。
- `TZCANTransmitter.py`：基于 `python-can` 的具体实现，支持：
  - Linux/WSL：`socketcan`，返回 `Bus` 句柄并按接口名（`canX`）打开
  - Windows：`candle`/`gs_usb`，FD 仅支持 `candle`
  - 方法：`_send_can_data`、`_receive_can_data`、`init_can_device`、`close_can_device`
- `new_test_tzcan_receive.py`：Linux/WSL 接收测试（socketcan）。支持 CAN/CAN FD，按 ESC 退出，支持采样点和 ID 过滤，并能检测错误帧。
- `new_test_tzcan_send.py`：Linux/WSL 发送测试（socketcan）。支持 loopback 与 `txqueuelen`，CAN/FD 速率批量测试、BRS 控制与频率调度。
- `test_tzcan_receive_win.py`：Windows 接收测试。支持 `candle`/`gs_usb`；按任意键退出；FD 模式仅 `candle`。
- `test_tzcan_send_win.py`：Windows 发送测试。支持 `candle`/`gs_usb`；FD 模式仅 `candle`；可选同时打开另一设备用于接收统计。

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

## TODO

- [ ] 发送用户自定义消息：
  - 增加命令行参数（如 `--id`、`--data`、`--ext`、`--fd`、`--brs`、`--esi`），直接从终端构造并发送指定报文
- [ ] 增加 Windows 可视化上位机界面：
  - 提供设备选择、速率配置、实时 RX/TX 列表与过滤、统计图表