# VESC CAN 集成测试脚本：`tests/test_tzcan_vesc.py`

面向 **VESC 电调** 的端到端示例：用 `CANMessageTransmitter`（`TZUSB2CAN`）打开物理通道，再挂载协议类 `VESC_CAN`，演示接收状态帧与下发 RPM/电流/位置等命令。脚本既可作手工联调工具，也可作为应用层调用模式的参考。

## 运行位置

在 **CAN-BOX 仓库根目录**（含 `tzcan/` 与 `tests/`）下执行：

```bash
cd drivers/can-box   # 若仓库内路径如此
python3 tests/test_tzcan_vesc.py --help
```

若已安装 `tzcan` 包，也可在其它目录运行（需保证 Python 能找到 `tzcan`）。

## 依赖与前置条件

- Python：`python-can`（见仓库 `requirements.txt`）。
- **socketcan（Linux）**：接口速率需与 `--can-br`（及可选的 FD 配置）一致；比特率通常由 `ip link` 预先配置，应用侧 `baud_rate` 应与内核一致。
- **CAN FD**：仅当使用 `--fd` 时启用；`gs_usb` 后端不支持 FD，脚本会报错退出。SocketCAN 示例：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can0 up
```

仲裁段速率对应 `--can-br`，数据段速率对应 `--fd-dbr`，须与 `ip link` 中 `bitrate` / `dbitrate` 一致。

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--iface` | `0` | 逻辑通道号；socketcan 下一般对应 `can0`→`0`、`can2`→`2`。 |
| `--can-br` | `500k` | 仲裁段波特率（经典 CAN 或 CAN FD 仲裁段）。支持 `500k`、`1m`、纯数字（bit/s）。 |
| `--fd` | 关闭 | 启用 CAN FD；会设置 `dbit_baud_rate`（见 `--fd-dbr`）。 |
| `--fd-dbr` | `2m` | CAN FD 数据段波特率；仅在指定 `--fd` 时生效。 |
| `--backend` | `socketcan` | `socketcan` / `candle` / `gs_usb`。 |
| `--vesc-id` | `1` | VESC 在 CAN 上的设备 ID（编码进仲裁 ID）。 |
| `--mode` | `receive` | 运行模式，见下表。 |
| `--rpm` | `1000` | `rpm` / `pass_through` 模式下的目标转速。 |
| `--current` | `0` | `current` / `pass_through` 模式下的目标电流 (A)。 |
| `--pos` | `0` | `pos` / `pass_through` 模式下的目标位置 (deg)。 |
| `--duration` | `5` | 运行秒数；`0` 表示持续到 Ctrl+C。 |
| `--freq` | `10` | 周期性发送模式下的发送频率 (Hz)。 |

### `--mode` 说明

| `mode` | 行为 |
|--------|------|
| `receive` | 仅解码并打印各 VESC 状态（RPM、电流、位置、输入电压等），不发命令。 |
| `rpm` | 按 `--freq` 向 `--vesc-id` 发送转速指令，并尝试短时接收反馈。 |
| `current` | 按 `--freq` 发送电流指令。 |
| `pos` | 按 `--freq` 发送位置指令。 |
| `pass_through` | 发送组合透传指令（位置 + 转速 + 电流），仅当收到的包 ID 匹配 `--vesc-id` 时打印一行。 |

## 常用示例

### 经典 CAN：监听状态

```bash
python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 --mode receive
```

### 经典 CAN：发 RPM

```bash
python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --vesc-id 1 \
  --mode rpm --rpm 2000 --duration 10 --freq 20
```

### 指定后端（例如第二路 CAN）

```bash
python3 tests/test_tzcan_vesc.py --iface 2 --can-br 500k --vesc-id 1 \
  --backend socketcan --mode receive
```

### CAN FD 总线

```bash
python3 tests/test_tzcan_vesc.py --iface 0 --can-br 500k --fd --fd-dbr 2m \
  --vesc-id 1 --mode receive
```

> 说明：VESC 常见报文仍为短数据经典帧形态；打开 FD 总线主要用于与已配置为 FD 的物理层一致。若需显式以 CAN FD 帧格式收发，需在协议层 `send`/`receive` 传入 `is_fd` 等参数（当前 `VESC_CAN` 默认未改）。

## 多电机与同脚本多实例

- **同一总线、多台 VESC**：共用一个 `VESC_CAN` 实例即可，在代码里对不同 `vesc_id` 调用 `send_*`；命令行脚本一次只针对一个 `--vesc-id`，多机可开多个终端各指定不同 `--vesc-id`。
- **不同总线**：应对每个物理通道分别 `CANMessageTransmitter.open(..., channels=[ch])` 并各建一个 `VESC_CAN(TX(bus))`；脚本内注释给出了 `channels=[0,1]` 时的示例结构。

## 与上层代码的对应关系

脚本中的推荐模式与 [用法指南 — 协议层 VESC](./usage.md#协议层--vesc) 一致：

1. `CANMessageTransmitter.open("TZUSB2CAN", ...)`  
2. `VESC_CAN(TX(m_dev["buses"][iface]))`  
3. 调用 `send_rpm` / `send_current` / `send_pos` / `receive_decode` 等。

## 相关文件

| 路径 | 说明 |
|------|------|
| `tests/test_tzcan_vesc.py` | 本测试脚本源码 |
| `tzcan/protocols/vesc.py` | `VESC_CAN` 协议实现 |
| `docs/usage.md` | 总用法、backend、多通道与 VESC 概述 |
