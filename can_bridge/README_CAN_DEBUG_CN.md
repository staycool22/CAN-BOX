# CAN Bridge 调试与设置指南

本文档说明如何管理、配置和调试 `can_bridge` 目录中的 CAN 接口。

## 1. 设备识别与设置 (`socketcan_tool.py`)

`socketcan_tool.py` 脚本用于确保物理 CAN 设备与逻辑名称（如 `can_handle_chassis`）之间的映射稳定，并处理接口配置。

### 设备识别 (发现)

该脚本基于设备的 **物理 USB 路径** 进行映射。这确保了即使操作系统在重启后分配了不同的接口名称（例如 `can0` 变为 `can1`），逻辑映射仍然保持一致。

配置定义在 `socketcan_tool.py` 中的 `DEVICE_CONFIG` 列表中：
```python
DEVICE_CONFIG = [
    {
        "name": "can_handle_chassis",
        "path_hint": "5-1",  # 物理路径的唯一部分
        "channel": 5,
        "bitrate": 500000,
        "default_iface": "can5",
    },
    # 使用特定接口后缀的示例（例如用于复合 USB 设备）
    {
        "name": "can_handle_dexhand",
        "path_hint": "1-1:1.0", # 可以使用 "1-1" 或更具体的 "1-1:1.0"
        "fd": True,
        "channel": 0,
        "bitrate": 1000000,
        "dbitrate": 5000000,
        "default_iface": "can0",
    }
    {
        "name": "example",
        "path_hint": "1-1:1.0", # 设置CANFD的采样点
        "fd": True,
        "channel": 0,
        "bitrate": 1000000,
        "dbitrate": 5000000,
        "sample_point": 0.875,
        "data_sample_point": 0.80,
        "default_iface": "can0",
    }
]
```

**发现当前设备及其路径：**
```bash
python3 socketcan_tool.py --discover
```

**验证当前逻辑到物理的映射：**
```bash
python3 -c "from socketcan_tool import identify_can_devices; import json; print(json.dumps(identify_can_devices(), indent=2))"
```

### 接口配置 (设置)

使用该脚本以正确的波特率和 FD 设置自动配置所有接口（定义在 `DEVICE_CONFIG` 中）。

**自动配置所有接口（推荐）：**
```bash
python3 socketcan_tool.py --setup
```

**关闭所有接口：**
```bash
python3 socketcan_tool.py --shutdown
```

### 外部接口函数

`socketcan_tool` 模块提供了辅助函数，供应用程序代码在运行时解析这些映射。

- **`identify_can_devices()`**: 返回一个字典，将逻辑名称（如 `can_handle_chassis`）映射到当前操作系统接口名称（如 `can0`）。
- **`_can_idx(iface_name)`**: 从接口名称中提取整数索引（例如 `'can1'` -> `1`）。

## 2. 应用程序集成指南

本节介绍在外部应用程序（如 `tz_arm_gui.py`）中实例化 CAN 设备的标准流程。

### 步骤 1: 解析设备名称

首先，使用 `socketcan_tool` 将逻辑设备名称解析为物理接口索引。

```python
from can_bridge.socketcan_tool import identify_can_devices, _can_idx

# 1. 识别所有已配置的设备
try:
    can_handles = identify_can_devices()
    # 返回: {'can_handle_chassis': 'can5', 'can_handle_arm_tree': 'can1', ...}
except Exception as e:
    print(f"无法识别 CAN 设备: {e}")
    can_handles = {}

# 2. 获取特定逻辑设备的接口索引
# _can_idx 从 'canX' 中提取数字（例如 'can1' -> 1）
can_channel_tree = _can_idx(can_handles.get('can_handle_arm_tree', 'can0'))
can_channel_hole = _can_idx(can_handles.get('can_handle_arm_hole', 'can1'))

print(f"树臂通道: {can_channel_tree}, 填土臂通道: {can_channel_hole}")
```

### 步骤 2: 实例化 CAN 接口

获取通道索引后，使用它们来实例化 CAN 接口类（例如 `can_py.py` 中的 `VESC_CAN` 或 `TZCanInterface`）。

**示例：实例化 VESC_CAN**

```python
from can_py import VESC_CAN, TZCanInterface

# 使用解析出的通道索引初始化 VESC_CAN 类。
# 该类接受可变参数作为通道。
try:
    # 基于 tz_arm_gui.py 中的示例
    # 创建一个句柄，在各自的通道上控制 '树臂' 和 '填土臂'。
    tzcan_handle = VESC_CAN(can_channel_tree, can_channel_hole)
    
    print("VESC_CAN 初始化成功。")
except Exception as e:
    print(f"VESC_CAN 初始化失败: {e}")

# 现在可以使用该句柄发送/接收命令
# tzcan_handle.send_rpm(...)
```

**`can_py.py` 类参考：**
- **`TZCanInterface`**: CAN 通信的基础封装（支持 CAN-FD）。
- **`VESC_CAN`**: 专用于 VESC 电机控制器的子类。继承自 `TZCanInterface`。

## 3. 调试工具 (`can-utils`)

如果应用程序无法通信，请使用标准 Linux 工具验证连接性。

**检查接口状态：**
```bash
ip -d link show can0
```

**监控所有 CAN 流量：**
```bash
candump any
```

**发送测试帧：**
```bash
# 标准帧 (ID 123, 数据 DEADBEEF)
cansend can0 123#DEADBEEF
```

### 常见问题
1.  **未找到设备**: 检查 USB 连接，运行 `ip a` 查看接口是否存在，或运行 `dmesg | grep -i can` 查看内核日志。
2.  **"Network is down"**: 运行 `python3 socketcan_tool.py --setup`。
3.  **导入错误**: 确保 `libcontrolcanfd.so` 位于 `libs/` 中，并且 `can_bridge` 在 `PYTHONPATH` 中。
