# DM_MOTOR 使用说明

本文档基于项目内的 `tzcan/protocols/dm_motor_protocol.py`，说明如何通过 `TZUSB2CANTransmitter + CANProtocolBase` 使用达妙电机 CAN 协议。

对应实现文件：

- `tzcan/protocols/dm_motor_protocol.py`
- `tzcan/protocols/base.py`
- `tzcan/devices/tzusb2can.py`

## 1. 与官方 `DM_CAN.py` 的对应关系

当前 `DM_MOTOR` 已完整移植 `DM_CAN.py` 里的协议能力，包含：

- 电机对象：`Motor`
- 控制类：`DM_MOTOR`
- 全部控制模式：
  - `controlMIT`
  - `control_delay`
  - `control_Pos_Vel`
  - `control_Vel`
  - `control_pos_force`
  - `control_Pos_Vel_CSP`
  - `control_Vel_CSP`
  - `control_Tor_CSP`
- 控制命令：
  - `enable`
  - `enable_old`
  - `disable`
  - `set_zero_position`
- 状态与参数相关接口：
  - `addMotor`
  - `refresh_motor_status`
  - `switchControlMode`
  - `save_motor_param`
  - `read_motor_param`
  - `change_motor_param`
- 枚举与辅助类型：
  - `DM_Motor_Type`
  - `DM_variable`
  - `Control_Type`

和官方 `USAGE.md` 相比，主要差异只有一处：

- 官方原版通过串口创建 `MotorControl(serial_device)`
- 当前项目改为通过 `TZUSB2CANTransmitter` 创建 `DM_MOTOR(transmitter)`

也就是说，协议内容、模式、数据拼包、参数读写逻辑都来自 `DM_CAN.py`，只是底层收发路径改成了本项目的 CAN 发收接口。

## 2. 前置条件

### 2.1 安装依赖

项目至少需要：

```bash
pip install numpy python-can
```

如果你还没有把项目加入 Python 路径，可在项目根目录运行：

```bash
pip install -e .
```

### 2.2 Linux 下配置 SocketCAN

如果你在 Linux / WSL 上使用 `socketcan` 后端，先把物理 CAN 口拉起，例如：

```bash
sudo ip link set can0 up type can bitrate 500000
```

Windows 下通常使用 `backend="candle"`，不需要这一步。

## 3. 导入方式

```python
from tzcan import CANMessageTransmitter
from tzcan.protocols.dm_motor_protocol import (
    DM_MOTOR,
    Motor,
    DM_Motor_Type,
    DM_variable,
    Control_Type,
)
```

## 4. 创建 CAN 控制对象

官方文档里是这样创建：

```python
MotorControl1 = MotorControl(serial_device)
```

在当前项目中，需要改成先打开 CAN 设备，再创建 `DM_MOTOR`：

```python
from tzcan import CANMessageTransmitter
from tzcan.protocols.dm_motor_protocol import DM_MOTOR

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN",
    baud_rate=500000,
    channels=[0],
)

dm = DM_MOTOR(TX(m_dev["buses"][0]))
```

使用完成后关闭设备：

```python
TX.close_can_device(m_dev)
```

完整写法建议如下：

```python
from tzcan import CANMessageTransmitter
from tzcan.protocols.dm_motor_protocol import DM_MOTOR

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN",
    baud_rate=500000,
    channels=[0],
)

try:
    dm = DM_MOTOR(TX(m_dev["buses"][0]))
    # 在这里进行电机控制
finally:
    TX.close_can_device(m_dev)
```

## 5. 创建电机对象

和官方文档一致：

```python
motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
motor2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
motor3 = Motor(DM_Motor_Type.DM4310, 0x03, 0x13)
```

参数含义：

- 第一个参数：电机类型
- 第二个参数：`SlaveID`，即电机 CAN ID
- 第三个参数：`MasterID`，即主机反馈 ID

注意事项：

- `MasterID` 不要设为 `0x00`
- `MasterID` 和 `SlaveID` 需要提前在达妙上位机中设置好
- 多个电机的 `MasterID` 建议互不相同

## 6. 添加电机

在控制前先把电机对象加入 `DM_MOTOR`：

```python
dm.addMotor(motor1)
dm.addMotor(motor2)
dm.addMotor(motor3)
```

## 7. 使能、失能、设置零点

### 7.1 使能

```python
dm.enable(motor1)
dm.enable(motor2)
```

旧固件兼容方式：

```python
dm.enable_old(motor1, Control_Type.MIT)
dm.enable_old(motor2, Control_Type.POS_VEL)
dm.enable_old(motor3, Control_Type.VEL)
```

### 7.2 设置零点

先让电机处于失能状态，并摆到目标零点位置，再执行：

```python
dm.set_zero_position(motor1)
```

### 7.3 失能

```python
dm.disable(motor1)
```

## 8. 电机控制模式

达妙电机是一发一收模式。发送控制帧后，电机会返回当前状态，并更新 `Motor` 对象内部缓存。

建议：

- 控制循环里每帧之间保留 `1ms` 到 `2ms` 间隔
- 参数修改和保存操作尽量在失能状态下进行

### 8.1 MIT 模式

```python
dm.controlMIT(motor1, 50, 0.3, 0, 0, 0)
```

参数顺序与官方版一致：

- `kp`
- `kd`
- `q`
- `dq`
- `tau`

带延迟版本：

```python
dm.control_delay(motor1, 50, 0.3, 0, 0, 0, 0.002)
```

### 8.2 位置速度模式

```python
dm.control_Pos_Vel(motor1, 1.0, 2.0)
```

### 8.3 速度模式

```python
dm.control_Vel(motor1, 5.0)
```

### 8.4 力位混合模式

```python
dm.control_pos_force(motor1, 10.0, 1000, 100)
```

参数说明与官方文档一致：

- 第二个参数：目标位置，单位 `rad`
- 第三个参数：速度，原协议按整型数值发送
- 第四个参数：电流标幺值，原协议按整型数值发送

### 8.5 CSP 模式

`dm_motor_protocol.py` 中还保留了 `DM_CAN.py` 里的 CSP 相关模式：

位置速度 CSP：

```python
dm.control_Pos_Vel_CSP(motor1, 1.0, 2.0)
```

速度 CSP：

```python
dm.control_Vel_CSP(motor1, 3.0)
```

力矩 CSP：

```python
dm.control_Tor_CSP(motor1, 1.5)
```

## 9. 状态读取

### 9.1 控制后自动刷新

发送控制帧后，`Motor` 对象中的状态会被更新，可以直接读取：

```python
pos = motor1.getPosition()
vel = motor1.getVelocity()
tau = motor1.getTorque()
err = motor1.getError()
```

### 9.2 主动刷新状态

如果当前没有发送控制帧，但希望主动获取一次状态：

```python
dm.refresh_motor_status(motor1)
print(
    "POS:", motor1.getPosition(),
    "VEL:", motor1.getVelocity(),
    "TOR:", motor1.getTorque(),
    "ERR:", motor1.getError(),
)
```

注意：

- 达妙电机不是后台持续推送状态
- 只有发送控制帧或调用 `refresh_motor_status()` 后，`Motor` 内部缓存才会刷新

## 10. 模式切换与参数操作

根据官方文档，新固件支持通过 CAN 修改控制模式和参数。建议在失能状态下执行。

### 10.1 切换控制模式

```python
if dm.switchControlMode(motor1, Control_Type.POS_VEL):
    print("switch POS_VEL success")

if dm.switchControlMode(motor2, Control_Type.VEL):
    print("switch VEL success")
```

说明：

- 当前实现保留了 `DM_CAN.py` 原始逻辑
- 切换后是否掉电保存，取决于是否再调用 `save_motor_param()`

### 10.2 保存参数到 Flash

```python
dm.save_motor_param(motor1)
```

该操作会保存当前电机参数到 Flash。原实现内部会先执行失能流程。

### 10.3 读取内部寄存器参数

```python
print("PMAX:", dm.read_motor_param(motor1, DM_variable.PMAX))
print("MST_ID:", dm.read_motor_param(motor1, DM_variable.MST_ID))
print("VMAX:", dm.read_motor_param(motor1, DM_variable.VMAX))
print("TMAX:", dm.read_motor_param(motor1, DM_variable.TMAX))
```

读取后，也可以从 `Motor` 对象缓存中再次取出：

```python
print("PMAX cache:", motor1.getParam(DM_variable.PMAX))
```

### 10.4 改写内部寄存器参数

```python
if dm.change_motor_param(motor1, DM_variable.KP_APR, 54):
    print("write success")
```

注意：

- 并不是所有寄存器都允许写入
- 该操作本身不等于掉电保存
- 如果要持久保存，仍需再调用 `save_motor_param()`

## 11. 完整示例

```python
from time import sleep

from tzcan import CANMessageTransmitter
from tzcan.protocols.dm_motor_protocol import (
    DM_MOTOR,
    Motor,
    DM_Motor_Type,
    Control_Type,
)

TX, m_dev, _, _ = CANMessageTransmitter.open(
    "TZUSB2CAN",
    baud_rate=500000,
    channels=[0],
)

try:
    dm = DM_MOTOR(TX(m_dev["buses"][0]))

    motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    dm.addMotor(motor1)

    dm.enable(motor1)
    sleep(0.1)

    for _ in range(100):
        dm.controlMIT(motor1, 50, 0.3, 0.0, 0.0, 0.0)
        print(
            "POS:", motor1.getPosition(),
            "VEL:", motor1.getVelocity(),
            "TOR:", motor1.getTorque(),
            "ERR:", motor1.getError(),
        )
        sleep(0.002)

    dm.disable(motor1)
finally:
    TX.close_can_device(m_dev)
```

## 12. 补充说明

- 当前实现走的是标准 CAN 发送与接收，使用标准帧，非 CAN-FD
- `DM_MOTOR` 的发送底层来自 `CANProtocolBase.send()`
- `DM_MOTOR` 的接收底层来自 `CANProtocolBase.receive()`
- 如果你要同时控制多条 CAN 总线，按总线分别创建多个 `DM_MOTOR` 实例

更多底层打开方式和多通道说明，可参考 `docs/usage.md`。
