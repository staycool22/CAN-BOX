# 4-Wheel Steering Chassis Control Module

此目录包含用于控制四轮转向（4WS）底盘的核心代码，支持基于 CAN 总线的电机控制和运动学解算。

## 目录结构

- **`Motor_ctl.py`**
  - 实现了基于 CAN 总线的电机底层控制。
  - 支持 CiA402 协议（控制字/状态字管理）。
  - 提供了 SDO 和 PDO 通信功能，用于配置和实时控制转向电机。

- **`Steering_wheel_chassis.py`**
  - 底盘控制的主要入口。
  - 定义了 `BasicConfig` 类，包含电机 ID、CAN 通道配置和零位偏置。
  - 提供了 `SteerController` 类，负责协调驱动电机和转向电机的动作。

- **`chassis_kinematics.py`**
  - 实现了四轮转向底盘的运动学模型。
  - `ChassisGeometry`: 定义底盘几何尺寸（轴距、轮距）。
  - `FourWheelSteeringKinematics`: 提供逆运动学（目标速度 -> 轮速/舵角）和正运动学（轮速/舵角 -> 里程计）解算。

- **`test_steer_control.py`**
  - 交互式测试脚本（CLI 工具）。
  - 允许通过键盘控制底盘运动（前进、后退、横移、自旋等）。
  - 用于验证电机转向和驱动逻辑是否正常。

- **`setup_can.sh`**
  - Linux 下的 CAN 接口初始化脚本。
  - 配置 `can0` 为 CAN 2.0 (500k)。
  - 配置 `can1` 为 CAN FD (1M/4M)。

## 硬件配置

### CAN 总线分配
*   **CAN0 (Drive Motors):** 负责轮毂电机（驱动），波特率 500k (CAN 2.0)。
*   **CAN1 (Steer Motors):** 负责转向电机，波特率 1M/4M (CAN FD)。

### 电机 ID 映射 (默认配置)
| 位置 | 转向电机 ID (Steer) | 驱动电机 ID (Drive) |
| :--- | :---: | :---: |
| 左前 (FL) | 46 | 103 |
| 右前 (FR) | 47 | 104 |
| 左后 (RL) | 105 | 107 |
| 右后 (RR) | 106 | 108 |

> **注意**: 请在 `Steering_wheel_chassis.py` 中的 `BasicConfig` 类里根据实际硬件调整电机 ID 和零位偏置 (`OFFSET`)。

## 依赖说明

此模块依赖于父目录中的通用 CAN 通信库：
*   `CANMessageTransmitter`: 底层 CAN 收发接口。
*   `can_vesc`: VESC 电机驱动相关协议实现。

确保项目根目录已加入 `PYTHONPATH`，或保持目录结构完整以便脚本能正确导入模块。

## 快速开始

1.  **配置 CAN 接口 (Linux)**
    ```bash
    sudo ./setup_can.sh
    ```

2.  **运行测试脚本**
    ```bash
    python test_steer_control.py
    ```

3.  **键盘控制说明**
    *   `w` / `s`: 前进 / 后退
    *   `a` / `d`: 左横移 / 右横移 (90° 蟹行)
    *   `q` / `e`: 原地左旋 / 右旋
    *   `o` / `p`: 斜向运动
    *   `Space`: 停止
    *   `z`: 退出程序
