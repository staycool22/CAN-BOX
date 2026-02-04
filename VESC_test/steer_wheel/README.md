# VESC 四轮转向底盘控制系统

本项目是一个用于四轮转向 (4WS) / 舵轮 (Swerve Drive) 机器人底盘的综合控制框架。它使用 VESC 电机控制器通过 CAN 总线通信实现全向移动。

## 系统架构

系统遵循模块化、分层的架构设计：

### 1. 输入层 (Input Layer)
*   **Keyboard (键盘)**: 用于快速测试和调试 (W/A/S/D 控制)。
*   **Joystick (手柄)**: 支持 USB 游戏手柄 (如 Logitech, Xbox 等)。
*   **Dashboard (仪表盘)**: 基于 Web 的用户界面，用于实时监控系统状态。

### 2. 核心层 (Core Layer)
*   **`ChassisState`**: 一个线程安全的单例类，作为中央数据交换枢纽。它将输入生产者（如手柄）与控制消费者（如电机控制循环）解耦。

### 3. 执行器层 (Actuator Layer)
*   **`ChassisSystem`**: 中央调度器，运行 100Hz 的控制循环。已移动到 `actuators` 目录以统一执行逻辑。
*   **`SteerController`**: 处理运动控制逻辑 (阿克曼转向/舵轮全向) 和指令分发。
*   **`VESCMonitor`**: 管理 CAN 总线通信和 VESC 状态跟踪。
*   **`SwerveModule`**: 物理轮组模组的抽象 (包含转向电机 + 驱动电机 + 编码器)。

### 4. 逻辑层 (Logic Layer)
*   **`ChassisKinematics`**: 实现 4WS 逆运动学解算和舵轮模组优化算法 (如最短路径转向逻辑)。
*   **`PIDController`**: 用于闭环控制的软件级 PID 算法。

### 5. 硬件层 (Hardware Layer)
*   **CAN Bus**: 物理通信介质。
*   **VESC Controllers**: 电机驱动器。

## 功能特性

*   **全向控制**: 同时支持阿克曼 (Ackermann) 和全向 (Holonomic/Swerve) 转向模式。
*   **实时监控**: 集成 Web 仪表盘 (HTML5/JS)，可可视化显示速度、角度和电机状态。
*   **跨平台**: 兼容 Windows 和 Linux (包括树莓派/Jetson 等嵌入式平台)。
*   **安全机制**: 内置心跳/看门狗机制，在信号丢失时自动停止电机。
*   **仿真模式**: 支持在无硬件连接的情况下运行，用于验证控制逻辑。

## 环境要求

*   Python 3.8+
*   CAN 适配器 (如 USB-CAN, SocketCAN 设备)
*   VESC 控制器 (需配置为 CAN 通信)

## 安装步骤

1.  克隆仓库。
2.  安装依赖库：
    ```bash
    pip install pyserial python-can bottle inputs
    ```

## 使用方法

### 1. 运行控制系统

运行主入口脚本：

```bash
# 真实硬件模式 (默认)
python test/test_steer_control_0203.py --mode real --input keyboard

# 仿真模式 (无硬件)
python test/test_steer_control_0203.py --mode sim
```

### 2. 控制说明 (键盘)

*   **W / S**: 前进 / 后退
*   **A / D**: 左横移 / 右横移 (舵轮模式) 或 左转 / 右转 (阿克曼模式)
*   **Q / E**: 原地旋转 (左旋/右旋)
*   **SPACE (空格)**: 紧急停止
*   **U / I**: 减小 / 增加 目标速度
*   **O / P**: 减小 / 增加 最大旋转角速度
*   **M**: 切换转向模式 (阿克曼 / 4WS全向)
*   **Z**: 退出程序

### 3. 仪表盘

使用以下标志启动仪表盘：
```bash
python test/test_steer_control_0203.py --dashboard-enable
```
访问地址: `http://localhost:8080`

## 文件结构

*   `test/test_steer_control_0203.py`: 主程序入口 (测试脚本)。
*   `actuators/`: 包含系统执行和硬件抽象文件。
    *   `chassis_system.py`: 中央控制系统。
    *   `Steering_wheel_chassis_0203.py`: VESC 监控器和转向控制器。
    *   `swerve_module.py`: 舵轮模组抽象类。
*   `core/`: 核心数据结构。
    *   `chassis_state.py`: 全局状态管理。
*   `algorithm/`: 运动算法。
    *   `chassis_kinematics.py`: 运动学解算库。
*   `ui/`: 用户界面。
    *   `dashboard_client.py` & `dashboard_ui.html`: Web 接口。
*   `config/`: 配置文件。
