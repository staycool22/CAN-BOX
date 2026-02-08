import sys
import os
import time
import threading
import math
from typing import List, Dict, Optional, Tuple

# 添加父目录到 path 以查找 CANMessageTransmitter
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

if project_root not in sys.path:
    sys.path.append(project_root)

# Import from package structure
from algorithm.chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics, AckermannSteeringKinematics
from config.steer_wheel_config import config
import driver.steer_wheel_can_bus as steer_wheel_can_bus
from driver.steer_wheel_can_bus import VESC_CAN_STATUS
from actuators.swerve_module import SwerveModule


class VESCMonitor:
    def __init__(self, bus_drive=None, bus_steer=None):
        
        self.bus_drive = bus_drive
        self.bus_steer = bus_steer
        self.m_dev = None
        self.vesc = None
        self.vesc_drive = None
        
        # 如果外部传入了 bus 对象，则跳过内部初始化
        if self.bus_drive and self.bus_steer:
            print("VESCMonitor 使用外部传入的 CAN 总线。")
            self.m_dev = None
        else:
            # 使用新模块初始化 CAN 设备
            self.m_dev, self.bus_drive, self.bus_steer = steer_wheel_can_bus.init_can_hardware()

        # 创建 VESC 接口
        self.vesc, self.vesc_drive = steer_wheel_can_bus.create_vesc_interfaces(self.bus_drive, self.bus_steer)
        
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
        # --- 初始化 Swerve Modules ---
        self.modules: Dict[str, SwerveModule] = {}
        self.id_to_module_map = {} # CAN ID -> (Module, is_steer)
        
        # 定义模块配置 (Name, SteerID, DriveID)
        module_configs = [
            ("FL", config.FL_STEER_ID, config.FL_DRIVE_ID),
            ("FR", config.FR_STEER_ID, config.FR_DRIVE_ID),
            ("RL", config.RL_STEER_ID, config.RL_DRIVE_ID),
            ("RR", config.RR_STEER_ID, config.RR_DRIVE_ID)
        ]
        
        for name, sid, did in module_configs:
            # 创建模块
            mod = SwerveModule(name, sid, did, config)
            self.modules[name] = mod
            
            # 建立映射
            self.id_to_module_map[sid] = (mod, True)   # True = Steer
            self.id_to_module_map[did] = (mod, False)  # False = Drive
            
        
        if config.USE_CURRENT_AS_ZERO:
            print("配置为: 使用当前位置作为零点 (忽略预设参数)")

    def perform_zero_calibration(self):
        """
        执行零位校准。
        """
        print("执行自动回零操作...")
        
        for name, mod in self.modules.items():
            mod.calibration_mode = True
            mod.target_angle = 0.0
            mod.target_speed = 0.0 # Force speed to 0 to prevent exiting calibration mode
            print(f"模块 {name} (ID {mod.steer_id}) 目标已设为 0.0 度 (自动回零)")
            print(f"  -> 当前状态: Angle={mod.total_angle:.2f}, Turns={mod.turns}, Enc={mod.last_pos}")

    def get_vesc_interface(self, motor_id: int):
        """
        根据电机 ID 获取对应的 VESC 接口实例
        """
        # 1. 优先使用 config 的实时配置（允许运行时切换）
        if config.ENABLE_WHEEL_GROUP_CAN_MODE:
            # 轮组分组模式: FL/RL(左侧) -> can0, FR/RR(右侧) -> can1
            for channel, ids in config.WHEEL_GROUP_CAN_MAPPING.items():
                if motor_id in ids:
                    if channel == 0: return self.vesc # can0
                    if channel == 1: return self.vesc_drive # can1
            
            # 如果 ID 不在映射中，回退到默认
            print(f"⚠️ 警告: Motor ID {motor_id} 未在轮组映射中找到，默认使用 can0")
            return self.vesc
        else:
            # 2. 默认功能分组模式
            # 转向电机 -> can0 (self.vesc)
            # 驱动电机 -> can1 (self.vesc_drive)
            if motor_id in config.get_steer_ids():
                return self.vesc
            else:
                return self.vesc_drive

    def _process_vesc_packet(self, msg_id, packet):
        """
        处理 VESC 状态包，更新电机状态
        """
        vesc_id = msg_id & 0xFF
        status_id = (msg_id >> 8) & 0xFF
        
        if vesc_id in self.id_to_module_map:
            module, is_steer = self.id_to_module_map[vesc_id]
            
            with self.lock:
                # Status 1: RPM/Current/Duty
                if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
                    if not is_steer:
                        # 驱动电机更新速度
                        module.update_feedback(packet, is_steer=False)
                    else:
                        # 转向电机也可更新 RPM (可选)
                        pass
                        
                # Status 2: Enc1/Enc2 (Custom)
                elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_2:
                    if is_steer:
                        module.update_feedback(packet, is_steer=True)

    def _monitor_loop(self):
        while self.running:
            if self.vesc:
                while self.running:
                    msg_id, packet = self.vesc.receive_decode(timeout=0)
                    if msg_id is None:
                        break
                    self._process_vesc_packet(msg_id, packet)
            
            if self.vesc_drive:
                while self.running:
                    msg_id, packet = self.vesc_drive.receive_decode(timeout=0)
                    if msg_id is None:
                        break
                    self._process_vesc_packet(msg_id, packet)
            
            time.sleep(0.0001)
            
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.thread.start()
            
            # 启动自动回零任务 (使用独立线程等待数据就绪)
            threading.Thread(target=self._auto_zero_task, daemon=True).start()
            
            print("底盘监控已启动")

    def _auto_zero_task(self):
        """
        自动回零任务：等待电机数据就绪后执行回零
        """
        print("⏳ 等待电机数据就绪以执行自动回零...")
        timeout = 5.0 # 缩短等待时间
        start_time = time.time()
        
        while self.running:
            # 检查是否所有转向电机都有数据
            all_ready = True
            any_ready = False
            for mod in self.modules.values():
                if mod.last_pos is not None:
                    any_ready = True
                else:
                    all_ready = False
            
            if all_ready:
                print("✅ 所有转向电机就绪，执行自动回零...")
                time.sleep(0.5)
                self.perform_zero_calibration()
                return

            if time.time() - start_time > timeout:
                if any_ready:
                    print(f"⚠️ 自动回零等待超时 (部分电机未就绪)，仅对在线电机执行回零...")
                    self.perform_zero_calibration()
                else:
                    print("❌ 自动回零失败：未检测到任何转向电机")
                return
                
            time.sleep(0.5)

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            
        if self.m_dev:
            # 只有当 m_dev 由本类创建时才执行关闭
            steer_wheel_can_bus.close_can_device(self.m_dev)
            self.m_dev = None
        else:
            print("VESCMonitor (外部总线) 已断开连接，但不关闭物理设备。")
            
        print("VESCMonitor 已停止")
        
    def get_state(self, motor_id):
        with self.lock:
            if motor_id in self.id_to_module_map:
                mod, is_steer = self.id_to_module_map[motor_id]
                if is_steer:
                    disp_angle = mod.total_angle % 360.0
                    if disp_angle > 180:
                        disp_angle -= 360.0
                    
                    return {
                        "total_angle": mod.total_angle,
                        "turns": mod.turns,
                        "last_pos": mod.last_pos,
                        "enc2": mod.enc2,
                        "display_angle": disp_angle,
                        "pid_pos": mod.last_pos, # Compatibility
                        "rpm": mod.current_rpm   # Added for compatibility
                    }
                else:
                    return {
                        "rpm": mod.current_rpm,
                        "speed": mod.current_speed,
                        "last_pos": 0
                    }
            return {}

class VESCControlLoop:
    def __init__(self, monitor: VESCMonitor):
        self.monitor = monitor
        self.running = False
        self.thread = None
        self.enable_control = True

    def _control_loop(self):
        last_control_time = time.time()
        CONTROL_INTERVAL = 0.01
        
        while self.running:
            now = time.time()
            if now - last_control_time >= CONTROL_INTERVAL:
                if self.enable_control:
                    for mod in self.monitor.modules.values():
                        s_rpm, d_rpm = mod.calculate_control()
                        
                        v_steer = self.monitor.get_vesc_interface(mod.steer_id)
                        if v_steer and mod.last_pos is not None:
                            v_steer.send_rpm(mod.steer_id, s_rpm)
                            
                        if config.ENABLE_DRIVE:
                            v_drive = self.monitor.get_vesc_interface(mod.drive_id)
                            if v_drive and mod.is_online:
                                v_drive.send_rpm(mod.drive_id, d_rpm)
                
                last_control_time = now
            
            time.sleep(0.0001)

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._control_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

class SteerController:
    def __init__(self, monitor: VESCMonitor):
        self.monitor = monitor
        self.vesc = monitor.vesc
        self.vesc_drive = monitor.vesc_drive # 获取驱动电机控制器 (VESC)
        
        # 初始化 kinematics
        self.geometry = ChassisGeometry(length=config.CHASSIS_LENGTH, width=config.CHASSIS_WIDTH, wheel_radius=config.DRIVE_WHEEL_RADIUS)
        
        # 根据配置选择运动学模型
        if config.ENABLE_ACKERMANN_MODE:
            print(f"🔧 运动模式: 阿克曼转向 (4WS={config.ACKERMANN_4WS})")
            self.kinematics = AckermannSteeringKinematics(self.geometry, is_4ws=config.ACKERMANN_4WS)
        else:
            print("🔧 运动模式: 全向移动 (Holonomic)")
            self.kinematics = FourWheelSteeringKinematics(self.geometry)
        
        # 初始化驱动电机（如果存在）
        if self.vesc_drive:
            # VESC 不需要复杂的初始化序列，只需确保连接即可
            print("驱动电机控制器 (VESC) 已连接")
            pass 
            
    def switch_kinematics_mode(self, use_ackermann: bool):
        """
        运行时切换运动学模式
        """
        if use_ackermann:
            print(f"🔄 切换至: 阿克曼转向模式 (4WS={config.ACKERMANN_4WS})")
            self.kinematics = AckermannSteeringKinematics(self.geometry, is_4ws=config.ACKERMANN_4WS)
        else:
            print("🔄 切换至: 全向移动模式 (Holonomic)")
            self.kinematics = FourWheelSteeringKinematics(self.geometry)
            
    def _send_steer_pos(self, motor_id: int, target_angle: float):
        """
        发送转向角度指令 (更新软件闭环控制的目标)。
        :param motor_id: 转向电机 ID
        :param target_angle: 逻辑目标角度（0为正前方，单位：度）
        """
        if not self.vesc:
            return # 转向控制器未启用
            
        if motor_id in self.monitor.id_to_module_map:
            mod, is_steer = self.monitor.id_to_module_map[motor_id]
            if is_steer:
                mod.target_angle = target_angle


    def apply_kinematics(self, wheel_states: Dict[str, Tuple[float, float]]):
        """
        应用运动学计算结果到电机。
        :param wheel_states: {wheel_name: (speed_mps, angle_rad)}
        """
        # 遍历计算结果并分发给模块
        for name, (target_speed, target_angle_val) in wheel_states.items():
            if name in self.monitor.modules:
                mod = self.monitor.modules[name]
                
                # 更新模块目标 (SwerveModule.calculate_control 会处理优化和互锁)
                mod.target_speed = target_speed
                mod.target_angle = target_angle_val # 假设输入已为度数
                
                # Debug print for FL module
                if name == "FL" and abs(target_speed) > 0.1:
                    # 限制打印频率
                    now = time.time()
                    if not hasattr(self, '_last_kinematics_print'): self._last_kinematics_print = 0
                    if now - self._last_kinematics_print > 0.5:
                        print(f"[DEBUG FL] Speed={target_speed:.2f}, Angle={target_angle_val:.2f}")
                        self._last_kinematics_print = now

    def chassis_move(self, angle_deg: float, speed_mps: float, omega_rad: float = 0.0):
        """
        全能移动函数: 通过逆运动学解算，同时支持平移、旋转及复合运动。
        
        :param angle_deg: 移动方向角度 (度), 0为正前, +90为左 (仅影响平移方向)
        :param speed_mps: 移动线速度 (m/s)
        :param omega_rad: 自旋角速度 (rad/s), 正值为左旋(逆时针), 负值为右旋
        """
        # 1. 将极坐标 (角度, 速度) 转换为 直角坐标 (Vx, Vy)
        # 注意: math.cos/sin 接收弧度
        # 坐标系定义: X轴朝前(0度), Y轴朝左(90度)
        move_rad = math.radians(angle_deg)
        vx = speed_mps * math.cos(move_rad)
        vy = speed_mps * math.sin(move_rad)
        
        # 2. 调用逆运动学解算全车轮子状态
        wheel_states = self.kinematics.inverse_kinematics(vx, vy, omega_rad, optimize_angle=True)
        
        # 3. 应用到电机
        self.apply_kinematics(wheel_states)

    def move_straight(self, speed_mps: float):
        """直行"""
        self.chassis_move(0.0, speed_mps, 0.0)

    def move_diagonal(self, angle_deg: float, speed_mps: float):
        """斜行 (平移)"""
        self.chassis_move(angle_deg, speed_mps, 0.0)
        
    def spin_clockwise(self, speed_mps: float):
        """顺时针原地旋转"""
        # 计算角速度 Omega = V / R
        radius = math.hypot(self.geometry.L/2, self.geometry.W/2)
        omega = 0.0
        if radius > 1e-4:
            omega = -abs(speed_mps) / radius # 负值为顺时针
            
        self.chassis_move(0.0, 0.0, omega)

    def spin_counter_clockwise(self, speed_mps: float):
        """逆时针原地旋转"""
        radius = math.hypot(self.geometry.L/2, self.geometry.W/2)
        omega = 0.0
        if radius > 1e-4:
            omega = abs(speed_mps) / radius # 正值为逆时针
            
        self.chassis_move(0.0, 0.0, omega)
    
    def stop(self):
        # 停止所有模组
        if self.monitor:
            with self.monitor.lock:
                for mod in self.monitor.modules.values():
                    mod.target_speed = 0.0
                    mod.target_angle = mod.current_angle # 保持当前角度
            print("🛑 发送停止指令")

if __name__ == "__main__":
    # 直接运行时的简单测试
    monitor = VESCMonitor()
    monitor.start()
    
    control_loop = VESCControlLoop(monitor)
    control_loop.start()
    
    controller = SteerController(monitor)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller.stop()
        control_loop.stop()
        monitor.stop()
