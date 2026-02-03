import sys
import os
import time
import threading
import math
from typing import List, Dict, Optional, Tuple

try:
    from chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics, AckermannSteeringKinematics
except ImportError:
    # 如果同级目录下找不到，可能是在其他路径运行，尝试添加路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    try:
        from chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics, AckermannSteeringKinematics
    except ImportError:
        print("警告: 未找到 chassis_kinematics 模块。转向控制器运动学功能可能失效。")


# 添加父目录到 path 以查找 CANMessageTransmitter
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", ".."))

if project_root not in sys.path:
    sys.path.append(project_root)

from steer_wheel_config import BasicConfig
import steer_wheel_can_bus
from steer_wheel_can_bus import VESC_CAN_STATUS


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
        
        # 状态跟踪
        self.motor_states = {
            mid: {
                "rpm": 0.0,
                "current": 0.0,
                "pid_pos": 0.0,
                "total_angle": 0.0,
                "turns": 0,
                "last_pos": None,
                "enc2": None,
                "last_raw_enc": None,
                "software_turns": 0,
                "initial_enc": None,
                "motor_abs_pos": 0.0,
                "enc2_zero_locked": False
            } for mid in BasicConfig.get_all_ids()
        }
        
        # 转向目标角度 (Wheel Angle, degrees)
        self.steer_targets: Dict[int, float] = {}
        
        # 运行时校准参数 (允许外部覆盖 BasicConfig 中的默认值)
        # 格式: { mid: (zero_turns, zero_enc) }
        self.runtime_zero_params = BasicConfig.STEER_ZERO_PARAMS.copy()
        
        if BasicConfig.USE_CURRENT_AS_ZERO:
            print("配置为: 使用当前位置作为零点 (忽略预设参数)")
            # 清空预设参数，等待首次数据到来时捕获
            self.runtime_zero_params.clear()

    def _update_angle(self, motor_id: int, packet):
        """
        基于自定义 Status 2 协议更新总角度。
        
        修改后逻辑 (软件计算圈数 + Enc2 动态校准):
        1. 读取 Enc1 (0-360) 并进行圈数累积，计算出 software_angle。
        2. 如果 software_angle 接近 0 度 (回零操作期间)，启动 Enc2 双重校验。
        3. 读取 Enc2 真实角度，与设定的零点参数 (zero_enc) 比较。
        4. 如果 Enc2 偏差超过容差，说明软件零点已漂移 -> 强制重置 Enc1 初始值和圈数，对齐到 Enc2。
        """
        if not hasattr(packet, 'enc1'):
            return

        state = self.motor_states[motor_id]
        current_enc = packet.enc1 # 当前原始角度 (0-360)
        current_enc2 = packet.enc2 if hasattr(packet, 'enc2') else None
        
        # --- 初始上电校准 (First Time Only) ---
        if not BasicConfig.USE_CURRENT_AS_ZERO:
            zero_params = self.runtime_zero_params.get(motor_id)
            if not state.get("enc2_zero_locked") and current_enc2 is not None and zero_params is not None:
                _, zero_enc = zero_params
                delta_wheel = ((current_enc2 - zero_enc + 180) % 360) - 180
                
                state["total_angle"] = delta_wheel
                state["turns"] = 0
                state["last_pos"] = current_enc
                state["motor_abs_pos"] = delta_wheel * BasicConfig.STEER_REDUCTION_RATIO
                state["enc2"] = current_enc2
                
                if abs(delta_wheel) <= BasicConfig.STEER_ANGLE_TOLERANCE:
                    state["enc2_zero_locked"] = True
                    state["initial_enc"] = current_enc
                    state["last_raw_enc"] = current_enc
                    state["software_turns"] = 0
                    print(f"✅ 电机 {motor_id} 初始零位已锁定 (Err={delta_wheel:.2f}°)")
                    # 更新参数，虽然这里 zero_turns 设为 0，但关键是记录下了对齐瞬间的 current_enc 作为 initial_enc
                    # self.runtime_zero_params[motor_id] = (0, current_enc) 
                return
        
        # 初始化: 如果是第一次收到数据 (且没有经过上面的 Enc2 锁定)
        if state.get("last_raw_enc") is None:
            state["last_raw_enc"] = current_enc
            state["software_turns"] = 0
            if state.get("initial_enc") is None:
                state["initial_enc"] = current_enc
            print(f"✅ [初始化] 电机 {motor_id} 初始位置: {current_enc:.2f}")
            
        # --- 圈数过零检测 (Enc1) ---
        last_enc = state["last_raw_enc"]
        diff = current_enc - last_enc
        
        if diff < -180:
            state["software_turns"] += 1
        elif diff > 180:
            state["software_turns"] -= 1
            
        state["last_raw_enc"] = current_enc
        
        # --- 计算当前软件角度 (相对于上电/锁定时刻) ---
        turns = state["software_turns"]
        initial_enc = state["initial_enc"]
        current_abs_angle = (turns * 360.0) + current_enc - initial_enc
        current_wheel_angle = (current_abs_angle / BasicConfig.STEER_REDUCTION_RATIO) % 360.0
        
        # --- 动态回零校验 (Dynamic Re-Calibration) ---
        # 扩展校验逻辑 (用户需求 01-29):
        # 1. 当软件角度接近零点时 (<10度)，持续检查 Enc2。
        # 2. 如果 Enc2 与零点有偏差，继续按照 Enc2 进行校正 (更新 initial_enc，使 total_angle 追踪 Enc2)。
        # 3. 在确认收到电机确实为 Enc2 设定的零点之后 (real_wheel_err < 0.5)，重新标记 Enc1 零点。
        soft_angle_norm = current_wheel_angle if current_wheel_angle < 180 else current_wheel_angle - 360
        
        if state.get("enc2_zero_locked") and abs(soft_angle_norm) < 10.0:
            if current_enc2 is not None and not BasicConfig.USE_CURRENT_AS_ZERO:
                zero_params = self.runtime_zero_params.get(motor_id)
                if zero_params:
                    _, zero_enc = zero_params
                    # 计算 Enc2 实际物理偏差 (-180 ~ 180)
                    real_wheel_err = ((current_enc2 - zero_enc + 180) % 360) - 180
                    deviation = abs(soft_angle_norm - real_wheel_err)
                    
                    # 情况 A: 偏差较大 (>1.0度)，说明 Enc1 累计误差或漂移
                    # 此时强制按照 Enc2 的读数来重置 Enc1 基准，引导电机往真正的 Enc2 零点走
                    if deviation > 1.0:
                        print(f"⚠️ [动态校准] 偏差修正: Soft={soft_angle_norm:.2f}°, Real(Enc2)={real_wheel_err:.2f}°, Diff={deviation:.2f}°")
                        
                        # 重置 Enc1 基准: new_initial 使得 (current_enc - new_initial) / Ratio = real_wheel_err
                        new_initial = current_enc - (real_wheel_err * BasicConfig.STEER_REDUCTION_RATIO)
                        state["initial_enc"] = new_initial
                        state["software_turns"] = 0 # 归零时强制重置圈数
                        
                        # 立即刷新当前角度
                        current_abs_angle = current_enc - new_initial
                        current_wheel_angle = (current_abs_angle / BasicConfig.STEER_REDUCTION_RATIO) % 360.0
                        print(f"   -> 修正后: Angle={current_wheel_angle:.2f}°")

                    # 情况 B: 物理上已经非常接近零点 (Enc2 < 0.5度)，但软件上可能还有微小偏差 (>0.1度)
                    # 此时执行“最终确认重置”，确保在物理零点时，软件零点也完美对齐
                    elif abs(real_wheel_err) < 0.5 and deviation > 0.1:
                        print(f"✅ [归零确认] 到达物理零点，微调对齐: Real(Enc2)={real_wheel_err:.2f}°")
                        
                        new_initial = current_enc - (real_wheel_err * BasicConfig.STEER_REDUCTION_RATIO)
                        state["initial_enc"] = new_initial
                        state["software_turns"] = 0
                        
                        current_abs_angle = current_enc - new_initial
                        current_wheel_angle = (current_abs_angle / BasicConfig.STEER_REDUCTION_RATIO) % 360.0

        # --- 更新状态 ---
        state["total_angle"] = current_wheel_angle
        state["turns"] = state["software_turns"]
        state["last_pos"] = current_enc
        # 归一化显示角度 (-180 ~ 180)
        disp_angle = current_wheel_angle % 360.0
        if disp_angle > 180:
            disp_angle -= 360.0
        state["display_angle"] = disp_angle
        
        state["motor_abs_pos"] = current_abs_angle
        
        if current_enc2 is not None:
             state["enc2"] = current_enc2

    def set_zero_calibration_params(self, motor_id: int, zero_turns: int, zero_enc: float):
        """
        外部接口：设置转向电机零位参数
        :param motor_id: 电机 CAN ID
        :param zero_turns: 零位时的圈数
        :param zero_enc: 零位时的编码器角度 (0-360)
        """
        self.runtime_zero_params[motor_id] = (zero_turns, zero_enc)
        print(f"✅ 更新电机 {motor_id} 零位参数: Turns={zero_turns}, Enc={zero_enc}")
        
        # 如果当前已经有状态数据，立即触发一次角度刷新
        if motor_id in self.motor_states:
             # 注意：这里我们无法直接调用 _update_angle 因为它需要 packet
             # 但下一次 CAN 消息到来时会自动应用新参数
             pass

    def perform_zero_calibration(self):
        """
        执行零位校准。
        上电时自动运行至零位。
        
        逻辑:
        1. 此时 _update_angle 已经根据 (Zero_Turns, Zero_Enc) 计算出了当前轮子的实际角度 total_angle。
           例如：如果当前在零位，total_angle 应该接近 0。
           如果当前偏离零位 10 度，total_angle 应该是 10 或 -10。
        2. 我们只需要将目标角度设为 0，控制器就会自动把轮子转回零位。
        """
        print("执行自动回零操作...")
        
        # 遍历所有转向电机
        for mid in BasicConfig.get_steer_ids():
            if mid not in self.motor_states:
                continue
            
            # 简单粗暴：将目标设为 0 度
            # 底层的 _control_steer_motor 会根据当前 total_angle (已校准) 和目标 0 度计算误差并控制
            self.steer_targets[mid] = 0.0
            print(f"电机 {mid} 目标已设为 0.0 度 (自动回零)")
            
            # 打印当前状态供确认
            state = self.motor_states[mid]
            curr_angle = state.get("total_angle", "N/A")
            curr_turns = state.get("turns", "N/A")
            curr_enc = state.get("last_pos", "N/A")
            print(f"  -> 当前状态: Angle={curr_angle}, Turns={curr_turns}, Enc={curr_enc}")
            print(f"  -> 使用零位参数: {self.runtime_zero_params.get(mid)}")

    def get_vesc_interface(self, motor_id: int):
        """
        根据电机 ID 获取对应的 VESC 接口实例
        支持两种模式：
        1. 默认模式: 转向电机 -> self.vesc, 驱动电机 -> self.vesc_drive
        2. 轮组模式: 根据 WHEEL_GROUP_CAN_MAPPING 查找对应通道
        """
        if BasicConfig.ENABLE_WHEEL_GROUP_CAN_MODE:
            # 轮组分组模式
            # 查找 motor_id 属于哪个通道
            for channel, ids in BasicConfig.WHEEL_GROUP_CAN_MAPPING.items():
                if motor_id in ids:
                    if channel == 0: return self.vesc # can0 (stored in self.vesc)
                    if channel == 1: return self.vesc_drive # can1 (stored in self.vesc_drive)
            
            # 如果没找到，默认返回 can0
            print(f"⚠️ 警告: Motor ID {motor_id} 未在映射中找到，默认使用 can0")
            return self.vesc
        else:
            # 默认模式
            if motor_id in BasicConfig.get_steer_ids():
                return self.vesc
            else:
                return self.vesc_drive

    def _control_steer_motor(self, motor_id: int, state: dict):
        """
        转向电机闭环控制逻辑 (由 Monitor 线程调用)
        """
        # print(f"DEBUG: Control loop for {motor_id}") # 暂时调试
        
        # 获取对应通道的 VESC 接口
        vesc_interface = self.get_vesc_interface(motor_id)
        if not vesc_interface:
            return

        # 如果没有设定目标，默认锁死当前位置 (使用 PID 位置保持)
        if motor_id not in self.steer_targets:
            # 保持当前位置不动
            # vesc_interface.send_pos(motor_id, state["pid_pos"]) 
            return

        target_wheel_angle = self.steer_targets[motor_id]
        
        # 获取当前轮子角度 (已在 _update_angle 中基于零位参数计算好)
        current_wheel_angle = state.get("total_angle", 0.0)
        
        # 误差 (轮子角度)
        error_wheel_deg = target_wheel_angle - current_wheel_angle
        # 归一化误差到 -180 ~ 180 (最短路径)
        error_wheel_deg = (error_wheel_deg + 180) % 360 - 180
        
        # 轮子误差 -> 电机误差
        error_motor_deg = error_wheel_deg * BasicConfig.STEER_REDUCTION_RATIO
        
        # 容差配置 (基于轮子角度)
        TOLERANCE_WHEEL_DEG = BasicConfig.STEER_ANGLE_TOLERANCE # 使用配置文件的容差
        # 自动计算电机角度容差 (例如: 0.5 * 20 = 10.0 度电机角度)
        TOLERANCE_MOTOR_DEG = TOLERANCE_WHEEL_DEG * BasicConfig.STEER_REDUCTION_RATIO
        
        if abs(error_motor_deg) > TOLERANCE_MOTOR_DEG:
            # RPM 控制模式
            kp = BasicConfig.STEER_KP # Kp 针对电机角度
            rpm_target = error_motor_deg * kp
            
            # --- Inversion Logic ---
            if motor_id in BasicConfig.STEER_INVERTED_IDS:
                rpm_target = -rpm_target
            
            # 最小启动 RPM (克服静摩擦力)
            # 如果计算出的 RPM 太小，电机可能不动
            # 增大到 500 以确保能克服转向阻力
            MIN_RPM = 500.0
            if abs(rpm_target) < MIN_RPM:
                rpm_target = math.copysign(MIN_RPM, rpm_target)
            
            # 限幅
            MAX_RPM = 8000.0
            rpm_target = max(min(rpm_target, MAX_RPM), -MAX_RPM)
            
            # 发送 RPM 指令
            # 开启调试打印，以便观察为何不动
            if abs(rpm_target) > 0.1:
                # 限制打印频率
                now = time.time()
                if not hasattr(self, '_last_debug_print'): self._last_debug_print = {}
                if now - self._last_debug_print.get(motor_id, 0) > 0.2:
                    enc2_val = state.get('enc2')
                    enc2_str = f"{enc2_val:.2f}" if enc2_val is not None else "N/A"
                    print(f"[DEBUG ID{motor_id}] Tgt={target_wheel_angle:.1f}, Cur={current_wheel_angle:.1f}, ErrWheel={error_wheel_deg:.1f}, Enc2={enc2_str}, RPM_Cmd={rpm_target:.1f}")
                    self._last_debug_print[motor_id] = now
            
            vesc_interface.send_rpm(motor_id, rpm_target)
        else:
            if state.get("last_pos") is not None:
                vesc_interface.send_pos(motor_id, state["last_pos"])    
            # 增加锁定状态的低频打印 (为了能看到已经到位的电机状态)
            now = time.time()
            if not hasattr(self, '_last_lock_print'): self._last_lock_print = {}
            if now - self._last_lock_print.get(motor_id, 0) > 2.0: # 2秒打印一次
                enc2_val = state.get('enc2')
                enc2_str = f"{enc2_val:.2f}" if enc2_val is not None else "N/A"
                print(f"[DEBUG ID{motor_id}] Locked at {state.get('last_pos', 'N/A')} (ErrWheel={error_wheel_deg:.2f}, Enc2={enc2_str})")
                self._last_lock_print[motor_id] = now


    def _process_vesc_packet(self, msg_id, packet):
        """
        处理 VESC 状态包，更新电机状态
        """
        # 提取 VESC ID（扩展帧 ID 的最后一个字节）
        vesc_id = msg_id & 0xFF
        # 提取 Status ID (扩展帧 ID 的中间字节)
        status_id = (msg_id >> 8) & 0xFF
        
        if vesc_id in self.motor_states:
            with self.lock:
                state = self.motor_states[vesc_id]
                
                # 根据状态帧类型更新特定字段
                if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
                    state["rpm"] = float(packet.rpm)
                    state["current"] = float(packet.current)
                    state["pid_pos"] = float(packet.pid_pos_now)
                    
                    # Fix: Update last_pos for drive motors to mark them as "online"
                    if vesc_id not in BasicConfig.get_steer_ids():
                        state["last_pos"] = state["pid_pos"]
                    
                    # 计算线速度 (m/s)
                    # Speed = (ERPM / PolePairs) * 2 * pi * R / 60
                    erpm = state["rpm"]
                    speed_mps = (erpm / BasicConfig.DRIVE_POLE_PAIRS) * (2 * math.pi * BasicConfig.DRIVE_WHEEL_RADIUS) / 60.0
                    state["speed"] = speed_mps
                    
                elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_2:
                    # 新逻辑：直接从 Status 2 读取圈数和编码器
                    if vesc_id in BasicConfig.get_steer_ids():
                        self._update_angle(vesc_id, packet)
                        
                # 记录数据 
                if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
                    # 实时打印供调试 (基于时间戳，每个ID独立控制频率)
                    if not hasattr(self, 'last_print_time'):
                        self.last_print_time = {}
                    
                    now = time.time()
                    if now - self.last_print_time.get(vesc_id, 0) > 0.1: # 10Hz
                        self.last_print_time[vesc_id] = now
                        
                        # 计算期望值
                        target_angle = self.steer_targets.get(vesc_id)
                        target_info = "Target: N/A"
                        if target_angle is not None:
                            zero_turns, zero_enc = self.runtime_zero_params.get(vesc_id, (0, 0.0))
                            zero_motor_abs = (zero_turns * 360.0) + zero_enc
                            target_motor_abs = target_angle * BasicConfig.STEER_REDUCTION_RATIO + zero_motor_abs
                            target_turns = math.floor(target_motor_abs / 360.0)
                            target_enc = target_motor_abs % 360.0
                            target_info = f"期望: Angle={target_angle:.2f}, Turns={target_turns}, Raw={target_enc:.2f}"
                        
                        # 当前值
                        raw_val = state.get('last_pos')
                        if raw_val is None: raw_val = 0.0
                        # 使用 display_angle (归一化到 -180~180)
                        disp_angle = state.get('display_angle', state.get('total_angle', 0))
                        current_info = f"当前: Angle={disp_angle:.2f}, Turns={state.get('turns', 0)}, Raw={raw_val:.2f}"
                        
                        # 仅打印转向电机 (ID 38, 39)，屏蔽驱动电机
                        if vesc_id in BasicConfig.get_steer_ids():
                            print(f"ID: {vesc_id} | {target_info} | {current_info}")

    def _monitor_loop(self):
        last_control_time = time.time()
        CONTROL_INTERVAL = 0.01 # 50Hz Control Loop
        
        while self.running:
            # --- 1. 接收 CAN 消息 ---
            # 使用循环读取所有可用消息，避免缓冲区积压
            # 注意：不再在接收循环中直接调用控制逻辑，而是解耦
            
            # 1.1 接收转向电机消息 (vesc)
            if self.vesc:
                while self.running:
                    msg_id, packet = self.vesc.receive_decode(timeout=0)
                    if msg_id is None:
                        break
                    self._process_vesc_packet(msg_id, packet)
            
            # 1.2 接收驱动电机消息 (vesc_drive)
            if self.vesc_drive:
                while self.running:
                    msg_id, packet = self.vesc_drive.receive_decode(timeout=0)
                    if msg_id is None:
                        break
                    self._process_vesc_packet(msg_id, packet)

            # --- 2. 执行控制逻辑 (定频 50Hz) ---
            now = time.time()
            if now - last_control_time >= CONTROL_INTERVAL:
                for mid in BasicConfig.get_steer_ids():
                    # 获取最新状态副本进行控制计算
                    with self.lock:
                        state = self.motor_states.get(mid)
                        if state:
                            # 只有当接收到过数据（pid_pos非0或已更新）才控制
                            # 简单检查：last_pos 不为 None
                            if state.get("last_pos") is not None:
                                self._control_steer_motor(mid, state)
                
                last_control_time = now
            
            # 短暂休眠以防止空转占用过多 CPU
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
            for mid in BasicConfig.get_steer_ids():
                state = self.motor_states.get(mid)
                if state and state.get("last_pos") is not None:
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
            return self.motor_states.get(motor_id, {}).copy()

class SteerController:
    def __init__(self, monitor: VESCMonitor):
        self.monitor = monitor
        self.vesc = monitor.vesc
        self.vesc_drive = monitor.vesc_drive # 获取驱动电机控制器 (VESC)
        
        # 初始化 kinematics
        # 几何参数 (与 test_steer_control.py 保持一致: L=0.30, W=0.44)
        self.geometry = ChassisGeometry(length=0.30, width=0.44, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
        
        # 根据配置选择运动学模型
        if BasicConfig.ENABLE_ACKERMANN_MODE:
            print(f"🔧 运动模式: 阿克曼转向 (4WS={BasicConfig.ACKERMANN_4WS})")
            self.kinematics = AckermannSteeringKinematics(self.geometry, is_4ws=BasicConfig.ACKERMANN_4WS)
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
            print(f"🔄 切换至: 阿克曼转向模式 (4WS={BasicConfig.ACKERMANN_4WS})")
            self.kinematics = AckermannSteeringKinematics(self.geometry, is_4ws=BasicConfig.ACKERMANN_4WS)
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
            
        # 更新目标，由 _monitor_loop 进行控制
        self.monitor.steer_targets[motor_id] = target_angle

    def calibrate_home(self):
        """
        校准归位：强制使用 Enc2 绝对零点进行回零。
        这会重置零位锁定状态，确保每次都重新对齐到 Enc2 标定的零点。
        """
        print("🔄 执行绝对零位校准 (Force Enc2 Re-alignment)...")
        for mid in BasicConfig.get_steer_ids():
            if mid in self.monitor.motor_states:
                 # 重置锁定标志，强制 _update_angle 重新使用 enc2 计算角度
                 self.monitor.motor_states[mid]["enc2_zero_locked"] = False
                 self.monitor.steer_targets[mid] = 0.0
                 print(f"  -> 电机 {mid} 正在回零 (Enc2 Lock Reset)")

    def apply_kinematics(self, wheel_states: Dict[str, Tuple[float, float]]):
        """
        应用运动学计算结果到电机。
        :param wheel_states: {wheel_name: (speed_mps, angle_rad)}
        """
        # 映射名称到 ID
        name_map = {
            "FL": (BasicConfig.FL_STEER_ID, BasicConfig.FL_DRIVE_ID),
            "FR": (BasicConfig.FR_STEER_ID, BasicConfig.FR_DRIVE_ID),
            "RL": (BasicConfig.RL_STEER_ID, BasicConfig.RL_DRIVE_ID),
            "RR": (BasicConfig.RR_STEER_ID, BasicConfig.RR_DRIVE_ID)
        }
        
        drive_speeds = {} # 驱动电机ID -> 转速(RPM)
        
        for name, (steer_id, drive_id) in name_map.items():
            if name not in wheel_states:
                continue
                
            target_speed, target_angle_val = wheel_states[name]
            
            # 1. 角度优化 (舵轮优化)
            # chassis_kinematics 返回的已经是度数了，不需要再转换
            target_angle_deg = target_angle_val
            
            # Debug: 打印 ID38 的输入和目标
            if steer_id == 38:
                 print(f"[DEBUG ID38] Speed: {target_speed:.2f}, Input Angle: {target_angle_val:.2f}, Target Deg: {target_angle_deg:.2f}")

            # 获取当前逻辑角度 (从 monitor 获取)
            # 注意: monitor 只有 motor_states (PID Pos -> Total Angle)
            # 我们需要当前的逻辑转向角度
            current_state = self.monitor.get_state(steer_id)
            current_angle = current_state.get("total_angle", 0.0)
            
            # 归一化误差到 -180 ~ 180
            diff = (target_angle_deg - current_angle + 180) % 360 - 180
            
            final_angle = current_angle + diff
            final_speed = target_speed
            
            # 2. 发送转向指令
            self._send_steer_pos(steer_id, final_angle)
            
            rpm = (final_speed / (2 * math.pi * BasicConfig.DRIVE_WHEEL_RADIUS)) * 60 * BasicConfig.DRIVE_REDUCTION_RATIO * BasicConfig.DRIVE_POLE_PAIRS
            
            # 打印调试信息 (仅在有速度时打印，避免刷屏)
            if abs(rpm) > 1.0 and steer_id == BasicConfig.FL_STEER_ID:
                 print(f"[调试] 速度: {final_speed:.2f} m/s -> 转速: {rpm:.2f} RPM")

            # 无论使用哪种控制器，都先计算目标转速并存入 drive_speeds
            final_rpm = rpm
            if drive_id == BasicConfig.FR_DRIVE_ID: # 右前轮
                 final_rpm = -rpm
            
            drive_speeds[drive_id] = final_rpm
        
        # 4. 发送驱动指令 (改为 VESC 逐个发送)
        # 注意: 兼容轮组分组模式，需通过 monitor 获取正确的接口
        if self.monitor:
            for drive_id, rpm in drive_speeds.items():
                # 限幅逻辑: 
                # 1. 最小启动转速 300 (克服摩擦力)
                # 2. 最大安全转速 (使用配置参数 MAX_RPM_REF)
                MAX_DRIVE_RPM = BasicConfig.MAX_RPM_REF
                MIN_DRIVE_RPM = 300.0
                
                if abs(rpm) > 1.0: # 如果有速度请求
                    if abs(rpm) < MIN_DRIVE_RPM:
                        rpm = MIN_DRIVE_RPM * (1 if rpm > 0 else -1)
                    elif abs(rpm) > MAX_DRIVE_RPM:
                        rpm = MAX_DRIVE_RPM * (1 if rpm > 0 else -1)
                else:
                    rpm = 0.0

                vesc_interface = self.monitor.get_vesc_interface(drive_id)
                if vesc_interface:
                    # 安全检查：只有当 monitor 监控到该电机在线(有数据)时才发送控制
                    # 这样即使配置了4轮，但只连接了2轮，也不会对未连接的电机报错
                    drive_state = self.monitor.get_state(drive_id)
                    if drive_state.get("last_pos") is not None:
                        vesc_interface.send_rpm(drive_id, rpm)
                        print(f"Drive VESC ID {drive_id} -> {rpm:.1f} RPM")
                    # else:
                    #     print(f"Skipping offline drive motor {drive_id}")
        else:
             print("⚠️ monitor is None! 无法发送驱动指令")

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
        # 如果 omega_rad 为 0，则是纯平移
        # 如果 vx, vy 为 0，则是纯旋转
        # 如果都有值，则是螺旋/复合运动
        wheel_states = self.kinematics.inverse_kinematics(vx, vy, omega_rad)
        
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
        # 停止 VESC 驱动
        if self.monitor:
            for mid in BasicConfig.get_drive_ids():
                vesc_interface = self.monitor.get_vesc_interface(mid)
                if vesc_interface:
                    vesc_interface.send_rpm(mid, 0)
            print("🛑 发送 VESC 停止指令")

if __name__ == "__main__":
    # 直接运行时的简单测试
    monitor = VESCMonitor()
    monitor.start()
    
    controller = SteerController(monitor)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        controller.stop()
        monitor.stop()
