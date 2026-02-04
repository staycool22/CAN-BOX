import math
import time
from utils.math_tools import optimize_swerve_angle, clamp, normalize_angle, get_shortest_path_to_angle
from utils.pid_controller import PIDController

class SwerveModule:
    """
    单个舵轮模组对象
    管理一个转向电机和一个驱动电机，处理局部的 PID 和安全逻辑。
    """
    def __init__(self, name, steer_id, drive_id, config):
        self.name = name
        self.steer_id = steer_id
        self.drive_id = drive_id
        self.config = config 
        
        # 目标状态 (internal storage)
        self._target_speed = 0.0 # m/s
        self._target_angle = 0.0 # degrees
        
        # 实际状态
        self.current_angle = 0.0
        self.current_speed = 0.0 # m/s (calculated from RPM)
        self.current_rpm = 0.0
        
        # 内部状态
        self.total_angle = 0.0 # 累计角度（未归一化，用于 PID）
        self.turns = 0
        self.last_pos = None
        self.enc2 = None
        self.last_raw_enc = None
        self.software_turns = 0
        self.initial_enc = None
        self.enc2_zero_locked = False
        self.calibration_mode = False
        
        self.last_update_time = 0
        self.is_online = False
        
        # 控制输出缓存
        self.steer_rpm_cmd = 0.0
        self.drive_rpm_cmd = 0.0
        
        # Watchdog for command update
        self.last_command_time = time.time()

        # PID 控制器初始化 (Kp from config, I/D = 0)
        self.pid = PIDController(kp=self.config.STEER_KP, ki=0.0, kd=0.0)

    @property
    def target_speed(self):
        return self._target_speed

    @target_speed.setter
    def target_speed(self, value):
        self._target_speed = value
        self.last_command_time = time.time() # Reset watchdog

    @property
    def target_angle(self):
        return self._target_angle

    @target_angle.setter
    def target_angle(self, value):
        self._target_angle = value
        self.last_command_time = time.time() # Reset watchdog

    def update_feedback(self, packet, is_steer=True):
        """
        处理来自 VESC 的反馈包
        :param packet: VESC CAN 数据包
        :param is_steer: True=转向电机, False=驱动电机
        """
        self.is_online = True
        self.last_update_time = time.time()
        
        if is_steer:
            # 转向电机逻辑: 更新角度 (复用原 _update_angle 核心逻辑)
            self._update_steer_angle_logic(packet)
        else:
            # 驱动电机逻辑: 更新速度
            self.current_rpm = float(packet.rpm)
            # Speed = (ERPM / PolePairs / ReductionRatio) * 2 * pi * R / 60
            self.current_speed = (self.current_rpm / (self.config.DRIVE_POLE_PAIRS * self.config.DRIVE_REDUCTION_RATIO)) * (2 * math.pi * self.config.DRIVE_WHEEL_RADIUS) / 60.0

    def _calc_enc2_error(self, current_enc2, zero_enc):
        err = normalize_angle(current_enc2 - zero_enc)
        return err

    def _update_steer_angle_logic(self, packet):
        """内部使用的转向角度更新逻辑"""
        if not hasattr(packet, 'enc1'):
            return

        current_enc = packet.enc1
        current_enc2 = packet.enc2 if hasattr(packet, 'enc2') else None
        
        # --- 初始上电校准 ---
        if not self.config.USE_CURRENT_AS_ZERO:
            zero_params = self.config.STEER_ZERO_PARAMS.get(self.steer_id)
            if not self.enc2_zero_locked and current_enc2 is not None and zero_params is not None:
                _, zero_enc = zero_params
                delta_wheel = self._calc_enc2_error(current_enc2, zero_enc)
                
                self.total_angle = delta_wheel
                self.turns = 0
                self.last_pos = current_enc
                self.enc2 = current_enc2
                
                if abs(delta_wheel) <= self.config.STEER_ANGLE_TOLERANCE:
                    self.enc2_zero_locked = True
                    self.initial_enc = current_enc
                    self.last_raw_enc = current_enc
                    self.software_turns = 0
                    # print(f"✅ {self.name} (ID {self.steer_id}) 初始零位已锁定 (Err={delta_wheel:.2f}°)")
                
                return

        # 初始化
        if self.last_raw_enc is None:
            self.last_raw_enc = current_enc
            self.software_turns = 0
            if self.initial_enc is None:
                self.initial_enc = current_enc
            # print(f"✅ {self.name} (ID {self.steer_id}) 初始位置: {current_enc:.2f}")

        # 圈数过零检测
        diff = current_enc - self.last_raw_enc
        if diff < -180:
            self.software_turns += 1
        elif diff > 180:
            self.software_turns -= 1
        self.last_raw_enc = current_enc

        # 计算当前软件角度
        current_abs_angle = (self.software_turns * 360.0) + current_enc - self.initial_enc
        current_wheel_angle = (current_abs_angle / self.config.STEER_REDUCTION_RATIO) % 360.0
        
        # --- 动态回零校验 ---
        soft_angle_norm = normalize_angle(current_wheel_angle)
        if current_enc2 is not None and not self.config.USE_CURRENT_AS_ZERO:
            zero_params = self.config.STEER_ZERO_PARAMS.get(self.steer_id)
            if zero_params and (self.enc2_zero_locked or self.calibration_mode):
                _, zero_enc = zero_params
                real_wheel_err = self._calc_enc2_error(current_enc2, zero_enc)
                deviation = abs(normalize_angle(soft_angle_norm - real_wheel_err))
                
                if deviation > 1.0:
                    print(f"⚠️ {self.name} [动态校准] 偏差修正: Soft={soft_angle_norm:.2f}°, Real(Enc2)={real_wheel_err:.2f}°, Diff={deviation:.2f}°")
                    
                    new_initial = current_enc - (real_wheel_err * self.config.STEER_REDUCTION_RATIO)
                    self.initial_enc = new_initial
                    self.software_turns = 0
                    current_abs_angle = current_enc - new_initial
                    current_wheel_angle = (current_abs_angle / self.config.STEER_REDUCTION_RATIO) % 360.0
                    print(f"   -> 修正后: Angle={current_wheel_angle:.2f}°")

                elif abs(real_wheel_err) < 0.5 and deviation > 0.1:
                    print(f"✅ {self.name} [归零确认] 到达物理零点，微调对齐: Real(Enc2)={real_wheel_err:.2f}°")
                    
                    new_initial = current_enc - (real_wheel_err * self.config.STEER_REDUCTION_RATIO)
                    self.initial_enc = new_initial
                    self.software_turns = 0
                    
                    current_abs_angle = current_enc - new_initial
                    current_wheel_angle = (current_abs_angle / self.config.STEER_REDUCTION_RATIO) % 360.0
                
                if abs(real_wheel_err) <= self.config.STEER_ANGLE_TOLERANCE:
                    self.enc2_zero_locked = True

        self.total_angle = current_wheel_angle
        self.turns = self.software_turns
        self.last_pos = current_enc
        if current_enc2 is not None:
             self.enc2 = current_enc2

    def calculate_control(self):
        """
        计算控制指令 (应在控制循环中调用)
        :return: (steer_rpm, drive_rpm)
        """
        # Watchdog Check: 如果超过 0.5s 未更新指令，强制停车
        if time.time() - self.last_command_time > 0.5:
            # print(f"⚠️ {self.name} Watchdog Timeout -> Stop")
            self._target_speed = 0.0
            # Keep target_angle to maintain position
            
        # 1. 角度优化
        if self.calibration_mode:
            # Check if we should exit calibration mode (if moving)
            if abs(self.target_speed) > 0.01:
                print(f"⚠️ {self.name} Exiting Calibration Mode: Target Speed={self.target_speed:.3f} > 0.01")
                self.calibration_mode = False
                # Normal optimization when moving
                final_angle, reversed_speed = optimize_swerve_angle(self.target_angle, self.total_angle)
            else:
                # In calibration mode (stopped), force physical alignment to target
                # Do NOT use 180-degree flip optimization
                final_angle = get_shortest_path_to_angle(self.target_angle, self.total_angle)
                reversed_speed = False
        else:
            # Normal operation
            # Special logic for 0 degree target (Forward/Backward):
            # User prefers to keep wheels at 0 degrees physically instead of 180 degrees reversed.
            if abs(self.target_angle) < 0.1:
                final_angle = get_shortest_path_to_angle(self.target_angle, self.total_angle)
                reversed_speed = False
            else:
                # Use optimize_swerve_angle for other angles to minimize rotation
                final_angle, reversed_speed = optimize_swerve_angle(self.target_angle, self.total_angle)
                
                # Special Check for 90/-90 degrees (Strafe)
                # If target is exactly 90 or -90, and optimization flipped it to -90 or 90 (reversed),
                # this might cause wheels to point in opposite directions if their starting positions were slightly different.
                # For pure strafing (speed > 0), we want consistent orientation if possible.
                # However, optimize_swerve_angle is purely local.
                # The user report says: "Two wheels sent 90 degrees, but one should be -90 to match direction?"
                # Actually, for right strafe (move right), wheel direction should be -90 deg (pointing right).
                # If target is -90, and current is 90, optimization might keep 90 and reverse speed.
                # If another wheel is at -90, it stays at -90 and positive speed.
                # Result: One wheel at 90 (reversed), one at -90 (forward). Both push right. This is physically correct.
                # BUT user says: "Actually should send opposite sign 90 degrees to ensure same direction rotation".
                # If the user implies the MOTORS should rotate same direction, that depends on mechanical installation.
                # If user implies WHEELS should point same angle, then we should disable optimization for pure strafe too?
                
                # Let's enforce physical angle consistency for pure 90/-90 targets too, similar to 0-degree logic.
                if abs(abs(self.target_angle) - 90.0) < 0.1:
                     final_angle = get_shortest_path_to_angle(self.target_angle, self.total_angle)
                     reversed_speed = False
        
        # if self.calibration_mode and abs(self.target_speed) > 0.01:
        #    self.calibration_mode = False
        
        target_speed_mps = self.target_speed
        if reversed_speed:
            target_speed_mps = -target_speed_mps
            
        # 2. 转向 PID 计算
        error_wheel_deg = final_angle - self.total_angle
        # error_wheel_deg 已经是差值，不需要再归一化，因为 optimize_swerve_angle 保证了它是最近路径
        
        error_motor_deg = error_wheel_deg * self.config.STEER_REDUCTION_RATIO
        
        TOLERANCE_MOTOR = self.config.STEER_ANGLE_TOLERANCE * self.config.STEER_REDUCTION_RATIO
        
        steer_rpm = 0.0
        if abs(error_motor_deg) > TOLERANCE_MOTOR:
            # 使用 PID 类计算
            # 更新参数以支持动态调整
            self.pid.kp = self.config.STEER_KP
            # self.pid.ki = 0.0 # 目前未使用
            # self.pid.kd = 0.0 # 目前未使用
            
            steer_rpm = self.pid.update(error_motor_deg)
            
            # 最小启动 RPM
            MIN_RPM = 500.0
            if abs(steer_rpm) < MIN_RPM:
                steer_rpm = math.copysign(MIN_RPM, steer_rpm)
                
            # 限幅
            MAX_RPM = 8000.0
            steer_rpm = clamp(steer_rpm, -MAX_RPM, MAX_RPM)
            
            # 反转逻辑
            if self.steer_id in self.config.STEER_INVERTED_IDS:
                steer_rpm = -steer_rpm
        
        self.steer_rpm_cmd = steer_rpm
        
        # 3. 驱动互锁逻辑
        # 计算当前轮子的物理误差（归一化后）
        phys_error = abs(normalize_angle(self.total_angle - final_angle))
        
        speed_factor = 1.0
        
        if phys_error > 60.0:
             # 大角度偏差：必须减速等待转向
             speed_factor = 0.0
        elif phys_error > 30.0:
             # 中等偏差：线性衰减，但不停
             speed_factor = 1.0 - (phys_error - 30.0) / 30.0 
             # 30deg -> 1.0, 60deg -> 0.0
        else:
             # 小偏差 (<30度)：全速驱动
             speed_factor = 1.0
             
        final_drive_speed = target_speed_mps * speed_factor
        
        # 转换为 RPM
        drive_rpm = (final_drive_speed / (2 * math.pi * self.config.DRIVE_WHEEL_RADIUS)) * 60 * self.config.DRIVE_REDUCTION_RATIO * self.config.DRIVE_POLE_PAIRS
        
        # 驱动电机反转逻辑 (FR, RR 右侧可能需要反转，视接线而定)
        # 这里假设 config 里没有专门的 DRIVE_INVERTED_IDS，根据旧代码逻辑：
        if self.drive_id == self.config.FR_DRIVE_ID or self.drive_id == self.config.RR_DRIVE_ID: 
             # 注意：旧代码只反转了 FR，这里根据通用性可能需要调整。
             # 暂时保留旧代码逻辑： FR 反转
             if self.drive_id == self.config.FR_DRIVE_ID:
                 drive_rpm = -drive_rpm
                 
        # 驱动限幅
        MAX_DRIVE_RPM = self.config.MAX_RPM_REF
        MIN_DRIVE_RPM = 300.0
        if abs(drive_rpm) > 1.0:
             if abs(drive_rpm) < MIN_DRIVE_RPM:
                 drive_rpm = math.copysign(MIN_DRIVE_RPM, drive_rpm)
             elif abs(drive_rpm) > MAX_DRIVE_RPM:
                 drive_rpm = math.copysign(MAX_DRIVE_RPM, drive_rpm)
        else:
            drive_rpm = 0.0
            
        self.drive_rpm_cmd = drive_rpm
        
        return self.steer_rpm_cmd, self.drive_rpm_cmd
