import sys
import os
import time
import threading
import logging
import math
from typing import List, Dict, Optional, Tuple

try:
    from chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics
except ImportError:
    # 如果同级目录下找不到，可能是在其他路径运行，尝试添加路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    try:
        from chassis_kinematics import ChassisGeometry, FourWheelSteeringKinematics
    except ImportError:
        print("警告: 未找到 chassis_kinematics 模块。转向控制器运动学功能可能失效。")


# 添加父目录到 path 以查找 CANMessageTransmitter
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", ".."))

if project_root not in sys.path:
    sys.path.append(project_root)

try:
    from CAN.CANMessageTransmitter import CANMessageTransmitter
    # 尝试通过 CANMessageTransmitter 选择设备，或者直接导入
    # 注意：CANMessageTransmitter.choose_can_device("TZCAN") 返回的是类
    TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
    from VESC_test.can_vesc import VESC, VESC_CAN_STATUS, buffer_get_int16, buffer_get_int32, buffer_get_float16, buffer_get_float32
except ImportError as e:
    print(f"Import Error: {e}")
    raise

# --- 配置类 ---
class BasicConfig:
    # VESC ID 配置
    FL_STEER_ID = 47  # 左前转向电机
    FR_STEER_ID = 48  # 右前转向电机
    RL_STEER_ID = 105  # 左后转向电机
    RR_STEER_ID = 106  # 右后转向电机

    FL_DRIVE_ID = 32  # 左前轮毂电机
    FR_DRIVE_ID = 104  # 右前轮毂电机
    RL_DRIVE_ID = 107  # 左后轮毂电机
    RR_DRIVE_ID = 108  # 右后轮毂电机

    # 转向电机零位偏置校准 (单位: 度)
    # 请在此处填写【当轮子物理朝向正前方时，读取到的编码器角度值】
    # 旧参数保留但暂不使用，使用下方新的绝对零位参数
    FL_STEER_OFFSET = 0
    FR_STEER_OFFSET = 0
    RL_STEER_OFFSET = 0.0
    RR_STEER_OFFSET = 0.0

    # --- 新增：绝对零位校准参数 ---
    # 定义轮子回正（0度）时，对应的【电机圈数】和【编码器角度(0-360)】
    # 格式: { MOTOR_ID: (ZERO_TURNS, ZERO_ENC_ANGLE) }
    STEER_ZERO_PARAMS = {}
    USE_TURNS_FOR_ANGLE = False

    # 转向电机（用于角度跟踪）
    @classmethod
    def get_steer_ids(cls):
        # return [cls.FL_STEER_ID, cls.FR_STEER_ID, cls.RL_STEER_ID, cls.RR_STEER_ID]
        # 暂时只启用前两个转向电机
        return [cls.FL_STEER_ID, cls.FR_STEER_ID]
    
    @classmethod
    def get_drive_ids(cls):
        # return [cls.FL_DRIVE_ID, cls.FR_DRIVE_ID, cls.RL_DRIVE_ID, cls.RR_DRIVE_ID]
        # 暂时只启用前两个驱动电机
        return [cls.FL_DRIVE_ID, cls.FR_DRIVE_ID]

    @classmethod
    def get_all_ids(cls):
        return cls.get_steer_ids() + cls.get_drive_ids()

    @classmethod
    def get_offset(cls, motor_id):
        if motor_id == cls.FL_STEER_ID:
            return cls.FL_STEER_OFFSET
        elif motor_id == cls.FR_STEER_ID:
            return cls.FR_STEER_OFFSET
        elif motor_id == cls.RL_STEER_ID:
            return cls.RL_STEER_OFFSET
        elif motor_id == cls.RR_STEER_ID:
            return cls.RR_STEER_OFFSET
        return 0.0

    # CAN 配置
    DRIVE_CAN_CHANNEL = 1 # 驱动电机 (CAN FD, 1M/4M)
    STEER_CAN_CHANNEL = 0 # 转向电机 (CAN FD, 1M/4M)
    
    # 驱动电机 CAN 参数 (CAN FD)
    DRIVE_BAUD_RATE = 1000000
    DRIVE_USE_CANFD = True
    DRIVE_DATA_BITRATE = 4000000
    
    # 转向电机 CAN 参数 (CAN FD)
    STEER_BAUD_RATE = 1000000
    STEER_USE_CANFD = True
    STEER_DATA_BITRATE = 4000000
    
    # CAN FD 特定参数 (保留旧兼容性，如果有其他地方用到)
    SAMPLE_POINT = 75.0
    DATA_SAMPLE_POINT = 80.0
    
    # 转向电机减速比 (电机转 8 圈 = 轮子转 1 圈)
    STEER_REDUCTION_RATIO = 20.0
    # 转向位置环 PID 参数
    STEER_KP = 20.0 
    STEER_KI = 0.05   # 新增: 积分系数
    STEER_KD = 0.1    # 新增: 微分系数
    STEER_MAX_I = 500.0 # 积分限幅 (RPM)

    # 驱动轮参数
    DRIVE_WHEEL_RADIUS = 0.085 # 米 (直径170mm)
    DRIVE_REDUCTION_RATIO = 1.0 # 假设为 1:1，如有减速箱请修改
    
    # 驱动电机参数限制
    DRIVE_MAX_RPM = 1500.0
    DRIVE_MIN_RPM = 300.0
    
    # 软件梯形加减速参数 (m/s^2)
    DRIVE_ACCEL = 0.5
    DRIVE_DECEL = 1.0
    
# 日志配置
logging.basicConfig(
    filename='motor.log',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)
logger = logging.getLogger(__name__)

from Motor_ctl import init_can_device as motor_ctl_init_can

class CustomVESC(VESC):
    """
    自定义 VESC 类，重写 receive_decode 以支持自定义 Status 2 协议
    Status 2 布局:
    - Byte 0-1: Encoder 1 (0-360)
    - Byte 2-3: Encoder 2 (0-360)
    - Byte 4-7: Motor Turns (int32)
    """
    def receive_decode(self, timeout=0.01):
        start_time = time.time()
        while True:
            # 计算剩余超时时间
            remaining = timeout - (time.time() - start_time)
            if remaining < 0:
                remaining = 0
                
            id, data = self.can_handle.receive(remaining)
            if id is None:
                return None, None

            status_id = (id >> 8) & 0xff
            vesc_id = id & 0xff
            
            # 全局调试打印：看看究竟收到了什么
            # 为了避免刷屏，只打印非 Status 1 的包，或者特定 ID 的包
            # print(f"RECV Raw: ID={hex(id)} (Status={hex(status_id)}, VESC={vesc_id})")
            
            # 性能优化：复用 self.can_packet 避免频繁创建对象
            decoded = False
            if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
                self.can_packet.rpm = int(buffer_get_float32(data, 1, 0))
                self.can_packet.current = buffer_get_float16(data, 1e2, 4)
                self.can_packet.pid_pos_now = buffer_get_float16(data, 50.0, 6)
                decoded = True
            elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_2:
                # --- 自定义 Status 2 解Enc1 Raw析 ---
                # Byte 0-1: Encoder 1 (0-360)
                # 假设为 uint16，范围 0-360，可能需要缩放？用户未指定缩放，暂按原始值或 1:1 处理
                # 如果是 0-360 对应 0-65535，则需要缩放。如果直接是角度整数，则直接读取。
                # 通常 VESC 协议 float16 是有缩放的。这里使用 buffer_get_int16 读取原始值。
                
                # Encoder 1 (High 2 bytes -> Index 0, 1)
                enc1_val = buffer_get_int16(data, 0)
                # 用户指定缩放因子: 50
                self.can_packet.enc1 = float(enc1_val) / 50.0

                # Encoder 2 (High 3-4 bytes -> Index 2, 3)
                enc2_val = buffer_get_int16(data, 2)
                self.can_packet.enc2 = float(enc2_val) / 50.0

                # Motor Turns (Low 4 bytes -> Index 4, 5, 6, 7)
                # int32
                turns_val = buffer_get_int32(data, 4)
                self.can_packet.motor_turns = turns_val
                
                # DEBUG PRINT: 打印原始数据
                # print(f"DEBUG: ID={id&0xFF} Status2 Data: {[hex(x) for x in data]}")
                # print(f"  -> Enc1 Raw: {enc1_val}, Enc2 Raw: {enc2_val}, Turns Raw: {turns_val}")
                
                decoded = True
            elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_3:
                self.can_packet.watt_hours = buffer_get_float32(data, 1e4, 0)
                self.can_packet.watt_hours_charged = buffer_get_float32(data, 1e4, 4)
                decoded = True
            elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_4:
                self.can_packet.temp_fet = buffer_get_float16(data, 1e1, 0)
                self.can_packet.temp_motor = buffer_get_float16(data, 1e1, 2)
                self.can_packet.tot_current_in = buffer_get_float16(data, 1e1, 4)
                self.can_packet.duty = buffer_get_float16(data, 1e3, 6)
                decoded = True
            elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_5:
                self.can_packet.tachometer_value = buffer_get_float32(data, 1, 0)
                self.can_packet.input_voltage = buffer_get_float16(data, 1e1, 4)
                decoded = True
            
            if decoded:
                return id, self.can_packet
            
            if remaining <= 0:
                return None, None

class VESCMonitor:
    def __init__(self, accel_time_ms=3500, decel_time_ms=2000, bus_drive=None, bus_steer=None):
        
        self.bus_drive = bus_drive
        self.bus_steer = bus_steer
        self.m_dev = None
        self.drive_ctl = None
        self.vesc = None
        self.vesc_drive = None
        self.adapter_steer = None
        self.adapter_drive = None
        
        # 如果外部传入了 bus 对象，则跳过内部初始化
        if self.bus_drive and self.bus_steer:
            print("VESCMonitor 使用外部传入的 CAN 总线。")
            # 这里不再调用 motor_ctl_init_can
            # m_dev 置空，表示非本类管理
            self.m_dev = None
        else:
            # 初始化 CAN 设备 (合并初始化 drive 和 steer 通道，以支持共享同一设备的通道)
            
            print(f"初始化 CAN 设备 (Drive: can{BasicConfig.DRIVE_CAN_CHANNEL}, Steer: can{BasicConfig.STEER_CAN_CHANNEL})...")
            
            # 构建通道特定配置
            # 注意：key 是 flat_idx (通常对应 0, 1...)
            channel_configs = {
                BasicConfig.DRIVE_CAN_CHANNEL: {
                    "arb_rate": BasicConfig.DRIVE_BAUD_RATE,
                    "data_rate": BasicConfig.DRIVE_DATA_BITRATE,
                    "fd": BasicConfig.DRIVE_USE_CANFD
                },
                BasicConfig.STEER_CAN_CHANNEL: {
                    "arb_rate": BasicConfig.STEER_BAUD_RATE,
                    "data_rate": BasicConfig.STEER_DATA_BITRATE,
                    "sp": BasicConfig.SAMPLE_POINT,
                    "dsp": BasicConfig.DATA_SAMPLE_POINT,
                    "fd": BasicConfig.STEER_USE_CANFD
                }
            }

            # 调用一次 init_can_device 同时初始化两个通道
            self.m_dev, self.bus_drive, self.bus_steer = motor_ctl_init_can(
                baud_rate=BasicConfig.DRIVE_BAUD_RATE, # 默认值
                dbit_baud_rate=BasicConfig.DRIVE_DATA_BITRATE, 
                channels=[BasicConfig.DRIVE_CAN_CHANNEL, BasicConfig.STEER_CAN_CHANNEL],
                can_type=1, # TYPE_CANFD
                fd=True, # 全局开启 FD 支持
                channel_configs=channel_configs
            )
            
            # 保持 m_dev_drive/steer 引用以便后续可能的独立引用 (虽然现在指向同一个 m_dev)
            self.m_dev_drive = self.m_dev
            self.m_dev_steer = self.m_dev

        # 检查 CAN 总线是否初始化成功
        if self.bus_drive is None:
            print(f"⚠️ 警告: 驱动电机 CAN 通道 (can{BasicConfig.DRIVE_CAN_CHANNEL}) 初始化失败或未连接。")
        else:
             print(f"✅ 驱动电机 CAN 就绪")
             
        if self.bus_steer is None:
            print(f"⚠️ 警告: 转向电机 CAN 通道 (can{BasicConfig.STEER_CAN_CHANNEL}) 初始化失败或未连接。")
        else:
             print(f"✅ 转向电机 CAN 就绪")

        # 创建 VESC 接口 (用于驱动电机 - can0)
        if self.bus_drive:
            self.tx_drive = TZCANTransmitter(self.bus_drive, channel_id=BasicConfig.DRIVE_CAN_CHANNEL)
            self.adapter_drive = self._TransmitterAdapter(self.tx_drive, BasicConfig.DRIVE_USE_CANFD)
            self.vesc_drive = VESC(self.adapter_drive)
        else:
            self.vesc_drive = None

        # 创建 VESC 接口 (用于转向电机 - can1)
        if self.bus_steer:
            # 在 Windows/Candle 多通道模式下，必须指定 channel_id
            self.tx_steer = TZCANTransmitter(self.bus_steer, channel_id=BasicConfig.STEER_CAN_CHANNEL)
            self.adapter_steer = self._TransmitterAdapter(self.tx_steer, BasicConfig.STEER_USE_CANFD)
            self.vesc = CustomVESC(self.adapter_steer)
        else:
            self.vesc = None

        self.drive_ctl = None
        
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
                "motor_angle_acc": 0.0,
                "hold_pid_pos": None
            } for mid in BasicConfig.get_all_ids()
        }
        
        self.target_freq = 200.0  # Hz
        
        # 转向目标角度 (Wheel Angle, degrees)
        self.steer_targets: Dict[int, float] = {}
        
        # 记录上电时的初始电机位置 (用于将当前位置作为0度)
        self.motor_initial_pos: Dict[int, float] = {}
        
        # 运行时校准参数 (允许外部覆盖 BasicConfig 中的默认值)
        # 格式: { mid: (zero_turns, zero_enc) }
        self.runtime_zero_params = BasicConfig.STEER_ZERO_PARAMS.copy()

    class _TransmitterAdapter:
        def __init__(self, transmitter, use_canfd):
            self.tx = transmitter
            self.use_canfd = use_canfd
            
        def send(self, id, data):
            self.tx._send_can_data(
                send_id=id,
                data_list=data,
                is_ext_frame=True,
                canfd_mode=self.use_canfd,
                brs=1 if self.use_canfd else 0,
                esi=0
            )
            
        def receive(self, timeout):
            result = self.tx._receive_can_data(
                target_id=None,
                timeout=timeout,
                canfd_mode=self.use_canfd,
                return_msg=True
            )
            if isinstance(result, tuple) and len(result) == 3:
                ok, data, msg = result
                if ok and msg:
                    return msg.arbitration_id, data
            return None, None

    def _update_angle(self, motor_id: int, packet):
        """
        基于自定义 Status 2 协议更新总角度。
        
        协议:
        - enc1: 编码器1 (0-360)
        - enc2: 编码器2 (0-360)
        - motor_turns: 电机圈数 (int32)
        
        计算:
        1. 获取当前绝对电机位置 (turns * 360 + enc)
        2. 获取零位绝对电机位置 (zero_turns * 360 + zero_enc)
        3. 差值 = 当前 - 零位
        4. 轮子角度 = 差值 / 减速比
        """
        if not hasattr(packet, 'motor_turns') or not hasattr(packet, 'enc1'):
            return

        state = self.motor_states[motor_id]
        
        turns = packet.motor_turns
        enc_angle = packet.enc1
        
        if motor_id not in self.runtime_zero_params:
            self.runtime_zero_params[motor_id] = (0, enc_angle)

        zero_turns, zero_enc = self.runtime_zero_params.get(motor_id, (0, 0.0))

        last_enc = state.get("last_pos")
        if last_enc is None:
            state["motor_angle_acc"] = enc_angle
        else:
            enc_delta = enc_angle - last_enc
            if enc_delta > 180.0:
                enc_delta -= 360.0
            elif enc_delta < -180.0:
                enc_delta += 360.0
            state["motor_angle_acc"] = state.get("motor_angle_acc", enc_angle) + enc_delta

        if BasicConfig.USE_TURNS_FOR_ANGLE:
            current_motor_abs = (turns * 360.0) + enc_angle
            zero_motor_abs = (zero_turns * 360.0) + zero_enc
        else:
            current_motor_abs = state.get("motor_angle_acc", enc_angle)
            zero_motor_abs = zero_enc

        delta_motor_angle = current_motor_abs - zero_motor_abs
        
        state["total_angle"] = delta_motor_angle / BasicConfig.STEER_REDUCTION_RATIO
        
        state["turns"] = turns
        state["last_pos"] = enc_angle
        state["motor_abs_pos"] = current_motor_abs
        
        # 保存编码器2数据供参考
        if hasattr(packet, 'enc2'):
             state["enc2"] = packet.enc2
        
        # 保存 enc1 供锁定使用
        if hasattr(packet, 'enc1'):
             state["enc1"] = packet.enc1

    def set_zero_calibration_params(self, motor_id: int, zero_turns: int, zero_enc: float):
        """
        外部接口：设置转向电机零位参数
        :param motor_id: 电机 CAN ID
        :param zero_turns: 零位时的圈数f
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

    def _control_steer_motor(self, motor_id: int, state: dict):
        """
        双环控制逻辑 (User Request):
        1. 外环 (Python): 位置环 PID -> 计算目标 RPM
           - 输入: 目标角度 (steer_targets) vs 真实角度 (state["total_angle"])
           - 输出: RPM 指令
        2. 内环 (VESC): 速度环 -> 执行 RPM
           - 使用 send_rpm 执行
        3. 锁定逻辑:
           - 当真实角度满足要求时，读取当前 enc1 值
           - 使用 send_pos(enc1) 锁定位置
        """
        if not self.vesc:
            return

        # 如果没有设定目标，默认锁死当前位置
        if motor_id not in self.steer_targets:
            return

        target_wheel_angle = self.steer_targets[motor_id]
        current_wheel_angle = state.get("total_angle", 0.0)
        
        # 1. 计算误差 (轮子角度)
        error_wheel_deg = target_wheel_angle - current_wheel_angle
        
        # 2. 转换为电机端误差 (用于 PID 计算参考)
        # 注意: 之前的 error_motor_deg 是用于 P 控制的，现在用于 PID
        error_motor_deg = error_wheel_deg * BasicConfig.STEER_REDUCTION_RATIO
        
        # 3. 判断是否满足锁定条件
        TOLERANCE_WHEEL_DEG = 0.2  # 轮子角度容差 (例如 0.2 度)
        
        if abs(error_wheel_deg) <= TOLERANCE_WHEEL_DEG:
            # --- 锁定模式 ---
            # 需求: 读取 enc1 的角度值, 使用 send_pos 下发 enc1
            # 注意: enc1 是 0-360 的原始值。如果 VESC 在 PID POS 模式下配置为使用该编码器且无多圈逻辑，这没问题。
            # 如果 VESC 配置了多圈 (累积角度)，直接发 0-360 可能会导致回转。
            # 这里按照用户指令执行，使用 enc1。
            
            curr_enc1 = state.get("enc1")
            
            if curr_enc1 is not None:
                # 只有当不在锁定状态或锁定目标改变时才发送(避免频繁发包?) 
                # 或者一直发送以保持锁定力? VESC send_pos 是设定目标，一直发也没事。
                self.vesc.send_pos(motor_id, curr_enc1)
                
                # 清除积分项，防止退出锁定后猛冲
                state["pid_integral"] = 0.0
                state["pid_err_prev"] = 0.0
                state["holding"] = True
            else:
                # 如果没有 enc1 数据，回退到发送 RPM 0
                self.vesc.send_rpm(motor_id, 0.0)
                
        else:
            # --- 逼近模式 (外环 PID -> RPM) ---
            state["holding"] = False
            
            # 初始化 PID 状态
            if "pid_err_prev" not in state:
                state["pid_err_prev"] = 0.0
                state["pid_integral"] = 0.0
            
            # PID 参数
            kp = BasicConfig.STEER_KP
            ki = BasicConfig.STEER_KI
            kd = BasicConfig.STEER_KD
            
            # 积分计算
            state["pid_integral"] += error_motor_deg
            # 积分限幅
            max_i = BasicConfig.STEER_MAX_I
            state["pid_integral"] = max(min(state["pid_integral"], max_i), -max_i)
            
            # 微分计算
            derivative = error_motor_deg - state["pid_err_prev"]
            state["pid_err_prev"] = error_motor_deg
            
            # 计算目标 RPM
            # RPM = Kp*Err + Ki*Int + Kd*Der
            rpm_target = (kp * error_motor_deg) + (ki * state["pid_integral"]) + (kd * derivative)
            
            # 最小启动力补偿 (克服静摩擦)
            MIN_ACTUATION_RPM = 300.0
            if abs(rpm_target) < MIN_ACTUATION_RPM and abs(rpm_target) > 1.0:
                rpm_target = math.copysign(MIN_ACTUATION_RPM, rpm_target)
            
            # 总幅值限制
            MAX_RPM = 3000.0
            rpm_target = max(min(rpm_target, MAX_RPM), -MAX_RPM)
            
            # 发送 RPM 指令
            self.vesc.send_rpm(motor_id, rpm_target)

    def _monitor_loop(self):
        last_control_time = time.time()
        CONTROL_INTERVAL = 0.02 # 50Hz Control Loop
        
        while self.running:
            # --- 1. 接收 CAN 消息 ---
            # 使用循环读取所有可用消息，避免缓冲区积压
            # 注意：不再在接收循环中直接调用控制逻辑，而是解耦
            if self.vesc:
                while self.running:
                    msg_id, packet = self.vesc.receive_decode(timeout=0)
                    
                    if msg_id is None:
                        break # 无新消息
                        
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
                                
                                # 处理转向电机的角度跟踪 (依赖 PID 位置)
                                # 旧逻辑：依赖 PID POS 和软件计算圈数
                                # if vesc_id in BasicConfig.get_steer_ids():
                                #     self._update_angle(vesc_id, state["pid_pos"])
                                pass
                                    
                            elif status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_2:
                                # 新逻辑：直接从 Status 2 读取圈数和编码器
                                if vesc_id in BasicConfig.get_steer_ids():
                                    self._update_angle(vesc_id, packet)
                                    
                            # 记录数据 (可选，避免日志过大可降频)
                            if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
                                log_msg = (f"[VESC] ID: {vesc_id} | 转速(RPM): {state['rpm']} | 电流: {state['current']} | "
                                            f"位置: {state['pid_pos']} | 角度: {state.get('total_angle', 0)}")
                                logger.info(log_msg)
                                # 实时打印供调试 (降频)
                                if not hasattr(self, 'print_counter'):
                                    self.print_counter = 0
                                self.print_counter += 1
                                if self.print_counter % 20 == 0: # 约 10Hz (取决于接收频率)
                                    print(f"ID: {vesc_id} | 圈数: {state['turns']} | 角度: {state.get('total_angle', 0):.2f} | 原始值: {state['pid_pos']:.2f}")

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
            time.sleep(0.001)
            
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.thread.start()
            
            # 启动时自动执行一次零位校准 (在新线程启动后稍等片刻以获取数据)
            threading.Timer(1.0, self.perform_zero_calibration).start()
            
            print("底盘监控已启动")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            
        if self.m_dev:
            # 只有当 m_dev 由本类创建时才执行关闭
            try:
                # 注意：motor_ctl_init_can 没有对应的 close 方法直接暴露在 Motor_ctl 模块？
                # 这里假设可以通过 TZCANTransmitter 关闭
                TZCANTransmitter.close_can_device(self.m_dev)
            except Exception as e:
                print(f"关闭 CAN 设备失败: {e}")
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
        self.vesc_drive = monitor.vesc_drive
        
        # 初始化 kinematics
        # 几何参数硬编码 (与 test_steer_control.py 保持一致)
        self.geometry = ChassisGeometry(length=0.25, width=0.354, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
        self.kinematics = FourWheelSteeringKinematics(self.geometry)
        
        # 初始化驱动电机（如果存在）
        if self.vesc_drive:
            print("驱动电机 VESC 已连接")
            
        # 驱动速度状态 (用于软件加减速斜坡)
        # 格式: { drive_id: current_rpm }
        self.current_drive_rpms = {
            BasicConfig.FL_DRIVE_ID: 0.0,
            BasicConfig.FR_DRIVE_ID: 0.0
        }
        self.last_drive_update_time = time.time()

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
        校准归位：将所有转向电机转动到逻辑 0 度位置（正前方）。
        这会应用 BasicConfig 中的 OFFSET 参数。
        注意：目前已禁用开机自动校准，仅作为手动调用接口。
        """
        print("⚠️ 归位校准已暂时禁用 (Software Position Control Mode)")
        pass

    def set_accel_decel(self, accel_mps2: float, decel_mps2: float = None):
        """
        设置软件加减速参数 (m/s^2)
        """
        if decel_mps2 is None:
            decel_mps2 = accel_mps2
        
        BasicConfig.DRIVE_ACCEL = abs(accel_mps2)
        BasicConfig.DRIVE_DECEL = abs(decel_mps2)
        print(f"🔄 更新软件加减速: Accel={BasicConfig.DRIVE_ACCEL:.2f}, Decel={BasicConfig.DRIVE_DECEL:.2f}")

    def apply_kinematics(self, wheel_states: Dict[str, Tuple[float, float]]):
        """
        应用运动学计算结果到电机 (包含软件梯形加减速)。
        :param wheel_states: {wheel_name: (target_speed_mps, angle_rad)}
        """
        now = time.time()
        dt = now - self.last_drive_update_time
        self.last_drive_update_time = now
        
        # 防止 dt 过大 (如调试中断) 导致速度跳变
        if dt > 0.1: dt = 0.1

        # 映射名称到 ID
        name_map = {
            "FL": (BasicConfig.FL_STEER_ID, BasicConfig.FL_DRIVE_ID),
            "FR": (BasicConfig.FR_STEER_ID, BasicConfig.FR_DRIVE_ID),
            # "RL": (BasicConfig.RL_STEER_ID, BasicConfig.RL_DRIVE_ID),
            # "RR": (BasicConfig.RR_STEER_ID, BasicConfig.RR_DRIVE_ID)
        }
        
        drive_speeds = {} # 驱动电机ID -> 转速(RPM)
        
        for name, (steer_id, drive_id) in name_map.items():
            if name not in wheel_states:
                continue
                
            target_speed, target_angle_rad = wheel_states[name]
            
            # 1. 角度优化 (舵轮优化)
            # 将目标角度转换为度
            target_angle_deg = math.degrees(target_angle_rad)
            
            # 获取当前逻辑角度 (从 monitor 获取)
            # 注意: monitor 只有 motor_states (PID Pos -> Total Angle)
            # 我们需要当前的逻辑转向角度
            current_state = self.monitor.get_state(steer_id)
            current_angle = current_state.get("total_angle", 0.0)
            
            # 优化逻辑: 寻找最近的等效角度 (+/- 180 度翻转速度)
            # 归一化误差到 -180 ~ 180
            diff = (target_angle_deg - current_angle + 180) % 360 - 180
            
            final_angle = current_angle + diff
            final_speed = target_speed
            
            # 如果误差超过 90 度，则反转轮子和速度
            # 仅当目标速度不为 0 时执行此优化。如果速度为 0 (如停止/归位)，则强制转到目标角度 (如 0 度)
            if abs(diff) > 90 and abs(target_speed) > 1e-3:
                final_angle = current_angle + diff - 180 * (1 if diff > 0 else -1)
                final_speed = -target_speed
            else:
                pass

            # 2. 发送转向指令
            self._send_steer_pos(steer_id, final_angle)
            
            # 记录目标角度以便检查是否到位
            # 我们需要检查所有轮子是否都到位
            
            # 3. 计算目标 RPM
            # 转速(RPM) = (线速度 / (2 * pi * 半径)) * 60 * 减速比
            target_rpm = (final_speed / (2 * math.pi * BasicConfig.DRIVE_WHEEL_RADIUS)) * 60 * BasicConfig.DRIVE_REDUCTION_RATIO
            
            # --- 软件梯形加减速逻辑 (修订版) ---
            # 需求:
            # 1. 正负反转时：直接重置为0，再加速 (不走 +1000 -> 0 -> -1000 的漫长过程)
            # 2. 停止时：直接下发0 (不走减速斜坡)
            # 3. 仅在同向加速时使用斜坡
            
            # 获取当前内部状态 RPM
            current_rpm = self.current_drive_rpms.get(drive_id, 0.0)
            
            # Case 1: 反向 (符号相反) -> 立即重置为 0 (准备在下一帧开始加速)
            # 注意: 如果 current_rpm 也是 0，不属于反向，属于正常起步
            # 只有当两者都不为0且符号相反时才触发
            if (abs(target_rpm) > 1e-3 and abs(current_rpm) > 1e-3 and target_rpm * current_rpm < 0):
                next_rpm = 0.0
                
            # Case 2: 同向或从0开始或停车 -> 应用加减速斜坡
            else:
                # 计算 RPM 变化率限制 (RPM/s)
                rpm_per_mps = (1.0 / (2 * math.pi * BasicConfig.DRIVE_WHEEL_RADIUS)) * 60 * BasicConfig.DRIVE_REDUCTION_RATIO
                max_accel_rpm_per_sec = BasicConfig.DRIVE_ACCEL * rpm_per_mps
                max_decel_rpm_per_sec = BasicConfig.DRIVE_DECEL * rpm_per_mps
                
                rpm_diff = target_rpm - current_rpm
                
                # 判断加速还是减速
                # 逻辑: 如果 abs(target) > abs(current) 则是加速过程
                #       如果 abs(target) < abs(current) 则是减速过程 (包括减速到0)
                if abs(target_rpm) > abs(current_rpm):
                    limit_rate = max_accel_rpm_per_sec
                else:
                    limit_rate = max_decel_rpm_per_sec
                
                max_delta = limit_rate * dt
                
                if abs(rpm_diff) > max_delta:
                    change = math.copysign(max_delta, rpm_diff)
                    next_rpm = current_rpm + change
                else:
                    next_rpm = target_rpm
            
            # 更新状态
            self.current_drive_rpms[drive_id] = next_rpm
            rpm = next_rpm
            
            # 限制最小/最大 RPM (输出阶段)
            if abs(rpm) > 1e-3: 
                # 只有当目标不为0时才维持最小转速，否则允许归零
                # 这是一个关键点：如果正在减速停车(target=0)，不要强制维持 300RPM，否则永远停不下来
                if abs(rpm) < BasicConfig.DRIVE_MIN_RPM and abs(target_rpm) > 1e-3: 
                     rpm = math.copysign(BasicConfig.DRIVE_MIN_RPM, rpm)
                elif abs(rpm) > BasicConfig.DRIVE_MAX_RPM:
                    rpm = math.copysign(BasicConfig.DRIVE_MAX_RPM, rpm)
            
            # 如果计算结果已经很小，且目标也是0，置0 (防止最后一点点拖尾)
            if abs(rpm) < 10.0 and abs(target_rpm) < 1e-3:
                rpm = 0.0

            # 打印调试信息
            # 过滤条件: 目标与当前不同，且变化量大于一定值
            if abs(target_rpm - current_rpm) > 50.0 and steer_id == BasicConfig.FL_STEER_ID:
                 print(f"[Drive Ramp] Tgt: {target_rpm:.0f} | Cur: {current_rpm:.0f} -> Next: {rpm:.0f} | dt: {dt*1000:.0f}ms")


            final_rpm = rpm
            if drive_id == BasicConfig.FR_DRIVE_ID: # 右前轮
                 final_rpm = -rpm
            
            drive_speeds[drive_id] = final_rpm
        
        # --- 检查转向是否到位 ---
        # 仅当有驱动速度且不是停止状态时才检查
        # 修改为非阻塞逻辑：如果未到位，则暂时不发送驱动速度 (Speed=0)，但允许函数返回
        
        has_speed = self.vesc_drive and any(abs(s) > 10.0 for s in drive_speeds.values()) # 这里的阈值是RPM
        if has_speed:
            # 检查是否已经到位
            needs_wait = False
            wait_reason = ""
            for steer_id in BasicConfig.get_steer_ids():
                current_state = self.monitor.get_state(steer_id)
                
                # 忽略未连接的电机 (last_pos 为 None 表示从未收到过数据)
                if current_state.get("last_pos") is None:
                    continue
                    
                target = self.monitor.steer_targets.get(steer_id, 0.0)
                current_angle = current_state.get("total_angle", 0.0)
                
                if abs(target - current_angle) > 5.0:
                    needs_wait = True
                    wait_reason = f"ID {steer_id} (T:{target:.1f}, C:{current_angle:.1f})"
                    break

            if needs_wait:
                # 尚未到位，抑制驱动速度
                if not hasattr(self, 'last_wait_print'):
                    self.last_wait_print = 0
                if time.time() - self.last_wait_print > 0.5:
                    print(f"⏳ 转向未到位，暂停驱动... {wait_reason}")
                    self.last_wait_print = time.time()
                    
                for drive_id in drive_speeds:
                    drive_speeds[drive_id] = 0.0
                    self.current_drive_rpms[drive_id] = 0.0 # 重置斜坡

        
        # 4. 发送驱动指令 (VESC)
        if self.vesc_drive:
            for drive_id, rpm in drive_speeds.items():
                try:
                    self.vesc_drive.send_rpm(drive_id, rpm)
                except Exception as e:
                    print(f"驱动 VESC 指令错误: {e}")

    def spin_left(self, speed: float = 0.5):
        """
        原地左旋（逆时针）。
        通过运动学计算四轮角度，实现阿克曼几何的中心旋转。
        :param speed: 线速度 m/s (轮子切向速度)
        """
        # print(f"执行左旋 (Kinematics) Speed={speed} m/s...")

        # 计算对应的 Omega
        # V = Omega * R (R is distance from center to wheel)
        radius = math.hypot(self.geometry.L/2, self.geometry.W/2)
        if radius < 1e-4:
            omega = 0
        else:
            omega = speed / radius
            
        # 调用逆运动学 (左旋: Omega > 0)
        wheel_states = self.kinematics.inverse_kinematics(0.0, 0.0, omega)
        
        self.apply_kinematics(wheel_states)
        
    def spin_right(self, speed: float = 0.5):
        """
        原地右旋（顺时针）。
        通过运动学计算四轮角度，实现阿克曼几何的中心旋转。
        :param speed: 线速度 m/s (轮子切向速度)
        """
        # print(f"执行右旋 (Kinematics) Speed={speed} m/s...")

        # 计算对应的 Omega
        radius = math.hypot(self.geometry.L/2, self.geometry.W/2)
        if radius < 1e-4:
            omega = 0
        else:
            omega = speed / radius
            
        # 调用逆运动学 (右旋: Omega < 0)
        wheel_states = self.kinematics.inverse_kinematics(0.0, 0.0, -omega)
        
        self.apply_kinematics(wheel_states)

    def stop(self):
        # 停止 VESC 转向 (可选，通常保持位置)
        # for mid in [BasicConfig.FL_DRIVE_ID, BasicConfig.FR_DRIVE_ID]:
        #     self.vesc.send_rpm(mid, 0)
            
        # 停止 VESC 驱动
        if self.vesc_drive:
            for drive_id in BasicConfig.get_drive_ids():
                try:
                    self.vesc_drive.send_rpm(drive_id, 0)
                except Exception:
                    pass

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
