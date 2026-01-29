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
    FL_STEER_ID = 39  # 左前转向电机
    FR_STEER_ID = 38  # 右前转向电机
    RL_STEER_ID = 105  # 左后转向电机
    RR_STEER_ID = 106  # 右后转向电机

    FL_DRIVE_ID = 32  # 左前轮毂电机
    FR_DRIVE_ID = 34  # 右前轮毂电机
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
    STEER_ZERO_PARAMS = {
        39: (0, 75.74), 
        38: (0, 19.20) 

    }
    
    # 是否使用上电时的当前位置作为零点
    # True: 上电时将当前位置记为 0 度 (忽略 STEER_ZERO_PARAMS)
    # False: 使用 STEER_ZERO_PARAMS 作为绝对零点
    USE_CURRENT_AS_ZERO = False

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
    ENABLE_DRIVE = True # 屏蔽驱动电机控制
    DRIVE_CAN_CHANNEL = 0 # 驱动电机 (CAN 2.0, 500k)
    STEER_CAN_CHANNEL = 1 # 转向电机 (CAN FD, 1M/4M)
    
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

    STEER_INVERTED_IDS = [38]

    # 转向电机限位保护
    # 当目标角度在死区范围内时，自动翻转驱动方向
    # 格式: (MIN_ANGLE, MAX_ANGLE) 
    # 例如 (90, 270) 表示如果不方便转到 180 度，就转到 0 度并反转驱动
    STEER_ANGLE_LIMITS = None # 暂时禁用，全向转向

    # 转向位置环 PID 参数 (简单 P 控制)
    STEER_KP = 35.0 # 误差 1 度 (Motor) -> 50 RPM (Increased from 27.0)
    # 轮子误差 1 度 -> 电机误差 20 度 -> 输出 1000 RPM
    
    # 转向角度容差 (度)
    # 当轮子角度误差小于此值时，不再进行 PID 调整，而是锁定位置
    STEER_ANGLE_TOLERANCE = 0.5


    # 驱动轮参数
    DRIVE_WHEEL_RADIUS = 0.85 # 米
    DRIVE_REDUCTION_RATIO = 1.0 # 假设为 1:1，如有减速箱请修改
    DRIVE_POLE_PAIRS = 15 # 极对数
    
    # 驱动电机最大参考转速 (用于计算加减速时间)
    # 假设 1000 RPM 对应满速控制量
    MAX_RPM_REF = 1500.0
    
    @staticmethod
    def calc_accel_time_ms(accel_mps2: float) -> int:
        """
        根据目标加速度 (m/s^2) 计算驱动电机所需的时间参数 (ms)
        计算基准：从 0 加速到 MAX_RPM_REF 所需的时间
        """
        if accel_mps2 <= 0.01:
            accel_mps2 = 0.01 # 防止除零
            
        # 1. 计算参考最大线速度
        # V = (RPM / 60) * 2 * pi * R
        physical_max_v = (BasicConfig.MAX_RPM_REF / 60.0) * (2 * math.pi * BasicConfig.DRIVE_WHEEL_RADIUS)
        
        # 2. 计算时间 t = v / a
        time_ms = (physical_max_v / accel_mps2) * 1000.0
        
        return int(time_ms)


# 日志配置
logging.basicConfig(
    filename='motor.log',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)
logger = logging.getLogger(__name__)

from Motor_ctl import Motor_CTL, init_can_device as motor_ctl_init_can

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

            # 如果配置屏蔽了驱动电机，则强制置空 bus_drive，避免后续初始化
            if not BasicConfig.ENABLE_DRIVE:
                self.bus_drive = None
                print("🚫 驱动电机已通过配置禁用 (ENABLE_DRIVE=False)")
            
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

        # 创建 VESC 接口 (用于转向电机 - can1)
        if self.bus_steer:
            # 在 Windows/Candle 多通道模式下，必须指定 channel_id
            self.tx_steer = TZCANTransmitter(self.bus_steer, channel_id=BasicConfig.STEER_CAN_CHANNEL)
            self.adapter_steer = self._TransmitterAdapter(self.tx_steer, BasicConfig.STEER_USE_CANFD)
            self.vesc = CustomVESC(self.adapter_steer)
        else:
            self.vesc = None
        
        # 创建 VESC 接口 (用于驱动电机 - can0)
        # 注意：这里我们改用 VESC 协议控制驱动电机，替换原有的 CANOpen Motor_CTL
        if self.bus_drive:
            self.tx_drive = TZCANTransmitter(self.bus_drive, channel_id=BasicConfig.DRIVE_CAN_CHANNEL)
            self.adapter_drive = self._TransmitterAdapter(self.tx_drive, BasicConfig.DRIVE_USE_CANFD)
            self.vesc_drive = VESC(self.adapter_drive)
        else:
            self.vesc_drive = None
            
        # 废弃原有的 CANOpen Motor_CTL
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
                "enc2": None,
                "last_raw_enc": None,
                "software_turns": 0,
                "initial_enc": None,
                "motor_abs_pos": 0.0,
                "enc2_zero_locked": False
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
        
        # 记录已初始化零点的电机 ID (用于 USE_CURRENT_AS_ZERO 模式)
        self.zero_initialized = set()
        
        if BasicConfig.USE_CURRENT_AS_ZERO:
            print("配置为: 使用当前位置作为零点 (忽略预设参数)")
            # 清空预设参数，等待首次数据到来时捕获
            self.runtime_zero_params.clear()

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
        
        修改后逻辑 (软件计算圈数):
        1. 读取 Enc1 (0-360)
        2. 比较本次 Enc1 与上次 Enc1，检测过零点 (从 360->0 或 0->360)
        3. 累计软件圈数 (software_turns)
        4. 计算绝对电机位置: software_turns * 360 + enc1 - initial_offset
        5. 计算轮子角度: 绝对位置 / 减速比
        """
        if not hasattr(packet, 'enc1'):
            return

        state = self.motor_states[motor_id]
        current_enc = packet.enc1 # 当前原始角度 (0-360)
        current_enc2 = packet.enc2 if hasattr(packet, 'enc2') else None
        
        if not BasicConfig.USE_CURRENT_AS_ZERO:
            zero_params = self.runtime_zero_params.get(motor_id)
            if not state.get("enc2_zero_locked") and current_enc2 is not None and zero_params is not None:
                _, zero_enc = zero_params
                # Enc2 is 1:1 with Wheel (Absolute)
                # Calculate deviation from zero in WHEEL DEGREES
                delta_wheel = ((current_enc2 - zero_enc + 180) % 360) - 180
                
                state["total_angle"] = delta_wheel
                state["turns"] = 0
                state["last_pos"] = current_enc
                # Motor position is Wheel * Reduction
                state["motor_abs_pos"] = delta_wheel * BasicConfig.STEER_REDUCTION_RATIO
                state["enc2"] = current_enc2
                
                # Check tolerance (using Wheel Tolerance directly: 0.5 deg)
                if abs(delta_wheel) <= BasicConfig.STEER_ANGLE_TOLERANCE:
                    state["enc2_zero_locked"] = True
                    state["initial_enc"] = current_enc
                    state["last_raw_enc"] = current_enc
                    state["software_turns"] = 0
                    print(f"✅ 电机 {motor_id} 零位已锁定 (Err={delta_wheel:.2f}°)")
                    self.runtime_zero_params[motor_id] = (0, current_enc)
                return
        
        # 初始化: 如果是第一次收到数据
        if state.get("last_raw_enc") is None:
            state["last_raw_enc"] = current_enc
            state["software_turns"] = 0
            if state.get("initial_enc") is None:
                state["initial_enc"] = current_enc
            
            # 如果启用了 USE_CURRENT_AS_ZERO，其实就是初始偏移为 0 (相对于初始位置)
            # 这里的逻辑是：我们将 "上电时刻" 定义为 0 度
            # 所以绝对位置 = (turns * 360 + enc) - initial_enc
            print(f"✅ [初始化] 电机 {motor_id} 初始位置: {current_enc:.2f}")
            
        # --- 圈数过零检测 ---
        last_enc = state["last_raw_enc"]
        diff = current_enc - last_enc
        
        # 阈值检测: 如果跳变超过 180 度，认为是过零
        # 例如: 上次 350，这次 10 -> 差值 -340 -> 实际是正向过了零点 -> 圈数 +1
        # 例如: 上次 10，这次 350 -> 差值 +340 -> 实际是反向过了零点 -> 圈数 -1
        if diff < -180:
            state["software_turns"] += 1
        elif diff > 180:
            state["software_turns"] -= 1
            
        # 更新上次值
        state["last_raw_enc"] = current_enc
        
        # --- 计算绝对位置 (相对于上电位置) ---
        # 绝对角度 = (圈数 * 360 + 当前角度) - 初始角度
        turns = state["software_turns"]
        initial_enc = state["initial_enc"]
        
        # 当前累积的绝对角度 (度)
        # 注意：这里我们不需要 VESC 发来的 motor_turns，完全自己计算
        current_abs_angle = (turns * 360.0) + current_enc - initial_enc
        
        # --- 计算轮子角度 ---
        # 轮子角度 = 电机绝对角度 / 减速比 (限幅到 0-360)
        state["total_angle"] = (current_abs_angle / BasicConfig.STEER_REDUCTION_RATIO) % 360.0
        
        # 更新状态显示
        state["turns"] = turns # 显示软件计算的圈数
        state["last_pos"] = current_enc # 显示当前原始角度
        state["motor_abs_pos"] = current_abs_angle
        
        # 保存编码器2数据供参考
        if hasattr(packet, 'enc2'):
             state["enc2"] = packet.enc2

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

    def _control_steer_motor(self, motor_id: int, state: dict):
        """
        转向电机闭环控制逻辑 (由 Monitor 线程调用)
        """
        # print(f"DEBUG: Control loop for {motor_id}") # 暂时调试
        if not self.vesc:
            return

        # 如果没有设定目标，默认锁死当前位置 (使用 PID 位置保持)
        if motor_id not in self.steer_targets:
            # 保持当前位置不动
            # self.vesc.send_pos(motor_id, state["pid_pos"]) 
            return

        target_wheel_angle = self.steer_targets[motor_id]
        
        # 获取当前轮子角度 (已在 _update_angle 中基于零位参数计算好)
        current_wheel_angle = state.get("total_angle", 0.0)
        
        # 误差 (轮子角度)
        error_wheel_deg = target_wheel_angle - current_wheel_angle
        # 归一化误差到 -180 ~ 180 (最短路径)
        error_wheel_deg = (error_wheel_deg + 180) % 360 - 180
        
        # 转换为电机误差 RPM
        # 误差 1 度 (Wheel) -> 误差 8 度 (Motor) -> RPM?
        # 简单的 P 控制: RPM = Kp * Error_Wheel
        # 之前 Kp=12.5 是针对电机角度误差。
        # 现在 error 是轮子角度，需要先转为电机角度误差，或者调整 Kp
        
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
            MAX_RPM = 5000.0
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
            
            self.vesc.send_rpm(motor_id, rpm_target)
        else:
            # 位置锁定模式
            # 当误差很小时，为了锁住位置，发送当前 PID 位置 (0-360) 作为目标
            # 注意：send_pos 接收的是 PID 角度 (0-360)，用于 VESC 内部的位置闭环
            # 这里的逻辑是让 VESC 锁死在当前物理位置
            # 修正：由于我们使用 Status 2 的 Enc1 作为位置源，而 Status 1 的 PID POS 可能无效
            # 所以这里必须使用 state["last_pos"] (即 Status 2 的 enc1)
            # 前提是 last_pos 有效
            if state.get("last_pos") is not None:
                self.vesc.send_pos(motor_id, state["last_pos"])
            else:
                if motor_id == 38:
                    # print(f"DEBUG: Cannot lock {motor_id}: No last_pos")
                    pass
                # 兜底：如果没有 Status 2 数据，暂时不发或尝试发 0 (风险)
                pass
            
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
                    
                    # 计算线速度 (m/s)
                    # Speed = (ERPM / PolePairs) * 2 * pi * R / 60
                    erpm = state["rpm"]
                    speed_mps = (erpm / BasicConfig.DRIVE_POLE_PAIRS) * (2 * math.pi * BasicConfig.DRIVE_WHEEL_RADIUS) / 60.0
                    state["speed"] = speed_mps
                    
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
                    log_msg = (f"[VESC] ID: {vesc_id} | 转速(RPM): {state['rpm']} | 速度: {state.get('speed', 0):.2f}m/s | 电流: {state['current']} | "
                                f"位置: {state['pid_pos']} | 角度: {state.get('total_angle', 0)}")
                    logger.info(log_msg)
                    
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
                        current_info = f"当前: Angle={state.get('total_angle', 0):.2f}, Turns={state.get('turns', 0)}, Raw={raw_val:.2f}"
                        
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
        timeout = 10.0 # 最多等待 10 秒
        start_time = time.time()
        
        while self.running:
            if time.time() - start_time > timeout:
                print("⚠️ 自动回零超时：未收到电机数据")
                return
                
            # 检查是否所有转向电机都有数据 (last_pos 不为 None)
            all_ready = True
            for mid in BasicConfig.get_steer_ids():
                state = self.motor_states.get(mid)
                if not state or state.get("last_pos") is None:
                    all_ready = False
                    break
            
            if all_ready:
                print("✅ 检测到电机数据，执行自动回零...")
                # 稍微延迟一下确保数据稳定
                time.sleep(0.5)
                self.perform_zero_calibration()
                return
                
            time.sleep(0.5)

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
        self.vesc_drive = monitor.vesc_drive # 获取驱动电机控制器 (VESC)
        self.drive_ctl = None # 废弃旧的 drive_ctl
        
        # 初始化 kinematics
        # 几何参数硬编码 (与 test_steer_control.py 保持一致)
        self.geometry = ChassisGeometry(length=0.25, width=0.354, wheel_radius=BasicConfig.DRIVE_WHEEL_RADIUS)
        self.kinematics = FourWheelSteeringKinematics(self.geometry)
        
        # 初始化驱动电机（如果存在）
        if self.vesc_drive:
            # VESC 不需要复杂的初始化序列，只需确保连接即可
            print("驱动电机控制器 (VESC) 已连接")
            pass 
            
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

    def set_accel_decel(self, accel_mps2: float, decel_mps2: float = None):
        """
        动态设置驱动电机加速度和减速度 (单位: m/s^2)
        对于 VESC，这通常意味着调整占空比斜率或电流斜率，
        但标准的 send_rpm 并没有直接的加速度参数。
        
        如果需要平滑控制，应该在软件层做 ramp 处理。
        目前暂时只打印日志，不做硬件设置。
        """
        if not self.vesc_drive:
            print("⚠️ 驱动控制器未连接，无法设置加速度")
            return

        if decel_mps2 is None:
            decel_mps2 = accel_mps2
            
        print(f"🔄 (VESC模式) 设置软加减速目标: Accel={accel_mps2:.2f} m/s^2, Decel={decel_mps2:.2f} m/s^2")
        # TODO: 在 _monitor_loop 或其他地方实现软加减速逻辑
        # 目前 VESC 默认响应较快，由 VESC 内部参数决定物理极限

    def apply_kinematics(self, wheel_states: Dict[str, Tuple[float, float]]):
        """
        应用运动学计算结果到电机。
        :param wheel_states: {wheel_name: (speed_mps, angle_rad)}
        """
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
            
            # --- 新增: 特定 ID (38) 的镜像对称处理 / 强制前向逻辑 ---
            # 用户反馈 ID38 在前进时停在 180 度。这通常是因为最短路径优化导致的。
            # 强制 ID38 偏好 "前向半圆" ([-90, 90] 即 [270, 360] U [0, 90])
            # 如果角度落在 (90, 270) 范围内 (即后向半圆)，则翻转 180 度
            # 注意：即便是停车状态(speed=0)，如果用户希望回正，也应该应用此逻辑。
            if steer_id == 38:
                norm_angle = final_angle % 360.0
                # 范围 (90, 270) 对应后方。包含 180。
                if 90.0 < norm_angle < 270.0:
                    final_angle += 180.0
                    final_speed = -final_speed
                    # print(f"[DEBUG ID38] Flip 180: {norm_angle:.2f} -> {final_angle % 360.0:.2f}")
            
            # 2. 发送转向指令
            self._send_steer_pos(steer_id, final_angle)
            
            # 记录目标角度以便检查是否到位
            # 我们需要检查所有轮子是否都到位
            
            # 3. 计算驱动 RPM (无论是否连接驱动电机都计算，方便调试)
            # 转速(RPM) = (线速度 / (2 * pi * 半径)) * 60 * 减速比 * 极对数 (转换为 ERPM)
            # 注意: Speed 单位 m/s
            
            rpm = (final_speed / (2 * math.pi * BasicConfig.DRIVE_WHEEL_RADIUS)) * 60 * BasicConfig.DRIVE_REDUCTION_RATIO * BasicConfig.DRIVE_POLE_PAIRS
            
            # 打印调试信息 (仅在有速度时打印，避免刷屏)
            if abs(rpm) > 1.0 and steer_id == BasicConfig.FL_STEER_ID:
                 print(f"[调试] 速度: {final_speed:.2f} m/s -> 转速: {rpm:.2f} RPM")

            # 无论使用哪种控制器，都先计算目标转速并存入 drive_speeds
            final_rpm = rpm
            if drive_id == BasicConfig.FR_DRIVE_ID: # 右前轮
                 final_rpm = -rpm
            
            drive_speeds[drive_id] = final_rpm
        
        # --- 检查转向是否到位 ---
        # 仅当有驱动速度且不是停止状态时才检查
        # 修改为非阻塞逻辑：如果未到位，则暂时不发送驱动速度 (Speed=0)，但允许函数返回
        
        # [用户需求] 规避转向运动时无法控制驱动电机的问题
        # 原始逻辑：如果舵角误差 > 5度，强制将 drive_speeds 置零 (暂停驱动)
        # 修改逻辑：允许行进间转向 (Running Steering)，取消强制暂停。
        # 仅在原地转向模式(Spin Turn)或大角度剧烈变化时可能需要等待，但在行进间微调(W+A/D)时不应暂停。
        
        # 识别是否是行进间微调模式？
        # 如果当前已经有一定速度，且舵角变化不大，则不暂停。
        # 这里简单起见，直接放宽限制，允许同时控制。
        # has_speed = any(abs(s) > 10.0 for s in drive_speeds.values()) # 这里的阈值是RPM
        # if has_speed:
        #     # 检查是否已经到位
        #     needs_wait = False
        #     for steer_id in BasicConfig.get_steer_ids():
        #         target = self.monitor.steer_targets.get(steer_id, 0.0)
        #         current_state = self.monitor.get_state(steer_id)
        #         current_angle = current_state.get("total_angle", 0.0)
        #         if abs(target - current_angle) > 5.0:
        #             needs_wait = True
        #             break
        #
        #     if needs_wait:
        #         # 尚未到位，抑制驱动速度
        #         print("⏳ 转向中，暂停驱动...")
        #         for drive_id in drive_speeds:
        #             drive_speeds[drive_id] = 0.0
        
        # 4. 发送驱动指令 (改为 VESC 逐个发送)
        if self.vesc_drive:
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

                self.vesc_drive.send_rpm(drive_id, rpm)
                print(f"Drive VESC ID {drive_id} -> {rpm:.1f} RPM")
        else:
             print("⚠️ self.vesc_drive is None! 无法发送驱动指令")

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
            for mid in BasicConfig.get_drive_ids():
                self.vesc_drive.send_rpm(mid, 0)
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
