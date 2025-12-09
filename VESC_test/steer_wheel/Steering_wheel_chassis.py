import sys
import os
import time
import threading
import logging
import math
from typing import List, Dict, Optional

# 添加父目录到 path 以查找 CANMessageTransmitter
current_dir = os.path.dirname(os.path.abspath(__file__))
vesc_test_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(vesc_test_dir)

if project_root not in sys.path:
    sys.path.append(project_root)
if vesc_test_dir not in sys.path:
    sys.path.append(vesc_test_dir)

try:
    from CANMessageTransmitter import CANMessageTransmitter
    # 尝试通过 CANMessageTransmitter 选择设备，或者直接导入
    # 注意：CANMessageTransmitter.choose_can_device("TZCAN") 返回的是类
    TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
    from can_vesc import VESC, VESC_CAN_STATUS
except ImportError as e:
    print(f"Import Error: {e}")
    # 备用：如果上面的路径添加失败，尝试相对导入或其他方式
    try:
        sys.path.append("..")
        from CANMessageTransmitter import CANMessageTransmitter
        TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
    except ImportError:
         # 如果还不行，尝试直接导入（保留旧兼容性）
        try:
            from TZCANTransmitter import TZCANTransmitter
        except ImportError:
            pass
    
    try:
        from VESC_test.can_vesc import VESC, VESC_CAN_STATUS
    except ImportError:
        pass

# --- 配置类 ---
class BasicConfig:
    # VESC ID 配置
    FL_STEER_ID = 45  # 左前转向电机
    FR_STEER_ID = 46  # 右前转向电机
    RL_STEER_ID = 105  # 左后转向电机
    RR_STEER_ID = 106  # 右后转向电机

    FL_DRIVE_ID = 103  # 左前轮毂电机
    FR_DRIVE_ID = 104  # 右前轮毂电机
    RL_DRIVE_ID = 107  # 左后轮毂电机
    RR_DRIVE_ID = 108  # 右后轮毂电机

    # 转向电机零位偏置校准 (单位: 度)
    # 请在此处填写【当轮子物理朝向正前方时，读取到的编码器角度值】
    FL_STEER_OFFSET = 0
    FR_STEER_OFFSET = 0
    RL_STEER_OFFSET = 0.0
    RR_STEER_OFFSET = 0.0

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
    DRIVE_CAN_CHANNEL = 0 # 驱动电机 (CAN 2.0, 500k)
    STEER_CAN_CHANNEL = 1 # 转向电机 (CAN FD, 1M/4M)
    
    # 驱动电机 CAN 参数 (CAN 2.0)
    DRIVE_BAUD_RATE = 500000
    DRIVE_USE_CANFD = True # 修改为 True，因为 Robotchassis.py 初始化使用了 TYPE_CANFD
    
    # 转向电机 CAN 参数 (CAN FD)
    STEER_BAUD_RATE = 1000000
    STEER_USE_CANFD = True
    STEER_DATA_BITRATE = 4000000
    
    # CAN FD 特定参数 (保留旧兼容性，如果有其他地方用到)
    SAMPLE_POINT = 75.0
    DATA_SAMPLE_POINT = 80.0
    
    # 转向电机减速比 (电机转 8 圈 = 轮子转 1 圈)
    STEER_REDUCTION_RATIO = 8.0
    # 转向位置环 PID 参数 (简单 P 控制)
    STEER_KP = 20.0 # 误差 1 度 (Motor) -> 20 RPM
    
    # 是否使用固定 RPM 逻辑 (Bang-Bang / 阶梯)
    # 用户要求：重新启用 PID 控制，不使用固定 RPM
    USE_FIXED_RPM_LOGIC = False
    
    # 以下参数仅在 USE_FIXED_RPM_LOGIC = True 时生效
    FIXED_ADJUST_RPM = 2000.0 
    FIXED_ADJUST_THRESHOLD = 50.0

# 日志配置
logging.basicConfig(
    filename='motor.log',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)
logger = logging.getLogger(__name__)

from Motor_ctl import Motor_CTL, init_can_device as motor_ctl_init_can

class VESCMonitor:
    def __init__(self):
        # 初始化 CAN 设备 (合并初始化 drive 和 steer 通道，以支持共享同一设备的通道)
        
        print(f"初始化 CAN 设备 (Drive: can{BasicConfig.DRIVE_CAN_CHANNEL}, Steer: can{BasicConfig.STEER_CAN_CHANNEL})...")
        
        # 构建通道特定配置
        # 注意：key 是 flat_idx (通常对应 0, 1...)
        channel_configs = {
            BasicConfig.DRIVE_CAN_CHANNEL: {
                "arb_rate": BasicConfig.DRIVE_BAUD_RATE,
                "data_rate": 500000, # 驱动电机数据波特率
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
            dbit_baud_rate=500000, 
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

        # 创建 VESC 接口 (用于转向电机 - can1)
        if self.bus_steer:
            # 在 Windows/Candle 多通道模式下，必须指定 channel_id
            self.tx_steer = TZCANTransmitter(self.bus_steer, channel_id=BasicConfig.STEER_CAN_CHANNEL)
            self.adapter_steer = self._TransmitterAdapter(self.tx_steer, BasicConfig.STEER_USE_CANFD)
            self.vesc = VESC(self.adapter_steer)
        else:
            self.vesc = None
        
        # 创建 Motor_CTL 接口 (用于驱动电机 - can0)
        if self.bus_drive:  
            self.drive_ctl = Motor_CTL(
                channel_handle=self.bus_drive,
                send_id=0x601,
                response_id=0x581
            )
            # 手动设置 channel_id (因为 Motor_CTL.__init__ 不支持传递该参数，但在 Windows 共享 Bus 模式下是必须的)
            if hasattr(self.drive_ctl, 'channel_id'):
                self.drive_ctl.channel_id = BasicConfig.DRIVE_CAN_CHANNEL
                print(f"🔧 已为驱动电机控制器设置 channel_id={BasicConfig.DRIVE_CAN_CHANNEL}")
            
            # 初始化驱动电机控制模式 (参考 Robotchassis.py 逻辑)
            # 1. 设置同步控制模式 (注意：如果不同步发送 SYNC 帧，某些驱动器可能不会更新输出)
            # Robotchassis.py 使用 SYNC_CONTROL，所以这里也改回 SYNC_CONTROL
            self.drive_ctl.set_control_mode(self.drive_ctl.SYNC_CONTROL)
            
            # 1.5 尝试清除故障 (Fault Reset)
            print("正在尝试清除驱动电机故障...")
            # 发送 Control Word = 0x80 (Fault Reset)
            reset_cmd = self.drive_ctl._build_sdo_write(self.drive_ctl.OD_CONTROL_WORD, 0, 0x80, 2)
            self.drive_ctl._send_can_data(self.drive_ctl.send_id, reset_cmd)
            time.sleep(0.1)
            # 发送 Control Word = 0x00 (Clear)
            reset_cmd_0 = self.drive_ctl._build_sdo_write(self.drive_ctl.OD_CONTROL_WORD, 0, 0x00, 2)
            self.drive_ctl._send_can_data(self.drive_ctl.send_id, reset_cmd_0)
            time.sleep(0.1)

            # 2. 初始化电机 (SDO配置)
            # 既然只有一个节点，只需调用一次 initialize_motor
            if not self.drive_ctl.initialize_motor():
                print("⚠️ 驱动电机初始化失败")
            else:
                print("✅ 驱动电机初始化成功")
                # 检查状态并尝试清除故障和重新使能
                time.sleep(0.5)
                status = self.drive_ctl.read_status_word()
                if status is not None:
                    print(f"当前状态字: 0x{status:X}")
                    if (status & 0x08): # Fault bit
                        print("⚠️ 检测到故障，尝试清除...")
                        # Fault Reset (0x80 -> 0x00)
                        self.drive_ctl._send_can_data(self.drive_ctl.send_id, self.drive_ctl._build_sdo_write(self.drive_ctl.OD_CONTROL_WORD, 0, 0x80, 2))
                        time.sleep(0.1)
                        self.drive_ctl._send_can_data(self.drive_ctl.send_id, self.drive_ctl._build_sdo_write(self.drive_ctl.OD_CONTROL_WORD, 0, 0x00, 2))
                        time.sleep(0.5)
                        
                        # 重新执行使能序列
                        print("重新执行使能序列...")
                        self.drive_ctl._send_can_data(self.drive_ctl.send_id, self.drive_ctl._build_sdo_write(self.drive_ctl.OD_CONTROL_WORD, 0, 0x06, 2)) # Shutdown
                        time.sleep(0.1)
                        self.drive_ctl._send_can_data(self.drive_ctl.send_id, self.drive_ctl._build_sdo_write(self.drive_ctl.OD_CONTROL_WORD, 0, 0x07, 2)) # Switch On
                        time.sleep(0.1)
                        self.drive_ctl._send_can_data(self.drive_ctl.send_id, self.drive_ctl._build_sdo_write(self.drive_ctl.OD_CONTROL_WORD, 0, 0x0F, 2)) # Enable
                        time.sleep(0.1)
                        
                        new_status = self.drive_ctl.read_status_word()
                        print(f"重置后状态字: 0x{new_status:X}")
                        if (new_status & 0x000F) == 0x0007:
                            print("✅ 重新使能成功")
                        else:
                            print("❌ 重新使能失败")
                 
            # 3. 配置 RPDO1 (参考 Robotchassis.py)
            # 映射 Target Velocity (60FF:03)
            # Robotchassis.py: mapped_objs_rpdo1 = [(self.motor_ctl.OD_TARGET_VELOCITY, 0x03, 4)]
            # 我们照搬
            mapped_objs_rpdo1 = [(self.drive_ctl.OD_TARGET_VELOCITY, 0x03, 4)]
            if not self.drive_ctl.init_pdo('rpdo1', mapped_objs_rpdo1, self.drive_ctl.PDO_TRANSMIT_EVENT):
                print("⚠️ 驱动电机 RPDO1 初始化失败")
            else:
                print("✅ 驱动电机 RPDO1 初始化成功")
                
            # 4. 配置 TPDO1 (参考 Robotchassis.py)
            # 映射 Actual Velocity (606C:03)
            mapped_objs_tpdo1 = [(self.drive_ctl.OD_VELOCITY_ACTUAL_VALUE, 0x03, 4)]
            if not self.drive_ctl.init_pdo('tpdo1', mapped_objs_tpdo1, self.drive_ctl.PDO_TRANSMIT_EVENT):
                 print("⚠️ 驱动电机 TPDO1 初始化失败")
            else:
                 print("✅ 驱动电机 TPDO1 初始化成功")

        else:
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
                "last_pos": None
            } for mid in BasicConfig.get_all_ids()
        }
        
        self.target_freq = 200.0  # Hz
        
        # 转向目标角度 (Wheel Angle, degrees)
        self.steer_targets: Dict[int, float] = {}
        
        # 记录上电时的初始电机位置 (用于将当前位置作为0度)
        self.motor_initial_pos: Dict[int, float] = {}

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

    def _update_angle(self, motor_id: int, current_pos: float):
        """
        基于单圈编码器数据更新总角度。
        逻辑修改：以上电时读取到的第一个位置作为基准（0度），
        后续所有角度都是相对于该初始位置的增量。
        考虑减速比 1:8。
        """
        state = self.motor_states[motor_id]
        
        # 如果是该电机第一次接收到位置数据，则将其记录为初始位置
        if motor_id not in self.motor_initial_pos:
            self.motor_initial_pos[motor_id] = current_pos
            state["last_pos"] = current_pos
            state["turns"] = 0
            state["total_angle"] = 0.0
            # print(f"Motor {motor_id} initialized at pos {current_pos}. Set as 0 degree.")
            return

        diff = current_pos - state["last_pos"]
        
        # 检测跨圈的阈值（例如跳变超过 180 度）
        threshold = 180.0 
        
        if diff < -threshold:
            state["turns"] += 1
        elif diff > threshold:
            state["turns"] -= 1
            
        # 计算相对于初始位置的电机总转角 (Abs Motor Delta Angle)
        # 当前绝对位置 = (Turns * 360 + Current_Pos)
        # 初始绝对位置 = (0 * 360 + Initial_Pos)
        # 电机增量 = 当前绝对位置 - 初始绝对位置
        
        current_abs_pos = (state["turns"] * 360.0) + current_pos
        initial_abs_pos = self.motor_initial_pos[motor_id]
        
        motor_delta_angle = current_abs_pos - initial_abs_pos
        
        # 计算轮子总角度 (Wheel Angle) = 电机增量 / 减速比
        state["total_angle"] = motor_delta_angle / BasicConfig.STEER_REDUCTION_RATIO
        
        state["last_pos"] = current_pos

    def _monitor_loop(self):
        while self.running:
            # 读取 CAN 数据
            # 使用循环读取所有可用消息，避免缓冲区积压
            while self.running:
                # 1. 处理 VESC 消息 (转向电机 - can1)
                if self.vesc:
                    # 循环读取直到缓冲区为空
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
                                    if vesc_id in BasicConfig.get_steer_ids():
                                        self._update_angle(vesc_id, state["pid_pos"])
                                        
                                        # Fix: Send TARGET position instead of CURRENT position
                                        target_pos = state["pid_pos"] # Default fallback
                                        
                                        if vesc_id in self.steer_targets and vesc_id in self.motor_initial_pos:
                                            target_wheel = self.steer_targets[vesc_id]
                                            ratio = BasicConfig.STEER_REDUCTION_RATIO
                                            initial_pos = self.motor_initial_pos[vesc_id]
                                            
                                            # Target Motor Angle = Initial Position + (Wheel Target * Ratio)
                                            target_pos = initial_pos + (target_wheel * ratio)
                                            # print(f"DEBUG: Locking Motor {vesc_id} to {target_pos:.2f}")

                                        self.vesc.send_pos(vesc_id, target_pos)
                                        
                                # 记录数据
                                if status_id == VESC_CAN_STATUS.VESC_CAN_PACKET_STATUS_1:
                                    log_msg = (f"[VESC] ID: {vesc_id} | RPM: {state['rpm']} | Cur: {state['current']} | "
                                                f"Pos: {state['pid_pos']} | Angle: {state.get('total_angle', 0)}")
                                    logger.info(log_msg)
                
                # 2. 处理 Motor_CTL 消息 (驱动电机 - can0)
                # Motor_CTL 使用 SDO/PDO，通常是请求-响应或自动上报
                # 如果 Motor_CTL 也有类似 receive_decode 的非阻塞方法，应该调用它
                # 目前 Motor_CTL 似乎没有独立的后台接收循环，它的 receive 是阻塞的或在 set_speed 中调用
                # 如果需要监控驱动电机状态，可能需要 Motor_CTL 提供非阻塞读取状态的方法
                # 暂时跳过，假设 Motor_CTL 在主线程控制时会处理响应
                
                # 退出外层读取循环 (本次时间片处理完毕)
                break
            
            # 短暂休眠以防止空转占用过多 CPU，但要足够短以保持高频率
            time.sleep(0.001)
            
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.thread.start()
            print("底盘监控已启动")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        
        # 关闭 CAN 设备
        # 调用 close_can_device 清理资源
        # 注意：现在有两个 m_dev，需要分别清理
        if hasattr(self, 'm_dev_drive') and self.m_dev_drive:
            TZCANTransmitter.close_can_device(self.m_dev_drive)
            
        if hasattr(self, 'm_dev_steer') and self.m_dev_steer:
            TZCANTransmitter.close_can_device(self.m_dev_steer)
            
        # 兼容旧代码清理 m_dev
        if hasattr(self, 'm_dev') and self.m_dev:
             TZCANTransmitter.close_can_device(self.m_dev)
        
        print("底盘监控已停止")
        
    def get_state(self, motor_id):
        with self.lock:
            return self.motor_states.get(motor_id, {}).copy()

class SteerController:
    def __init__(self, monitor: VESCMonitor):
        self.monitor = monitor
        self.vesc = monitor.vesc
        self.drive_ctl = monitor.drive_ctl # 获取驱动电机控制器
        
        # 初始化驱动电机（如果存在）
        if self.drive_ctl:
            # 确保控制模式与初始化时一致 (SYNC_CONTROL)
            # VESCMonitor 已完成初始化，这里无需再次设置模式或初始化
            # 仅打印确认
            print("驱动电机控制器已连接")
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
        
        # offset = BasicConfig.get_offset(motor_id)
        # final_angle = target_angle + offset
        # self.vesc.send_pos(motor_id, final_angle)

    def calibrate_home(self):
        """
        校准归位：将所有转向电机转动到逻辑 0 度位置（正前方）。
        这会应用 BasicConfig 中的 OFFSET 参数。
        注意：目前已禁用开机自动校准，仅作为手动调用接口。
        """
        # if not self.vesc:
        #     print("⚠️ 转向控制器未初始化，跳过归位校准。")
        #     return
            
        # print("正在执行转向归位校准...")
        # self._send_steer_pos(BasicConfig.FL_STEER_ID, 0.0)
        # self._send_steer_pos(BasicConfig.FR_STEER_ID, 0.0)
        # # 给一点时间让电机转到位
        # time.sleep(2.0)
        # print("转向归位完成。")
        print("⚠️ 归位校准已暂时禁用 (Software Position Control Mode)")
        pass

    def spin_left(self, rpm: float = 1000.0, duration: float = None):
        """
        原地左旋（逆时针）。
        保持车轮朝前 (0度)，通过差速驱动旋转。
        左轮后退，右轮前进。
        :param duration: 持续运行时间（秒）。如果提供，将在此方法内循环发送驱动指令。
        """
        print("执行左旋...")

        fl_angle = 0.0
        fr_angle = 0.0
        
        self._send_steer_pos(BasicConfig.FL_STEER_ID, fl_angle)
        self._send_steer_pos(BasicConfig.FR_STEER_ID, fr_angle)
        
        # 等待转向到位 (仅在初次设置时等待)
        time.sleep(1.0) 
        
        # 使用 Motor_CTL 控制驱动电机
        if self.drive_ctl:
            # 左右速度
            left_speed = -rpm
            right_speed = rpm
            left_speed_int = int(left_speed)
            right_speed_int = int(right_speed)
            
            try:
                # 构建 PDO 数据
                left_bytes = left_speed_int.to_bytes(2, byteorder='little', signed=True)
                right_bytes = right_speed_int.to_bytes(2, byteorder='little', signed=True)
                pdo_data = list(left_bytes) + list(right_bytes)
                
                # 定义发送函数
                def send_drive_cmd():
                    # print(f"🚀 发送PDO速度指令: 左={left_speed_int}, 右={right_speed_int}")
                    if not self.drive_ctl.send_pdo('rpdo1', pdo_data):
                        # print("❌ PDO速度指令发送失败") # 降低日志噪音
                        pass

                if duration is None:
                    # 发送一次
                    print(f"🚀 发送PDO速度指令: 左={left_speed_int}, 右={right_speed_int}")
                    if not self.drive_ctl.send_pdo('rpdo1', pdo_data):
                        print("❌ PDO速度指令发送失败")
                else:
                    # 持续发送
                    print(f"🚀 开始持续发送PDO速度指令 ({duration}s): 左={left_speed_int}, 右={right_speed_int}")
                    start_time = time.time()
                    while time.time() - start_time < duration:
                        send_drive_cmd()
                        time.sleep(0.01) # 100Hz
                    print("✅ 持续发送结束")

            except Exception as e:
                print(f"❌ 构建PDO数据出错: {e}")

        else:
            print("⚠️ 驱动控制器未初始化，无法执行 spin_left")
        
    def spin_right(self, rpm: float = 1000.0, duration: float = None):
        """
        原地右旋（顺时针）。
        保持车轮朝前 (0度)，通过差速驱动旋转。
        左轮前进，右轮后退。
        :param duration: 持续运行时间（秒）。如果提供，将在此方法内循环发送驱动指令。
        """
        print("执行右旋...")

        fl_angle = 0.0
        fr_angle = 0.0
        
        self._send_steer_pos(BasicConfig.FL_STEER_ID, fl_angle)
        self._send_steer_pos(BasicConfig.FR_STEER_ID, fr_angle)
        
        # 等待转向到位
        time.sleep(1.0)
        
        if self.drive_ctl:
             # 参考 Robotchassis.py 使用 PDO 发送速度指令
             
             # 左电机正转，右电机反转
             left_speed = rpm
             right_speed = -rpm
             
             left_speed_int = int(left_speed)
             right_speed_int = int(right_speed)
             
             try:
                 left_bytes = left_speed_int.to_bytes(2, byteorder='little', signed=True)
                 right_bytes = right_speed_int.to_bytes(2, byteorder='little', signed=True)
                 pdo_data = list(left_bytes) + list(right_bytes)
                 
                 # 定义发送函数
                 def send_drive_cmd():
                     if not self.drive_ctl.send_pdo('rpdo1', pdo_data):
                         # print("❌ PDO速度指令发送失败")
                         pass

                 if duration is None:
                     print(f"🚀 发送PDO速度指令: 左={left_speed_int}, 右={right_speed_int}")
                     if not self.drive_ctl.send_pdo('rpdo1', pdo_data):
                         print("❌ PDO速度指令发送失败")
                 else:
                     print(f"🚀 开始持续发送PDO速度指令 ({duration}s): 左={left_speed_int}, 右={right_speed_int}")
                     start_time = time.time()
                     while time.time() - start_time < duration:
                         send_drive_cmd()
                         time.sleep(0.01) # 100Hz
                     print("✅ 持续发送结束")
                     
             except Exception as e:
                 print(f"❌ 构建PDO数据出错: {e}")
        else:
            print("⚠️ 驱动控制器未初始化，无法执行 spin_right")

    def stop(self):
        # 停止 VESC 转向 (可选，通常保持位置)
        # for mid in [BasicConfig.FL_DRIVE_ID, BasicConfig.FR_DRIVE_ID]:
        #     self.vesc.send_rpm(mid, 0)
            
        # 停止 Motor_CTL 驱动
        if self.drive_ctl:
            # 发送 0 速度 (PDO)
            try:
                left_bytes = (0).to_bytes(2, byteorder='little', signed=True)
                right_bytes = (0).to_bytes(2, byteorder='little', signed=True)
                pdo_data = list(left_bytes) + list(right_bytes)
                print(f"🛑 发送PDO停止指令: 左=0, 右=0")
                self.drive_ctl.send_pdo('rpdo1', pdo_data)
            except Exception:
                pass
            



# --- 圈数计数函数（如果需要单独作为包装器）---
# 逻辑已集成到 VESCMonitor._update_angle 中，因为它需要状态持久化。

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
