import sys
import os
import time
from steer_wheel_config import BasicConfig

# 添加父目录到 path 以查找 CANMessageTransmitter
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", ".."))

if project_root not in sys.path:
    sys.path.append(project_root)

try:
    from CAN.CANMessageTransmitter import CANMessageTransmitter
    TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
    from VESC_test.can_vesc import VESC, VESC_CAN_STATUS, buffer_get_int16, buffer_get_int32, buffer_get_float16, buffer_get_float32
except ImportError as e:
    print(f"Import Error: {e}")
    raise

class TransmitterAdapter:
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

def init_can_hardware():
    """
    初始化 CAN 硬件设备 (TZCAN)
    :return: (m_dev, bus_drive, bus_steer)
    """
    print(f"初始化 CAN 设备 (Channel {BasicConfig.CAN_CHANNEL_ZERO}, Channel {BasicConfig.CAN_CHANNEL_ONE})...")
    
    channel_configs = {
        BasicConfig.CAN_CHANNEL_ZERO: {
            "arb_rate": BasicConfig.CAN_ZERO_BAUD_RATE,
            "data_rate": BasicConfig.CAN_ZERO_DATA_BITRATE,
            "fd": BasicConfig.CAN_ZERO_USE_CANFD
        },
        BasicConfig.CAN_CHANNEL_ONE: {
            "arb_rate": BasicConfig.CAN_ONE_BAUD_RATE,
            "data_rate": BasicConfig.CAN_ONE_DATA_BITRATE,
            "sp": BasicConfig.SAMPLE_POINT,
            "dsp": BasicConfig.DATA_SAMPLE_POINT,
            "fd": BasicConfig.CAN_ONE_USE_CANFD
        }
    }

    # 调用一次 init_can_device 同时初始化两个通道
    m_dev, bus_drive, bus_steer = TZCANTransmitter.init_can_device(
        baud_rate=BasicConfig.CAN_ZERO_BAUD_RATE, # 默认值
        dbit_baud_rate=BasicConfig.CAN_ZERO_DATA_BITRATE, 
        channels=[BasicConfig.CAN_CHANNEL_ZERO, BasicConfig.CAN_CHANNEL_ONE],
        can_type=1, # TYPE_CANFD
        fd=True, # 全局开启 FD 支持
        channel_configs=channel_configs
    )

    # 如果配置屏蔽了驱动电机，则强制置空 bus_drive，避免后续初始化
    if not BasicConfig.ENABLE_DRIVE:
        bus_drive = None
        print("🚫 驱动电机已通过配置禁用 (ENABLE_DRIVE=False)")
    
    # 检查 CAN 总线是否初始化成功
    if bus_drive is None:
        print(f"⚠️ 警告: CAN 通道 {BasicConfig.CAN_CHANNEL_ZERO} 初始化失败或未连接。")
    else:
            print(f"✅ CAN 通道 {BasicConfig.CAN_CHANNEL_ZERO} 就绪")
            
    if bus_steer is None:
        print(f"⚠️ 警告: CAN 通道 {BasicConfig.CAN_CHANNEL_ONE} 初始化失败或未连接。")
    else:
            print(f"✅ CAN 通道 {BasicConfig.CAN_CHANNEL_ONE} 就绪")
            
    return m_dev, bus_drive, bus_steer

def create_vesc_interfaces(bus_drive, bus_steer):
    """
    创建 VESC 接口对象
    :return: (vesc_if1, vesc_if2)
    """
    
    if BasicConfig.ENABLE_WHEEL_GROUP_CAN_MODE:
        # 新模式: 轮组分组模式
        # bus_drive 对应 can0 (左侧轮组: FL_Steer, FL_Drive, RL_Steer, RL_Drive)
        # bus_steer 对应 can1 (右侧轮组: FR_Steer, FR_Drive, RR_Steer, RR_Drive)
        # 每个通道都混合了转向和驱动电机，因此都需要 CustomVESC 来解析 Status 2
        vesc0 = None
        vesc1 = None
        
        if bus_drive: # can0
            tx0 = TZCANTransmitter(bus_drive, channel_id=BasicConfig.CAN_CHANNEL_ZERO)
            adapter0 = TransmitterAdapter(tx0, BasicConfig.CAN_ZERO_USE_CANFD) # Use Config
            vesc0 = CustomVESC(adapter0)
            
        if bus_steer: # can1
            tx1 = TZCANTransmitter(bus_steer, channel_id=BasicConfig.CAN_CHANNEL_ONE)
            adapter1 = TransmitterAdapter(tx1, BasicConfig.CAN_ONE_USE_CANFD) # Use Config
            vesc1 = CustomVESC(adapter1)
            
        return vesc0, vesc1
        
    else:
        # 原模式: bus_steer 是转向(can1), bus_drive 是驱动(can0)
        vesc_steer = None
        vesc_drive = None
        
        # 创建 VESC 接口 (用于转向电机 - can1)
        if bus_steer:
            # 在 Windows/Candle 多通道模式下，必须指定 channel_id
            tx_steer = TZCANTransmitter(bus_steer, channel_id=BasicConfig.CAN_CHANNEL_ONE)
            adapter_steer = TransmitterAdapter(tx_steer, BasicConfig.CAN_ONE_USE_CANFD)
            vesc_steer = CustomVESC(adapter_steer)
        
        # 创建 VESC 接口 (用于驱动电机 - can0)
        if bus_drive:
            tx_drive = TZCANTransmitter(bus_drive, channel_id=BasicConfig.CAN_CHANNEL_ZERO)
            adapter_drive = TransmitterAdapter(tx_drive, BasicConfig.CAN_ZERO_USE_CANFD)
            vesc_drive = VESC(adapter_drive)
            
        return vesc_steer, vesc_drive

def close_can_device(m_dev):
    """
    关闭 CAN 设备
    """
    if m_dev:
        try:
            TZCANTransmitter.close_can_device(m_dev)
        except Exception as e:
            print(f"关闭 CAN 设备失败: {e}")
