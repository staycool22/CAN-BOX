import time
import sys
import os

# 添加父目录到 sys.path 以查找 CANMessageTransmitter
# 假设 Motor_ctl.py 在 CAN-BOX/VESC_test/steer_wheel/
# CANMessageTransmitter.py 在 CAN-BOX/
current_dir = os.path.dirname(os.path.abspath(__file__))
vesc_test_dir = os.path.dirname(current_dir)
project_root = os.path.dirname(vesc_test_dir)

if project_root not in sys.path:
    sys.path.append(project_root)

# CAN module is in the CAN subdirectory
can_dir = os.path.join(project_root, 'CAN')
if can_dir not in sys.path:
    sys.path.append(can_dir)

try:
    from CANMessageTransmitter import CANMessageTransmitter
except ImportError:
    # 尝试作为包导入
    try:
        from CAN.CANMessageTransmitter import CANMessageTransmitter
    except ImportError:
        # 尝试备用路径
        sys.path.append(os.path.abspath(os.path.join(current_dir, "../../..")))
        from CANMessageTransmitter import CANMessageTransmitter

# 选择后端并保持兼容常量导出
BackendTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
TYPE_CAN = 0
TYPE_CANFD = 1
STATUS_OK = 1
INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0

# 模块级兼容封装
def init_can_device(baud_rate=500000, dbit_baud_rate=2000000, channels=[0], can_type=TYPE_CANFD, canfd_standard=0, channel_count=None, **kwargs):
    return BackendTransmitter.init_can_device(baud_rate, dbit_baud_rate, channels, can_type, canfd_standard, channel_count, **kwargs)

def close_can_device(m_dev, channel_handle0=None, channel_handle1=None):
    return BackendTransmitter.close_can_device(m_dev)
# -------------------------- 核心功能类 --------------------------
class CANMessageSequence(BackendTransmitter):
    """CAN消息序列管理类，支持CANopen CiA402协议，新增PDO发送功能"""

    # CiA402 状态机常量
    STATE_NOT_READY_TO_SWITCH_ON = 0
    STATE_SWITCH_ON_DISABLED = 1
    STATE_READY_TO_SWITCH_ON = 2
    STATE_SWITCHED_ON = 3
    STATE_OPERATION_ENABLED = 4
    STATE_QUICK_STOP_ACTIVE = 5
    STATE_FAULT_REACTION_ACTIVE = 6
    STATE_FAULT = 7
    
    # CiA402 控制字命令
    CONTROL_SHUTDOWN = 0x06
    CONTROL_SWITCH_ON = 0x07
    CONTROL_ENABLE_OPERATION = 0x0F
    CONTROL_DISABLE_VOLTAGE = 0x00
    CONTROL_QUICK_STOP = 0x02
    CONTROL_DISABLE_OPERATION = 0x07
    CONTROL_FAULT_RESET = 0x80
    
    # CiA402 对象字典索引
    OD_CONTROL_WORD = 0x6040
    OD_STATUS_WORD = 0x6041
    OD_MODES_OF_OPERATION = 0x6060
    OD_MODES_OF_OPERATION_DISPLAY = 0x6061
    OD_POSITION_ACTUAL_VALUE = 0x6064
    OD_VELOCITY_ACTUAL_VALUE = 0x606C
    OD_TORQUE_ACTUAL_VALUE = 0x6077
    OD_TARGET_POSITION = 0x607A
    OD_TARGET_VELOCITY = 0x60FF
    OD_TARGET_TORQUE = 0x6071
    OD_MAX_PROFILE_VELOCITY = 0x607F
    OD_PROFILE_VELOCITY = 0x6081
    OD_PROFILE_ACCELERATION = 0x6083
    OD_PROFILE_DECELERATION = 0x6084
    
    # 操作模式
    MODE_POSITION = 1
    MODE_VELOCITY = 3
    MODE_TORQUE = 4
    MODE_HOMING = 6
    MODE_INTERPOLATED_POSITION = 7
    MODE_CYCLIC_SYNC_POSITION = 8
    MODE_CYCLIC_SYNC_VELOCITY = 9
    MODE_CYCLIC_SYNC_TORQUE = 10

    # -------------------------- 新增：PDO相关常量（基于文档1 2.6节） --------------------------
    # PDO传输类型（文档1 2.6.1节，仅支持254/255异步模式）
    PDO_TRANSMIT_EVENT = 254  # 事件触发（数据变化+定时器）
    PDO_TRANSMIT_TIMER = 255  # 定时器触发

    def __init__(self, channel_handle, send_id=0x601, response_id=0x581, response_timeout=5, node_id=1):
        """
        初始化CAN消息序列管理器，支持CANopen CiA402协议
        
        Args:
            channel_handle: CAN通道句柄
            send_id: 发送消息的ID (默认0x601为SDO客户端)
            response_id: 期望接收响应的ID (默认0x581为SDO服务器)
            response_timeout: 等待响应的超时时间（秒）
            node_id: 驱动器节点号（1-127，新增参数，用于计算PDO的COB-ID）
        """
        # 原有初始化逻辑（完全保留）
        super().__init__(channel_handle)
        self.send_id = send_id
        self.response_id = response_id
        self.response_timeout = response_timeout
        self.message_sequences = []  # 存储所有消息序列
        self.current_state = self.STATE_SWITCH_ON_DISABLED
        self.current_mode = self.MODE_POSITION

        # -------------------------- 动态生成PDO配置缓存（支持4组TPDO/RPDO） --------------------------
        self.node_id = node_id
        self.pdo_config = {}
        
        # 根据文档1 2.1节的COB-ID规则动态生成配置
        tpdo_base_cob_ids = [0x180, 0x280, 0x380, 0x480]
        rpdo_base_cob_ids = [0x200, 0x300, 0x400, 0x500]

        for i in range(4):
            # 配置TPDO
            tpdo_name = f"tpdo{i}"
            self.pdo_config[tpdo_name] = {
                "cob_id": tpdo_base_cob_ids[i] + node_id,
                "transmit_type": None,
                "inhibit_time": 0,
                "event_timer": 0,
                "mapped": False,
                "mapped_objs": []
            }
            # 配置RPDO
            rpdo_name = f"rpdo{i}"
            self.pdo_config[rpdo_name] = {
                "cob_id": rpdo_base_cob_ids[i] + node_id,
                "transmit_type": None,
                "inhibit_time": 0,
                "event_timer": 0,
                "mapped": False,
                "mapped_objs": []
            }

    # -------------------------- 原有方法（完全保留，无任何修改） --------------------------
    def add_sequence(self, name, messages):
        """添加一个消息序列"""
        self.message_sequences.append({
            'name': name,
            'messages': messages
        })
    
    def add_cia402_sequence(self, sequence_type, **kwargs):
        """添加标准的CiA402协议序列"""
        if sequence_type == 'init':
            self._add_init_sequence()
        elif sequence_type == 'shutdown':
            self._add_shutdown_sequence()
        elif sequence_type == 'switch_on':
            self._add_switch_on_sequence()
        elif sequence_type == 'enable':
            self._add_enable_operation_sequence()
        elif sequence_type == 'disable':
            self._add_disable_operation_sequence()
        elif sequence_type == 'quick_stop':
            self._add_quick_stop_sequence()
        elif sequence_type == 'fault_reset':
            self._add_fault_reset_sequence()
        elif sequence_type == 'set_mode':
            mode = kwargs.get('mode', self.MODE_POSITION)
            self._add_set_mode_sequence(mode)
        elif sequence_type == 'set_velocity':
            velocity = kwargs.get('velocity', 0)
            self._add_set_velocity_sequence(velocity)
        elif sequence_type == 'set_position':
            position = kwargs.get('position', 0)
            self._add_set_position_sequence(position)
        elif sequence_type == 'set_torque':
            torque = kwargs.get('torque', 0)
            self._add_set_torque_sequence(torque)
        else:
            print(f"❌ 未知的CiA402序列类型: {sequence_type}")
    
    def _add_init_sequence(self):
        """添加初始化序列"""
        self.message_sequences.append({
            'name': 'CiA402_初始化序列',
            'messages': [
                self._build_sdo_write(self.OD_CONTROL_WORD, 0, 0x00, 2),  # 禁用电压
                self._build_sdo_write(self.OD_MODES_OF_OPERATION, 0, self.MODE_POSITION, 1),  # 设置位置模式
            ]
        })
    
    def _add_shutdown_sequence(self):
        """添加关机序列"""
        self.message_sequences.append({
            'name': 'CiA402_关机序列',
            'messages': [
                self._build_sdo_write(self.OD_CONTROL_WORD, 0, self.CONTROL_SHUTDOWN, 2),
            ]
        })
    
    def _add_switch_on_sequence(self):
        """添加上电序列"""
        self.message_sequences.append({
            'name': 'CiA402_上电序列',
            'messages': [
                self._build_sdo_write(self.OD_CONTROL_WORD, 0, self.CONTROL_SWITCH_ON, 2),
            ]
        })
    
    def _add_enable_operation_sequence(self):
        """添加使能操作序列"""
        self.message_sequences.append({
            'name': 'CiA402_使能操作序列',
            'messages': [
                self._build_sdo_write(self.OD_CONTROL_WORD, 0, self.CONTROL_ENABLE_OPERATION, 2),
            ]
        })
    
    def _add_disable_operation_sequence(self):
        """添加禁用操作序列"""
        self.message_sequences.append({
            'name': 'CiA402_禁用操作序列',
            'messages': [
                self._build_sdo_write(self.OD_CONTROL_WORD, 0, self.CONTROL_DISABLE_OPERATION, 2),
            ]
        })
    
    def _add_quick_stop_sequence(self):
        """添加快速停止序列"""
        self.message_sequences.append({
            'name': 'CiA402_快速停止序列',
            'messages': [
                self._build_sdo_write(self.OD_CONTROL_WORD, 0, self.CONTROL_QUICK_STOP, 2),
            ]
        })
    
    def _add_fault_reset_sequence(self):
        """添加故障复位序列"""
        self.message_sequences.append({
            'name': 'CiA402_故障复位序列',
            'messages': [
                self._build_sdo_write(self.OD_CONTROL_WORD, 0, self.CONTROL_FAULT_RESET, 2),
            ]
        })
    
    def _add_set_mode_sequence(self, mode):
        """添加设置操作模式序列"""
        self.message_sequences.append({
            'name': f'CiA402_设置模式_{mode}',
            'messages': [
                self._build_sdo_write(self.OD_MODES_OF_OPERATION, 0, mode, 1),
            ]
        })
    
    def _add_set_velocity_sequence(self, velocity):
        """添加设置速度序列"""
        self.message_sequences.append({
            'name': f'CiA402_设置速度_{velocity}',
            'messages': [
                self._build_sdo_write(self.OD_TARGET_VELOCITY, 0, velocity, 4),
            ]
        })
    
    def _add_set_position_sequence(self, position):
        """添加设置位置序列"""
        self.message_sequences.append({
            'name': f'CiA402_设置位置_{position}',
            'messages': [
                self._build_sdo_write(self.OD_TARGET_POSITION, 0, position, 4),
            ]
        })
    
    def _add_set_torque_sequence(self, torque):
        """添加设置扭矩序列"""
        self.message_sequences.append({
            'name': f'CiA402_设置扭矩_{torque}',
            'messages': [
                self._build_sdo_write(self.OD_TARGET_TORQUE, 0, torque, 2),
            ]
        })
    
    def _build_sdo_write(self, index, subindex, data, data_size):
        """构建SDO写命令"""
        if data_size == 1:
            cmd_byte = 0x2F  # 写1字节
            return [cmd_byte, index & 0xFF, (index >> 8) & 0xFF, subindex, data & 0xFF, 0, 0, 0]
        elif data_size == 2:
            cmd_byte = 0x2B  # 写2字节
            return [cmd_byte, index & 0xFF, (index >> 8) & 0xFF, subindex, 
                   data & 0xFF, (data >> 8) & 0xFF, 0, 0]
        elif data_size == 4:
            cmd_byte = 0x23  # 写4字节
            return [cmd_byte, index & 0xFF, (index >> 8) & 0xFF, subindex,
                   data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF]
        else:
            cmd_byte = 0x2B  # 默认使用2字节写命令
            return [cmd_byte, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
    
    def _build_sdo_read(self, index, subindex):
        """构建SDO读命令"""
        return [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
    
    def read_status_word(self):
        """读取状态字，保持向后兼容"""
        result = self._send_and_receive_sdo(self.OD_STATUS_WORD, 0)
        if isinstance(result, tuple):
            status_word, _ = result
            return status_word
        return result
    
    def _send_and_receive_sdo(self, index, subindex, canfd_mode=False, brs=0, esi=0):
        """发送SDO读命令并接收响应，增强错误处理和重试机制"""
        max_retries = 2
        retry_count = 0
        while retry_count <= max_retries:
            # 发送SDO读命令
            read_cmd = self._build_sdo_read(index, subindex)
            if canfd_mode:
                send_success = self._send_can_data(self.send_id, read_cmd, 
                                               is_ext_frame=False, 
                                               canfd_mode=True, 
                                               brs=brs, 
                                               esi=esi)
            else:
                send_success = self._send_can_data(self.send_id, read_cmd, is_ext_frame=False)
            
            if not send_success:
                mode_desc = "CANFD" if canfd_mode else "CAN"
                print(f"❌ {mode_desc} SDO读命令发送失败: index=0x{index:04X}, subindex=0x{subindex:02X}")
                if retry_count < max_retries:
                    print(f"   第{retry_count+1}次重试...")
                    retry_count += 1
                    time.sleep(0.1)
                    continue
                return None
            
            # 等待响应
            if canfd_mode:
                response_success, response_data = self._receive_can_data(self.response_id, 
                                                                        self.response_timeout, 
                                                                        is_ext_frame=False, 
                                                                        canfd_mode=True)
            else:
                response_success, response_data = self._receive_can_data(self.response_id, 
                                                                        self.response_timeout, 
                                                                        is_ext_frame=False)
            
            if not response_success:
                mode_desc = "CANFD" if canfd_mode else "CAN"
                print(f"❌ {mode_desc} SDO读响应超时: index=0x{index:04X}, subindex=0x{subindex:02X}")
                if retry_count < max_retries:
                    print(f"   第{retry_count+1}次重试...")
                    retry_count += 1
                    time.sleep(0.1)
                    continue
                return None
            
            # 解析响应数据
            if len(response_data) >= 8:
                cmd_byte = response_data[0]
                response_index = (response_data[2] << 8) | response_data[1]
                response_subindex = response_data[3]
                if response_index != index or response_subindex != subindex:
                    print(f"❌ SDO响应索引不匹配: 请求(0x{index:04X}/{subindex})，响应(0x{response_index:04X}/{response_subindex})")
                    if retry_count < max_retries:
                        retry_count += 1
                        time.sleep(0.1)
                        continue
                    return None
                
                try:
                    # 根据命令字解析数据长度
                    if cmd_byte == 0x43:    # 4字节数据 (有符号)
                        data = int.from_bytes(response_data[4:8], byteorder='little', signed=True)
                    elif cmd_byte == 0x4B:  # 2字节数据 (无符号)
                        data = int.from_bytes(response_data[4:6], byteorder='little', signed=False)
                    elif cmd_byte == 0x4F:  # 1字节数据 (无符号)
                        data = response_data[4]
                    else:
                        # 默认/兼容处理
                        data = (response_data[5] << 8) | response_data[4]

                    if cmd_byte in [0x43, 0x4B, 0x4F]:
                        # print(f"✅ SDO响应解析成功: 命令字=0x{cmd_byte:02X}, 数据=0x{data:X}")
                        pass
                    else:
                        print(f"⚠️ SDO响应命令字非标准(0x{cmd_byte:02X})，但成功解析数据=0x{data:04X}")
                    
                    if index == self.OD_STATUS_WORD:
                        state = self._parse_state_from_status_word(data)
                        return data, state
                    return data, None
                except Exception as e:
                    print(f"❌ SDO数据解析异常: {e}")
            
            print(f"❌ SDO响应格式错误: {[hex(d) for d in response_data]}")
            if retry_count < max_retries:
                print(f"   第{retry_count+1}次重试...")
                retry_count += 1
                time.sleep(0.1)
            else:
                print(f"   期望: 命令字=0x4B(快速)或0x43(标准)，实际=0x{response_data[0]:02X}")
        
        return None
    
    def _parse_state_from_status_word(self, status_word):
        """从状态字解析当前状态"""
        bit0 = (status_word >> 0) & 0x01  # Ready to switch on
        bit1 = (status_word >> 1) & 0x01  # Switched on
        bit2 = (status_word >> 2) & 0x01  # Operation enabled
        bit3 = (status_word >> 3) & 0x01  # Fault
        bit5 = (status_word >> 5) & 0x01  # Quick stop
        bit6 = (status_word >> 6) & 0x01  # Switch on disabled
        
        if bit3 == 1:
            return self.STATE_FAULT
        elif bit5 == 0:  # Quick stop active (bit5=0)
            return self.STATE_QUICK_STOP_ACTIVE
        elif bit0 == 0 and bit1 == 0 and bit2 == 0 and bit6 == 1:
            return self.STATE_SWITCH_ON_DISABLED
        elif bit0 == 1 and bit1 == 0 and bit2 == 0 and bit6 == 0:
            return self.STATE_READY_TO_SWITCH_ON
        elif bit0 == 1 and bit1 == 1 and bit2 == 0 and bit6 == 0:
            return self.STATE_SWITCHED_ON
        elif bit0 == 1 and bit1 == 1 and bit2 == 1 and bit6 == 0:
            return self.STATE_OPERATION_ENABLED
        else:
            return self.STATE_NOT_READY_TO_SWITCH_ON
    
    def get_state_name(self, state):
        """获取状态名称"""
        states = {
            self.STATE_NOT_READY_TO_SWITCH_ON: "Not ready to switch on",
            self.STATE_SWITCH_ON_DISABLED: "Switch on disabled",
            self.STATE_READY_TO_SWITCH_ON: "Ready to switch on",
            self.STATE_SWITCHED_ON: "Switched on",
            self.STATE_OPERATION_ENABLED: "Operation enabled",
            self.STATE_QUICK_STOP_ACTIVE: "Quick stop active",
            self.STATE_FAULT_REACTION_ACTIVE: "Fault reaction active",
            self.STATE_FAULT: "Fault"
        }
        return states.get(state, "Unknown state")
    
    def get_mode_name(self, mode):
        """获取模式名称"""
        modes = {
            self.MODE_POSITION: "Position mode",
            self.MODE_VELOCITY: "Velocity mode",
            self.MODE_TORQUE: "Torque mode",
            self.MODE_HOMING: "Homing mode",
            self.MODE_INTERPOLATED_POSITION: "Interpolated position mode",
            self.MODE_CYCLIC_SYNC_POSITION: "Cyclic sync position mode",
            self.MODE_CYCLIC_SYNC_VELOCITY: "Cyclic sync velocity mode",
            self.MODE_CYCLIC_SYNC_TORQUE: "Cyclic sync torque mode"
        }
        return modes.get(mode, "Unknown mode")
    
    def send_sequence(self, sequence_index):
        """发送指定索引的消息序列"""
        if sequence_index >= len(self.message_sequences):
            print(f"❌ 序列索引 {sequence_index} 超出范围")
            return False
        
        sequence = self.message_sequences[sequence_index]
        print(f"\n🚀 开始发送序列: {sequence['name']}")
        
        for i, message_data in enumerate(sequence['messages']):
            print(f"\n📝 发送序列中的第 {i+1}/{len(sequence['messages'])} 条消息")
            # 发送消息
            send_success = self._send_can_data(self.send_id, message_data)
            if not send_success:
                print(f"❌ 发送序列中的第 {i+1} 条消息失败")
                return False
            
            # 等待响应
            print(f"⏳ 等待响应 from 0x{self.response_id:x}...")
            response_success, response_data = self._receive_can_data(self.response_id, self.response_timeout)
            if not response_success:
                print(f"❌ 等待第 {i+1} 条消息的响应超时")
                return False
            
            print(f"✅ 第 {i+1} 条消息处理完成")
        
        print(f"\n🎉 序列 {sequence['name']} 全部发送完成")
        return True
        
    # def read_status_word_canfd(self, brs=0, esi=0):
    #     """使用CANFD模式读取状态字"""
    #     result = self._send_and_receive_sdo(self.OD_STATUS_WORD, 0, canfd_mode=True, brs=brs, esi=esi)
    #     if isinstance(result, tuple):
    #         status_word, _ = result
    #         return status_word
    #     return result
    
    # def send_sequence_canfd(self, sequence_index, brs=0, esi=0):
    #     """使用CANFD模式发送指定索引的消息序列"""
    #     if sequence_index >= len(self.message_sequences):
    #         print(f"❌ 序列索引 {sequence_index} 超出范围")
    #         return False
        
    #     sequence = self.message_sequences[sequence_index]
    #     print(f"\n🚀 开始发送CANFD序列: {sequence['name']}")
        
    #     for i, message_data in enumerate(sequence['messages']):
    #         print(f"\n📝 发送CANFD序列中的第 {i+1}/{len(sequence['messages'])} 条消息")
    #         # 使用CANFD模式发送消息
    #         send_success = self._send_can_data(self.send_id, message_data, 
    #                                          is_ext_frame=False, 
    #                                          canfd_mode=True, 
    #                                          brs=brs, 
    #                                          esi=esi)
    #         if not send_success:
    #             print(f"❌ 发送CANFD序列中的第 {i+1} 条消息失败")
    #             return False
            
    #         # 使用CANFD模式等待响应
    #         print(f"⏳ 等待CANFD响应 from 0x{self.response_id:x}...")
    #         response_success, response_data = self._receive_can_data(self.response_id, 
    #                                                                self.response_timeout, 
    #                                                                is_ext_frame=False, 
    #                                                                canfd_mode=True)
    #         if not response_success:
    #             print(f"❌ 等待CANFD第 {i+1} 条消息的响应超时")
    #             return False
            
    #         print(f"✅ 第 {i+1} 条CANFD消息处理完成")
        
    #     print(f"\n🎉 CANFD序列 {sequence['name']} 全部发送完成")
    #     return True
    
    # def send_all_sequences_canfd(self, brs=0, esi=0):
    #     """使用CANFD模式发送所有消息序列"""
    #     for i in range(len(self.message_sequences)):
    #         if not self.send_sequence_canfd(i, brs=brs, esi=esi):
    #             print(f"❌ CANFD序列 {i} 发送失败")
    #             return False
    #     return True

    def _send_sdo_and_validate(self, sdo_command, description):
        """发送SDO命令并验证响应，增加健壮性"""
        print(f"  - {description}: {[hex(d) for d in sdo_command]}")
        if not self._send_can_data(self.send_id, sdo_command):
            print(f"    ❌ SDO发送失败: {description}")
            return False
        
        response_success, response_data = self._receive_can_data(self.response_id, self.response_timeout)
        
        if not response_success:
            print(f"    ❌ SDO响应超时: {description}")
            return False
            
        if response_data and len(response_data) >= 8:
            # 检查SDO错误响应 (command byte 0x80)
            if response_data[0] == 0x80:
                error_code = int.from_bytes(bytes(response_data[4:8]), 'little')
                print(f"    ❌ SDO从站返回错误: {description}, 错误码: 0x{error_code:08X}")
                return False
            # 检查SDO成功响应 (command byte 0x60)
            elif response_data[0] == 0x60:
                print(f"    ✅ SDO响应成功: {description}")
                return True
            else:
                print(f"    ⚠️ SDO响应格式未知: {description}, 响应: {[hex(d) for d in response_data]}")
                # 假设成功，但记录警告
                return True
        else:
            print(f"    ❌ SDO响应数据无效: {description}, 响应: {response_data}")
            return False
    
    def send_cia402_sequence(self, sequence_type, **kwargs):
        """添加并发送标准的CiA402协议序列"""
        current_count = len(self.message_sequences)
        self.add_cia402_sequence(sequence_type, **kwargs)
        if len(self.message_sequences) > current_count:
            return self.send_sequence(current_count)
        return False    

    

    
    def send_standard_frame(self, send_id, data_list):
        """发送标准帧"""
        return self._send_can_data(send_id, data_list, is_ext_frame=False)
    
    # def send_extended_frame(self, send_id, data_list):
    #     """发送扩展帧"""
    #     return self._send_can_data(send_id, data_list, is_ext_frame=True)
    
    # def send_canfd_standard_frame(self, send_id, data_list, brs=0, esi=0):
    #     """发送CANFD标准帧"""
    #     return self._send_can_data(send_id, data_list, is_ext_frame=False, canfd_mode=True, brs=brs, esi=esi)
    
    # def send_canfd_extended_frame(self, send_id, data_list, brs=0, esi=0):
    #     """发送CANFD扩展帧"""
    #     return self._send_can_data(send_id, data_list, is_ext_frame=True, canfd_mode=True, brs=brs, esi=esi)
    
    def receive_standard_frame(self, target_id=None, timeout=5):
        """接收标准帧"""
        return self._receive_can_data(target_id, timeout, is_ext_frame=False)
    
    # def receive_extended_frame(self, target_id=None, timeout=5):
    #     """接收扩展帧"""
    #     return self._receive_can_data(target_id, timeout, is_ext_frame=True)
    
    # def receive_canfd_standard_frame(self, target_id=None, timeout=5):
    #     """接收CANFD标准帧"""
    #     return self._receive_can_data(target_id, timeout, is_ext_frame=False, canfd_mode=True)
    
    # def receive_canfd_extended_frame(self, target_id=None, timeout=5):
    #     """接收CANFD扩展帧"""
    #     return self._receive_can_data(target_id, timeout, is_ext_frame=True, canfd_mode=True)
        
    def set_nmt_state(self, state_command, node_id=0):
        """
        发送NMT（网络管理）命令以更改节点的状态。
        Args:
            state_command: NMT命令（例如，0x01表示'启动'，0x02表示'停止'，0x80表示'预操作'）。
            node_id: 目标节点ID。如果为0，则为对所有节点的广播。
        Returns:
            bool: 如果命令发送成功，则返回True，否则返回False。
        """
        # NMT命令始终使用COB-ID 0x000发送
        nmt_cob_id = 0x000
        # 数据格式为[命令说明符, 节点ID]
        nmt_data = [state_command, node_id]
        
        state_map = {0x01: "启动节点", 0x02: "停止节点", 0x80: "进入预操作状态", 0x81: "复位节点", 0x82: "复位通信"}
        state_name = state_map.get(state_command, f"未知(0x{state_command:02X})")

        print(f"\n🔌 发送NMT命令: '{state_name}' 到节点ID: {node_id if node_id != 0 else '全部'}")
        
        # NMT消息是标准的CAN帧，不期望响应。
        return self._send_can_data(send_id=nmt_cob_id, data_list=nmt_data, is_ext_frame=False)

    def send_nmt_start(self, node_id=1, is_ext_frame=False):
        """发送NMT启动命令（激活PDO通信，文档2.4节）"""
        # 0x01 = 启动远程节点
        return self.set_nmt_state(0x01, node_id)

    # -------------------------- 新增：PDO核心功能（独立方法，不影响原有逻辑） --------------------------
    def _config_pdo_communication(self, pdo_type, transmit_type, inhibit_time=0, event_timer=0):
        """
        配置PDO通信参数（文档1 2.6.1节）：COB-ID、传输类型、禁止时间、事件定时器
        Args:
            pdo_type: PDO类型（"tpdo0"/"tpdo1"/"rpdo0"/"rpdo1"）
            transmit_type: 传输类型（254=事件触发，255=定时器触发）
            inhibit_time: 禁止时间（单位100us，仅TPDO有效）
            event_timer: 事件定时器（单位500us，仅254/255模式有效）
        Returns:
            bool: 配置成功返回True，失败返回False
        """
        # 参数校验
        if pdo_type not in self.pdo_config:
            print(f"❌ PDO配置错误：无效PDO类型「{pdo_type}」，仅支持tpdo0/tpdo1/rpdo0/rpdo1")
            return False
        if transmit_type not in [self.PDO_TRANSMIT_EVENT, self.PDO_TRANSMIT_TIMER]:
            print(f"❌ PDO配置错误：传输类型仅支持254/255，当前{transmit_type}")
            return False
        if inhibit_time < 0 or inhibit_time > 65535:
            print(f"❌ PDO配置错误：禁止时间范围0-65535（100us），当前{inhibit_time}")
            return False
        if event_timer < 0 or event_timer > 65535:
            print(f"❌ PDO配置错误：事件定时器范围0-65535（500us），当前{event_timer}")
            return False
        
        # 动态计算PDO通信参数索引（如tpdo0->0x1800, rpdo1->0x1401）
        try:
            pdo_prefix = pdo_type[:4]
            pdo_num = int(pdo_type[4:])
            if pdo_prefix == "tpdo":
                comm_index = 0x1800 + pdo_num
            elif pdo_prefix == "rpdo":
                comm_index = 0x1400 + pdo_num
            else:
                raise ValueError("Invalid PDO prefix")
        except (ValueError, IndexError):
            print(f"❌ PDO配置错误：无法从「{pdo_type}」解析PDO索引")
            return False
        pdo_cob_id = self.pdo_config[pdo_type]["cob_id"]

        # 构建SDO配置命令（通信参数需配置4个子索引：COB-ID、传输类型、禁止时间、事件定时器）
        comm_config_cmds = [
            # 子索引01：配置COB-ID（最高位0=启用PDO，文档1 2.6.1节）
            self._build_sdo_write(comm_index, 0x01, pdo_cob_id & 0x7FFFFFFF, 4),
            # 子索引02：配置传输类型
            self._build_sdo_write(comm_index, 0x02, transmit_type, 1),
            # 子索引03：配置禁止时间（TPDO有效，RPDO忽略）
            self._build_sdo_write(comm_index, 0x03, inhibit_time, 2),
            # 子索引05：配置事件定时器（仅254/255模式有效）
            self._build_sdo_write(comm_index, 0x05, event_timer, 2)
        ]

        # 使用新的验证函数发送SDO配置命令
        print(f"\n📝 配置{pdo_type}通信参数：COB-ID=0x{pdo_cob_id:X}，传输类型={transmit_type}")
        descriptions = [
            f"配置{pdo_type} COB-ID",
            f"配置{pdo_type} 传输类型",
            f"配置{pdo_type} 禁止时间",
            f"配置{pdo_type} 事件定时器"
        ]
        for i, cmd in enumerate(comm_config_cmds):
            if not self._send_sdo_and_validate(cmd, descriptions[i]):
                return False
        
        # 更新PDO配置缓存
        self.pdo_config[pdo_type]["transmit_type"] = transmit_type
        self.pdo_config[pdo_type]["inhibit_time"] = inhibit_time
        self.pdo_config[pdo_type]["event_timer"] = event_timer
        print(f"✅ {pdo_type}通信参数配置完成")
        return True

    def _config_pdo_mapping(self, pdo_type, mapped_objs):
        """
        配置PDO映射参数（文档1 2.6.2节）：将对象字典中的参数映射到PDO
        Args:
            pdo_type: PDO类型（"tpdo0"/"tpdo1"/"rpdo0"/"rpdo1"）
            mapped_objs: 映射对象列表，格式：[(索引, 子索引, 数据长度), ...]
                         例：[(0x60FF, 0x02, 2)] 表示映射右电机目标速度（2字节）
        Returns:
            bool: 配置成功返回True，失败返回False
        """
        # 参数校验
        if pdo_type not in self.pdo_config:
            print(f"❌ PDO映射错误：无效PDO类型「{pdo_type}」")
            return False
        if len(mapped_objs) == 0 or len(mapped_objs) > 4:
            print(f"❌ PDO映射错误：映射对象数量需1-4个，当前{len(mapped_objs)}个")
            return False
        
        # 计算总映射长度（需≤8字节，文档1 2.6.2节）
        total_len = sum([len_ for (idx, subidx, len_) in mapped_objs])
        if total_len > 8:
            print(f"❌ PDO映射错误：总长度需≤8字节，当前{total_len}字节")
            return False
        
        # 动态计算PDO映射参数索引（如tpdo0->0x1A00, rpdo1->0x1601）
        try:
            pdo_prefix = pdo_type[:4]
            pdo_num = int(pdo_type[4:])
            if pdo_prefix == "tpdo":
                map_index = 0x1A00 + pdo_num
            elif pdo_prefix == "rpdo":
                map_index = 0x1600 + pdo_num
            else:
                raise ValueError("Invalid PDO prefix")
        except (ValueError, IndexError):
            print(f"❌ PDO映射错误：无法从「{pdo_type}」解析PDO索引")
            return False

        # 构建SDO配置命令（映射参数需先清空→写入映射→设置数量）
        mapping_cmds = []
        # 步骤1：清空现有映射（子索引00写入0）
        mapping_cmds.append(self._build_sdo_write(map_index, 0x00, 0, 1))
        # 步骤2：写入每个映射对象（子索引1~n）
        for i, (idx, subidx, len_) in enumerate(mapped_objs, start=1):
            # 映射数据格式：(索引 << 16) | (子索引 << 8) | (数据长度，单位bit)
            # 例如：映射 60FFh:03 (4字节=32bit) -> (0x60FF << 16) | (0x03 << 8) | 32 = 0x60FF0320
            map_data = (idx << 16) | (subidx << 8) | (len_ * 8)
            mapping_cmds.append(self._build_sdo_write(map_index, i, map_data, 4))
        # 步骤3：设置映射对象数量（子索引00写入映射个数）
        mapping_cmds.append(self._build_sdo_write(map_index, 0x00, len(mapped_objs), 1))

        # 使用新的验证函数发送SDO配置命令
        print(f"\n📝 配置{pdo_type}映射参数：{mapped_objs}，总长度{total_len}字节")
        descriptions = [f"清空{pdo_type}映射"]
        for i in range(len(mapped_objs)):
            descriptions.append(f"配置{pdo_type}映射对象{i+1}")
        descriptions.append(f"启用{pdo_type}的{len(mapped_objs)}个映射")

        for i, cmd in enumerate(mapping_cmds):
            if not self._send_sdo_and_validate(cmd, descriptions[i]):
                return False
        
        # 更新PDO配置缓存
        self.pdo_config[pdo_type]["mapped"] = True
        self.pdo_config[pdo_type]["mapped_objs"] = mapped_objs
        print(f"✅ {pdo_type}映射参数配置完成")
        return True

    def init_pdo(self, pdo_type, mapped_objs, transmit_type, inhibit_time=0, event_timer=0):
        """
        一键初始化PDO（通信参数+映射参数+NMT启动），确保PDO可正常发送
        Args:
            pdo_type: PDO类型（"tpdo0"/"tpdo1"/"rpdo0"/"rpdo1"）
            mapped_objs: 映射对象列表（同_config_pdo_mapping）
            transmit_type: 传输类型（254=事件触发，255=定时器触发）
            inhibit_time: 禁止时间（单位100us，仅TPDO有效）
            event_timer: 事件定时器（单位500us，仅254/255模式有效）
        Returns:
            bool: 初始化成功返回True，失败返回False
        """
        print(f"\n🚀 开始初始化PDO：类型={pdo_type}")
        # 步骤1：配置通信参数
        if not self._config_pdo_communication(pdo_type, transmit_type, inhibit_time, event_timer):
            return False
        # 步骤2：配置映射参数
        if not self._config_pdo_mapping(pdo_type, mapped_objs):
            return False
        # 步骤3：发送NMT启动命令（激活PDO通信，文档1 2.4节强制要求）
        if not self.send_nmt_start(node_id=self.node_id):
            print(f"⚠️ {pdo_type}初始化警告：NMT启动命令发送失败，PDO可能无法激活")
            return True
        print(f"🎉 {pdo_type}初始化完成，可开始发送PDO数据")
        return True

    def send_pdo(self, pdo_type, data_list, canfd_mode=False, brs=0, esi=0, force=False):
        """
        发送PDO数据（文档1 2.6.3节，单向传输无需应答）
        Args:
            pdo_type: PDO类型（"tpdo0"/"tpdo1"/"rpdo0"/"rpdo1"）
            data_list: 发送数据列表（长度需与映射总长度一致，1-8字节）
            canfd_mode: 是否使用CANFD模式（默认False）
            brs: CANFD比特率切换（0=不切换，1=切换，仅CANFD模式有效）
            esi: CANFD错误状态指示（0=主动错误，1=被动错误，仅CANFD模式有效）
            force: (bool) 是否强制发送，忽略初始化检查
        Returns:
            bool: 发送成功返回True，失败返回False
        """
        if not force:
            # 前置校验
            if pdo_type not in self.pdo_config or not self.pdo_config[pdo_type]["mapped"]:
                print(f"❌ PDO发送错误：{pdo_type}未初始化，请先调用init_pdo()配置")
                return False
            
            # 校验数据长度（与映射总长度一致）
            mapped_objs = self.pdo_config[pdo_type]["mapped_objs"]
            expected_len = sum([len_ for (idx, subidx, len_) in mapped_objs])
            if len(data_list) != expected_len:
                print(f"❌ PDO发送错误：{pdo_type}数据长度不匹配，期望{expected_len}字节，实际{len(data_list)}字节")
                return False
        
        # 获取PDO的COB-ID
        pdo_cob_id = self.pdo_config[pdo_type]["cob_id"]

        # 发送PDO数据（复用原有_send_can_data方法，确保CAN/CANFD兼容性）
        print(f"\n🚀 发送{pdo_type}数据：COB-ID=0x{pdo_cob_id:X}，Data={[hex(d) for d in data_list]}")
        return self._send_can_data(
            send_id=pdo_cob_id,
            data_list=data_list,
            is_ext_frame=False,  # PDO使用标准帧（文档1 2.1节）
            canfd_mode=canfd_mode,
            brs=brs,
            esi=esi
        )

    def shutdown_motor(self):
        """
        发送“禁用电压”命令（控制字=0x0000）以重置电机的状态机。
        此版本仅发送命令而不等待响应，以避免访问冲突错误。
        """
        print("\n🔌 Shutting down motor (Control Word -> 0x0000)...")
        # CONTROL_DISABLE_VOLTAGE is 0x00
        shutdown_cmd = self._build_sdo_write(self.OD_CONTROL_WORD, 0, self.CONTROL_DISABLE_VOLTAGE, 2)
        
        # 直接发送命令，不等待验证
        print(f"  - Disabling voltage (fire and forget): {[hex(d) for d in shutdown_cmd]}")
        if self._send_can_data(self.send_id, shutdown_cmd):
            print("✅ Motor shutdown command sent. Assuming success.")
            # 留出一点时间让驱动器处理状态变化
            time.sleep(0.2) # 稍微增加延时
            return True
        else:
            print("❌ Failed to send motor shutdown command.")
            return False

# -------------------------- 速度控制类 --------------------------
class Motor_CTL(CANMessageSequence):
    """电机控制类，用于控制电机的速度模式"""
    
    # 控制方式常量
    SYNC_CONTROL = 0      # 同步控制
    ASYNC_CONTROL = 1     # 异步控制
    
    def __init__(self, channel_handle, send_id=0x601, response_id=0x581, response_timeout=5):
        """
        初始化电机控制器
        
        Args:
            channel_handle: CAN通道句柄
            send_id: 发送消息的ID
            response_id: 期望接收响应的ID
            response_timeout: 等待响应的超时时间（秒）
        """
        # 调用父类初始化
        super().__init__(channel_handle, send_id, response_id, response_timeout)
        
        self.control_mode = None  # 当前控制方式
        self.speed = 0            # 当前速度值
        
        # 速度控制序列模板
        self.speed_control_template = [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    def set_control_mode(self, mode):
        """
        设置控制方式
        
        Args:
            mode: 控制方式 (SYNC_CONTROL 或 ASYNC_CONTROL)
        """
        if mode not in [self.SYNC_CONTROL, self.ASYNC_CONTROL]:
            print("❌ 无效的控制方式")
            return False
        
        self.control_mode = mode
        print(f"✅ 控制方式设置为: {'同步控制' if mode == self.SYNC_CONTROL else '异步控制'}")
        return True
    
    def _convert_speed_to_bytes(self, speed):
        """
        将速度值转换为字节序列
        
        Args:
            speed: 速度值 (-1000 到 1000)
            
        Returns:
            tuple: (byte_3, byte_4, byte_5, byte_6) 分别对应序列中的第4、5、6、7位
        """
        # 限制速度范围
        speed = max(-1000, min(1000, speed))
        
        if speed >= 0:
            # 正数处理
            byte_3 = speed & 0xFF           # 低8位
            byte_4 = (speed >> 8) & 0xFF    # 高8位
            byte_5 = 0x00                   # 负数标志位
            byte_6 = 0x00                   # 负数标志位
        else:
            # 负数处理
            speed = (1<<16) + speed
            byte_3 = speed & 0xFF      # 低8位
            byte_4 = (speed >> 8) & 0xFF    # 高8位
            byte_5 = 0xFF                   # 负数标志位
            byte_6 = 0xFF                   # 负数标志位
            
        return byte_3, byte_4, byte_5, byte_6
    
    def set_speed(self, speed, run_time_ms=None, left_speed=None, right_speed=None):
        """
        设置电机速度
        
        Args:
            speed: 速度值 (-1000 到 1000)，在同步控制模式下使用，或在异步控制模式下作为默认速度
            run_time_ms: 运行时间（毫秒），可选参数，如果提供则在指定时间后发送停止命令
            left_speed: 左电机速度值 (-1000 到 1000)，仅在异步控制模式下使用
            right_speed: 右电机速度值 (-1000 到 1000)，仅在异步控制模式下使用
            
        Returns:
            bool: 是否设置成功
        """
        if self.control_mode is None:
            print("❌ 请先设置控制方式")
            return False
        
        success = False
        if self.control_mode == self.SYNC_CONTROL:
            # 同步控制：同时控制两个电机，只需发送一条消息
            # 限制速度范围
            speed = max(-1000, min(1000, speed))
            self.speed = speed
            
            # 转换速度值为字节序列
            byte_3, byte_4, byte_5, byte_6 = self._convert_speed_to_bytes(speed)
            
            # 第4、5位（索引3、4）是左电机速度
            # 第6、7位（索引5、6）是右电机速度
            sync_speed_seq = self.speed_control_template[:]
            sync_speed_seq[3] = 0x03
            sync_speed_seq[4] = byte_3      # 左电机速度低8位
            sync_speed_seq[5] = byte_4      # 左电机速度高8位
            sync_speed_seq[6] = byte_3      # 右电机速度低8位
            sync_speed_seq[7] = byte_4      # 右电机速度高8位
            
            print(f"🚀 发送同步控制速度命令: {[hex(d) for d in sync_speed_seq]}")
            success = self._send_can_data(self.send_id, sync_speed_seq)
            
            if not success:
                print("❌ 同步控制速度设置失败")
                return False
            
            # 等待响应
            print(f"⏳ 等待响应 from 0x{self.response_id:x}...")
            response_success, _ = self._receive_can_data(self.response_id, self.response_timeout)
            
            if not response_success:
                print("❌ 同步控制响应超时")
                return False
                
            print(f"✅ 同步控制速度设置成功: {speed}")
        else:  # ASYNC_CONTROL
            # 异步控制：分别控制两个电机
            # 如果没有提供单独的左右电机速度，则使用speed参数作为默认速度
            if left_speed is None:
                left_speed = speed
            if right_speed is None:
                right_speed = speed
                
            # 限制速度范围
            left_speed = max(-1000, min(1000, left_speed))
            right_speed = max(-1000, min(1000, right_speed))
            self.speed = (left_speed, right_speed)  # 保存左右电机速度
            
            # 转换左右电机速度值为字节序列
            left_byte_3, left_byte_4, left_byte_5, left_byte_6 = self._convert_speed_to_bytes(left_speed)
            right_byte_3, right_byte_4, right_byte_5, right_byte_6 = self._convert_speed_to_bytes(right_speed)
            
            # 发送左电机速度控制序列
            left_speed_seq = self.speed_control_template[:]
            left_speed_seq[3] = 0x01        # 左电机ID
            left_speed_seq[4] = left_byte_3      # 速度低8位
            left_speed_seq[5] = left_byte_4      # 速度高8位
            left_speed_seq[6] = left_byte_5      # 负数标志位
            left_speed_seq[7] = left_byte_6      # 负数标志位
            
            print(f"🚀 发送左电机速度控制命令: {[hex(d) for d in left_speed_seq]}")
            success1 = self._send_can_data(self.send_id, left_speed_seq)
            
            if not success1:
                print("❌ 左电机速度设置失败")
                # 不立即返回，继续尝试右电机
            else:
                # 等待响应
                print(f"⏳ 等待左电机响应 from 0x{self.response_id:x}...")
                response_success1, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success1:
                    print("❌ 左电机响应超时")
                    # 不立即返回，继续尝试右电机
            
            # 发送右电机速度控制序列
            right_speed_seq = self.speed_control_template[:]
            right_speed_seq[3] = 0x02       # 右电机ID
            right_speed_seq[4] = right_byte_3     # 速度低8位
            right_speed_seq[5] = right_byte_4     # 速度高8位
            right_speed_seq[6] = right_byte_5     # 负数标志位
            right_speed_seq[7] = right_byte_6     # 负数标志位
            
            print(f"🚀 发送右电机速度控制命令: {[hex(d) for d in right_speed_seq]}")
            success2 = self._send_can_data(self.send_id, right_speed_seq)
            
            if not success2:
                print("❌ 右电机速度设置失败")
                # 不立即返回，继续处理运行时间
            else:
                # 等待响应
                print(f"⏳ 等待右电机响应 from 0x{self.response_id:x}...")
                response_success2, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success2:
                    print("❌ 右电机响应超时")
                    # 不立即返回，继续处理运行时间
                
            # 只要至少有一个电机设置成功，就认为成功
            success = success1 or success2
            if success:
                print(f"✅ 异步控制速度设置成功: 左电机 {left_speed}, 右电机 {right_speed}")
            else:
                print("❌ 异步控制速度设置完全失败")
        
        # 如果提供了运行时间参数，则等待指定时间后发送停止命令
        # 注意：无论同步还是异步控制模式，只要提供了运行时间参数，就应该发送停止命令
        if run_time_ms is not None and success:
            print(f"⏳ 电机将运行 {run_time_ms} 毫秒...")
            time.sleep(run_time_ms / 1000.0)  # 转换为秒
            
            # 发送停止命令
            stop_command = [0x2b, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00]
            print(f"🛑 发送停止命令: {[hex(d) for d in stop_command]}")
            stop_success = self._send_can_data(self.send_id, stop_command)
            
            if not stop_success:
                print("❌ 停止命令发送失败")
                return False
            
            # 等待响应
            print(f"⏳ 等待停止命令响应 from 0x{self.response_id:x}...")
            stop_response_success, _ = self._receive_can_data(self.response_id, self.response_timeout)
            
            if not stop_response_success:
                print("❌ 停止命令响应超时")
                return False
                
            print("✅ 电机已停止")
        
        return success

    def update_acceleration(self, accel_time_ms, decel_time_ms):
        """
        动态更新加减速时间（写入 6083h/6084h）
        无需重新初始化电机，直接通过 SDO 修改参数
        :param accel_time_ms: 加速时间 (ms)
        :param decel_time_ms: 减速时间 (ms)
        :return: bool 是否成功
        """
        print(f"\n🚀 动态更新加减速时间: Accel={accel_time_ms}ms, Decel={decel_time_ms}ms")
        
        cmds = [
            # 左电机加速 (6083:01)
            (self._build_sdo_write(0x6083, 0x01, int(accel_time_ms), 4), "左电机加速时间"),
            # 右电机加速 (6083:02)
            (self._build_sdo_write(0x6083, 0x02, int(accel_time_ms), 4), "右电机加速时间"),
            # 左电机减速 (6084:01)
            (self._build_sdo_write(0x6084, 0x01, int(decel_time_ms), 4), "左电机减速时间"),
            # 右电机减速 (6084:02)
            (self._build_sdo_write(0x6084, 0x02, int(decel_time_ms), 4), "右电机减速时间")
        ]
        
        for cmd, desc in cmds:
            if not self._send_sdo_and_validate(cmd, desc):
                print(f"❌ 更新 {desc} 失败")
                return False
        
        print("✅ 加减速时间更新完成")
        return True

    def initialize_motor(self, accel_time_ms=3500, decel_time_ms=2000):
        """
        基于 CiA402 标准协议初始化速度模式（替代固定序列）
        实现逻辑：同步/异步标志→速度模式→加减速时间→状态机切换→NMT 启动
        :param accel_time_ms: 加速时间 (ms)，默认 3500ms
        :param decel_time_ms: 减速时间 (ms)，默认 2000ms
        """
        if self.control_mode is None:
            print("❌ 请先调用 set_control_mode() 设置同步/异步控制方式")
            return False
        
        # 1. 清空现有序列，避免冲突
        self.message_sequences = []
        control_mode_name = "同步" if self.control_mode == self.SYNC_CONTROL else "异步"
        print(f"\n🚀 开始{control_mode_name}控制速度模式初始化（CiA402 标准流程）")

        # 2. 步骤1：配置同步/异步控制标志（200Fh，文档 200Fh 定义）
        sync_flag = 1 if self.control_mode == self.SYNC_CONTROL else 0
        self.add_sequence(
            name=f"{control_mode_name}控制标志配置",
            messages=[self._build_sdo_write(
                index=0x200F,    # 对象索引（同步/异步标志）
                subindex=0x00,   # 子索引（无多子索引）
                data=sync_flag, # 1=同步，0=异步
                data_size=2     # 数据长度（U16 类型，文档 200Fh 类型定义）
            )]
        )

        # 3. 步骤2：设置速度工作模式（6060h=3，文档 3.4 节）
        self.add_sequence(
            name="速度模式配置",
            messages=[self._build_sdo_write(
                index=0x6060,    # 对象索引（工作模式设置）
                subindex=0x00,   # 子索引
                data=self.MODE_VELOCITY, # 3=速度模式（父类常量）
                data_size=1      # 数据长度（I8 类型，文档 6060h 类型定义）
            )]
        )

        # 4. 步骤3：配置加减速时间（6083h/6084h，文档 3.4.2 节例程参数）
        # 参数化传入的时间
        accel_decel_params = [
            # 左电机加速时间（6083h.01）
            self._build_sdo_write(0x6083, 0x01, int(accel_time_ms), 4),
            # 右电机加速时间（6083h.02）
            self._build_sdo_write(0x6083, 0x02, int(accel_time_ms), 4),
            # 左电机减速时间（6084h.01）
            self._build_sdo_write(0x6084, 0x01, int(decel_time_ms), 4),
            # 右电机减速时间（6084h.02）
            self._build_sdo_write(0x6084, 0x02, int(decel_time_ms), 4)
        ]
        self.add_sequence(name="加减速时间配置", messages=accel_decel_params)

        # 5. 步骤4：CiA402 状态机逐步切换（文档 3.2 节控制字规则）
        state_switch_params = [
            # 禁用电压（0x6040=0x00，切换至 SWITCH ON DISABLED）
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, 0x00, 2),
            # Shutdown（0x6040=0x06，切换至 READY TO SWITCH ON）
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_SHUTDOWN, 2),
            # Switch On（0x6040=0x07，切换至 SWITCHED ON）
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_SWITCH_ON, 2),
            # Enable Operation（0x6040=0x0F，切换至 OPERATION ENABLED）
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_ENABLE_OPERATION, 2)
        ]
        self.add_sequence(name="CiA402 状态机切换", messages=state_switch_params)

        # 6. 发送所有初始化序列（逐序列发送，每步等待应答）
        init_success = True
        for seq_idx in range(len(self.message_sequences)):
            if not self.send_sequence(seq_idx):
                init_success = False
                print(f"❌ {self.message_sequences[seq_idx]['name']} 失败")
                break

        # 7. 步骤5：发送 NMT 启动命令（激活 PDO 通信，文档 2.4 节强制要求）
        if init_success:
            nmt_success = self.send_nmt_start(node_id=1)  # Node-ID 可根据实际配置修改
            if nmt_success:
                print(f"✅ NMT 启动命令发送成功，PDO 通信激活")
            else:
                print(f"⚠️ NMT 启动命令发送失败，PDO 可能无法传输实时数据")

        # 8. 验证最终状态（读取 6041h，确认低4位=0111，即 OPERATION ENABLED）
        if init_success:
            status_word = self.read_status_word()
            if status_word is not None and (status_word & 0x000F) == 0x0007:
                print(f"\n🎉 {control_mode_name}控制速度模式初始化完成，当前状态：OPERATION ENABLED")
            else:
                print(f"\n⚠️ {control_mode_name}控制初始化状态异常，6041h=0x{status_word:04X}（期望低4位=0111）")

        return init_success
# -------------------------- 位置控制类 --------------------------
class Motor_CTL_Pos(CANMessageSequence):
    """电机位置控制类，用于控制电机的位置模式（同步控制）"""
    
    # 位置模式常量
    RELATIVE_POSITION = 0  # 相对位置
    ABSOLUTE_POSITION = 1  # 绝对位置
    
    def __init__(self, channel_handle, send_id=0x601, response_id=0x581, response_timeout=5):
        """
        初始化电机位置控制器
        
        Args:
            channel_handle: CAN通道句柄
            send_id: 发送消息的ID
            response_id: 期望接收响应的ID
            response_timeout: 等待响应的超时时间（秒）
        """
        super().__init__(channel_handle, send_id, response_id, response_timeout)
        self.position_mode = None  # 当前位置模式
        self.max_velocity = 60    # 默认最大速度60r/min
        
        # 位置控制序列模板
        self.position_control_template = [0x23, 0x7a, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    def set_max_velocity(self, velocity):
        """
        设置最大速度
        
        Args:
            velocity: 最大速度值 (r/min)
        """
        self.max_velocity = velocity
        print(f"✅ 最大速度设置为: {velocity} r/min")
        return True
    
    def set_position_mode(self, mode):
        """
        设置位置模式
        
        Args:
            mode: 位置模式 (RELATIVE_POSITION 或 ABSOLUTE_POSITION)
        """
        if mode not in [self.RELATIVE_POSITION, self.ABSOLUTE_POSITION]:
            print("❌ 无效的位置模式")
            return False
        
        self.position_mode = mode
        print(f"✅ 位置模式设置为: {'相对位置' if mode == self.RELATIVE_POSITION else '绝对位置'}")
        return True

    def _convert_position_to_bytes(self, position):
        """
        将位置值转换为字节序列
        
        Args:
            position: 位置值 (有符号整数)
            
        Returns:
            tuple: (pos_l, pos_h, sign_flag) 分别对应位置的低8位、高8位和符号标志
        """
        if position >= 0:
            # 正数处理
            pos_l = position & 0xFF           # 低8位
            pos_h = (position >> 8) & 0xFF    # 高8位
            sign_flag = 0x00                  # 正数标志位
        else:
            # 负数处理（补码方式）
            position = (1 << 16) + position   # 转换为补码
            pos_l = position & 0xFF           # 低8位
            pos_h = (position >> 8) & 0xFF    # 高8位
            sign_flag = 0xFF                  # 负数标志位
            
        return pos_l, pos_h, sign_flag
    
    def set_position(self, position_mode, left_position, right_position):
        """
        设置电机位置
        
        Args:
            position_mode: 位置模式 (RELATIVE_POSITION 或 ABSOLUTE_POSITION)
            left_position: 左电机目标位置
            right_position: 右电机目标位置
            
        Returns:
            bool: 是否设置成功
        """
        if position_mode not in [self.RELATIVE_POSITION, self.ABSOLUTE_POSITION]:
            print("❌ 无效的位置模式")
            return False
        
        # 设置位置模式
        self.position_mode = position_mode
        
        success = True
        
        # 设置左电机位置
        left_success = self._set_single_motor_position(0x01, left_position)
        if not left_success:
            print("❌ 左电机位置设置失败")
            success = False
        
        # 设置右电机位置
        right_success = self._set_single_motor_position(0x02, right_position)
        if not right_success:
            print("❌ 右电机位置设置失败")
            success = False
        
        if success:
            # 使用标准CiA402协议发送启动命令
            # 根据用户提供的信息：
            # - 相对运动启动需要发送两条命令：4F和5F
            # - 绝对运动启动需要发送两条命令：0F和1F
            if position_mode == self.RELATIVE_POSITION:
                print("\n🚀 发送相对位置启动命令")
                # 发送第一条命令
                first_cmd = self._build_sdo_write(self.OD_CONTROL_WORD, 0, 0x4F, 2)
                print(f"📝 发送启动命令1: {[hex(d) for d in first_cmd]}")
                first_success = self._send_can_data(self.send_id, first_cmd)
                
                if not first_success:
                    print("❌ 启动命令1发送失败")
                    return False
                
                # 等待响应
                print(f"⏳ 等待启动命令1响应 from 0x{self.response_id:x}...")
                response_success, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success:
                    print("❌ 启动命令1响应超时")
                    return False
                
                # 发送第二条命令
                second_cmd = self._build_sdo_write(self.OD_CONTROL_WORD, 0, 0x5F, 2)
                print(f"📝 发送启动命令2: {[hex(d) for d in second_cmd]}")
                second_success = self._send_can_data(self.send_id, second_cmd)
                
                if not second_success:
                    print("❌ 启动命令2发送失败")
                    return False
                
                # 等待响应
                print(f"⏳ 等待启动命令2响应 from 0x{self.response_id:x}...")
                response_success, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success:
                    print("❌ 启动命令2响应超时")
                    return False
                    
                print("✅ 相对位置控制启动成功")
            else:  # 绝对位置模式
                print("\n🚀 发送绝对位置启动命令")
                # 发送第一条命令
                first_cmd = self._build_sdo_write(self.OD_CONTROL_WORD, 0, 0x0F, 2)
                print(f"📝 发送启动命令1: {[hex(d) for d in first_cmd]}")
                first_success = self._send_can_data(self.send_id, first_cmd)
                
                if not first_success:
                    print("❌ 启动命令1发送失败")
                    return False
                
                # 等待响应
                print(f"⏳ 等待启动命令1响应 from 0x{self.response_id:x}...")
                response_success, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success:
                    print("❌ 启动命令1响应超时")
                    return False
                
                # 发送第二条命令
                second_cmd = self._build_sdo_write(self.OD_CONTROL_WORD, 0, 0x1F, 2)
                print(f"📝 发送启动命令2: {[hex(d) for d in second_cmd]}")
                second_success = self._send_can_data(self.send_id, second_cmd)
                
                if not second_success:
                    print("❌ 启动命令2发送失败")
                    return False
                
                # 等待响应
                print(f"⏳ 等待启动命令2响应 from 0x{self.response_id:x}...")
                response_success, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success:
                    print("❌ 启动命令2响应超时")
                    return False
                    
                print("✅ 绝对位置控制启动成功")
                
            print(f"✅ 位置设置完成: 左电机 {left_position}, 右电机 {right_position}")
        
        return success
    
    def _set_single_motor_position(self, motor_id, position):
        """
        设置单个电机的位置
        
        Args:
            motor_id: 电机ID (0x01=左电机, 0x02=右电机)
            position: 目标位置
            
        Returns:
            bool: 是否设置成功
        """
        # 转换位置值为字节序列
        pos_l, pos_h, sign_flag = self._convert_position_to_bytes(position)
        
        # 构建位置控制序列
        position_seq = self.position_control_template[:]
        position_seq[3] = motor_id        # 电机ID
        position_seq[4] = pos_l           
        position_seq[5] = pos_h           
        position_seq[6] = sign_flag        
        position_seq[7] = sign_flag       # 符号标志位
        
        motor_name = "左电机" if motor_id == 0x01 else "右电机"
        
        # 重试机制：最多尝试3次
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            print(f"🔄 第{attempt}次尝试设置{motor_name}位置...")
            
            print(f"🚀 发送{motor_name}位置控制命令: {[hex(d) for d in position_seq]}")
            success = self._send_can_data(self.send_id, position_seq)
            
            if not success:
                print(f"❌ 第{attempt}次发送{motor_name}位置控制命令失败")
                if attempt < max_retries:
                    print(f"⏳ 等待100ms后重试...")
                    time.sleep(0.1)
                    continue
                else:
                    print(f"❌ {motor_name}位置设置失败，已达到最大重试次数{max_retries}")
                    return False
            
            # 等待响应并验证响应数据
            print(f"⏳ 等待{motor_name}响应 from 0x{self.response_id:x}...")
            response_success, response_data = self._receive_can_data(self.response_id, self.response_timeout)
            
            if not response_success:
                print(f"❌ 第{attempt}次{motor_name}响应超时")
                if attempt < max_retries:
                    print(f"⏳ 等待100ms后重试...")
                    time.sleep(0.1)
                    continue
                else:
                    print(f"❌ {motor_name}响应超时，已达到最大重试次数{max_retries}")
                    return False
            
            # 验证响应数据格式
            # 左电机期望: [0x60, 0x7a, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00]
            # 右电机期望: [0x60, 0x7a, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00]
            expected_response = [0x60, 0x7a, 0x60, motor_id, 0x00, 0x00, 0x00, 0x00]
            
            if len(response_data) >= 8:
                # 检查关键字节：索引0、1、2、3应该匹配期望值
                if (response_data[0] == expected_response[0] and  # 0x60
                    response_data[1] == expected_response[1] and  # 0x7a
                    response_data[2] == expected_response[2] and  # 0x60
                    response_data[3] == expected_response[3]):    # motor_id
                    print(f"✅ {motor_name}位置设置成功: {position}")
                    print(f"✅ 从机响应正确: {[hex(d) for d in response_data]}")
                    return True
                else:
                    print(f"❌ 第{attempt}次{motor_name}响应数据格式错误")
                    print(f"   期望: {[hex(d) for d in expected_response]}")
                    print(f"   实际: {[hex(d) for d in response_data]}")
                    if attempt < max_retries:
                        print(f"⏳ 等待100ms后重试...")
                        time.sleep(0.1)
                        continue
                    else:
                        print(f"❌ {motor_name}响应数据格式错误，已达到最大重试次数{max_retries}")
                        print("⚠️  请检查电机配置和通信状态")
                        return False
            else:
                print(f"❌ 第{attempt}次{motor_name}响应数据长度不足: {len(response_data)}字节")
                print(f"   实际响应: {[hex(d) for d in response_data]}")
                if attempt < max_retries:
                    print(f"⏳ 等待100ms后重试...")
                    time.sleep(0.1)
                    continue
                else:
                    print(f"❌ {motor_name}响应数据长度不足，已达到最大重试次数{max_retries}")
                    return False
        
        return False

    def initialize_motor(self):
        """
        基于 CiA402 标准协议初始化位置模式（替代固定序列）
        实现逻辑：位置模式→最大速度→加减速时间→状态机切换→NMT 启动
        """
        print(f"\n🚀 开始位置控制初始化（最大速度: {self.max_velocity} r/min，CiA402 标准流程）")

        # 1. 清空现有序列
        self.message_sequences = []

        # 2. 步骤1：设置位置工作模式（6060h=1，文档 3.3 节）
        self.add_sequence(
            name="位置模式配置",
            messages=[self._build_sdo_write(
                index=0x6060,    # 对象索引（工作模式设置）
                subindex=0x00,   # 子索引
                data=self.MODE_POSITION, # 1=位置模式（父类常量）
                data_size=1      # 数据长度（I8 类型，文档 6060h 类型定义）
            )]
        )

        # 3. 步骤2：配置加减速时间（6083h/6084h=100ms，文档 3.3.2 节例程参数）
        accel_decel_params = [
            self._build_sdo_write(0x6083, 0x01, 100, 4),  # 左电机加速
            self._build_sdo_write(0x6083, 0x02, 100, 4),  # 右电机加速
            self._build_sdo_write(0x6084, 0x01, 100, 4),  # 左电机减速
            self._build_sdo_write(0x6084, 0x02, 100, 4)   # 右电机减速
        ]
        self.add_sequence(name="加减速时间配置", messages=accel_decel_params)

        # 4. 步骤3：配置最大速度（6081h=当前设置值，文档 3.3.2 节）
        max_vel_params = [
            self._build_sdo_write(0x6081, 0x01, self.max_velocity, 4),  # 左电机最大速度
            self._build_sdo_write(0x6081, 0x02, self.max_velocity, 4)   # 右电机最大速度
        ]
        self.add_sequence(name="最大速度配置", messages=max_vel_params)

        # 5. 步骤4：CiA402 状态机逐步切换（文档 3.2 节）
        state_switch_params = [
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, 0x00, 2),          # 禁用电压
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_SHUTDOWN, 2),  # Shutdown
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_SWITCH_ON, 2),  # Switch On
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_ENABLE_OPERATION, 2)  # Enable
        ]
        self.add_sequence(name="CiA402 状态机切换", messages=state_switch_params)

        # 6. 发送所有初始化序列
        init_success = True
        for seq_idx in range(len(self.message_sequences)):
            if not self.send_sequence(seq_idx):
                init_success = False
                print(f"❌ {self.message_sequences[seq_idx]['name']} 失败")
                break

        # 7. 步骤5：发送 NMT 启动命令
        if init_success:
            nmt_success = self.send_nmt_start(node_id=1)
            if nmt_success:
                print("✅ NMT 启动命令发送成功，PDO 通信激活")
            else:
                print("⚠️ NMT 启动命令发送失败，PDO 可能无法传输实时数据")

        # 8. 验证最终状态
        if init_success:
            status_word = self.read_status_word()
            if status_word is not None and (status_word & 0x000F) == 0x0007:
                print(f"\n🎉 位置控制初始化完成，当前状态：OPERATION ENABLED")
            else:
                print(f"\n⚠️ 位置控制初始化状态异常，6041h=0x{status_word:04X}（期望低4位=0111）")

        return init_success
# -------------------------- 力矩控制类 --------------------------
class Motor_CTL_Ser(CANMessageSequence):
    """电机转矩控制类，用于控制电机的转矩模式"""
    
    # 控制方式常量
    SYNC_CONTROL = 0      # 同步控制
    ASYNC_CONTROL = 1     # 异步控制
    
    def __init__(self, channel_handle, send_id=0x601, response_id=0x581, response_timeout=5):
        """
        初始化电机转矩控制器
        
        Args:
            channel_handle: CAN通道句柄
            send_id: 发送消息的ID
            response_id: 期望接收响应的ID
            response_timeout: 等待响应的超时时间（秒）
        """
        super().__init__(channel_handle, send_id, response_id, response_timeout)
        self.control_mode = None  # 当前控制方式
        self.torque = 0           # 当前转矩值
        
        # 转矩控制序列模板
        self.torque_control_template = [0x23, 0x71, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    def set_control_mode(self, mode):
        """
        设置控制方式
        
        Args:
            mode: 控制方式 (SYNC_CONTROL 或 ASYNC_CONTROL)
        """
        if mode not in [self.SYNC_CONTROL, self.ASYNC_CONTROL]:
            print("❌ 无效的控制方式")
            return False
        
        self.control_mode = mode
        print(f"✅ 控制方式设置为: {'同步控制' if mode == self.SYNC_CONTROL else '异步控制'}")
        return True

    def _convert_torque_to_bytes(self, torque):
        """
        将转矩值转换为字节序列
        
        Args:
            torque: 转矩值 (-32768 到 32767)
            
        Returns:
            tuple: (byte_low, byte_high) 分别对应转矩数据的低8位和高8位
        """
        # 限制转矩范围
        torque = max(-32768, min(32767, torque))
        
        if torque >= 0:
            # 正数处理
            byte_low = torque & 0xFF           # 低8位
            byte_high = (torque >> 8) & 0xFF   # 高8位
        else:
            # 负数处理（补码方式）
            torque = (1<<16) + torque
            byte_low = torque & 0xFF      # 低8位
            byte_high = (torque >> 8) & 0xFF    # 高8位
            
        return byte_low, byte_high
    
    def set_torque(self, control_mode, torque, left_torque=None, right_torque=None):
        """
        设置电机转矩
        
        Args:
            control_mode: 控制方式 (SYNC_CONTROL 或 ASYNC_CONTROL)
            torque: 转矩值，在同步控制模式下使用，或在异步控制模式下作为默认转矩
            left_torque: 左电机转矩值，仅在异步控制模式下使用
            right_torque: 右电机转矩值，仅在异步控制模式下使用
            
        Returns:
            bool: 是否设置成功
        """
        # 设置控制方式
        if not self.set_control_mode(control_mode):
            return False
        
        success = False
        if self.control_mode == self.SYNC_CONTROL:
            # 同步控制：同时控制两个电机，使用一条消息
            # 限制转矩范围
            torque = max(-32768, min(32767, torque))
            self.torque = torque
            
            # 转换转矩值为字节序列
            left_byte_low, left_byte_high = self._convert_torque_to_bytes(torque)
            right_byte_low, right_byte_high = self._convert_torque_to_bytes(torque)
            
            # 同步控制转矩序列 [23 71 60 03 ser_left_l ser_left_h ser_right_l ser_right_h]
            sync_torque_seq = [0x23, 0x71, 0x60, 0x03, 
                              left_byte_low, left_byte_high, 
                              right_byte_low, right_byte_high]
            
            print(f"🚀 发送同步控制转矩命令: {[hex(d) for d in sync_torque_seq]}")
            success = self._send_can_data(self.send_id, sync_torque_seq)
            
            if not success:
                print("❌ 同步控制转矩设置失败")
                return False
            
            # 等待响应
            print(f"⏳ 等待响应 from 0x{self.response_id:x}...")
            response_success, _ = self._receive_can_data(self.response_id, self.response_timeout)
            
            if not response_success:
                print("❌ 同步控制响应超时")
                return False
                
            print(f"✅ 同步控制转矩设置成功: {torque}")
        else:  # ASYNC_CONTROL
            # 异步控制：分别控制两个电机
            # 如果没有提供单独的左右电机转矩，则使用torque参数作为默认转矩
            if left_torque is None:
                left_torque = torque
            if right_torque is None:
                right_torque = torque
                
            # 限制转矩范围
            left_torque = max(-32768, min(32767, left_torque))
            right_torque = max(-32768, min(32767, right_torque))
            self.torque = (left_torque, right_torque)  # 保存左右电机转矩
            
            # 发送左电机转矩控制序列 [2B 71 60 01 ser_l ser_h 00 00]
            left_byte_low, left_byte_high = self._convert_torque_to_bytes(left_torque)
            # 如果是负数，第6和第7位设为FF
            left_byte_6 = 0xFF if left_torque < 0 else 0x00
            left_byte_7 = 0xFF if left_torque < 0 else 0x00
            left_torque_seq = [0x2B, 0x71, 0x60, 0x01, 
                              left_byte_low, left_byte_high, left_byte_6, left_byte_7]
            
            print(f"🚀 发送左电机转矩控制命令: {[hex(d) for d in left_torque_seq]}")
            success1 = self._send_can_data(self.send_id, left_torque_seq)
            
            if not success1:
                print("❌ 左电机转矩设置失败")
                # 不立即返回，继续尝试右电机
            else:
                # 等待响应
                print(f"⏳ 等待左电机响应 from 0x{self.response_id:x}...")
                response_success1, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success1:
                    print("❌ 左电机响应超时")
                    # 不立即返回，继续尝试右电机
            
            # 发送右电机转矩控制序列 [2B 71 60 02 ser_l ser_h 00 00]
            right_byte_low, right_byte_high = self._convert_torque_to_bytes(right_torque)
            # 如果是负数，第6和第7位设为FF
            right_byte_6 = 0xFF if right_torque < 0 else 0x00
            right_byte_7 = 0xFF if right_torque < 0 else 0x00
            right_torque_seq = [0x2B, 0x71, 0x60, 0x02, 
                               right_byte_low, right_byte_high, right_byte_6, right_byte_7]
            
            print(f"🚀 发送右电机转矩控制命令: {[hex(d) for d in right_torque_seq]}")
            success2 = self._send_can_data(self.send_id, right_torque_seq)
            
            if not success2:
                print("❌ 右电机转矩设置失败")
                # 不立即返回，继续处理
            else:
                # 等待响应
                print(f"⏳ 等待右电机响应 from 0x{self.response_id:x}...")
                response_success2, _ = self._receive_can_data(self.response_id, self.response_timeout)
                
                if not response_success2:
                    print("❌ 右电机响应超时")
                    # 不立即返回，继续处理
                
            # 只要至少有一个电机设置成功，就认为成功
            success = success1 or success2
            if success:
                print(f"✅ 异步控制转矩设置成功: 左电机 {left_torque}, 右电机 {right_torque}")
            else:
                print("❌ 异步控制转矩设置完全失败")
        
        return success

    def initialize_motor(self):
        """
        基于 CiA402 标准协议初始化转矩模式（替代固定序列）
        实现逻辑：同步/异步标志→转矩模式→转矩斜率→状态机切换→NMT 启动
        """
        if self.control_mode is None:
            print("❌ 请先调用 set_control_mode() 设置同步/异步控制方式")
            return False
        
        # 1. 清空现有序列
        self.message_sequences = []
        control_mode_name = "同步" if self.control_mode == self.SYNC_CONTROL else "异步"
        print(f"\n🚀 开始{control_mode_name}控制转矩模式初始化（CiA402 标准流程）")

        # 2. 步骤1：配置同步/异步控制标志（200Fh，文档 200Fh 定义）
        sync_flag = 1 if self.control_mode == self.SYNC_CONTROL else 0
        self.add_sequence(
            name=f"{control_mode_name}控制标志配置",
            messages=[self._build_sdo_write(0x200F, 0x00, sync_flag, 2)]
        )

        # 3. 步骤2：设置转矩工作模式（6060h=4，文档 3.5 节）
        self.add_sequence(
            name="转矩模式配置",
            messages=[self._build_sdo_write(0x6060, 0x00, self.MODE_TORQUE, 1)]  # 4=转矩模式
        )

        # 4. 步骤3：配置转矩斜率（6087h=100mA/s，文档 3.5.2 节例程参数）
        torque_slope_params = [
            self._build_sdo_write(0x6087, 0x01, 100, 4),  # 左电机转矩斜率
            self._build_sdo_write(0x6087, 0x02, 100, 4)   # 右电机转矩斜率
        ]
        self.add_sequence(name="转矩斜率配置", messages=torque_slope_params)

        # 5. 步骤4：CiA402 状态机逐步切换（文档 3.2 节）
        state_switch_params = [
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, 0x00, 2),          # 禁用电压
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_SHUTDOWN, 2),  # Shutdown
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_SWITCH_ON, 2),  # Switch On
            self._build_sdo_write(self.OD_CONTROL_WORD, 0x00, self.CONTROL_ENABLE_OPERATION, 2)  # Enable
        ]
        self.add_sequence(name="CiA402 状态机切换", messages=state_switch_params)

        # 6. 发送所有初始化序列
        init_success = True
        for seq_idx in range(len(self.message_sequences)):
            if not self.send_sequence(seq_idx):
                init_success = False
                print(f"❌ {self.message_sequences[seq_idx]['name']} 失败")
                break

        # 7. 步骤5：发送 NMT 启动命令
        if init_success:
            nmt_success = self.send_nmt_start(node_id=1)
            if nmt_success:
                print("✅ NMT 启动命令发送成功，PDO 通信激活")
            else:
                print("⚠️ NMT 启动命令发送失败，PDO 可能无法传输实时数据")

        # 8. 验证最终状态
        if init_success:
            status_word = self.read_status_word()
            if status_word is not None and (status_word & 0x000F) == 0x0007:
                print(f"\n🎉 {control_mode_name}控制转矩模式初始化完成，当前状态：OPERATION ENABLED")
            else:
                print(f"\n⚠️ {control_mode_name}控制初始化状态异常，6041h=0x{status_word:04X}（期望低4位=0111）")

        return init_success

