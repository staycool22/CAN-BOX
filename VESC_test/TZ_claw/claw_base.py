import sys
import os
import time
import can

# 添加路径到 sys.path 以允许从父目录导入模块
current_dir = os.path.dirname(os.path.abspath(__file__))
vesc_test_dir = os.path.dirname(current_dir) # e:\CAN-BOX\VESC_test
root_dir = os.path.dirname(vesc_test_dir) # e:\CAN-BOX

if vesc_test_dir not in sys.path:
    sys.path.append(vesc_test_dir)
if root_dir not in sys.path:
    sys.path.append(root_dir)

# 导入所需的模块
try:
    from TZCANTransmitter import TZCANTransmitter, BasicConfig
except ImportError:
    # 如果从根目录不在路径中的不同上下文运行，则作为后备方案
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    from TZCANTransmitter import TZCANTransmitter, BasicConfig

try:
    from can_vesc import VESC
except ImportError:
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from can_vesc import VESC

class Claw_config:
    """
    Claw 控制器的配置类。
    存放并保护需要用到的全局变量，如 CAN 类型、波特率等。
    """
    def __init__(self):
        # CAN 配置
        self.can_type = BasicConfig.TYPE_CAN
        self.baud_rate = 500000
        self.channel_list = [0] # 默认使用通道 0
        
        # VESC 配置
        self.vesc_id = 32 # 目标 VESC ID
        
        # 控制参数
        self.open_rpm = 1000.0 # 打开时的转速 (RPM)
        self.close_rpm = -1000.0 # 关闭时的转速 (RPM)
        self.open_loop_duty = 0.2 # 开环打开时的占空比 (0.0 - 1.0)
        self.close_loop_duty = -0.2 # 开环关闭时的占空比 (-1.0 - 0.0)

class CANHandleAdapter:
    """
    适配器，使 python-can Bus 对象兼容 VESC 类的期望。
    VESC 类期望一个具有 send(id, data) 和 receive(timeout) 方法的对象。
    """
    def __init__(self, bus):
        self.bus = bus

    def send(self, arbitration_id, data):
        # VESC 通信通常使用扩展帧 ID
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(msg)

    def receive(self, timeout):
        # python-can recv 返回 Message 对象或 None
        msg = self.bus.recv(timeout)
        if msg:
            return msg.arbitration_id, list(msg.data)
        return None, None

class TZ_Claw:
    """
    Claw 的硬件抽象层。
    管理 CAN 初始化和 VESC 实例。
    """
    def __init__(self, config: Claw_config = None):
        if config is None:
            self.config = Claw_config()
        else:
            self.config = config
            
        self.m_dev = None
        self.bus = None
        self.vesc = None
        
        # 初始化硬件
        self.init_hardware()

    def init_hardware(self):
        """初始化 CAN 设备和 VESC 实例。"""
        print("正在初始化 TZ_Claw 硬件...")
        self.m_dev, ch0, ch1 = TZCANTransmitter.init_can_device(
            baud_rate=self.config.baud_rate,
            channels=self.config.channel_list,
            can_type=self.config.can_type
        )
        
        # 使用第一个初始化的通道
        self.bus = ch0
        if not self.bus:
            raise RuntimeError("初始化 CAN 总线失败 (ch0 为 None)")
            
        # 创建适配器和 VESC 实例
        adapter = CANHandleAdapter(self.bus)
        self.vesc = VESC(adapter)
        print("TZ_Claw 硬件初始化完成。")

    def close(self):
        """关闭 CAN 设备。"""
        if self.m_dev:
            TZCANTransmitter.close_can_device(self.m_dev)
            self.m_dev = None
            self.bus = None
            print("TZ_Claw 硬件已关闭。")

class claw_controller:
    """
    Claw 的高级控制器。
    提供 open/close 等控制方法。
    """
    def __init__(self, tz_claw: TZ_Claw):
        self.tz_claw = tz_claw
        self.vesc = tz_claw.vesc
        self.config = tz_claw.config

    def open(self):
        """使用转速控制(RPM)打开爪子。"""
        print(f"爪子打开 (转速: {self.config.open_rpm})")
        self.vesc.send_rpm(self.config.vesc_id, self.config.open_rpm)

    def close(self):
        """使用转速控制(RPM)关闭爪子。"""
        print(f"爪子关闭 (转速: {self.config.close_rpm})")
        self.vesc.send_rpm(self.config.vesc_id, self.config.close_rpm)

    def open_loop_open(self):
        """使用开环控制（占空比）打开爪子。"""
        print(f"爪子开环打开 (占空比: {self.config.open_loop_duty})")
        self.vesc.send_duty(self.config.vesc_id, self.config.open_loop_duty)

    def open_loop_close(self):
        """使用开环控制（占空比）关闭爪子。"""
        print(f"爪子开环关闭 (占空比: {self.config.close_loop_duty})")
        self.vesc.send_duty(self.config.vesc_id, self.config.close_loop_duty)
