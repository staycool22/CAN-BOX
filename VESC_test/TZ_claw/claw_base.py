import sys
import os
import time
import can
import threading

# 添加路径到 sys.path 以允许从父目录导入模块
# 确保项目根目录在 path 中
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", ".."))

if project_root not in sys.path:
    sys.path.append(project_root)

# 导入所需的模块
try:
    from CAN.CANMessageTransmitter import CANMessageTransmitter
    # 通过抽象基类选择具体的设备实现
    TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
    
    # 直接从类属性获取配置，更加稳健，不再依赖 importlib
    BasicConfig = TZCANTransmitter.Config
    
except ImportError as e:
    print(f"Import Error: {e}")
    raise

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
        self.baud_rate = 500000
        self.channel_list = [0] # 默认使用通道 0
        
        # CANFD 配置 (默认不启用)
        self.use_canfd = False
        self.data_bitrate = 2000000
        self.sp = 75.0
        self.dsp = 80.0
        
        # VESC 配置
        self.vesc_id = 32 # 目标 VESC ID

class CANHandleAdapter:
    """
    适配器，使 python-can Bus 对象兼容 VESC 类的期望。
    VESC 类期望一个具有 send(id, data) 和 receive(timeout) 方法的对象。
    """
    def __init__(self, bus, use_canfd=False):
        self.bus = bus
        self.use_canfd = use_canfd

    def send(self, arbitration_id, data):
        # VESC 通信通常使用扩展帧 ID
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=True,
            is_fd=self.use_canfd,
            bitrate_switch=self.use_canfd
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
    夹爪控制类。
    负责初始化硬件、发送控制指令以及解析反馈数据。
    """
    def __init__(self, config: Claw_config = None, bus=None):
        if config is None:
            self.config = Claw_config()
        else:
            self.config = config
            
        self.m_dev = None
        self.bus = bus
        self.vesc = None
        
        # 初始化硬件
        if self.bus is None:
            self.init_hardware()
        else:
            # 使用传入的 bus
            print("TZ_Claw 使用外部传入的 CAN 总线。")
            # 适配器需要知道是否使用 CANFD
            actual_use_canfd = self.config.use_canfd
            adapter = CANHandleAdapter(self.bus, use_canfd=actual_use_canfd)
            self.vesc = VESC(adapter)

    def init_hardware(self):
        """初始化 CAN 设备和 VESC 实例。"""
        print("正在初始化 TZ_Claw 硬件...")
        
        # 根据 use_canfd 自动选择 can_type
        actual_can_type = BasicConfig.TYPE_CANFD if self.config.use_canfd else BasicConfig.TYPE_CAN
        
        self.m_dev, ch0, ch1 = TZCANTransmitter.init_can_device(
            baud_rate=self.config.baud_rate,
            channels=self.config.channel_list,
            can_type=actual_can_type,
            # CANFD 参数
            dbit_baud_rate=self.config.data_bitrate,
            fd=self.config.use_canfd,
            sp=self.config.sp,
            dsp=self.config.dsp
        )
        
        # 使用第一个初始化的通道
        self.bus = ch0
        if not self.bus:
            raise RuntimeError("初始化 CAN 总线失败 (ch0 为 None)")
            
        # 创建适配器和 VESC 实例
        adapter = CANHandleAdapter(self.bus, use_canfd=self.config.use_canfd)
        self.vesc = VESC(adapter)
        print("TZ_Claw 硬件初始化完成。")

    def close(self):
        """关闭 CAN 设备。"""
        if self.m_dev:
            TZCANTransmitter.close_can_device(self.m_dev)
            self.m_dev = None
            self.bus = None
            print("TZ_Claw 硬件已关闭。")
        elif self.bus:
            # 外部传入的 bus，不在此处关闭物理设备，仅断开引用
            self.bus = None
            print("TZ_Claw (外部总线) 已断开连接。")

class claw_controller:
    """
    Claw 的高级控制器。
    提供 open/close 等基础控制方法。
    不包含后台线程，需由上层应用负责循环调用以维持心跳。
    """
    def __init__(self, tz_claw: TZ_Claw):
        self.tz_claw = tz_claw
        self.vesc = tz_claw.vesc
        self.config = tz_claw.config

    def get_status(self, timeout=0.02):
        """
        读取 VESC 状态。
        :return: (msg_id, packet)
        """
        return self.vesc.receive_decode(timeout=timeout)

    def stop(self):
        """停止夹爪 (发送 Duty 0)。"""
        self.vesc.send_duty(self.config.vesc_id, 0.0)

    def open(self, rpm):
        """
        使用转速控制(RPM)打开爪子。
        :param rpm: 目标转速
        """
        self.vesc.send_rpm(self.config.vesc_id, rpm)

    def close(self, rpm):
        """
        使用转速控制(RPM)关闭爪子。
        :param rpm: 目标转速 (内部会自动取反)
        """
        self.vesc.send_rpm(self.config.vesc_id, -rpm)

    def open_current(self, current):
        """
        使用电流控制打开爪子。
        :param current: 目标电流 (A)
        """
        self.vesc.send_current(self.config.vesc_id, current)

    def close_current(self, current):
        """
        使用电流控制关闭爪子。
        :param current: 目标电流 (A) (内部会自动取反)
        """
        self.vesc.send_current(self.config.vesc_id, -current)

    def open_loop_open(self, duty):
        """使用开环控制（占空比）打开爪子。"""
        self.vesc.send_duty(self.config.vesc_id, duty)

    def open_loop_close(self, duty):
        """使用开环控制（占空比）关闭爪子 (内部会自动取反)。"""
        self.vesc.send_duty(self.config.vesc_id, -duty)
