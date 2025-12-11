import sys
import os
import time
import can
import threading

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
    from CANMessageTransmitter import CANMessageTransmitter
    # 通过抽象基类选择具体的设备实现
    TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
    
    # 直接从类属性获取配置，更加稳健，不再依赖 importlib
    BasicConfig = TZCANTransmitter.Config
    
except ImportError as e:
    # 如果从根目录不在路径中的不同上下文运行，则作为后备方案
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    try:
        from CANMessageTransmitter import CANMessageTransmitter
        TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
        BasicConfig = TZCANTransmitter.Config
    except ImportError:
        # 如果还是失败，回退到直接导入（旧方式），以防万一
        print(f"警告: 通过 CANMessageTransmitter 加载失败 ({e})，尝试直接导入...")
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
        self.open_rpm = 5000.0 # 打开时的转速 (RPM)
        self.close_rpm = -5000.0 # 关闭时的转速 (RPM)
        self.open_loop_duty = 0.2 # 开环打开时的占空比 (0.0 - 1.0)
        self.close_loop_duty = -0.2 # 开环关闭时的占空比 (-1.0 - 0.0)
        
        # 保护参数
        self.max_current = 3.5 # 最大电流限制 (A)

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
        
        self.running = True
        self.mode = "IDLE" # IDLE, OPEN, CLOSE, OPEN_LOOP_OPEN, OPEN_LOOP_CLOSE
        self.current_val = 0.0
        self.limit_triggered = False # 是否触发了电流限制
        
        self.thread = threading.Thread(target=self._control_loop, daemon=True)
        self.thread.start()

    def _control_loop(self):
        """控制主循环：读取状态 + 发送指令"""
        print("启动控制线程...")
        while self.running:
            try:
                # 1. 读取状态 (超时 0.02s)
                msg_id, packet = self.vesc.receive_decode(timeout=0.02)
                
                if msg_id and (msg_id & 0xFF) == self.config.vesc_id:
                    status_id = (msg_id >> 8) & 0xFF
                    if status_id == 0x09: # Status 1
                        self.current_val = packet.current
                        # 实时打印电流、转速和占空比
                        # 这里暂存数据，统一在循环末尾打印，以便包含最新的模式信息
                        pass

                # 2. 发送指令
                display_mode = self.mode # 默认显示模式

                if self.mode == "IDLE":
                    self.vesc.send_duty(self.config.vesc_id, 0.0)
                    
                elif self.mode == "OPEN":
                    self.vesc.send_rpm(self.config.vesc_id, self.config.open_rpm)
                    
                elif self.mode == "CLOSE":
                    # 如果已经触发过限流，或者当前电流超过限制，则进入限流模式并锁定
                    if self.limit_triggered or abs(self.current_val) > self.config.max_current:
                        self.limit_triggered = True # 锁定限流状态
                        
                        # 保持电流符号（通常 Close 是负转速，对应负电流，但也可能因外力变为正？）
                        # 简单起见，假设 Close 对应负电流方向（或根据 current_val 符号）
                        # 更加稳健的方式是根据 close_rpm 的符号来决定目标电流符号，或者沿用当前电流符号
                        # 这里沿用当前电流符号，如果当前电流接近0可能不稳定，但通常超限时符号是确定的
                        target = self.config.max_current if self.current_val > 0 else -self.config.max_current
                        
                        self.vesc.send_current(self.config.vesc_id, target)
                        display_mode = "CLOSE(LIM)"
                    else:
                        self.vesc.send_rpm(self.config.vesc_id, self.config.close_rpm)
                        
                elif self.mode == "OPEN_LOOP_OPEN":
                    self.vesc.send_duty(self.config.vesc_id, self.config.open_loop_duty)
                    
                elif self.mode == "OPEN_LOOP_CLOSE":
                    self.vesc.send_duty(self.config.vesc_id, self.config.close_loop_duty)
                
                # 统一打印状态
                sys.stdout.write(f"\r[状态] Cur: {self.current_val:6.2f} A | RPM: {packet.rpm:6d} | Duty: {packet.duty:5.2f} | 模式: {display_mode:10s}")
                sys.stdout.flush()

                time.sleep(0.01)

            except Exception as e:
                print(f"\n控制线程错误: {e}")
                time.sleep(0.5)

    def stop_thread(self):
        """完全停止控制线程，准备退出程序。"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        print("控制线程已退出。")

    def stop(self):
        """停止夹爪。"""
        self.mode = "IDLE"
        self.limit_triggered = False
        print("\n指令: 停止 (IDLE)")

    def open(self):
        """使用转速控制(RPM)打开爪子。"""
        self.mode = "OPEN"
        self.limit_triggered = False
        print(f"\n指令: 夹爪打开 (转速: {self.config.open_rpm})")

    def close(self):
        """使用转速控制(RPM)关闭爪子，并开启电流监控。"""
        self.mode = "CLOSE"
        self.limit_triggered = False
        print(f"\n指令: 夹爪关闭 (转速: {self.config.close_rpm})，开启电流监控")

    def open_loop_open(self):
        """使用开环控制（占空比）打开爪子。"""
        self.mode = "OPEN_LOOP_OPEN"
        self.limit_triggered = False
        print(f"\n指令: 夹爪开环打开 (占空比: {self.config.open_loop_duty})")

    def open_loop_close(self):
        """使用开环控制（占空比）关闭爪子。"""
        self.mode = "OPEN_LOOP_CLOSE"
        self.limit_triggered = False
        print(f"\n指令: 夹爪开环关闭 (占空比: {self.config.close_loop_duty})")
