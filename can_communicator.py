# 说明：跨平台的 CAN/CAN FD 通信器，封装了 Windows 和 Linux 的后端差异
# 用途：为上层应用（如 GUI）提供统一的 CAN 连接、收发和监控接口
import os
import sys
import time
import platform
import threading
from typing import Optional, List, Callable, Dict, Any

# 依赖 python-can；未安装时在运行时给出明确提示
try:
    import can
    from can.bus import BusState
except ImportError:
    can = None
    BusState = None

# 动态添加项目根目录到 sys.path，便于导入
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    from CAN.CANMessageTransmitter import CANMessageTransmitter
except Exception:
    from CANMessageTransmitter import CANMessageTransmitter


def ensure_python_can():
    """运行前校验 python-can 是否已安装"""
    if can is None:
        raise ImportError("依赖库 `python-can` 未安装。请运行: pip install python-can")


def parse_bitrate_token(token: str) -> int:
    """将 '500k' / '1m' 这类速率字符串解析为整数位速率（bit/s）"""
    s = str(token).strip().lower()
    try:
        if s.isdigit():
            return int(s)
        if s.endswith('k'):
            return int(float(s[:-1]) * 1000)
        if s.endswith('m'):
            return int(float(s[:-1]) * 1000000)
    except Exception:
        pass
    raise ValueError(f"无法解析速率值: {token}")


class CANCommunicator:
    """
    一个统一的 CAN 通信类，旨在屏蔽底层操作系统和硬件后端的差异。
    - 支持 Windows (candle, gs_usb) 和 Linux (socketcan)。
    - 支持 CAN 2.0 和 CAN FD。
    - 提供后台线程用于报文接收和周期性发送。
    - 提供回调函数将接收到的消息、状态更新和统计数据传递给上层应用。
    """

    def __init__(
        self,
        on_message_received: Optional[Callable[[can.Message], None]] = None,
        on_status_changed: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        ensure_python_can()
        self.os_type = platform.system().lower()

        self.on_message_received = on_message_received
        self.on_status_changed = on_status_changed

        self.transmitter_class = CANMessageTransmitter.choose_can_device("TZCAN")
        self.device_handle = None
        self.bus: Optional[can.BusABC] = None
        self.bus_map: Dict[int, can.BusABC] = {}
        self.primary_channel: Optional[int] = None
        self.device_handles: List[Any] = []
        self.is_connected = False
        self.is_fd = False
        self.arb_rate = 0
        self.data_rate = 0
        self.bus_busy_time_accum = 0.0

        self.receive_thread: Optional[threading.Thread] = None
        self.receive_threads: Dict[int, threading.Thread] = {}
        self.periodic_send_thread: Optional[threading.Thread] = None
        self.periodic_task = None
        self.stats_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.stop_periodic_send_event = threading.Event()

        self.stats = {
            'rx_count': 0, 'tx_count': 0, 'rx_fps': 0.0, 'tx_fps': 0.0,
            'bus_load': 0.0, 'bus_state': "未连接"
        }
        self.last_stats_time = time.perf_counter()
        self.last_rx_count = 0
        self.last_tx_count = 0

    def _configure_socketcan(self, interface: str, bitrate: int, fd: bool, data_bitrate: Optional[int], sp: Optional[float], dsp: Optional[float]):
        sp_cmd = f"sample-point {sp}" if sp else ""
        dsp_cmd = f"dsample-point {dsp}" if dsp and fd else ""
        
        if fd:
            if not data_bitrate:
                raise ValueError("FD 模式需要 data_bitrate")
            cmd = f"ip link set {interface} type can bitrate {bitrate} {sp_cmd} dbitrate {data_bitrate} {dsp_cmd} fd on"
        else:
            cmd = f"ip link set {interface} type can bitrate {bitrate} {sp_cmd}"

        full_cmd = f"sudo ip link set {interface} down && sudo {cmd} && sudo ip link set {interface} up"
        print(f"执行: {full_cmd}")
        rc = os.system(full_cmd)
        if rc != 0:
            # Fallback to non-sudo
            full_cmd_nosudo = full_cmd.replace("sudo ", "")
            print(f"Sudo 失败, 尝试: {full_cmd_nosudo}")
            os.system(full_cmd_nosudo)

    def connect(self, interface: Any, backend: str, is_fd: bool, arb_rate: int, data_rate: Optional[int] = None, sp: Optional[float] = None, dsp: Optional[float] = None):
        if self.is_connected:
            print("已经连接，请先断开。")
            return

        self.is_fd = is_fd
        self.arb_rate = arb_rate
        self.data_rate = int(data_rate or 0)
        self.stop_event.clear()

        try:
            if self.os_type == 'linux':
                if backend != 'socketcan':
                    raise ValueError("Linux 平台目前仅支持 'socketcan' 后端")
                
                ifc = str(interface).strip()
                self._configure_socketcan(ifc, arb_rate, is_fd, data_rate, sp, dsp)
                channel = int(str(ifc).replace("can", ""))
                
                # 使用原生 python-can 接口进行连接
                kwargs = {'bustype': 'socketcan', 'channel': ifc, 'bitrate': arb_rate}
                if is_fd:
                    kwargs.update({'fd': True, 'data_bitrate': data_rate})
                
                _bus = can.interface.Bus(**kwargs)
                self.bus_map = {channel: _bus}
                self.primary_channel = channel
                self.bus = _bus
                print(f"✅ 原生 python-can 打开 {ifc} 成功")

            elif self.os_type == 'windows':
                if backend not in ['candle', 'gs_usb']:
                    raise ValueError("Windows 平台仅支持 'candle' 或 'gs_usb' 后端")
                if is_fd and backend != 'candle':
                    raise ValueError("Windows 平台 FD 模式仅支持 'candle' 后端")
                
                channel = int(str(interface).strip())
                print(f"INFO: 正在尝试打开通道: {channel}")

                # 尝试使用封装库打开
                try:
                    self.device_handle, _, _ = self.transmitter_class.init_can_device(
                        baud_rate=arb_rate, dbit_baud_rate=data_rate, channels=[channel],
                        backend=backend, fd=is_fd, sp=sp, dsp=dsp
                    )
                    opened_buses = self.device_handle.get('buses', {})
                    if channel in opened_buses:
                        self.bus_map = {channel: opened_buses[channel]}
                        print(f"✅ 封装库打开通道 {channel} 成功")
                    else:
                        raise RuntimeError(f"封装库未能返回通道 {channel} 的 bus 对象")
                except Exception as e:
                    print(f"INFO: 封装库打开通道失败: {e}。将回退到原生 python-can 接口。")
                    self.bus_map = {}
                    if self.device_handle:
                        try:
                            self.transmitter_class.close_can_device(self.device_handle)
                        except Exception: pass
                        self.device_handle = None

                # 如果封装库失败，则回退到 python-can
                if not self.bus_map:
                    print(f"INFO: 正在尝试使用 python-can 原生接口打开通道: {channel}")
                    try:
                        kwargs = {'bustype': backend, 'channel': channel, 'bitrate': arb_rate}
                        if is_fd:
                            kwargs.update({'fd': True, 'data_bitrate': data_rate})
                        _bus = can.interface.Bus(**kwargs)
                        self.bus_map = {channel: _bus}
                        print(f"✅ python-can 打开 {backend} 通道 {channel} 成功")
                    except Exception as e:
                        print(f"❌ python-can 打开 {backend} 通道 {channel} 失败: {e}")
                        self.bus_map = {}
                
                # 设置主通道和总线对象
                if self.bus_map:
                    self.primary_channel = channel
                    self.bus = self.bus_map.get(channel)
                else:
                    self.primary_channel = None
                    self.bus = None
            else:
                raise NotImplementedError(f"不支持的操作系统: {self.os_type}")

            if not self.bus_map:
                raise RuntimeError("未成功打开任何通道")
            self.is_connected = True
            for _, _bus in self.bus_map.items():
                try:
                    _bus.set_filters([])
                except Exception:
                    pass

            self.receive_threads = {}
            for ch in list(self.bus_map.keys()):
                t = threading.Thread(target=self._run_receiver_for_channel, args=(ch,), daemon=True)
                self.receive_threads[ch] = t
                t.start()
            self.stats_thread = threading.Thread(target=self._run_stats_updater, daemon=True)
            self.stats_thread.start()
            
            opened = ','.join(str(ch) for ch in self.bus_map.keys())
            print(f"连接成功: {backend} @ {opened}")

        except Exception as e:
            print(f"连接失败: {e}")
            self.is_connected = False
            
            # --- Start of new cleanup logic ---
            if self.bus_map:
                print("INFO: 正在关闭连接失败过程中已打开的 CAN bus 对象...")
                for ch, bus in self.bus_map.items():
                    try:
                        bus.shutdown()
                        print(f"  - ✅ 通道 {ch} 的 bus 已关闭")
                    except Exception as shutdown_e:
                        print(f"  - ❌ 关闭通道 {ch} 的 bus 时出错: {shutdown_e}")

            if self.device_handle:
                print("INFO: 正在关闭连接失败过程中已打开的主设备句柄...")
                try:
                    self.transmitter_class.close_can_device(self.device_handle)
                    print("  - ✅ 主设备句柄已关闭")
                except Exception as close_e:
                    print(f"  - ❌ 关闭主设备句柄时出错: {close_e}")
            # --- End of new cleanup logic ---

            # Reset state
            self.bus = None
            self.bus_map = {}
            self.primary_channel = None
            self.device_handle = None
            
            raise

    def disconnect(self):
        if not self.is_connected:
            return

        print("INFO: 开始断开连接...")
        self.stop_event.set()
        self.stop_periodic_send()

        # 等待所有线程结束
        for ch, t in self.receive_threads.items():
            if t.is_alive():
                print(f"  - 等待通道 {ch} 接收线程停止...")
                t.join(timeout=1)
        if self.stats_thread and self.stats_thread.is_alive():
            print("  - 等待统计线程停止...")
            self.stats_thread.join(timeout=1)
        print("INFO: 所有线程已停止。")

        # 优先关闭 python-can 的 Bus 对象，这能解决 "not properly shut down" 警告
        if self.bus_map:
            print("INFO: 正在关闭 CAN bus 对象...")
            for ch, bus in self.bus_map.items():
                try:
                    bus.shutdown()
                    print(f"  - ✅ 通道 {ch} 的 bus 已关闭")
                except Exception as e:
                    print(f"  - ❌ 关闭通道 {ch} 的 bus 时出错: {e}")

        # 如果存在封装库的设备句柄，则关闭它
        if self.device_handle:
            print("INFO: 正在关闭主设备句柄...")
            try:
                self.transmitter_class.close_can_device(self.device_handle)
                print("  - ✅ 主设备句柄已关闭")
            except Exception as e:
                print(f"  - ❌ 关闭主设备句柄时出错: {e}")

        # 重置所有状态变量
        self.is_connected = False
        self.bus = None
        self.bus_map = {}
        self.primary_channel = None
        self.device_handle = None
        self.device_handles = [] # 确保这个也清空
        self.stats['bus_state'] = "未连接"
        if self.on_status_changed:
            self.on_status_changed(self.stats)
        print("✅ 已断开连接。")

    def send_one(self, arbitration_id: int, data: List[int], is_extended_id: bool = False, is_fd: bool = False, brs: bool = False):
        if not self.is_connected or not self.bus:
            raise ConnectionError("尚未连接到 CAN 总线。")
        
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id,
            is_fd=is_fd,
            bitrate_switch=brs
        )
        self.bus.send(msg)
        self.stats['tx_count'] += 1

    def start_periodic_send(self, arbitration_id: int, data: List[int], frequency: float, is_extended_id: bool = False, is_fd: bool = False, brs: bool = False):
        if not self.is_connected:
            raise ConnectionError("尚未连接到 CAN 总线。")
        if self.periodic_send_thread and self.periodic_send_thread.is_alive():
            print("周期发送已在运行，请先停止。")
            return
        if self.periodic_task:
            print("周期发送任务已在运行，请先停止。")
            return

        self.stop_periodic_send_event.clear()
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id,
            is_fd=is_fd,
            bitrate_switch=brs
        )
        frame_time = self._estimate_frame_time(
            is_fd or self.is_fd,
            brs,
            len(data),
            self.arb_rate,
            self.data_rate
        )
        theoretical_max_fps = (1.0 / frame_time) if frame_time > 0 else frequency
        if frequency > theoretical_max_fps:
            reason = []
            if not (is_fd or self.is_fd):
                reason.append("非FD")
            if not brs:
                reason.append("BRS未启用")
            if (is_fd or self.is_fd) and (self.data_rate <= 0):
                reason.append("数据速率未知")
            suffix = ("（" + ",".join(reason) + "）") if reason else ""
            print(f"设定频率 {frequency}Hz 超出总线能力，自动限制为 {theoretical_max_fps:.1f}Hz{suffix}")
            frequency = max(1.0, theoretical_max_fps)

        period = 1.0 / float(frequency)
        try:
            # 优先使用 python-can 的周期发送任务（若后端支持）
            self.periodic_task = self.bus.send_periodic(msg, period)
            print(f"已开始以 {frequency}Hz 频率周期发送 ID: {hex(arbitration_id)}（硬件/驱动任务）")
        except Exception:
            # 后端不支持时，降级为线程循环
            self.periodic_task = None
            self.periodic_send_thread = threading.Thread(
                target=self._run_periodic_sender, args=(msg, frequency), daemon=True
            )
            self.periodic_send_thread.start()
            print(f"已开始以 {frequency}Hz 频率周期发送 ID: {hex(arbitration_id)}（线程任务）")

    def stop_periodic_send(self):
        # 停止硬件/驱动周期任务
        if self.periodic_task:
            try:
                self.periodic_task.stop()
            except Exception:
                pass
            self.periodic_task = None
        # 停止线程周期任务
        self.stop_periodic_send_event.set()
        if self.periodic_send_thread and self.periodic_send_thread.is_alive():
            self.periodic_send_thread.join(timeout=1)
        print("已停止周期发送。")

    def _run_periodic_sender(self, msg: can.Message, frequency: float):
        """高精度周期发送，校正漂移"""
        period = 1.0 / float(frequency)
        next_t = time.perf_counter()
        while not self.stop_periodic_send_event.is_set() and not self.stop_event.is_set():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
                continue
            try:
                self.bus.send(msg)
                self.stats['tx_count'] += 1
            except can.CanError as e:
                print(f"周期发送错误: {e}")
                break
            # 计算下一个目标时间点，避免累积误差
            next_t += period

    def _estimate_frame_time(self, is_fd: bool, brs: bool, data_len: int, arb_rate: int, data_rate: Optional[int]) -> float:
        if arb_rate <= 0:
            return 0.0
        if not is_fd:
            overhead = 54
            total_bits = overhead + max(0, data_len) * 8
            return float(total_bits) / float(arb_rate)
        if brs and data_rate and data_rate > 0:
            arb_overhead = 54
            crc_bits = 17 if data_len <= 16 else 21
            data_phase_overhead = 10
            data_bits = max(0, data_len) * 8 + crc_bits + data_phase_overhead
            return float(arb_overhead) / float(arb_rate) + float(data_bits) / float(data_rate)
        overhead_fd = 80
        crc_bits2 = 17 if data_len <= 16 else 21
        total_bits_fd = overhead_fd + max(0, data_len) * 8 + crc_bits2
        return float(total_bits_fd) / float(arb_rate)

    def _run_receiver_for_channel(self, channel: int):
        bus = self.bus_map.get(channel)
        while not self.stop_event.is_set() and bus:
            try:
                msg = bus.recv(timeout=0.1)
                if msg:
                    self.stats['rx_count'] += 1
                    try:
                        dl = len(getattr(msg, 'data', b''))
                        brs = bool(getattr(msg, 'bitrate_switch', False))
                        fd = bool(getattr(msg, 'is_fd', False))
                        self.bus_busy_time_accum += self._estimate_frame_time(fd, brs, dl, self.arb_rate, self.data_rate)
                    except Exception:
                        pass
                    if self.on_message_received:
                        try:
                            setattr(msg, 'channel', channel)
                        except Exception:
                            pass
                        self.on_message_received(msg)
            except can.CanError as e:
                print(f"接收错误: {e}")
                self.stats['bus_state'] = (BusState.ERROR if BusState else "错误")
                break
        print("接收线程已停止。")

    def _run_stats_updater(self):
        """后台统计线程，用于计算 FPS 和总线负载。"""
        while not self.stop_event.is_set():
            current_time = time.perf_counter()
            elapsed = current_time - self.last_stats_time
            
            if elapsed >= 1.0:
                rx_now = self.stats['rx_count']
                tx_now = self.stats['tx_count']

                self.stats['rx_fps'] = (rx_now - self.last_rx_count) / elapsed
                self.stats['tx_fps'] = (tx_now - self.last_tx_count) / elapsed
                load = (self.bus_busy_time_accum / elapsed) * 100 if elapsed > 0 else 0.0
                if load < 0:
                    load = 0.0
                if load > 100:
                    load = 100.0
                self.stats['bus_load'] = load
                state = "未连接"
                if self.is_connected and self.primary_channel is not None:
                    b = self.bus_map.get(self.primary_channel)
                    if b:
                        state = b.state
                self.stats['bus_state'] = state

                self.last_rx_count = rx_now
                self.last_tx_count = tx_now
                self.last_stats_time = current_time
                self.bus_busy_time_accum = 0.0

                if self.on_status_changed:
                    self.on_status_changed(self.stats.copy())

            time.sleep(0.2) # 更新频率
        print("统计线程已停止。")


if __name__ == '__main__':
    # 一个简单的使用示例
    def print_message(msg: can.Message):
        print(f"RX: {msg.timestamp:.4f} ID={hex(msg.arbitration_id)} DLC={msg.dlc} Data={' '.join(f'{b:02X}' for b in msg.data)}")

    def print_status(status: Dict[str, Any]):
        state_str = status['bus_state'].name if isinstance(status['bus_state'], BusState) else str(status['bus_state'])
        print(
            f"Status: RX FPS: {status['rx_fps']:.1f} | TX FPS: {status['tx_fps']:.1f} | "
            f"Bus Load: {status['bus_load']:.2f}% | State: {state_str}"
        )

    print("初始化 CAN Communicator...")
    comm = CANCommunicator(on_message_received=print_message, on_status_changed=print_status)
    
    try:
        # --- 根据您的系统选择以下配置之一 ---
        current_os = platform.system().lower()
        
        if current_os == 'linux':
            # Linux (socketcan) 示例
            print("检测到 Linux，使用 socketcan 配置...")
            config = {
                'interface': 'can0', 'backend': 'socketcan', 'is_fd': False,
                'arb_rate': 500000, 'sp': 0.8
            }
            comm.connect(**config)
            
        elif current_os == 'windows':
            # Windows (candle) 示例
            print("检测到 Windows，使用 candle 配置...")
            config = {
                'interface': 0, 'backend': 'candle', 'is_fd': False,
                'arb_rate': 500000, 'sp': 0.8
            }
            comm.connect(**config)
        else:
            print(f"不支持的操作系统: {current_os}，无法运行示例。")
            sys.exit(1)

        # 启动周期发送
        comm.start_periodic_send(0x123, [0xDE, 0xAD, 0xBE, 0xEF], frequency=10)

        print("正在收发数据... 按 Ctrl+C 退出。")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("检测到 Ctrl+C，正在关闭...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        comm.disconnect()
        print("示例运行结束。")
