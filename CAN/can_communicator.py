# 说明：跨平台的 CAN/CAN FD 通信器，封装了 Windows 和 Linux 的后端差异
# 用途：为上层应用（如 GUI）提供统一的 CAN 连接、收发和监控接口
import os
import sys
import time
import platform
import threading
from typing import Optional, List, Callable, Dict, Any
import queue
from collections import deque, defaultdict

# 依赖 python-can；未安装时在运行时给出明确提示
try:
    import can
    import can.broadcastmanager
    from can.bus import BusState
except ImportError:
    can = None
    BusState = None

# 动态添加项目根目录到 sys.path，便于导入
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    # 优先尝试相对导入 (当作为包导入时)
    from .CANMessageTransmitter import CANMessageTransmitter
except ImportError:
    try:
        # 尝试通过包名导入
        from CAN.CANMessageTransmitter import CANMessageTransmitter
    except ImportError:
         pass


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
        on_message_batch_received: Optional[Callable[[List[can.Message]], None]] = None,
    ):
        ensure_python_can()
        self.os_type = platform.system().lower()

        self.on_message_received = on_message_received
        self.on_status_changed = on_status_changed
        self.on_message_batch_received = on_message_batch_received

        try:
            self.transmitter_class = CANMessageTransmitter.choose_can_device("TZCAN")
        except Exception:
            # Fallback for Linux or if TZCAN is not available
            self.transmitter_class = None
        self.device_handle = None
        self.bus: Optional[can.BusABC] = None
        self.bus_map: Dict[int, can.BusABC] = {}
        self.primary_channel: Optional[int] = None
        self.channel_name_map: Dict[int, str] = {}
        self.is_connected = False
        self.is_fd = False
        self.arb_rate = 0
        self.data_rate = 0
        self.bus_busy_time_accum = 0.0

        self.receive_threads: Dict[int, threading.Thread] = {}
        
        # Periodic send tasks map: {channel_id: task_obj_or_thread}
        self.periodic_tasks: Dict[int, Any] = {} 
        self.periodic_threads: Dict[int, threading.Thread] = {}
        self.periodic_stop_events: Dict[int, threading.Event] = {}
        self.periodic_task_info: Dict[int, Dict[str, Any]] = {}
        
        self.stats_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.echo_filter_window = 0.05
        self.echo_queues = defaultdict(queue.SimpleQueue) 

        self.rx_listeners: Dict[int, Any] = {}
        self.rx_notifiers: Dict[int, Any] = {}
        self.batch_emit_interval = 0.01
        self.max_batch_size = 256
        # 默认开启回环过滤；逐帧推送以保证队列写入与接收频率一致
        self.echo_filter_enabled = True
        self.push_every_message = True
        
        
        self.stats = {
            'rx_count': 0, 'tx_count': 0, 'rx_fps': 0.0, 'tx_fps': 0.0,
            'bus_load': 0.0, 'bus_state': "未连接",
            'channels': {}
        }
        self.last_stats_time = time.perf_counter()
        self.last_rx_count = 0
        self.last_tx_count = 0
        
        # Per-channel stats helpers
        self.channel_stats = {} # {ch: {'rx_count': 0, 'tx_count': 0, 'rx_fps': 0.0, 'tx_fps': 0.0, 'bus_load': 0.0}}
        self.bus_busy_time_accum_map = {} # {ch: 0.0}
        self.last_rx_count_map = {}
        self.last_tx_count_map = {}

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

    @staticmethod
    def list_devices(backend: str) -> List[str]:
        """
        列出可用设备通道，返回格式化字符串列表
        例如: ["0 (SN:8888001:0)", "1 (SN:8888002:0)"]
        """
        try:
            TZCANTransmitter = CANMessageTransmitter.choose_can_device("TZCAN")
            channels = TZCANTransmitter.get_all_channels(backend)
            result = []
            for flat_idx, sn, ch_idx in channels:
                if backend == 'candle':
                    result.append(f"{flat_idx} (SN:{sn}:{ch_idx})")
                else:
                    result.append(f"{flat_idx}")
            return result
        except Exception:
            # Fallback
            return [str(i) for i in range(8)]

    def connect(self, interface: Any, backend: str, is_fd: bool, arb_rate: int, data_rate: Optional[int] = None, sp: Optional[float] = None, dsp: Optional[float] = None, channel_configs: Optional[Dict] = None):
        if self.is_connected:
            print("已经连接，请先断开。")
            return

        self.is_fd = is_fd
        self.arb_rate = arb_rate
        self.data_rate = int(data_rate or 0)
        self.stop_event.clear()

        # 解析 channels (用于 Windows/Candle 多通道)
        channels = []
        if isinstance(interface, (list, tuple)):
            channels = [int(x) for x in interface]
        elif isinstance(interface, str) and ',' in interface:
            channels = [int(x.strip()) for x in interface.split(',') if x.strip()]
        else:
            try:
                channels = [int(str(interface).strip())]
            except ValueError:
                pass

        try:
            if self.os_type == 'linux':
                if backend != 'socketcan':
                    raise ValueError("Linux 平台目前仅支持 'socketcan' 后端")
                
                # 确定要打开的通道列表 (indices)
                target_channels = []
                if channels:
                    target_channels = channels
                else:
                    # 尝试解析 "can0" -> 0
                    ifc_str = str(interface).strip()
                    if ifc_str.startswith("can"):
                        try:
                            target_channels = [int(ifc_str.replace("can", ""))]
                        except:
                            pass
                    # 如果解析失败且没有 channels 列表，尝试直接使用 interface 字符串 (作为单通道)
                    # 但为了统一管理，最好都映射为 int 索引。
                    # 如果 interface 是 "vcan0"，int转换会失败。
                    # 暂时假设都是 canX 格式，或者 interface 已经被解析为 channels list。
                
                if not target_channels:
                     # Fallback: try to treat interface as a single channel index if possible
                     try:
                         target_channels = [int(str(interface).strip())]
                     except:
                         pass

                if not target_channels and isinstance(interface, str):
                     # Last resort: just try to open it as is, mapped to index 0 if unknown
                     # But this breaks the multi-channel map structure if we don't have unique IDs.
                     # We will rely on `channels` being correctly populated by the caller (main_gui).
                     print(f"Warning: Could not parse channels from {interface}, assuming can0")
                     target_channels = [0]

                print(f"INFO: Linux SocketCAN 准备打开通道: {target_channels}")

                for ch_idx in target_channels:
                    ifc = f"can{ch_idx}"
                    
                    # 获取该通道的特定配置
                    c_arb = arb_rate
                    c_data = data_rate
                    c_sp = sp
                    c_dsp = dsp
                    
                    if channel_configs and ch_idx in channel_configs:
                        cfg = channel_configs[ch_idx]
                        c_arb = cfg.get('arb_rate', c_arb)
                        c_data = cfg.get('data_rate', c_data)
                        c_sp = cfg.get('sp', c_sp)
                        c_dsp = cfg.get('dsp', c_dsp)

                    try:
                        self._configure_socketcan(ifc, c_arb, is_fd, c_data, c_sp, c_dsp)
                        
                        # 使用原生 python-can 接口进行连接
                        # Linux: 关闭内核层本机回显，避免收到自身发送的回环帧
                        kwargs = {'bustype': 'socketcan', 'channel': ifc, 'bitrate': c_arb, 'receive_own_messages': False}
                        if is_fd:
                            kwargs.update({'fd': True, 'data_bitrate': c_data})
                        
                        _bus = can.interface.Bus(**kwargs)
                        self.bus_map[ch_idx] = _bus
                        print(f"✅ 原生 python-can 打开 {ifc} 成功 (Arb: {c_arb}, Data: {c_data})")
                    except Exception as e:
                        print(f"❌ 打开 {ifc} 失败: {e}")
                        # Continue trying other channels? Or fail hard?
                        # Fail hard is safer to avoid partial state confusion
                        raise e

                if self.bus_map:
                    self.primary_channel = sorted(self.bus_map.keys())[0]
                    self.bus = self.bus_map[self.primary_channel]
                    self.channel_name_map = {ch: f"can{ch}" for ch in self.bus_map}
                else:
                     raise RuntimeError("未成功打开任何 SocketCAN 通道")

            elif self.os_type == 'windows':
                if backend not in ['candle', 'gs_usb']:
                    raise ValueError("Windows 平台仅支持 'candle' 或 'gs_usb' 后端")
                if is_fd and backend != 'candle':
                    raise ValueError("Windows 平台 FD 模式仅支持 'candle' 后端")
                
                if not channels:
                     raise ValueError(f"无效的接口参数: {interface}")
                
                print(f"INFO: 正在尝试打开通道: {channels}")

                # 尝试使用封装库打开
                try:
                    self.device_handle, _, _ = self.transmitter_class.init_can_device(
                        baud_rate=arb_rate, dbit_baud_rate=data_rate, channels=channels,
                        backend=backend, fd=is_fd, sp=sp, dsp=dsp, channel_configs=channel_configs
                    )
                    opened_buses = self.device_handle.get('buses', {})
                    
                    # 检查是否真的有 bus
                    if opened_buses:
                        self.bus_map = opened_buses
                        print(f"✅ 封装库打开通道 {list(opened_buses.keys())} 成功")
                    else:
                        raise RuntimeError(f"封装库未能返回任何 bus 对象")
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    print(f"INFO: 封装库打开通道失败: {e}。将回退到原生 python-can 接口。")
                    self.bus_map = {}
                    if self.device_handle:
                        try:
                            self.transmitter_class.close_can_device(self.device_handle)
                        except Exception: pass
                        self.device_handle = None

                # 如果封装库失败，则回退到 python-can
                if not self.bus_map:
                    print(f"INFO: 正在尝试使用 python-can 原生接口逐个打开通道: {channels}")
                    for ch in channels:
                        try:
                            kwargs = {'bustype': backend, 'channel': ch, 'bitrate': arb_rate}
                            if is_fd:
                                kwargs.update({'fd': True, 'data_bitrate': data_rate})
                            _bus = can.interface.Bus(**kwargs)
                            self.bus_map[ch] = _bus
                            print(f"✅ python-can 打开 {backend} 通道 {ch} 成功")
                        except Exception as e:
                            print(f"❌ python-can 打开 {backend} 通道 {ch} 失败: {e}")
                
                # 设置主通道和总线对象
                if self.bus_map:
                    self.primary_channel = sorted(self.bus_map.keys())[0]
                    self.bus = self.bus_map[self.primary_channel]
                    self.channel_name_map = {ch: str(ch) for ch in self.bus_map}
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
            # Initialize stats maps
            self.channel_stats = {ch: {'rx_count': 0, 'tx_count': 0, 'rx_fps': 0.0, 'tx_fps': 0.0, 'bus_load': 0.0} for ch in self.bus_map}
            self.bus_busy_time_accum_map = {ch: 0.0 for ch in self.bus_map}
            self.last_rx_count_map = {ch: 0 for ch in self.bus_map}
            self.last_tx_count_map = {ch: 0 for ch in self.bus_map}

            # 消息接收线程启动：识别共享 Bus 并启动合并接收线程
            # 如果多个通道共享同一个 Bus 对象（如新版 Candle 驱动），只启动一个接收线程以避免竞争
            unique_buses = {} # id(bus) -> (bus, [channel_list])
            for ch, bus in self.bus_map.items():
                bid = id(bus)
                if bid not in unique_buses:
                    unique_buses[bid] = (bus, [])
                unique_buses[bid][1].append(ch)

            for bid, (bus, ch_list) in unique_buses.items():
                # 使用列表中的第一个通道作为线程 key (仅用于管理)
                key_ch = ch_list[0]
                t = threading.Thread(target=self._run_shared_receiver, args=(bus, ch_list), daemon=True)
                self.receive_threads[key_ch] = t
                t.start()
                print(f"已启动接收线程 (BusID: {bid}, Channels: {ch_list})")
                
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

        if self.rx_notifiers:
            for ch, n in self.rx_notifiers.items():
                try:
                    n.stop()
                except Exception:
                    pass
        self.rx_notifiers = {}
        self.rx_listeners = {}

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
        self.channel_name_map = {}
        self.device_handle = None
        self.stats['bus_state'] = "未连接"
        if self.on_status_changed:
            self.on_status_changed(self.stats)
        
        # 强制等待一段时间，确保底层驱动（特别是 candle）完成资源释放
        # 避免立即重连导致崩溃
        time.sleep(0.5)
        
        print("✅ 已断开连接。")

    def send_one(self, arbitration_id: int, data: List[int], is_extended_id: bool = False, is_fd: bool = False, brs: bool = False, channel: Optional[int] = None):
        if not self.is_connected:
            raise ConnectionError("尚未连接到 CAN 总线。")
        
        bus = self.bus
        if channel is not None and self.bus_map:
             bus = self.bus_map.get(channel, self.bus)

        if not bus:
             raise ConnectionError("找不到可用的 CAN 总线对象。")
        
        # 显式设置目标通道，支持共享 Bus
        target_ch = channel if channel is not None else self.primary_channel
        
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id,
            is_fd=is_fd,
            bitrate_switch=brs,
            channel=target_ch  # 关键修复：确保多通道模式下发送到正确通道
        )
        bus.send(msg)
        self.stats['tx_count'] += 1
        
        if target_ch in self.channel_stats:
            self.channel_stats[target_ch]['tx_count'] += 1

        try:
            # Accumulate bus load for TX
            dl = len(data)
            frame_time = self._estimate_frame_time(is_fd, brs, dl, self.arb_rate, self.data_rate)
            self.bus_busy_time_accum += frame_time
            if target_ch in self.bus_busy_time_accum_map:
                self.bus_busy_time_accum_map[target_ch] += frame_time
        except Exception:
            pass

        if self.echo_filter_enabled:
            try:
                tx_key = (msg.arbitration_id, bool(msg.is_extended_id), bool(getattr(msg, 'is_fd', False)), bytes(getattr(msg, 'data', b'')))
                self.echo_queues[target_ch].put((tx_key, time.perf_counter()))
            except Exception:
                pass
        try:
            # Use channel argument for channel name if provided, otherwise primary
            ch_idx = channel if channel is not None else self.primary_channel
            chan_name = self.channel_name_map.get(ch_idx, str(ch_idx) if ch_idx is not None else "")
            setattr(msg, 'channel', f"TX:{chan_name}")
        except Exception:
            pass
        if self.on_message_received:
            try:
                self.on_message_received(msg)
            except Exception:
                pass

    def start_periodic_send(self, arbitration_id: int, data: List[int], frequency: float, is_extended_id: bool = False, is_fd: bool = False, brs: bool = False, channel: Optional[int] = None):
        if not self.is_connected:
            raise ConnectionError("尚未连接到 CAN 总线。")
        
        target_ch = channel if channel is not None else self.primary_channel
        if target_ch is None:
             raise ConnectionError("无法确定目标通道。")

        if self.is_periodic_sending(target_ch):
            print(f"通道 {target_ch} 的周期发送已在运行，请先停止。")
            return

        if target_ch not in self.periodic_stop_events:
            self.periodic_stop_events[target_ch] = threading.Event()
        self.periodic_stop_events[target_ch].clear()
        
        bus = self.bus_map.get(target_ch)
        if not bus:
             raise ConnectionError(f"找不到通道 {target_ch} 的 CAN 总线对象。")

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
        
        # 决策：使用 python-can 的任务还是自定义优化线程
        # 关于多进程 (Multiprocessing) 的说明：
        # 虽然 Python 的多进程可以避开 GIL，但在此场景下无法使用。
        # 原因：底层 CAN 驱动 (如 Candle/SocketCAN) 持有的设备句柄 (Handle/File Descriptor) 
        # 是进程独有的 C 指针或内核对象，无法简单地跨进程共享或传递。
        # 且 USB 设备通常不支持被多个进程同时打开。
        # 
        # 当前方案：使用多线程 (Threading)。
        # 1. 每个通道分配一个独立的发送线程 (periodic_threads[ch])。
        # 2. 底层 ctypes 调用通常会释放 GIL，允许并发执行。
        # 3. 如果遇到阻塞，通常是 USB 硬件或驱动层的物理限制，而非 Python 线程限制。
        
        # 如果频率较高 (>500Hz)，python-can 的 ThreadBasedCyclicSendTask (基于 sleep) 可能精度不足
        # 此时优先使用自定义的 Spin-Wait 线程
        use_custom_thread = False
        
        # 尝试使用 python-can 接口
        try:
            task = bus.send_periodic(msg, period)
            
            # 检查是否为软件模拟任务
            is_software_task = isinstance(task, can.broadcastmanager.ThreadBasedCyclicSendTask)
            
            if is_software_task and frequency > 500:
                # 频率过高，软件任务不满足要求，停止它并转为自定义线程
                task.stop()
                use_custom_thread = True
            else:
                self.periodic_tasks[target_ch] = task
                self.periodic_task_info[target_ch] = {
                    'freq': frequency,
                    'frame_time': frame_time
                }
                task_type = "软件模拟任务" if is_software_task else "硬件/驱动任务"
                print(f"已开始在通道 {target_ch} 以 {frequency}Hz 频率周期发送 ID: {hex(arbitration_id)}（{task_type}）")
        except Exception:
            use_custom_thread = True

        if use_custom_thread:
            t = threading.Thread(
                target=self._run_periodic_sender, args=(msg, frequency, bus, target_ch), daemon=True
            )
            self.periodic_threads[target_ch] = t
            t.start()
            print(f"已开始在通道 {target_ch} 以 {frequency}Hz 频率周期发送 ID: {hex(arbitration_id)}（高精度线程任务）")

    def start_burst_send(self, arbitration_id: int, data: List[int], frequency: float, count: int, is_extended_id: bool = False, is_fd: bool = False, brs: bool = False, channel: Optional[int] = None, on_finish: Optional[Callable[[], None]] = None) -> Callable[[], None]:
        """
        开始发送突发（固定数量）消息。
        返回一个 stop() 函数，用于提前停止发送。
        """
        if not self.is_connected:
            raise ConnectionError("尚未连接到 CAN 总线。")
        
        target_ch = channel if channel is not None else self.primary_channel
        bus = self.bus_map.get(target_ch)
        if not bus:
             raise ConnectionError(f"找不到通道 {target_ch} 的 CAN 总线对象。")

        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=is_extended_id,
            is_fd=is_fd,
            bitrate_switch=brs
        )

        stop_event = threading.Event()
        
        # 定义停止函数
        def stop():
            stop_event.set()
            
        # 启动发送线程
        t = threading.Thread(
            target=self._run_periodic_sender,
            args=(msg, frequency, bus, target_ch),
            kwargs={
                'stop_event': stop_event,
                'limit': count,
                'on_finish': on_finish
            },
            daemon=True
        )
        t.start()
        
        return stop

    def stop_periodic_send(self, channel: Optional[int] = None):
        if channel is None:
            # Stop all
            targets = list(self.periodic_tasks.keys()) + list(self.periodic_threads.keys())
            targets = set(targets)
            for ch in targets:
                self.stop_periodic_send(ch)
            if not targets:
                print("没有正在运行的周期发送任务。")
            else:
                print("已停止所有周期发送。")
            return

        # Stop specific channel
        if channel in self.periodic_tasks:
            try:
                self.periodic_tasks[channel].stop()
            except Exception:
                pass
            del self.periodic_tasks[channel]
            if channel in self.periodic_task_info:
                del self.periodic_task_info[channel]
        
        if channel in self.periodic_stop_events:
            self.periodic_stop_events[channel].set()
            
        if channel in self.periodic_threads:
            t = self.periodic_threads[channel]
            if t.is_alive():
                t.join(timeout=1)
            del self.periodic_threads[channel]
        
        # Cleanup event
        if channel in self.periodic_stop_events:
             del self.periodic_stop_events[channel]
             
        print(f"已停止通道 {channel} 的周期发送。")

    def is_periodic_sending(self, channel: Optional[int] = None) -> bool:
        if channel is None:
             channel = self.primary_channel
        
        if channel is None: return False
        
        return (channel in self.periodic_tasks) or \
               (channel in self.periodic_threads and self.periodic_threads[channel].is_alive())

    def _run_periodic_sender(self, msg: can.Message, frequency: float, bus: Optional[can.BusABC] = None, channel: Optional[int] = None, stop_event: Optional[threading.Event] = None, limit: int = 0, on_finish: Optional[Callable] = None):
        """高精度周期发送，校正漂移，优化高频性能"""
        if bus is None: bus = self.bus
        period = 1.0 / float(frequency)
        next_t = time.perf_counter()
        
        target_ch = channel if channel is not None else self.primary_channel
        
        # 如果没有提供独立的 stop_event，则使用周期发送管理的 stop_event
        if stop_event is None:
            stop_event = self.periodic_stop_events.get(target_ch)
        
        if not stop_event:
             # Fallback or error
             if on_finish: on_finish()
             return

        # --- 性能优化预计算 ---
        dl = len(getattr(msg, 'data', b''))
        brs = bool(getattr(msg, 'bitrate_switch', False))
        fd = bool(getattr(msg, 'is_fd', False))
        is_ext = bool(msg.is_extended_id)
        
        # 预计算帧时间
        frame_time = self._estimate_frame_time(fd, brs, dl, self.arb_rate, self.data_rate)
        
        # 预设置通道名称 (用于UI显示)
        ch_idx = target_ch
        chan_name = self.channel_name_map.get(ch_idx, str(ch_idx) if ch_idx is not None else "")
        ui_channel_str = f"TX:{chan_name}"
        setattr(msg, 'channel', ui_channel_str)

        # 预计算 echo key (用于 recent_tx)
        tx_key = (msg.arbitration_id, is_ext, fd, bytes(getattr(msg, 'data', b'')))

        # 优化策略
        use_spin_wait = period < 0.002 # 周期小于2ms时使用忙等待
        last_ui_update = 0.0
        
        # 本地变量缓存
        stats = self.stats
        channel_stats = self.channel_stats
        bus_busy_time_accum_map = self.bus_busy_time_accum_map
        # tx_lock = self.tx_lock # 冗余：已移除全局锁，改用 echo_queue
        # echo_queue for this channel (lock-free IPC)
        echo_queue = self.echo_queues[target_ch]
        
        # 优化：绕过 VirtualBus 直接调用 shared_bus 以减少 Python 栈帧开销
        send_func = bus.send
        virtual_bus_channel = None
        target_bus = bus
        
        if hasattr(bus, 'shared_bus') and hasattr(bus, 'channel'):
            try:
                # 确认是 _VirtualCandleBus
                target_bus = bus.shared_bus
                virtual_bus_channel = bus.channel
                send_func = target_bus.send
            except:
                pass
        
        # 极速优化：尝试获取底层 Ctypes 函数与句柄 (Candle 专用)
        # 目的：绕过 python-can 的 Message 转换和 Python 方法调用开销
        native_c_send = None
        native_handle = None
        native_frame = None
        
        try:
            # 1. 确保 channel 正确
            if virtual_bus_channel is not None:
                msg.channel = virtual_bus_channel
            
            # 2. 尝试调用 _build_frame 生成 ctypes 结构体
            # 大多数 python-can backend (如 candle, pcan) 都有这个内部方法
            build_frame_fn = getattr(target_bus, '_build_frame', None)
            if build_frame_fn:
                native_frame = build_frame_fn(msg)
                
                # 3. 寻找底层 device 对象
                # candle_bus 通常有 self._dev 或 self.dev
                drv_dev = getattr(target_bus, '_dev', None) or getattr(target_bus, 'dev', None)
                if drv_dev:
                    # 4. 寻找 C 函数和 Handle
                    # candle_driver: self._candle_dll.candle_frame_send(self._dev, frame)
                    # self._dev 在这里是 handle
                    dll = getattr(drv_dev, '_candle_dll', None)
                    handle = getattr(drv_dev, '_dev', None)
                    
                    if dll and handle:
                        c_send_fn = getattr(dll, 'candle_frame_send', None)
                        if c_send_fn:
                            native_c_send = c_send_fn
                            native_handle = handle
                            print(f"INFO: 通道 {target_ch} 已启用 C-Level 极速发送模式")
        except Exception as e:
            print(f"DEBUG: 无法启用极速模式: {e}")
            native_c_send = None

        on_msg = self.on_message_received
        
        # 预加载当前通道的统计字典，避免循环内哈希查找
        if target_ch not in channel_stats:
             channel_stats[target_ch] = {'rx_count': 0, 'tx_count': 0, 'rx_fps': 0.0, 'tx_fps': 0.0, 'bus_load': 0.0}
        my_ch_stats = channel_stats[target_ch]
        
        if target_ch not in bus_busy_time_accum_map:
             bus_busy_time_accum_map[target_ch] = 0.0
        # float 不可变，无法直接引用更新，只能在循环内操作字典

        # 尝试提高当前线程/进程优先级
        try:
            sys_platform = platform.system().lower()
            if sys_platform == 'windows':
                import ctypes
                # REALTIME_PRIORITY_CLASS = 0x00000100
                # HIGH_PRIORITY_CLASS = 0x00000080
                # 设置进程优先级为 HIGH (谨慎使用 REALTIME)
                ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x00000080)
            elif sys_platform == 'linux':
                # Linux: 尝试降低 nice 值以提高优先级 (需要 root 权限)
                # SocketCAN 模式下，每个通道对应独立的 Socket 文件描述符
                # 线程并发写入不同的 Socket 是真正的并行操作 (由内核调度)
                try:
                    os.nice(-10)
                except Exception:
                    pass
        except:
            pass

        sent_count = 0
        while not stop_event.is_set() and not self.stop_event.is_set():
            # 检查 limit
            if limit > 0 and sent_count >= limit:
                break

            now = time.perf_counter()
            remaining = next_t - now
            
            if remaining > 0:
                if use_spin_wait or remaining < 0.002:
                    # 忙等待以获得微秒级精度，但适当让出 GIL
                    while True:
                        t = time.perf_counter()
                        if t >= next_t:
                            break
                        if next_t - t > 0.0002: # 增加到 200us 释放阈值，减少上下文切换频率
                            time.sleep(0)
                else:
                    time.sleep(remaining)
            
            # 补偿机制：如果严重滞后 (>10ms)，则跳过等待直接追赶，重置 next_t
            if now - next_t > 0.01:
                next_t = now

            try:
                # 诊断埋点：测量底层驱动的阻塞耗时
                t_before_send = time.perf_counter()
                
                if native_c_send:
                    # 极速模式：直接调用 C 函数
                    native_c_send(native_handle, native_frame)
                else:
                    # 普通模式
                    if virtual_bus_channel is not None:
                        # 对于 VirtualBus 包装器，使用内部通道 ID
                        msg.channel = virtual_bus_channel
                    else:
                        # 优化：为 CandleBus 传入整数通道 ID 以启用快速路径 (避免字符串解析)
                        msg.channel = target_ch
                    
                    send_func(msg)
                    
                    # 恢复 UI 显示用的字符串通道名
                    msg.channel = ui_channel_str
                
                t_send_cost = time.perf_counter() - t_before_send
                # 性能告警：如果驱动阻塞时间超过周期的 80% (且周期 < 10ms)，说明硬件或驱动已饱和
                if t_send_cost > period * 0.8 and period < 0.01:
                    # 告警限流：每秒最多打印一次
                    if time.perf_counter() - last_ui_update > 1.0:
                        print(f"WARN: Channel {target_ch} driver send blocked for {t_send_cost*1000:.2f}ms (Period: {period*1000:.2f}ms)")
            except can.CanError as e:
                print(f"周期发送错误: {e}")
                break
            
            sent_count += 1
            # --- 快速统计更新 ---
            stats['tx_count'] += 1
            my_ch_stats['tx_count'] += 1

            self.bus_busy_time_accum += frame_time
            if target_ch in bus_busy_time_accum_map:
                bus_busy_time_accum_map[target_ch] += frame_time

            # --- Echo 记录 (用于过滤回环) ---
            if self.echo_filter_enabled:
                echo_queue.put((tx_key, time.perf_counter()))

            # --- UI 更新 (限流 10Hz) ---
            if on_msg:
                now_ui = time.perf_counter()
                if now_ui - last_ui_update > 0.1:
                    try:
                        on_msg(msg)
                        last_ui_update = now_ui
                    except Exception:
                        pass

            next_t += period

        if on_finish:
            try:
                on_finish()
            except:
                pass


    def _estimate_frame_time(self, is_fd: bool, brs: bool, data_len: int, arb_rate: int, data_rate: Optional[int]) -> float:
        if arb_rate <= 0:
            return 0.0
        if not is_fd:
            overhead = 45
            total_bits = overhead + max(0, data_len) * 8
            return float(total_bits) / float(arb_rate)
        if brs and data_rate and data_rate > 0:
            arb_overhead = 45
            crc_bits = 17 if data_len <= 16 else 21
            data_phase_overhead = 10
            data_bits = max(0, data_len) * 8 + crc_bits + data_phase_overhead
            return float(arb_overhead) / float(arb_rate) + float(data_bits) / float(data_rate)
        overhead_fd = 45
        crc_bits2 = 17 if data_len <= 16 else 21
        total_bits_fd = overhead_fd + max(0, data_len) * 8 
        return float(total_bits_fd) / float(arb_rate)

    def _run_shared_receiver(self, bus: can.BusABC, channels: List[int]):
        """
        共享接收线程：处理一个 Bus 对象上的所有通道消息。
        适用于原生支持多通道的驱动（如 Candle）。
        """
        listener = None
        notifier = None
        batch = []
        last_emit = time.perf_counter()
        
        # 本地缓存：为每个通道维护 recent_tx_buffer
        # {channel: deque}
        recent_tx_buffers = defaultdict(deque)
        
        # 注册 Listener
        try:
            listener = can.BufferedReader()
            notifier = can.Notifier(bus, [listener], timeout=0.001)
            # 记录 notifier (任意归属到一个通道即可，disconnect 时会遍历停止)
            self.rx_listeners[channels[0]] = listener
            self.rx_notifiers[channels[0]] = notifier
        except Exception:
            listener = None
            notifier = None

        while not self.stop_event.is_set() and bus:
            try:
                msg = None
                if listener:
                    try:
                        msg = listener.get_message(timeout=0.001)
                    except TypeError:
                        msg = listener.get_message()
                else:
                    msg = bus.recv(timeout=0.005)
                
                if msg:
                    # 确定消息所属通道
                    # 如果 msg.channel 是 int (Candle)，直接使用
                    # 如果是 None，默认为列表第一个
                    msg_ch = getattr(msg, 'channel', None)
                    if msg_ch is None:
                        msg_ch = channels[0]
                    
                    # 确保是 int
                    if not isinstance(msg_ch, int):
                         # 尝试解析或映射? 暂时不做复杂处理，假设驱动返回的是 int
                         try:
                             msg_ch = int(msg_ch)
                         except:
                             pass
                    
                    # 如果收到的通道不在本线程管理的列表里（可能是误收？），暂时也处理，或者过滤
                    # 但为了统计准确，最好只处理已知的
                    current_ch = msg_ch
                    if current_ch not in self.bus_map:
                         # 可能是默认 0
                         current_ch = channels[0]

                    try:
                        # --- Echo Filter Logic (Per Channel) ---
                        if self.echo_filter_enabled:
                            echo_queue = self.echo_queues[current_ch]
                            tx_buf = recent_tx_buffers[current_ch]
                            
                            # Sync from sender queue
                            try:
                                while True:
                                    tx_buf.append(echo_queue.get_nowait())
                            except queue.Empty:
                                pass

                            now = time.perf_counter()
                            cutoff = now - self.echo_filter_window
                            fp = (msg.arbitration_id, bool(msg.is_extended_id), bool(getattr(msg, 'is_fd', False)), bytes(getattr(msg, 'data', b'')))
                            drop = False
                            
                            while tx_buf and tx_buf[0][1] < cutoff - 1.0:
                                tx_buf.popleft()
                            
                            for fpt, t in reversed(tx_buf):
                                if t < cutoff:
                                    break
                                if fpt == fp:
                                    drop = True
                                    break
                            if drop:
                                continue
                    except Exception:
                        pass

                    # --- Stats Update ---
                    try:
                        dl = len(getattr(msg, 'data', b''))
                        brs = bool(getattr(msg, 'bitrate_switch', False))
                        fd = bool(getattr(msg, 'is_fd', False))
                        frame_time = self._estimate_frame_time(fd, brs, dl, self.arb_rate, self.data_rate)
                        self.bus_busy_time_accum += frame_time
                        if current_ch in self.bus_busy_time_accum_map:
                            self.bus_busy_time_accum_map[current_ch] += frame_time
                    except Exception:
                        pass
                    
                    self.stats['rx_count'] += 1
                    if current_ch in self.channel_stats:
                        self.channel_stats[current_ch]['rx_count'] += 1
                    
                    # Set UI channel name
                    try:
                        setattr(msg, 'channel', self.channel_name_map.get(current_ch, str(current_ch)))
                    except Exception:
                        pass
                    
                    # --- Dispatch ---
                    if self.push_every_message:
                        self._emit_batch([msg])
                        last_emit = time.perf_counter()
                    else:
                        batch.append(msg)
                        if len(batch) >= self.max_batch_size:
                            self._emit_batch(batch)
                            batch = []
                            last_emit = time.perf_counter()
                else:
                    now2 = time.perf_counter()
                    if batch and (now2 - last_emit) >= self.batch_emit_interval:
                        self._emit_batch(batch)
                        batch = []
                        last_emit = now2
                    time.sleep(0.001)
            except can.CanError as e:
                print(f"接收错误: {e}")
                self.stats['bus_state'] = (BusState.ERROR if BusState else "错误")
                break
        
        try:
            if notifier:
                notifier.stop()
        except Exception:
            pass
        print(f"接收线程已停止 (Channels: {channels})。")

    def _run_receiver_for_channel(self, channel: int):
        # 兼容旧代码，实际上应该不会再被调用，除非有特殊情况
        self._run_shared_receiver(self.bus_map.get(channel), [channel])

    def _emit_batch(self, batch: List[can.Message]):
        # 消息接收出口：批量/逐帧分发到上层（GUI），避免在接收线程直接触摸 GUI
        if not batch:
            return
        if self.on_message_batch_received:
            try:
                self.on_message_batch_received(batch)
            except Exception:
                pass
        elif self.on_message_received:
            for m in batch:
                try:
                    self.on_message_received(m)
                except Exception:
                    pass

    def configure_receive_push(
        self,
        push_every_message: Optional[bool] = None,
        batch_emit_interval: Optional[float] = None,
        max_batch_size: Optional[int] = None,
        echo_filter_enabled: Optional[bool] = None,
        echo_filter_window: Optional[float] = None,
    ):
        # 接收分发策略配置：逐帧/批量、批量参数、回环过滤开关与窗口
        if push_every_message is not None:
            self.push_every_message = bool(push_every_message)
        if batch_emit_interval is not None:
            try:
                self.batch_emit_interval = float(batch_emit_interval)
            except Exception:
                pass
        if max_batch_size is not None:
            try:
                self.max_batch_size = int(max_batch_size)
            except Exception:
                pass
        if echo_filter_enabled is not None:
            self.echo_filter_enabled = bool(echo_filter_enabled)
        if echo_filter_window is not None:
            try:
                self.echo_filter_window = float(echo_filter_window)
            except Exception:
                pass

    def _run_stats_updater(self):
        """后台统计线程，用于计算 FPS 和总线负载。"""
        while not self.stop_event.is_set():
            current_time = time.perf_counter()
            elapsed = current_time - self.last_stats_time
            
            if elapsed >= 1.0:
                # 0. Accumulate virtual stats from hardware periodic tasks
                for ch, info in list(self.periodic_task_info.items()):
                    added_tx = info['freq'] * elapsed
                    added_load = added_tx * info['frame_time']
                    
                    self.stats['tx_count'] += int(added_tx)
                    if ch in self.channel_stats:
                         self.channel_stats[ch]['tx_count'] += int(added_tx)
                    
                    self.bus_busy_time_accum += added_load
                    if ch in self.bus_busy_time_accum_map:
                         self.bus_busy_time_accum_map[ch] += added_load

                # 1. Update Global Stats
                rx_now = self.stats['rx_count']
                tx_now = self.stats['tx_count']

                self.stats['rx_fps'] = (rx_now - self.last_rx_count) / elapsed
                self.stats['tx_fps'] = (tx_now - self.last_tx_count) / elapsed
                load = (self.bus_busy_time_accum / elapsed) * 100 if elapsed > 0 else 0.0
                self.stats['bus_load'] = max(0.0, min(100.0, load))
                
                state = "未连接"
                if self.is_connected and self.primary_channel is not None:
                    b = self.bus_map.get(self.primary_channel)
                    if b:
                        state = b.state
                self.stats['bus_state'] = state
                
                # 2. Update Per-Channel Stats
                for ch in list(self.channel_stats.keys()):
                    c_stats = self.channel_stats[ch]
                    
                    # RX FPS
                    c_rx_now = c_stats['rx_count']
                    c_rx_last = self.last_rx_count_map.get(ch, 0)
                    c_stats['rx_fps'] = (c_rx_now - c_rx_last) / elapsed
                    self.last_rx_count_map[ch] = c_rx_now
                    
                    # TX FPS
                    c_tx_now = c_stats['tx_count']
                    c_tx_last = self.last_tx_count_map.get(ch, 0)
                    c_stats['tx_fps'] = (c_tx_now - c_tx_last) / elapsed
                    self.last_tx_count_map[ch] = c_tx_now
                    
                    # Bus Load
                    c_busy = self.bus_busy_time_accum_map.get(ch, 0.0)
                    c_load = (c_busy / elapsed) * 100 if elapsed > 0 else 0.0
                    c_stats['bus_load'] = max(0.0, min(100.0, c_load))
                    
                    # Reset accum
                    self.bus_busy_time_accum_map[ch] = 0.0
                    
                self.stats['channels'] = self.channel_stats.copy()

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
