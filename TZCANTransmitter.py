"""
TZCANTransmitter: 基于 python-can 的通用 CAN/CAN-FD 发送/接收实现
- 继承 CANMessageTransmitter 抽象基类
- 支持多通道 Candle 设备（共享句柄模式）
- 支持 socketcan, gs_usb 等其他后端
"""
import sys
import time
import queue
import threading
from typing import Tuple, List, Optional, Dict, Any
from collections import defaultdict

import can
try:
    from .CANMessageTransmitter import CANMessageTransmitter
except Exception:
    try:
        from CANMessageTransmitter import CANMessageTransmitter
    except ImportError:
         # Fallback definition if base class is missing during dev/test
        class CANMessageTransmitter:
            def __init__(self, channel_handle): pass

class BasicConfig:
    TYPE_CAN = 0
    TYPE_CANFD = 1
    STATUS_OK = 1

# 兼容旧代码的全局变量别名
TYPE_CAN = BasicConfig.TYPE_CAN
TYPE_CANFD = BasicConfig.TYPE_CANFD
STATUS_OK = BasicConfig.STATUS_OK

class TZCANTransmitter(CANMessageTransmitter):
    """
    使用 python-can 封装的 CAN/CAN-FD 发送接收器，实现抽象基类要求的方法。

    
    注意：
    - 在 Linux socketcan 下，波特率等物理参数需通过系统命令配置：
      例如：`sudo ip link set can0 up type can bitrate 500000`
    - 本类的 init_can_device 返回 python-can 的 Bus 句柄，作为 channel_handle 使用。
    - 支持 CAN 与 CAN-FD（is_fd）。CAN-FD 的 brs、esi 通过 Message 属性设置。
    """

    def __init__(self, channel_handle: can.BusABC, channel_id: Optional[int] = None):
        super().__init__(channel_handle)
        self.bus: can.BusABC = channel_handle
        self.channel_id = channel_id
    
    # 将配置类绑定到主类，方便外部通过类直接访问，无需反射导入模块
    Config = BasicConfig

    def _send_can_data(
        self,
        send_id: int,
        data_list: List[int],
        is_ext_frame: bool = False,
        canfd_mode: bool = False,
        brs: int = 0,
        esi: int = 0,
    ) -> bool:
        """发送 CAN/CAN-FD 数据。
        - send_id: 报文 ID
        - data_list: 数据字节列表（CAN: ≤8, CAN-FD: ≤64）
        - is_ext_frame: 是否扩展帧（29位ID）
        - canfd_mode: 是否使用 CAN-FD
        - brs: Bit Rate Switch（CAN-FD 专用，1=启用）
        - esi: Error State Indicator（CAN-FD 专用，1=错误状态）
        返回：True/False 是否发送成功
        """
        try:
            # 校验 bus 句柄
            if not isinstance(self.bus, can.BusABC):
                print("❌ 无效的通道句柄，未初始化 Bus")
                return False

            # 长度校验
            if canfd_mode:
                if len(data_list) > 64:
                    print(f"❌ CAN-FD 数据长度超限（{len(data_list)} > 64）")
                    return False
            else:
                if len(data_list) > 8:
                    print(f"❌ CAN 数据长度超限（{len(data_list)} > 8）")
                    return False

            msg = can.Message(
                arbitration_id=send_id,
                data=bytes(bytearray(data_list)),
                is_extended_id=bool(is_ext_frame),
                is_fd=bool(canfd_mode),
                bitrate_switch=bool(brs),
                error_state_indicator=bool(esi),
                channel=self.channel_id  # 显式指定通道（适配共享 Bus）
            )
            self.bus.send(msg)
            return True
        except can.CanError as e:
            print(f"❌ 发送 CAN 报文失败: {e}")
            return False
        except Exception as e:
            print(f"❌ 发送过程异常: {e}")
            return False

    def _receive_can_data(
        self,
        target_id: Optional[int] = None,
        timeout: float = 5,
        is_ext_frame: Optional[bool] = None,
        canfd_mode: bool = False,
        stop_on_error: bool = False,
        return_msg: bool = False,
    ) -> Tuple[bool, List[int]]:
        """接收 CAN/CAN-FD 数据。
        - target_id: 期望的报文 ID（None 表示接受任何报文）
        - timeout: 超时时间（秒）
        - is_ext_frame: 期望扩展帧标志（None 不强制）
        - canfd_mode: 是否期望接收 CAN-FD 报文（用于过滤）
        返回：(是否成功, 数据列表)
        
        注意：在共享 Bus 模式下（多通道 Candle），此方法可能会从 Bus 中“窃取”其他通道的消息。
        建议在多通道场景下使用 can_communicator 的统一接收机制，而不是直接调用此方法。
        """
        if not isinstance(self.bus, can.BusABC):
            print("❌ 无效的通道句柄，未初始化 Bus")
            return False, []

        end_ts = time.time() + (timeout if timeout is not None else 0)
        remaining = timeout

        while True:
            try:
                msg = self.bus.recv(timeout=max(0.0, remaining) if timeout is not None else None)
            except Exception:
                return (False, []) if not return_msg else (False, [], None)

            if msg is None:
                return (False, []) if not return_msg else (False, [], None)

            # 通道过滤：如果指定了 channel_id，则忽略其他通道的消息
            if self.channel_id is not None:
                msg_ch = getattr(msg, 'channel', None)
                # 尝试匹配 channel (兼容 int 和其他类型)
                if msg_ch is not None and msg_ch != self.channel_id:
                     # 如果不匹配，继续接收（注意：这会丢弃该消息，导致其他消费者收不到！）
                     # 这是一个简单的单实例接收实现，不适合多实例并发竞争
                     if timeout is None:
                         continue
                     remaining = end_ts - time.time()
                     if remaining <= 0:
                         return (False, []) if not return_msg else (False, [], None)
                     continue

            if stop_on_error and bool(getattr(msg, "is_error_frame", False)):
                raise RuntimeError("接收到错误帧")

            if target_id is not None and msg.arbitration_id != target_id:
                pass
            else:
                if is_ext_frame is not None and bool(msg.is_extended_id) != bool(is_ext_frame):
                    pass
                else:
                    if canfd_mode and not msg.is_fd:
                        pass
                    else:
                        return (True, list(msg.data)) if not return_msg else (True, list(msg.data), msg)

            if timeout is None:
                continue
            remaining = end_ts - time.time()
            if remaining <= 0:
                return (False, []) if not return_msg else (False, [], None)

    @staticmethod
    def _resolve_channel_name(ch: int) -> str:
        return f"can{int(ch)}"

    @staticmethod
    def _normalize_sample_point(sp: Optional[float]) -> Optional[float]:
        if sp is None:
            return None
        try:
            if sp <= 1.0:
                return float(sp * 100.0)
            return float(sp)
        except Exception:
            return None

    _cached_candle_mapping = None

    @staticmethod
    def _detect_candle_mapping(force_refresh=False):
        """检测 candle 设备及其对应的通道列表"""
        if not force_refresh and TZCANTransmitter._cached_candle_mapping is not None:
            return TZCANTransmitter._cached_candle_mapping

        mapping = defaultdict(list)
        try:
            # 尝试检测
            configs = can.detect_available_configs("candle")
            for cfg in configs:
                ch = getattr(cfg, "channel", None)
                if not ch or ":" not in ch:
                    continue
                sn, idx = ch.split(":")
                mapping[sn].append(int(idx))
            for sn in mapping:
                mapping[sn] = sorted(mapping[sn])
        except Exception:
            pass
        
        # Fallback: 直接使用 candle_api (如果上述检测为空或失败)
        if not mapping:
            try:
                import candle_api as api
                for dev in api.list_device():
                    sn = dev.serial_number
                    count = 0
                    try: count = len(dev)
                    except: count = 1 
                    
                    if count > 0:
                        mapping[sn] = list(range(count))
            except Exception:
                pass

        TZCANTransmitter._cached_candle_mapping = mapping
        return mapping

    @staticmethod
    def get_all_channels(backend: str = 'candle') -> List[Tuple[int, str, int]]:
        """
        返回所有可用通道的扁平列表。
        Returns: [(flat_index, serial_number, channel_index), ...]
        """
        if backend == 'candle':
            # 刷新列表，确保获取最新设备
            mapping = TZCANTransmitter._detect_candle_mapping(force_refresh=True)
            all_interfaces = []
            flat_idx = 0
            for sn in sorted(mapping.keys()):
                for ch_idx in sorted(mapping[sn]):
                    all_interfaces.append((flat_idx, sn, ch_idx))
                    flat_idx += 1
            return all_interfaces
        else:
            # 对于非 candle 后端，暂时返回简单的索引列表
            return [(i, "Unknown", i) for i in range(4)]

    @staticmethod
    def init_can_device(
        baud_rate: int = 500000,
        dbit_baud_rate: int = 500000,
        channels: List[int] = [0],
        can_type: int = BasicConfig.TYPE_CAN,
        canfd_standard: int = 0,
        channel_count: Optional[int] = None,
        backend: Optional[str] = None,
        **kwargs,
    ) -> Tuple[dict, Optional[can.BusABC], Optional[can.BusABC]]:
        
        buses = {}
        ch0 = None
        ch1 = None
        fd = kwargs.get('fd', False)
        sp = kwargs.get('sp', None)
        dsp = kwargs.get('dsp', None)

        if backend is None:
            backend = 'socketcan' if sys.platform.startswith('linux') else 'candle'

        if backend == 'socketcan':
            # Linux socketcan 逻辑
            for ch in channels:
                ch_name = TZCANTransmitter._resolve_channel_name(ch)
                try:
                    if fd:
                        bus = can.interface.Bus(channel=ch_name, interface='socketcan', fd=True)
                    else:
                        bus = can.interface.Bus(channel=ch_name, interface='socketcan')
                    buses[ch] = bus
                    if ch == 0: ch0 = bus
                    elif ch == 1: ch1 = bus
                    print(f"✅ 已打开接口 {ch_name}（socketcan）")
                except OSError as e:
                    print(f"❌ 打开接口 {ch_name} 失败: {e}")
                except Exception as e:
                    print(f"❌ 初始化接口 {ch_name} 异常: {e}")
        
        elif backend == 'candle':
            # Candle 多通道逻辑 (原生支持)
            mapping = TZCANTransmitter._detect_candle_mapping(force_refresh=False)
            
            # 1. 扁平化所有可用接口： [(SN, ch_idx), (SN, ch_idx), ...]
            all_interfaces = []
            for sn in sorted(mapping.keys()):
                for ch_idx in sorted(mapping[sn]):
                    all_interfaces.append((sn, ch_idx))
            
            # 2. 解析用户请求的索引
            requested_interfaces = {} # user_idx -> (sn, ch_idx)
            sns_to_init = set()
            
            for req_idx in channels:
                if req_idx < len(all_interfaces):
                    sn, ch_idx = all_interfaces[req_idx]
                    requested_interfaces[req_idx] = (sn, ch_idx)
                    sns_to_init.add(sn)
                else:
                    print(f"⚠️ 索引 {req_idx} 超出可用通道范围 (总数: {len(all_interfaces)})")
            
            # 3. 按 SN 分组初始化共享 Bus
            sn_to_bus = {} # sn -> shared_bus
            
            for sn in sns_to_init:
                dev_channels = mapping[sn] # 该设备的所有物理通道
                
                channel_configs = kwargs.get('channel_configs') or {}
                
                bitrate_list = []
                data_bitrate_list = []
                sp_list = []
                dsp_list = []
                
                # 默认值
                def_baud = baud_rate
                def_dbaud = dbit_baud_rate
                def_sp = TZCANTransmitter._normalize_sample_point(sp)
                def_dsp = TZCANTransmitter._normalize_sample_point(dsp)
                
                for dev_ch_idx in dev_channels:
                    flat_idx = -1
                    try:
                        flat_idx = all_interfaces.index((sn, dev_ch_idx))
                    except ValueError:
                        pass
                    
                    cfg = channel_configs.get(flat_idx, {}) if flat_idx != -1 else {}
                    
                    b = cfg.get('arb_rate', def_baud)
                    bitrate_list.append(b)
                    
                    db = cfg.get('data_rate', def_dbaud)
                    data_bitrate_list.append(db)
                    
                    s = cfg.get('sp', None)
                    s_norm = TZCANTransmitter._normalize_sample_point(s) if s is not None else def_sp
                    if s_norm is None: s_norm = 87.5
                    sp_list.append(s_norm)
                    
                    ds = cfg.get('dsp', None)
                    ds_norm = TZCANTransmitter._normalize_sample_point(ds) if ds is not None else def_dsp
                    if ds_norm is None: ds_norm = 87.5
                    dsp_list.append(ds_norm)

                try:
                    base_bitrate = bitrate_list[0] if bitrate_list else baud_rate
                    base_data_bitrate = data_bitrate_list[0] if (fd and data_bitrate_list) else dbit_baud_rate
                    base_sp = sp_list[0] if sp_list else def_sp
                    base_dsp = dsp_list[0] if (fd and dsp_list) else def_dsp
                    
                    dev_channel_configs = {}
                    for i, phy_ch in enumerate(dev_channels):
                         dev_channel_configs[phy_ch] = {
                             "bitrate": bitrate_list[i] if i < len(bitrate_list) else base_bitrate,
                             "data_bitrate": data_bitrate_list[i] if (fd and i < len(data_bitrate_list)) else base_data_bitrate,
                             "sample_point": sp_list[i] if i < len(sp_list) else base_sp,
                             "data_sample_point": dsp_list[i] if (fd and i < len(dsp_list)) else base_dsp,
                             "fd": fd,
                             "listen_only": False,
                             "termination": None
                         }

                    kwargs_bus = dict(
                        interface='candle',
                        channel=dev_channels, # 传入 int 列表, 启用多通道模式
                        serial_number=sn,     # 显式指定序列号
                        fd=fd,
                        bitrate=base_bitrate,      
                        data_bitrate=(base_data_bitrate if fd else None),
                        sample_point=base_sp,      
                        data_sample_point=(base_dsp if fd else None),
                        channel_configs=dev_channel_configs 
                    )
                    
                    # 初始化共享 Bus (Native CandleBus)
                    shared_bus = can.Bus(**kwargs_bus)
                    sn_to_bus[sn] = shared_bus
                    
                    print(f"✅ 已初始化设备 {sn} (包含通道 {dev_channels})")
                    print(f"   配置: Arb={bitrate_list}, Data={data_bitrate_list}")
                    
                except Exception as e:
                    print(f"❌ 初始化设备 {sn} 失败: {e}")
            
            # 4. 将共享 Bus 映射回用户请求的 buses 字典
            for req_idx, (sn, ch_idx) in requested_interfaces.items():
                if sn in sn_to_bus:
                    # 直接返回共享 Bus 对象
                    # 注意：上层需要在发送时指定 msg.channel = ch_idx
                    # 接收时 msg.channel 会是 ch_idx
                    shared_bus = sn_to_bus[sn]
                    buses[req_idx] = shared_bus
                    if req_idx == 0: ch0 = shared_bus
                    elif req_idx == 1: ch1 = shared_bus
                    print(f"  -> 映射索引 {req_idx} 到 {sn} (共享Bus)")

        elif backend == 'gs_usb':
             for idx in channels:
                try:
                    if fd:
                        raise RuntimeError("gs_usb 不支持 FD")
                    bus = can.Bus(interface='gs_usb', channel=idx, bitrate=baud_rate)
                    buses[idx] = bus
                    if idx == 0: ch0 = bus
                    elif idx == 1: ch1 = bus
                    print(f"✅ 已打开设备 index={idx} (gs_usb)")
                except Exception as e:
                    print(f"❌ 打开 gs_usb 设备 {idx} 失败: {e}")

        m_dev = {
            "backend": backend,
            "buses": buses,
            # "internal_data": internal_data, # 已废弃
            "config": {
                "baud_rate": baud_rate,
                "dbit_baud_rate": dbit_baud_rate,
                "can_type": can_type,
                "canfd_standard": canfd_standard,
                "channels": channels,
                "fd": fd,
            },
        }
        return m_dev, ch0, ch1

    @staticmethod
    def close_can_device(
        m_dev: dict,
        channel_handle0: Optional[can.BusABC] = None,
        channel_handle1: Optional[can.BusABC] = None,
    ) -> int:
        try:
            # 停止 Notifiers 和 关闭 Shared Buses
            # 注意：新版 init_can_device 不再使用 internal_data 存储 notifiers
            # 而是直接将 Bus 对象返回给 buses。
            # 但为了兼容可能存在的旧对象（虽然这里我们已经改了），还是保留一些检查。
            
            if "internal_data" in m_dev:
                for notifier in m_dev["internal_data"].get("notifiers", []):
                    try:
                        notifier.stop()
                    except Exception:
                        pass
                for bus in m_dev["internal_data"].get("shared_buses", []):
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
            
            # 关闭 buses 字典中的所有 Bus
            if isinstance(m_dev, dict) and "buses" in m_dev:
                # 收集唯一的 bus 对象，避免重复关闭
                unique_buses = set(m_dev["buses"].values())
                for bus in unique_buses:
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
                            
            print("✅ CAN 接口已关闭")
            return BasicConfig.STATUS_OK
        except Exception as e:
            print(f"⚠️ 关闭接口过程中出现异常: {e}")
            return BasicConfig.STATUS_OK
