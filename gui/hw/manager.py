"""硬件层管理模块
负责 CAN 设备的物理连接、配置和断开，屏蔽 Linux/Windows 差异。
将操作系统检测、后端选择、Bus 初始化等硬件相关逻辑集中于此。
"""
import platform
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any

try:
    import can
except ImportError:
    can = None


@dataclass
class HardwareConfig:
    """CAN 硬件连接配置"""
    backend: str                            # 'socketcan' | 'candle' | 'gs_usb'
    channels: List[int]                     # 请求打开的通道索引列表
    is_fd: bool = False
    arb_rate: int = 500000
    data_rate: int = 500000
    arb_sample_point: Optional[float] = None
    data_sample_point: Optional[float] = None
    channel_configs: Dict[int, dict] = field(default_factory=dict)  # 每通道独立参数


@dataclass
class HardwareContext:
    """已建立的硬件连接上下文"""
    config: HardwareConfig
    bus_map: Dict[int, Any]           # channel_idx → can.BusABC
    device_handle: Optional[dict]     # m_dev，关闭时使用（Windows 封装库路径）
    channel_name_map: Dict[int, str]  # channel_idx → 逻辑名（"can0" 等）
    os_type: str                      # 'linux' | 'windows'


class HardwareManager:
    """
    硬件层：负责物理 CAN 设备的连接、配置与释放。
    屏蔽 Linux/Windows、socketcan/candle/gs_usb 后端差异。
    """

    def __init__(self):
        self.os_type = 'linux' if platform.system() == 'Linux' else 'windows'
        self._transmitter_class = None

    @property
    def transmitter_class(self):
        if self._transmitter_class is None:
            from tzcan import CANMessageTransmitter as _CMT
            self._transmitter_class = _CMT.choose_can_device("TZUSB2CAN")
        return self._transmitter_class

    def connect(self, config: HardwareConfig) -> HardwareContext:
        """根据配置建立 CAN 连接，返回硬件上下文。"""
        if self.os_type == 'linux':
            bus_map, channel_name_map = self._connect_linux(config)
            device_handle = None
        else:
            bus_map, channel_name_map, device_handle = self._connect_windows(config)

        if not bus_map:
            raise RuntimeError("未成功打开任何 CAN 通道")

        return HardwareContext(
            config=config,
            bus_map=bus_map,
            device_handle=device_handle,
            channel_name_map=channel_name_map,
            os_type=self.os_type,
        )

    def _connect_linux(self, config: HardwareConfig):
        """Linux SocketCAN 连接逻辑"""
        if config.backend != 'socketcan':
            raise ValueError("Linux 平台目前仅支持 'socketcan' 后端")

        from tzcan.socketcan_tool import configure_single_interface

        bus_map: Dict[int, Any] = {}
        channel_name_map: Dict[int, str] = {}
        try:
            for ch_idx in config.channels:
                ifc = f"can{ch_idx}"

                # 获取该通道的特定配置（可在 channel_configs 中覆盖全局值）
                c_arb = config.arb_rate
                c_data = config.data_rate
                c_sp = config.arb_sample_point
                c_dsp = config.data_sample_point

                if ch_idx in config.channel_configs:
                    cfg = config.channel_configs[ch_idx]
                    c_arb = cfg.get('arb_rate', c_arb)
                    c_data = cfg.get('data_rate', c_data)
                    c_sp = cfg.get('sp', c_sp)
                    c_dsp = cfg.get('dsp', c_dsp)

                # 配置物理接口（ip link set ...）
                configure_single_interface(
                    iface=ifc,
                    bitrate=c_arb,
                    dbitrate=c_data if config.is_fd else None,
                    fd=config.is_fd,
                    sample_point=c_sp,
                    data_sample_point=c_dsp,
                )

                # 打开 python-can Bus（关闭回环避免收到自身帧）
                kwargs = {
                    'bustype': 'socketcan',
                    'channel': ifc,
                    'bitrate': c_arb,
                    'receive_own_messages': False,
                }
                if config.is_fd:
                    kwargs.update({'fd': True, 'data_bitrate': c_data})

                _bus = can.interface.Bus(**kwargs)
                bus_map[ch_idx] = _bus
                channel_name_map[ch_idx] = ifc
                print(f"✅ 原生 python-can 打开 {ifc} 成功 (Arb: {c_arb}, Data: {c_data})")

        except Exception:
            # 清理已打开的 bus，避免资源泄漏
            for bus in bus_map.values():
                try:
                    bus.shutdown()
                except Exception:
                    pass
            raise

        return bus_map, channel_name_map

    def _connect_windows(self, config: HardwareConfig):
        """Windows Candle/GS_USB 连接逻辑"""
        if config.backend not in ['candle', 'gs_usb']:
            raise ValueError("Windows 平台仅支持 'candle' 或 'gs_usb' 后端")
        if config.is_fd and config.backend != 'candle':
            raise ValueError("Windows 平台 FD 模式仅支持 'candle' 后端")
        if not config.channels:
            raise ValueError("channels 不能为空")

        bus_map: Dict[int, Any] = {}
        channel_name_map: Dict[int, str] = {}
        device_handle = None

        # 优先尝试封装库
        try:
            device_handle, _, _ = self.transmitter_class.init_can_device(
                baud_rate=config.arb_rate,
                dbit_baud_rate=config.data_rate,
                channels=config.channels,
                backend=config.backend,
                fd=config.is_fd,
                sp=config.arb_sample_point,
                dsp=config.data_sample_point,
                channel_configs=config.channel_configs,
            )
            opened_buses = device_handle.get('buses', {})
            if opened_buses:
                bus_map = opened_buses
                channel_name_map = {ch: str(ch) for ch in bus_map}
                print(f"✅ 封装库打开通道 {list(bus_map.keys())} 成功")
            else:
                raise RuntimeError("封装库未能返回任何 bus 对象")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"INFO: 封装库打开失败: {e}，回退到原生 python-can 接口")
            if device_handle:
                try:
                    self.transmitter_class.close_can_device(device_handle)
                except Exception:
                    pass
            device_handle = None

        # 封装库失败后回退到 python-can 原生
        if not bus_map:
            print(f"INFO: 正在尝试使用 python-can 原生接口逐个打开通道: {config.channels}")
            try:
                for ch in config.channels:
                    kwargs: dict = {'bustype': config.backend, 'channel': ch, 'bitrate': config.arb_rate}
                    if config.is_fd:
                        kwargs.update({'fd': True, 'data_bitrate': config.data_rate})
                    _bus = can.interface.Bus(**kwargs)
                    bus_map[ch] = _bus
                    channel_name_map[ch] = str(ch)
                    print(f"✅ python-can 打开 {config.backend} 通道 {ch} 成功")
            except Exception:
                for bus in bus_map.values():
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
                raise

        return bus_map, channel_name_map, device_handle

    def disconnect(self, ctx: HardwareContext):
        """关闭硬件连接"""
        # 关闭 bus 对象（去重避免重复关闭）
        seen = set()
        for ch, bus in ctx.bus_map.items():
            if id(bus) not in seen:
                seen.add(id(bus))
                try:
                    bus.shutdown()
                    print(f"  - ✅ 通道 {ch} 的 bus 已关闭")
                except Exception as e:
                    print(f"  - ❌ 关闭通道 {ch} 的 bus 时出错: {e}")

        # 关闭设备句柄（Windows 封装库）
        if ctx.device_handle:
            print("INFO: 正在关闭主设备句柄...")
            try:
                self.transmitter_class.close_can_device(ctx.device_handle)
                print("  - ✅ 主设备句柄已关闭")
            except Exception as e:
                print(f"  - ❌ 关闭主设备句柄时出错: {e}")

    def list_devices(self, backend: str) -> List[str]:
        """列出可用设备通道，返回格式化字符串列表"""
        try:
            channels = self.transmitter_class.get_all_channels(backend)
            result = []
            for flat_idx, sn, ch_idx in channels:
                if backend == 'candle':
                    result.append(f"{flat_idx} (SN:{sn}:{ch_idx})")
                else:
                    result.append(f"{flat_idx}")
            return result
        except Exception:
            return [str(i) for i in range(8)]
