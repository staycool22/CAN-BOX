"""
TZCANTransmitter: 基于 python-can 的通用 CAN/CAN-FD 发送/接收实现
- 继承 /home/tz4/CanMotorCtroller/CANMessageTransmitter.py 中的抽象基类 CANMessageTransmitter
- 使用 socketcan 接口初始化（参考 can_loopback_test.py）
- 提供与基类一致的方法签名，方便在各模块中替换或直接调用
"""
import sys
import time
from typing import Tuple, List, Optional

import can
try:
    from .CANMessageTransmitter import CANMessageTransmitter
except Exception:
    from CANMessageTransmitter import CANMessageTransmitter

# 与其它模块保持一致的常量（可按需扩展）
TYPE_CAN = 0
TYPE_CANFD = 1
STATUS_OK = 1

class TZCANTransmitter(CANMessageTransmitter):
    """
    使用 python-can 封装的 CAN/CAN-FD 发送接收器，实现抽象基类要求的方法。

    
    注意：
    - 在 Linux socketcan 下，波特率等物理参数需通过系统命令配置：
      例如：`sudo ip link set can0 up type can bitrate 500000`
    - 本类的 init_can_device 返回 python-can 的 Bus 句柄，作为 channel_handle 使用。
    - 支持 CAN 与 CAN-FD（is_fd）。CAN-FD 的 brs、esi 通过 Message 属性设置。
    """

    def __init__(self, channel_handle: can.BusABC):
        super().__init__(channel_handle)
        self.bus: can.BusABC = channel_handle

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
        """
        if not isinstance(self.bus, can.BusABC):
            print("❌ 无效的通道句柄，未初始化 Bus")
            return False, []

        end_ts = time.time() + (timeout if timeout is not None else 0)
        remaining = timeout

        while True:
            try:
                msg = self.bus.recv(timeout=max(0.0, remaining) if timeout is not None else None)
            except can.CanError as e:
                return (False, []) if not return_msg else (False, [], None)
            except Exception as e:
                return (False, []) if not return_msg else (False, [], None)

            if msg is None:
                return (False, []) if not return_msg else (False, [], None)

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
        """将数字通道映射为 socketcan 接口名。"""
        return f"can{int(ch)}"

    @staticmethod
    def _normalize_sample_point(sp: Optional[float]) -> Optional[float]:
        """采样点规格化为百分比（0-100）。None 保持 None；0-1 -> *100；>=1 直接用。"""
        if sp is None:
            return None
        try:
            if sp <= 1.0:
                return float(sp * 100.0)
            return float(sp)
        except Exception:
            return None

    @staticmethod
    def _build_bus_win(
        backend: str,
        index: int,
        bitrate: int,
        fd: bool = False,
        data_bitrate: Optional[int] = None,
        sample_point: Optional[float] = None,
        data_sample_point: Optional[float] = None,
    ) -> can.BusABC:
        """在 Windows 下构建总线（candle/gs_usb）。FD 仅支持 candle。"""
        # 可选依赖（按需导入）
        try:
            import candle  # type: ignore
        except Exception:
            candle = None  # noqa: F841
        try:
            import usb  # type: ignore
        except Exception:
            usb = None  # noqa: F841

        if backend == 'candle':
            if fd:
                if data_bitrate is None:
                    raise ValueError("FD 模式需要提供 data_bitrate（数据域速率）")
                sp_norm = TZCANTransmitter._normalize_sample_point(sample_point)
                dsp_norm = TZCANTransmitter._normalize_sample_point(data_sample_point)
                kwargs = dict(interface='candle', channel=index, fd=True, bitrate=bitrate, data_bitrate=data_bitrate)
                if sp_norm is not None:
                    kwargs['sample_point'] = sp_norm
                if dsp_norm is not None:
                    kwargs['data_sample_point'] = dsp_norm
                try:
                    return can.Bus(**kwargs)
                except Exception as e:
                    msg = str(e)
                    if "No suitable bit timings found" in msg:
                        raise RuntimeError(
                            f"candle 设备无法配置指定 FD 速率: arb={bitrate}, data={data_bitrate}。可尝试降低数据域至 8M/5M 或关闭 BRS。"
                        ) from e
                    raise
            # 标准 CAN 模式
            sp_norm = TZCANTransmitter._normalize_sample_point(sample_point)
            kwargs = dict(interface='candle', channel=index, fd=False, bitrate=bitrate)
            if sp_norm is not None:
                kwargs['sample_point'] = sp_norm
            return can.Bus(**kwargs)

        if backend == 'gs_usb':
            if fd:
                raise RuntimeError("gs_usb 后端不支持 CAN FD。请使用 backend='candle' 进行 FD 测试")
            return can.Bus(interface='gs_usb', channel=index, bitrate=bitrate)

        raise ValueError(f"不支持的 Windows 后端: {backend}")

    @staticmethod
    def init_can_device(
        baud_rate: int = 500000,
        dbit_baud_rate: int = 500000,
        channels: List[int] = [0],
        can_type: int = TYPE_CAN,
        canfd_standard: int = 0,
        channel_count: Optional[int] = None,
        backend: Optional[str] = None,
        **kwargs,
    ) -> Tuple[dict, Optional[can.BusABC], Optional[can.BusABC]]:
        """初始化 CAN 设备，跨平台支持。
        返回：(m_dev, ch0, ch1)，其中 ch0/ch1 为 python-can 的 Bus 句柄。

        平台与后端选择：
        - Linux/WSL：默认使用 socketcan（需系统层配置速率）
        - Windows：默认使用 candle；可指定 backend='candle' 或 'gs_usb'

        channels 语义：
        - Linux：元素映射为 'canX' 接口，例如 0 -> 'can0'
        - Windows：元素映射为设备索引，例如 0 -> 第一个设备
        """
        buses = {}
        ch0 = None
        ch1 = None
        fd = kwargs.get('fd', False)
        sp = kwargs.get('sp', None)
        dsp = kwargs.get('dsp', None)

        # 自动选择后端
        if backend is None:
            backend = 'socketcan' if sys.platform.startswith('linux') else 'candle'

        if backend == 'socketcan':
            for ch in channels:
                ch_name = TZCANTransmitter._resolve_channel_name(ch)
                try:
                    if fd:
                        bus = can.interface.Bus(channel=ch_name, interface='socketcan', fd=True)
                    else:
                        bus = can.interface.Bus(channel=ch_name, interface='socketcan')
                    buses[ch] = bus
                    if ch == 0:
                        ch0 = bus
                    elif ch == 1:
                        ch1 = bus
                    print(f"✅ 已打开接口 {ch_name}（socketcan）")
                except OSError as e:
                    print(f"❌ 打开接口 {ch_name} 失败: {e}")
                    print(f"请先在系统中启用并配置速率，例如: sudo ip link set {ch_name} up type can bitrate {baud_rate}")
                except Exception as e:
                    print(f"❌ 初始化接口 {ch_name} 异常: {e}")
        elif backend in ('candle', 'gs_usb'):
            for idx in channels:
                try:
                    bus = TZCANTransmitter._build_bus_win(
                        backend=backend,
                        index=idx,
                        bitrate=baud_rate,
                        fd=fd,
                        data_bitrate=(dbit_baud_rate if fd else None),
                        sample_point=sp,
                        data_sample_point=dsp,
                    )
                    buses[idx] = bus
                    if idx == 0:
                        ch0 = bus
                    elif idx == 1:
                        ch1 = bus
                    print(f"✅ 已打开设备 index={idx}（{backend}）")
                except Exception as e:
                    print(f"❌ 打开 {backend} 设备 index={idx} 失败: {e}")
        else:
            raise ValueError(f"未知后端: {backend}")

        m_dev = {
            "backend": backend,
            "buses": buses,
            "config": {
                "baud_rate": baud_rate,
                "dbit_baud_rate": dbit_baud_rate,
                "can_type": can_type,
                "canfd_standard": canfd_standard,
                "channels": channels,
                "fd": fd,
                "sp": sp,
                "dsp": dsp,
            },
        }
        return m_dev, ch0, ch1

    @staticmethod
    def close_can_device(
        m_dev: dict,
        channel_handle0: Optional[can.BusABC] = None,
        channel_handle1: Optional[can.BusABC] = None,
    ) -> int:
        """关闭 CAN 设备与通道。
        - 对传入的 Bus 逐一 shutdown
        返回：STATUS_OK（1）以与既有代码保持一致
        """
        try:
            # 先关闭显式传入的句柄
            for bus in [channel_handle0, channel_handle1]:
                if isinstance(bus, can.BusABC):
                    try:
                        bus.shutdown()
                    except Exception:
                        pass

            # 再关闭 m_dev 集合中的句柄
            if isinstance(m_dev, dict) and "buses" in m_dev:
                for _, bus in m_dev["buses"].items():
                    try:
                        if isinstance(bus, can.BusABC):
                            bus.shutdown()
                    except Exception:
                        pass
            print("✅ CAN 接口已关闭")
            return STATUS_OK
        except Exception as e:
            print(f"⚠️ 关闭接口过程中出现异常: {e}")
            return STATUS_OK