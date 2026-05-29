# 说明：CAN 报文信号编解码引擎（纯 Python，无 Qt 依赖）
# 用途：让用户按协议把报文字节拆解为命名信号（工程量），供绘图窗口解码与绘制。
#       定义可保存为 JSON 复用分享。位域语义见下方 decode/encode 注释。
import json
import struct
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# 数据模型
# ---------------------------------------------------------------------------

@dataclass
class SignalDef:
    """单个信号的定义。

    位域语义（start_bit 在字节内按 LSB=0 计数）：
    - 小端 (little / Intel)：信号最低位在 (start_byte, start_bit)，向高位方向延伸。
      字节对齐时等价于 int.from_bytes(data[start_byte:start_byte+N], 'little')。
    - 大端 (big / Motorola)：从 start_byte 起按大端读取 N 个字节再右移 start_bit、
      取 bit_length 位。字节对齐时等价于 int.from_bytes(data[start_byte:start_byte+N], 'big')，
      与 tzcan/protocols/vesc.py 的 buffer_get_int*（大端、有符号）一致。
    工程量 = raw * scale + offset。
    """
    name: str
    start_byte: int = 0
    start_bit: int = 0          # 0~7，字节内 LSB=0
    bit_length: int = 8
    byte_order: str = "big"     # "big"(Motorola) 或 "little"(Intel)
    signed: bool = False
    is_float: bool = False      # True 时按 IEEE-754 浮点解释（位长 32=单精度/64=双精度）
    scale: float = 1.0
    offset: float = 0.0
    unit: str = ""

    def validate(self) -> None:
        if not self.name:
            raise ValueError("信号名不能为空")
        if self.start_byte < 0:
            raise ValueError("起始字节必须 >= 0")
        if not (0 <= self.start_bit <= 7):
            raise ValueError("起始位必须在 0~7 之间")
        if self.bit_length <= 0:
            raise ValueError("位长度必须 > 0")
        if self.byte_order not in ("big", "little"):
            raise ValueError("字节序必须是 'big' 或 'little'")
        if self.scale == 0:
            raise ValueError("缩放系数不能为 0")
        if self.is_float:
            if self.bit_length not in (32, 64):
                raise ValueError("IEEE-754 浮点信号的位长度必须是 32（单精度）或 64（双精度）")
            if self.start_bit != 0:
                raise ValueError("浮点信号必须字节对齐（起始位为 0）")


@dataclass
class MessageDef:
    """一条报文的定义：仲裁 ID + 若干信号。"""
    name: str
    can_id: int
    is_extended: bool = False
    dlc: int = 8
    signals: List[SignalDef] = field(default_factory=list)

    def key(self) -> Tuple[int, bool]:
        return (int(self.can_id), bool(self.is_extended))


class CodecDatabase:
    """报文/信号定义的集合，按 (can_id, is_extended) 建索引用于解码。"""

    def __init__(self, messages: Optional[List[MessageDef]] = None):
        self.messages: List[MessageDef] = list(messages or [])
        self._index: Dict[Tuple[int, bool], MessageDef] = {}
        self.reindex()

    def reindex(self) -> None:
        self._index = {m.key(): m for m in self.messages}

    def add_message(self, msg: MessageDef) -> None:
        self.messages.append(msg)
        self._index[msg.key()] = msg

    def remove_message(self, msg: MessageDef) -> None:
        if msg in self.messages:
            self.messages.remove(msg)
        self.reindex()

    def find(self, can_id: int, is_extended: bool) -> Optional[MessageDef]:
        return self._index.get((int(can_id), bool(is_extended)))

    def all_signal_names(self) -> List[str]:
        """返回 "报文名.信号名" 形式的全部信号标识，供绘图选择使用。"""
        names: List[str] = []
        for m in self.messages:
            for s in m.signals:
                names.append(f"{m.name}.{s.name}")
        return names

    # --- 解码 ---

    def decode_frame(self, can_id: int, is_extended: bool, data) -> Dict[str, float]:
        """解码一帧，返回 {"报文名.信号名": 工程量}。无匹配报文时返回空字典。"""
        msg = self.find(can_id, is_extended)
        if msg is None:
            return {}
        out: Dict[str, float] = {}
        for sig in msg.signals:
            try:
                out[f"{msg.name}.{sig.name}"] = decode_signal(sig, data)
            except Exception:
                # 单个信号解码失败（如数据长度不足）不影响其余信号
                pass
        return out

    # --- 编码（decode 的逆，主要用于自测/校验位域逻辑；窗口 UI 不暴露） ---

    def encode_message(self, message_name: str, values: Dict[str, float]) -> bytes:
        """按 message_name 的定义，把 {信号名: 工程量} 打包成 dlc 字节。"""
        msg = next((m for m in self.messages if m.name == message_name), None)
        if msg is None:
            raise KeyError(f"未找到报文: {message_name}")
        buf = bytearray(msg.dlc)
        for sig in msg.signals:
            if sig.name in values:
                encode_signal(sig, values[sig.name], buf)
        return bytes(buf)

    # --- JSON 持久化 ---

    def to_dict(self) -> dict:
        return {
            "version": 1,
            "messages": [
                {
                    "name": m.name,
                    "can_id": int(m.can_id),
                    "is_extended": bool(m.is_extended),
                    "dlc": int(m.dlc),
                    "signals": [asdict(s) for s in m.signals],
                }
                for m in self.messages
            ],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CodecDatabase":
        msgs: List[MessageDef] = []
        for md in d.get("messages", []):
            sigs = [SignalDef(**sd) for sd in md.get("signals", [])]
            msgs.append(MessageDef(
                name=md["name"],
                can_id=int(md["can_id"]),
                is_extended=bool(md.get("is_extended", False)),
                dlc=int(md.get("dlc", 8)),
                signals=sigs,
            ))
        return cls(msgs)

    def save_json(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)

    @classmethod
    def load_json(cls, path: str) -> "CodecDatabase":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))


# ---------------------------------------------------------------------------
# 位域抽取 / 写入
# ---------------------------------------------------------------------------

def _extract_raw(sig: SignalDef, data) -> int:
    """从 data 抽取无符号原始整数（不含缩放/偏移/符号扩展）。"""
    b = bytes(data)
    length = sig.bit_length
    if sig.byte_order == "little":
        # 小端：(start_byte, start_bit) 为最低位，向高位读取
        raw = 0
        base = sig.start_byte * 8 + sig.start_bit
        for i in range(length):
            p = base + i
            byte_idx = p >> 3
            if byte_idx >= len(b):
                break
            bit = (b[byte_idx] >> (p & 7)) & 1
            raw |= bit << i
        return raw
    # 大端：从 start_byte 起按大端组合若干字节，再右移 start_bit、取 bit_length 位
    span = (sig.start_bit + length + 7) // 8
    raw_span = 0
    for i in range(span):
        byte_idx = sig.start_byte + i
        val = b[byte_idx] if byte_idx < len(b) else 0
        raw_span = (raw_span << 8) | val
    return (raw_span >> sig.start_bit) & ((1 << length) - 1)


def _float_fmt(sig: SignalDef) -> str:
    return (">" if sig.byte_order == "big" else "<") + ("f" if sig.bit_length == 32 else "d")


def decode_signal(sig: SignalDef, data) -> float:
    """解码单个信号为工程量。"""
    raw = _extract_raw(sig, data)
    if sig.is_float:
        nbytes = sig.bit_length // 8
        value = struct.unpack(_float_fmt(sig), raw.to_bytes(nbytes, sig.byte_order))[0]
        return value * sig.scale + sig.offset
    if sig.signed and (raw >> (sig.bit_length - 1)) & 1:
        raw -= (1 << sig.bit_length)
    return raw * sig.scale + sig.offset


def encode_signal(sig: SignalDef, value: float, buf: bytearray) -> None:
    """把工程量写回 buf 的对应位域（encode/round-trip 用）。"""
    mask = (1 << sig.bit_length) - 1
    if sig.is_float:
        phys = (float(value) - sig.offset) / sig.scale
        nbytes = sig.bit_length // 8
        raw = int.from_bytes(struct.pack(_float_fmt(sig), phys), sig.byte_order) & mask
    else:
        raw = int(round((float(value) - sig.offset) / sig.scale))
        if sig.signed:
            lo, hi = -(1 << (sig.bit_length - 1)), (1 << (sig.bit_length - 1)) - 1
            raw = max(lo, min(hi, raw))
            raw &= mask  # 转两's complement 位模式
        else:
            raw = max(0, min(mask, raw))
    if sig.byte_order == "little":
        base = sig.start_byte * 8 + sig.start_bit
        for i in range(sig.bit_length):
            p = base + i
            byte_idx = p >> 3
            if byte_idx >= len(buf):
                break
            bit = (raw >> i) & 1
            if bit:
                buf[byte_idx] |= (1 << (p & 7))
            else:
                buf[byte_idx] &= ~(1 << (p & 7)) & 0xFF
        return
    span = (sig.start_bit + sig.bit_length + 7) // 8
    field_shifted = (raw & mask) << sig.start_bit
    for i in range(span):
        byte_idx = sig.start_byte + (span - 1 - i)
        if byte_idx >= len(buf):
            continue
        buf[byte_idx] |= (field_shifted >> (8 * i)) & 0xFF


# ---------------------------------------------------------------------------
# 自测：解码 + round-trip
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # 参照 VESC STATUS_1: 大端 int32 rpm@byte0, 有符号 int16 current@byte4 (scale 100)
    db = CodecDatabase([
        MessageDef("STATUS_1", can_id=0x0901, is_extended=True, dlc=8, signals=[
            SignalDef("rpm", start_byte=0, bit_length=32, byte_order="big", signed=True),
            SignalDef("current", start_byte=4, bit_length=16, byte_order="big",
                      signed=True, scale=0.01, unit="A"),
            SignalDef("pid_pos", start_byte=6, bit_length=16, byte_order="big",
                      signed=True, scale=1 / 50.0, unit="deg"),
        ]),
        MessageDef("FLAGS", can_id=0x100, is_extended=False, dlc=8, signals=[
            SignalDef("low_nibble", start_byte=0, start_bit=0, bit_length=4,
                      byte_order="big"),
            SignalDef("temp_le", start_byte=1, bit_length=16, byte_order="little",
                      signed=False, scale=0.1, unit="C"),
        ]),
    ])

    # rpm=2000 (0x000007D0), current=-1.5A -> raw -150 = 0xFF6A, pid_pos=90deg -> raw 4500=0x1194
    frame = bytes([0x00, 0x00, 0x07, 0xD0, 0xFF, 0x6A, 0x11, 0x94])
    decoded = db.decode_frame(0x0901, True, frame)
    print("STATUS_1 decoded:", decoded)
    assert decoded["STATUS_1.rpm"] == 2000, decoded
    assert abs(decoded["STATUS_1.current"] - (-1.5)) < 1e-9, decoded
    assert abs(decoded["STATUS_1.pid_pos"] - 90.0) < 1e-9, decoded

    # round-trip: encode 回去应与原帧一致
    re = db.encode_message("STATUS_1", {"rpm": 2000, "current": -1.5, "pid_pos": 90.0})
    print("STATUS_1 re-encoded:", re.hex())
    assert re == frame, (re.hex(), frame.hex())

    # 小端 + 子字节
    f2 = bytes([0x0A, 0x2C, 0x01, 0, 0, 0, 0, 0])  # low_nibble=0x0A&0xF=10, temp raw=0x012C=300 ->30.0
    d2 = db.decode_frame(0x100, False, f2)
    print("FLAGS decoded:", d2)
    assert d2["FLAGS.low_nibble"] == 10, d2
    assert abs(d2["FLAGS.temp_le"] - 30.0) < 1e-9, d2

    # IEEE-754 单精度浮点（大端 + 小端），仿力/力矩传感器协议
    fdb = CodecDatabase([
        MessageDef("FT", can_id=0x200, is_extended=False, dlc=8, signals=[
            SignalDef("Fx_be", start_byte=0, bit_length=32, byte_order="big", is_float=True, unit="N"),
            SignalDef("Fy_le", start_byte=4, bit_length=32, byte_order="little", is_float=True, unit="N"),
        ]),
    ])
    fframe = struct.pack(">f", 12.5) + struct.pack("<f", -3.25)
    fd = fdb.decode_frame(0x200, False, fframe)
    print("FT decoded:", fd)
    assert abs(fd["FT.Fx_be"] - 12.5) < 1e-6, fd
    assert abs(fd["FT.Fy_le"] - (-3.25)) < 1e-6, fd
    fre = fdb.encode_message("FT", {"Fx_be": 12.5, "Fy_le": -3.25})
    assert fre == fframe, (fre.hex(), fframe.hex())

    # JSON round-trip
    import tempfile, os
    p = os.path.join(tempfile.gettempdir(), "_codec_selftest.json")
    db.save_json(p)
    db2 = CodecDatabase.load_json(p)
    assert db2.decode_frame(0x0901, True, frame) == decoded
    os.remove(p)
    print("✅ codec self-test passed")
