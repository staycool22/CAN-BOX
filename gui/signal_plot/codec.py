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
class Segment:
    """信号的一个位段（多段拼接用）。每段内部按大端连续读取。"""
    start_byte: int = 0
    start_bit: int = 0          # 0~7，字节内 LSB=0
    bit_length: int = 8


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
    # 非空=多段拼接：列表顺序即 MSB→LSB，第一段贡献最高位；为空=用上面的单段字段
    segments: List["Segment"] = field(default_factory=list)

    def total_bits(self) -> int:
        """有效总位宽：有 segments 时为各段之和，否则为 bit_length。"""
        if self.segments:
            return sum(s.bit_length for s in self.segments)
        return self.bit_length

    def validate(self) -> None:
        if not self.name:
            raise ValueError("信号名不能为空")
        if self.byte_order not in ("big", "little"):
            raise ValueError("字节序必须是 'big' 或 'little'")
        if self.scale == 0:
            raise ValueError("缩放系数不能为 0")
        if self.segments:
            for seg in self.segments:
                if seg.start_byte < 0:
                    raise ValueError("段起始字节必须 >= 0")
                if not (0 <= seg.start_bit <= 7):
                    raise ValueError("段起始位必须在 0~7 之间")
                if seg.bit_length <= 0:
                    raise ValueError("段位长度必须 > 0")
            if self.is_float and self.total_bits() not in (32, 64):
                raise ValueError("IEEE-754 浮点（多段拼接）的总位宽必须是 32 或 64")
            return
        if self.start_byte < 0:
            raise ValueError("起始字节必须 >= 0")
        if not (0 <= self.start_bit <= 7):
            raise ValueError("起始位必须在 0~7 之间")
        if self.bit_length <= 0:
            raise ValueError("位长度必须 > 0")
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
            sigs = []
            for sd in md.get("signals", []):
                sd = dict(sd)
                seg_list = [Segment(**x) for x in sd.pop("segments", [])]
                sigs.append(SignalDef(segments=seg_list, **sd))
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

def _extract_bits(b: bytes, start_byte: int, start_bit: int, bit_length: int,
                  byte_order: str = "big") -> int:
    """从字节序列抽取一段连续位的无符号整数（单段原语）。"""
    if byte_order == "little":
        # 小端：(start_byte, start_bit) 为最低位，向高位读取
        raw = 0
        base = start_byte * 8 + start_bit
        for i in range(bit_length):
            p = base + i
            byte_idx = p >> 3
            if byte_idx >= len(b):
                break
            bit = (b[byte_idx] >> (p & 7)) & 1
            raw |= bit << i
        return raw
    # 大端：从 start_byte 起按大端组合若干字节，再右移 start_bit、取 bit_length 位
    span = (start_bit + bit_length + 7) // 8
    raw_span = 0
    for i in range(span):
        byte_idx = start_byte + i
        val = b[byte_idx] if byte_idx < len(b) else 0
        raw_span = (raw_span << 8) | val
    return (raw_span >> start_bit) & ((1 << bit_length) - 1)


def _write_bits(buf: bytearray, start_byte: int, start_bit: int, bit_length: int,
                raw: int, byte_order: str = "big") -> None:
    """把一段连续位写回 buf（_extract_bits 的逆，单段原语）。"""
    raw &= (1 << bit_length) - 1
    if byte_order == "little":
        base = start_byte * 8 + start_bit
        for i in range(bit_length):
            p = base + i
            byte_idx = p >> 3
            if byte_idx >= len(buf):
                break
            if (raw >> i) & 1:
                buf[byte_idx] |= (1 << (p & 7))
            else:
                buf[byte_idx] &= ~(1 << (p & 7)) & 0xFF
        return
    span = (start_bit + bit_length + 7) // 8
    field_shifted = raw << start_bit
    for i in range(span):
        byte_idx = start_byte + (span - 1 - i)
        if byte_idx >= len(buf):
            continue
        buf[byte_idx] |= (field_shifted >> (8 * i)) & 0xFF


def _extract_raw(sig: SignalDef, data) -> int:
    """抽取信号的无符号原始整数（不含缩放/偏移/符号扩展）。

    多段：按 segments 顺序（MSB→LSB）逐段大端抽取再拼接；单段：用 sig 自身字段。
    """
    b = bytes(data)
    if sig.segments:
        raw = 0
        for seg in sig.segments:
            seg_raw = _extract_bits(b, seg.start_byte, seg.start_bit, seg.bit_length, "big")
            raw = (raw << seg.bit_length) | seg_raw
        return raw
    return _extract_bits(b, sig.start_byte, sig.start_bit, sig.bit_length, sig.byte_order)


def _float_fmt(sig: SignalDef) -> str:
    return (">" if sig.byte_order == "big" else "<") + ("f" if sig.total_bits() == 32 else "d")


def decode_signal(sig: SignalDef, data) -> float:
    """解码单个信号为工程量。"""
    raw = _extract_raw(sig, data)
    nbits = sig.total_bits()
    if sig.is_float:
        value = struct.unpack(_float_fmt(sig), raw.to_bytes(nbits // 8, sig.byte_order))[0]
        return value * sig.scale + sig.offset
    if sig.signed and (raw >> (nbits - 1)) & 1:
        raw -= (1 << nbits)
    return raw * sig.scale + sig.offset


def encode_signal(sig: SignalDef, value: float, buf: bytearray) -> None:
    """把工程量写回 buf 的对应位域（encode/round-trip 用）。"""
    nbits = sig.total_bits()
    mask = (1 << nbits) - 1
    if sig.is_float:
        phys = (float(value) - sig.offset) / sig.scale
        raw = int.from_bytes(struct.pack(_float_fmt(sig), phys), sig.byte_order) & mask
    else:
        raw = int(round((float(value) - sig.offset) / sig.scale))
        if sig.signed:
            lo, hi = -(1 << (nbits - 1)), (1 << (nbits - 1)) - 1
            raw = max(lo, min(hi, raw))
            raw &= mask  # 转两's complement 位模式
        else:
            raw = max(0, min(mask, raw))
    if sig.segments:
        # 按 MSB→LSB 把 raw 拆回各段并大端写入
        pos = nbits
        for seg in sig.segments:
            pos -= seg.bit_length
            seg_val = (raw >> pos) & ((1 << seg.bit_length) - 1)
            _write_bits(buf, seg.start_byte, seg.start_bit, seg.bit_length, seg_val, "big")
        return
    _write_bits(buf, sig.start_byte, sig.start_bit, sig.bit_length, raw, sig.byte_order)


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

    # 多段拼接：rpm 高字节@byte0 + 低字节@byte7 拼成 16 位有符号
    sdb = CodecDatabase([
        MessageDef("SPLIT", can_id=0x300, is_extended=False, dlc=8, signals=[
            SignalDef("rpm", segments=[Segment(0, 0, 8), Segment(7, 0, 8)], signed=True),
            # 含子字节段：高 4 位@byte1 高半字节 + 低 8 位@byte2 -> 12 位无符号
            SignalDef("code", segments=[Segment(1, 4, 4), Segment(2, 0, 8)], signed=False),
        ]),
    ])
    sframe = bytes([0x12, 0xA0, 0xBC, 0, 0, 0, 0, 0x34])  # rpm=0x1234, code=(0xA<<8)|0xBC=0xABC
    sd = sdb.decode_frame(0x300, False, sframe)
    print("SPLIT decoded:", sd)
    assert sd["SPLIT.rpm"] == 0x1234, sd
    assert sd["SPLIT.code"] == 0xABC, sd
    assert sdb.messages[0].signals[0].total_bits() == 16
    # round-trip
    sre = sdb.encode_message("SPLIT", {"rpm": 0x1234, "code": 0xABC})
    assert sre == sframe, (sre.hex(), sframe.hex())

    # 多段 + 有符号负值
    neg = bytes([0xFF, 0, 0, 0, 0, 0, 0, 0xFF])  # 0xFFFF = -1 (16位有符号)
    assert sdb.decode_frame(0x300, False, neg)["SPLIT.rpm"] == -1

    # 多段定义 JSON 往返
    import tempfile as _tf, os as _os
    sp = _os.path.join(_tf.gettempdir(), "_codec_seg.json")
    sdb.save_json(sp)
    sdb2 = CodecDatabase.load_json(sp)
    assert sdb2.decode_frame(0x300, False, sframe) == sd
    _os.remove(sp)

    # JSON round-trip
    import tempfile, os
    p = os.path.join(tempfile.gettempdir(), "_codec_selftest.json")
    db.save_json(p)
    db2 = CodecDatabase.load_json(p)
    assert db2.decode_frame(0x0901, True, frame) == decoded
    os.remove(p)
    print("✅ codec self-test passed")
