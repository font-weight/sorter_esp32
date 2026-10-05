"""Strict, dependency-free implementation of sorter UART v1."""
from dataclasses import dataclass
import re

MAX_PAYLOAD = 400
MAX_BODY = 480
MAX_FRAME = 512
MAX_OBJECTS = 8
U32_MAX = (1 << 32) - 1
I32_MIN, I32_MAX = -(1 << 31), (1 << 31) - 1


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Packet:
    type: str
    session: int
    seq: int
    payload: str


@dataclass(frozen=True)
class Detection:
    class_id: int
    x10: int
    y10: int
    pixels: int


@dataclass(frozen=True)
class Scene:
    camera_boot: int
    calibration_id: int
    objects: tuple


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def parse_u32(text: str, nonzero: bool = False) -> int:
    if not isinstance(text, str) or re.fullmatch(r"[0-9]+", text) is None:
        raise ProtocolError("Expected unsigned decimal integer")
    # Bound length before int() as input may be untrusted.
    stripped = text.lstrip("0") or "0"
    if len(stripped) > 10:
        raise ProtocolError("uint32 overflow")
    value = int(stripped)
    if value > U32_MAX or (nonzero and value == 0):
        raise ProtocolError("uint32 out of range")
    return value


def parse_i32(text: str) -> int:
    if not isinstance(text, str) or re.fullmatch(r"-?[0-9]+", text) is None:
        raise ProtocolError("Expected signed decimal integer")
    unsigned = text.removeprefix("-").lstrip("0") or "0"
    if len(unsigned) > 10:
        raise ProtocolError("int32 overflow")
    value = int(unsigned) * (-1 if text.startswith("-") else 1)
    if not I32_MIN <= value <= I32_MAX:
        raise ProtocolError("int32 out of range")
    return value


def _integer(value, minimum, maximum, field):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ProtocolError(f"{field} out of range")
    return value


def encode_scene(scene: Scene) -> str:
    _integer(scene.camera_boot, 1, U32_MAX, "camera_boot")
    _integer(scene.calibration_id, 1, U32_MAX, "calibration_id")
    if len(scene.objects) > MAX_OBJECTS:
        raise ProtocolError("Too many objects")
    rows = [f"{scene.camera_boot},{scene.calibration_id},{len(scene.objects)}"]
    for obj in scene.objects:
        _integer(obj.class_id, 1, 3, "class_id")
        _integer(obj.x10, I32_MIN, I32_MAX, "x10")
        _integer(obj.y10, I32_MIN, I32_MAX, "y10")
        _integer(obj.pixels, 1, U32_MAX, "pixels")
        rows.append(f"{obj.class_id},{obj.x10},{obj.y10},{obj.pixels}")
    result = ";".join(rows)
    if len(result) > MAX_PAYLOAD:
        raise ProtocolError("Scene exceeds payload limit")
    return result


def decode_scene(payload: str) -> Scene:
    if not isinstance(payload, str) or len(payload) > MAX_PAYLOAD:
        raise ProtocolError("Scene exceeds payload limit")
    rows = payload.split(";")
    head = rows[0].split(",")
    if len(head) != 3:
        raise ProtocolError("Scene header must have three fields")
    boot, cal_id, count = (parse_u32(head[0], True), parse_u32(head[1], True), parse_u32(head[2]))
    if count > MAX_OBJECTS or len(rows) != count + 1:
        raise ProtocolError("Scene object count mismatch")
    objects = []
    for row in rows[1:]:
        values = row.split(",")
        if len(values) != 4:
            raise ProtocolError("Detection must have four fields")
        cls, x10, y10, pixels = (parse_u32(values[0]), parse_i32(values[1]),
                                parse_i32(values[2]), parse_u32(values[3], True))
        if not 1 <= cls <= 3:
            raise ProtocolError("class_id must be 1..3")
        objects.append(Detection(cls, x10, y10, pixels))
    return Scene(boot, cal_id, tuple(objects))


def validate_payload(packet: Packet):
    if packet.type == "Q":
        return parse_u32(packet.payload, True)
    if packet.type == "D":
        return decode_scene(packet.payload)
    if packet.type == "E":
        if re.fullmatch(r"[A-Z][A-Z0-9_]{0,31}", packet.payload) is None:
            raise ProtocolError("Invalid error code")
        return packet.payload
    raise ProtocolError("Unknown packet type")


def encode_packet(packet: Packet) -> bytes:
    _integer(packet.session, 1, U32_MAX, "session")
    _integer(packet.seq, 1, U32_MAX, "seq")
    if packet.type not in ("Q", "D", "E"):
        raise ProtocolError("Unknown packet type")
    if not isinstance(packet.payload, str) or any(c in packet.payload for c in "@|*\r\n"):
        raise ProtocolError("Forbidden payload character")
    try:
        raw_payload = packet.payload.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolError("Payload is not ASCII") from exc
    if len(raw_payload) > MAX_PAYLOAD or any(c < 32 or c > 126 for c in raw_payload):
        raise ProtocolError("Payload length or character range invalid")
    validate_payload(packet)
    body = f"1|{packet.type}|{packet.session}|{packet.seq}|{packet.payload}".encode("ascii")
    if len(body) > MAX_BODY:
        raise ProtocolError("Body too long")
    return b"@" + body + f"*{crc16(body):04X}\n".encode("ascii")


def decode_packet(frame: bytes | str) -> Packet:
    try:
        data = frame.encode("ascii") if isinstance(frame, str) else bytes(frame)
        line = data.decode("ascii")
    except (UnicodeError, TypeError, ValueError) as exc:
        raise ProtocolError("Frame is not ASCII bytes") from exc
    if len(data) >= MAX_FRAME:
        raise ProtocolError("Frame too long")
    if line.endswith("\n"):
        line = line[:-1]
    if line.endswith("\r"):
        line = line[:-1]
    if not line.startswith("@") or len(line) < 8 or line.count("*") != 1:
        raise ProtocolError("Invalid framing")
    body, checksum = line[1:].split("*")
    if len(body) > MAX_BODY or re.fullmatch(r"[0-9a-fA-F]{4}", checksum) is None:
        raise ProtocolError("Invalid CRC field")
    if crc16(body.encode("ascii")) != int(checksum, 16):
        raise ProtocolError("CRC mismatch")
    fields = body.split("|")
    if len(fields) != 5 or fields[0] != "1":
        raise ProtocolError("Invalid version or field count")
    packet = Packet(fields[1], parse_u32(fields[2], True), parse_u32(fields[3], True), fields[4])
    # Canonical encoding also applies type-specific range and payload checks.
    encode_packet(packet)
    return packet


class LineParser:
    """Streaming parser. Junk is ignored; a fresh @ resynchronizes the stream."""

    def __init__(self):
        self._buffer = bytearray()
        self.errors = 0

    def reset(self):
        self._buffer.clear()

    def feed(self, data: bytes) -> list[Packet]:
        packets = []
        for value in data:
            if value == ord("@"):
                if self._buffer:
                    self.errors += 1
                self._buffer = bytearray(b"@")
            elif not self._buffer:
                continue
            elif value == 10:
                try:
                    packets.append(decode_packet(bytes(self._buffer)))
                except ProtocolError:
                    self.errors += 1
                self.reset()
            elif len(self._buffer) >= MAX_FRAME - 2 or value == 0:
                self.errors += 1
                self.reset()
            else:
                self._buffer.append(value)
        return packets
