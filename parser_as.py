from __future__ import annotations

"""
parser_as_improved.py
=====================

用途
----
這是一份加強版 Area Scanner 封包解析器，保留原本 parser_as.py 的核心 API：
- AreaScannerParser
- ParsedFrame
- parse_packet

這版特別補強兩件事：
1. **TLV 長度模式判斷更嚴謹**：避免 payload-only / inclusive 模式判錯，
   導致 dynamic point 看得到，但 target list (TLV type 10) 被吃掉或對齊錯位。
2. **目標偵測診斷欄位**：會把 TLV type 清單、採用的解析模式、警告訊息一併存進 frame，
   方便 GUI 或 console 快速判斷「到底是沒有 target TLV，還是 tracker 沒分配出目標」。

建議環境
--------
- Python 3.10.x
"""

from dataclasses import dataclass, field
from typing import List, Literal, Optional, Tuple
import math
import struct

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_LEN = 44
TLV_HEADER_LEN = 8
DEFAULT_MAX_BUFFER_SIZE = 2 ** 16

# TI Area Scanner / MATLAB parseBytes_AS.m 對照的 TLV type
MMWDEMO_UART_MSG_DETECTED_POINTS = 1
MMWDEMO_UART_MSG_RANGE_PROFILE = 2
MMWDEMO_UART_MSG_NOISE_PROFILE = 3
MMWDEMO_UART_MSG_AZIMUT_STATIC_HEAT_MAP = 4
MMWDEMO_UART_MSG_RANGE_DOPPLER_HEAT_MAP = 5
MMWDEMO_UART_MSG_STATS = 6
MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO = 7
MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS = 8
MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS_SIDE_INFO = 9
MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST = 10
MMWDEMO_UART_MSG_TRACKERPROC_TARGET_INDEX = 11
KNOWN_TLV_TYPES = set(range(1, 12))


class PacketFormatError(Exception):
    """封包格式不正確或無法合理解析。"""


@dataclass(slots=True)
class FrameHeader:
    magic_word: bytes
    version: int
    total_packet_len: int
    platform: int
    frame_number: int
    time_cpu_cycles: int
    num_detected_obj: int
    num_tlvs: int
    sub_frame_number: int
    num_static_detected_obj: int


@dataclass(slots=True)
class TLVRecord:
    tlv_type: int
    tlv_length: int
    payload_length: int


@dataclass(slots=True)
class DynamicPoint:
    raw_range: float
    raw_angle: float
    raw_elev: float
    doppler: float
    x: float
    y: float
    z: float


@dataclass(slots=True)
class StaticPoint:
    x: float
    y: float
    z: float
    doppler: float


@dataclass(slots=True)
class SideInfoEntry:
    snr: int
    noise: int


@dataclass(slots=True)
class TargetRecord:
    tid: int
    pos_x: float
    pos_y: float
    vel_x: float
    vel_y: float
    acc_x: float
    acc_y: float
    pos_z: float
    vel_z: float
    acc_z: float


@dataclass(slots=True)
class ParsedFrame:
    header: FrameHeader
    tlvs: List[TLVRecord] = field(default_factory=list)
    dynamic_points: List[DynamicPoint] = field(default_factory=list)
    dynamic_side_info: List[SideInfoEntry] = field(default_factory=list)
    static_points: List[StaticPoint] = field(default_factory=list)
    static_side_info: List[SideInfoEntry] = field(default_factory=list)
    targets: List[TargetRecord] = field(default_factory=list)
    target_indices: List[int] = field(default_factory=list)

    # 加強版診斷欄位
    tlv_types: List[int] = field(default_factory=list)
    tlv_length_mode: str = ""
    has_target_list_tlv: bool = False
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "header": {
                "frame_number": self.header.frame_number,
                "num_tlvs": self.header.num_tlvs,
                "num_detected_obj": self.header.num_detected_obj,
                "num_static_detected_obj": self.header.num_static_detected_obj,
                "total_packet_len": self.header.total_packet_len,
            },
            "num_tlvs": self.header.num_tlvs,
            "dynamic_points": [
                {"x": p.x, "y": p.y, "z": p.z, "doppler": p.doppler}
                for p in self.dynamic_points
            ],
            "static_points": [
                {"x": p.x, "y": p.y, "z": p.z, "doppler": p.doppler}
                for p in self.static_points
            ],
            "tracked_targets": [
                {
                    "tid": t.tid,
                    "x": t.pos_x,
                    "y": t.pos_y,
                    "z": t.pos_z,
                    "vx": t.vel_x,
                    "vy": t.vel_y,
                    "vz": t.vel_z,
                }
                for t in self.targets
            ],
            "target_indices": list(self.target_indices),
            "tlv_types": list(self.tlv_types),
            "tlv_length_mode": self.tlv_length_mode,
            "has_target_list_tlv": self.has_target_list_tlv,
            "warnings": list(self.warnings),
        }


def parse_frame_header(packet: bytes) -> FrameHeader:
    if len(packet) < HEADER_LEN:
        raise PacketFormatError("封包長度不足，無法解析 frame header。")

    unpacked = struct.unpack_from("<8s9I", packet, 0)
    return FrameHeader(
        magic_word=unpacked[0],
        version=unpacked[1],
        total_packet_len=unpacked[2],
        platform=unpacked[3],
        frame_number=unpacked[4],
        time_cpu_cycles=unpacked[5],
        num_detected_obj=unpacked[6],
        num_tlvs=unpacked[7],
        sub_frame_number=unpacked[8],
        num_static_detected_obj=unpacked[9],
    )


def parse_dynamic_points(payload: bytes) -> List[DynamicPoint]:
    point_size = 16
    if len(payload) % point_size != 0:
        raise PacketFormatError(f"dynamic point payload 長度不是 16 的倍數：{len(payload)}")

    points: List[DynamicPoint] = []
    for offset in range(0, len(payload), point_size):
        point_range, angle, elev, doppler = struct.unpack_from("<ffff", payload, offset)
        z = point_range * math.sin(elev)
        r = point_range * math.cos(elev)
        y = r * math.cos(angle)
        x = r * math.sin(angle)
        points.append(
            DynamicPoint(
                raw_range=point_range,
                raw_angle=angle,
                raw_elev=elev,
                doppler=doppler,
                x=x,
                y=y,
                z=z,
            )
        )
    return points


def parse_static_points(payload: bytes) -> List[StaticPoint]:
    point_size = 16
    if len(payload) % point_size != 0:
        raise PacketFormatError(f"static point payload 長度不是 16 的倍數：{len(payload)}")

    points: List[StaticPoint] = []
    for offset in range(0, len(payload), point_size):
        x, y, z, doppler = struct.unpack_from("<ffff", payload, offset)
        points.append(StaticPoint(x=x, y=y, z=z, doppler=doppler))
    return points


def parse_side_info(payload: bytes) -> List[SideInfoEntry]:
    entry_size = 4
    if len(payload) % entry_size != 0:
        raise PacketFormatError(f"side info payload 長度不是 4 的倍數：{len(payload)}")

    entries: List[SideInfoEntry] = []
    for offset in range(0, len(payload), entry_size):
        snr, noise = struct.unpack_from("<hh", payload, offset)
        entries.append(SideInfoEntry(snr=snr, noise=noise))
    return entries


def parse_target_list(payload: bytes) -> List[TargetRecord]:
    target_size = 40

    # 某些版本若尾端有多餘 bytes，這裡採用「吃完整 40-byte chunk、忽略零頭」的保守策略。
    usable = (len(payload) // target_size) * target_size
    if usable == 0:
        return []

    targets: List[TargetRecord] = []
    for offset in range(0, usable, target_size):
        tid, pos_x, pos_y, vel_x, vel_y, acc_x, acc_y, pos_z, vel_z, acc_z = struct.unpack_from(
            "<I9f", payload, offset
        )
        targets.append(
            TargetRecord(
                tid=tid,
                pos_x=pos_x,
                pos_y=pos_y,
                vel_x=vel_x,
                vel_y=vel_y,
                acc_x=acc_x,
                acc_y=acc_y,
                pos_z=pos_z,
                vel_z=vel_z,
                acc_z=acc_z,
            )
        )
    return targets


def parse_target_indices(payload: bytes) -> List[int]:
    return list(payload)


def _score_frame(frame: ParsedFrame, final_offset: int, packet_len: int) -> int:
    """幫兩種 TLV length 模式打分，選比較合理的那個。"""
    score = 0

    # TLV type 愈多落在已知範圍愈好。
    for t in frame.tlv_types:
        score += 3 if t in KNOWN_TLV_TYPES else -4

    # 跟 header 宣告數量對得上會加分。
    if len(frame.dynamic_points) == frame.header.num_detected_obj:
        score += 8
    elif frame.header.num_detected_obj == 0 and len(frame.dynamic_points) == 0:
        score += 5
    else:
        score -= 8

    if len(frame.static_points) == frame.header.num_static_detected_obj:
        score += 8
    elif frame.header.num_static_detected_obj == 0 and len(frame.static_points) == 0:
        score += 5
    else:
        score -= 8

    # target TLV 出現不一定代表一定有 target，但出現本身有助於判斷模式沒歪。
    if frame.has_target_list_tlv:
        score += 6

    # 剩餘尾巴若很小，通常只是 packet padding；太大則代表 offset 可能歪了。
    leftover = packet_len - final_offset
    if leftover == 0:
        score += 6
    elif 0 < leftover <= 32:
        score += 3
    else:
        score -= 6

    return score


def _parse_packet_with_mode(
    packet: bytes,
    tlv_length_mode: Literal["payload_only", "inclusive"],
) -> Tuple[ParsedFrame, int]:
    header = parse_frame_header(packet)

    if header.magic_word != MAGIC_WORD:
        raise PacketFormatError("magic word 不正確。")

    if header.total_packet_len != len(packet):
        raise PacketFormatError(
            f"封包長度不符：header={header.total_packet_len}, actual={len(packet)}"
        )

    frame = ParsedFrame(header=header)
    frame.tlv_length_mode = tlv_length_mode
    offset = HEADER_LEN

    for tlv_index in range(header.num_tlvs):
        if offset + TLV_HEADER_LEN > len(packet):
            raise PacketFormatError(f"TLV header 不完整，index={tlv_index}")

        tlv_type, tlv_length = struct.unpack_from("<II", packet, offset)
        offset += TLV_HEADER_LEN

        payload_length = tlv_length if tlv_length_mode == "payload_only" else tlv_length - TLV_HEADER_LEN
        if payload_length < 0:
            raise PacketFormatError(
                f"TLV payload_length 變成負數，type={tlv_type}, length={tlv_length}"
            )
        if offset + payload_length > len(packet):
            raise PacketFormatError(
                f"TLV 超出封包範圍，index={tlv_index}, type={tlv_type}, "
                f"payload_length={payload_length}, offset={offset}, packet_len={len(packet)}"
            )

        payload = packet[offset: offset + payload_length]
        offset += payload_length

        frame.tlvs.append(TLVRecord(tlv_type=tlv_type, tlv_length=tlv_length, payload_length=payload_length))
        frame.tlv_types.append(tlv_type)
        if tlv_type == MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST:
            frame.has_target_list_tlv = True

        try:
            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
                frame.dynamic_points = parse_dynamic_points(payload)
            elif tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO:
                frame.dynamic_side_info = parse_side_info(payload)
            elif tlv_type == MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS:
                frame.static_points = parse_static_points(payload)
            elif tlv_type == MMWDEMO_UART_MSG_STATIC_DETECTED_POINTS_SIDE_INFO:
                frame.static_side_info = parse_side_info(payload)
            elif tlv_type == MMWDEMO_UART_MSG_TRACKERPROC_TARGET_LIST:
                frame.targets = parse_target_list(payload)
            elif tlv_type == MMWDEMO_UART_MSG_TRACKERPROC_TARGET_INDEX:
                frame.target_indices = parse_target_indices(payload)
        except Exception as exc:
            frame.warnings.append(f"TLV {tlv_type} 解析警告：{exc}")

    return frame, offset


def parse_packet(packet: bytes) -> ParsedFrame:
    candidates: List[Tuple[int, ParsedFrame]] = []
    errors: List[str] = []

    for mode in ("payload_only", "inclusive"):
        try:
            frame, final_offset = _parse_packet_with_mode(packet, mode)  # type: ignore[arg-type]
            candidates.append((_score_frame(frame, final_offset, len(packet)), frame))
        except Exception as exc:
            errors.append(f"{mode}: {exc}")

    if not candidates:
        raise PacketFormatError("packet 解析失敗：" + " | ".join(errors))

    candidates.sort(key=lambda item: item[0], reverse=True)
    best = candidates[0][1]

    if best.header.num_detected_obj > 0 and not best.has_target_list_tlv:
        best.warnings.append(
            "本 frame 有 dynamic point，但沒有看到 TLV type 10 (TRACKERPROC_TARGET_LIST)。"
        )
    if best.has_target_list_tlv and len(best.targets) == 0:
        best.warnings.append(
            "本 frame 有 target TLV，但 target 數量為 0；這通常表示 tracker 尚未分配出穩定目標。"
        )
    return best


class AreaScannerParser:
    def __init__(self, max_buffer_size: int = DEFAULT_MAX_BUFFER_SIZE) -> None:
        self.max_buffer_size = max_buffer_size
        self.buffer = bytearray()

    def clear(self) -> None:
        self.buffer.clear()

    def append_data(self, data: bytes) -> None:
        if not data:
            return
        self.buffer.extend(data)
        if len(self.buffer) > self.max_buffer_size:
            del self.buffer[:-self.max_buffer_size]

    def _extract_one_packet(self) -> Optional[bytes]:
        start = self.buffer.find(MAGIC_WORD)
        if start == -1:
            keep = len(MAGIC_WORD) - 1
            if len(self.buffer) > keep:
                del self.buffer[:-keep]
            return None

        if start > 0:
            del self.buffer[:start]

        if len(self.buffer) < HEADER_LEN:
            return None

        try:
            header = parse_frame_header(self.buffer[:HEADER_LEN])
        except Exception:
            del self.buffer[0]
            return None

        total_len = header.total_packet_len
        if total_len < HEADER_LEN or total_len > self.max_buffer_size:
            del self.buffer[0]
            return None

        if len(self.buffer) < total_len:
            return None

        packet = bytes(self.buffer[:total_len])
        del self.buffer[:total_len]
        return packet

    def extract_packets(self) -> List[bytes]:
        packets: List[bytes] = []
        while True:
            packet = self._extract_one_packet()
            if packet is None:
                break
            packets.append(packet)
        return packets

    def feed_and_parse(self, data: bytes) -> List[ParsedFrame]:
        self.append_data(data)
        frames: List[ParsedFrame] = []
        for packet in self.extract_packets():
            frames.append(parse_packet(packet))
        return frames
