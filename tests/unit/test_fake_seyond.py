"""Unit tier: the synthetic Robin-W of the smoke test against the SDK's packet definitions.

docker/smoke/fake_seyond.py writes InnoDataPacket records by hand; this pins its sizes and field
offsets to seyond_sdk/src/sdk_common/inno_lidar_packet.h (read as text) and its checksum to CRC32C.
The real check is in the smoke tier (seyond_node replays the file; a bad size or CRC gives no
cloud). Locally on 2026-11-21 every packet also passed the SDK's own
InnoDataPacketUtils::check_data_packet (a small C++ program linked against the built SDK).
"""

from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "docker" / "smoke"))
import fake_seyond as f  # noqa: E402

HEADER = REPO / "src/seyond_lidar_ros/src/seyond_sdk/src/sdk_common/inno_lidar_packet.h"
SIZES = {"uint8_t": 1, "int8_t": 1, "uint16_t": 2, "int16_t": 2, "uint32_t": 4, "int32_t": 4,
         "uint64_t": 8, "float": 4, "double": 8, "InnoTimestampUs": 8}


def header_text() -> str:
    if not HEADER.is_file():
        pytest.fail("the seyond_sdk submodule is not checked out: ABORT, not a pass")
    return HEADER.read_text()


def packed_size(text: str, name: str) -> int:
    body = re.search(r"DEFINE_INNO_COMPACT_STRUCT\(" + name + r"\) \{(.*?)\n\} " + name + ";", text, re.S).group(1)
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)
    body = re.sub(r"//[^\n]*", "", body)
    body = re.sub(r"#\s*if !defined\(_MSC_VER\).*?#endif", "", body, flags=re.S)  # flexible payload
    total, bits = 0, 0
    for typ, _, count, bitfield in re.findall(
            r"^\s*([A-Za-z_][\w]*)\s+\w+(\[(\d+)\])?\s*(?::\s*(\d+))?\s*;", body, re.M):
        if bitfield:
            bits += int(bitfield)
            if bits % (SIZES[typ] * 8) == 0:
                total += bits // 8
                bits = 0
            continue
        size = SIZES[typ] if typ in SIZES else packed_size(text, typ)
        total += size * (int(count) if count else 1)
    assert bits == 0, f"{name}: unfinished bit field"
    return total


def test_sizes_match_the_sdk():
    text = header_text()
    assert packed_size(text, "InnoCommonVersion") == 6
    assert packed_size(text, "InnoCommonHeader") == f.COMMON_SIZE == 26
    assert packed_size(text, "InnoDataPacket") == f.PACKET_HEADER_SIZE == 70
    assert packed_size(text, "InnoEnXyzPoint") == f.POINT_SIZE == 40


def test_constants_match_the_sdk():
    text = header_text()
    assert f"kInnoMagicNumberDataPacket = 0x{f.MAGIC_DATA:X}" in text
    assert f"kInnoMajorVersionDataPacket = {f.MAJOR};" in text
    assert f"INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD = {f.ITEM_TYPE_ROBINW_XYZ}," in text
    assert f"INNO_LIDAR_TYPE_ROBINW = {f.LIDAR_TYPE_ROBINW}," in text
    assert f"INNO_MULTIPLE_RETURN_MODE_SINGLE = {f.MULTI_RETURN_SINGLE}," in text


def test_crc32c():
    assert f.crc32c(b"123456789") ^ 0xFFFFFFFF == 0xE3069283  # the CRC-32C check value


def test_packet_layout_and_checksum():
    pkt = next(f.frames(1, 2, 5))
    magic, major, minor, _, checksum, size = struct.unpack_from("<HBBHII", pkt, 0)
    assert (magic, major, minor, size, len(pkt)) == (0x176A, 4, 0, 70 + 5 * 40, 270)
    # InnoPacketReader::calculate_packet_crc32: version, then everything from the size field on.
    assert checksum == f.crc32c(pkt[:6] + pkt[f.SIZE_OFFSET:]) ^ 0xFFFFFFFF
    type_items = struct.unpack_from("<I", pkt, 26 + 12)[0]
    assert type_items & 0xFF == 8 and type_items >> 8 == 5
    assert struct.unpack_from("<H", pkt, 26 + 16)[0] == 40  # item_size


def test_every_point_is_at_the_range():
    for pkt in f.frames(2, 3, 50):
        for i in range(struct.unpack_from("<I", pkt, 38)[0] >> 8):
            x, y, z, r = struct.unpack_from("<ffff", pkt, 70 + 40 * i)
            assert abs((x * x + y * y + z * z) ** 0.5 - f.RANGE_M) < 1e-4 and r == f.RANGE_M


def test_frames_and_sub_sequences():
    pkts = list(f.frames(3, 4, 2))
    idx = [struct.unpack_from("<QHH", p, 26) for p in pkts]
    assert idx == [(fr, 0, s) for fr in range(3) for s in range(4)]
    stamps = sorted({struct.unpack_from("<d", p, 16)[0] for p in pkts})
    assert [b - a for a, b in zip(stamps, stamps[1:])] == [f.FRAME_US, f.FRAME_US]
