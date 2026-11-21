#!/usr/bin/env python3
"""Synthetic Seyond Robin-W data for the smoke test: an .inno_pc file the driver replays.

No recording of a Seyond sensor exists in Metrolla's repos, so this writes one from the SDK's own
packet definitions (seyond_sdk/src/sdk_common/inno_lidar_packet.h, all structs packed):

  InnoCommonHeader  26 B  magic 0x176A, version 4.0, CRC32C checksum, size, source/sync nibbles,
                          lidar_type (1 = Robin-W), ts_start_us (double), lidar_mode, lidar_status
  InnoDataPacket    +44 B idx (frame), sub_idx, sub_seq, type:8|item_number:24, item_size, topic,
                          flag bits, roi angles, extend_reserved[4]   (70 B in all)
  InnoEnXyzPoint    40 B  x y z radius (float), ts_10us, scan_id, scan_idx, timestamp_s (double),
                          reflectance, intensity, facet, channel, firing, in_roi, is_2nd_return,
                          multi_return

Packets are INNO_ROBINW_ITEM_TYPE_XYZ_POINTCLOUD (type 8), single return. The checksum is the
SDK's InnoPacketReader::calculate_packet_crc32: CRC32C (the SSE4.2 crc32 instruction, start
0xffffffff, final xor) over the 6 version bytes and everything from the size field on. A file is
packets back to back (FileInput::read_packet_). Every point is RANGE_M from the sensor.

Usage: fake_seyond.py OUTPUT [--frames 60] [--packets-per-frame 10] [--points-per-packet 200]
"""

from __future__ import annotations

import argparse
import math
import struct

MAGIC_DATA = 0x176A
MAJOR, MINOR = 4, 0  # kInnoMajorVersionDataPacket, kInnoMinorVersionDataPacket
LIDAR_TYPE_ROBINW = 1
ITEM_TYPE_ROBINW_XYZ = 8
MULTI_RETURN_SINGLE = 1
COMMON_SIZE = 26
PACKET_HEADER_SIZE = 70
POINT_SIZE = 40
SIZE_OFFSET = 10  # offsetof(InnoCommonHeader, size)
RANGE_M = 10.0
SCAN_LINES = 192  # Robin-W
FRAME_US = 100_000  # 10 Hz
START_US = 1_767_323_045_000_000.0  # 2026-01-02 03:04:05 UTC, as the Hesai fake

_CRC32C = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ 0x82F63B78 if _c & 1 else _c >> 1
    _CRC32C.append(_c)


def crc32c(data: bytes, crc: int = 0xFFFFFFFF) -> int:
    for b in data:
        crc = _CRC32C[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc


def point(frame: int, n: int, per_frame: int, ts_10us: int) -> bytes:
    """Point n of a frame: azimuth -60..60 deg across the frame, elevation by scan line."""
    scan_id = n % SCAN_LINES
    az = math.radians(-60.0 + 120.0 * n / max(per_frame - 1, 1))
    el = math.radians(-12.0 + 24.0 * scan_id / (SCAN_LINES - 1))
    x = RANGE_M * math.cos(el) * math.cos(az)
    y = RANGE_M * math.cos(el) * math.sin(az)
    z = RANGE_M * math.sin(el)
    refl = (n * 7 + frame) % 256
    return struct.pack("<ffffHHHdHHBBBBBB", x, y, z, RANGE_M, ts_10us, scan_id, n // SCAN_LINES,
                       0.0, refl, refl, n % 5, n % 4, 0, 0, 0, 0)


def data_packet(frame: int, sub_seq: int, points: list[bytes], ts_start_us: float,
                last: bool) -> bytes:
    size = PACKET_HEADER_SIZE + POINT_SIZE * len(points)
    flags = (1 << 1)  # use_reflectance
    flags |= MULTI_RETURN_SINGLE << 2
    if last:
        flags |= (1 << 7) | (1 << 8)  # is_last_sub_frame, is_last_sequence
    if sub_seq == 0:
        flags |= 1 << 11  # is_first_sub_frame
    rest = struct.pack("<I", size)
    rest += struct.pack("<BB", 0 | (0 << 4), LIDAR_TYPE_ROBINW)  # source_id 0, sync type 0
    rest += struct.pack("<d", ts_start_us)
    rest += struct.pack("<BB", 3, 0)  # lidar_mode, lidar_status
    rest += struct.pack("<QHH", frame, 0, sub_seq)
    rest += struct.pack("<IHI", ITEM_TYPE_ROBINW_XYZ | (len(points) << 8), POINT_SIZE, 0)
    rest += struct.pack("<Hhh", flags, 0, 0)
    rest += b"\0" * 16  # extend_reserved[4]
    rest += b"".join(points)
    version = struct.pack("<HBBH", MAGIC_DATA, MAJOR, MINOR, 0)
    checksum = crc32c(version + rest) ^ 0xFFFFFFFF
    out = version + struct.pack("<I", checksum) + rest
    assert len(out) == size, (len(out), size)
    return out


def frames(count: int, packets_per_frame: int, points_per_packet: int):
    per_frame = packets_per_frame * points_per_packet
    for f in range(count):
        ts = START_US + f * FRAME_US
        for p in range(packets_per_frame):
            pts = []
            for k in range(points_per_packet):
                n = p * points_per_packet + k
                pts.append(point(f, n, per_frame, ts_10us=n * 5))  # 50 us apart
            yield data_packet(f, p, pts, ts, p == packets_per_frame - 1)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("output")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--packets-per-frame", type=int, default=10)
    ap.add_argument("--points-per-packet", type=int, default=200)
    a = ap.parse_args()
    n = 0
    with open(a.output, "wb") as out:
        for pkt in frames(a.frames, a.packets_per_frame, a.points_per_packet):
            out.write(pkt)
            n += 1
    print(f"fake_seyond: wrote {n} packets, {a.frames} frames of "
          f"{a.packets_per_frame * a.points_per_packet} points at {RANGE_M} m to {a.output}")


if __name__ == "__main__":
    main()
