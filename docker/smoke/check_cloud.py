#!/usr/bin/env python3
"""Smoke checker: subscribe to a PointCloud2 topic and require clouds decoded from fake_seyond.py.

Requires N clouds (after the first, which can be partial) on TOPIC with header.frame_id FRAME,
fields x y z intensity among them, at least MIN_POINTS points each, every point RANGE_M from the
sensor (the driver's coordinate_mode 3 only permutes axes), and header stamps 100 ms apart from the
synthetic frames' start times. Prints a JSON summary; exit 0 on success, 1 otherwise.

Usage: check_cloud.py TOPIC FRAME [--clouds 3] [--min-points 2000] [--timeout 60]
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

RANGE_M = 10.0
TOLERANCE_M = 0.001
START_S = 1767323045.0  # fake_seyond.START_US
FRAME_S = 0.1


def summarise(m: PointCloud2) -> dict:
    data = bytes(m.data)
    off = {f.name: f.offset for f in m.fields}
    n = m.width * m.height
    bad, lo, hi = 0, math.inf, 0.0
    for i in range(n):
        x, y, z = struct.unpack_from("<fff", data, i * m.point_step + off["x"])
        r = math.sqrt(x * x + y * y + z * z)
        lo, hi = min(lo, r), max(hi, r)
        if abs(r - RANGE_M) > TOLERANCE_M:
            bad += 1
    stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    return {"stamp": f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}", "frame_id": m.header.frame_id,
            "points": n, "fields": [f.name for f in m.fields], "point_step": m.point_step,
            "range_min": round(lo, 5), "range_max": round(hi, 5), "off_range": bad,
            "frame_number": round((stamp - START_S) / FRAME_S, 3)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("frame")
    ap.add_argument("--clouds", type=int, default=3)
    ap.add_argument("--min-points", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=60.0)
    a = ap.parse_args()
    got: list[PointCloud2] = []
    rclpy.init()
    node = rclpy.create_node("seyond_smoke_check")
    node.create_subscription(PointCloud2, a.topic, got.append,
                             QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
    t0 = time.monotonic()
    while len(got) < a.clouds + 1 and time.monotonic() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()
    clouds = [summarise(m) for m in got[1:a.clouds + 1]]
    problems = []
    if len(clouds) < a.clouds:
        problems.append(f"{len(got)} clouds on {a.topic} in {a.timeout:g} s, wanted {a.clouds + 1}")
    for c in clouds:
        if c["frame_id"] != a.frame:
            problems.append(f"frame_id {c['frame_id']!r}, wanted {a.frame!r}")
        if not {"x", "y", "z", "intensity"} <= set(c["fields"]):
            problems.append(f"fields {c['fields']} lack x y z intensity")
        if c["points"] < a.min_points:
            problems.append(f"{c['points']} points, wanted at least {a.min_points}")
        if c["off_range"]:
            problems.append(f"{c['off_range']} points not {RANGE_M} m from the sensor")
        if abs(c["frame_number"] - round(c["frame_number"])) > 1e-3 or c["frame_number"] < 0:
            problems.append(f"stamp {c['stamp']} is not a synthetic frame start")
    print(json.dumps({"topic": a.topic, "clouds": clouds, "problems": problems}, indent=1))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
