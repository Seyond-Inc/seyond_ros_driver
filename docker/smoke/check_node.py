#!/usr/bin/env python3
"""Wait for an exact fully qualified node name using one persistent DDS participant."""

import argparse
import sys
import time

import rclpy


def wait_for_node(node, expected, timeout):
    deadline = time.monotonic() + timeout
    names = []
    while True:
        names = sorted(
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in node.get_node_names_and_namespaces()
        )
        if expected in names:
            print("\n".join(names))
            return 0
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            print(
                f"Timed out after {timeout:g} s waiting for node {expected} in the ROS graph; "
                f"last observed nodes: {', '.join(names) or '(none)'}",
                file=sys.stderr,
            )
            return 1
        rclpy.spin_once(node, timeout_sec=min(0.2, remaining))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("node")
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("seyond_smoke_graph_check")
    try:
        return wait_for_node(node, args.node, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
