#!/usr/bin/env python3
"""Smoke test: rclpy and xarm-python-sdk coexist in the same Python process.

This is the key integration check — if rclpy's C extensions and xarm's
imports conflict (e.g. numpy ABI, libstdc++ versions), it fails here.
"""
import sys


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from xarm.wrapper import XArmAPI
    import xarm

    rclpy.init()
    try:
        node = Node("ros2_xarm_combined_smoke")
        arm = XArmAPI("127.0.0.1", do_not_open=True)
        if arm.connected:
            print("FAIL: xarm should be offline with do_not_open=True", file=sys.stderr)
            return 1
        print(
            f"OK: rclpy + xarm=={xarm.version.__version__} "
            f"share Python {sys.version_info.major}.{sys.version_info.minor}"
        )
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
