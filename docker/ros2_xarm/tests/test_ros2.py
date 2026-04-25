#!/usr/bin/env python3
"""Smoke test: rclpy pub/sub roundtrip on the local DDS domain."""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    rclpy.init()
    received: list[str] = []

    node = Node("ros2_xarm_smoke_test")
    pub = node.create_publisher(String, "/ros2_xarm_smoke", 10)
    node.create_subscription(
        String,
        "/ros2_xarm_smoke",
        lambda msg: received.append(msg.data),
        10,
    )

    msg = String()
    msg.data = "hello from ros2_xarm smoke test"

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and not received:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not received:
        print("FAIL: no message received within 5s", file=sys.stderr)
        return 1
    print(f"OK: rclpy roundtrip received {received[0]!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
