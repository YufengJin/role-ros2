#!/usr/bin/env python3
"""Smoke test: xarm-python-sdk import + offline instantiation + pyserial."""
import sys


def main() -> int:
    import xarm
    from xarm.wrapper import XArmAPI

    print(f"OK: xarm-python-sdk version {xarm.version.__version__}")

    arm = XArmAPI("127.0.0.1", do_not_open=True)
    if arm.connected:
        print("FAIL: do_not_open=True but arm.connected is True", file=sys.stderr)
        return 1
    print("OK: XArmAPI instantiated with do_not_open=True (connected=False)")

    import serial  # pyserial — xarm's only third-party dep
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    print(f"OK: pyserial {serial.__version__} importable, {len(ports)} serial port(s) visible")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
