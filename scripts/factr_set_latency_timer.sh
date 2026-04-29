#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  factr-set-latency [--config PATH] [--port PORT_OR_BY_ID] [--latency N]

Examples:
  factr-set-latency --config config/factr_teleop_config.yaml
  factr-set-latency --port usb-FTDI_USB__-__Serial_Converter_FTAKRKHW-if00-port0
  factr-set-latency --port /dev/ttyUSB0 --latency 1

If --port is omitted, the script tries FACTR_DYNAMIXEL_PORT, then reads
dynamixel.dynamixel_port from FACTR_CONFIG or config/factr_teleop_config.yaml.
EOF
}

CONFIG_PATH="${FACTR_CONFIG:-config/factr_teleop_config.yaml}"
PORT="${FACTR_DYNAMIXEL_PORT:-}"
LATENCY="${FACTR_FTDI_LATENCY_TIMER:-1}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG_PATH="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --latency)
            LATENCY="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -z "${PORT}" && -f "${CONFIG_PATH}" ]]; then
    PORT="$(
        python3 - "${CONFIG_PATH}" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
print((cfg.get("dynamixel") or {}).get("dynamixel_port", ""))
PY
    )"
fi

if [[ -z "${PORT}" ]]; then
    echo "FACTR Dynamixel port is unknown. Pass --port or set FACTR_DYNAMIXEL_PORT." >&2
    exit 1
fi

case "${PORT}" in
    /dev/serial/by-id/*|/dev/ttyUSB*|/dev/ttyACM*)
        DEVICE="${PORT}"
        ;;
    ttyUSB*|ttyACM*)
        DEVICE="/dev/${PORT}"
        ;;
    *)
        DEVICE="/dev/serial/by-id/${PORT}"
        ;;
esac

if [[ ! -e "${DEVICE}" ]]; then
    echo "FACTR serial device does not exist: ${DEVICE}" >&2
    echo "Available by-id devices:" >&2
    ls -l /dev/serial/by-id 2>/dev/null || true
    exit 1
fi

RESOLVED_DEVICE="$(readlink -f "${DEVICE}")"
TTY_NAME="$(basename "${RESOLVED_DEVICE}")"
LATENCY_PATH="/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"

if [[ ! -e "${LATENCY_PATH}" ]]; then
    echo "Latency timer path does not exist: ${LATENCY_PATH}" >&2
    echo "This is usually available for FTDI ttyUSB devices. Resolved device: ${RESOLVED_DEVICE}" >&2
    exit 1
fi

CURRENT="$(cat "${LATENCY_PATH}")"
if [[ "${CURRENT}" == "${LATENCY}" ]]; then
    echo "FACTR ${TTY_NAME} latency_timer already ${LATENCY} ms"
    exit 0
fi

echo "${LATENCY}" > "${LATENCY_PATH}"
UPDATED="$(cat "${LATENCY_PATH}")"
echo "FACTR ${TTY_NAME} latency_timer: ${CURRENT} -> ${UPDATED} ms"
