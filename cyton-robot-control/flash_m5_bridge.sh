#!/usr/bin/env bash
# Build and upload the M5StickC Plus USB-to-UART bridge firmware.
# Usage: bash flash_m5_bridge.sh [serial-port]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$ROOT/outputs/m5_uart_bridge"

if ! command -v python3 >/dev/null 2>&1; then
  echo "Python 3 is required. Install Python 3, then run this script again." >&2
  exit 1
fi

if [ ! -x "$ROOT/.venv/bin/python" ]; then
  python3 -m venv "$ROOT/.venv"
fi

if [ ! -x "$ROOT/.venv/bin/pio" ]; then
  echo "Installing PlatformIO in the local .venv..."
  "$ROOT/.venv/bin/python" -m pip install platformio
fi

PORT="${1:-}"
if [ -z "$PORT" ]; then
  CANDIDATES=()
  for CANDIDATE in /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART* /dev/cu.wchusbserial* /dev/cu.usbmodem*; do
    [ -e "$CANDIDATE" ] && CANDIDATES+=("$CANDIDATE")
  done

  if [ "${#CANDIDATES[@]}" -ne 1 ]; then
    echo "Could not choose one M5 serial port automatically." >&2
    echo "Reconnect the M5 or specify its port explicitly:" >&2
    echo "  bash flash_m5_bridge.sh /dev/cu.usbserial-XXXXXXXX" >&2
    exit 1
  fi
  PORT="${CANDIDATES[0]}"
fi

if [ ! -e "$PORT" ]; then
  echo "Serial port not found: $PORT" >&2
  exit 1
fi

echo "Uploading M5 bridge firmware to $PORT..."
cd "$PROJECT"
"$ROOT/.venv/bin/pio" run -t upload --upload-port "$PORT"
