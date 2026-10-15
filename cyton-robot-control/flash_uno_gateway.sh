#!/usr/bin/env bash
# Build and upload the Arduino Uno + DYNAMIXEL Shield gateway firmware.
# Usage: bash flash_uno_gateway.sh [serial-port]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$ROOT/outputs/cyton_gateway_uno"

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
  for CANDIDATE in /dev/cu.usbmodem* /dev/cu.wchusbserial* /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART*; do
    [ -e "$CANDIDATE" ] && CANDIDATES+=("$CANDIDATE")
  done

  if [ "${#CANDIDATES[@]}" -ne 1 ]; then
    echo "Could not choose one Uno serial port automatically." >&2
    echo "Reconnect the Uno or specify its port explicitly:" >&2
    echo "  bash flash_uno_gateway.sh /dev/cu.usbmodemXXXXXXXX" >&2
    exit 1
  fi
  PORT="${CANDIDATES[0]}"
fi

if [ ! -e "$PORT" ]; then
  echo "Serial port not found: $PORT" >&2
  exit 1
fi

echo "Before continuing, set the DYNAMIXEL Shield UART switch to UPLOAD."
read -r -p "Press Return when the switch is on UPLOAD, or Ctrl-C to cancel: "

echo "Uploading Uno gateway firmware to $PORT..."
cd "$PROJECT"
"$ROOT/.venv/bin/pio" run -t upload --upload-port "$PORT"

echo
echo "Upload complete. Return the shield UART switch to DYNAMIXEL before using the arm."
