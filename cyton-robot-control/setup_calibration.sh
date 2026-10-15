#!/usr/bin/env bash
# Create the local Python environment needed by the arm + VRing calibration GUI.
set -euo pipefail

cd "$(dirname "$0")"

if ! command -v python3 >/dev/null 2>&1; then
  echo "Python 3 is required. Install Python 3, then run this script again." >&2
  exit 1
fi

python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-calibration.txt

if ! .venv/bin/python -c "import tkinter" >/dev/null 2>&1; then
  echo "Python was installed without Tkinter, which the calibration GUI needs." >&2
  echo "Install a Python distribution with Tk support, then run this script again." >&2
  exit 1
fi

echo
echo "Setup complete. Start the calibration GUI with:"
echo "  bash outputs/vring_cyton_control/launch_vring_gui.sh"
