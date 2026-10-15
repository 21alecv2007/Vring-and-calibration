#!/usr/bin/env bash
# macOS / Linux launcher for the Unified Cyton + VRing Controller.
# Automatically uses the repository virtualenv (.venv).
cd "$(dirname "$0")"
ROOT="$(cd ../.. && pwd)"
if [ -x "$ROOT/.venv/bin/python" ]; then
  exec "$ROOT/.venv/bin/python" vring_cyton_gui.py "$@"
fi
exec python3 vring_cyton_gui.py "$@"
