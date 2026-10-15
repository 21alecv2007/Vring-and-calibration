#!/usr/bin/env bash
# macOS / Linux launcher for the IK controller. Uses the repo-level .venv when present.
cd "$(dirname "$0")"
ROOT="$(cd ../.. && pwd)"
if [ -x "$ROOT/.venv/bin/python" ]; then
  exec "$ROOT/.venv/bin/python" cyton_ik_gui.py "$@"
fi
exec python3 cyton_ik_gui.py "$@"
