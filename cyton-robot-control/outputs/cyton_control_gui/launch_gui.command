#!/usr/bin/env bash
# macOS / Linux launcher. Uses the repo-level .venv when present.
cd "$(dirname "$0")"
ROOT="$(cd ../.. && pwd)"
if [ -x "$ROOT/.venv/bin/python" ]; then
  exec "$ROOT/.venv/bin/python" cyton_control_gui.py "$@"
fi
exec python3 cyton_control_gui.py "$@"
