#!/usr/bin/env bash
# Mini test UI: pick STEP → run pipeline → see 3D viewer
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
PORT="${UI_PORT:-8787}"

# Avoid FreeCAD PYTHONPATH breaking system python used by the UI
unset PYTHONPATH || true

echo "Starting CAD Robot Test UI…"
echo "  UI:     http://127.0.0.1:${PORT}/"
echo "  Viewer: http://127.0.0.1:8765/"
(sleep 0.8; open "http://127.0.0.1:${PORT}/" 2>/dev/null || true) &
exec python3 "$ROOT/ui_app.py"
