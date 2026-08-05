#!/usr/bin/env bash
# Opens the robot in your browser (no Godot needed).
# Usage:
#   ./open_viewer.sh              # uses viewer/ already synced, or out/step
#   ./open_viewer.sh out/robot    # sync from that run folder first
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
PORT="${PORT:-8765}"
SRC="${1:-}"

if [[ -n "$SRC" ]]; then
  if [[ ! -f "$SRC/robot.json" ]]; then
    echo "No robot.json in: $SRC" >&2
    exit 1
  fi
  mkdir -p "$ROOT/viewer/meshes"
  cp "$SRC/robot.json" "$ROOT/viewer/robot.json"
  rm -f "$ROOT/viewer/meshes/"*.glb 2>/dev/null || true
  cp -f "$SRC/meshes/"*.glb "$ROOT/viewer/meshes/" 2>/dev/null || true
  [[ -f "$SRC/debug_overlay.json" ]] && cp -f "$SRC/debug_overlay.json" "$ROOT/viewer/debug_overlay.json"
  echo "Synced from $SRC"
elif [[ -f "$ROOT/out/step/robot.json" ]] && [[ ! -f "$ROOT/viewer/robot.json" ]]; then
  cp "$ROOT/out/step/robot.json" "$ROOT/viewer/robot.json"
  mkdir -p "$ROOT/viewer/meshes"
  cp -f "$ROOT/out/step/meshes/"*.glb "$ROOT/viewer/meshes/" 2>/dev/null || true
fi

cd "$ROOT/viewer"
pkill -f "http.server ${PORT}" 2>/dev/null || true
sleep 0.2
echo "Viewer: http://127.0.0.1:${PORT}/"
echo "Durdurmak için Ctrl+C"
(sleep 0.5; open "http://127.0.0.1:${PORT}/" 2>/dev/null || true) &
exec python3 -m http.server "$PORT"
