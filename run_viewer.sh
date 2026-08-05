#!/usr/bin/env bash
# Run CAD→robot pipeline on any STEP and open the browser viewer.
#
# Usage:
#   ./run_viewer.sh "/path/to/robot.step"
#   ./run_viewer.sh "/path/to/robot.step" my_name
#   ./run_viewer.sh "/path/to/robot.step" my_name out/my_name
#
# Then refresh http://127.0.0.1:8765/ if the server was already running.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

if [[ $# -lt 1 ]]; then
  echo "Usage: ./run_viewer.sh /absolute/or/relative/path/to/file.step [name] [out_dir]" >&2
  echo "" >&2
  echo "Examples:" >&2
  echo "  ./run_viewer.sh \"\$HOME/Desktop/Robot.STEP\"" >&2
  echo "  ./run_viewer.sh \"\$HOME/Desktop/6_AXIS robot arm.step\" six_axis" >&2
  exit 1
fi

STEP="$1"
if [[ ! -f "$STEP" ]]; then
  # try resolving relative to Desktop / cwd
  if [[ -f "$ROOT/$STEP" ]]; then
    STEP="$ROOT/$STEP"
  elif [[ -f "$HOME/Desktop/$STEP" ]]; then
    STEP="$HOME/Desktop/$STEP"
  else
    echo "STEP not found: $1" >&2
    exit 1
  fi
fi
STEP="$(cd "$(dirname "$STEP")" && pwd)/$(basename "$STEP")"

NAME="${2:-}"
if [[ -z "$NAME" ]]; then
  NAME="$(basename "$STEP")"
  NAME="${NAME%.*}"
  # filesystem-safe
  NAME="$(echo "$NAME" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9_-]+/_/g; s/^_|_$//g')"
  NAME="${NAME:-robot}"
fi

OUT="${3:-out/$NAME}"
PORT="${PORT:-8765}"

echo "=== STEP: $STEP"
echo "=== name: $NAME"
echo "=== out:  $OUT"
echo "=== (küçük montaj ~20-60 sn, orta ~2-5 dk; Kuka-ölçeği uzun sürer)"
echo ""

"$ROOT/run_with_freecad.sh" run "$STEP" --out "$OUT" --name "$NAME"

if [[ ! -f "$OUT/robot.json" ]]; then
  echo "Pipeline finished but $OUT/robot.json missing." >&2
  exit 1
fi

mkdir -p "$ROOT/viewer/meshes"
cp "$OUT/robot.json" "$ROOT/viewer/robot.json"
rm -f "$ROOT/viewer/meshes/"*.glb 2>/dev/null || true
cp -f "$OUT/meshes/"*.glb "$ROOT/viewer/meshes/" 2>/dev/null || true

echo ""
echo "=== Sonuç: $OUT"
if command -v python3 >/dev/null; then
  python3 - <<PY
import json
from pathlib import Path
r=json.load(open("$OUT/robot.json"))
v=json.load(open("$OUT/validation_report.json"))
print(f"  links={len(r.get('links',[]))} joints={len(r.get('joints',[]))} frame={r.get('frame')}")
print(f"  validation ok={v.get('ok')}")
for j in r.get("joints", []):
    print(f"    {j.get('type','?'):8} {j.get('parent')} -> {j.get('child')}  conf={float(j.get('confidence',0)):.2f}")
PY
fi

# Restart viewer server on PORT
pkill -f "http.server ${PORT}" 2>/dev/null || true
sleep 0.3
(cd "$ROOT/viewer" && python3 -m http.server "$PORT" >/tmp/cad_viewer_${PORT}.log 2>&1) &
sleep 0.8
URL="http://127.0.0.1:${PORT}/"
echo ""
echo "Viewer: $URL"
open "$URL" 2>/dev/null || true
echo "Bitti. Rest pose’u kontrol et; sonra ‘Joint’leri hareket ettir’."
