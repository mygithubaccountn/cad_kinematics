#!/usr/bin/env bash
# Run Godot project against latest out/step (or given dir).
# Usage:
#   ./run_godot_test.sh
#   ./run_godot_test.sh out/step
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
SRC="${1:-$ROOT/out/step}"

if [[ ! -f "$SRC/robot.json" ]]; then
  echo "No robot.json in $SRC — run pipeline first" >&2
  exit 1
fi

mkdir -p "$ROOT/godot_test/robot_data/meshes"
cp -f "$SRC/robot.json" "$ROOT/godot_test/robot_data/robot.json"
[[ -f "$SRC/debug_overlay.json" ]] && cp -f "$SRC/debug_overlay.json" "$ROOT/godot_test/robot_data/debug_overlay.json"
cp -f "$SRC/meshes/"*.glb "$ROOT/godot_test/robot_data/meshes/" 2>/dev/null || true

# Prefer Python Godot-contract runtime (always available)
export PYTHONPATH="$ROOT/src:$ROOT"
python3 - <<PY
from pathlib import Path
from validation.godot_runtime import run_godot_runtime_test
src = Path("$SRC")
r = run_godot_runtime_test(src / "robot.json", src)
print("godot_runtime_report:", "ok" if r.get("ok") else "FAIL", r.get("n_movable"), "joints")
for j in r.get("joints", []):
    print(f"  {j['id']} {j['type']} ok={j['ok']} pivot_drift={j['pivot_drift_m']:.2e} sample_Δ={j['sample_delta_m']:.4f}")
if not r.get("ok"):
    raise SystemExit(2)
PY

# Optional: launch Godot editor/player if installed
GODOT=""
for c in \
  "${GODOT_BIN:-}" \
  "$(command -v godot4 2>/dev/null || true)" \
  "$(command -v godot 2>/dev/null || true)" \
  "/Applications/Godot.app/Contents/MacOS/Godot" \
  "/Applications/Godot_4.app/Contents/MacOS/Godot" \
  "$HOME/Desktop/Godot_mono.app/Contents/MacOS/Godot" \
  "$HOME/Downloads/Godot_mono.app/Contents/MacOS/Godot"
do
  if [[ -n "$c" && -x "$c" ]]; then
    GODOT="$c"
    break
  fi
done

if [[ -z "$GODOT" ]]; then
  echo "Godot binary not found — Python runtime test passed; open godot_test/ in Godot Editor to view."
  echo "Set GODOT_BIN=/path/to/Godot to let this script find it."
  exit 0
fi

# New/changed .glb files aren't visible to Godot until the asset database
# imports them — without this pass, parts render invisible on first launch.
echo "Importing new assets..."
"$GODOT" --headless --editor --path "$ROOT/godot_test" --import --quit >/dev/null 2>&1 || true

echo "Launching Godot: $GODOT"
exec "$GODOT" --path "$ROOT/godot_test"
