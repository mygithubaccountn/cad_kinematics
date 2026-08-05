#!/usr/bin/env bash
# Run pipeline.py with FreeCAD's bundled Python (required for real STEP).
# Does NOT inherit a polluted shell PYTHONPATH (that breaks system Python later).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
FC_APP="${FREECAD_APP:-/Applications/FreeCAD.app}"
FC_RES="$FC_APP/Contents/Resources"
FC_PY="$FC_RES/bin/python"

if [[ ! -x "$FC_PY" ]]; then
  echo "FreeCAD python not found at: $FC_PY" >&2
  echo "Set FREECAD_APP to your FreeCAD.app path." >&2
  exit 1
fi

if [[ $# -lt 1 ]]; then
  echo "Usage: ./run_with_freecad.sh run <STEP> --out out/step" >&2
  echo "Example: ./run_with_freecad.sh run \"$HOME/Desktop/robot_assembly.stp\" --out out/step" >&2
  echo "Iterate:  ./run_with_freecad.sh run \"$HOME/Desktop/robot_assembly.stp\" --out out/step --from-stage joints" >&2
  echo "Meshes:   cached by topology (joint edits reuse GLB); --remesh | --final-meshes | --no-meshes" >&2
  exit 1
fi

# Reject obvious placeholder paths
for arg in "$@"; do
  if [[ "$arg" == *"/path/to/"* ]]; then
    echo "Error: replace /path/to/... with a real STEP file path on your machine." >&2
    echo "Example: $HOME/Desktop/robot_assembly.stp" >&2
    exit 1
  fi
done

# Clean env: only project + FreeCAD (ignore shell PYTHONPATH)
export PYTHONPATH="$ROOT/src:$ROOT:$FC_RES/lib:$FC_RES/lib/python3.11/site-packages"
cd "$ROOT"
exec "$FC_PY" pipeline.py "$@"
