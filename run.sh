#!/usr/bin/env bash
# Run pipeline with system/conda Python (synthetic fixtures + tests).
# Clears FreeCAD PYTHONPATH pollution that breaks numpy on Python 3.14.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
unset PYTHONPATH
export PYTHONPATH="$ROOT/src:$ROOT"

cd "$ROOT"
exec python3 pipeline.py "$@"
