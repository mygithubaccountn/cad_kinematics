"""Legacy entry: prefer `python pipeline.py` or ./run.sh / ./run_with_freecad.sh."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    print(
        "Note: use `python pipeline.py ...` or `./run.sh` / `./run_with_freecad.sh` "
        "(phase-gated CLI). Forwarding…",
        file=sys.stderr,
    )
    sys.path.insert(0, str(root / "src"))
    sys.path.insert(0, str(root))
    # Re-exec root pipeline.py with same argv after module name
    pipeline = root / "pipeline.py"
    sys.argv = [str(pipeline), *sys.argv[1:]]
    runpy.run_path(str(pipeline), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
