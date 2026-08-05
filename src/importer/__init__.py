"""01_import — STEP / assembly → AssemblyIR."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from common.io_util import write_json
from common.models import AssemblyIR
from common.tolerances import Tolerances
from importer.freecad_backend import import_step_freecad, freecad_available
from importer.synthetic import load_synthetic_assembly, is_synthetic_path


def run_import(
    source: str | Path,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
) -> AssemblyIR:
    """Import STEP (FreeCAD) or synthetic fixture into AssemblyIR."""
    tolerances = tolerances or Tolerances()
    source_p = Path(source)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    if is_synthetic_path(source_p):
        ir = load_synthetic_assembly(source_p)
    elif freecad_available():
        ir = import_step_freecad(source_p, tolerances)
    else:
        from importer.freecad_backend import freecad_status

        raise RuntimeError(
            f"{freecad_status()} "
            "For real STEP files run: ./run_with_freecad.sh run robot.step --out out/ "
            "Or use tests/fixtures/*.synthetic.json with system Python."
        )

    write_json(out / "assembly_ir.json", ir.to_dict())
    return ir
