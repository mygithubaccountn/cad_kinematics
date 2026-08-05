"""Phase 0 exporter: parts + raw meshes only. No joints / pivots / kinematics."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from common.io_util import write_json
from common.models import AssemblyIR, RobotDesc
from common.tolerances import Tolerances
from exporter import run_mesh_export


def export_phase0(
    ir: AssemblyIR,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
    name: str = "robot",
    *,
    export_meshes: bool = False,
    mesh_quality: str = "preview",
) -> RobotDesc:
    """
    STEP geometry → robot.json with empty joints.
    ``export_meshes=True`` writes FreeCAD/bbox GLBs; default is kinematics-only paths.
    """
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    mapping = run_mesh_export(
        ir,
        out,
        tolerances,
        tree=None,
        tessellate=export_meshes,
        write_glb=export_meshes,
        quality=mesh_quality if mesh_quality in ("preview", "final") else "preview",
    )

    from common.timing import timed

    with timed("Index/package"):
        links = []
        for part in sorted(ir.parts, key=lambda p: p.id):
            mesh = mapping.get(part.id, f"meshes/{part.id}.glb")
            links.append(
                {
                    "id": part.id,
                    "name": part.name,
                    "mesh": mesh,
                    "part_ids": [part.id],
                }
            )

        base = links[0]["id"] if links else ""
        # Prefer lowest-Z / largest volume part as visual root (not a kinematic base yet)
        if ir.parts:
            base_part = max(
                ir.parts,
                key=lambda p: (p.volume if p.volume == p.volume else 0.0, -p.bbox.min_xyz[2], p.id),
            )
            base = base_part.id

        desc = RobotDesc(
            name=name,
            frame="cad_z_up",
            base_link=base,
            links=links,
            joints=[],  # Phase 0: no joints
            rest_pose={"joints": {}},
            meta={
                "phase": 0,
                "source": ir.source_path,
                "note": "Phase 0 raw geometry export — no joint/pivot/hierarchy",
            },
        )
        write_json(out / "robot.json", desc.to_dict())
        # Placeholders only if analyze has not written real artifacts yet
        geom_path = out / "geometry.json"
        if not geom_path.is_file() or read_json_safe_empty(geom_path):
            if not (out / "features.json").is_file():
                write_json(
                    geom_path,
                    {
                        "phase": 0,
                        "cylinders": [],
                        "planes": [],
                        "clusters": [],
                        "contacts": [],
                        "adjacency": [],
                        "meta": {"status": "not_analyzed", "message": "Run analyze at phase >= 1"},
                    },
                )
        trace_path = out / "decision_trace.json"
        if not trace_path.is_file():
            write_json(
                trace_path,
                {
                    "phase": 0,
                    "traces": [],
                    "meta": {"status": "empty", "message": "Decision traces start at phase 2"},
                },
            )
    return desc


def read_json_safe_empty(path: Path) -> bool:
    """True if geometry file is still the phase-0 placeholder."""
    from common.io_util import read_json

    try:
        data = read_json(path)
        return data.get("meta", {}).get("status") == "not_analyzed"
    except Exception:
        return True
