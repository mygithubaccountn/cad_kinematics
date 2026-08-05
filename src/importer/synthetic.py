"""Synthetic assembly fixtures for deterministic tests without FreeCAD."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from common.io_util import read_json
from common.math3d import mat4_identity, mat4_to_list
from common.models import (
    AssemblyIR,
    AssemblyNode,
    BBox,
    CylFeature,
    CylKind,
    FeatureGraph,
    MateHint,
    MateKind,
    PartInstance,
    Provenance,
)


def is_synthetic_path(path: Path) -> bool:
    name = path.name.lower()
    return (
        name.endswith(".synthetic.json")
        or name.endswith(".ir.json")
        or "serial_3dof" in name
        or "scara" in name
    )


def _box_mesh(xmin, ymin, zmin, xmax, ymax, zmax) -> tuple[list[list[float]], list[list[int]]]:
    v = [
        [xmin, ymin, zmin],
        [xmax, ymin, zmin],
        [xmax, ymax, zmin],
        [xmin, ymax, zmin],
        [xmin, ymin, zmax],
        [xmax, ymin, zmax],
        [xmax, ymax, zmax],
        [xmin, ymax, zmax],
    ]
    f = [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [1, 5, 6],
        [1, 6, 2],
        [2, 6, 7],
        [2, 7, 3],
        [3, 7, 4],
        [3, 4, 0],
    ]
    return v, f


def build_serial_3dof_fixture() -> tuple[AssemblyIR, FeatureGraph]:
    """
    Minimal serial robot: base + link1 + link2 + link3.
    Revolute joints about Z at (0,0,0.1), Y at (0.25,0,0.15), Y at (0.45,0,0.15).
    Includes shaft-in-hole cylinder features at each joint.
    """
    parts: list[PartInstance] = []
    specs = [
        # id, name, bbox, volume approx
        ("base", "base_link", [-0.08, -0.08, 0.0], [0.08, 0.08, 0.10], 0.002),
        ("link1", "link_1", [-0.04, -0.04, 0.10], [0.04, 0.04, 0.20], 0.0008),
        ("link2", "link_2", [0.04, -0.03, 0.12], [0.30, 0.03, 0.18], 0.0006),
        ("link3", "link_3", [0.30, -0.025, 0.12], [0.50, 0.025, 0.18], 0.0004),
    ]
    for pid, name, lo, hi, vol in specs:
        vmin = list(map(float, lo))
        vmax = list(map(float, hi))
        verts, faces = _box_mesh(*vmin, *vmax)
        parts.append(
            PartInstance(
                id=pid,
                name=name,
                placement=mat4_to_list(mat4_identity()),
                volume=vol,
                bbox=BBox(min_xyz=vmin, max_xyz=vmax),
                provenance=Provenance(source="synthetic"),
                mesh_vertices=verts,
                mesh_faces=faces,
                shape_ref=pid,
            )
        )

    nodes = [AssemblyNode(id=f"n_{p.id}", name=p.name, part_id=p.id) for p in parts]
    ir = AssemblyIR(
        source_path="synthetic://serial_3dof",
        parts=parts,
        assembly_nodes=nodes,
        mate_hints=[
            MateHint(MateKind.CONCENTRIC, "base", "link1", 0.4, "fixture"),
            MateHint(MateKind.CONCENTRIC, "link1", "link2", 0.4, "fixture"),
            MateHint(MateKind.CONCENTRIC, "link2", "link3", 0.4, "fixture"),
        ],
        unit="metre",
        meta={"fixture": "serial_3dof"},
    )

    # Cylinder features: shaft (outer) + hole (inner) pairs
    cyls: list[CylFeature] = []
    # J1 about Z at origin height 0.1
    cyls.append(
        CylFeature("c_base_hole", "base", [0, 0, 0.05], [0, 0, 1], 0.025, 0.08, CylKind.INNER, ["f0"])
    )
    cyls.append(
        CylFeature("c_l1_shaft", "link1", [0, 0, 0.05], [0, 0, 1], 0.024, 0.08, CylKind.OUTER, ["f1"])
    )
    # J2 about Y at (0.0, 0, 0.15) — shoulder between link1 and link2
    cyls.append(
        CylFeature("c_l1_hole", "link1", [0.0, 0, 0.15], [0, 1, 0], 0.020, 0.06, CylKind.INNER, ["f2"])
    )
    cyls.append(
        CylFeature("c_l2_shaft", "link2", [0.0, 0, 0.15], [0, 1, 0], 0.019, 0.06, CylKind.OUTER, ["f3"])
    )
    # J3 about Y at (0.30, 0, 0.15)
    cyls.append(
        CylFeature("c_l2_hole", "link2", [0.30, 0, 0.15], [0, 1, 0], 0.015, 0.05, CylKind.INNER, ["f4"])
    )
    cyls.append(
        CylFeature("c_l3_shaft", "link3", [0.30, 0, 0.15], [0, 1, 0], 0.014, 0.05, CylKind.OUTER, ["f5"])
    )

    # Decorative false cylinder (should score low — no shaft-hole partner / low contact)
    cyls.append(
        CylFeature("c_l2_deco", "link2", [0.15, 0, 0.20], [0, 0, 1], 0.005, 0.01, CylKind.OUTER, ["f6"])
    )

    fg = FeatureGraph(cylinders=cyls, meta={"fixture": "serial_3dof", "prebuilt": True})
    return ir, fg


def load_synthetic_assembly(path: Path) -> AssemblyIR:
    name = path.name.lower()
    if "serial_3dof" in name:
        ir, _ = build_serial_3dof_fixture()
        ir.source_path = str(path)
        return ir
    if "scara" in name:
        from importer.scara_fixture import build_scara_fixture

        ir, _ = build_scara_fixture()
        ir.source_path = str(path)
        return ir
    data = read_json(path)
    if "parts" in data:
        return AssemblyIR.from_dict(data)
    raise ValueError(f"Unrecognized synthetic fixture: {path}")


def load_prebuilt_features(path: Path) -> FeatureGraph | None:
    """Return prebuilt FeatureGraph for known fixtures, else None."""
    name = path.name.lower()
    src = str(path)
    if "serial_3dof" in name or "serial_3dof" in src:
        _, fg = build_serial_3dof_fixture()
        return fg
    if "scara" in name or "scara" in src:
        from importer.scara_fixture import build_scara_fixture

        _, fg = build_scara_fixture()
        return fg
    feat = path.with_suffix("").with_suffix(".features.json")
    if feat.is_file():
        return FeatureGraph.from_dict(read_json(feat))
    return None
