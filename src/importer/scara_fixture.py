"""SCARA-like fixture: two revolutes + one prismatic (Faz 5)."""

from __future__ import annotations

from common.math3d import mat4_identity, mat4_to_list
from common.models import (
    AssemblyIR,
    AssemblyNode,
    BBox,
    CylFeature,
    CylKind,
    FeatureGraph,
    PartInstance,
    Provenance,
)


def _box(xmin, ymin, zmin, xmax, ymax, zmax):
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


def build_scara_fixture() -> tuple[AssemblyIR, FeatureGraph]:
    specs = [
        ("base", "base", [-0.1, -0.1, 0.0], [0.1, 0.1, 0.08], 0.003),
        ("arm1", "arm1", [-0.03, -0.03, 0.08], [0.03, 0.03, 0.12], 0.0005),
        ("arm2", "arm2", [0.05, -0.025, 0.09], [0.25, 0.025, 0.13], 0.0004),
        ("zslide", "z_slide", [0.22, -0.02, 0.05], [0.28, 0.02, 0.20], 0.0003),
    ]
    parts = []
    for pid, name, lo, hi, vol in specs:
        vmin, vmax = list(map(float, lo)), list(map(float, hi))
        v, f = _box(*vmin, *vmax)
        parts.append(
            PartInstance(
                id=pid,
                name=name,
                placement=mat4_to_list(mat4_identity()),
                volume=vol,
                bbox=BBox(min_xyz=vmin, max_xyz=vmax),
                provenance=Provenance(source="synthetic"),
                mesh_vertices=v,
                mesh_faces=f,
                shape_ref=pid,
            )
        )
    ir = AssemblyIR(
        source_path="synthetic://scara",
        parts=parts,
        assembly_nodes=[AssemblyNode(id=f"n_{p.id}", name=p.name, part_id=p.id) for p in parts],
        unit="metre",
        meta={"fixture": "scara"},
    )
    cyls = [
        # J1 revolute Z
        CylFeature("c0", "base", [0, 0, 0.04], [0, 0, 1], 0.02, 0.06, CylKind.INNER),
        CylFeature("c1", "arm1", [0, 0, 0.04], [0, 0, 1], 0.019, 0.06, CylKind.OUTER),
        # J2 revolute Z at arm elbow
        CylFeature("c2", "arm1", [0.05, 0, 0.10], [0, 0, 1], 0.015, 0.04, CylKind.INNER),
        CylFeature("c3", "arm2", [0.05, 0, 0.10], [0, 0, 1], 0.014, 0.04, CylKind.OUTER),
        # Prismatic Z guides between arm2 and zslide (similar radius outers)
        CylFeature("c4", "arm2", [0.25, 0, 0.10], [0, 0, 1], 0.008, 0.12, CylKind.OUTER),
        CylFeature("c5", "zslide", [0.25, 0, 0.10], [0, 0, 1], 0.008, 0.12, CylKind.OUTER),
    ]
    return ir, FeatureGraph(cylinders=cyls, meta={"fixture": "scara", "prebuilt": True})
