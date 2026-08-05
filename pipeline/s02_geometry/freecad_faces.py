"""OCC/FreeCAD cylindrical face extraction."""

from __future__ import annotations

from typing import Any

import numpy as np

from pipeline.common.math3d import as_vec3, normalize
from pipeline.common.models import AssemblyIR, CylFeature, CylKind
from pipeline.common.tolerances import Tolerances
from pipeline.s01_import.freecad_backend import freecad_available


def extract_cylinders_freecad(ir: AssemblyIR, tol: Tolerances) -> list[CylFeature]:
    if not freecad_available():
        return []
    try:
        import FreeCAD
        import Part
    except Exception:
        return []

    # Re-open source if possible
    path = ir.source_path
    if not path or path.startswith("synthetic://"):
        return []

    import Import

    doc = FreeCAD.newDocument("cad_robot_geom")
    cylinders: list[CylFeature] = []
    try:
        Import.insert(str(path), doc.Name)
        doc.recompute()
        part_by_ref = {p.shape_ref: p for p in ir.parts if p.shape_ref}
        part_by_name = {p.name: p for p in ir.parts}
        cid = 0
        for obj in doc.Objects:
            if not hasattr(obj, "Shape") or obj.Shape.isNull():
                continue
            part = part_by_ref.get(obj.Name) or part_by_name.get(obj.Label)
            if part is None:
                continue
            for fi, face in enumerate(obj.Shape.Faces):
                surf = face.Surface
                if not hasattr(surf, "Radius"):
                    # Cylinder surfaces in FreeCAD Part have .Axis / .Radius / .Center
                    continue
                type_id = type(surf).__name__
                if "Cylinder" not in type_id and not hasattr(surf, "Axis"):
                    continue
                try:
                    radius_m = float(surf.Radius) * 1e-3
                    axis = surf.Axis
                    center = surf.Center
                    direction = normalize(
                        as_vec3([float(axis.x), float(axis.y), float(axis.z)])
                    )
                    point = as_vec3([float(center.x), float(center.y), float(center.z)]) * 1e-3
                    # Approximate height from face UV bounds if available
                    try:
                        u0, u1, v0, v1 = face.ParameterRange
                        height = abs(float(v1 - v0)) * 1e-3
                    except Exception:
                        height = 0.01
                    kind = _classify_inner_outer(face, surf)
                    cylinders.append(
                        CylFeature(
                            id=f"cyl_{cid:04d}",
                            part_id=part.id,
                            axis_point=point.tolist(),
                            axis_dir=direction.tolist(),
                            radius=radius_m,
                            height=max(height, 1e-4),
                            kind=kind,
                            face_ids=[f"{obj.Name}:F{fi}"],
                        )
                    )
                    cid += 1
                except Exception:
                    continue
    finally:
        try:
            FreeCAD.closeDocument(doc.Name)
        except Exception:
            pass
    cylinders.sort(key=lambda c: c.id)
    return cylinders


def _classify_inner_outer(face: Any, surf: Any) -> CylKind:
    """
    Heuristic: sample face normal vs vector from axis to point.
    If normals point toward axis → outer (shaft); away → inner (hole).
    """
    try:
        import FreeCAD

        u0, u1, v0, v1 = face.ParameterRange
        u = 0.5 * (u0 + u1)
        v = 0.5 * (v0 + v1)
        p = face.valueAt(u, v)
        n = face.normalAt(u, v)
        axis = surf.Axis
        center = surf.Center
        # Vector from axis to point (reject along axis)
        ax = FreeCAD.Vector(axis.x, axis.y, axis.z).normalize()
        c = FreeCAD.Vector(center.x, center.y, center.z)
        pv = FreeCAD.Vector(p.x, p.y, p.z)
        w = pv - c
        w = w - ax.multiply(w.dot(ax))
        if w.Length < 1e-9:
            return CylKind.UNKNOWN
        radial = w.normalize()
        # Face normal in FreeCAD; Orientation may flip
        nd = FreeCAD.Vector(n.x, n.y, n.z).normalize()
        dot = nd.dot(radial)
        # For solid material outside hole: hole wall normal points inward (toward axis) → -radial
        if dot < -0.2:
            return CylKind.INNER
        if dot > 0.2:
            return CylKind.OUTER
    except Exception:
        pass
    return CylKind.UNKNOWN
