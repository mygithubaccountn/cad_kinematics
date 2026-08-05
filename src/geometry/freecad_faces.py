"""OCC/FreeCAD cylindrical face extraction."""

from __future__ import annotations

from typing import Any

import numpy as np

from common.math3d import as_vec3, normalize, transform_dir, transform_point
from common.models import AssemblyIR, CylFeature, CylKind
from common.tolerances import Tolerances
from importer.freecad_backend import _obj_placement, _world_placement_mat4, freecad_available


def extract_cylinders_freecad(ir: AssemblyIR, tol: Tolerances) -> list[CylFeature]:
    if not freecad_available():
        return []
    try:
        import FreeCAD  # noqa: F401
        import Part  # noqa: F401
    except Exception:
        return []

    path = ir.source_path
    if not path or path.startswith("synthetic://"):
        return []

    from importer.freecad_session import get_step_document

    doc = get_step_document(path)
    part_by_ref = {p.shape_ref: p for p in ir.parts if p.shape_ref}
    part_by_name = {p.name: p for p in ir.parts}
    by_part: dict[str, list[CylFeature]] = {}
    cid = 0

    for obj in doc.Objects:
        if not hasattr(obj, "Shape") or obj.Shape.isNull():
            continue
        part = part_by_ref.get(obj.Name) or part_by_name.get(obj.Label)
        if part is None:
            continue
        # Face geometry is shape-local; map into the same world frame as IR bboxes.
        M = _world_placement_mat4(
            obj.Shape, _obj_placement(obj), ir.meta.get("world_frame_mode")
        )
        faces = list(obj.Shape.Faces)
        # Hard cap face walk on pathological solids
        if len(faces) > 2500:
            faces = faces[:: max(1, len(faces) // 2000)]
        for fi, face in enumerate(faces):
            surf = face.Surface
            if not hasattr(surf, "Radius"):
                continue
            type_id = type(surf).__name__
            if "Cylinder" not in type_id and not hasattr(surf, "Axis"):
                continue
            try:
                radius_m = float(surf.Radius) * 1e-3
                if radius_m < tol.min_cylinder_radius_m:
                    continue
                axis = surf.Axis
                center = surf.Center
                direction_l = normalize(
                    as_vec3([float(axis.x), float(axis.y), float(axis.z)])
                )
                point_l = as_vec3([float(center.x), float(center.y), float(center.z)]) * 1e-3
                direction = normalize(transform_dir(M, direction_l))
                point = transform_point(M, point_l)
                try:
                    u0, u1, v0, v1 = face.ParameterRange
                    height = abs(float(v1 - v0)) * 1e-3
                except Exception:
                    height = 0.01
                if height < tol.min_cylinder_height_m:
                    continue
                kind = _classify_inner_outer(face, surf)
                feat = CylFeature(
                    id=f"cyl_{cid:04d}",
                    part_id=part.id,
                    axis_point=point.tolist(),
                    axis_dir=direction.tolist(),
                    radius=radius_m,
                    height=max(height, 1e-4),
                    kind=kind,
                    face_ids=[f"{obj.Name}:F{fi}"],
                )
                by_part.setdefault(part.id, []).append(feat)
                cid += 1
            except Exception:
                continue

    # Keep the most joint-relevant cylinders per part (large radius × height)
    cylinders: list[CylFeature] = []
    for pid, feats in by_part.items():
        feats.sort(key=lambda c: -(c.radius * c.height))
        keep = feats[: tol.max_cylinders_per_part]
        cylinders.extend(keep)

    # Re-id stably
    cylinders.sort(key=lambda c: (c.part_id, -c.radius * c.height, c.id))
    for i, c in enumerate(cylinders):
        c.id = f"cyl_{i:04d}"
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
        ax = FreeCAD.Vector(axis.x, axis.y, axis.z).normalize()
        c = FreeCAD.Vector(center.x, center.y, center.z)
        pv = FreeCAD.Vector(p.x, p.y, p.z)
        w = pv - c
        w = w - ax.multiply(w.dot(ax))
        if w.Length < 1e-9:
            return CylKind.UNKNOWN
        radial = w.normalize()
        nd = FreeCAD.Vector(n.x, n.y, n.z).normalize()
        dot = nd.dot(radial)
        if dot < -0.2:
            return CylKind.INNER
        if dot > 0.2:
            return CylKind.OUTER
    except Exception:
        pass
    return CylKind.UNKNOWN
