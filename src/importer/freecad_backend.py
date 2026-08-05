"""FreeCAD discovery and STEP import."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

from common.math3d import mat4_identity, mat4_to_list, transform_point
from common.models import (
    AssemblyIR,
    AssemblyNode,
    BBox,
    MateHint,
    MateKind,
    PartInstance,
    Provenance,
)
from common.tolerances import Tolerances

_FC_CHECKED = False
_FC_OK = False
_FC_ERROR = ""


def _try_add_freecad_paths() -> None:
    candidates = [
        Path("/Applications/FreeCAD.app/Contents/Resources/lib"),
        Path("/Applications/FreeCAD.app/Contents/Resources/lib/python3.11/site-packages"),
        Path.home() / "Applications/FreeCAD.app/Contents/Resources/lib",
        Path("/usr/lib/freecad/lib"),
        Path("/usr/lib/freecad-python3/lib"),
        Path("/usr/local/lib/freecad/lib"),
    ]
    for p in candidates:
        if p.is_dir() and str(p) not in sys.path:
            sys.path.append(str(p))


def freecad_available() -> bool:
    global _FC_CHECKED, _FC_OK, _FC_ERROR
    if _FC_CHECKED:
        return _FC_OK
    _FC_CHECKED = True
    _try_add_freecad_paths()
    try:
        import FreeCAD  # noqa: F401

        _FC_OK = True
        _FC_ERROR = ""
    except Exception as exc:
        _FC_OK = False
        _FC_ERROR = str(exc)
    return _FC_OK


def freecad_status() -> str:
    """Human-readable FreeCAD discovery status."""
    ok = freecad_available()
    if ok:
        import FreeCAD

        ver = ".".join(str(x) for x in FreeCAD.Version()[:3])
        return f"FreeCAD {ver} OK (Python {sys.version.split()[0]})"
    detail = f" ({_FC_ERROR})" if _FC_ERROR else ""
    return (
        f"FreeCAD NOT available under Python {sys.version.split()[0]}{detail}. "
        "Use ./run_with_freecad.sh … (FreeCAD.app bundled Python 3.11)."
    )


def _placement_to_mat4(placement: Any) -> np.ndarray:
    """FreeCAD Placement → 4x4."""
    M = mat4_identity()
    try:
        base = placement.Base
        rot = placement.Rotation
        # FreeCAD Matrix is row-major compatible via toMatrix()
        mat = rot.toMatrix()
        # FreeCAD Matrix A11.. indexing 1-based
        R = np.array(
            [
                [mat.A11, mat.A12, mat.A13],
                [mat.A21, mat.A22, mat.A23],
                [mat.A31, mat.A32, mat.A33],
            ],
            dtype=np.float64,
        )
        M[:3, :3] = R
        M[0, 3] = float(base.x)
        M[1, 3] = float(base.y)
        M[2, 3] = float(base.z)
        # FreeCAD default length unit is mm → convert to metres
        M[:3, 3] *= 1e-3
    except Exception:
        pass
    return M


def _nearly_identity_placement(placement: Any, tol_mm: float = 1e-3, tol_rad: float = 1e-6) -> bool:
    try:
        base = placement.Base
        if abs(float(base.x)) > tol_mm or abs(float(base.y)) > tol_mm or abs(float(base.z)) > tol_mm:
            return False
        return abs(float(placement.Rotation.Angle)) <= tol_rad
    except Exception:
        return True


def _shape_center_mm(shape: Any) -> np.ndarray:
    box = shape.BoundBox
    return np.array(
        [
            0.5 * (box.XMin + box.XMax),
            0.5 * (box.YMin + box.YMax),
            0.5 * (box.ZMin + box.ZMax),
        ],
        dtype=np.float64,
    )


def _placed_center_mm(shape: Any, placement: Any) -> np.ndarray:
    c = _shape_center_mm(shape)
    try:
        v = placement.toMatrix().multiply(
            __import__("FreeCAD").Vector(float(c[0]), float(c[1]), float(c[2]))
        )
        return np.array([float(v.x), float(v.y), float(v.z)], dtype=np.float64)
    except Exception:
        M = _placement_to_mat4(placement)
        # M is metres; c is mm
        return transform_point(M, c * 1e-3) * 1e3


def detect_world_frame_mode(doc: Any) -> str:
    """Pick one CAD world policy for the whole STEP document.

    Returns:
      - ``shape_as_world``: BREP coords are already the assembled rest pose;
        ignore FreeCAD Placement (avoids double-transform on many STEP exports).
      - ``apply_placement``: BREP is local; multiply by Placement for world.
    """
    raw: list[np.ndarray] = []
    placed: list[np.ndarray] = []
    for obj in _iter_solid_objects(doc):
        for _name, shape, placement in _explode_compounds(obj):
            try:
                if abs(float(shape.Volume)) < 1e-3:
                    continue
                raw.append(_shape_center_mm(shape))
                placed.append(_placed_center_mm(shape, placement))
            except Exception:
                continue
    if len(raw) < 2:
        return "apply_placement"
    raw_a = np.vstack(raw)
    pl_a = np.vstack(placed)
    span_raw = float(np.linalg.norm(raw_a.max(axis=0) - raw_a.min(axis=0)))
    span_pl = float(np.linalg.norm(pl_a.max(axis=0) - pl_a.min(axis=0)))
    # If applying Placement blows up the assembly envelope, shapes are world-baked.
    if span_pl > span_raw * 1.35 + 50.0:
        return "shape_as_world"
    return "apply_placement"


def _world_placement_mat4(
    shape: Any,
    placement: Any,
    mode: Optional[str] = None,
) -> np.ndarray:
    """4x4 (metres) mapping shape-local points → CAD world under ``mode``."""
    if mode == "shape_as_world":
        return mat4_identity()
    if mode == "apply_placement":
        if placement is None or _nearly_identity_placement(placement):
            return mat4_identity()
        return _placement_to_mat4(placement)
    # Per-solid fallback when IR has no document mode yet
    if placement is None or _nearly_identity_placement(placement):
        return mat4_identity()
    try:
        local_c = _shape_center_mm(shape)
        box = shape.BoundBox
        extent = float(max(box.XLength, box.YLength, box.ZLength, 1.0))
        if float(np.linalg.norm(local_c)) > max(2.0 * extent, 50.0):
            return mat4_identity()
    except Exception:
        pass
    return _placement_to_mat4(placement)


def _shape_volume_bbox(shape: Any, placement_m: Optional[np.ndarray] = None) -> tuple[float, BBox]:
    """Return volume (m^3) and axis-aligned bbox in CAD world metres."""
    try:
        vol_mm3 = float(shape.Volume)
        box = shape.BoundBox
        if not all(
            np.isfinite(x)
            for x in (vol_mm3, box.XMin, box.XMax, box.YMin, box.YMax, box.ZMin, box.ZMax)
        ):
            return 0.0, BBox(min_xyz=[0, 0, 0], max_xyz=[0, 0, 0])
        xs = (box.XMin * 1e-3, box.XMax * 1e-3)
        ys = (box.YMin * 1e-3, box.YMax * 1e-3)
        zs = (box.ZMin * 1e-3, box.ZMax * 1e-3)
        corners = np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=np.float64)
        if placement_m is not None:
            corners = np.array([transform_point(placement_m, c) for c in corners])
        bbox = BBox(
            min_xyz=corners.min(axis=0).tolist(),
            max_xyz=corners.max(axis=0).tolist(),
        )
        return abs(vol_mm3) * 1e-9, bbox
    except Exception:
        return 0.0, BBox(min_xyz=[0, 0, 0], max_xyz=[0, 0, 0])


def _obj_placement(obj: Any) -> Any:
    """Prefer global placement when FreeCAD provides it (nested links)."""
    try:
        return obj.getGlobalPlacement()
    except Exception:
        return obj.Placement


def _is_plausible_solid(vol: float, bbox: BBox, tol: Tolerances) -> bool:
    if not np.isfinite(vol) or vol < tol.min_part_volume_m3 or vol > tol.max_part_volume_m3:
        return False
    extents = np.asarray(bbox.max_xyz, dtype=np.float64) - np.asarray(bbox.min_xyz, dtype=np.float64)
    if not np.all(np.isfinite(extents)):
        return False
    if float(np.max(extents)) > tol.max_bbox_extent_m:
        return False
    # Reject pure construction planes (near-zero thickness, huge span)
    sorted_ext = np.sort(extents)
    if sorted_ext[0] < 1e-6 and sorted_ext[2] > 1.0:
        return False
    return True


def _dedupe_parts(parts: list[PartInstance]) -> list[PartInstance]:
    """Drop duplicate solids (e.g. assembly compound copies of the same body)."""

    def fingerprint(p: PartInstance) -> tuple:
        bb = p.bbox.as_array()
        # 0.5 mm grid
        q = tuple(int(round(x * 2000.0)) for x in bb.reshape(-1))
        vq = int(round(p.volume * 1e9))
        return q + (vq,)

    def rank(p: PartInstance) -> tuple:
        name = p.name.lower()
        # Prefer explicit part names over exploded assembly copies
        assembly_copy = 1 if "_s" in name or name.startswith("robot_assembly") else 0
        return (assembly_copy, len(name), p.id)

    best: dict[tuple, PartInstance] = {}
    for p in parts:
        fp = fingerprint(p)
        prev = best.get(fp)
        if prev is None or rank(p) < rank(prev):
            best[fp] = p
    return sorted(best.values(), key=lambda p: p.id)


def _iter_solid_objects(doc: Any) -> list[Any]:
    objs = []
    for obj in doc.Objects:
        try:
            # Skip FreeCAD datum / origin / plane objects
            type_id = getattr(obj, "TypeId", "")
            if any(k in type_id for k in ("Plane", "Datum", "Origin", "Line", "Point")):
                continue
            label = (obj.Label or "").lower()
            if "plane" in label and "origin" in type_id.lower():
                continue
            if hasattr(obj, "Shape") and not obj.Shape.isNull():
                sh = obj.Shape
                # Prefer solids / compounds with volume
                if getattr(sh, "Volume", 0) and abs(float(sh.Volume)) > 1e-6:
                    if np.isfinite(float(sh.Volume)):
                        objs.append(obj)
        except Exception:
            continue
    return objs


def _explode_compounds(obj: Any) -> list[tuple[str, Any, Any]]:
    """Return list of (name, shape, placement) solids.

    Mega-compounds (dozens of solids) are skipped when exploded — they are usually
    a fused assembly copy that duplicates every fastener. Prefer named part features.
    """
    results: list[tuple[str, Any, Any]] = []
    shape = obj.Shape
    placement = _obj_placement(obj)
    solids = list(getattr(shape, "Solids", []) or [])
    if len(solids) <= 1:
        results.append((obj.Label, shape, placement))
        return results
    if len(solids) > 12:
        # Keep as one display body only if it looks like a single major casting
        # (few faces relative to solids is rare); otherwise skip noise compound.
        return []
    for i, solid in enumerate(solids):
        results.append((f"{obj.Label}_s{i}", solid, placement))
    return results


def import_step_freecad(path: Path, tolerances: Tolerances) -> AssemblyIR:
    if not freecad_available():
        raise RuntimeError("FreeCAD not available")

    import Part  # noqa: F401
    from importer.freecad_session import get_step_document

    doc = get_step_document(path)
    world_mode = detect_world_frame_mode(doc)

    parts: list[PartInstance] = []
    nodes: list[AssemblyNode] = []
    idx = 0
    for obj in _iter_solid_objects(doc):
        for name, shape, placement in _explode_compounds(obj):
            # Resolve to a single CAD world frame BEFORE any joint/pivot work.
            M = _world_placement_mat4(shape, placement, world_mode)
            vol, bbox = _shape_volume_bbox(shape, M)
            if not _is_plausible_solid(vol, bbox, tolerances):
                continue
            pid = f"part_{idx:04d}"
            idx += 1
            parts.append(
                PartInstance(
                    id=pid,
                    name=name,
                    # BBox (and later mesh/features) are world metres; keep I.
                    placement=mat4_to_list(mat4_identity()),
                    volume=vol,
                    bbox=bbox,
                    provenance=Provenance(
                        freecad_path=obj.FullName if hasattr(obj, "FullName") else obj.Name,
                        source="freecad_step_world",
                    ),
                    shape_ref=obj.Name,
                )
            )

    parts = _dedupe_parts(parts)
    # Re-assign stable ids after dedupe
    remapped: list[PartInstance] = []
    for i, p in enumerate(sorted(parts, key=lambda x: (x.name, x.id))):
        remapped.append(
            PartInstance(
                id=f"part_{i:04d}",
                name=p.name,
                placement=p.placement,
                volume=p.volume,
                bbox=p.bbox,
                material=p.material,
                provenance=p.provenance,
                mesh_vertices=p.mesh_vertices,
                mesh_faces=p.mesh_faces,
                shape_ref=p.shape_ref,
            )
        )
    parts = remapped
    nodes = [AssemblyNode(id=f"node_{p.id}", name=p.name, part_id=p.id) for p in parts]

    if not parts:
        raise RuntimeError(f"No solid parts found in STEP: {path}")

    if len(parts) == 1:
        # Policy: warn via meta — fused single solid cannot yield joints
        meta = {
            "warning": "single_solid_assembly",
            "message": "STEP contains a single solid; joint detection requires an assembly of solids.",
        }
    else:
        meta = {"n_parts_raw_before_dedupe": idx, "world_frame_mode": world_mode}

    if "world_frame_mode" not in meta:
        meta["world_frame_mode"] = world_mode

    mate_hints: list[MateHint] = []
    # Best-effort: FreeCAD Assembly / App::Link constraints if present
    for obj in doc.Objects:
        type_id = getattr(obj, "TypeId", "")
        if "Constraint" in type_id or "Assembly" in type_id:
            mate_hints.append(
                MateHint(
                    kind=MateKind.UNKNOWN,
                    part_a="",
                    part_b="",
                    confidence=0.2,
                    detail=f"constraint_object:{obj.Label}",
                )
            )

    return AssemblyIR(
        source_path=str(path.resolve()),
        parts=parts,
        assembly_nodes=nodes,
        mate_hints=mate_hints,
        unit="metre",
        meta={"backend": "freecad", **meta},
    )
