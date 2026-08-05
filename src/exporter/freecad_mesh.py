"""Tessellate FreeCAD/STEP solids into triangle meshes (metres, world frame)."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Literal, Optional

import numpy as np

from common.models import AssemblyIR, PartInstance
from common.tolerances import Tolerances
from importer.freecad_backend import _obj_placement, _world_placement_mat4, freecad_available

MeshQuality = Literal["preview", "final"]
MeshArrays = tuple[np.ndarray, np.ndarray]  # V float64 Nx3 metres world, F int32 Mx3


def enrich_ir_with_freecad_meshes(
    ir: AssemblyIR,
    tolerances: Optional[Tolerances] = None,
    only_part_ids: Optional[set[str]] = None,
    *,
    quality: MeshQuality = "preview",
    world_align: Optional[np.ndarray] = None,
) -> AssemblyIR:
    """
    Tessellate using the shared FreeCAD STEP document (no second Import.insert).

    Preview: coarse bbox-relative ``Shape.tessellate``, triangle caps, bbox for tiny
    solids. Vertices stay as numpy on ``part._mesh_np`` (no huge ``.tolist()``).
    Optional ``world_align`` is baked into the placement matrix.
    """
    tolerances = tolerances or Tolerances()
    if not freecad_available():
        return ir
    source = ir.source_path
    if not source or str(source).startswith("synthetic://"):
        return ir
    path = Path(source)
    if not path.is_file():
        return ir

    targets = [p for p in ir.parts if only_part_ids is None or p.id in only_part_ids]
    if not targets:
        return ir

    from importer.freecad_session import get_step_document

    doc = get_step_document(path)

    by_name: dict[str, MeshArrays] = {}
    by_label: dict[str, MeshArrays] = {}
    lin_mm, ang, relative, rel_defl, max_tris, min_vol = _tess_params(tolerances, quality)
    preview = quality == "preview"

    wanted_labels = {p.name for p in targets}
    wanted_refs = {p.shape_ref for p in targets if p.shape_ref}
    # Skip FreeCAD tessellate for tiny IR parts — bbox placeholder is enough for preview.
    tiny_labels = {
        p.name for p in targets if preview and (p.volume or 0.0) < min_vol
    }
    tiny_refs = {
        p.shape_ref
        for p in targets
        if preview and p.shape_ref and (p.volume or 0.0) < min_vol
    }

    n_done = 0
    n_skip_tiny = 0
    t0 = time.perf_counter()

    for obj in doc.Objects:
        if not hasattr(obj, "Shape") or obj.Shape.isNull():
            continue
        label = obj.Label or ""
        name = obj.Name or ""
        solids = list(getattr(obj.Shape, "Solids", []) or [])
        if len(solids) > 12 and label not in wanted_labels and name not in wanted_refs:
            continue
        try:
            vol = abs(float(getattr(obj.Shape, "Volume", 0) or 0))
            if vol < 1e-6:
                continue
        except Exception:
            continue

        wanted = label in wanted_labels or name in wanted_refs or not wanted_labels
        if not wanted and not (1 < len(solids) <= 12):
            continue

        if preview and (label in tiny_labels or name in tiny_refs):
            n_skip_tiny += 1
            continue

        M = _world_placement_mat4(
            obj.Shape, _obj_placement(obj), ir.meta.get("world_frame_mode")
        )
        if world_align is not None:
            M = world_align @ M

        if wanted:
            mesh = _tessellate_shape_world(
                obj.Shape,
                lin_mm,
                M,
                angular_rad=ang,
                relative=relative,
                relative_deflection=rel_defl,
                max_tris=max_tris if preview else 0,
                prefer_fast=preview,
            )
            if mesh is not None:
                by_name[name] = mesh
                by_label[label] = mesh
                n_done += 1
                if preview and n_done % 8 == 0:
                    print(
                        f"    tessellate {n_done}/{len(targets)} "
                        f"({time.perf_counter() - t0:.1f}s)",
                        flush=True,
                    )

        if 1 < len(solids) <= 12:
            for i, solid in enumerate(solids):
                sub = f"{label}_s{i}"
                if sub not in wanted_labels:
                    continue
                if preview and sub in tiny_labels:
                    n_skip_tiny += 1
                    continue
                sm = _tessellate_shape_world(
                    solid,
                    lin_mm,
                    M,
                    angular_rad=ang,
                    relative=relative,
                    relative_deflection=rel_defl,
                    max_tris=max_tris if preview else 0,
                    prefer_fast=preview,
                )
                if sm:
                    by_label[sub] = sm
                    n_done += 1

    if preview:
        print(
            f"    tessellated={n_done} tiny_bbox={n_skip_tiny} "
            f"in {time.perf_counter() - t0:.1f}s",
            flush=True,
        )

    for part in targets:
        mesh = None
        if part.shape_ref and part.shape_ref in by_name:
            mesh = by_name[part.shape_ref]
        elif part.name in by_label:
            mesh = by_label[part.name]
        elif part.name in by_name:
            mesh = by_name[part.name]

        if preview and (part.volume or 0.0) < min_vol:
            mesh = _bbox_mesh_world(part, world_align)

        if mesh is not None:
            part._mesh_np = mesh  # type: ignore[attr-defined]
            V, F = mesh
            # Avoid huge .tolist() — export reads _mesh_np.
            if V.shape[0] <= 40_000:
                part.mesh_vertices = V.tolist()
                part.mesh_faces = F.tolist()
            else:
                part.mesh_vertices = None
                part.mesh_faces = None
        elif preview:
            mesh = _bbox_mesh_world(part, world_align)
            part._mesh_np = mesh  # type: ignore[attr-defined]
            part.mesh_vertices = mesh[0].tolist()
            part.mesh_faces = mesh[1].tolist()
    return ir


def _tess_params(tol: Tolerances, quality: MeshQuality):
    if quality == "final":
        return (
            max(float(tol.mesh_linear_deflection_m) * 1000.0, 1.0),
            float(tol.mesh_angular_deflection_rad),
            False,
            0.0,
            0,
            0.0,
        )
    return (
        max(float(tol.mesh_preview_linear_deflection_m) * 1000.0, 1.0),
        float(tol.mesh_preview_angular_deflection_rad),
        bool(tol.mesh_preview_relative),
        float(tol.mesh_preview_relative_deflection),
        int(tol.mesh_preview_max_tris_per_part),
        float(tol.mesh_preview_min_volume_m3),
    )


def _bbox_mesh_world(
    part: PartInstance, world_align: Optional[np.ndarray]
) -> MeshArrays:
    bb = part.bbox.as_array()
    xmin, ymin, zmin = bb[0]
    xmax, ymax, zmax = bb[1]
    V = np.array(
        [
            [xmin, ymin, zmin],
            [xmax, ymin, zmin],
            [xmax, ymax, zmin],
            [xmin, ymax, zmin],
            [xmin, ymin, zmax],
            [xmax, ymin, zmax],
            [xmax, ymax, zmax],
            [xmin, ymax, zmax],
        ],
        dtype=np.float64,
    )
    if world_align is not None:
        R, t = world_align[:3, :3], world_align[:3, 3]
        V = (R @ V.T).T + t
    F = np.array(
        [
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
        ],
        dtype=np.int32,
    )
    return V, F


def _tessellate_shape_world(
    shape,
    linear_deflection_mm: float,
    placement_m: np.ndarray,
    *,
    angular_rad: float = 0.5,
    relative: bool = False,
    relative_deflection: float = 0.02,
    max_tris: int = 0,
    prefer_fast: bool = False,
) -> Optional[MeshArrays]:
    try:
        local, faces = _call_tessellate(
            shape,
            linear_deflection_mm,
            angular_rad,
            relative=relative,
            relative_deflection=relative_deflection,
            prefer_fast=prefer_fast,
        )
        if local is None or faces is None or len(local) < 3 or len(faces) < 1:
            return None
        # FreeCAD tessellate returns mm
        local = local * 1e-3
        R = placement_m[:3, :3]
        t = placement_m[:3, 3]
        world = (R @ local.T).T + t
        if not np.all(np.isfinite(world)):
            return None
        if max_tris > 0 and faces.shape[0] > max_tris:
            faces = _stride_subsample_faces(faces, max_tris)
        return world.astype(np.float64, copy=False), faces.astype(np.int32, copy=False)
    except Exception:
        return None


def _stride_subsample_faces(faces: np.ndarray, max_tris: int) -> np.ndarray:
    n = faces.shape[0]
    step = int(np.ceil(n / max_tris))
    return faces[::step][:max_tris]


def _call_tessellate(
    shape,
    linear_deflection_mm: float,
    angular_rad: float,
    *,
    relative: bool,
    relative_deflection: float,
    prefer_fast: bool = False,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Return (Nx3 mm points, Mx3 int faces)."""
    lin_mm = _absolute_deflection_mm(
        shape, linear_deflection_mm, relative, relative_deflection
    )

    # Preview: Shape.tessellate is typically much faster than MeshPart.
    if prefer_fast:
        out = _tessellate_shape_native(shape, lin_mm)
        if out[0] is not None:
            return out

    try:
        import MeshPart

        kwargs = {
            "Shape": shape,
            "LinearDeflection": float(relative_deflection if relative else lin_mm),
            "AngularDeflection": float(angular_rad),
            "Relative": bool(relative),
        }
        mesh = MeshPart.meshFromShape(**kwargs)
        return _mesh_to_numpy(mesh)
    except Exception:
        pass

    return _tessellate_shape_native(shape, lin_mm)


def _absolute_deflection_mm(
    shape,
    linear_deflection_mm: float,
    relative: bool,
    relative_deflection: float,
) -> float:
    """Absolute linear deflection in mm for Shape.tessellate."""
    lin = max(float(linear_deflection_mm), 1.0)
    if not relative:
        return lin
    try:
        diag = float(shape.BoundBox.DiagonalLength)
        if diag > 1e-9:
            return max(diag * float(relative_deflection), lin)
    except Exception:
        pass
    return lin


def _tessellate_shape_native(
    shape, linear_deflection_mm: float
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    try:
        points, facets = shape.tessellate(float(linear_deflection_mm))
        if not points or not facets:
            return None, None
        local = np.asarray(
            [(float(p.x), float(p.y), float(p.z)) for p in points],
            dtype=np.float64,
        )
        faces = np.asarray(facets, dtype=np.int32).reshape(-1, 3)
        return local, faces
    except Exception:
        return None, None


def _mesh_to_numpy(mesh) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Convert FreeCAD Mesh to numpy."""
    try:
        topo = mesh.Topology
        pts, fcs = topo[0], topo[1]
        n = len(pts)
        if n < 3 or not fcs:
            return None, None
        p0 = pts[0]
        if hasattr(p0, "x"):
            local = np.asarray(
                [(float(p.x), float(p.y), float(p.z)) for p in pts],
                dtype=np.float64,
            )
        else:
            local = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        faces = np.asarray(fcs, dtype=np.int32).reshape(-1, 3)
        return local, faces
    except Exception:
        return None, None
