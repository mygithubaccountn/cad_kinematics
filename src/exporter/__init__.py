"""03_mesh — mesh extraction / GLB export."""

from __future__ import annotations

from pathlib import Path
from typing import Literal, Optional

import numpy as np

from common.io_util import ensure_dir, write_json
from common.math3d import invert_mat4, mat4_from_list, transform_point
from common.models import AssemblyIR, KinematicTree, PartInstance
from common.tolerances import Tolerances
from exporter.glb import write_glb_triangles

MeshQuality = Literal["preview", "final"]


def run_mesh_export(
    ir: AssemblyIR,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
    tree: Optional[KinematicTree] = None,
    to_gltf_y_up: bool = False,
    world_align: Optional[np.ndarray] = None,
    *,
    tessellate: bool = True,
    write_glb: bool = True,
    quality: MeshQuality = "preview",
) -> dict[str, str]:
    """
    Export meshes. If kinematic tree provided, merge parts per link into link-local GLB.

    Preview: coarse/relative tessellation, no GLB normals, world_align baked into tessellate.
    """
    from common.timing import timed

    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    mesh_dir = ensure_dir(out / "meshes")
    mapping: dict[str, str] = {}
    preview = quality == "preview"
    # Preview bakes world_align into FreeCAD tessellate → one fewer export transform.
    bake_align = world_align if preview else None
    export_align = None if preview else world_align

    if tree is None:
        for part in sorted(ir.parts, key=lambda p: p.id):
            mapping[part.id] = f"meshes/{part.id}.glb"
    else:
        for link in sorted(tree.links, key=lambda l: l.id):
            rel = f"meshes/{link.id}.glb"
            mapping[link.id] = rel
            link.mesh_path = rel

    if not write_glb:
        from common.timing import mark_skip

        mark_skip("Mesh generation", note="skipped (kinematics-only package)")
        mark_skip("GLB export", note="skipped (kinematics-only package)")
        return mapping

    # Drop stale GLBs so remesh never leaves multi-million-tri leftovers for Godot.
    for stale in mesh_dir.glob("*.glb"):
        try:
            stale.unlink()
        except OSError:
            pass

    if tessellate:
        with timed("Mesh generation"):
            try:
                from exporter.freecad_mesh import enrich_ir_with_freecad_meshes

                needed = set()
                if tree is None:
                    needed = {p.id for p in ir.parts}
                else:
                    for link in tree.links:
                        needed.update(link.part_ids)
                enrich_ir_with_freecad_meshes(
                    ir,
                    tolerances,
                    only_part_ids=needed,
                    quality=quality,
                    world_align=bake_align,
                )
            except Exception:
                pass
    else:
        from common.timing import mark_skip

        mark_skip("Mesh generation", note="skipped (no tessellate)")

    R_yup = None
    if to_gltf_y_up:
        from common.frames import cad_z_up_to_gltf_y_up

        R_yup = cad_z_up_to_gltf_y_up()[:3, :3]

    def _xform(V: np.ndarray, M: np.ndarray) -> np.ndarray:
        return (M[:3, :3] @ V.T).T + M[:3, 3]

    def _align_world(V: np.ndarray) -> np.ndarray:
        if export_align is None:
            return V
        return _xform(V, export_align)

    def _maybe_yup(V: np.ndarray) -> np.ndarray:
        if R_yup is None:
            return V
        return (R_yup @ V.T).T

    with timed("GLB export"):
        include_normals = not preview
        if tree is None:
            for part in sorted(ir.parts, key=lambda p: p.id):
                verts, faces = _part_triangles(part)
                if verts is None:
                    continue
                verts = _maybe_yup(_align_world(verts))
                write_glb_triangles(
                    mesh_dir / f"{part.id}.glb",
                    verts,
                    faces,
                    include_normals=include_normals,
                )
        else:
            part_map = ir.part_map()
            for link in sorted(tree.links, key=lambda l: l.id):
                M_world = mat4_from_list(tree.link_world[link.id])
                if export_align is not None:
                    pivot = transform_point(export_align, M_world[:3, 3])
                    M_world = np.eye(4)
                    M_world[:3, 3] = pivot
                elif bake_align is not None:
                    # world_align already baked into vertices; pivots must match
                    pivot = transform_point(bake_align, M_world[:3, 3])
                    M_world = np.eye(4)
                    M_world[:3, 3] = pivot
                M_inv = invert_mat4(M_world)
                all_v: list[np.ndarray] = []
                all_f: list[np.ndarray] = []
                v_offset = 0
                for pid in link.part_ids:
                    part = part_map[pid]
                    verts, faces = _part_triangles(part)
                    if verts is None:
                        continue
                    W = _align_world(verts)
                    local = _maybe_yup(_xform(W, M_inv))
                    all_v.append(local)
                    all_f.append(faces + v_offset)
                    v_offset += len(local)
                if not all_v:
                    continue
                V = np.vstack(all_v)
                F = np.vstack(all_f)
                if preview:
                    budget = int(getattr(tolerances, "mesh_preview_max_tris_per_link", 120_000))
                    if budget > 0 and F.shape[0] > budget:
                        step = int(np.ceil(F.shape[0] / budget))
                        F = F[::step][:budget]
                        V, F = _compact_indexed_mesh(V, F)
                write_glb_triangles(
                    out / mapping[link.id],
                    V,
                    F,
                    include_normals=include_normals,
                )

        write_json(
            out / "mesh_index.json",
            {"quality": quality, "meshes": mapping},
        )
    return mapping


def _compact_indexed_mesh(
    vertices: np.ndarray, faces: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Drop unused vertices after face subsampling."""
    used = np.unique(faces.reshape(-1))
    remap = np.full(len(vertices), -1, dtype=np.int32)
    remap[used] = np.arange(len(used), dtype=np.int32)
    return vertices[used], remap[faces]


def _part_triangles(part: PartInstance) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    cached = getattr(part, "_mesh_np", None)
    if cached is not None:
        V, F = cached
        return np.asarray(V, dtype=np.float64), np.asarray(F, dtype=np.int32)
    if part.mesh_vertices and part.mesh_faces:
        V = np.asarray(part.mesh_vertices, dtype=np.float64)
        F = np.asarray(part.mesh_faces, dtype=np.int32)
        return V, F
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
