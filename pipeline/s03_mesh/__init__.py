"""03_mesh — mesh extraction / GLB export."""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Optional

import numpy as np

from pipeline.common.io_util import ensure_dir, write_json
from pipeline.common.math3d import invert_mat4, mat4_from_list, transform_point
from pipeline.common.models import AssemblyIR, KinematicTree, PartInstance
from pipeline.common.tolerances import Tolerances
from pipeline.s03_mesh.glb import write_glb_triangles


def run_mesh_export(
    ir: AssemblyIR,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
    tree: Optional[KinematicTree] = None,
) -> dict[str, str]:
    """
    Export meshes. If kinematic tree provided, merge parts per link into link-local GLB.
    Otherwise export one GLB per part (raw stage).
    Returns map id -> relative mesh path.
    """
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    mesh_dir = ensure_dir(out / "meshes")
    mapping: dict[str, str] = {}

    if tree is None:
        for part in sorted(ir.parts, key=lambda p: p.id):
            verts, faces = _part_triangles(part)
            if verts is None:
                continue
            rel = f"meshes/{part.id}.glb"
            write_glb_triangles(mesh_dir / f"{part.id}.glb", verts, faces)
            mapping[part.id] = rel
    else:
        part_map = ir.part_map()
        for link in sorted(tree.links, key=lambda l: l.id):
            M_world = mat4_from_list(tree.link_world[link.id])
            M_inv = invert_mat4(M_world)
            all_v: list[np.ndarray] = []
            all_f: list[np.ndarray] = []
            v_offset = 0
            for pid in link.part_ids:
                part = part_map[pid]
                verts, faces = _part_triangles(part)
                if verts is None:
                    continue
                # Transform world verts into link local
                local = np.array([transform_point(M_inv, v) for v in verts])
                all_v.append(local)
                all_f.append(faces + v_offset)
                v_offset += len(local)
            if not all_v:
                continue
            V = np.vstack(all_v)
            F = np.vstack(all_f)
            rel = f"meshes/{link.id}.glb"
            write_glb_triangles(mesh_dir / f"{link.id}.glb", V, F)
            mapping[link.id] = rel
            link.mesh_path = rel

    write_json(out / "mesh_index.json", mapping)
    return mapping


def _part_triangles(part: PartInstance) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    if part.mesh_vertices and part.mesh_faces:
        V = np.asarray(part.mesh_vertices, dtype=np.float64)
        F = np.asarray(part.mesh_faces, dtype=np.int32)
        return V, F
    # Fallback: bbox box mesh
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
