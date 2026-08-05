"""Minimal binary glTF (.glb) writer for triangle meshes."""

from __future__ import annotations

import json
import struct
from pathlib import Path

import numpy as np


def write_glb_triangles(
    path: Path | str,
    vertices: np.ndarray,
    faces: np.ndarray,
    *,
    include_normals: bool = True,
) -> Path:
    """
    Write a single-mesh GLB. Vertices in metres, Y-up conversion is NOT applied here;
    scene generation decides frame. Positions are written as-is (CAD metres).

    ``include_normals=False`` skips normal computation (faster preview export; Godot OK).
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    V = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    V = np.nan_to_num(V, nan=0.0, posinf=0.0, neginf=0.0)
    V = np.clip(V, -1.0e3, 1.0e3).astype(np.float32, copy=False)
    F = np.asarray(faces, dtype=np.uint32).reshape(-1, 3)

    indices = F.reshape(-1).astype(np.uint32, copy=False)
    mins = V.min(axis=0).tolist()
    maxs = V.max(axis=0).tolist()

    if include_normals and F.size:
        v0 = V[F[:, 0]]
        v1 = V[F[:, 1]]
        v2 = V[F[:, 2]]
        face_n = np.cross(v1 - v0, v2 - v0)
        lens = np.linalg.norm(face_n, axis=1, keepdims=True)
        np.maximum(lens, 1e-12, out=lens)
        face_n /= lens
        normals = np.zeros_like(V)
        np.add.at(normals, F[:, 0], face_n)
        np.add.at(normals, F[:, 1], face_n)
        np.add.at(normals, F[:, 2], face_n)
        nl = np.linalg.norm(normals, axis=1, keepdims=True)
        np.maximum(nl, 1e-12, out=nl)
        normals = (normals / nl).astype(np.float32)
        bin_blob = indices.tobytes() + V.tobytes() + normals.tobytes()
        nrm_bytes = normals.nbytes
    else:
        normals = None
        bin_blob = indices.tobytes() + V.tobytes()
        nrm_bytes = 0

    pad = (4 - (len(bin_blob) % 4)) % 4
    if pad:
        bin_blob += b"\x00" * pad

    idx_bytes = indices.nbytes
    vtx_bytes = V.nbytes

    buffer_views = [
        {"buffer": 0, "byteOffset": 0, "byteLength": idx_bytes, "target": 34963},
        {"buffer": 0, "byteOffset": idx_bytes, "byteLength": vtx_bytes, "target": 34962},
    ]
    accessors = [
        {
            "bufferView": 0,
            "componentType": 5125,
            "count": int(indices.size),
            "type": "SCALAR",
            "max": [int(indices.max())] if indices.size else [0],
            "min": [int(indices.min())] if indices.size else [0],
        },
        {
            "bufferView": 1,
            "componentType": 5126,
            "count": int(len(V)),
            "type": "VEC3",
            "max": maxs,
            "min": mins,
        },
    ]
    attributes: dict = {"POSITION": 1}
    if normals is not None:
        buffer_views.append(
            {
                "buffer": 0,
                "byteOffset": idx_bytes + vtx_bytes,
                "byteLength": nrm_bytes,
                "target": 34962,
            }
        )
        accessors.append(
            {
                "bufferView": 2,
                "componentType": 5126,
                "count": int(len(normals)),
                "type": "VEC3",
            }
        )
        attributes["NORMAL"] = 2

    gltf = {
        "asset": {"version": "2.0", "generator": "cad-godot-robot-pipeline"},
        "buffers": [{"byteLength": len(bin_blob)}],
        "bufferViews": buffer_views,
        "accessors": accessors,
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": attributes,
                        "indices": 0,
                        "mode": 4,
                    }
                ]
            }
        ],
        "nodes": [{"mesh": 0, "name": path.stem}],
        "scenes": [{"nodes": [0]}],
        "scene": 0,
    }

    json_bytes = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    json_pad = (4 - (len(json_bytes) % 4)) % 4
    if json_pad:
        json_bytes += b" " * json_pad

    total_len = 12 + 8 + len(json_bytes) + 8 + len(bin_blob)
    with path.open("wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total_len))
        f.write(struct.pack("<I4s", len(json_bytes), b"JSON"))
        f.write(json_bytes)
        f.write(struct.pack("<I4s", len(bin_blob), b"BIN\x00"))
        f.write(bin_blob)
    return path
