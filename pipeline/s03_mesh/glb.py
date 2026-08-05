"""Minimal binary glTF (.glb) writer for triangle meshes."""

from __future__ import annotations

import json
import struct
from pathlib import Path

import numpy as np


def write_glb_triangles(path: Path | str, vertices: np.ndarray, faces: np.ndarray) -> Path:
    """
    Write a single-mesh GLB. Vertices in metres, Y-up conversion is NOT applied here;
    scene generation decides frame. Positions are written as-is (CAD metres).
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    V = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    V = np.nan_to_num(V, nan=0.0, posinf=0.0, neginf=0.0)
    V = np.clip(V, -1.0e3, 1.0e3).astype(np.float32)
    F = np.asarray(faces, dtype=np.uint32).reshape(-1, 3)

    # Compute normals (flat average)
    normals = np.zeros_like(V)
    for tri in F:
        a, b, c = V[tri[0]], V[tri[1]], V[tri[2]]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n)
        if ln > 1e-12:
            n = n / ln
        normals[tri[0]] += n
        normals[tri[1]] += n
        normals[tri[2]] += n
    lens = np.linalg.norm(normals, axis=1, keepdims=True)
    lens[lens < 1e-12] = 1.0
    normals = (normals / lens).astype(np.float32)

    indices = F.reshape(-1).astype(np.uint32)
    # Pad index buffer to 4-byte alignment (already uint32)
    bin_parts = [indices.tobytes(), V.tobytes(), normals.tobytes()]
    bin_blob = b"".join(bin_parts)
    # Pad to 4 bytes
    pad = (4 - (len(bin_blob) % 4)) % 4
    bin_blob += b"\x00" * pad

    idx_bytes = indices.nbytes
    vtx_bytes = V.nbytes
    nrm_bytes = normals.nbytes

    mins = V.min(axis=0).tolist()
    maxs = V.max(axis=0).tolist()

    gltf = {
        "asset": {"version": "2.0", "generator": "cad-godot-robot-pipeline"},
        "buffers": [{"byteLength": len(bin_blob)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": idx_bytes, "target": 34963},
            {"buffer": 0, "byteOffset": idx_bytes, "byteLength": vtx_bytes, "target": 34962},
            {
                "buffer": 0,
                "byteOffset": idx_bytes + vtx_bytes,
                "byteLength": nrm_bytes,
                "target": 34962,
            },
        ],
        "accessors": [
            {
                "bufferView": 0,
                "componentType": 5125,
                "count": int(indices.size),
                "type": "SCALAR",
                "max": [int(indices.max())],
                "min": [int(indices.min())],
            },
            {
                "bufferView": 1,
                "componentType": 5126,
                "count": int(len(V)),
                "type": "VEC3",
                "max": maxs,
                "min": mins,
            },
            {
                "bufferView": 2,
                "componentType": 5126,
                "count": int(len(normals)),
                "type": "VEC3",
            },
        ],
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": {"POSITION": 1, "NORMAL": 2},
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
    json_bytes += b" " * json_pad

    total_len = 12 + 8 + len(json_bytes) + 8 + len(bin_blob)
    with path.open("wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total_len))
        f.write(struct.pack("<I4s", len(json_bytes), b"JSON"))
        f.write(json_bytes)
        f.write(struct.pack("<I4s", len(bin_blob), b"BIN\x00"))
        f.write(bin_blob)
    return path
