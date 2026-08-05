"""CAD (Z-up) ↔ glTF/Godot (Y-up) frame conversions — single source of truth."""

from __future__ import annotations

import numpy as np

from common.math3d import Mat4, mat4_identity


def cad_z_up_to_gltf_y_up() -> Mat4:
    """Map CAD +Z up to glTF +Y up: (x,y,z)_cad -> (x,z,-y)_gltf."""
    M = mat4_identity()
    M[:3, :3] = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=np.float64,
    )
    return M


def apply_frame(M_world_cad: Mat4, to_gltf: bool = True) -> Mat4:
    if not to_gltf:
        return M_world_cad
    T = cad_z_up_to_gltf_y_up()
    return T @ M_world_cad @ np.linalg.inv(T)
