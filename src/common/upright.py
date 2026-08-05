"""Upright / gravity-frame normalization for assembled STEP robots.

CAD files arrive with arbitrary +up. We estimate up from base→COM, then rotate
so +Z is up and the base sits on z=0. Viewer maps Z-up → Three.js Y-up.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from common.math3d import as_vec3, mat4_from_rt, mat4_identity, normalize
from common.models import AssemblyIR, KinematicTree


def estimate_up_axis(ir: AssemblyIR) -> np.ndarray:
    """Rough world-up before base is known: axis of greatest part-center spread."""
    if not ir.parts:
        return np.array([0.0, 0.0, 1.0])
    C = np.vstack([p.bbox.center() for p in ir.parts])
    var = np.var(C, axis=0)
    mins = np.min(np.vstack([p.bbox.min_xyz for p in ir.parts]), axis=0)
    maxs = np.max(np.vstack([p.bbox.max_xyz for p in ir.parts]), axis=0)
    ext = maxs - mins
    score = var + 0.05 * (ext ** 2)
    i = int(np.argmax(score))
    up = np.zeros(3, dtype=np.float64)
    up[i] = 1.0
    # Prefer orientation where COM is on the +up side of the AABB midplane
    com = _volume_com(ir)
    mid = 0.5 * (mins + maxs)
    if float(np.dot(com - mid, up)) < 0:
        up = -up
    return up


def upright_from_base(
    ir: AssemblyIR,
    tree: KinematicTree,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return (R, t) mapping CAD world → upright CAD (Z-up, base on z≈0).

    p_upright = R @ p + t
    """
    part_map = ir.part_map()
    base_link = tree.base_link
    base_parts = []
    for link in tree.links:
        if link.id == base_link:
            base_parts = [part_map[pid] for pid in link.part_ids if pid in part_map]
            break
    if not base_parts:
        base_parts = list(ir.parts)

    base_com = _parts_com(base_parts)
    asm_com = _volume_com(ir)
    up = normalize(asm_com - base_com)
    if float(np.linalg.norm(up)) < 1e-9:
        up = estimate_up_axis(ir)

    # Snap to nearest world axis — avoids skewed "diagonal upright" that looks broken
    ai = int(np.argmax(np.abs(up)))
    up_snap = np.zeros(3, dtype=np.float64)
    up_snap[ai] = 1.0 if up[ai] >= 0 else -1.0
    up = up_snap

    # If a base→child revolute axis is nearly parallel to up, keep that sign
    for j in tree.joints:
        if j.parent != base_link:
            continue
        if j.joint_type.value != "revolute":
            continue
        ax = normalize(as_vec3(j.axis_world))
        if abs(float(np.dot(ax, up))) > 0.85:
            if float(np.dot(ax, up)) < 0:
                up = -up
            break

    R = rotation_aligning(up, np.array([0.0, 0.0, 1.0]))
    # Sit base on z=0: lowest base corner after rotation
    corners = []
    for p in base_parts:
        bb = p.bbox.as_array()
        for x in (bb[0, 0], bb[1, 0]):
            for y in (bb[0, 1], bb[1, 1]):
                for z in (bb[0, 2], bb[1, 2]):
                    corners.append(R @ np.array([x, y, z], dtype=np.float64))
    if not corners:
        t = np.zeros(3)
    else:
        C = np.vstack(corners)
        t = np.array([0.0, 0.0, -float(C[:, 2].min())], dtype=np.float64)
        # Center XY on origin for nicer framing
        t[0] -= float(C[:, 0].mean())
        t[1] -= float(C[:, 1].mean())
    return R, t


def rotation_aligning(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """3x3 rotation taking unit vector a → unit vector b."""
    a = normalize(as_vec3(a))
    b = normalize(as_vec3(b))
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if float(np.linalg.norm(v)) < 1e-12:
        if c > 0:
            return np.eye(3, dtype=np.float64)
        # 180°: pick orthogonal axis
        axis = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(a, axis))) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis = normalize(np.cross(a, axis))
        K = _skew(axis)
        return np.eye(3) + 2.0 * (K @ K)
    vx = _skew(v)
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / float(np.dot(v, v)))


def upright_mat4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return mat4_from_rt(R, t)


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)


def _volume_com(ir: AssemblyIR) -> np.ndarray:
    return _parts_com(list(ir.parts))


def _parts_com(parts) -> np.ndarray:
    num = np.zeros(3, dtype=np.float64)
    den = 0.0
    for p in parts:
        w = abs(float(p.volume)) if np.isfinite(p.volume) else 0.0
        if w <= 0:
            w = 1e-9
        num += w * p.bbox.center()
        den += w
    return num / max(den, 1e-12)


def height_along(p_center: np.ndarray, up: np.ndarray) -> float:
    return float(np.dot(as_vec3(p_center), normalize(up)))
