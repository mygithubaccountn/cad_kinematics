"""3D vector / matrix helpers. All units metres; rotations right-handed."""

from __future__ import annotations

from typing import Iterable

import numpy as np

Vec3 = np.ndarray
Mat4 = np.ndarray


def as_vec3(v: Iterable[float]) -> np.ndarray:
    a = np.asarray(list(v), dtype=np.float64).reshape(3)
    return a


def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.zeros(3, dtype=np.float64)
    return v / n


def nearly_parallel(a: np.ndarray, b: np.ndarray, angle_eps_rad: float) -> bool:
    a_n = normalize(a)
    b_n = normalize(b)
    if np.linalg.norm(a_n) < 1e-12 or np.linalg.norm(b_n) < 1e-12:
        return False
    c = abs(float(np.clip(np.dot(a_n, b_n), -1.0, 1.0)))
    return c >= float(np.cos(angle_eps_rad))


def axis_distance(p0: np.ndarray, d0: np.ndarray, p1: np.ndarray, d1: np.ndarray) -> float:
    """Shortest distance between two 3D lines (point + direction)."""
    d0n = normalize(d0)
    d1n = normalize(d1)
    n = np.cross(d0n, d1n)
    nn = float(np.linalg.norm(n))
    w = p1 - p0
    if nn < 1e-12:
        # Parallel: distance from p1 to line0
        return float(np.linalg.norm(np.cross(w, d0n)))
    return abs(float(np.dot(w, n))) / nn


def project_point_to_axis(point: np.ndarray, origin: np.ndarray, direction: np.ndarray) -> np.ndarray:
    d = normalize(direction)
    return origin + d * float(np.dot(point - origin, d))


def orthonormal_basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (x, y, z) with z = normalized axis, deterministic x/y."""
    z = normalize(axis)
    # Prefer world up that is not parallel to z
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z, up))) > 0.9:
        up = np.array([0.0, 1.0, 0.0])
    x = normalize(np.cross(up, z))
    if np.linalg.norm(x) < 1e-12:
        x = np.array([1.0, 0.0, 0.0])
    y = normalize(np.cross(z, x))
    return x, y, z


def mat4_identity() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def mat4_from_rt(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = mat4_identity()
    M[:3, :3] = R
    M[:3, 3] = as_vec3(t)
    return M


def mat4_from_origin_axis(origin: np.ndarray, axis: np.ndarray) -> np.ndarray:
    x, y, z = orthonormal_basis(axis)
    R = np.column_stack([x, y, z])
    return mat4_from_rt(R, origin)


def transform_point(M: np.ndarray, p: np.ndarray) -> np.ndarray:
    v = np.ones(4, dtype=np.float64)
    v[:3] = as_vec3(p)
    return (M @ v)[:3]


def transform_dir(M: np.ndarray, d: np.ndarray) -> np.ndarray:
    return M[:3, :3] @ as_vec3(d)


def invert_mat4(M: np.ndarray) -> np.ndarray:
    R = M[:3, :3]
    t = M[:3, 3]
    Ri = R.T
    Mi = mat4_identity()
    Mi[:3, :3] = Ri
    Mi[:3, 3] = -Ri @ t
    return Mi


def mat4_to_list(M: np.ndarray) -> list[list[float]]:
    return [[float(M[r, c]) for c in range(4)] for r in range(4)]


def mat4_from_list(rows: list[list[float]]) -> np.ndarray:
    return np.asarray(rows, dtype=np.float64).reshape(4, 4)


def bbox_union(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a,b shape (2,3) min/max."""
    return np.vstack([np.minimum(a[0], b[0]), np.maximum(a[1], b[1])])


def bbox_center(bbox: np.ndarray) -> np.ndarray:
    return 0.5 * (bbox[0] + bbox[1])


def bbox_contains(bbox: np.ndarray, p: np.ndarray, margin: float = 0.0) -> bool:
    lo = bbox[0] - margin
    hi = bbox[1] + margin
    return bool(np.all(p >= lo) and np.all(p <= hi))


def rotation_matrix_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    a = normalize(axis)
    x, y, z = a
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=np.float64,
    )
