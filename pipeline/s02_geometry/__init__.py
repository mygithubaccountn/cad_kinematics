"""02_geometry — BRep/feature extraction, concentric clusters, contact graph."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from pipeline.common.io_util import write_json
from pipeline.common.math3d import (
    as_vec3,
    axis_distance,
    bbox_center,
    nearly_parallel,
    normalize,
    project_point_to_axis,
)
from pipeline.common.models import (
    AdjacencyEdge,
    AssemblyIR,
    ConcentricCluster,
    ContactPair,
    CylFeature,
    CylKind,
    FeatureGraph,
    PartInstance,
)
from pipeline.common.tolerances import Tolerances
from pipeline.s01_import.synthetic import load_prebuilt_features
from pipeline.s02_geometry.freecad_faces import extract_cylinders_freecad


def run_geometry(
    ir: AssemblyIR,
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
) -> FeatureGraph:
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Prefer prebuilt features for synthetic fixtures
    pre = load_prebuilt_features(Path(ir.source_path))
    if pre is not None and pre.cylinders:
        fg = pre
    else:
        cylinders = extract_cylinders_freecad(ir, tolerances)
        if not cylinders:
            cylinders = _infer_cylinders_from_bbox_heuristic(ir, tolerances)
        fg = FeatureGraph(cylinders=cylinders)

    fg.clusters = cluster_concentric(fg.cylinders, tolerances)
    fg.contacts = estimate_contacts(ir, tolerances)
    fg.adjacency = build_adjacency(fg.contacts, fg.clusters, fg.cylinders)
    fg.meta = {**fg.meta, "n_cylinders": len(fg.cylinders), "n_clusters": len(fg.clusters)}

    write_json(out / "features.json", fg.to_dict())
    return fg


def cluster_concentric(cylinders: list[CylFeature], tol: Tolerances) -> list[ConcentricCluster]:
    """Union-find cylinders with nearly parallel axes and small axis distance."""
    n = len(cylinders)
    parent = list(range(n))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i: int, j: int) -> None:
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)  # deterministic

    for i in range(n):
        for j in range(i + 1, n):
            a, b = cylinders[i], cylinders[j]
            if nearly_parallel(as_vec3(a.axis_dir), as_vec3(b.axis_dir), tol.angle_eps_rad):
                dist = axis_distance(
                    as_vec3(a.axis_point),
                    as_vec3(a.axis_dir),
                    as_vec3(b.axis_point),
                    as_vec3(b.axis_dir),
                )
                if dist <= tol.axis_dist_eps_m:
                    union(i, j)

    groups: dict[int, list[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    clusters: list[ConcentricCluster] = []
    for gi, idxs in sorted(groups.items(), key=lambda kv: min(kv[1])):
        idxs = sorted(idxs)
        pts = np.array([cylinders[i].axis_point for i in idxs], dtype=np.float64)
        dirs = np.array([normalize(as_vec3(cylinders[i].axis_dir)) for i in idxs])
        # Align direction signs to first
        ref = dirs[0]
        for k in range(len(dirs)):
            if float(np.dot(dirs[k], ref)) < 0:
                dirs[k] = -dirs[k]
        axis_dir = normalize(dirs.mean(axis=0))
        # Robust axis point: median of projections onto mean axis through first point
        origin = pts[0]
        ts = [float(np.dot(pts[k] - origin, axis_dir)) for k in range(len(pts))]
        axis_point = origin + axis_dir * float(np.median(ts))
        cyl_ids = [cylinders[i].id for i in idxs]
        part_ids = sorted({cylinders[i].part_id for i in idxs})
        clusters.append(
            ConcentricCluster(
                id=f"cluster_{len(clusters):03d}",
                axis_point=axis_point.tolist(),
                axis_dir=axis_dir.tolist(),
                cyl_ids=cyl_ids,
                part_ids=part_ids,
            )
        )
    return clusters


def estimate_contacts(ir: AssemblyIR, tol: Tolerances) -> list[ContactPair]:
    """AABB proximity + surface sample distances between part pairs."""
    parts = sorted(ir.parts, key=lambda p: p.id)
    contacts: list[ContactPair] = []
    rng = np.random.default_rng(tol.seed)

    for i, a in enumerate(parts):
        ba = a.bbox.as_array()
        for b in parts[i + 1 :]:
            bb = b.bbox.as_array()
            # Expanded AABB overlap test
            lo = np.maximum(ba[0], bb[0]) - tol.bbox_expand_m
            hi = np.minimum(ba[1], bb[1]) + tol.bbox_expand_m
            if np.any(lo > hi + tol.contact_gap_m):
                continue
            sa = _sample_part_points(a, tol.contact_sample_count, rng)
            sb = _sample_part_points(b, tol.contact_sample_count, rng)
            # Pairwise min distances (coarse)
            strength = 0.0
            cents = []
            for p in sa:
                d = np.linalg.norm(sb - p, axis=1)
                m = float(d.min())
                if m <= tol.contact_gap_m:
                    strength += 1.0
                    cents.append(p)
            if strength <= 0:
                continue
            strength /= float(tol.contact_sample_count)
            centroid = np.mean(cents, axis=0) if cents else bbox_center(ba)
            contacts.append(
                ContactPair(
                    part_a=a.id,
                    part_b=b.id,
                    strength=float(strength),
                    centroid=centroid.tolist(),
                    sample_count=len(cents),
                )
            )
    contacts.sort(key=lambda c: (c.part_a, c.part_b))
    return contacts


def build_adjacency(
    contacts: list[ContactPair],
    clusters: list[ConcentricCluster],
    cylinders: list[CylFeature],
) -> list[AdjacencyEdge]:
    edges: dict[tuple[str, str], AdjacencyEdge] = {}

    def edge(a: str, b: str) -> AdjacencyEdge:
        key = tuple(sorted((a, b)))
        if key not in edges:
            edges[key] = AdjacencyEdge(part_a=key[0], part_b=key[1])
        return edges[key]

    for c in contacts:
        e = edge(c.part_a, c.part_b)
        e.contact_weight = max(e.contact_weight, c.strength)

    cyl_map = {c.id: c for c in cylinders}
    for cl in clusters:
        parts = sorted(set(cl.part_ids))
        if len(parts) < 2:
            continue
        # Weight by number of cylinders spanning distinct parts
        for i, pa in enumerate(parts):
            for pb in parts[i + 1 :]:
                e = edge(pa, pb)
                e.shared_axis_weight = max(e.shared_axis_weight, 0.5 + 0.1 * len(cl.cyl_ids))

    return [edges[k] for k in sorted(edges.keys())]


def _sample_part_points(part: PartInstance, n: int, rng: np.random.Generator) -> np.ndarray:
    if part.mesh_vertices:
        v = np.asarray(part.mesh_vertices, dtype=np.float64)
        if len(v) >= n:
            idx = rng.choice(len(v), size=n, replace=False)
            return v[idx]
        # Upsample with noise on bbox
        extra = n - len(v)
        bb = part.bbox.as_array()
        rnd = rng.uniform(bb[0], bb[1], size=(extra, 3))
        return np.vstack([v, rnd])
    bb = part.bbox.as_array()
    return rng.uniform(bb[0], bb[1], size=(n, 3))


def _infer_cylinders_from_bbox_heuristic(ir: AssemblyIR, tol: Tolerances) -> list[CylFeature]:
    """
    Last-resort heuristic when OCC faces unavailable: no fake joints from bbox alone.
    Returns empty — joint detection must not invent shafts from bbox.
    """
    return []
