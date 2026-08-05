"""06_hierarchy — build kinematic tree from joints + adjacency."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from pipeline.common.io_util import write_json
from pipeline.common.math3d import (
    as_vec3,
    invert_mat4,
    mat4_from_origin_axis,
    mat4_identity,
    mat4_to_list,
    normalize,
    transform_dir,
    transform_point,
)
from pipeline.common.models import (
    AssemblyIR,
    FeatureGraph,
    JointType,
    KinematicJoint,
    KinematicLink,
    KinematicTree,
    ResolvedJoint,
)
from pipeline.common.tolerances import Tolerances
from pipeline.common.trace import DecisionTrace
from pipeline.s06_hierarchy.parallel import detect_parallel_loops


def run_hierarchy(
    ir: AssemblyIR,
    features: FeatureGraph,
    joints: list[ResolvedJoint],
    out_dir: str | Path,
    tolerances: Optional[Tolerances] = None,
) -> KinematicTree:
    tolerances = tolerances or Tolerances()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    base = choose_base(ir, features, joints)
    # Undirected joint edges
    edge_joints: dict[tuple[str, str], ResolvedJoint] = {}
    for j in joints:
        key = tuple(sorted((j.parent, j.child)))
        prev = edge_joints.get(key)
        if prev is None or j.confidence > prev.confidence:
            edge_joints[key] = j

    # Fixed welds for high contact without joint
    weld_edges: list[tuple[str, str, float]] = []
    joint_pairs = set(edge_joints.keys())
    for e in features.adjacency:
        key = tuple(sorted((e.part_a, e.part_b)))
        if key in joint_pairs:
            continue
        if e.contact_weight >= 0.15:
            weld_edges.append((key[0], key[1], e.contact_weight))

    # Union-find welds into link components, but joints keep parts separate
    # Simpler MVP: each part is its own link; welds merge into parent link clusters
    components = _weld_components([p.id for p in ir.parts], weld_edges, joint_pairs)

    # Map part -> component root link id
    part_to_link = {p: f"link_{root}" for p, root in components.items()}
    # Ensure unique link ids stable
    link_parts: dict[str, list[str]] = {}
    for pid, lid in part_to_link.items():
        link_parts.setdefault(lid, []).append(pid)
    for lid in link_parts:
        link_parts[lid] = sorted(link_parts[lid])

    # Remap joints to link ids
    link_joints: list[tuple[str, str, ResolvedJoint]] = []
    for key, j in edge_joints.items():
        la, lb = part_to_link[j.parent], part_to_link[j.child]
        # parent/child on ResolvedJoint may be either order
        la2, lb2 = part_to_link[key[0]], part_to_link[key[1]]
        if la2 == lb2:
            continue  # welded somehow
        link_joints.append((la2, lb2, j))

    base_link = part_to_link[base]

    # Build spanning tree rooted at base using joint edges weighted by confidence
    undirected: dict[str, list[tuple[str, ResolvedJoint]]] = {lid: [] for lid in link_parts}
    for la, lb, j in link_joints:
        undirected.setdefault(la, []).append((lb, j))
        undirected.setdefault(lb, []).append((la, j))

    # If chosen base is disconnected from joint graph, re-root to heaviest link in joints
    jointed_links = {la for la, lb, _ in link_joints} | {lb for la, lb, _ in link_joints}
    if jointed_links and base_link not in jointed_links:
        def link_volume(lid: str) -> float:
            return sum(ir.part_map()[pid].volume for pid in link_parts[lid])

        base_link = max(jointed_links, key=lambda lid: (link_volume(lid), lid))
        # Update base part meta later via reverse map
        for pid, lid in part_to_link.items():
            if lid == base_link:
                base = pid
                break

    parent: dict[str, Optional[str]] = {base_link: None}
    joint_to_child: dict[str, ResolvedJoint] = {}
    order = sorted(link_parts.keys())
    # BFS with deterministic neighbor order by -confidence, id
    from collections import deque

    q = deque([base_link])
    seen = {base_link}
    while q:
        u = q.popleft()
        nbrs = sorted(
            undirected.get(u, []),
            key=lambda t: (-t[1].confidence, t[0], t[1].id),
        )
        for v, j in nbrs:
            if v in seen:
                continue
            seen.add(v)
            parent[v] = u
            joint_to_child[v] = j
            q.append(v)

    # Orphans: attach via strongest adjacency as fixed
    for lid in order:
        if lid in seen:
            continue
        # find nearest seen link via adjacency of member parts
        best = None
        for e in features.adjacency:
            la = part_to_link[e.part_a]
            lb = part_to_link[e.part_b]
            if la == lid and lb in seen:
                cand = (e.weight, lb)
            elif lb == lid and la in seen:
                cand = (e.weight, la)
            else:
                continue
            if best is None or cand[0] > best[0]:
                best = cand
        if best:
            parent[lid] = best[1]
            seen.add(lid)
            attach_to = best[1]
        else:
            parent[lid] = base_link
            seen.add(lid)
            attach_to = base_link
        # Always create a fixed joint so assign() / export stay consistent
        joint_to_child[lid] = ResolvedJoint(
            id=f"fixed_{lid}",
            parent=attach_to,
            child=lid,
            joint_type=JointType.FIXED,
            origin=ir.part_map()[link_parts[lid][0]].bbox.center().tolist(),
            axis=[0.0, 0.0, 1.0],
            confidence=0.2,
            trace=DecisionTrace(subject=f"orphan_weld:{lid}"),
        )

    # Link world poses: use volume-weighted bbox center of parts as link origin proxy,
    # but joint child frame origin = joint pivot with axis basis.
    part_map = ir.part_map()
    link_world: dict[str, list[list[float]]] = {}

    # First assign base world = identity at base geometric center? Keep CAD world.
    # Link frame = joint frame for non-base; base frame = identity world.
    link_world[base_link] = mat4_to_list(mat4_identity())

    # Topological order
    children_map: dict[str, list[str]] = {lid: [] for lid in link_parts}
    for child, par in parent.items():
        if par is not None:
            children_map[par].append(child)
    for k in children_map:
        children_map[k] = sorted(children_map[k])

    def assign(lid: str) -> None:
        for child in children_map.get(lid, []):
            j = joint_to_child.get(child)
            if j is None:
                # Defensive: synthesize fixed weld if graph is inconsistent
                j = ResolvedJoint(
                    id=f"fixed_{child}",
                    parent=lid,
                    child=child,
                    joint_type=JointType.FIXED,
                    origin=ir.part_map()[link_parts[child][0]].bbox.center().tolist(),
                    axis=[0.0, 0.0, 1.0],
                    confidence=0.1,
                    trace=DecisionTrace(subject=f"missing_joint:{child}"),
                )
                joint_to_child[child] = j
                parent[child] = lid
            # Orient axis; ensure consistent later
            axis = normalize(as_vec3(j.axis))
            origin = as_vec3(j.origin)
            M = mat4_from_origin_axis(origin, axis)
            link_world[child] = mat4_to_list(M)
            # Fix ResolvedJoint parent/child
            j.parent = lid
            j.child = child
            assign(child)

    assign(base_link)

    links = [
        KinematicLink(id=lid, name=lid, part_ids=link_parts[lid])
        for lid in sorted(link_parts.keys())
    ]

    kjoints: list[KinematicJoint] = []
    for child, j in sorted(joint_to_child.items(), key=lambda kv: kv[0]):
        par = parent[child]
        assert par is not None
        M_parent = np.asarray(link_world[par], dtype=np.float64)
        M_inv = invert_mat4(M_parent)
        origin_w = as_vec3(j.origin)
        axis_w = normalize(as_vec3(j.axis))
        # Flip axis for chain consistency: prefer positive dot with parent z
        parent_z = M_parent[:3, 2]
        if float(np.dot(axis_w, parent_z)) < 0:
            axis_w = -axis_w
        origin_l = transform_point(M_inv, origin_w)
        axis_l = normalize(transform_dir(M_inv, axis_w))
        kjoints.append(
            KinematicJoint(
                id=j.id,
                name=j.id,
                parent=par,
                child=child,
                joint_type=j.joint_type,
                origin_local=origin_l.tolist(),
                axis_local=axis_l.tolist(),
                origin_world=origin_w.tolist(),
                axis_world=axis_w.tolist(),
                confidence=j.confidence,
            )
        )

    loops = detect_parallel_loops(list(edge_joints.values()), part_to_link)
    tree = KinematicTree(
        base_link=base_link,
        links=links,
        joints=kjoints,
        link_world=link_world,
        meta={
            "base_part": base,
            "parallel_loops": loops,
            "n_welds": len(weld_edges),
        },
    )
    write_json(out / "kinematic_tree.json", tree.to_dict())
    return tree


def choose_base(
    ir: AssemblyIR,
    features: FeatureGraph,
    joints: Optional[list[ResolvedJoint]] = None,
) -> str:
    """Largest plausible volume with ground-plane contact; prefer joint-connected parts."""
    parts = [p for p in ir.parts if np.isfinite(p.volume) and p.volume < 10.0]
    if not parts:
        parts = list(ir.parts)
    joint_parts: set[str] = set()
    if joints:
        for j in joints:
            joint_parts.add(j.parent)
            joint_parts.add(j.child)

    best = None
    for p in parts:
        extents = np.asarray(p.bbox.max_xyz) - np.asarray(p.bbox.min_xyz)
        if float(np.max(extents)) > 50.0 or not np.all(np.isfinite(extents)):
            continue
        zmin = p.bbox.min_xyz[2]
        finite_zmins = [q.bbox.min_xyz[2] for q in parts if np.isfinite(q.bbox.min_xyz[2])]
        ground = 1.0 if finite_zmins and zmin <= min(finite_zmins) + 1e-6 else 0.0
        joint_bonus = 0.5 if p.id in joint_parts else 0.0
        score = p.volume * (1.0 + 0.5 * ground + joint_bonus) - zmin * 1e-6
        cand = (score, -zmin, p.id)
        if best is None or cand > best:
            best = cand
    if best is None:
        # Fallback: max volume among finite
        return max(parts, key=lambda p: (p.volume if np.isfinite(p.volume) else -1.0, p.id)).id
    return best[2]


def _weld_components(
    part_ids: list[str],
    weld_edges: list[tuple[str, str, float]],
    joint_pairs: set[tuple[str, str]],
) -> dict[str, str]:
    parent = {p: p for p in part_ids}

    def find(a: str) -> str:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a: str, b: str) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    for a, b, _ in sorted(weld_edges, key=lambda t: (t[0], t[1])):
        if tuple(sorted((a, b))) in joint_pairs:
            continue
        union(a, b)
    return {p: find(p) for p in part_ids}
