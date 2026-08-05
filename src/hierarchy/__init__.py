"""06_hierarchy — build kinematic tree from joints + adjacency."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from common.io_util import write_json
from common.math3d import (
    as_vec3,
    bbox_aabb_gap,
    invert_mat4,
    mat4_from_origin_axis,
    mat4_identity,
    mat4_to_list,
    normalize,
    transform_dir,
    transform_point,
)
from common.models import (
    AssemblyIR,
    FeatureGraph,
    JointType,
    KinematicJoint,
    KinematicLink,
    KinematicTree,
    ResolvedJoint,
)
from common.tolerances import Tolerances
from common.trace import DecisionTrace
from hierarchy.parallel import detect_parallel_loops


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

    # Fixed welds for high contact without joint — reject distant false contacts
    weld_edges: list[tuple[str, str, float]] = []
    joint_pairs = set(edge_joints.keys())
    part_map = ir.part_map()
    for e in features.adjacency:
        key = tuple(sorted((e.part_a, e.part_b)))
        if key in joint_pairs:
            continue
        if e.contact_weight < 0.15:
            continue
        ca = part_map[e.part_a].bbox.center()
        cb = part_map[e.part_b].bbox.center()
        if float(np.linalg.norm(ca - cb)) > tolerances.max_weld_distance_m:
            continue
        weld_edges.append((key[0], key[1], e.contact_weight))

    # Weld tiny tip/fastener parts onto the nearest larger neighbor by distance.
    # Prefer strictly larger solids so equal-size jaw pairs don't form an orphan island.
    for p in sorted(ir.parts, key=lambda x: (x.volume, x.id)):
        if p.volume >= tolerances.min_link_volume_m3:
            continue
        pc = p.bbox.center()
        best_larger: Optional[tuple[float, str]] = None
        best_equal: Optional[tuple[float, str]] = None
        for q in ir.parts:
            if q.id == p.id:
                continue
            d = float(np.linalg.norm(pc - q.bbox.center()))
            if d > tolerances.max_weld_distance_m:
                continue
            if q.volume > p.volume + 1e-15:
                if best_larger is None or d < best_larger[0] - 1e-12 or (
                    abs(d - best_larger[0]) <= 1e-12 and q.volume > part_map[best_larger[1]].volume
                ):
                    best_larger = (d, q.id)
            elif abs(q.volume - p.volume) <= 1e-15:
                if best_equal is None or d < best_equal[0]:
                    best_equal = (d, q.id)
        best = best_larger or best_equal
        if best is None:
            continue
        a, b = tuple(sorted((p.id, best[1])))
        if (a, b) not in joint_pairs:
            weld_edges.append((a, b, 0.5 + max(0.0, 0.2 - best[0])))

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

    # Orphans: multi-evidence attach (proximity + adjacency + volume), never blind-to-base.
    # Strong evidence → merge into host link (same mesh/DOF). Medium → fixed joint.
    # Weak → still attach to best host but flag low confidence (better than floating).
    part_map = ir.part_map()
    contact_pairs = {
        tuple(sorted((c.part_a, c.part_b))): c for c in features.contacts
    }
    suspicious_orphans: list[dict] = []
    merged_orphans: list[dict] = []

    def _link_volume(lid: str) -> float:
        return float(sum(part_map[pid].volume for pid in link_parts[lid] if pid in part_map))

    def _link_bbox(lid: str) -> np.ndarray:
        bbs = [part_map[pid].bbox.as_array() for pid in link_parts[lid] if pid in part_map]
        lo = np.min([b[0] for b in bbs], axis=0)
        hi = np.max([b[1] for b in bbs], axis=0)
        return np.vstack([lo, hi])

    def _orphan_host_score(orphan: str, host: str) -> tuple[float, dict]:
        """Higher is better. Combines gap, adjacency, contact, volume direction."""
        ev: dict = {}
        gap = bbox_aabb_gap(_link_bbox(orphan), _link_bbox(host))
        ev["aabb_gap_m"] = gap
        if gap > tolerances.orphan_max_aabb_gap_m:
            return -1.0, ev

        # proximity score: 1 at touch, 0 at max gap
        prox = 1.0 - gap / max(tolerances.orphan_max_aabb_gap_m, 1e-9)
        score = 0.45 * prox
        ev["proximity"] = prox

        # adjacency / contact between member parts
        adj_w = 0.0
        contact_w = 0.0
        for pa in link_parts[orphan]:
            for pb in link_parts[host]:
                key = tuple(sorted((pa, pb)))
                for e in features.adjacency:
                    if tuple(sorted((e.part_a, e.part_b))) == key:
                        adj_w = max(adj_w, float(e.weight), float(getattr(e, "contact_weight", 0) or 0))
                cp = contact_pairs.get(key)
                if cp is not None:
                    contact_w = max(contact_w, float(cp.strength))
        if adj_w > 0:
            score += 0.25 * min(1.0, adj_w)
            ev["adjacency"] = adj_w
        if contact_w > 0:
            score += 0.20 * min(1.0, contact_w)
            ev["contact"] = contact_w

        vo, vh = _link_volume(orphan), _link_volume(host)
        # Prefer attaching smaller body onto larger host
        if vh >= vo - 1e-15 and vh > 0:
            score += 0.10
            ev["volume_ok"] = True
        elif vo > vh * 1.5:
            score -= 0.15
            ev["volume_ok"] = False

        # Prefer hosts already in the movable tree
        if host in seen:
            score += 0.05
            ev["host_in_tree"] = True
        return float(score), ev

    orphans = [lid for lid in order if lid not in seen]
    for lid in orphans:
        cands: list[tuple[float, str, dict]] = []
        for host in list(seen):
            sc, ev = _orphan_host_score(lid, host)
            if sc < 0:
                continue
            cands.append((sc, host, ev))
        cands.sort(key=lambda t: (-t[0], t[1]))

        if not cands:
            # Last resort: nearest by center distance among seen links
            oc = 0.5 * (_link_bbox(lid)[0] + _link_bbox(lid)[1])
            nearest = None
            for host in seen:
                hc = 0.5 * (_link_bbox(host)[0] + _link_bbox(host)[1])
                d = float(np.linalg.norm(oc - hc))
                if nearest is None or d < nearest[0]:
                    nearest = (d, host)
            attach_to = nearest[1] if nearest else base_link
            score = 0.05
            ev = {"fallback": "nearest_center", "note": "weak_evidence"}
            suspicious_orphans.append({"orphan": lid, "host": attach_to, "score": score, **ev})
        else:
            score, attach_to, ev = cands[0]
            if score < tolerances.orphan_attach_min_score:
                suspicious_orphans.append({"orphan": lid, "host": attach_to, "score": score, **ev})

        # Strong evidence: merge into host link (guaranteed co-motion)
        if score >= tolerances.orphan_merge_min_score and attach_to in link_parts:
            link_parts[attach_to].extend(link_parts[lid])
            link_parts[attach_to] = sorted(set(link_parts[attach_to]))
            for pid in link_parts[lid]:
                part_to_link[pid] = attach_to
            merged_orphans.append({"orphan": lid, "host": attach_to, "score": score, **ev})
            del link_parts[lid]
            # drop from parent maps if any
            continue

        parent[lid] = attach_to
        seen.add(lid)
        joint_to_child[lid] = ResolvedJoint(
            id=f"fixed_{lid}",
            parent=attach_to,
            child=lid,
            joint_type=JointType.FIXED,
            origin=part_map[link_parts[lid][0]].bbox.center().tolist(),
            axis=[0.0, 0.0, 1.0],
            confidence=float(np.clip(score, 0.05, 0.85)),
            trace=DecisionTrace(subject=f"orphan_attach:{lid}->{attach_to}:score={score:.2f}"),
        )

    # Rebuild order after merges
    order = sorted(link_parts.keys())
    if base_link not in link_parts:
        # base link id unchanged normally; if somehow merged away, re-root
        base_link = part_to_link[base]

    # Link world poses: translation-only frames at joint pivots.
    # Keeping world orientation (R=I) ensures rest-pose meshes match STEP when
    # the viewer/Godot places the child at origin_local with identity rotation.
    link_world: dict[str, list[list[float]]] = {}

    link_world[base_link] = mat4_to_list(mat4_identity())

    # Topological order
    children_map: dict[str, list[str]] = {lid: [] for lid in link_parts}
    for child, par in parent.items():
        if child not in link_parts:
            continue
        if par is not None and par in link_parts:
            children_map[par].append(child)
    for k in children_map:
        children_map[k] = sorted(children_map[k])

    def _translation_mat(origin: np.ndarray) -> np.ndarray:
        M = mat4_identity()
        M[:3, 3] = as_vec3(origin)
        return M

    def assign(lid: str) -> None:
        for child in children_map.get(lid, []):
            if child not in link_parts:
                continue
            j = joint_to_child.get(child)
            if j is None:
                j = ResolvedJoint(
                    id=f"fixed_{child}",
                    parent=lid,
                    child=child,
                    joint_type=JointType.FIXED,
                    origin=part_map[link_parts[child][0]].bbox.center().tolist(),
                    axis=[0.0, 0.0, 1.0],
                    confidence=0.1,
                    trace=DecisionTrace(subject=f"missing_joint:{child}"),
                )
                joint_to_child[child] = j
                parent[child] = lid
            origin = as_vec3(j.origin)
            # Translation-only link frame (preserve CAD world axes)
            link_world[child] = mat4_to_list(_translation_mat(origin))
            j.parent = lid
            j.child = child
            assign(child)

    assign(base_link)

    # Drop joints whose child link was merged away
    joint_to_child = {c: j for c, j in joint_to_child.items() if c in link_parts}

    links = [
        KinematicLink(id=lid, name=lid, part_ids=link_parts[lid])
        for lid in sorted(link_parts.keys())
    ]

    kjoints: list[KinematicJoint] = []
    for child, j in sorted(joint_to_child.items(), key=lambda kv: kv[0]):
        par = parent.get(child)
        if par is None or par not in link_parts:
            continue
        M_parent = np.asarray(link_world[par], dtype=np.float64)
        M_inv = invert_mat4(M_parent)
        origin_w = as_vec3(j.origin)
        axis_w = normalize(as_vec3(j.axis))
        # Prefer consistent axis sign along the chain (dot with world +Z, else +Y)
        ref = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(axis_w, ref))) < 0.2:
            ref = np.array([0.0, 1.0, 0.0])
        if float(np.dot(axis_w, ref)) < 0:
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
            "merged_orphans": merged_orphans,
            "suspicious_orphans": suspicious_orphans,
        },
    )
    write_json(out / "kinematic_tree.json", tree.to_dict())
    return tree


def choose_base(
    ir: AssemblyIR,
    features: FeatureGraph,
    joints: Optional[list[ResolvedJoint]] = None,
) -> str:
    """Prefer ground contact along estimated up + volume + joint connectivity."""
    from common.upright import estimate_up_axis, height_along

    parts = [p for p in ir.parts if np.isfinite(p.volume) and p.volume < 10.0]
    if not parts:
        parts = list(ir.parts)
    joint_parts: set[str] = set()
    if joints:
        for j in joints:
            joint_parts.add(j.parent)
            joint_parts.add(j.child)

    up = estimate_up_axis(ir)
    # Lowest support height = min of bbox corners along up
    def support_h(p) -> float:
        bb = p.bbox.as_array()
        corners = [
            np.array([x, y, z], dtype=np.float64)
            for x in (bb[0, 0], bb[1, 0])
            for y in (bb[0, 1], bb[1, 1])
            for z in (bb[0, 2], bb[1, 2])
        ]
        return min(height_along(c, up) for c in corners)

    heights = [support_h(p) for p in parts]
    h_floor = min(heights) if heights else 0.0

    best = None
    for p, h in zip(parts, heights):
        extents = np.asarray(p.bbox.max_xyz) - np.asarray(p.bbox.min_xyz)
        if float(np.max(extents)) > 50.0 or not np.all(np.isfinite(extents)):
            continue
        ground = 1.0 if h <= h_floor + 1e-4 else 0.0
        # Near-ground partial credit
        span = max(float(np.ptp(heights)), 1e-3)
        ground += 0.5 * max(0.0, 1.0 - (h - h_floor) / (0.15 * span + 1e-6))
        joint_bonus = 0.5 if p.id in joint_parts else 0.0
        # Ground contact outweighs raw volume (covers sitting under a heavy torso)
        score = p.volume * (1.0 + joint_bonus) * (1.0 + 3.0 * ground) - h * 1e-3
        cand = (score, -h, p.volume, p.id)
        if best is None or cand > best:
            best = cand
    if best is None:
        return max(parts, key=lambda p: (p.volume if np.isfinite(p.volume) else -1.0, p.id)).id
    return best[3]  # part id


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
