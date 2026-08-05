"""Intermediate representation dataclasses for the full pipeline."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Optional

import numpy as np

from common.math3d import mat4_from_list, mat4_to_list
from common.trace import DecisionTrace


class MateKind(str, Enum):
    COINCIDENT = "coincident"
    CONCENTRIC = "concentric"
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    UNKNOWN = "unknown"


class JointType(str, Enum):
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    UNKNOWN = "unknown"


class CylKind(str, Enum):
    INNER = "inner"  # hole
    OUTER = "outer"  # shaft
    UNKNOWN = "unknown"


@dataclass
class BBox:
    min_xyz: list[float]
    max_xyz: list[float]

    def as_array(self) -> np.ndarray:
        return np.array([self.min_xyz, self.max_xyz], dtype=np.float64)

    def center(self) -> np.ndarray:
        a = self.as_array()
        return 0.5 * (a[0] + a[1])

    def to_dict(self) -> dict[str, Any]:
        return {"min_xyz": list(self.min_xyz), "max_xyz": list(self.max_xyz)}

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "BBox":
        return BBox(min_xyz=list(d["min_xyz"]), max_xyz=list(d["max_xyz"]))


@dataclass
class Provenance:
    freecad_path: str = ""
    step_entity_id: str = ""
    source: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "Provenance":
        return Provenance(**{k: d.get(k, "") for k in ("freecad_path", "step_entity_id", "source")})


@dataclass
class PartInstance:
    id: str
    name: str
    placement: list[list[float]]  # 4x4 row-major world
    volume: float
    bbox: BBox
    material: Optional[str] = None
    provenance: Provenance = field(default_factory=Provenance)
    # Optional triangle mesh in world frame (for synthetic / export without FreeCAD)
    mesh_vertices: Optional[list[list[float]]] = None
    mesh_faces: Optional[list[list[int]]] = None
    # Opaque handle name for FreeCAD document object
    shape_ref: str = ""

    def placement_mat(self) -> np.ndarray:
        return mat4_from_list(self.placement)

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "id": self.id,
            "name": self.name,
            "placement": self.placement,
            "volume": self.volume,
            "bbox": self.bbox.to_dict(),
            "material": self.material,
            "provenance": self.provenance.to_dict(),
            "shape_ref": self.shape_ref,
        }
        if self.mesh_vertices is not None:
            d["mesh_vertices"] = self.mesh_vertices
            d["mesh_faces"] = self.mesh_faces
        return d

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "PartInstance":
        return PartInstance(
            id=d["id"],
            name=d["name"],
            placement=d["placement"],
            volume=float(d["volume"]),
            bbox=BBox.from_dict(d["bbox"]),
            material=d.get("material"),
            provenance=Provenance.from_dict(d.get("provenance", {})),
            mesh_vertices=d.get("mesh_vertices"),
            mesh_faces=d.get("mesh_faces"),
            shape_ref=d.get("shape_ref", ""),
        )


@dataclass
class AssemblyNode:
    id: str
    name: str
    part_id: Optional[str] = None
    children: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "AssemblyNode":
        return AssemblyNode(
            id=d["id"],
            name=d["name"],
            part_id=d.get("part_id"),
            children=list(d.get("children", [])),
        )


@dataclass
class MateHint:
    kind: MateKind
    part_a: str
    part_b: str
    confidence: float = 0.5
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind.value,
            "part_a": self.part_a,
            "part_b": self.part_b,
            "confidence": self.confidence,
            "detail": self.detail,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "MateHint":
        return MateHint(
            kind=MateKind(d["kind"]),
            part_a=d["part_a"],
            part_b=d["part_b"],
            confidence=float(d.get("confidence", 0.5)),
            detail=d.get("detail", ""),
        )


@dataclass
class AssemblyIR:
    source_path: str
    parts: list[PartInstance]
    assembly_nodes: list[AssemblyNode] = field(default_factory=list)
    mate_hints: list[MateHint] = field(default_factory=list)
    unit: str = "metre"
    meta: dict[str, Any] = field(default_factory=dict)

    def part_map(self) -> dict[str, PartInstance]:
        return {p.id: p for p in self.parts}

    def to_dict(self) -> dict[str, Any]:
        return {
            "source_path": self.source_path,
            "unit": self.unit,
            "parts": [p.to_dict() for p in self.parts],
            "assembly_nodes": [n.to_dict() for n in self.assembly_nodes],
            "mate_hints": [m.to_dict() for m in self.mate_hints],
            "meta": self.meta,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "AssemblyIR":
        return AssemblyIR(
            source_path=d.get("source_path", ""),
            parts=[PartInstance.from_dict(p) for p in d.get("parts", [])],
            assembly_nodes=[AssemblyNode.from_dict(n) for n in d.get("assembly_nodes", [])],
            mate_hints=[MateHint.from_dict(m) for m in d.get("mate_hints", [])],
            unit=d.get("unit", "metre"),
            meta=dict(d.get("meta", {})),
        )


@dataclass
class CylFeature:
    id: str
    part_id: str
    axis_point: list[float]
    axis_dir: list[float]
    radius: float
    height: float
    kind: CylKind = CylKind.UNKNOWN
    face_ids: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "part_id": self.part_id,
            "axis_point": list(self.axis_point),
            "axis_dir": list(self.axis_dir),
            "radius": self.radius,
            "height": self.height,
            "kind": self.kind.value,
            "face_ids": list(self.face_ids),
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "CylFeature":
        return CylFeature(
            id=d["id"],
            part_id=d["part_id"],
            axis_point=list(d["axis_point"]),
            axis_dir=list(d["axis_dir"]),
            radius=float(d["radius"]),
            height=float(d["height"]),
            kind=CylKind(d.get("kind", "unknown")),
            face_ids=list(d.get("face_ids", [])),
        )


@dataclass
class PlaneFeature:
    id: str
    part_id: str
    origin: list[float]
    normal: list[float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "PlaneFeature":
        return PlaneFeature(**d)


@dataclass
class ContactPair:
    part_a: str
    part_b: str
    strength: float
    centroid: list[float]
    sample_count: int = 0

    def ordered(self) -> tuple[str, str]:
        return tuple(sorted((self.part_a, self.part_b)))  # type: ignore[return-value]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "ContactPair":
        return ContactPair(**d)


@dataclass
class ConcentricCluster:
    id: str
    axis_point: list[float]
    axis_dir: list[float]
    cyl_ids: list[str]
    part_ids: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "ConcentricCluster":
        return ConcentricCluster(**d)


@dataclass
class AdjacencyEdge:
    part_a: str
    part_b: str
    contact_weight: float = 0.0
    shared_axis_weight: float = 0.0

    @property
    def weight(self) -> float:
        return self.contact_weight + self.shared_axis_weight

    def ordered(self) -> tuple[str, str]:
        return tuple(sorted((self.part_a, self.part_b)))  # type: ignore[return-value]

    def to_dict(self) -> dict[str, Any]:
        return {
            "part_a": self.part_a,
            "part_b": self.part_b,
            "contact_weight": self.contact_weight,
            "shared_axis_weight": self.shared_axis_weight,
            "weight": self.weight,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "AdjacencyEdge":
        return AdjacencyEdge(
            part_a=d["part_a"],
            part_b=d["part_b"],
            contact_weight=float(d.get("contact_weight", 0.0)),
            shared_axis_weight=float(d.get("shared_axis_weight", 0.0)),
        )


@dataclass
class FeatureGraph:
    cylinders: list[CylFeature] = field(default_factory=list)
    planes: list[PlaneFeature] = field(default_factory=list)
    contacts: list[ContactPair] = field(default_factory=list)
    clusters: list[ConcentricCluster] = field(default_factory=list)
    adjacency: list[AdjacencyEdge] = field(default_factory=list)
    meta: dict[str, Any] = field(default_factory=dict)

    def cyl_map(self) -> dict[str, CylFeature]:
        return {c.id: c for c in self.cylinders}

    def to_dict(self) -> dict[str, Any]:
        return {
            "cylinders": [c.to_dict() for c in self.cylinders],
            "planes": [p.to_dict() for p in self.planes],
            "contacts": [c.to_dict() for c in self.contacts],
            "clusters": [c.to_dict() for c in self.clusters],
            "adjacency": [a.to_dict() for a in self.adjacency],
            "meta": self.meta,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "FeatureGraph":
        return FeatureGraph(
            cylinders=[CylFeature.from_dict(c) for c in d.get("cylinders", [])],
            planes=[PlaneFeature.from_dict(p) for p in d.get("planes", [])],
            contacts=[ContactPair.from_dict(c) for c in d.get("contacts", [])],
            clusters=[ConcentricCluster.from_dict(c) for c in d.get("clusters", [])],
            adjacency=[AdjacencyEdge.from_dict(a) for a in d.get("adjacency", [])],
            meta=dict(d.get("meta", {})),
        )


@dataclass
class JointHypothesis:
    id: str
    part_a: str
    part_b: str
    joint_type: JointType
    axis_point: list[float]
    axis_dir: list[float]
    pivot: list[float]
    confidence: float
    evidence: list[dict[str, Any]] = field(default_factory=list)
    trace: Optional[DecisionTrace] = None
    cluster_id: str = ""

    def ordered_parts(self) -> tuple[str, str]:
        return tuple(sorted((self.part_a, self.part_b)))  # type: ignore[return-value]

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "part_a": self.part_a,
            "part_b": self.part_b,
            "joint_type": self.joint_type.value,
            "axis_point": list(self.axis_point),
            "axis_dir": list(self.axis_dir),
            "pivot": list(self.pivot),
            "confidence": self.confidence,
            "evidence": list(self.evidence),
            "trace": self.trace.to_dict() if self.trace else None,
            "cluster_id": self.cluster_id,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "JointHypothesis":
        trace = None
        if d.get("trace"):
            trace = DecisionTrace.from_dict(d["trace"], default_subject=d["id"])
        return JointHypothesis(
            id=d["id"],
            part_a=d["part_a"],
            part_b=d["part_b"],
            joint_type=JointType(d["joint_type"]),
            axis_point=list(d["axis_point"]),
            axis_dir=list(d["axis_dir"]),
            pivot=list(d["pivot"]),
            confidence=float(d["confidence"]),
            evidence=list(d.get("evidence", [])),
            trace=trace,
            cluster_id=d.get("cluster_id", ""),
        )


@dataclass
class ResolvedJoint:
    id: str
    parent: str
    child: str
    joint_type: JointType
    origin: list[float]  # pivot in world (CAD)
    axis: list[float]  # unit axis in world
    confidence: float
    trace: Optional[DecisionTrace] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "parent": self.parent,
            "child": self.child,
            "joint_type": self.joint_type.value,
            "origin": list(self.origin),
            "axis": list(self.axis),
            "confidence": self.confidence,
            "trace": self.trace.to_dict() if self.trace else None,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "ResolvedJoint":
        from common.serialize import resolved_joint_from_dict

        return resolved_joint_from_dict(d)


@dataclass
class KinematicLink:
    id: str
    name: str
    part_ids: list[str]
    mesh_path: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "KinematicLink":
        return KinematicLink(**d)


@dataclass
class KinematicJoint:
    id: str
    name: str
    parent: str
    child: str
    joint_type: JointType
    # Origin & axis expressed in parent link local frame
    origin_local: list[float]
    axis_local: list[float]
    # Also keep world for validation
    origin_world: list[float]
    axis_world: list[float]
    confidence: float = 1.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "parent": self.parent,
            "child": self.child,
            "joint_type": self.joint_type.value,
            "origin_local": list(self.origin_local),
            "axis_local": list(self.axis_local),
            "origin_world": list(self.origin_world),
            "axis_world": list(self.axis_world),
            "confidence": self.confidence,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "KinematicJoint":
        return KinematicJoint(
            id=d["id"],
            name=d["name"],
            parent=d["parent"],
            child=d["child"],
            joint_type=JointType(d["joint_type"]),
            origin_local=list(d["origin_local"]),
            axis_local=list(d["axis_local"]),
            origin_world=list(d["origin_world"]),
            axis_world=list(d["axis_world"]),
            confidence=float(d.get("confidence", 1.0)),
        )


@dataclass
class KinematicTree:
    base_link: str
    links: list[KinematicLink]
    joints: list[KinematicJoint]
    # link_id -> 4x4 world rest pose (CAD Z-up)
    link_world: dict[str, list[list[float]]] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "base_link": self.base_link,
            "links": [l.to_dict() for l in self.links],
            "joints": [j.to_dict() for j in self.joints],
            "link_world": self.link_world,
            "meta": self.meta,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "KinematicTree":
        return KinematicTree(
            base_link=d["base_link"],
            links=[KinematicLink.from_dict(l) for l in d["links"]],
            joints=[KinematicJoint.from_dict(j) for j in d["joints"]],
            link_world=dict(d.get("link_world", {})),
            meta=dict(d.get("meta", {})),
        )


@dataclass
class RobotDesc:
    """Godot-facing contract: hierarchy + pivots + axes + mesh paths."""

    name: str
    frame: str  # "gltf_y_up" or "cad_z_up"
    base_link: str
    links: list[dict[str, Any]]
    joints: list[dict[str, Any]]
    rest_pose: dict[str, Any] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "frame": self.frame,
            "base_link": self.base_link,
            "links": self.links,
            "joints": self.joints,
            "rest_pose": self.rest_pose,
            "meta": self.meta,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "RobotDesc":
        return RobotDesc(
            name=d["name"],
            frame=d.get("frame", "gltf_y_up"),
            base_link=d["base_link"],
            links=list(d["links"]),
            joints=list(d["joints"]),
            rest_pose=dict(d.get("rest_pose", {})),
            meta=dict(d.get("meta", {})),
        )


@dataclass
class ValidationIssue:
    severity: str  # error | warning | info
    code: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ValidationReport:
    """
    ok=False only for critical geometric failures.
    Soft / low-quality STEP issues stay as warnings + quality scores.
    """

    ok: bool
    issues: list[ValidationIssue] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)
    overall_confidence: float = 1.0
    unresolved_parts: list[Any] = field(default_factory=list)
    suspicious_joints: list[Any] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    fk_motion: Optional[dict[str, Any]] = None

    def to_dict(self) -> dict[str, Any]:
        d = {
            "ok": self.ok,
            "overall_confidence": float(self.overall_confidence),
            "unresolved_parts": list(self.unresolved_parts),
            "suspicious_joints": list(self.suspicious_joints),
            "warnings": list(self.warnings),
            "issues": [i.to_dict() for i in self.issues],
            "metrics": self.metrics,
        }
        if self.fk_motion is not None:
            d["fk_motion"] = self.fk_motion
        return d
