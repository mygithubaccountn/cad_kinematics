"""Fix ResolvedJoint serialization helpers on models."""

from pipeline.common.models import ResolvedJoint
from pipeline.common.trace import DecisionTrace
from pipeline.common.models import JointType


def resolved_joint_from_dict(d: dict) -> ResolvedJoint:
    trace = None
    if d.get("trace"):
        t = d["trace"]
        trace = DecisionTrace(subject=t.get("subject", d["id"]))
        for e in t.get("evidence", []):
            trace.add(e["name"], e["score"], e.get("detail", ""))
        for r in t.get("rejected", []):
            trace.reject(r)
    return ResolvedJoint(
        id=d["id"],
        parent=d["parent"],
        child=d["child"],
        joint_type=JointType(d["joint_type"]),
        origin=list(d["origin"]),
        axis=list(d["axis"]),
        confidence=float(d["confidence"]),
        trace=trace,
    )
