"""Fix ResolvedJoint serialization helpers on models."""

from common.models import ResolvedJoint
from common.trace import DecisionTrace
from common.models import JointType


def resolved_joint_from_dict(d: dict) -> ResolvedJoint:
    trace = None
    if d.get("trace"):
        trace = DecisionTrace.from_dict(d["trace"], default_subject=d["id"])
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
