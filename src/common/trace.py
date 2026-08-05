"""Explainable decision traces for joint / hierarchy / pivot-axis choices."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class Evidence:
    name: str
    score: float
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "score": self.score, "detail": self.detail}


@dataclass
class DecisionTrace:
    """
    Explainable record for one decision (joint select or pivot/axis resolve).

    Architecture fields:
      subject, evidence[], rejected[], chosen pivot/axis, confidence, runner-up
    """

    subject: str
    evidence: list[Evidence] = field(default_factory=list)
    rejected: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    confidence: Optional[float] = None
    chosen: Optional[dict[str, Any]] = None
    runner_up: Optional[dict[str, Any]] = None

    def add(self, name: str, score: float, detail: str = "") -> None:
        self.evidence.append(Evidence(name, float(score), detail))

    def reject(self, reason: str) -> None:
        self.rejected.append(reason)

    def note(self, text: str) -> None:
        self.notes.append(text)

    def set_chosen(
        self,
        *,
        origin: list[float],
        axis: list[float],
        method: str,
        confidence: float,
        detail: str = "",
    ) -> None:
        self.confidence = float(confidence)
        self.chosen = {
            "origin": [float(x) for x in origin],
            "axis": [float(x) for x in axis],
            "method": method,
            "confidence": float(confidence),
            "detail": detail,
        }

    def set_runner_up(
        self,
        *,
        name: str,
        origin: list[float],
        axis: list[float],
        score: float,
        detail: str = "",
    ) -> None:
        self.runner_up = {
            "name": name,
            "origin": [float(x) for x in origin],
            "axis": [float(x) for x in axis],
            "score": float(score),
            "detail": detail,
        }

    @property
    def total_score(self) -> float:
        return float(sum(e.score for e in self.evidence))

    def summary_line(self) -> str:
        parts = [f"{e.name}({e.score:+.2f})" for e in self.evidence]
        rej = f" rejected=[{'; '.join(self.rejected)}]" if self.rejected else ""
        method = ""
        if self.chosen:
            method = f" chosen={self.chosen.get('method')}"
        ru = ""
        if self.runner_up:
            ru = f" runner_up={self.runner_up.get('name')}"
        return f"{self.subject}: evidence=[{', '.join(parts)}]{rej}{method}{ru}"

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "subject": self.subject,
            "evidence": [e.to_dict() for e in self.evidence],
            "rejected": list(self.rejected),
            "notes": list(self.notes),
            "total_score": self.total_score,
            "summary": self.summary_line(),
        }
        if self.confidence is not None:
            d["confidence"] = self.confidence
        if self.chosen is not None:
            d["chosen"] = self.chosen
        if self.runner_up is not None:
            d["runner_up"] = self.runner_up
        return d

    @staticmethod
    def from_dict(d: dict[str, Any], default_subject: str = "") -> "DecisionTrace":
        trace = DecisionTrace(subject=d.get("subject", default_subject or "unknown"))
        for e in d.get("evidence", []) or []:
            trace.add(e["name"], e["score"], e.get("detail", ""))
        for r in d.get("rejected", []) or []:
            trace.reject(r)
        for n in d.get("notes", []) or []:
            trace.note(n)
        if d.get("confidence") is not None:
            trace.confidence = float(d["confidence"])
        if d.get("chosen"):
            trace.chosen = dict(d["chosen"])
        if d.get("runner_up"):
            trace.runner_up = dict(d["runner_up"])
        return trace
