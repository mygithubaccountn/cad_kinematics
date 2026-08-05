"""Explainable decision traces for joint / hierarchy choices."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class Evidence:
    name: str
    score: float
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "score": self.score, "detail": self.detail}


@dataclass
class DecisionTrace:
    subject: str
    evidence: list[Evidence] = field(default_factory=list)
    rejected: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    def add(self, name: str, score: float, detail: str = "") -> None:
        self.evidence.append(Evidence(name, float(score), detail))

    def reject(self, reason: str) -> None:
        self.rejected.append(reason)

    def note(self, text: str) -> None:
        self.notes.append(text)

    @property
    def total_score(self) -> float:
        return float(sum(e.score for e in self.evidence))

    def summary_line(self) -> str:
        parts = [f"{e.name}({e.score:+.2f})" for e in self.evidence]
        rej = f" rejected=[{'; '.join(self.rejected)}]" if self.rejected else ""
        return f"{self.subject}: evidence=[{', '.join(parts)}]{rej}"

    def to_dict(self) -> dict[str, Any]:
        return {
            "subject": self.subject,
            "evidence": [e.to_dict() for e in self.evidence],
            "rejected": list(self.rejected),
            "notes": list(self.notes),
            "total_score": self.total_score,
            "summary": self.summary_line(),
        }
