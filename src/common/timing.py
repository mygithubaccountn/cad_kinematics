"""Lightweight wall-clock timing for pipeline stage comparison."""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Iterator, Optional


# Preferred print order (unknown labels append after these).
SUMMARY_ORDER = (
    "STEP import",
    "OCC analysis",
    "Feature extraction",
    "Joint inference",
    "Hierarchy",
    "Mesh generation",
    "GLB export",
    "Index/package",
    "Validation",
    "FreeCAD cleanup",
)


@dataclass
class Span:
    name: str
    seconds: float = 0.0
    skipped: bool = False
    note: str = ""


@dataclass
class PerfTimer:
    """Collect named spans; print a single summary at the end."""

    order: list[str] = field(default_factory=list)
    spans: dict[str, Span] = field(default_factory=dict)
    t0: float = field(default_factory=time.perf_counter)

    def _ensure(self, name: str) -> Span:
        if name not in self.spans:
            self.order.append(name)
            self.spans[name] = Span(name=name)
        return self.spans[name]

    def mark_skip(self, name: str, note: str = "cache hit") -> None:
        sp = self._ensure(name)
        # Don't overwrite a real measurement with a later skip mark.
        if sp.seconds > 0 and not sp.skipped:
            return
        sp.skipped = True
        sp.seconds = 0.0
        sp.note = note

    def add(self, name: str, seconds: float, *, note: str = "") -> None:
        sp = self._ensure(name)
        if sp.skipped:
            sp.skipped = False
            sp.seconds = 0.0
        sp.seconds += max(0.0, seconds)
        if note:
            sp.note = note

    @contextmanager
    def span(self, name: str, *, note: str = "") -> Iterator[None]:
        t0 = time.perf_counter()
        try:
            yield
        finally:
            self.add(name, time.perf_counter() - t0, note=note)

    def total_seconds(self) -> float:
        return time.perf_counter() - self.t0

    def _ordered_names(self) -> list[str]:
        seen = set(self.spans)
        ordered = [n for n in SUMMARY_ORDER if n in seen]
        ordered.extend(n for n in self.order if n not in SUMMARY_ORDER and n in seen)
        return ordered

    def to_dict(self) -> dict:
        return {
            "spans": [
                {
                    "name": n,
                    "seconds": round(self.spans[n].seconds, 3),
                    "skipped": self.spans[n].skipped,
                    "note": self.spans[n].note,
                    "display": format_duration(self.spans[n].seconds),
                }
                for n in self._ordered_names()
            ],
            "total_seconds": round(self.total_seconds(), 3),
            "total_display": format_duration(self.total_seconds()),
        }

    def summary_lines(self) -> list[str]:
        lines = ["", "Performance summary:"]
        for name in self._ordered_names():
            sp = self.spans[name]
            if sp.skipped:
                extra = f" ({sp.note})" if sp.note else " (skipped)"
                lines.append(f"  {name}: skipped{extra}")
            else:
                lines.append(f"  {name}: {format_duration(sp.seconds)}")
        lines.append(f"  Total: {format_duration(self.total_seconds())}")
        return lines

    def print_summary(self) -> None:
        print("\n".join(self.summary_lines()))


def format_duration(seconds: float) -> str:
    """Human duration: 5s / 1m 20s / 11m 48s."""
    if seconds < 0:
        seconds = 0.0
    if seconds < 1.0:
        return f"{seconds:.1f}s"
    if seconds < 60:
        return f"{int(round(seconds))}s"
    m = int(seconds // 60)
    s = int(round(seconds - m * 60))
    if s == 60:
        m += 1
        s = 0
    return f"{m}m {s:02d}s"


# Process-local timer used by stages (set by cmd_run / cmd_stage).
_ACTIVE: Optional[PerfTimer] = None


def get_timer() -> Optional[PerfTimer]:
    return _ACTIVE


def set_timer(timer: Optional[PerfTimer]) -> None:
    global _ACTIVE
    _ACTIVE = timer


@contextmanager
def timed(name: str, *, note: str = "") -> Iterator[None]:
    t = _ACTIVE
    if t is None:
        yield
        return
    with t.span(name, note=note):
        yield


def mark_skip(name: str, note: str = "cache hit") -> None:
    t = _ACTIVE
    if t is not None:
        t.mark_skip(name, note=note)
