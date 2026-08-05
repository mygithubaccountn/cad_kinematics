"""Pipeline manifest — content hashes + stage skip for incremental runs.

Each stage records ``inputs_hash`` (source/artifacts + algorithm version +
tolerances fingerprint). On the next run, if the hash matches and outputs
exist, the stage is skipped.
"""

from __future__ import annotations

import hashlib
import json
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

from common.io_util import ensure_dir, read_json, write_json

MANIFEST_NAME = "manifest.json"

# Bump when stage logic changes in a way that invalidates caches.
ALGORITHM_VERSIONS: dict[str, str] = {
    "ingest": "ingest-1",
    "features": "features-2",  # cylinder caps / session
    "joints": "joints-4",  # coaxial_mate + endpoint chain grow + candidate report
    "hierarchy": "hierarchy-2",  # evidence orphans
    "package": "package-3",  # robot.json only (meshes optional stage)
    "meshes": "meshes-4",  # fast preview: relative tess + no normals + caps
    "validate": "validate-5",  # missing meshes are soft (dev / no --meshes)
}

# Ordered pipeline stages (architecture S0–S6 mapped to names)
# ``meshes`` is optional: default ``run`` skips it unless --meshes/--remesh.
STAGE_ORDER = (
    "ingest",
    "features",
    "joints",
    "hierarchy",
    "package",
    "meshes",
    "validate",
)

# Stages executed by default ``run`` (kinematics + validate; no tessellate).
DEFAULT_RUN_STAGES = (
    "ingest",
    "features",
    "joints",
    "hierarchy",
    "package",
    "validate",
)

STAGE_OUTPUTS: dict[str, tuple[str, ...]] = {
    "ingest": ("assembly_ir.json",),
    "features": ("features.json", "geometry.json"),
    "joints": (
        "joint_hypotheses.json",
        "joints_selected.json",
        "resolved_axes.json",
        "decision_traces.json",
    ),
    "hierarchy": ("kinematic_tree.json",),
    "package": ("robot.json",),
    "meshes": ("mesh_index.json",),
    "validate": ("validation_report.json",),
}


def file_sha256(path: Path | str, chunk: int = 1 << 20) -> str:
    p = Path(path)
    h = hashlib.sha256()
    with p.open("rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def bytes_sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def stable_json_sha256(obj: Any) -> str:
    raw = json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)
    return bytes_sha256(raw.encode("utf-8"))


def tolerances_fingerprint(tol: Any) -> str:
    if is_dataclass(tol):
        return stable_json_sha256(asdict(tol))
    if isinstance(tol, dict):
        return stable_json_sha256(tol)
    return stable_json_sha256(str(tol))


def combine_hashes(parts: Iterable[str]) -> str:
    h = hashlib.sha256()
    for p in parts:
        h.update(p.encode("utf-8"))
        h.update(b"\0")
    return h.hexdigest()


class Manifest:
    def __init__(self, out_dir: Path | str, data: Optional[dict] = None):
        self.out_dir = Path(out_dir)
        self.path = self.out_dir / MANIFEST_NAME
        self.data: dict[str, Any] = data or {
            "version": 1,
            "source": None,
            "source_hash": None,
            "stages": {},
        }

    @classmethod
    def load(cls, out_dir: Path | str) -> "Manifest":
        out = Path(out_dir)
        path = out / MANIFEST_NAME
        if path.is_file():
            return cls(out, read_json(path))
        return cls(out)

    def save(self) -> Path:
        ensure_dir(self.out_dir)
        return write_json(self.path, self.data)

    def set_source(self, source: Path | str, source_hash: Optional[str] = None) -> None:
        sp = Path(source).resolve()
        self.data["source"] = str(sp)
        if source_hash is None and sp.is_file():
            source_hash = file_sha256(sp)
        elif source_hash is None:
            source_hash = stable_json_sha256(str(sp))
        self.data["source_hash"] = source_hash

    def stage_record(self, name: str) -> dict[str, Any]:
        return dict(self.data.setdefault("stages", {}).get(name) or {})

    def outputs_exist(self, stage: str) -> bool:
        required = STAGE_OUTPUTS.get(stage, ())
        if not required:
            return False
        # For joints/features, require primary files (first listed) at minimum
        primary = required[0]
        if not (self.out_dir / primary).is_file():
            return False
        # package also needs at least one mesh if mesh_index says so — soft check
        if stage == "package":
            robot = self.out_dir / "robot.json"
            if not robot.is_file():
                return False
        if stage == "meshes":
            idx = self.out_dir / "mesh_index.json"
            if not idx.is_file():
                return False
            try:
                raw = read_json(idx)
            except Exception:
                return False
            if isinstance(raw, dict) and "meshes" in raw:
                mapping = raw.get("meshes") or {}
            elif isinstance(raw, dict):
                mapping = {k: v for k, v in raw.items() if isinstance(v, str)}
            else:
                return False
            if not mapping:
                return False
            # Require real GLB files (package may write path-only mesh_index).
            for rel in mapping.values():
                if not (self.out_dir / str(rel)).is_file():
                    return False
        return True

    def is_fresh(self, stage: str, inputs_hash: str) -> bool:
        rec = self.stage_record(stage)
        if not rec:
            return False
        if rec.get("status") != "done":
            return False
        if rec.get("inputs_hash") != inputs_hash:
            return False
        if rec.get("algorithm_version") != ALGORITHM_VERSIONS.get(stage):
            return False
        return self.outputs_exist(stage)

    def mark_done(
        self,
        stage: str,
        inputs_hash: str,
        *,
        extra: Optional[dict[str, Any]] = None,
    ) -> None:
        stages = self.data.setdefault("stages", {})
        rec = {
            "status": "done",
            "inputs_hash": inputs_hash,
            "algorithm_version": ALGORITHM_VERSIONS.get(stage, "unknown"),
            "outputs": list(STAGE_OUTPUTS.get(stage, ())),
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        }
        if extra:
            rec["meta"] = extra
        stages[stage] = rec
        self.save()

    def mark_skipped(self, stage: str, inputs_hash: str) -> None:
        # Keep previous record; optionally stamp last_skip
        stages = self.data.setdefault("stages", {})
        rec = dict(stages.get(stage) or {})
        rec["last_skip_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        rec["inputs_hash"] = inputs_hash
        stages[stage] = rec
        self.save()

    def invalidate_from(self, stage: str) -> None:
        """Drop cache for ``stage`` and kinematic downstream stages.

        ``meshes`` is independent: joint/pivot/hierarchy edits must not force
        CAD remesh. Meshes only invalidate when geometry source changes
        (ingest/features) or meshes itself is targeted.
        """
        if stage not in STAGE_ORDER:
            return
        idx = STAGE_ORDER.index(stage)
        stages = self.data.setdefault("stages", {})
        clear_meshes = stage in ("ingest", "features", "meshes")
        for name in STAGE_ORDER[idx:]:
            if name == "meshes" and not clear_meshes:
                continue
            stages.pop(name, None)
        self.save()

    def status_table(self) -> list[dict[str, Any]]:
        rows = []
        for name in STAGE_ORDER:
            rec = self.stage_record(name)
            rows.append(
                {
                    "stage": name,
                    "status": rec.get("status", "missing"),
                    "algorithm_version": rec.get("algorithm_version"),
                    "outputs_ok": self.outputs_exist(name),
                    "updated_at": rec.get("updated_at"),
                    "inputs_hash": (rec.get("inputs_hash") or "")[:12] or None,
                }
            )
        return rows


def hash_artifact(out_dir: Path, relative: str) -> str:
    p = Path(out_dir) / relative
    if not p.is_file():
        return f"missing:{relative}"
    return file_sha256(p)


def inputs_hash_ingest(source: Path, tol: Any) -> str:
    src_h = file_sha256(source) if source.is_file() else stable_json_sha256(str(source))
    return combine_hashes(
        [ALGORITHM_VERSIONS["ingest"], src_h, tolerances_fingerprint(tol)]
    )


def inputs_hash_features(out_dir: Path, tol: Any) -> str:
    return combine_hashes(
        [
            ALGORITHM_VERSIONS["features"],
            hash_artifact(out_dir, "assembly_ir.json"),
            tolerances_fingerprint(tol),
        ]
    )


def inputs_hash_joints(out_dir: Path, tol: Any, include_prismatic: bool) -> str:
    return combine_hashes(
        [
            ALGORITHM_VERSIONS["joints"],
            hash_artifact(out_dir, "assembly_ir.json"),
            hash_artifact(out_dir, "features.json"),
            tolerances_fingerprint(tol),
            f"prismatic={include_prismatic}",
        ]
    )


def inputs_hash_hierarchy(out_dir: Path, tol: Any) -> str:
    return combine_hashes(
        [
            ALGORITHM_VERSIONS["hierarchy"],
            hash_artifact(out_dir, "assembly_ir.json"),
            hash_artifact(out_dir, "features.json"),
            hash_artifact(out_dir, "joints_selected.json"),
            hash_artifact(out_dir, "resolved_axes.json"),
            tolerances_fingerprint(tol),
        ]
    )


def inputs_hash_package(out_dir: Path, tol: Any, name: str, remesh: bool = False) -> str:
    # ``remesh`` kept for call-site compat; no longer part of package hash.
    _ = remesh
    return combine_hashes(
        [
            ALGORITHM_VERSIONS["package"],
            hash_artifact(out_dir, "assembly_ir.json"),
            hash_artifact(out_dir, "kinematic_tree.json"),
            tolerances_fingerprint(tol),
            f"name={name}",
        ]
    )


def inputs_hash_meshes(out_dir: Path, tol: Any, quality: str = "preview") -> str:
    """Hash geometry + link↔part topology + quality — not joint pivots/axes.

    Pivot/joint algorithm changes update robot.json but must not invalidate CAD meshes.
    """
    out = Path(out_dir)
    topo = _link_topology_fingerprint(out)
    return combine_hashes(
        [
            ALGORITHM_VERSIONS["meshes"],
            hash_artifact(out, "assembly_ir.json"),
            topo,
            tolerances_fingerprint(tol),
            f"quality={quality}",
        ]
    )


def _link_topology_fingerprint(out_dir: Path) -> str:
    """Stable fingerprint of which parts belong to which link (not joint frames)."""
    out = Path(out_dir)
    tree_path = out / "kinematic_tree.json"
    if not tree_path.is_file():
        return "no-tree"
    try:
        raw = read_json(tree_path)
    except Exception:
        return "bad-tree"
    links = []
    for L in raw.get("links") or []:
        links.append(
            {
                "id": L.get("id"),
                "part_ids": sorted(L.get("part_ids") or []),
            }
        )
    links.sort(key=lambda x: str(x["id"]))
    return stable_json_sha256({"base": raw.get("base_link"), "links": links})


def inputs_hash_validate(out_dir: Path, tol: Any) -> str:
    return combine_hashes(
        [
            ALGORITHM_VERSIONS["validate"],
            hash_artifact(out_dir, "assembly_ir.json"),
            hash_artifact(out_dir, "kinematic_tree.json"),
            hash_artifact(out_dir, "robot.json"),
            tolerances_fingerprint(tol),
        ]
    )
