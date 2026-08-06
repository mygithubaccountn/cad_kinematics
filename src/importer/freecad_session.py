"""Shared FreeCAD document cache — open each STEP once per process, and
persist the imported document to disk so later *processes* don't pay for
STEP import again either.

Import.insert() on a STEP file doesn't just parse text — OpenCASCADE runs
shape-healing/validation over every face, edge and B-spline curve it reads.
On heavy files (dense NURBS content, many solids) this can take minutes.
The in-process cache below (open each STEP once per process) already
avoided doing it 3x within one `pipeline.py run` (ingest + tessellate used
to each call Import.insert separately). But it resets on every new process
— so `--from-stage meshes` run afterward, or a retry after a crash, or a
second --out folder for the same file, paid the full STEP-import cost
again, because nothing survived the previous process exiting.

The disk cache closes that gap: after a fresh Import.insert(), the already
-imported, already-healed document is saved once as a native .FCStd next
to no output folder in particular (out/_cache/step_import/, shared across
every --out run), keyed by the STEP file's own content hash — not its
path, not tolerances (tolerances never affect what Import.insert
produces, only downstream interpretation of it). Any future
get_step_document() call for the same STEP content, from any process,
loads that .FCStd instead — verified byte-identical (same solid names,
volumes, bboxes) to a fresh import, and far faster: FreeCAD is deserializing
its own already-healed B-Rep, not re-running STEP parse + shape healing.
Best-effort throughout — any cache read/write failure falls back to a
normal Import.insert(), never a hard failure.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Optional

_DOC_NAME = "cad_robot_shared"
_loaded_path: Optional[str] = None
_doc: Any = None

_CACHE_DIR = Path(__file__).resolve().parents[2] / "out" / "_cache" / "step_import"


def _cache_path(step_path: Path) -> Optional[Path]:
    """Content-hash + FreeCAD version -> cache file. None if it can't be computed
    (missing file, hashing error) — caller treats that as a cache miss."""
    try:
        from common.manifest import file_sha256
        import FreeCAD

        h = file_sha256(step_path)
        ver = "-".join(str(v) for v in FreeCAD.Version()[:3])
        return _CACHE_DIR / f"{h}_{ver}.FCStd"
    except Exception:
        return None


def _heartbeat(stop: threading.Event, t0: float, interval: float = 10.0) -> None:
    """Proves the process is alive during Import.insert() — a single opaque
    blocking OCC call with no progress callback exposed to Python. Without
    this, a genuinely-slow import on a heavy file is indistinguishable from
    a hung one from the terminal/log alone."""
    while not stop.wait(interval):
        print(f"  ... STEP import devam ediyor ({time.time() - t0:.0f}s)", flush=True)


def get_step_document(path: str | Path) -> Any:
    """Return an open FreeCAD doc with ``path`` imported (in-process cache,
    backed by a persistent on-disk cache across processes)."""
    global _loaded_path, _doc
    from importer.freecad_backend import freecad_available

    if not freecad_available():
        raise RuntimeError("FreeCAD not available")

    import FreeCAD
    import Import

    key = str(Path(path).resolve())
    if _doc is not None and _loaded_path == key:
        try:
            # Still alive?
            _ = _doc.Name
            return _doc
        except Exception:
            _doc = None
            _loaded_path = None

    close_step_document()

    cache_file = _cache_path(Path(key))
    if cache_file is not None and cache_file.is_file():
        t0 = time.time()
        try:
            doc = FreeCAD.openDocument(str(cache_file))
            print(f"STEP import: disk cache'ten yüklendi ({time.time() - t0:.2f}s, "
                  f"{cache_file.name})", flush=True)
            _doc = doc
            _loaded_path = key
            return doc
        except Exception as e:
            print(f"STEP import: cache dosyası okunamadı ({e}) — taze import'a "
                  f"düşülüyor", flush=True)
            # corrupt/incompatible cache entry — fall through to a real import

    print("STEP import: cache miss — FreeCAD Import.insert() başlıyor "
          "(büyük/karmaşık dosyalarda dakikalar sürebilir)...", flush=True)
    t0 = time.time()
    stop_heartbeat = threading.Event()
    hb = threading.Thread(target=_heartbeat, args=(stop_heartbeat, t0), daemon=True)
    hb.start()
    doc = FreeCAD.newDocument(_DOC_NAME)
    try:
        Import.insert(key, doc.Name)
        doc.recompute()
    finally:
        stop_heartbeat.set()
        hb.join(timeout=1.0)
    print(f"STEP import: tamamlandı ({time.time() - t0:.1f}s)", flush=True)
    _doc = doc
    _loaded_path = key

    if cache_file is not None:
        try:
            cache_file.parent.mkdir(parents=True, exist_ok=True)
            doc.saveAs(str(cache_file))
            print(f"STEP import: sonuç cache'e yazıldı ({cache_file.name}) — "
                  f"aynı dosya bir daha ne zaman içe aktarılsa (farklı --out "
                  f"klasörü olsa bile) bu adım atlanacak", flush=True)
        except Exception:
            pass  # caching is an optimization — never fail the pipeline over it

    return doc


def close_step_document() -> None:
    global _loaded_path, _doc
    if _doc is None:
        return
    try:
        import FreeCAD

        FreeCAD.closeDocument(_doc.Name)
    except Exception:
        pass
    _doc = None
    _loaded_path = None
