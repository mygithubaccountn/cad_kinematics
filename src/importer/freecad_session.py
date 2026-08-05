"""Shared FreeCAD document cache — open each STEP once per process.

Import / cylinder extract / tessellation used to call Import.insert 3×;
that dominated runtime on mid-size assemblies.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

_DOC_NAME = "cad_robot_shared"
_loaded_path: Optional[str] = None
_doc: Any = None


def get_step_document(path: str | Path) -> Any:
    """Return an open FreeCAD doc with ``path`` imported (cached)."""
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
    doc = FreeCAD.newDocument(_DOC_NAME)
    Import.insert(key, doc.Name)
    doc.recompute()
    _doc = doc
    _loaded_path = key
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
