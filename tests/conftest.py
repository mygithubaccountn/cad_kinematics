"""Skip later-phase tests until CURRENT_PHASE unlocks them."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from phase import CURRENT_PHASE  # noqa: E402


def pytest_collection_modifyitems(config, items):
    for item in items:
        path = str(item.fspath)
        if "test_faz0" in path or "test_faz1" in path:
            continue
        if CURRENT_PHASE < 1 and "test_faz1" in path:
            item.add_marker(pytest.mark.skip(reason="Phase 1 locked"))
        # Legacy full-pipeline tests require phase >= 4
        if any(x in path for x in ("test_pipeline", "test_scara", "test_core", "integration", "unit/test_")):
            if CURRENT_PHASE < 4:
                item.add_marker(
                    pytest.mark.skip(
                        reason=f"Legacy/full tests locked until phase >= 4 (now {CURRENT_PHASE})"
                    )
                )
        if "test_faz2" in path and CURRENT_PHASE < 2:
            item.add_marker(pytest.mark.skip(reason="Phase 2 locked"))
        if "test_faz3" in path and CURRENT_PHASE < 3:
            item.add_marker(pytest.mark.skip(reason="Phase 3 locked"))
        if "test_faz4" in path and CURRENT_PHASE < 4:
            item.add_marker(pytest.mark.skip(reason="Phase 4 locked"))
        if "test_faz5" in path and CURRENT_PHASE < 5:
            item.add_marker(pytest.mark.skip(reason="Phase 5 locked"))
        if "test_faz6" in path and CURRENT_PHASE < 6:
            item.add_marker(pytest.mark.skip(reason="Phase 6 locked"))
