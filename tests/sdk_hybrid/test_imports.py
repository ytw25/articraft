from __future__ import annotations

import sdk_hybrid
from sdk_hybrid.v0.cadquery import mesh_from_cadquery


def test_sdk_hybrid_exports() -> None:
    assert hasattr(sdk_hybrid, "ArticulatedObject")
    assert hasattr(sdk_hybrid, "ValidationError")
    assert hasattr(sdk_hybrid, "TestContext")
    assert hasattr(sdk_hybrid, "UnsupportedPartFinding")
    assert callable(mesh_from_cadquery)
