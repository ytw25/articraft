from __future__ import annotations

import sdk


def test_sdk_exports() -> None:
    assert hasattr(sdk, "ArticulatedObject")
    assert hasattr(sdk, "ValidationError")
    assert hasattr(sdk, "TestContext")
