from __future__ import annotations

from agent.feedback import format_compile_exception


def test_compile_exception_formats_loft_hint() -> None:
    exc = RuntimeError(
        "ValueError: Loft profile area must be non-zero in XY projection; "
        "profiles should usually be closed XY loops at constant z"
    )
    formatted = format_compile_exception(exc)

    assert "URDF compile failed: RuntimeError" in formatted
    assert "Loft profile area must be non-zero in XY projection" in formatted
    assert "Hint: LoftGeometry checks profile area in the XY projection." in formatted
    assert "author it in XY first and rotate the mesh afterward" in formatted
